/* Motors - motors_motion_task.c
 *
 * Purpose: FreeRTOS motion task — consumes MotionCommands from the queue
 * and drives axis positions via RMT+DMA step pulse generation.
 *
 * The motion task is the SINGLE WRITER of motors_state position,
 * status, and tracking fields — all other code only reads them.
 *
 * Two execution paths, dispatched by command type:
 *   slewing_loop_rmt  — distance-bounded, ramped accel/decel, batched RMT
 *   tracking_loop_rmt — open-ended, constant velocity, fractional accumulator
 *
 * RMT+DMA replaces the previous software GPIO bit-banging. Step pulses
 * are hardware-timed with zero jitter. The CPU sleeps on a semaphore
 * while DMA streams symbols to the RMT peripheral.
 */

#include "motors_internal.h"

#include <math.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

static const char *TAG = "MOTORS_MOTION_TASK";

/*
 * Task stack size — sized to accommodate batch encoding loops,
 * condition checks, and FreeRTOS queue operations.
 */
#define MOTION_TASK_STACK_WORDS 6144 /* 24 KB — room for 2×256-symbol ping-pong buffers */
#define MOTION_TASK_PRIORITY 23 /* near real-time, above WiFi/lwIP */

/* Step period ceiling in RMT ticks — used when velocity is zero or unknown. */
#define MAX_STEP_PERIOD_TICKS (20U * 1000U * 1000U) /* 10 s at 2 MHz */

/* --------------------------------------------------------------------------
 * RMT batch control
 * -------------------------------------------------------------------------- */

/*
 * Target batch duration in RMT ticks.  Shorter batches give finer ramp
 * granularity but increase CPU overhead.  40k ticks = 20 ms at 2 MHz.
 */
#define RMT_BATCH_TARGET_TICKS 40000U

/*
 * Buffer and batch limits — unified for both axes.
 * 256 symbols (1 KB) fits one GDMA descriptor, batch ≤ 100 steps.
 */
#define RMT_BUFFER_SYMBOLS 256U
#define RMT_BATCH_MAX_STEPS 100U

/*
 * Compute the number of steps for one RMT batch.
 *
 * Targets RMT_BATCH_TARGET_TICKS worth of symbols, clamped to the
 * remaining step count and the hard batch-size ceiling.
 */
static uint32_t
compute_batch_size(uint32_t period_ticks, uint32_t remaining)
{
    uint32_t batch = (period_ticks < RMT_BATCH_TARGET_TICKS) ? RMT_BATCH_TARGET_TICKS / period_ticks : 1;

    if (batch > remaining)
        batch = remaining;

    if (batch > RMT_BATCH_MAX_STEPS)
        batch = RMT_BATCH_MAX_STEPS;

    return batch;
}

TaskHandle_t motors_motion_task_handle = NULL;

/* --------------------------------------------------------------------------
 * Local motion state — active command being executed by the task.
 * -------------------------------------------------------------------------- */
static struct
{
    bool active; /* motion-active flag — own stop/abort lifecycle */
    MotionCommandType active_cmd_type;
    int64_t ra_target; /* steps */
    int64_t dec_target; /* steps */
    int64_t ra_start; /* steps — captured at motion start (for ramps) */
    int64_t dec_start; /* steps */

    /* PulseGuide state — single-writer (motion task only). */
    int64_t ra_guide_deadline_us;
    int64_t dec_guide_deadline_us;
    float ra_guide_offset_dps;
    float dec_guide_offset_dps;
} s_motion;

/* --------------------------------------------------------------------------
 * Slew acceleration / deceleration
 * -------------------------------------------------------------------------- */

/*
 * Minimum slew velocity in centidegrees/second — floor for ramp curves.
 * Raised from 80 (0.8 °/s) to 200 (2.0 °/s) so the last degree doesn't crawl.
 */
#define MIN_SLEW_CDS 200

/* Distance thresholds in centidegrees for speed capping. */
#define SHORT_SLEW_CDS 200 /* constant slow speed below this */
#define GENTLE_SLEW_CDS 800 /* cap target speed below this    */

/*
 * Velocity curve — 100 columns, each value is the percentage of
 * (target_vel − MIN_SLEW_CDS) added on top of the floor.
 *
 * 20 % quadratic accel, 72 % cruise, 8 % sharp decel.
 */
static const uint8_t VELOCITY_CURVE[100] = {
     1,  5, 10, 15, 21, 26, 30, 34, 38, 42, 48, 53, 59, 63, 69, 74, 79, 83, 88, 93,
    100, 100, 100, 100, 100, 100, 100, 100, 100, 100,
    100, 100, 100, 100, 100, 100, 100, 100, 100, 100,
    100, 100, 100, 100, 100, 100, 100, 100, 100, 100,
    100, 100, 100, 100, 100, 100, 100, 100, 100, 100,
    100, 100, 100, 100, 100, 100, 100, 100, 100, 100,
    100, 100, 100, 100, 100, 100, 100, 100, 100, 100,
    100, 100, 100, 100, 100, 100, 100, 100, 100, 100,
    100, 100,
     95, 85, 70, 55, 42, 32, 25, 20,
};

/*
 * Compute the effective velocity for a single axis during a slew.
 *
 * target_vel_cds — target velocity in centidegrees/second (user-specified).
 * travelled_steps — steps taken so far in this slew.
 * distance_steps — total step count for this slew.
 * distance_cds   — total angular distance in centidegrees (precomputed).
 *
 * Returns velocity in deg/s (float).
 */
static float
ramp_velocity(int target_vel_cds, int64_t travelled_steps, int64_t distance_steps, int distance_cds)
{
    if (target_vel_cds == 0)
        return 0.0f;

    if (distance_cds == 0)
        return (float)target_vel_cds / 100.0f;

    if (distance_cds < SHORT_SLEW_CDS)
        return (float)MIN_SLEW_CDS / 100.0f;

    int capped_vel = target_vel_cds;
    if (distance_cds < GENTLE_SLEW_CDS) {
        int speed_limit = 320; /* 3.2 °/s — keep short slews gentle */
        if (capped_vel > speed_limit)
            capped_vel = speed_limit;
    }

    if (travelled_steps < 0)
        travelled_steps = -travelled_steps;
    int percent_index = (int)((int64_t)travelled_steps * 99 / distance_steps);

    int vel = MIN_SLEW_CDS + (capped_vel - MIN_SLEW_CDS) * VELOCITY_CURVE[percent_index] / 100;
    return (float)vel / 100.0f;
}

/* --------------------------------------------------------------------------
 * Step period helpers
 * -------------------------------------------------------------------------- */

/*
 * Convert an angular speed (deg/s) to a step period in RMT ticks.
 * Uses the runtime microstep resolution from the TMC driver.
 * Returns MAX_STEP_PERIOD_TICKS when velocity is effectively zero.
 */
static uint32_t
step_period_ticks(float velocity_dps)
{
    if (fabsf(velocity_dps) < 1e-9f)
        return MAX_STEP_PERIOD_TICKS;

    float deg_per_step = motors_get_deg_per_microstep();
    float period_s = deg_per_step / fabsf(velocity_dps);
    uint32_t ticks = (uint32_t)(period_s * (float)RMT_RESOLUTION_HZ);
    if (ticks < STEP_MIN_PERIOD_TICKS)
        ticks = STEP_MIN_PERIOD_TICKS;
    return ticks;
}

/*
 * Recompute RA and DEC step periods for the current slew point.
 *
 * For MOVE_AXIS the period is constant (target speed).
 * For SLEW the period varies per the velocity ramp curve.
 */
static void
compute_slew_periods(bool is_move_axis, int64_t ra_travelled, int64_t dec_travelled, int64_t ra_dist, int64_t dec_dist,
                     int distance_ra_cds, int distance_dec_cds, uint32_t *ra_period, uint32_t *dec_period)
{
    if (is_move_axis) {
        *ra_period = step_period_ticks(motors_state.ra_speed);
        *dec_period = step_period_ticks(motors_state.dec_speed);
        return;
    }

    float ra_vel = ramp_velocity((int)(motors_state.ra_speed * 100.0f), ra_travelled, ra_dist, distance_ra_cds);

    float dec_vel = ramp_velocity((int)(motors_state.dec_speed * 100.0f), dec_travelled, dec_dist, distance_dec_cds);

    *ra_period = step_period_ticks(ra_vel);
    *dec_period = step_period_ticks(dec_vel);
}

/* --------------------------------------------------------------------------
 * Owns s_motion.active, RMT abort, and task notification.
 * Safe to call from any task.
 * -------------------------------------------------------------------------- */
void
motors_motion_stop(void)
{
    ESP_LOGW(TAG, "STOP called — active=%d, status=%d, cmd_type=%d", s_motion.active, motors_state.status,
             s_motion.active_cmd_type);
    s_motion.active = false;
    motors_rmt_abort_both();
    if (motors_motion_task_handle) {
        xTaskNotify(motors_motion_task_handle, 0, eNoAction);
    }
}

bool
motors_is_active_move_axis(void)
{
    return s_motion.active && s_motion.active_cmd_type == MOTION_CMD_MOVE_AXIS;
}

/* --------------------------------------------------------------------------
 * Motion conditions check — returns false when the current motion should end.
 * -------------------------------------------------------------------------- */
static bool
check_motion_conditions(void)
{
    int64_t ra_diff = s_motion.ra_target - motors_state.ra_steps;
    if (ra_diff < 0)
        ra_diff = -ra_diff;
    int64_t dec_diff = s_motion.dec_target - motors_state.dec_steps;
    if (dec_diff < 0)
        dec_diff = -dec_diff;

    bool ra_has_target = (ra_diff > 0);
    bool dec_has_target = (dec_diff > 0);

    /*
     * Slew / move-axis completion: both axes at target.
     *
     * Use active_cmd_type rather than motors_state.status because the
     * caller may have already set status to TRACKING (resume-after-slew
     * pattern in motors_slew_to_angle / motors_slew_axis_*) before the
     * motion task reaches the target.  Checking the authoritative
     * s_motion field guarantees completion is always detected.
     */
    if ((s_motion.active_cmd_type == MOTION_CMD_SLEW || s_motion.active_cmd_type == MOTION_CMD_MOVE_AXIS) &&
        !ra_has_target && !dec_has_target) {
        motors_state.status = MOTORS_STATUS_READY;
        motors_state.tracking = TRACKING_NONE;
        s_motion.active = false;

        return false;
    }

    /* External tracking stop. */
    if (motors_state.tracking == TRACKING_NONE && motors_state.status == MOTORS_STATUS_TRACKING) {
        motors_state.status = MOTORS_STATUS_READY;
        s_motion.active = false;

        return false;
    }

    return true;
}

/* --------------------------------------------------------------------------
 * Motion lifecycle helpers — factor out the repeated housekeeping
 * that appears after every RMT error in the slewing loop.
 * -------------------------------------------------------------------------- */

static void
finish_motion(void)
{
    s_motion.active = false;
    motors_state.status = MOTORS_STATUS_READY;
    motors_state.tracking = TRACKING_NONE;
}

static void
abort_motion(void)
{
    motors_rmt_abort_both();
    finish_motion();
}

/* --------------------------------------------------------------------------
 * PulseGuide helpers
 * -------------------------------------------------------------------------- */

/*
 * Apply or extend a PulseGuide deadline for one axis.
 *
 * Collision policy: same direction extends the deadline,
 * opposite direction replaces it.
 */
static void
update_guide(int64_t *deadline_us, float *offset_dps, const MotionCommand *cmd)
{
    int64_t new_deadline = esp_timer_get_time() + (int64_t)cmd->guide_duration_ms * 1000;

    if (*deadline_us && *offset_dps == cmd->guide_offset_dps) {
        if (new_deadline > *deadline_us)
            *deadline_us = new_deadline;
        return;
    }

    *deadline_us = new_deadline;
    *offset_dps = cmd->guide_offset_dps;
}

/* Expire a single guide deadline if elapsed.  Returns true if expired. */
static bool
expire_guide(int64_t *deadline_us, float *offset_dps, int64_t now)
{
    if (*deadline_us && now >= *deadline_us) {
        *deadline_us = 0;
        *offset_dps = 0.0f;
        return true;
    }
    return false;
}

/* --------------------------------------------------------------------------
 * Command processing — handle one MotionCommand and set up motion state.
 *
 * Only motion-producing commands (SLEW, TRACK, MOVE_AXIS) go through
 * the queue.  Stop / park / enable are handled directly
 * by their callers via motors_motion_stop() + motors_state update.
 * -------------------------------------------------------------------------- */
static void
process_command(MotionCommand cmd)
{
    s_motion.active_cmd_type = cmd.type;

    switch (cmd.type) {
        case MOTION_CMD_SLEW:
            motors_state.ra_speed = cmd.ra_speed;
            motors_state.dec_speed = cmd.dec_speed;
            motors_state.status = MOTORS_STATUS_SLEWING;
            motors_state.tracking = TRACKING_NONE;

            if (cmd.relative) {
                s_motion.ra_target = motors_state.ra_steps + motors_deg_to_steps(cmd.ra_delta_deg);
                s_motion.dec_target = motors_state.dec_steps + motors_deg_to_steps(cmd.dec_delta_deg);
            } else {
                s_motion.ra_target = motors_deg_to_steps(cmd.ra_target_deg);
                s_motion.dec_target = motors_deg_to_steps(cmd.dec_target_deg);
            }
            s_motion.ra_start = motors_state.ra_steps;
            s_motion.dec_start = motors_state.dec_steps;
            s_motion.active = true;

            break;

        case MOTION_CMD_TRACK:
            motors_state.ra_speed = cmd.ra_speed;
            motors_state.dec_speed = 0.0f;
            motors_state.status = MOTORS_STATUS_TRACKING;
            motors_state.tracking = cmd.tracking_mode;

            /*
             * Tracking runs open-ended: target is set to the axis limit so
             * the loop never completes on its own — it only stops when an
             * external status change (STOP, PARK) is detected.
             *
             * Hemisphere selection: positive velocity → ra_max (northern),
             * negative velocity → ra_min (southern).  The sign is set by
             * motors_start_tracking based on site latitude.
             */
            s_motion.ra_target = (cmd.ra_speed >= 0.0f) ? motors_deg_to_steps(motors_state.limits.ra_max)
                                                        : motors_deg_to_steps(motors_state.limits.ra_min);
            s_motion.dec_target = motors_state.dec_steps;
            s_motion.ra_start = motors_state.ra_steps;
            s_motion.dec_start = motors_state.dec_steps;
            s_motion.active = true;

            break;

        case MOTION_CMD_MOVE_AXIS:
            motors_state.ra_speed = fabsf(cmd.ra_speed);
            motors_state.dec_speed = fabsf(cmd.dec_speed);
            motors_state.status = MOTORS_STATUS_SLEWING;
            motors_state.tracking = TRACKING_NONE;

            s_motion.ra_target = (cmd.ra_speed > 0.0f)   ? motors_deg_to_steps(motors_state.limits.ra_max)
                                 : (cmd.ra_speed < 0.0f) ? motors_deg_to_steps(motors_state.limits.ra_min)
                                                         : motors_state.ra_steps;
            s_motion.dec_target = (cmd.dec_speed > 0.0f)   ? motors_deg_to_steps(motors_state.limits.dec_max)
                                  : (cmd.dec_speed < 0.0f) ? motors_deg_to_steps(motors_state.limits.dec_min)
                                                           : motors_state.dec_steps;

            s_motion.ra_start = motors_state.ra_steps;
            s_motion.dec_start = motors_state.dec_steps;
            s_motion.active = true;

            break;

        case MOTION_CMD_PULSE_GUIDE:
            if (cmd.guide_axis == 0) {
                update_guide(&s_motion.ra_guide_deadline_us, &s_motion.ra_guide_offset_dps, &cmd);
            } else {
                update_guide(&s_motion.dec_guide_deadline_us, &s_motion.dec_guide_offset_dps, &cmd);
            }
            motors_state.guiding = true;
            /* Don't set s_motion.active — the existing loop handles it. */
            break;
    }
}
/*
 * Encode one axis batch — shared between the initial batch and pre-encoded
 * batches inside the double-buffered slewing loop.  Preserves the exact same
 * encoding logic (MOVE_AXIS constant-period vs SLEW per-step ramp).
 */
static uint32_t
encode_axis_batch(rmt_symbol_word_t *buf, uint32_t max_sym, uint32_t batch, uint32_t period_ticks, float speed_dps,
                  int64_t base_travelled, int64_t dist_steps, int distance_cds, bool is_move_axis)
{
    if (batch == 0)
        return 0;

    if (is_move_axis) {
        return motors_rmt_encode_steps(buf, max_sym, period_ticks, batch);
    }

    uint32_t total_sym = 0;
    rmt_symbol_word_t *sym = buf;
    for (uint32_t i = 0; i < batch && total_sym < max_sym; i++) {
        int64_t travelled = base_travelled + (int64_t)i;
        float vel = ramp_velocity((int)(speed_dps * 100.0f), travelled, dist_steps, distance_cds);
        uint32_t period = step_period_ticks(vel);
        uint32_t n = motors_rmt_encode_steps(sym, max_sym - total_sym, period, 1);
        sym += n;
        total_sym += n;
    }
    return total_sym;
}

/*
 * Double-buffered slewing / move-axis motion loop.
 *
 * Ping-pong strategy:
 *   buf[ping]  — currently transmitting via RMT+DMA
 *   buf[pong]  — pre-encoded while ping is in flight
 *
 *   First iteration:  encode → transmit (with semaphore drain)
 *   Each subsequent:  pre-encode pong → wait ping → confirm ping →
 *                      transmit pong (no drain, immediate) → swap
 *
 * This eliminates the software encoding gap between batches — when one
 * batch's RMT transmission finishes, the next batch is already queued
 * in the RMT TX FIFO and starts with zero gap.
 */
static void
slewing_loop_rmt(void)
{
    /* Total step counts — exact from int64_t targets. */
    int64_t ra_dist = s_motion.ra_target - s_motion.ra_start;
    int64_t dec_dist = s_motion.dec_target - s_motion.dec_start;
    if (ra_dist < 0)
        ra_dist = -ra_dist;
    if (dec_dist < 0)
        dec_dist = -dec_dist;
    uint32_t total_ra_steps = (uint32_t)ra_dist;
    uint32_t total_dec_steps = (uint32_t)dec_dist;
    uint32_t ra_steps_done = 0;
    uint32_t dec_steps_done = 0;

    /* Distances in centidegrees for ramp threshold comparisons. */
    int distance_ra_cds = (int)(motors_steps_to_deg(ra_dist) * 100.0f);
    int distance_dec_cds = (int)(motors_steps_to_deg(dec_dist) * 100.0f);

    /*
     * Direction is constant for a slew — set DIR pins once.
     */
    MotorDirection ra_dir = (s_motion.ra_target > motors_state.ra_steps) ? MOTOR_DIRECTION_POSITIVE
                                                                         : MOTOR_DIRECTION_NEGATIVE;
    MotorDirection dec_dir = (s_motion.dec_target > motors_state.dec_steps) ? MOTOR_DIRECTION_POSITIVE
                                                                            : MOTOR_DIRECTION_NEGATIVE;
    motors_hw_set_direction_ra(ra_dir);
    motors_hw_set_direction_dec(dec_dir);

    int ra_sign = (ra_dir == MOTOR_DIRECTION_POSITIVE) ? 1 : -1;
    int dec_sign = (dec_dir == MOTOR_DIRECTION_POSITIVE) ? 1 : -1;

    /*
     * Double-buffer: ping = in-flight, pong = being pre-encoded.
     * Each holds one batch.  Stack-allocated, DMA-safe on ESP32-S3.
     */
    rmt_symbol_word_t ra_buf[2][RMT_BUFFER_SYMBOLS];
    rmt_symbol_word_t dec_buf[2][RMT_BUFFER_SYMBOLS];
    uint32_t ra_num[2] = { 0, 0 };
    uint32_t dec_num[2] = { 0, 0 };
    uint32_t ra_batch[2] = { 0, 0 };
    uint32_t dec_batch[2] = { 0, 0 };
    int ping = 0; /* currently transmitting */
    int pong = 1; /* being pre-encoded for next */

    uint32_t ra_period = MAX_STEP_PERIOD_TICKS;
    uint32_t dec_period = MAX_STEP_PERIOD_TICKS;
    int64_t last_ramp_recalc_us = 0;
    bool is_move_axis = (s_motion.active_cmd_type == MOTION_CMD_MOVE_AXIS);

    /* ── Compute + encode + submit first batch (buf[0]) ── */

    /* Recalculate velocity for the initial batch. */
    compute_slew_periods(is_move_axis, 0, 0, ra_dist, dec_dist, distance_ra_cds, distance_dec_cds, &ra_period,
                         &dec_period);
    last_ramp_recalc_us = esp_timer_get_time();

    /* Compute first batch sizes. */
    ra_batch[0] = compute_batch_size(ra_period, total_ra_steps - ra_steps_done);
    dec_batch[0] = compute_batch_size(dec_period, total_dec_steps - dec_steps_done);

    /* Encode first batch. */
    int64_t ra_trav_base = (int64_t)ra_sign * (motors_state.ra_steps - s_motion.ra_start);
    int64_t dec_trav_base = (int64_t)dec_sign * (motors_state.dec_steps - s_motion.dec_start);

    ra_num[0] = encode_axis_batch(ra_buf[0], RMT_BUFFER_SYMBOLS, ra_batch[0], ra_period, motors_state.ra_speed,
                                  ra_trav_base, ra_dist, distance_ra_cds, is_move_axis);
    dec_num[0] = encode_axis_batch(dec_buf[0], RMT_BUFFER_SYMBOLS, dec_batch[0], dec_period, motors_state.dec_speed,
                                   dec_trav_base, dec_dist, distance_dec_cds, is_move_axis);

    /* Check stop before touching hardware. */
    if (!s_motion.active)
        return;

    /* Submit first batch (with semaphore drain). */
    esp_err_t tx_err = ESP_OK;
    if (ra_batch[0] > 0)
        tx_err = motors_rmt_transmit_ra(ra_buf[0], ra_num[0]);
    if (dec_batch[0] > 0 && tx_err == ESP_OK)
        tx_err = motors_rmt_transmit_dec(dec_buf[0], dec_num[0]);
    if (tx_err != ESP_OK) {
        ESP_LOGE(TAG, "RMT transmit failed: %s", esp_err_to_name(tx_err));
        abort_motion();
        return;
    }

    /*
     * Pre-encode AND pre-submit the second batch (buf[1]) while buf[0]
     * is transmitting.  This is the key: by the time buf[0] finishes,
     * buf[1] is already queued in the RMT TX FIFO and starts with
     * zero CPU-dependent gap.
     */
    {
        int64_t ra_proj = (int64_t)ra_sign *
                          (motors_state.ra_steps + (int64_t)ra_sign * (int64_t)ra_batch[0] - s_motion.ra_start);
        int64_t dec_proj = (int64_t)dec_sign *
                           (motors_state.dec_steps + (int64_t)dec_sign * (int64_t)dec_batch[0] - s_motion.dec_start);

        ra_batch[1] = compute_batch_size(ra_period, total_ra_steps - ra_steps_done - ra_batch[0]);
        dec_batch[1] = compute_batch_size(dec_period, total_dec_steps - dec_steps_done - dec_batch[0]);

        ra_num[1] = encode_axis_batch(ra_buf[1], RMT_BUFFER_SYMBOLS, ra_batch[1], ra_period, motors_state.ra_speed,
                                      ra_proj, ra_dist, distance_ra_cds, is_move_axis);
        dec_num[1] = encode_axis_batch(dec_buf[1], RMT_BUFFER_SYMBOLS, dec_batch[1], dec_period, motors_state.dec_speed,
                                       dec_proj, dec_dist, distance_dec_cds, is_move_axis);

        /* Submit buf[1] while buf[0] is still transmitting — ZERO GAP */
        if (ra_batch[1] > 0 || dec_batch[1] > 0) {
            tx_err = ESP_OK;
            if (ra_batch[1] > 0)
                tx_err = motors_rmt_transmit_no_drain_ra(ra_buf[1], ra_num[1]);
            if (dec_batch[1] > 0 && tx_err == ESP_OK)
                tx_err = motors_rmt_transmit_no_drain_dec(dec_buf[1], dec_num[1]);
            if (tx_err != ESP_OK) {
                ESP_LOGE(TAG, "RMT transmit failed: %s", esp_err_to_name(tx_err));
                abort_motion();
                return;
            }
        }
    }

    /* ── Main double-buffered loop — submit BEFORE wait ──
     *
     * Invariant at loop entry:
     *   buf[0] — already submitted, currently transmitting
     *   buf[1] — already submitted, queued in RMT TX FIFO
     *
     * Each iteration:
     *   1. wait for oldest submitted batch (ping)
     *   2. confirm position, check conditions
     *   3. encode + submit next batch into the now-free ping buffer
     *      (submit BEFORE the next wait — stays ahead by 1)
     *   4. swap ping↔pong
     */

    ping = 0; /* waiting for this one (just submitted above) */
    pong = 1; /* already queued (submitted right after ping) */

    while (s_motion.active) {
        /*
         * 1. Wait for ping to finish.  Pong is already queued in the
         *    RMT TX FIFO — when ping's DMA completes, pong starts
         *    immediately with zero CPU involvement.
         */
        esp_err_t wait_err = ESP_OK;
        if (ra_batch[ping] > 0)
            wait_err = motors_rmt_wait_ra(pdMS_TO_TICKS(500));
        if (dec_batch[ping] > 0 && wait_err == ESP_OK)
            wait_err = motors_rmt_wait_dec(pdMS_TO_TICKS(500));

        if (wait_err != ESP_OK) {
            ESP_LOGW(TAG, "RMT wait %s", (wait_err == ESP_ERR_TIMEOUT) ? "timeout" : "error");
            abort_motion();
            break;
        }

        /* 2. Ping batch confirmed — apply its steps. */
        motors_state.ra_steps += (int64_t)ra_sign * (int64_t)ra_batch[ping];
        motors_state.dec_steps += (int64_t)dec_sign * (int64_t)dec_batch[ping];
        ra_steps_done += ra_batch[ping];
        dec_steps_done += dec_batch[ping];

        /* Conditions check after position update. */
        if (!check_motion_conditions())
            break;
        if (!s_motion.active) {
            ESP_LOGW(TAG, "Motion aborted");
            break;
        }

        /* Target reached? */
        if (ra_steps_done >= total_ra_steps && dec_steps_done >= total_dec_steps) {
            finish_motion();
            break;
        }

        /*
         * 3. Encode the next batch into the now-free ping buffer.
         *    Pong is still queued/transmitting while we do this.
         */
        {
            int64_t ra_proj_trav = (int64_t)ra_sign * (motors_state.ra_steps +
                                                       (int64_t)ra_sign * (int64_t)ra_batch[pong] - s_motion.ra_start);
            int64_t dec_proj_trav =
                    (int64_t)dec_sign *
                    (motors_state.dec_steps + (int64_t)dec_sign * (int64_t)dec_batch[pong] - s_motion.dec_start);

            int64_t now = esp_timer_get_time();
            if (now - last_ramp_recalc_us > 5000) {
                compute_slew_periods(is_move_axis, ra_proj_trav, dec_proj_trav, ra_dist, dec_dist, distance_ra_cds,
                                     distance_dec_cds, &ra_period, &dec_period);
                last_ramp_recalc_us = now;
            }

            ra_batch[ping] = compute_batch_size(ra_period, total_ra_steps - ra_steps_done - ra_batch[pong]);
            dec_batch[ping] = compute_batch_size(dec_period, total_dec_steps - dec_steps_done - dec_batch[pong]);

            ra_num[ping] = encode_axis_batch(ra_buf[ping], RMT_BUFFER_SYMBOLS, ra_batch[ping], ra_period,
                                             motors_state.ra_speed, ra_proj_trav, ra_dist, distance_ra_cds,
                                             is_move_axis);
            dec_num[ping] = encode_axis_batch(dec_buf[ping], RMT_BUFFER_SYMBOLS, dec_batch[ping], dec_period,
                                              motors_state.dec_speed, dec_proj_trav, dec_dist, distance_dec_cds,
                                              is_move_axis);
        }

        /*
         * 4. Submit the new batch NOW — before we wait for pong.
         *    This puts it in the RMT TX queue behind pong.
         *    When pong finishes → this batch starts with zero gap.
         */
        if (ra_batch[ping] > 0 || dec_batch[ping] > 0) {
            tx_err = ESP_OK;
            if (ra_batch[ping] > 0)
                tx_err = motors_rmt_transmit_no_drain_ra(ra_buf[ping], ra_num[ping]);
            if (dec_batch[ping] > 0 && tx_err == ESP_OK)
                tx_err = motors_rmt_transmit_no_drain_dec(dec_buf[ping], dec_num[ping]);
            if (tx_err != ESP_OK) {
                ESP_LOGE(TAG, "RMT transmit failed: %s", esp_err_to_name(tx_err));
                abort_motion();
                break;
            }
        }

        /* 5. Swap: the batch we just submitted (ping) becomes the queued
         *    one; the previously queued one (pong) becomes what we wait
         *    for next iteration. */
        {
            int tmp = ping;
            ping = pong;
            pong = tmp;
        }
    }
}

/* --------------------------------------------------------------------------
 * Tracking motion loop — absolute-time fractional accumulator + RMT pulse.
 *
 * Designed for continuous open-ended tracking (sidereal, solar, lunar)
 * where timing precision must hold over arbitrarily long sessions.
 *
 * Scheduling strategy (hybrid sleep + fine-wait → RMT):
 *
 *   dt = now - last_time
 *   accumulator += dt / period_us     (period_us converted from RMT ticks)
 *   while (accumulator >= 1.0):  encode 1 step, rmt_transmit, wait, acc -= 1.0
 *
 *   deadline = now + (1.0 - accumulator) * period_us    (µs-exact)
 *   if deadline - now > 2 ms:
 *       vTaskDelay most of it (capped 50 ms, yields CPU → near-zero consumption)
 *   fine-wait remaining margin with busy-wait → µs precision
 *
 * The actual STEP pulse is generated by the RMT peripheral with zero
 * jitter.  The fine-wait determines *when* the pulse begins; the RMT
 * determines the pulse *shape*.  Long idle periods (tracking ≈ 841 ms)
 * are streamed as idle-only RMT symbols via DMA — the CPU sleeps
 * through the entire step period.
 *
 * Only RA is stepped during tracking; DEC velocity is always zero.
 * -------------------------------------------------------------------------- */

/*
 * Emit one RA tracking step via RMT.
 *
 * Encodes a single step pulse, transmits it on the RA channel,
 * waits for completion, and updates the position accumulator.
 *
 * Returns true on success.  On failure (limit hit, RMT error,
 * or motion aborted) cleans up state and returns false — the
 * caller must return from tracking_loop_rmt().
 */
static bool
emit_ra_tracking_step(int direction, rmt_symbol_word_t *buffer)
{
    int64_t next = motors_state.ra_steps + (int64_t)direction;

    if (!motors_is_valid_ra_steps(next)) {
        ESP_LOGW(TAG, "RA limit at %.3f deg", (double)motors_steps_to_deg(motors_state.ra_steps));
        s_motion.active = false;
        motors_state.status = MOTORS_STATUS_READY;
        motors_state.tracking = TRACKING_NONE;
        motors_state.guiding = false;
        return false;
    }

    if (!s_motion.active)
        return false;

    motors_rmt_encode_pulse(buffer);

    esp_err_t tx_err = motors_rmt_transmit_ra(buffer, 1);
    if (tx_err != ESP_OK) {
        ESP_LOGE(TAG, "RMT RA tx fail: %s", esp_err_to_name(tx_err));
        motors_rmt_abort_ra();
        s_motion.active = false;
        motors_state.status = MOTORS_STATUS_READY;
        motors_state.tracking = TRACKING_NONE;
        motors_state.guiding = false;
        return false;
    }

    esp_err_t wait_err = motors_rmt_wait_ra(pdMS_TO_TICKS(2000));
    if (wait_err != ESP_OK) {
        ESP_LOGW(TAG, "RMT RA wait %s", (wait_err == ESP_ERR_TIMEOUT) ? "timeout" : "error");
        motors_rmt_abort_ra();
        s_motion.active = false;
        motors_state.status = MOTORS_STATUS_READY;
        motors_state.tracking = TRACKING_NONE;
        motors_state.guiding = false;
        return false;
    }

    if (!s_motion.active)
        return false;

    motors_state.ra_steps = next;
    return true;
}

/*
 * Emit one DEC guide step via RMT.
 *
 * Handles direction pin management, limit checking, and RMT
 * transmission for a single DEC microstep during guiding.
 *
 * On success returns true.  On failure:
 *   - If s_motion.active was cleared externally, the caller must
 *     return from tracking_loop_rmt().
 *   - Otherwise (limit hit, RMT error) the guide state is cleared
 *     and the caller must break from the emit loop.
 */
static bool
emit_dec_guide_step(int direction, rmt_symbol_word_t *buffer, double *dec_phase, int *dec_dir_set)
{
    /* Set direction pin if it changed. */
    if (*dec_dir_set != direction) {
        motors_hw_set_direction_dec((direction > 0) ? MOTOR_DIRECTION_POSITIVE : MOTOR_DIRECTION_NEGATIVE);
        *dec_dir_set = direction;
    }

    int64_t next = motors_state.dec_steps + (int64_t)direction;

    if (!motors_is_valid_dec_steps(next)) {
        ESP_LOGW(TAG, "DEC limit at %.3f deg", (double)motors_steps_to_deg(motors_state.dec_steps));
        s_motion.dec_guide_deadline_us = 0;
        s_motion.dec_guide_offset_dps = 0.0f;
        *dec_dir_set = 0;
        *dec_phase = 0.0;
        return false;
    }

    if (!s_motion.active) {
        *dec_phase = 0.0;
        return false;
    }

    motors_rmt_encode_pulse(buffer);

    esp_err_t tx_err = motors_rmt_transmit_dec(buffer, 1);
    if (tx_err != ESP_OK) {
        ESP_LOGE(TAG, "RMT DEC tx fail: %s", esp_err_to_name(tx_err));
        motors_rmt_abort_dec();
        s_motion.dec_guide_deadline_us = 0;
        s_motion.dec_guide_offset_dps = 0.0f;
        *dec_dir_set = 0;
        *dec_phase = 0.0;
        return false;
    }

    esp_err_t wait_err = motors_rmt_wait_dec(pdMS_TO_TICKS(2000));
    if (wait_err != ESP_OK) {
        ESP_LOGW(TAG, "RMT DEC wait %s", (wait_err == ESP_ERR_TIMEOUT) ? "timeout" : "error");
        motors_rmt_abort_dec();
        s_motion.dec_guide_deadline_us = 0;
        s_motion.dec_guide_offset_dps = 0.0f;
        *dec_dir_set = 0;
        *dec_phase = 0.0;
        return false;
    }

    if (!s_motion.active) {
        *dec_phase = 0.0;
        return false;
    }

    motors_state.dec_steps = next;
    return true;
}

static void
tracking_loop_rmt(void)
{
    const int64_t FINE_MARGIN_US = 2000;
    const float deg_per_step = motors_get_deg_per_microstep();

    /* Signed phase accumulators in microsteps. */
    double ra_phase = 0.0;
    double dec_phase = 0.0;
    int64_t last_time_us = esp_timer_get_time();
    int64_t last_check_us = last_time_us;
    int64_t last_guide_check_us = last_time_us;

    int ra_sign = (motors_state.ra_speed >= 0.0f) ? 1 : -1;
    motors_hw_set_direction_ra((ra_sign > 0) ? MOTOR_DIRECTION_POSITIVE : MOTOR_DIRECTION_NEGATIVE);

    int dec_dir_set = 0;
    rmt_symbol_word_t ra_sym[RMT_BUFFER_SYMBOLS];
    rmt_symbol_word_t dec_sym[RMT_BUFFER_SYMBOLS];

    while (s_motion.active) {
        int64_t now = esp_timer_get_time();
        double dt_s = (double)(now - last_time_us) / 1000000.0;
        last_time_us = now;

        /* Guide expiry (every 10 ms). */
        if (now - last_guide_check_us >= 10000) {
            last_guide_check_us = now;
            expire_guide(&s_motion.ra_guide_deadline_us, &s_motion.ra_guide_offset_dps, now);
            if (expire_guide(&s_motion.dec_guide_deadline_us, &s_motion.dec_guide_offset_dps, now)) {
                dec_dir_set = 0;
            }
            motors_state.guiding = (s_motion.ra_guide_deadline_us != 0 || s_motion.dec_guide_deadline_us != 0);
        }

        /* Effective velocities (deg/s). */
        float ra_vel = motors_state.ra_speed;
        if (s_motion.ra_guide_deadline_us)
            ra_vel += s_motion.ra_guide_offset_dps;
        float dec_vel = 0.0f;
        if (s_motion.dec_guide_deadline_us)
            dec_vel = s_motion.dec_guide_offset_dps;

        /* Accumulate signed phase (microsteps). */
        ra_phase += (double)ra_vel * dt_s / (double)deg_per_step;
        dec_phase += (double)dec_vel * dt_s / (double)deg_per_step;

        /* Conditions check (every ~500 us). */
        if (now - last_check_us >= 500) {
            last_check_us = now;
            if (!check_motion_conditions())
                break;
            if (motors_state.status != MOTORS_STATUS_TRACKING && !motors_state.guiding) {
                break;
            }
        }

        /* ── Emit RA steps ─────────────────────────────────── */
        while (fabs(ra_phase) >= 1.0) {
            int direction = (ra_phase > 0.0) ? 1 : -1;

            if (!emit_ra_tracking_step(direction, ra_sym))
                return;

            ra_phase -= (double)direction;
        }

        /* ── Emit DEC steps ─────────────────────────────────── */
        while (dec_phase >= 1.0) {
            if (!emit_dec_guide_step(1, dec_sym, &dec_phase, &dec_dir_set)) {
                if (!s_motion.active)
                    return;
                break;
            }
            dec_phase -= 1.0;
        }
        while (dec_phase <= -1.0) {
            if (!emit_dec_guide_step(-1, dec_sym, &dec_phase, &dec_dir_set)) {
                if (!s_motion.active)
                    return;
                break;
            }
            dec_phase += 1.0;
        }

        /* ── Sleep ─────────────────────────────────────────── */
        double ra_s = (ra_vel != 0.0) ? (1.0 - ra_phase) * deg_per_step / fabs((double)ra_vel) : 1e9;
        double dec_s = (dec_vel != 0.0) ? (1.0 - dec_phase) * deg_per_step / fabs((double)dec_vel) : 1e9;
        int64_t w_us = (int64_t)((ra_s < dec_s ? ra_s : dec_s) * 1000000.0);
        if (w_us < 0)
            w_us = 0;

        if (w_us > FINE_MARGIN_US) {
            uint32_t sm = (uint32_t)((w_us - FINE_MARGIN_US) / 1000);
            if (sm > 50)
                sm = 50;
            if (sm < 1)
                sm = 1;
            ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(sm));
            continue;
        }
        if (w_us > 0) {
            int64_t dl = esp_timer_get_time() + w_us;
            while (esp_timer_get_time() < dl) {
                if ((esp_timer_get_time() & 0x1FF) == 0)
                    taskYIELD();
            }
        }
    }
}

/* --------------------------------------------------------------------------
 * Motion loop dispatcher.
 *
 * Routes to the appropriate execution path based on mount status:
 *   - TRACKING (sidereal / solar / lunar) → tracking_loop_rmt()
 *     (absolute-time fractional accumulator, zero cumulative error)
 *   - Everything else (SLEWING, MOVE_AXIS, etc.) → slewing_loop_rmt()
 *     (batched RMT with ramps)
 * -------------------------------------------------------------------------- */
static void
motion_loop(void)
{
    /* Standalone guide pulse (no tracking) — use the tracking loop
     * with zero base rate.  It handles DEC-only and RA-only guiding. */
    if ((motors_state.status == MOTORS_STATUS_TRACKING && motors_state.tracking != TRACKING_NONE) ||
        motors_state.guiding) {
        tracking_loop_rmt();
    } else {
        slewing_loop_rmt();
    }
}

/* --------------------------------------------------------------------------
 * Motion task entry point.
 *
 * Blocks on the command queue when idle. When a motion-producing command
 * arrives (SLEW, TRACK, or MOVE_AXIS), enters motion_loop() which dispatches
 * to the appropriate RMT-driven execution path.
 * Stop / park / enable are handled directly by their
 * callers via motors_motion_stop() + motors_state update.
 * -------------------------------------------------------------------------- */
static void
motors_motion_task_run(void *arg)
{
    (void)arg;

    /* Initialize RMT from this task — pins the RMT ISR to core 1
     * where WiFi/lwIP never runs, eliminating ISR latency jitter. */
    esp_err_t rmt_err = motors_rmt_init();
    if (rmt_err != ESP_OK) {
        ESP_LOGE(TAG, "motors_rmt_init failed: %s", esp_err_to_name(rmt_err));
        motors_state.status = MOTORS_STATUS_DISABLED;
        motors_hw_disable();
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "RMT initialized on core 1");

    while (true) {
        MotionCommand cmd;
        if (xQueueReceive(motion_cmd_queue, &cmd, portMAX_DELAY) != pdTRUE)
            continue;

        process_command(cmd);

        if (cmd.type == MOTION_CMD_SLEW || cmd.type == MOTION_CMD_TRACK || cmd.type == MOTION_CMD_MOVE_AXIS) {
            motion_loop();
        }
        /* PULSE_GUIDE: the tracking loop picks up guide state on each
         * iteration.  If tracking is NOT active, the guide pulse runs
         * via a minimal single-step loop below. */
        if (cmd.type == MOTION_CMD_PULSE_GUIDE && motors_state.status != MOTORS_STATUS_TRACKING) {
            /* Standalone guiding — run a short motion loop just for
             * the guide pulse.  DEC-only: no tracking base rate. */
            motion_loop();
        }

        /* Clear guiding flag when both axes are idle. */
        bool ra_active = (s_motion.ra_guide_deadline_us != 0) && (esp_timer_get_time() < s_motion.ra_guide_deadline_us);
        bool dec_active = (s_motion.dec_guide_deadline_us != 0) &&
                          (esp_timer_get_time() < s_motion.dec_guide_deadline_us);
        motors_state.guiding = (ra_active || dec_active);
    }
}

/* --------------------------------------------------------------------------
 * Public API
 * -------------------------------------------------------------------------- */

void
motors_motion_task_init(void)
{
    xTaskCreatePinnedToCore(motors_motion_task_run, "motors_motion", MOTION_TASK_STACK_WORDS, NULL,
                            MOTION_TASK_PRIORITY, &motors_motion_task_handle,
                            1); /* dedicated core — isolated from WiFi/lwIP on CPU 0 */

    /* Report stack high-water mark for diagnostics. */
    if (motors_motion_task_handle != NULL) {
        UBaseType_t high_water = uxTaskGetStackHighWaterMark(motors_motion_task_handle);
        ESP_LOGI(TAG, "Stack high-water mark: %lu words (total %d)", (unsigned long)high_water,
                 MOTION_TASK_STACK_WORDS);
    }
}
