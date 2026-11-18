/* Motors - motors_move_axis_speed.c
 *
 * Purpose: request continuous single-axis speed motion.
 * Positive rate = forward, negative = reverse, zero = stop that axis.
 * Used by MoveAxis, joystick, and guiding.
 *
 * Speeds are clamped to [-MOTORS_MAX_MOVE_AXIS_SPEED_DPS, +MOTORS_MAX_MOVE_AXIS_SPEED_DPS].
 */
#include "motors.h"
#include "motors_internal.h"

#include <math.h>

/* MoveAxis setpoint — the authoritative continuous-move rate.  Writing it
 * bumps the version, which the motion task watches to pick up the newest rate
 * without relying on the (FIFO) command queue. */
volatile float motors_move_axis_ra_dps = 0.0f;
volatile float motors_move_axis_dec_dps = 0.0f;
volatile uint32_t motors_move_axis_version = 0;

MotorResultCode motors_set_move_axis_speed(float ra_speed, float dec_speed) {
    if (motors_status_is_error(motors_state.status)) {
        return MOTOR_ERR_HARDWARE_ERROR;
    }

    /* Defense in depth: the protocol parser already rejects non-finite floats,
     * but a NaN rate would silently slip past the clamp below. */
    if (!isfinite(ra_speed) || !isfinite(dec_speed)) {
        return MOTOR_ERR_OUT_OF_RANGE;
    }

    if (ra_speed == 0.0f && dec_speed == 0.0f) {
        /* Zero is a stop: publish the zero setpoint and stop everything. */
        portENTER_CRITICAL(&motors_state_lock);
        motors_move_axis_ra_dps = 0.0f;
        motors_move_axis_dec_dps = 0.0f;
        motors_move_axis_version++;
        portEXIT_CRITICAL(&motors_state_lock);
        return motors_stop();
    }

    if (motors_status_is_parked(motors_state.status)) {
        return MOTOR_ERR_PARKED;
    }

    /* Clamp to the manual move-axis ceiling. */
    float max_dps = MOTORS_MAX_MOVE_AXIS_SPEED_DPS;
    if (ra_speed > max_dps) ra_speed = max_dps;
    if (ra_speed < -max_dps) ra_speed = -max_dps;
    if (dec_speed > max_dps) dec_speed = max_dps;
    if (dec_speed < -max_dps) dec_speed = -max_dps;

    MotionCommand cmd = {
        .type = MOTION_CMD_MOVE_AXIS,
        .ra_target_deg = 0.0f, /* set by the task from limits */
        .dec_target_deg = 0.0f,
        .ra_speed = ra_speed,
        .dec_speed = dec_speed,
        .tracking_mode = TRACKING_NONE,
    };
    if (!motors_queue_put(&cmd)) {
        /* A rejected order must not alter an in-flight move: publish the
         * setpoint only after the enqueue is confirmed. */
        return MOTOR_ERR_BUSY;
    }

    /* Publish both values and their version together, so the motion task never
     * observes a half-written combination. */
    portENTER_CRITICAL(&motors_state_lock);
    motors_move_axis_ra_dps = ra_speed;
    motors_move_axis_dec_dps = dec_speed;
    motors_move_axis_version++;
    portEXIT_CRITICAL(&motors_state_lock);

    return MOTOR_OK;
}
