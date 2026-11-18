/* Motors - motors_limits.c
 *
 * Purpose: load, persist, and update operational axis limits.
 *
 * Limits are stored in NVS under the "mount" namespace alongside site
 * settings.  On first boot (no NVS) they default to the factory values
 * below.  The UI can override them at runtime; changes are committed
 * immediately and survive reboot.
 */
#include "motors.h"
#include "motors_internal.h"

#include <math.h>

#include "nvs.h"
#include "utils/utils.h"

#include "esp_log.h"

static const char *TAG = "MOTORS_LIMITS";
static const char *NVS_NS = "mount";

#define LIMITS_DEFAULT_RA_MIN  -100.0f
#define LIMITS_DEFAULT_RA_MAX   100.0f
#define LIMITS_DEFAULT_DEC_MIN -150.0f
#define LIMITS_DEFAULT_DEC_MAX  150.0f

/*
 * Absolute mechanical range (degrees).  Each axis is physically meaningful
 * within a half-turn: the DEC body sweeps ±180° between the two celestial
 * poles and the RA body points below the horizon beyond ±180°.  Stored limits
 * must stay inside this range and keep min < max, or the clamp logic breaks.
 */
#define LIMITS_MECHANICAL_RA_RANGE  180.0f
#define LIMITS_MECHANICAL_DEC_RANGE 180.0f

/* True when a full limit config is usable: finite, ordered, within range. */
static bool limits_are_valid(float ra_min, float ra_max,
                             float dec_min, float dec_max) {
    if (!isfinite(ra_min) || !isfinite(ra_max)
        || !isfinite(dec_min) || !isfinite(dec_max)) {
        return false;
    }
    if (ra_min >= ra_max || dec_min >= dec_max) {
        return false;
    }
    if (ra_min < -LIMITS_MECHANICAL_RA_RANGE || ra_max > LIMITS_MECHANICAL_RA_RANGE
        || dec_min < -LIMITS_MECHANICAL_DEC_RANGE || dec_max > LIMITS_MECHANICAL_DEC_RANGE) {
        return false;
    }
    return true;
}

/* Apply a candidate config only if it stays valid.  Returns false to reject,
 * leaving both the runtime state and NVS untouched. */
static bool apply_limits_if_valid(float ra_min, float ra_max,
                                  float dec_min, float dec_max) {
    if (!limits_are_valid(ra_min, ra_max, dec_min, dec_max)) {
        ESP_LOGW(TAG, "Rejected invalid limits: ra[%.3f, %.3f] dec[%.3f, %.3f]",
                 (double) ra_min, (double) ra_max,
                 (double) dec_min, (double) dec_max);
        return false;
    }

    portENTER_CRITICAL(&motors_state_lock);
    motors_state.limits.ra_min = ra_min;
    motors_state.limits.ra_max = ra_max;
    motors_state.limits.dec_min = dec_min;
    motors_state.limits.dec_max = dec_max;
    portEXIT_CRITICAL(&motors_state_lock);

    motors_limits_save();
    return true;
}

/* ── NVS persistence ─────────────────────────────────────── */

void motors_limits_load(void) {
    motors_state.limits.ra_min  = LIMITS_DEFAULT_RA_MIN;
    motors_state.limits.ra_max  = LIMITS_DEFAULT_RA_MAX;
    motors_state.limits.dec_min = LIMITS_DEFAULT_DEC_MIN;
    motors_state.limits.dec_max = LIMITS_DEFAULT_DEC_MAX;

    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NS, NVS_READONLY, &h);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGI(TAG, "NVS not found, using factory limits");
        return;
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "NVS open failed, using factory limits: %s", esp_err_to_name(err));
        return;
    }

    uint32_t v;
    if (nvs_get_u32(h, "ra_min", &v) == ESP_OK)  motors_state.limits.ra_min  = uint32_to_float(v);
    if (nvs_get_u32(h, "ra_max", &v) == ESP_OK)  motors_state.limits.ra_max  = uint32_to_float(v);
    if (nvs_get_u32(h, "dec_min", &v) == ESP_OK) motors_state.limits.dec_min = uint32_to_float(v);
    if (nvs_get_u32(h, "dec_max", &v) == ESP_OK) motors_state.limits.dec_max = uint32_to_float(v);

    nvs_close(h);

    /* A partially-written or corrupted NVS could yield NaN/inf, min >= max, or
     * values outside the mechanical range.  Never boot with a broken clamp —
     * fall back to the factory defaults. */
    if (!limits_are_valid(motors_state.limits.ra_min, motors_state.limits.ra_max,
                          motors_state.limits.dec_min, motors_state.limits.dec_max)) {
        ESP_LOGW(TAG, "Stored limits invalid, using factory defaults");
        motors_state.limits.ra_min  = LIMITS_DEFAULT_RA_MIN;
        motors_state.limits.ra_max  = LIMITS_DEFAULT_RA_MAX;
        motors_state.limits.dec_min = LIMITS_DEFAULT_DEC_MIN;
        motors_state.limits.dec_max = LIMITS_DEFAULT_DEC_MAX;
    }
}

void motors_limits_save(void) {
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS open for write failed: %s", esp_err_to_name(err));
        return;
    }

    nvs_set_u32(h, "ra_min",  float_to_uint32(motors_state.limits.ra_min));
    nvs_set_u32(h, "ra_max",  float_to_uint32(motors_state.limits.ra_max));
    nvs_set_u32(h, "dec_min", float_to_uint32(motors_state.limits.dec_min));
    nvs_set_u32(h, "dec_max", float_to_uint32(motors_state.limits.dec_max));

    err = nvs_commit(h);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS commit failed: %s", esp_err_to_name(err));
    }
    nvs_close(h);
}

/* ── Runtime setters ─────────────────────────────────────── */

bool motors_set_current_as_home(void) {
    if (motors_stop() != MOTOR_OK) {
        /* Never re-zero the counters if the stop did not complete — the motion
         * task may still be running and would clobber the reference. */
        return false;
    }
    portENTER_CRITICAL(&motors_state_lock);
    motors_state.ra_steps  = 0;
    motors_state.dec_steps = 0;
    motors_state.status    = MOTORS_STATUS_READY;
    /* The operator just re-established the physical origin, so the counters
     * are trustworthy again — clear any uncertainty left by an aborted move. */
    motors_state.position_uncertain = false;
    portEXIT_CRITICAL(&motors_state_lock);
    motors_limits_save();
    return true;
}

bool motors_set_current_as_ra_left_limit(void) {
    if (motors_stop() != MOTOR_OK) {
        return false;
    }
    float pos = motors_get_ra_deg();
    return apply_limits_if_valid(pos, motors_state.limits.ra_max,
                                 motors_state.limits.dec_min, motors_state.limits.dec_max);
}

bool motors_set_current_as_ra_right_limit(void) {
    if (motors_stop() != MOTOR_OK) {
        return false;
    }
    float pos = motors_get_ra_deg();
    return apply_limits_if_valid(motors_state.limits.ra_min, pos,
                                 motors_state.limits.dec_min, motors_state.limits.dec_max);
}

bool motors_set_current_as_dec_left_limit(void) {
    if (motors_stop() != MOTOR_OK) {
        return false;
    }
    float pos = motors_get_dec_deg();
    return apply_limits_if_valid(motors_state.limits.ra_min, motors_state.limits.ra_max,
                                 pos, motors_state.limits.dec_max);
}

bool motors_set_current_as_dec_right_limit(void) {
    if (motors_stop() != MOTOR_OK) {
        return false;
    }
    float pos = motors_get_dec_deg();
    return apply_limits_if_valid(motors_state.limits.ra_min, motors_state.limits.ra_max,
                                 motors_state.limits.dec_min, pos);
}

void motors_limits_reset(void) {
    motors_stop();
    portENTER_CRITICAL(&motors_state_lock);
    motors_state.limits.ra_min  = LIMITS_DEFAULT_RA_MIN;
    motors_state.limits.ra_max  = LIMITS_DEFAULT_RA_MAX;
    motors_state.limits.dec_min = LIMITS_DEFAULT_DEC_MIN;
    motors_state.limits.dec_max = LIMITS_DEFAULT_DEC_MAX;
    portEXIT_CRITICAL(&motors_state_lock);
    motors_limits_save();
}
