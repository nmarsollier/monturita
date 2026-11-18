/* Proto — proto_action.c — ACTION: operations with a beginning and an end.
 *
 * Each operation returns accepted/rejected immediately; the outcome is later
 * reported through STATE.  A retried request with the same id replays the
 * original result instead of re-executing (see proto_action_lookup/record).
 * SYNC is intentionally rejected in this iteration: the mount has no
 * axis↔equatorial origin adjustment yet.
 */
#include "proto_internal.h"

#include <stdio.h>
#include <string.h>

#include "esp_system.h"
#include "esp_timer.h"

static bool guide_direction_parse(const char *value, GuideDirection *out) {
    if (strcmp(value, "north") == 0) { *out = GUIDE_DIRECTION_NORTH; return true; }
    if (strcmp(value, "south") == 0) { *out = GUIDE_DIRECTION_SOUTH; return true; }
    if (strcmp(value, "east") == 0)  { *out = GUIDE_DIRECTION_EAST;  return true; }
    if (strcmp(value, "west") == 0)  { *out = GUIDE_DIRECTION_WEST;  return true; }
    return false;
}

/* Delayed reboot: the handler returns and the response is flushed by the
 * dispatcher before this fires. */
static void proto_reset_restart(void *arg) {
    (void) arg;
    esp_restart();
}

static void emit_action_result(char *out, size_t cap, uint32_t id, bool ok, const char *error) {
    if (ok) {
        snprintf(out, cap, "{\"type\":\"action\",\"id\":%lu,\"ok\":true}",
                 (unsigned long) id);
    } else {
        snprintf(out, cap, "{\"type\":\"action\",\"id\":%lu,\"ok\":false,\"error\":\"%s\"}",
                 (unsigned long) id, error ? error : "error");
    }
}

void proto_handle_action(const char *req, uint32_t id, char *out, size_t cap) {
    /* Idempotent replay: a retried id returns the original result verbatim,
     * without re-executing. */
    if (id != 0) {
        proto_seen_entry_t prev;
        if (proto_action_lookup(id, &prev)) {
            emit_action_result(out, cap, id, prev.ok, prev.error);
            return;
        }
    }

    char action[16];
    if (!proto_json_get_string(req, "action", action, sizeof(action))) {
        MountResult r = proto_error("missing action");
        proto_action_record(id, r.ok, r.ok ? NULL : r.message);
        emit_action_result(out, cap, id, r.ok, r.ok ? NULL : r.message);
        return;
    }

    MountResult r = proto_error("unknown action");

    if (strcmp(action, "stop") == 0) {
        char scope[8] = "all";
        proto_json_get_string(req, "scope", scope, sizeof(scope));
        if (strcmp(scope, "manual") == 0) {
            /* Stop manual motion but restore the tracking that was active. */
            r = mount_set_move_axis_speed(0.0f, 0.0f);
        } else {
            r = mount_stop();
        }
    } else if (strcmp(action, "goto") == 0) {
        float ra = 0.0f;
        float dec = 0.0f;
        int speed = 3;
        if (proto_json_get_float(req, "ra", &ra) && proto_json_get_float(req, "dec", &dec)) {
            if (ra < 0.0f || ra >= 24.0f) {
                r = proto_error("ra out of range [0,24)");
            } else if (dec < -90.0f || dec > 90.0f) {
                r = proto_error("dec out of range [-90,90]");
            } else {
                proto_json_get_int(req, "speed", &speed);

                /* Optional destination pier side: absent -> AUTO; "east"/"west"
                 * force that side (an explicit meridian flip).  A `side` that is
                 * present but not a valid value is rejected rather than silently
                 * treated as AUTO — a flip request must not degrade into a
                 * different operation. */
                PierSideRequirement side = PIER_SIDE_AUTO;
                bool side_ok = true;
                if (proto_json_has_key(req, "side")) {
                    char side_str[8];
                    if (proto_json_get_string(req, "side", side_str, sizeof(side_str))) {
                        if (strcmp(side_str, "east") == 0) {
                            side = PIER_SIDE_EAST;
                        } else if (strcmp(side_str, "west") == 0) {
                            side = PIER_SIDE_WEST;
                        } else {
                            side_ok = false;
                        }
                    } else {
                        side_ok = false;   /* present but not a string (e.g. side: 123) */
                    }
                }
                r = side_ok ? mount_slew_to_coordinates(ra, dec, speed, side)
                            : proto_error("side must be east or west");
            }
        } else {
            r = proto_error("goto requires ra and dec");
        }
    } else if (strcmp(action, "guide") == 0) {
        char direction[8];
        uint32_t duration_ms = 0;
        GuideDirection dir;
        if (proto_json_get_string(req, "direction", direction, sizeof(direction))
            && guide_direction_parse(direction, &dir)
            && proto_json_get_u32(req, "duration_ms", &duration_ms)) {
            r = mount_pulse_guide(dir, duration_ms);
        } else {
            r = proto_error("guide requires direction and duration_ms");
        }
    } else if (strcmp(action, "home") == 0) {
        r = mount_home();
    } else if (strcmp(action, "park") == 0) {
        r = mount_park();
    } else if (strcmp(action, "unpark") == 0) {
        r = mount_unpark();
    } else if (strcmp(action, "sync") == 0) {
        r = proto_error("sync not supported");
    } else if (strcmp(action, "reset") == 0) {
        esp_timer_handle_t timer = NULL;
        esp_timer_create_args_t args = {
            .callback = proto_reset_restart,
            .name = "proto_reset",
        };
        if (esp_timer_create(&args, &timer) == ESP_OK) {
            /* Fire after the response is flushed by the dispatcher. */
            esp_timer_start_once(timer, 200 * 1000);
            r = (MountResult){ .ok = true, .message = "OK" };
        } else {
            r = proto_error("reset unavailable");
        }
    } else if (strcmp(action, "move") == 0) {
        char axis[8];
        float degrees = 0.0f;
        int speed = 4;
        if (proto_json_get_string(req, "axis", axis, sizeof(axis))
            && proto_json_get_float(req, "degrees", &degrees)) {
            proto_json_get_int(req, "speed", &speed);
            if (strcmp(axis, "ra") == 0) {
                r = mount_move_axis_ra(degrees, speed);
            } else if (strcmp(axis, "dec") == 0) {
                r = mount_move_axis_dec(degrees, speed);
            } else {
                r = proto_error("axis must be ra or dec");
            }
        } else {
            r = proto_error("move requires axis and degrees");
        }
    } else if (strcmp(action, "limits") == 0) {
        char param[24];
        if (proto_json_get_string(req, "param", param, sizeof(param))) {
            r = mount_limits_set(param);
        } else {
            r = proto_error("limits requires param");
        }
    } else if (strcmp(action, "motors") == 0) {
        bool enabled;
        if (proto_json_get_bool(req, "enabled", &enabled)) {
            MotorResultCode rc = enabled ? motors_enable() : motors_disable();
            r = (rc == MOTOR_OK)
                    ? (MountResult){ .ok = true, .message = "OK" }
                    : proto_error("motors operation rejected");
        } else {
            r = proto_error("motors requires enabled (bool)");
        }
    }

    proto_action_record(id, r.ok, r.ok ? NULL : r.message);
    emit_action_result(out, cap, id, r.ok, r.ok ? NULL : r.message);
}
