/* Mount - mount_slew_to_coordinates.c
 *
 * Purpose: move the mount to a requested equatorial position.
 */
#include "mount.h"
#include "mount_internal.h"

#include <math.h>
#include <time.h>

#include "motors.h"

MountResult mount_slew_to_coordinates(float ra, float dec, int speed_rate,
                                     PierSideRequirement required) {
    if (mount_is_error()) {
        return mount_result_error_state();
    }

    /* GOTO needs a valid clock: without it LST/HA are wrong and the mount
     * would point at the wrong sky position. */
    if (!mount_time_valid) {
        return mount_result_error("Time not synchronized — set the UTC date first");
    }

    EquatorialCoordinates eq = {
        .ra_hours = ra,
        .dec_deg = dec
    };

    AxisCoordinates current = { .ra_axis_deg = motors_get_ra_deg(),
                                .dec_axis_deg = motors_get_dec_deg() };

    /* The sky keeps rotating during the slew: converting the destination once
     * at the request time would point at where the target *was*.  Convert at
     * the predicted arrival time instead — a first-order compensation for the
     * hour angle that advances during the move (≈ 15 arcsec/s). */
    time_t now = time(NULL);

    /* Decide the destination side ONCE, at the request instant, and lock it for
     * the whole move.  The arrival-time conversion below only re-solves the axis
     * coordinates for the (slightly later) hour angle on that same side — it
     * must never flip the side mid-slew when HA crosses zero. */
    AxisCoordinates axis_now;
    if (!equatorial_to_axis(eq, current, now, required, &axis_now)) {
        return mount_result_error("Target unreachable — no valid axis solution");
    }
    PierSideRequirement locked = (axis_now.pier_side == PIER_SIDE_EAST)
                                 ? PIER_SIDE_EAST : PIER_SIDE_WEST;

    float speed = motors_get_slewing_speed(speed_rate);
    float ra_dist  = fabsf(axis_now.ra_axis_deg  - current.ra_axis_deg);
    float dec_dist = fabsf(axis_now.dec_axis_deg - current.dec_axis_deg);
    /* Two-axis slew: the slower axis (larger per-axis duration) wins.  Use the
     * real ramp profile rather than distance/speed, which under- or over-
     * estimates short/gentle moves and skews the arrival-time compensation. */
    float duration_s = motors_slew_duration_seconds(ra_dist, speed);
    float dec_duration_s = motors_slew_duration_seconds(dec_dist, speed);
    if (dec_duration_s > duration_s) {
        duration_s = dec_duration_s;
    }
    time_t arrive_at = now + (time_t) duration_s;

    AxisCoordinates axis;
    if (!equatorial_to_axis(eq, current, arrive_at, locked, &axis)) {
        return mount_result_error("Target unreachable — no valid axis solution");
    }

    MotorResultCode rc1 = motors_slew_to_angle(axis.ra_axis_deg, axis.dec_axis_deg, speed_rate);

    if (rc1 != MOTOR_OK) {
        /* Propagate the specific code (queue full, parked, out of range) so a
         * rejected goto never reads as a generic success and clients get the
         * same "command queue full — try again" contract as PulseGuide. */
        return motors_result_code_error_result(rc1);
    }

    return mount_result_ok();
}
