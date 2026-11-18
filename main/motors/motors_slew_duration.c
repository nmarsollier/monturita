/* Motors - motors_slew_duration.c
 *
 * Purpose: estimate how long a single-axis slew takes under the real ramp
 * profile, so the mount layer can predict a goto's arrival time and compensate
 * for the sky rotation during the move.
 *
 * The motion task does not run at a constant commanded speed: below the
 * short-slew threshold an axis crawls at MIN_SLEW, below the gentle-slew
 * threshold the target speed is capped, and every move spends a fixed angular
 * window accelerating to / decelerating from that speed.  Assuming
 * distance / speed would under- or over-estimate the duration (and so the
 * arrival hour angle) by several arcminutes on short moves.  This reproduces
 * the profile with the closed form of dt = ∫ dx / v(x).
 */
#include "motors_internal.h"

#include <math.h>

float motors_slew_duration_seconds(float distance_deg, float speed_dps) {
    const float min_slew_dps   = (float) MIN_SLEW_CDS / 100.0f;
    const float short_slew_deg = (float) SHORT_SLEW_CDS / 100.0f;
    const float gentle_slew_deg = (float) GENTLE_SLEW_CDS / 100.0f;
    const float accel_dist_deg = (float) ACCEL_DIST_CDS / 100.0f;
    const float decel_dist_deg = (float) DECEL_DIST_CDS / 100.0f;

    if (distance_deg <= 0.0f || speed_dps <= 0.0f) {
        return 0.0f;
    }

    /* Target speed after the gentle-slew cap (mirrors ramp_velocity()). */
    float capped = speed_dps;
    if (distance_deg < short_slew_deg) {
        capped = min_slew_dps;
    } else if (distance_deg < gentle_slew_deg) {
        float cap = (float) GENTLE_SPEED_CAP_CDS / 100.0f;
        if (capped > cap) capped = cap;
    }

    /* Below the short-slew threshold the axis runs at constant MIN_SLEW. */
    if (distance_deg < short_slew_deg) {
        return distance_deg / min_slew_dps;
    }

    /*
     * The velocity ramps linearly with travelled distance from MIN_SLEW up to
     * `capped` (and back down), so dt = ∫ dx / v(x) over one ramp leg is
     * ln(capped / MIN_SLEW) / (capped - MIN_SLEW) per unit distance.
     */
    float ramp_ln = logf(capped / min_slew_dps) / (capped - min_slew_dps);

    if (distance_deg <= accel_dist_deg + decel_dist_deg) {
        /* Triangular: accel to the midpoint, decel to the end. */
        return distance_deg * ramp_ln;
    }

    /* Trapezoidal: accel + cruise + decel. */
    float t_ramp = accel_dist_deg * ramp_ln;
    return 2.0f * t_ramp
           + (distance_deg - accel_dist_deg - decel_dist_deg) / capped;
}
