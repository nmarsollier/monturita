/* Mount - mount_coordinate_tools.c
 *
 * Purpose: convert between physical axis coordinates and equatorial
 * coordinates for a German equatorial mount in the southern hemisphere.
 *
 * =========================================================================
 * MECHANICAL MODEL
 * =========================================================================
 *
 * Home position (mechanical zero):
 *   - Counterweight down, scope parallel to RA axis.
 *   - Scope points at the South Celestial Pole (SCP).
 *   - ra_axis = 0, dec_axis = 0.
 *   - Reported celestial coordinates: RA = LST, DEC = -90.
 *
 * At mechanical home the DEC movement plane is NOT aligned with the
 * meridian — it is rotated ~90 from it.  If you move only DEC from
 * home, the scope sweeps laterally, not toward the zenith.  To make
 * DEC open toward the zenith, the RA axis must first rotate to
 * roughly +/-90 from home.
 *
 * The angle at which the DEC plane aligns with the meridian is a fixed
 * mechanical constant: HOME_RA_OFFSET_DEG = 90.
 *
 * =========================================================================
 * TWO MECHANICAL SOLUTIONS
 * =========================================================================
 *
 * A German equatorial can point at the same sky position from two
 * mechanically-distinct configurations (one on each side of the pier).
 * Both are computed, and the side is selected by a destination-side policy:
 *
 *   pierEast (0):  ra = HA - HOME_RA_OFFSET,   dec =  DEC + 90
 *   pierWest (1):  ra = HA + HOME_RA_OFFSET,   dec = -(DEC + 90)
 *
 *   target west of the meridian (HA >= 0)  ->  pierEast
 *   target east of the meridian (HA <  0)  ->  pierWest
 *
 * AUTO applies an asymmetric policy keyed on the current side (the sign of the
 * linear DEC axis):
 *
 *   From WEST -> flip to EAST as soon as the target crosses the meridian
 *                (HA >= 0); if EAST is unreachable, reject — never silently
 *                stay WEST.
 *   From EAST -> keep EAST while it is mechanically reachable, regardless of
 *                hour angle; return to WEST only when EAST is out of limits
 *                and WEST is reachable.  This keeps post-flip Slew-and-Center
 *                gotos from snapping back across the meridian.
 *
 * An explicit EAST/WEST request forces that side and rejects the goto when it
 * is unreachable — so a client can demand a meridian flip and tell it apart
 * from an ordinary goto.
 *
 * The mechanical jump at HA = 0 is the flip itself — inherent to a German
 * equatorial, not a discontinuity to be avoided.
 *
 * DEC axis is a linear coordinate (NOT circular).  It is NOT
 * normalised with normalize_degf() — that would incorrectly wrap
 * +180 to -180, inverting the pier side sign.
 *
 * =========================================================================
 * COORDINATE SYSTEM
 * =========================================================================
 *
 * Equatorial (RA, DEC):
 *   RA  in decimal hours   [0, 24)
 *   DEC in decimal degrees  [-90, +90]   (+90 = north pole)
 *
 * Axis (ra_axis, dec_axis):
 *   ra_axis  = rotation around polar axis, degrees [-180, +180).
 *   dec_axis = tilt from polar axis, degrees, linear (NOT wrapped).
 *              0 = scope parallel to RA axis (SCP, DEC=-90).
 *              +90 = scope at celestial equator (DEC=0).
 *
 * Hour Angle (HA):
 *   HA = LST - RA   (hours, wrapped to [-12, +12))
 *   HA > 0  ->  target is west of the meridian.
 *   HA < 0  ->  target is east of the meridian.
 *
 * =========================================================================
 * CONSTANTS
 * =========================================================================
 *
 * HOME_RA_OFFSET_DEG (90.0)
 *   RA axis angle at which the DEC plane aligns with the meridian.
 *
 * Axis limits (±100° RA, ±150° DEC) are defined in motors_init.c
 * via motors_state.limits and validated at runtime through
 * motors_current_state().limits.
 *   When |dec_axis| < 1 the scope is at the pole; RA is singular.
 *
 * =========================================================================
 * ASTRONOMICAL FORMULAS
 * =========================================================================
 *
 * Julian Date:  JD = unix_time / 86400 + 2440587.5
 *   86400 = seconds/day.  2440587.5 = JD at Unix epoch.
 *
 * GMST (USNO, ~1 arcsec):
 *   T = (JD - 2451545.0) / 36525.0   — centuries from J2000.0
 *   GMST = 280.46061837 + 360.98564736629*(JD-2451545.0)
 *        + 0.000387933*T^2 - T^3/38710000.0   (degrees)
 *   GMST_h = GMST / 15   (hours)
 *
 * LST = GMST + lon / 15
 * HA  = LST - RA   (hours, [-12, +12))
 * RA  = LST - HA   (hours, [0, 24))
 */
#include "mount.h"
#include "mount_internal.h"

#include <math.h>
#include <stdbool.h>
#include <sys/time.h>
#include <time.h>
#include <string.h>
#include <esp_log.h>

/* --------------------------------------------------------------------------
 * Internal helpers
 * -------------------------------------------------------------------------- */

static double unix_time_to_julian_date(double t) {
    return t / 86400.0 + 2440587.5;
}

static double gmst_hours_from_jd(double jd) {
    double T = (jd - 2451545.0) / 36525.0;
    double gmst_deg = 280.46061837 + 360.98564736629 * (jd - 2451545.0)
                      + 0.000387933 * T * T - (T * T * T) / 38710000.0;
    gmst_deg = fmod(gmst_deg, 360.0);
    if (gmst_deg < 0.0) gmst_deg += 360.0;
    return gmst_deg / 15.0;
}

/* Normalise a circular angle to [-180, +180).  +180 -> -180. */
static float normalize_degf(float d) {
    d = fmodf(d, 360.0f);
    if (d >= 180.0f)  d -= 360.0f;
    if (d < -180.0f)  d += 360.0f;
    return d;
}

/* Normalise hours to [0, 24).  fmodf keeps this O(1) and, unlike a `while`
 * loop, cannot spin forever on a non-finite input. */
static float normalize_hoursf(float h) {
    h = fmodf(h, 24.0f);
    if (h < 0.0f) h += 24.0f;
    return h;
}

#define HOME_RA_OFFSET_DEG  90.0f

/* Validate against motor hardware limits (single source of truth).
 * motors_current_state().limits defines the authoritative boundaries. */
static bool axis_within_limits(float ra, float dec) {
    MotorsState ms = motors_current_state();
    return (ra >= ms.limits.ra_min && ra <= ms.limits.ra_max)
        && (dec >= ms.limits.dec_min && dec <= ms.limits.dec_max);
}

/* --------------------------------------------------------------------------
 * Equatorial -> Axis  (forward conversion)
 *
 * Two mechanically-distinct candidates.  `required` selects the pier side:
 * AUTO applies the asymmetric meridian policy (see the module header);
 * EAST/WEST force that side and reject when it is unreachable.
 * -------------------------------------------------------------------------- */
bool equatorial_to_axis(EquatorialCoordinates eq, AxisCoordinates current,
                        time_t at_time, PierSideRequirement required,
                        AxisCoordinates *out) {

    if (!isfinite(eq.ra_hours) || !isfinite(eq.dec_deg)) {
        ESP_LOGE("COORD_CONV", "Non-finite RA/DEC — rejecting");
        return false;
    }

    if (eq.dec_deg < -90.0f || eq.dec_deg > 90.0f) {
        ESP_LOGE("COORD_CONV", "Invalid DEC %.4f — rejecting", eq.dec_deg);
        return false;
    }

    /* `required` is an internal, reusable function's parameter: a caller that
     * passes anything outside the AUTO/EAST/WEST range would index c[] out of
     * bounds below. */
    if (required < PIER_SIDE_AUTO || required > PIER_SIDE_WEST) {
        ESP_LOGE("COORD_CONV", "Invalid pier-side requirement %d — rejecting",
                 (int) required);
        return false;
    }

    eq.ra_hours = normalize_hoursf(eq.ra_hours);

    double lst  = gmst_hours_from_jd(unix_time_to_julian_date(at_time))
                  + (mount_internal_state.lon / 15.0);

    float ha_h = fmodf((float)(lst - eq.ra_hours), 24.0f);
    if (ha_h > 12.0f)  ha_h -= 24.0f;
    if (ha_h < -12.0f) ha_h += 24.0f;

    float ha_deg = ha_h * 15.0f;
    float dec_base = eq.dec_deg + 90.0f;

    typedef struct { float ra; float dec; int pier; } Candidate;
    Candidate c[2];

    /* pierEast: scope on east side of pier, pointing west.
     *   ra = -(offset - HA) = HA - offset   (negated for physical motor direction) */
    c[0].ra   = normalize_degf(ha_deg - HOME_RA_OFFSET_DEG);
    c[0].dec  = dec_base;
    c[0].pier = 0;

    /* pierWest: scope on west side of pier, pointing east.
     *   ra = -(-offset - HA) = HA + offset   (negated for physical motor direction) */
    c[1].ra   = normalize_degf(ha_deg + HOME_RA_OFFSET_DEG);
    c[1].dec  = -dec_base;
    c[1].pier = 1;

    /* Asymmetric AUTO policy, keyed on the current side (sign of the linear DEC
     * axis — the same convention as axis_to_equatorial() and STATE's pier_side):
     *   WEST -> flip EAST as soon as HA >= 0, no fallback (a flip must reject
     *           rather than silently stay WEST).
     *   EAST -> keep EAST while reachable; fall back to WEST only when EAST is
     *           out of limits and WEST is reachable. */
    int current_side = (current.dec_axis_deg >= 0.0f) ? PIER_SIDE_EAST
                                                      : PIER_SIDE_WEST;

    int desired;
    bool allow_fallback;
    if (required == PIER_SIDE_AUTO) {
        if (current_side == PIER_SIDE_EAST) {
            desired = PIER_SIDE_EAST;    /* stick to EAST */
            allow_fallback = true;       /* -> WEST only when EAST is out of limits */
        } else if (ha_deg >= 0.0f) {
            desired = PIER_SIDE_EAST;    /* the flip: must not degrade to staying WEST */
            allow_fallback = false;
        } else {
            desired = PIER_SIDE_WEST;    /* stay WEST; EAST allowed if WEST unreachable */
            allow_fallback = true;
        }
    } else {
        desired = required;              /* PIER_SIDE_EAST or PIER_SIDE_WEST */
        allow_fallback = false;
    }

    int best = -1;
    if (axis_within_limits(c[desired].ra, c[desired].dec)) {
        best = desired;
    } else if (allow_fallback
               && axis_within_limits(c[1 - desired].ra, c[1 - desired].dec)) {
        best = 1 - desired;
    }

    if (best < 0) {
        ESP_LOGE("COORD_CONV", "No valid axis solution for RA=%.4fh DEC=%.4f",
                 eq.ra_hours, eq.dec_deg);
        return false;
    }

    out->ra_axis_deg  = c[best].ra;
    out->dec_axis_deg = c[best].dec;
    out->pier_side    = c[best].pier;

    return true;
}

/* --------------------------------------------------------------------------
 * Axis -> Equatorial  (reverse conversion)
 *
 * The pier side is reliably encoded in the sign of dec_axis because
 * DEC is linear (not normalised):
 *
 *   dec_axis >= 0  ->  pierEast  ->  HA = ra + offset,  DEC = dec - 90
 *   dec_axis <  0  ->  pierWest  ->  HA = ra - offset,  DEC = -dec - 90
 * -------------------------------------------------------------------------- */
EquatorialCoordinates axis_to_equatorial(AxisCoordinates axis) {
    EquatorialCoordinates out;

    /* DEC is computed directly for every position.  At dec_axis == 0 (the
     * exact pole) DEC is -90 and RA is degenerate: the formula still returns
     * a valid, if arbitrary, RA — no threshold erases the nearby cone. */
    float ha_deg, dec_deg;
    if (axis.dec_axis_deg >= 0.0f) {
        /* pierEast: stored = -(offset - HA) = HA - offset  ->  HA = ra + offset */
        ha_deg  = normalize_degf(axis.ra_axis_deg + HOME_RA_OFFSET_DEG);
        dec_deg = axis.dec_axis_deg - 90.0f;
    } else {
        /* pierWest: stored = -(-offset - HA) = HA + offset  ->  HA = ra - offset */
        ha_deg  = normalize_degf(axis.ra_axis_deg - HOME_RA_OFFSET_DEG);
        dec_deg = -axis.dec_axis_deg - 90.0f;
    }

    float ha_h = ha_deg / 15.0f;

    /* Fractional-second clock — avoids the ~15"/s sawtooth in reported RA. */
    struct timeval tv;
    gettimeofday(&tv, NULL);
    double now = (double) tv.tv_sec + (double) tv.tv_usec / 1e6;

    double lst = gmst_hours_from_jd(unix_time_to_julian_date(now))
                 + (mount_internal_state.lon / 15.0);

    out.ra_hours = normalize_hoursf((float)(lst - ha_h));

    if (dec_deg < -90.0f) dec_deg = -90.0f;
    if (dec_deg >  90.0f) dec_deg =  90.0f;
    out.dec_deg = dec_deg;

    return out;
}

/* =========================================================================
 * HMS / DMS formatting
 * ========================================================================= */

RaHMS ra_hours_to_hms(float ra_hours) {
    RaHMS out;
    float total = fabsf(ra_hours);
    out.hours   = (int)total;
    float rem   = (total - (float)out.hours) * 60.0f;
    out.minutes = (int)rem;
    out.seconds = (rem - (float)out.minutes) * 60.0f;
    return out;
}

DecDMS dec_deg_to_dms(float dec_deg) {
    DecDMS out;
    out.sign    = (dec_deg >= 0.0f) ? 1 : -1;
    float total = fabsf(dec_deg);
    out.degrees = (int)total;
    float rem   = (total - (float)out.degrees) * 60.0f;
    out.minutes = (int)rem;
    out.seconds = (rem - (float)out.minutes) * 60.0f;
    return out;
}
