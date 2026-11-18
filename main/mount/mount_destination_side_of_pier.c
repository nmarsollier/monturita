/* Mount - mount_destination_side_of_pier.c
 *
 * Purpose: predict, without moving, which pier side a goto to the given
 * equatorial position would select at this instant.  Backs the ASCOM
 * DestinationSideOfPier property; it must select exactly as a real goto would,
 * so it delegates to the same equatorial_to_axis() used by
 * mount_slew_to_coordinates.
 */
#include "mount.h"
#include "mount_internal.h"

#include <time.h>

#include "motors.h"

MountResult mount_destination_side_of_pier(float ra, float dec, int *side) {
    if (!mount_time_valid) {
        return mount_result_error("Time not synchronized — set the UTC date first");
    }

    EquatorialCoordinates eq = { .ra_hours = ra, .dec_deg = dec };
    AxisCoordinates current = { .ra_axis_deg = motors_get_ra_deg(),
                                .dec_axis_deg = motors_get_dec_deg() };
    AxisCoordinates out;
    if (!equatorial_to_axis(eq, current, time(NULL), PIER_SIDE_AUTO, &out)) {
        return mount_result_error("Target unreachable — no valid axis solution");
    }

    if (side != NULL) {
        *side = out.pier_side;
    }
    return mount_result_ok();
}
