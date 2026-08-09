/* Motors - motors_stop.c
 *
 * Purpose: stop all motor movement immediately.
 *
 * Clears the command queue, resets state, and delegates hardware stop
 * and task notification to motors_motion_stop().
 */
#include "motors.h"
#include "motors_internal.h"

#include "esp_log.h"

void motors_stop(void) {
    ESP_LOGW("MOTORS_STOP", "stopping — clearing queue and aborting motion");
    motors_queue_clear();
    motors_state.status = MOTORS_STATUS_READY;
    motors_state.tracking = TRACKING_NONE;
    motors_state.guiding = false;
    motors_motion_stop();
}
