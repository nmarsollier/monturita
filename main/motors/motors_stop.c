/* Motors - motors_stop.c
 *
 * Purpose: stop all motor movement immediately.
 *
 * Clears the command queue, resets state, and delegates hardware stop
 * and task notification to motors_motion_stop().
 */
#include "motors.h"
#include "motors_internal.h"

MotorResultCode motors_stop(void) {
    /* ERROR is unrecoverable — never transition out of it. */
    if (motors_status_is_error(motors_state.status)) {
        return MOTOR_ERR_HARDWARE_ERROR;
    }

    motors_queue_clear();
    /* STOP must not unpark: only motors_unpark() exits PARKED. */
    portENTER_CRITICAL(&motors_state_lock);
    if (motors_state.status != MOTORS_STATUS_PARKED) {
        motors_state.status = MOTORS_STATUS_READY;
    }
    motors_state.tracking = TRACKING_NONE;
    motors_state.guiding = false;
    portEXIT_CRITICAL(&motors_state_lock);
    motors_motion_stop();

    /*
     * STOP is a barrier: it must not report READY (nor return to the caller)
     * until the motion task has actually left its loop and finished tearing
     * down RMT.  Without this wait a caller could observe READY and enqueue a
     * new move while the motion task is still emitting steps or resetting RMT,
     * and could read a position the task has not yet reconciled.  If the task
     * fails to become idle in time, report the stop as failed rather than
     * claiming the mount stopped.
     */
    if (!motors_motion_wait_idle()) {
        return MOTOR_ERR_NOT_READY;
    }

    return MOTOR_OK;
}
