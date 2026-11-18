/* Motors - motors_enable.c
 *
 * Purpose: user-facing motor enable/disable — assert/deassert the shared
 * TMC2209 ENABLE line and gate motion.
 *
 * Monturita keeps the ENABLE line (the reference firmware's drivers are always
 * enabled), so the serial protocol exposes a switch to idle the drivers.  When
 * disabled every motion command is rejected at the command queue; STOP and PARK
 * remain available.  This is distinct from the power gate (motors_power.c),
 * which senses the 12 V rail through the TMC2209 UART probe.
 */
#include "motors.h"
#include "motors_internal.h"

static bool s_motors_enabled = true;

bool motors_is_enabled(void) {
    return s_motors_enabled;
}

MotorResultCode motors_enable(void) {
    motors_hw_enable();
    s_motors_enabled = true;
    return MOTOR_OK;
}

MotorResultCode motors_disable(void) {
    motors_queue_clear();
    motors_motion_stop();
    motors_hw_disable();
    s_motors_enabled = false;
    return MOTOR_OK;
}
