/*
 * tmc.h — TMC2209 driver public API.
 *
 * This module is the SINGLE source of truth for microstep configuration.
 * All other layers MUST reference MICROSTEPS rather than
 * defining their own constants.
 */

#pragma once

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

/* --------------------------------------------------------------------------
 * Microstep configuration — single source of truth for the entire system.
 *
 * All other layers (motors, motion) MUST reference this value rather than
 * defining their own constants.
 * -------------------------------------------------------------------------- */
#define MICROSTEPS  128

/* --------------------------------------------------------------------------
 * Public API
 * -------------------------------------------------------------------------- */

/*
 * Initialize UART, configure GCONF/IHOLD_IRUN/CHOPCONF on both RA and DEC
 * axes, and verify that the hardware latched the requested microsteps.
 *
 * Must be called before any step/direction motion is started.
 * Called internally from motors_hw_init().
 */
esp_err_t tmc2209_hw_init(void);

/*
 * Query whether the TMC2209 UART and both axes were initialised
 * successfully.  Used for error-state LED signalling.
 */
bool tmc2209_is_initialized(void);

/* --------------------------------------------------------------------------
 * Per-axis init diagnostics — exposed so the UI can report which axis
 * failed and why (UART vs register config).
 * -------------------------------------------------------------------------- */

#define TMC_AXIS_RA    0
#define TMC_AXIS_DEC   1
#define TMC_AXIS_COUNT 2

/* Per-axis init status. */
typedef enum {
    TMC_AXIS_NOT_INIT = 0,   /* init not run / not reached */
    TMC_AXIS_OK,             /* configured and verified */
    TMC_AXIS_ERROR,          /* failed — see tmc2209_get_axis_error() */
} TmcAxisStatus;

/* Per-axis failure reason (valid when status == TMC_AXIS_ERROR). */
typedef enum {
    TMC_AXIS_ERROR_NONE = 0,
    TMC_AXIS_ERROR_UART,           /* uart_driver_install / param_config / set_pin */
    TMC_AXIS_ERROR_GCONF_WRITE,    /* GCONF write + readback */
    TMC_AXIS_ERROR_IHOLD_WRITE,    /* IHOLD_IRUN write (IFCNT verify) */
    TMC_AXIS_ERROR_CHOPCONF_VERIFY,/* CHOPCONF write + readback / MRES mismatch */
    TMC_AXIS_ERROR_TPWMTHRS_WRITE  /* TPWMTHRS write (IFCNT verify) */
} TmcAxisError;

TmcAxisStatus tmc2209_get_axis_status(int axis);
TmcAxisError  tmc2209_get_axis_error(int axis);

const char *tmc2209_axis_status_to_string(TmcAxisStatus status);
const char *tmc2209_axis_error_to_string(TmcAxisError error);
