#pragma once

#include "driver/gpio.h"
#include "driver/uart.h"

/*
 * Mount hardware configuration.
 *
 * Single source of truth for the build-time constants that vary between
 * mounts: pin assignments, motor resolution and mechanical reduction.
 * To adapt the firmware to a different mount, edit this file only —
 * no other module may hardcode these values.
 */

/* =========================================================================
 * Mount identity.
 * ========================================================================= */

#define MOUNT_NAME             "Monturita"
#define MOUNT_DEVICE_ID        "monturita"
#define MOUNT_FIRMWARE_VERSION "0.1.0"

/* =========================================================================
 * Pin assignments — Monturita (ESP32-S3 N16R8, TMC2209).
 * ========================================================================= */

/* Step / direction pins — direct to the TMC2209 STEP/DIR inputs (3.3 V logic,
 * no level shifting). */
#define RA_STEP_GPIO   GPIO_NUM_10
#define RA_DIR_GPIO    GPIO_NUM_9
#define DEC_STEP_GPIO  GPIO_NUM_15
#define DEC_DIR_GPIO   GPIO_NUM_7

/* Motor enable — shared global ENABLE line for both TMC2209 drivers (EN is
 * active-low on the TMC2209). */
#define MOTORS_ENABLE_GPIO          GPIO_NUM_14
#define MOTORS_ENABLE_ACTIVE_LEVEL  0
#define MOTORS_ENABLE_INACTIVE_LEVEL 1

/* TMC2209 drivers — one shared single-wire UART bus (TX through a 1 kΩ series
 * resistor, RX direct to the PDN_UART line).  The two drivers are told apart
 * by their MS1/MS2 address. */
#define TMC_UART_NUM    UART_NUM_2
#define TMC_TX_GPIO     GPIO_NUM_17
#define TMC_RX_GPIO     GPIO_NUM_18
#define TMC_RA_ADDRESS  0x03
#define TMC_DEC_ADDRESS 0x00

/* Status LED — external indicator on GPIO 4, driven via LEDC PWM. */
#define LED_GPIO        GPIO_NUM_4

/* =========================================================================
 * Motor resolution & mechanics.
 * ========================================================================= */

/* Microstep resolution — TMC2209 CHOPCONF.MRES (MRES=1).  Monturita: 128
 * microsteps.  This is the canonical (finest) resolution: the STEP counters are
 * always in these units, and every motion runs at this resolution. */
#define MOTORS_MICROSTEPS_TRACKING 128

/* Coarse microstep resolution for large ramped slews.  Monturita's 4:1
 * reduction keeps the STEP rate well within RMT limits even at 15°/s, so no
 * coarser resolution is needed: this equals the tracking resolution and the
 * dynamic-switch machinery (kept verbatim from the reference firmware) never
 * actually changes the driver resolution. */
#define MOTORS_MICROSTEPS_SLEW 128

/* Gentle-slew boundary (degrees): a slew whose larger axis travel stays below
 * this is treated gently (capped speed).  Single source for both the microstep
 * switch and the ramp speed cap. */
#define MOTORS_SLEW_GENTLE_DEG 8.0f

/* TMC2209 coil current (0–31 fields, IRUN/IHOLD), per axis.
 * I_rms = (CS + 1) / 32 × 1.77 A (VSENSE=0).
 * RA  — NEMA 17 1.5 A:  IRUN=19 → 1.11 A (~74 %), IHOLD=9 → 0.55 A.
 * DEC — smaller NEMA 17: lower current, IRUN=12, IHOLD=7. */
#define TMC_RA_IRUN   19
#define TMC_RA_IHOLD  9
#define TMC_DEC_IRUN  12
#define TMC_DEC_IHOLD 7

/* Chopper mode follows the microstep strategy: StealthChop for tracking and
 * gentle slews, SpreadCycle for coarse slews.  It is forced via
 * GCONF.en_SpreadCycle (see motors_microsteps.c / tmc_set_chop_mode.c);
 * TPWMTHRS is left at reset (0) so no automatic speed-based transition occurs
 * mid-motion. */

/* Total gear reduction: motor shaft turns : axis turns (80T/20T pulleys). */
#define TOTAL_GEAR_REDUCTION (4.0f)

/* Motion calibration factor — compensates for discrepancies between configured
 * and actual step resolution.  factor = commanded_angle / actual_angle. */
#define MOTION_CALIBRATION_FACTOR 1.0f
