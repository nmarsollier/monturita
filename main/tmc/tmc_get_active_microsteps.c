/* TMC - tmc_get_active_microsteps.c
 *
 * Purpose: store the verified microstep cache and the initialisation flag
 * used by tmc2209_is_initialized() for LED error signalling.
 *
 * The single static variable s_active_microsteps is owned by this file and
 * updated by tmc_set_microsteps() (in tmc_init.c) after each successful
 * hardware write + read-back verification.
 *
 * tmc2209_is_initialized() reuses the same cache: a non-zero value means
 * both axes were configured and verified successfully.
 */

#include "tmc.h"
#include "tmc_internal.h"

/* --------------------------------------------------------------------------
 * Verified microstep cache — 0 until tmc2209_hw_init() succeeds.
 * -------------------------------------------------------------------------- */

static uint16_t s_active_microsteps = 0;

void tmc2209_set_active_microsteps(uint16_t microsteps) {
    s_active_microsteps = microsteps;
}

bool tmc2209_is_initialized(void) {
    return s_active_microsteps != 0;
}
