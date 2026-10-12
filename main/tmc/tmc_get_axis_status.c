/* TMC - tmc_get_axis_status.c
 *
 * Purpose: per-axis init diagnostics — status and failure reason.
 *
 * tmc_init.c writes tmc_axis_status[] / tmc_axis_error[] as it configures
 * each driver.  The public getters and string helpers here let the REST
 * layer (and thus the UI) report which axis failed and why.
 */
#include "tmc.h"
#include "tmc_internal.h"

TmcAxisStatus tmc_axis_status[TMC_AXIS_COUNT];
TmcAxisError  tmc_axis_error[TMC_AXIS_COUNT];

TmcAxisStatus tmc2209_get_axis_status(int axis) {
    if (axis < 0 || axis >= TMC_AXIS_COUNT) {
        return TMC_AXIS_NOT_INIT;
    }
    return tmc_axis_status[axis];
}

TmcAxisError tmc2209_get_axis_error(int axis) {
    if (axis < 0 || axis >= TMC_AXIS_COUNT) {
        return TMC_AXIS_ERROR_NONE;
    }
    return tmc_axis_error[axis];
}

const char *tmc2209_axis_status_to_string(TmcAxisStatus status) {
    switch (status) {
        case TMC_AXIS_OK:
            return "ok";
        case TMC_AXIS_ERROR:
            return "error";
        case TMC_AXIS_NOT_INIT:
        default:
            return "not_init";
    }
}

const char *tmc2209_axis_error_to_string(TmcAxisError error) {
    switch (error) {
        case TMC_AXIS_ERROR_UART:
            return "uart";
        case TMC_AXIS_ERROR_GCONF_WRITE:
            return "gconf_write";
        case TMC_AXIS_ERROR_IHOLD_WRITE:
            return "ihold_write";
        case TMC_AXIS_ERROR_CHOPCONF_VERIFY:
            return "chopconf_verify";
        case TMC_AXIS_ERROR_TPWMTHRS_WRITE:
            return "tpwmthrs_write";
        case TMC_AXIS_ERROR_NONE:
        default:
            return "none";
    }
}
