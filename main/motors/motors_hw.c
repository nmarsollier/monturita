/* Motors - motors_hw.c
 *
 * Purpose: DIR and ENABLE GPIO control for the two TMC2209 stepper drivers.
 *
 * STEP pulse generation is handled by the RMT peripheral (motors_rmt.c).
 * The drivers share a global ENABLE line (TMC2209 EN, active-low).
 *
 * Hardware: NEMA 17 stepper motors driven by TMC2209 in STEP/DIR mode with
 * UART-configured microstepping and hardware interpolation.
 */

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "motors_internal.h"

static const char *TAG = "MOTORS_HW";

/* DIR setup time before the STEP pulse — conservative value, safe for the
 * TMC2209 (which requires only ~100 ns). */
#define DIR_SETUP_TIME_US 50

/* Cached last directions to avoid redundant GPIO writes. */
static int last_dir_ra = -1;
static int last_dir_dec = -1;

esp_err_t motors_hw_init(void) {
    /*
     * DIR and ENABLE pins are managed via GPIO.  STEP pins are owned by the
     * RMT peripheral and configured by motors_rmt_init().
     */
    const uint64_t pin_mask =
            (1ULL << RA_DIR_GPIO) |
            (1ULL << DEC_DIR_GPIO) |
            (1ULL << MOTORS_ENABLE_GPIO);

    gpio_config_t config = {
        .pin_bit_mask = pin_mask,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    esp_err_t result = gpio_config(&config);

    if (result != ESP_OK) {
        return result;
    }

    last_dir_ra = 0;
    last_dir_dec = 0;

    motors_hw_enable();

    return ESP_OK;
}

void motors_hw_enable(void) {
    ESP_LOGI(TAG, "ENABLE -- GPIO %d -> %d", MOTORS_ENABLE_GPIO,
             MOTORS_ENABLE_ACTIVE_LEVEL);
    gpio_set_level(MOTORS_ENABLE_GPIO, MOTORS_ENABLE_ACTIVE_LEVEL);
}

void motors_hw_disable(void) {
    ESP_LOGW(TAG, "DISABLE -- GPIO %d -> %d", MOTORS_ENABLE_GPIO,
             MOTORS_ENABLE_INACTIVE_LEVEL);
    gpio_set_level(MOTORS_ENABLE_GPIO, MOTORS_ENABLE_INACTIVE_LEVEL);
}

void motors_hw_set_direction_ra(MotorDirection direction) {
    /* Monturita wiring inverts the driver DIR input: positive rotation maps to
     * DIR LOW. */
    int dir = direction == MOTOR_DIRECTION_POSITIVE ? 0 : 1;
    if (last_dir_ra != dir) {
        last_dir_ra = dir;
        gpio_set_level(RA_DIR_GPIO, dir);
        esp_rom_delay_us(DIR_SETUP_TIME_US);
    }
}

void motors_hw_set_direction_dec(MotorDirection direction) {
    /* Monturita wiring inverts the driver DIR input: positive rotation maps to
     * DIR LOW. */
    int dir = direction == MOTOR_DIRECTION_POSITIVE ? 0 : 1;
    if (last_dir_dec != dir) {
        last_dir_dec = dir;
        gpio_set_level(DEC_DIR_GPIO, dir);
        esp_rom_delay_us(DIR_SETUP_TIME_US);
    }
}
