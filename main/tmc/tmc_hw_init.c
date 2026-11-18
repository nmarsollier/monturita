/* TMC — tmc_hw_init.c
 *
 * Purpose: install the shared single-wire UART bus and configure the drivers.
 * Must be called before any step/direction motion is started.
 */

#include "tmc.h"
#include "tmc_internal.h"

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "esp_log.h"

static const char *TAG = "TMC_HW_INIT";

esp_err_t tmc2209_hw_init(void)
{
    if (tmc_uart_lock() != ESP_OK) {
        return ESP_FAIL;
    }
    tmc_reset_diagnostics();
    tmc_uart_ready = false;

    uart_config_t config = {
        .baud_rate  = TMC_BAUD_RATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t result = uart_driver_install(TMC_UART_NUM, 512, 512, 0, NULL, 0);
    if (result != ESP_OK) {
        tmc_status = TMC_ERROR;
        tmc_error = TMC_ERROR_UART;
        ESP_LOGE(TAG, "uart_driver_install failed: %s", esp_err_to_name(result));
        tmc_uart_unlock();
        return result;
    }

    result = uart_param_config(TMC_UART_NUM, &config);
    if (result != ESP_OK) {
        tmc_status = TMC_ERROR;
        tmc_error = TMC_ERROR_UART;
        ESP_LOGE(TAG, "uart_param_config failed: %s", esp_err_to_name(result));
        tmc_uart_unlock();
        return result;
    }

    /* Single-wire: TX through 1 kΩ, RX direct to the PDN_UART line. */
    result = uart_set_pin(TMC_UART_NUM, TMC_TX_GPIO, TMC_RX_GPIO,
                          UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (result != ESP_OK) {
        tmc_status = TMC_ERROR;
        tmc_error = TMC_ERROR_UART;
        ESP_LOGE(TAG, "uart_set_pin failed: %s", esp_err_to_name(result));
        tmc_uart_unlock();
        return result;
    }

    gpio_set_pull_mode(TMC_RX_GPIO, GPIO_PULLUP_ONLY);
    tmc_uart_ready = true;

    result = tmc_configure_drivers();
    tmc_uart_unlock();
    return result;
}
