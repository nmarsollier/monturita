/* TMC — tmc_uart.c
 *
 * Purpose: single-wire UART transport to the two TMC2209 drivers — the axis
 * table, register read/write primitives, and the mutex that serialises access
 * across cores.  Both drivers share one UART bus and are addressed by their
 * MS1/MS2 pins.
 */

#include "tmc.h"
#include "tmc_internal.h"
#include "config.h"

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

/* ── Per-axis table ────────────────────────────────────────── */

const TmcAxis tmc_axes[TMC_AXIS_COUNT] = {
    { .name = "RA",  .address = TMC_RA_ADDRESS,  .irun = TMC_RA_IRUN,  .ihold = TMC_RA_IHOLD },
    { .name = "DEC", .address = TMC_DEC_ADDRESS, .irun = TMC_DEC_IRUN, .ihold = TMC_DEC_IHOLD },
};

/* Shared UART readiness — set by tmc2209_hw_init() once the bus is installed. */
bool tmc_uart_ready = false;

/* ── UART access serialisation ─────────────────────────────── */

/*
 * The single-wire UART bus is shared across cores: the power probe
 * (tmc2209_probe, core 0) and the runtime microstep switch (tmc2209_set_microsteps,
 * motion task, core 1) both touch the same hardware.  A mutex serialises every
 * public UART operation so two tasks never interleave frames on the bus.
 */
static SemaphoreHandle_t tmc_uart_mutex = NULL;

esp_err_t tmc_uart_lock(void) {
    if (tmc_uart_mutex == NULL) {
        tmc_uart_mutex = xSemaphoreCreateMutex();
        if (tmc_uart_mutex == NULL) {
            return ESP_ERR_NO_MEM;
        }
    }
    return (xSemaphoreTake(tmc_uart_mutex, pdMS_TO_TICKS(2000)) == pdTRUE)
               ? ESP_OK : ESP_FAIL;
}

void tmc_uart_unlock(void) {
    if (tmc_uart_mutex != NULL) {
        xSemaphoreGive(tmc_uart_mutex);
    }
}

/* ── UART helpers ──────────────────────────────────────────── */

static uint8_t tmc_crc(const uint8_t *data, size_t len)
{
    uint8_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        uint8_t byte = data[i];
        for (int bit = 0; bit < 8; bit++) {
            if (((crc >> 7) ^ (byte & 0x01)) != 0) {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc <<= 1;
            }
            byte >>= 1;
        }
    }
    return crc;
}

/*
 * Write a TMC2209 register over the shared single-wire UART bus.
 *
 * The written frame loops back into the RX FIFO (RX is on the same wire);
 * the echo is drained here so it never leaks into a subsequent read.
 */
esp_err_t tmc_write_register(const TmcAxis *axis, uint8_t reg, uint32_t value)
{
    uint8_t request[8] = {
        0x05,
        axis->address,
        (uint8_t)(reg | 0x80),
        (uint8_t)(value >> 24),
        (uint8_t)(value >> 16),
        (uint8_t)(value >> 8),
        (uint8_t)(value),
        0x00,
    };
    request[7] = tmc_crc(request, 7);

    uart_flush_input(TMC_UART_NUM);

    int written = uart_write_bytes(TMC_UART_NUM, request, sizeof(request));
    if (written != (int)sizeof(request)) {
        return ESP_FAIL;
    }

    uart_wait_tx_done(TMC_UART_NUM, pdMS_TO_TICKS(10));

    /* Absorb the 8-byte loopback echo from the single-wire bus. */
    uint8_t echo_buffer[8];
    uart_read_bytes(TMC_UART_NUM, echo_buffer, sizeof(request), pdMS_TO_TICKS(10));

    return ESP_OK;
}

/*
 * Read a TMC2209 register over the shared single-wire UART bus.
 *
 * TX connects through a 1 kΩ series resistor to the TMC2209 PDN_UART
 * pin, and RX connects directly to that pin. After sending the read
 * request and draining the loopback echo, TX is floated (switched to
 * input) so the TMC2209 output can drive the line cleanly for its 8-byte
 * response — the internal pull-up holds the idle HIGH.
 *
 * Retries up to 6 attempts on timeout or invalid response — single-wire
 * UART reads are inherently prone to occasional corruption.
 */
esp_err_t tmc_read_register(const TmcAxis *axis, uint8_t reg, uint32_t *value)
{
    if (value == NULL) return ESP_ERR_INVALID_ARG;

    uint8_t request[4] = {
        0x05,
        axis->address,
        (uint8_t)(reg & 0x7F),   /* bit 7 = 0 → read */
        0x00,
    };
    request[3] = tmc_crc(request, 3);

    esp_err_t last_err = ESP_FAIL;

    for (int attempt = 0; attempt < 6; attempt++) {
        if (attempt > 0)
            vTaskDelay(pdMS_TO_TICKS(10));

        uart_flush_input(TMC_UART_NUM);

        int written = uart_write_bytes(TMC_UART_NUM, request, sizeof(request));
        if (written != (int)sizeof(request)) return ESP_FAIL;

        uart_wait_tx_done(TMC_UART_NUM, pdMS_TO_TICKS(10));

        /* Drain the 4-byte loopback echo before floating TX. */
        uint8_t echo_buf[4];
        uart_read_bytes(TMC_UART_NUM, echo_buf, sizeof(request), pdMS_TO_TICKS(10));

        /*
         * Float TX so the TMC2209 output has a clean, high-impedance
         * line to drive for the remainder of its 8-byte response frame
         * (the internal pull-up holds the idle HIGH level).
         */
        gpio_set_direction(TMC_TX_GPIO, GPIO_MODE_INPUT);
        gpio_set_pull_mode(TMC_TX_GPIO, GPIO_PULLUP_ONLY);

        /* Collect the remaining bytes of the combined (echo + reply) stream.
         * The reply is a fixed 8-byte datagram, so stop as soon as it is
         * complete instead of waiting out the whole deadline.  The buffer stays
         * 32 bytes to tolerate a little framing noise; a short read that misses
         * the frame is handled by the retry loop above. */
        uint8_t response[32];
        int len = 0;
        TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(50);

        while (xTaskGetTickCount() < deadline && len < (int)sizeof(response)) {
            int n = uart_read_bytes(TMC_UART_NUM, response + len,
                                    sizeof(response) - len, pdMS_TO_TICKS(5));
            if (n > 0) len += n;
            if (len >= 8) break;   /* full reply frame received */
        }

        /* Restore TX — drive HIGH before re-enabling the output so the
         * bus doesn't glitch LOW while the IO MUX is reconnected. */
        gpio_set_level(TMC_TX_GPIO, 1);
        gpio_set_direction(TMC_TX_GPIO, GPIO_MODE_OUTPUT);
        uart_set_pin(TMC_UART_NUM, TMC_TX_GPIO, TMC_RX_GPIO,
                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

        if (len < 8) {
            last_err = ESP_ERR_TIMEOUT;
            continue;
        }

        /* Find and validate a frame in the response buffer. */
        for (int i = 0; i <= len - 8; i++) {
            uint8_t *frame = &response[i];
            if (frame[0] != 0x05) continue;
            if ((frame[2] & 0x7F) != (reg & 0x7F)) continue;
            if (tmc_crc(frame, 7) != frame[7]) continue;

            *value = ((uint32_t)frame[3] << 24) |
                     ((uint32_t)frame[4] << 16) |
                     ((uint32_t)frame[5] << 8)  |
                     ((uint32_t)frame[6]);
            return ESP_OK;
        }

        last_err = ESP_ERR_INVALID_RESPONSE;
    }

    return last_err;
}
