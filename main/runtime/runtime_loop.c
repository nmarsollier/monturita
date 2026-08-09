/* Runtime - runtime_loop.c
 *
 * Purpose: run the periodic mount cycle for inputs polling.
 */
#include "runtime.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "led.h"

/*
 * Business use case: orchestrate the mount's periodic operational cycle.
 *
 * Objective: keep input polling running at a stable cadence.
 * Motor motion runs in its own task owned by the motors_motion subsystem.
 */

#define MAIN_LOOP_PERIOD_MS    20
#define INPUTS_POLL_PERIOD_MS  50

static void main_loop_task(void *arg) {
    (void) arg;
    TickType_t main_period_ticks = pdMS_TO_TICKS(MAIN_LOOP_PERIOD_MS);
    TickType_t inputs_period_ticks = pdMS_TO_TICKS(INPUTS_POLL_PERIOD_MS);

    TickType_t elapsed_inputs_ticks = inputs_period_ticks;

    while (true) {
        if (elapsed_inputs_ticks >= inputs_period_ticks) {
            led_update();
            elapsed_inputs_ticks = 0;
        }

        elapsed_inputs_ticks += main_period_ticks;

        vTaskDelay(main_period_ticks);
    }
}

/*
 * Business use case: start the main runtime in the background.
 *
 * Objective: delegate continuous operation to a dedicated FreeRTOS task
 * on CPU 0.
 */

#define MAIN_TASK_STACK_SIZE 4096
#define MAIN_TASK_PRIORITY   5

void setup_runtime_start(void) {
    xTaskCreatePinnedToCore(
        main_loop_task,
        "main_loop",
        MAIN_TASK_STACK_SIZE,
        NULL,
        MAIN_TASK_PRIORITY,
        NULL,
        0);  /* CPU 0 */
}
