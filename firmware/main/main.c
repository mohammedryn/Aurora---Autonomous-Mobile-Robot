#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "encoder.h"
#include "motor.h"
#include "shared_state.h"
#include "tasks/task_encoder_read.h"
#include "tasks/task_pid_control.h"
#include "tasks/task_serial_comms.h"
#include "driver/uart.h"
#include "esp_log.h"
#include <string.h>

shared_state_t g_state;
static const char *TAG = "main";

void app_main(void)
{
    memset(&g_state, 0, sizeof(g_state));
    g_state.mutex = xSemaphoreCreateMutex();

    ESP_LOGI(TAG, "AMR firmware starting");
    ESP_LOGI(TAG, "Motor pins: FL PWM=5 DIR=26 | FR PWM=33 DIR=2 | RL PWM=32 DIR=27 | RR PWM=52 DIR=4");
    ESP_LOGI(TAG, "Encoder A: FL=48 FR=49 RL=50 RR=51 | B: FL=46 FR=47 RL=3 RR=7");

    encoder_init();
    ESP_LOGI(TAG, "encoder_init done");

    motor_init();
    ESP_LOGI(TAG, "motor_init done");

    /* Install UART driver for UART0 with a 4KB RX ring buffer.
     * This bypasses the VFS shared-lock deadlock that occurs when fwrite(stdout)
     * and fread(stdin) are called concurrently on the same console UART device.
     * task_serial_comms uses uart_read_bytes/uart_write_bytes directly. */
    uart_driver_install(CONFIG_ESP_CONSOLE_UART_NUM, 4096, 0, 0, NULL, 0);

    /* Silence ESP_LOG — UART0 is now binary protocol only */
    esp_log_level_set("*", ESP_LOG_NONE);

    /* Core 0: hard real-time encoder + PID at 1kHz */
    xTaskCreatePinnedToCore(task_encoder_read, "enc",    2048, NULL,  9, NULL, 0);
    xTaskCreatePinnedToCore(task_pid_control,  "pid",    2048, NULL, 10, NULL, 0);

    /* Core 1: serial comms 100Hz */
    xTaskCreatePinnedToCore(task_serial_comms, "serial", 4096, NULL,  8, NULL, 1);
}
