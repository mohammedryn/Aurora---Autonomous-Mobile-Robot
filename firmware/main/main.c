#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "encoder.h"
#include "motor.h"
#include "shared_state.h"
#include "tasks/task_encoder_read.h"
#include "tasks/task_pid_control.h"
#include "tasks/task_serial_comms.h"
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

    /* Silence ESP_LOG — USB CDC stdout is now binary protocol only */
    esp_log_level_set("*", ESP_LOG_NONE);

    /* Core 0: hard real-time encoder + PID at 1kHz */
    xTaskCreatePinnedToCore(task_encoder_read, "enc",    2048, NULL,  9, NULL, 0);
    xTaskCreatePinnedToCore(task_pid_control,  "pid",    2048, NULL, 10, NULL, 0);

    /* Core 1: serial comms 100Hz (spawns its own RX task internally) */
    xTaskCreatePinnedToCore(task_serial_comms, "serial", 4096, NULL,  8, NULL, 1);
}
