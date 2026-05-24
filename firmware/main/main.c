#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "encoder.h"
#include "motor.h"
#include "ism330dhcx.h"
#include "vl53l5cx_drv.h"
#include "shared_state.h"
#include "tasks/task_encoder_read.h"
#include "tasks/task_pid_control.h"
#include "tasks/task_imu_read.h"
#include "tasks/task_tof_read.h"
#include "tasks/task_serial_comms.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_system.h"
#include <string.h>

shared_state_t g_state;
static const char *TAG="main";

void app_main(void){
    esp_rom_printf("[diag] A: app_main entered, reset_reason=%d\n", (int)esp_reset_reason());

    memset(&g_state,0,sizeof(g_state));
    esp_rom_printf("[diag] B: memset done, sizeof(g_state)=%d\n", (int)sizeof(g_state));

    g_state.mutex=xSemaphoreCreateMutex();
    esp_rom_printf("[diag] C: mutex=%p\n", g_state.mutex);

    ESP_LOGI(TAG,"Init hardware...");
    esp_rom_printf("[diag] D: after ESP_LOGI\n");

    ESP_LOGW(TAG,"BOOT_DIAG_SKIP_ENCODER_INIT=1: skipping encoder_init()");
    esp_rom_printf("[diag] E: after ESP_LOGW (encoder skip)\n");

    ESP_LOGW(TAG,"BOOT_DIAG_SKIP_MOTOR_INIT=1: skipping motor_init()");
    esp_rom_printf("[diag] F: after ESP_LOGW (motor skip)\n");

    esp_rom_printf("[diag] G: idle loop\n");
    while(1) vTaskDelay(pdMS_TO_TICKS(1000));
}
