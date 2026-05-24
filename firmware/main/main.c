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
#include <string.h>

shared_state_t g_state;
static const char *TAG="main";
/*
 * Temporary board-health diagnostic:
 * skip hardware bring-up step by step to find which peripheral init is hanging.
 * encoder_init also hangs (same symptom as motor_init was) — likely GPIO 48-52
 * or 2-4 conflicting with C6 SDIO / strapping. Skip it too and see if the
 * board boots all the way through IMU/ToF/gyro/tasks.
 */
#define BOOT_DIAG_SKIP_ENCODER_INIT 1
#define BOOT_DIAG_SKIP_MOTOR_INIT   1

void app_main(void){
    memset(&g_state,0,sizeof(g_state));
    g_state.mutex=xSemaphoreCreateMutex();
    ESP_LOGI(TAG,"Init hardware...");

#if BOOT_DIAG_SKIP_ENCODER_INIT
    ESP_LOGW(TAG,"BOOT_DIAG_SKIP_ENCODER_INIT=1: skipping encoder_init()");
#else
    ESP_LOGI(TAG,"Init encoders...");
    encoder_init();
    ESP_LOGI(TAG,"Encoders ready.");
#endif

#if BOOT_DIAG_SKIP_MOTOR_INIT
    ESP_LOGW(TAG,"BOOT_DIAG_SKIP_MOTOR_INIT=1: skipping motor_init()");
#else
    ESP_LOGI(TAG,"Init motors...");
    motor_init();
    ESP_LOGI(TAG,"Motors ready.");
#endif

    ESP_LOGI(TAG,"Init IMU...");
    if(!ism330dhcx_init()) ESP_LOGE(TAG,"IMU init FAILED");
    else ESP_LOGI(TAG,"IMU ready.");

    ESP_LOGI(TAG,"Init ToF...");
    if(!vl53l5cx_drv_init()) ESP_LOGE(TAG,"ToF init FAILED");
    else ESP_LOGI(TAG,"ToF ready.");

    ESP_LOGI(TAG,"Calibrating gyro - hold still 5s...");
    ism330dhcx_calibrate_gyro();
    ESP_LOGI(TAG,"Calibration done. Starting tasks.");
    xTaskCreatePinnedToCore(task_encoder_read,"enc",  4096,NULL, 9,NULL,0);
    xTaskCreatePinnedToCore(task_pid_control, "pid",  4096,NULL,10,NULL,0);
    xTaskCreatePinnedToCore(task_imu_read,    "imu",  4096,NULL, 7,NULL,1);
    xTaskCreatePinnedToCore(task_tof_read,    "tof",  8192,NULL, 6,NULL,1);
    xTaskCreatePinnedToCore(task_serial_comms,"ser",  8192,NULL, 8,NULL,1);
}
