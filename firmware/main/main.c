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

void app_main(void){
    memset(&g_state,0,sizeof(g_state));
    g_state.mutex=xSemaphoreCreateMutex();

    ESP_LOGI(TAG,"Init hardware...");
    encoder_init();
    motor_init();

    if(!ism330dhcx_init()){
        ESP_LOGW(TAG,"ism330dhcx_init failed");
    }

    if(!vl53l5cx_drv_init()){
        ESP_LOGW(TAG,"vl53l5cx_drv_init failed");
    }

    ESP_LOGI(TAG,"Creating tasks...");
    xTaskCreatePinnedToCore(task_encoder_read,"enc_read",2048,NULL,9,NULL,0);
    xTaskCreatePinnedToCore(task_pid_control,"pid_ctrl",2048,NULL,10,NULL,0);
    xTaskCreatePinnedToCore(task_imu_read,"imu_read",4096,NULL,7,NULL,1);
    xTaskCreatePinnedToCore(task_tof_read,"tof_read",4096,NULL,6,NULL,1);
    xTaskCreatePinnedToCore(task_serial_comms,"serial_comms",4096,NULL,8,NULL,1);

    vTaskDelete(NULL);
}
