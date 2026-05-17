#include "tasks/task_imu_read.h"
#include "shared_state.h"
#include "ism330dhcx.h"
#include "freertos/task.h"

void task_imu_read(void *arg) {
    TickType_t last = xTaskGetTickCount();
    while (1) {
        ism330dhcx_data_t d; ism330dhcx_read(&d);
        xSemaphoreTake(g_state.mutex, portMAX_DELAY);
        for(int i=0;i<3;i++){g_state.state.accel[i]=d.accel[i];g_state.state.gyro[i]=d.gyro[i];}
        xSemaphoreGive(g_state.mutex);
        vTaskDelayUntil(&last, pdMS_TO_TICKS(10));
    }
}
