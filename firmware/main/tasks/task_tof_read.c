#include "tasks/task_tof_read.h"
#include "shared_state.h"
#include "vl53l5cx_drv.h"
#include "freertos/task.h"

void task_tof_read(void *arg) {
    TickType_t last = xTaskGetTickCount();
    while (1) {
        uint16_t dist[64];
        if (vl53l5cx_drv_read(dist)) {
            xSemaphoreTake(g_state.mutex, portMAX_DELAY);
            for(int i=0;i<64;i++) g_state.tof.distances[i]=dist[i];
            xSemaphoreGive(g_state.mutex);
        }
        vTaskDelayUntil(&last, pdMS_TO_TICKS(100));
    }
}
