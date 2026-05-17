#include "tasks/task_encoder_read.h"
#include "shared_state.h"
#include "encoder.h"
#include "freertos/task.h"
#include "esp_timer.h"

#define RAD_PER_COUNT (2.0f*3.14159265f/384.0f)

void task_encoder_read(void *arg) {
    TickType_t last = xTaskGetTickCount();
    while (1) {
        int32_t d[4]; encoder_get_deltas(d);
        xSemaphoreTake(g_state.mutex, portMAX_DELAY);
        for (int i=0;i<4;i++) {
            g_state.state.enc_delta[i] = d[i];
            g_state.omega_meas[i] = d[i]*RAD_PER_COUNT*1000.0f; /* 1kHz -> rad/s */
        }
        g_state.state.timestamp_ms = (uint32_t)(esp_timer_get_time()/1000);
        xSemaphoreGive(g_state.mutex);
        vTaskDelayUntil(&last, pdMS_TO_TICKS(1));
    }
}
