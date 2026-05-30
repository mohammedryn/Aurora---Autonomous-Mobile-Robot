#include "tasks/task_encoder_read.h"
#include "shared_state.h"
#include "encoder.h"
#include "freertos/task.h"
#include "esp_timer.h"

/* 7PPR × 4 edges × 19.2:1 gear = 537.6 counts per output shaft revolution */
#define RAD_PER_COUNT (2.0f * 3.14159265f / 537.6f)

/* Physical test confirmed: on forward command FL and RL count negative,
 * FR and RR count positive. Negate FL/RL so all wheels report positive
 * velocity when the robot moves forward. */
static const int SIGN[] = {-1, 1, -1, 1};  /* FL FR RL RR */

/* IIR low-pass filter for velocity measurement.
 * At low speed the encoder counts 0-1 pulse per 1ms PID cycle, creating
 * binary noise (0 or 11.7 rad/s) that makes the PID jerk.
 * alpha=0.08 → time constant ~11ms; smooths noise without killing response. */
#define VEL_ALPHA 0.08f
static float s_omega_filtered[4] = {0};

void task_encoder_read(void *arg)
{
    TickType_t last = xTaskGetTickCount();
    while (1) {
        int32_t d[4];
        encoder_get_deltas(d);

        xSemaphoreTake(g_state.mutex, portMAX_DELAY);
        for (int i = 0; i < 4; i++) {
            int32_t signed_delta = d[i] * SIGN[i];
            g_state.enc_accum[i] += signed_delta;
            float raw = (float)signed_delta * RAD_PER_COUNT * 1000.0f;
            s_omega_filtered[i]  = VEL_ALPHA * raw + (1.0f - VEL_ALPHA) * s_omega_filtered[i];
            g_state.omega_meas[i] = s_omega_filtered[i];
        }
        g_state.state.timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000);
        xSemaphoreGive(g_state.mutex);

        vTaskDelayUntil(&last, pdMS_TO_TICKS(1));
    }
}
