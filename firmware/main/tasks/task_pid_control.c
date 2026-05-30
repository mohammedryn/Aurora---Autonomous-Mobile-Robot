#include "tasks/task_pid_control.h"
#include "shared_state.h"
#include "pid.h"
#include "motor.h"
#include "freertos/task.h"

static pid_t s_pid[4];

void task_pid_control(void *arg) {
    /* Conservative gains for initial tuning — avoids duty saturation and overheating.
     * Kp=0.3 limits overshoot; output clamped to ±0.6 (60% max duty). */
    for (int i=0;i<4;i++) pid_init(&s_pid[i],0.3f,0.8f,0.005f,0.001f,-0.6f,0.6f);
    TickType_t last = xTaskGetTickCount();
    while (1) {
        xSemaphoreTake(g_state.mutex, portMAX_DELAY);
        float sp[4], meas[4]; bool ok=g_state.watchdog_ok;
        for(int i=0;i<4;i++){sp[i]=g_state.cmd_vel.omega[i];meas[i]=g_state.omega_meas[i];}
        xSemaphoreGive(g_state.mutex);
        if (!ok) { motor_stop_all(); vTaskDelayUntil(&last,pdMS_TO_TICKS(1)); continue; }
        for (int i=0;i<4;i++) motor_set_duty(i,pid_update(&s_pid[i],sp[i],meas[i]));
        vTaskDelayUntil(&last, pdMS_TO_TICKS(1));
    }
}
