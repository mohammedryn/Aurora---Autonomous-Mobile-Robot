#include "tasks/task_pid_control.h"
#include "shared_state.h"
#include "pid.h"
#include "motor.h"
#include "freertos/task.h"
#include <math.h>

static pid_t s_pid[4];

/* Deadband: don't drive motors for tiny setpoints — prevents drift and heat */
#define SP_DEADBAND 0.1f   /* rad/s — below this, command zero */

void task_pid_control(void *arg) {
    /* Kp=0.05: at 1.67 rad/s error, initial duty = 8.5% (no overshoot).
     * Ki=0.15: slow integration, minimal windup.
     * Kd=0:    encoder noise makes D term unhelpful at this resolution.
     * Max duty ±0.5 (50%) — plenty for 0.1 m/s, safe for drivers. */
    for (int i=0;i<4;i++) pid_init(&s_pid[i],0.05f,0.15f,0.0f,0.001f,-0.5f,0.5f);
    TickType_t last = xTaskGetTickCount();
    while (1) {
        xSemaphoreTake(g_state.mutex, portMAX_DELAY);
        float sp[4], meas[4]; bool ok=g_state.watchdog_ok;
        for(int i=0;i<4;i++){sp[i]=g_state.cmd_vel.omega[i];meas[i]=g_state.omega_meas[i];}
        xSemaphoreGive(g_state.mutex);

        if (!ok) {
            for (int i=0;i<4;i++) pid_reset(&s_pid[i]);
            motor_stop_all();
            vTaskDelayUntil(&last, pdMS_TO_TICKS(1));
            continue;
        }

        for (int i=0;i<4;i++) {
            if (fabsf(sp[i]) < SP_DEADBAND) {
                /* Setpoint near zero — reset integral and stop motor */
                pid_reset(&s_pid[i]);
                motor_set_duty(i, 0.0f);
            } else {
                motor_set_duty(i, pid_update(&s_pid[i], sp[i], meas[i]));
            }
        }
        vTaskDelayUntil(&last, pdMS_TO_TICKS(1));
    }
}
