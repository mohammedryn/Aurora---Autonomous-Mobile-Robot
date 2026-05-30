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
    /* Kp=0.12: initial duty ~20% for 1.67 rad/s error — clears stiction reliably.
     * Ki=0.3:  moderate integration to handle load variation.
     * Kd=0:    filtered omega_meas still too noisy for D.
     * Max duty ±0.45 (45%) — safe for drivers at sustained run. */
    for (int i=0;i<4;i++) pid_init(&s_pid[i],0.12f,0.3f,0.0f,0.001f,-0.45f,0.45f);
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
