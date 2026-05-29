#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "motor.h"
#include "esp_log.h"
#include <stdio.h>

/*
 * Motor-test firmware — reads single chars from stdin (USB CDC /dev/ttyACM0).
 * Same commands as Arduino teleop sketch — teleop_simple.py works unchanged.
 *
 * GPIO pins (confirmed-working Waveshare ESP32-P4-WIFI6):
 *   FL  PWM=5   DIR=26
 *   FR  PWM=33  DIR=2
 *   RL  PWM=32  DIR=27
 *   RR  PWM=52  DIR=4
 */

static const char *TAG = "motor_test";

#define S  0.35f
#define Z  0.0f

static void task_motor_test(void *arg)
{
    while (1) {
        int c = fgetc(stdin);
        if (c == EOF) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        /* FL=0  FR=1  RL=2  RR=3 */
        switch (c) {
            case 'f': motor_set_duty(0, S); motor_set_duty(1, S); motor_set_duty(2, S); motor_set_duty(3, S); break;
            case 'b': motor_set_duty(0,-S); motor_set_duty(1,-S); motor_set_duty(2,-S); motor_set_duty(3,-S); break;
            case 'l': motor_set_duty(0,-S); motor_set_duty(1, S); motor_set_duty(2,-S); motor_set_duty(3, S); break;
            case 'r': motor_set_duty(0, S); motor_set_duty(1,-S); motor_set_duty(2, S); motor_set_duty(3,-S); break;
            case 'q': motor_set_duty(0, Z); motor_set_duty(1, S); motor_set_duty(2, S); motor_set_duty(3, Z); break;
            case 'e': motor_set_duty(0, S); motor_set_duty(1, Z); motor_set_duty(2, Z); motor_set_duty(3, S); break;
            case 'z': motor_set_duty(0,-S); motor_set_duty(1, Z); motor_set_duty(2, Z); motor_set_duty(3,-S); break;
            case 'x': motor_set_duty(0, Z); motor_set_duty(1,-S); motor_set_duty(2,-S); motor_set_duty(3, Z); break;
            default:  motor_stop_all(); break;
        }
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Motor test firmware");
    ESP_LOGI(TAG, "FL PWM=5 DIR=26 | FR PWM=33 DIR=2 | RL PWM=32 DIR=27 | RR PWM=52 DIR=4");

    motor_init();
    ESP_LOGI(TAG, "motor_init done — all motors stopped");
    ESP_LOGI(TAG, "Ready. f=fwd b=bck l=CCW r=CW q/e/z/x=diag s=stop");

    xTaskCreatePinnedToCore(task_motor_test, "motor_test", 4096, NULL, 5, NULL, 1);
}
