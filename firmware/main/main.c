#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/usb_serial_jtag.h"
#include "motor.h"
#include "esp_log.h"

/*
 * Motor-test firmware — drivetrain bring-up only.
 * Encoders, IMU, ToF, PID are not compiled.
 *
 * GPIO pins (locked to confirmed-working Waveshare ESP32-P4-WIFI6 wiring):
 *   FL  PWM=5   DIR=26
 *   FR  PWM=33  DIR=2
 *   RL  PWM=32  DIR=27
 *   RR  PWM=52  DIR=4
 *
 * Same char commands as the Arduino teleop sketch so teleop_simple.py works
 * unchanged (default port /dev/amr_mcu, 115200 baud):
 *   f = forward       b = backward
 *   l = rotate CCW    r = rotate CW
 *   q = fwd-left diag e = fwd-right diag
 *   z = bwd-left diag x = bwd-right diag
 *   s / space = stop
 */

static const char *TAG = "motor_test";

#define S  0.35f   /* 35% duty — conservative for bench testing */
#define Z  0.0f

static void task_motor_test(void *arg)
{
    uint8_t buf[4];
    while (1) {
        int n = usb_serial_jtag_read_bytes(buf, 1, pdMS_TO_TICKS(50));
        if (n <= 0) continue;

        switch ((char)buf[0]) {
            /* FL=0  FR=1  RL=2  RR=3 */
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

    usb_serial_jtag_driver_config_t usb_cfg = {
        .rx_buffer_size = 512,
        .tx_buffer_size = 512,
    };
    usb_serial_jtag_driver_install(&usb_cfg);

    ESP_LOGI(TAG, "Ready. f=fwd b=bck l=CCW r=CW q/e/z/x=diag s=stop");

    xTaskCreatePinnedToCore(task_motor_test, "motor_test", 4096, NULL, 5, NULL, 1);
}
