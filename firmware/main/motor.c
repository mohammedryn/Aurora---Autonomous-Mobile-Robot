#include "motor.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include <math.h>

#define PWM_FREQ   20000
#define PWM_RES    LEDC_TIMER_10_BIT
#define PWM_MAX    1023u

static const int PWM_GPIO[] = {20, 21, 22, 23};  /* Waveshare right-side, confirmed safe */
static const int DIR_GPIO[] = {26, 27, 46, 47};  /* avoid 28-33: camera/display zones */
/* LEDC_CHANNEL_2 triggers a deferred GAMMA_RAM bus fault on ESP32-P4; skip it */
static const ledc_channel_t PWM_CH[] = {
    LEDC_CHANNEL_0, LEDC_CHANNEL_1, LEDC_CHANNEL_3, LEDC_CHANNEL_4
};

void motor_init(void) {
    ledc_timer_config_t t = {.speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0, .duty_resolution = PWM_RES,
        .freq_hz = PWM_FREQ, .clk_cfg = LEDC_AUTO_CLK};
    ledc_timer_config(&t);
    for (int i = 0; i < 4; i++) {
        ledc_channel_config_t ch = {.speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = PWM_CH[i], .timer_sel = LEDC_TIMER_0,
            .gpio_num = PWM_GPIO[i], .duty = 0, .hpoint = 0};
        ledc_channel_config(&ch);
        gpio_config_t d = {.pin_bit_mask = 1ULL << DIR_GPIO[i], .mode = GPIO_MODE_OUTPUT};
        gpio_config(&d);
        gpio_set_level(DIR_GPIO[i], 0);
    }
}

void motor_set_duty(int idx, float duty) {
    if (duty >  1.0f) duty =  1.0f;
    if (duty < -1.0f) duty = -1.0f;
    gpio_set_level(DIR_GPIO[idx], duty >= 0.0f ? 1 : 0);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, PWM_CH[idx], (uint32_t)(fabsf(duty)*PWM_MAX));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, PWM_CH[idx]);
}

void motor_stop_all(void) { for (int i=0;i<4;i++) motor_set_duty(i,0.0f); }
