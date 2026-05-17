#include "motor.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include <math.h>

#define PWM_FREQ   20000
#define PWM_RES    LEDC_TIMER_12_BIT
#define PWM_MAX    4095u

static const int PWM_GPIO[] = {4,  6,  8,  10};  /* Verify against board */
static const int DIR_GPIO[] = {5,  7,  9,  11};

void motor_init(void) {
    ledc_timer_config_t t = {.speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0, .duty_resolution = PWM_RES,
        .freq_hz = PWM_FREQ, .clk_cfg = LEDC_AUTO_CLK};
    ledc_timer_config(&t);
    for (int i = 0; i < 4; i++) {
        ledc_channel_config_t ch = {.speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = (ledc_channel_t)i, .timer_sel = LEDC_TIMER_0,
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
    ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)idx, (uint32_t)(fabsf(duty)*PWM_MAX));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)idx);
}

void motor_stop_all(void) { for (int i=0;i<4;i++) motor_set_duty(i,0.0f); }
