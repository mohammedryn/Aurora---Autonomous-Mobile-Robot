#include "motor.h"
#include "driver/mcpwm_prelude.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include <math.h>

#define PWM_FREQ_HZ  20000
#define PWM_RES_HZ   10000000UL              /* 10 MHz */
#define PWM_PERIOD   (PWM_RES_HZ / PWM_FREQ_HZ)  /* 500 ticks */

static const int PWM_GPIO[] = {5, 32, 33, 46};   /* FL FR RL RR — pins verified on Waveshare ESP32-P4 board */
static const int DIR_GPIO[] = {26, 27, 20, 21};  /* FL FR RL RR */

static mcpwm_cmpr_handle_t s_cmpr[4];

/* motors 0-2 → group 0 (ops 0,1,2); motor 3 → group 1 (op 0) */
static const int s_group[] = {0, 0, 0, 1};

#define MOTOR_CHECK(label, expr) do {                                      \
        esp_err_t err__ = (expr);                                          \
        if (err__ != ESP_OK) {                                             \
            esp_rom_printf("[motor] %s failed: %s (%d)\n",                 \
                (label), esp_err_to_name(err__), err__);                   \
            ESP_ERROR_CHECK(err__);                                        \
        }                                                                  \
    } while (0)

void motor_init(void) {
    esp_log_level_set("gpio", ESP_LOG_WARN);
    mcpwm_timer_handle_t timers[2] = {0};
    mcpwm_timer_config_t tcfg = {
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = PWM_RES_HZ,
        .period_ticks  = PWM_PERIOD,
        .count_mode    = MCPWM_TIMER_COUNT_MODE_UP,
    };
    for (int g = 0; g < 2; g++) {
        tcfg.group_id = g;
        MOTOR_CHECK("mcpwm_new_timer", mcpwm_new_timer(&tcfg, &timers[g]));
    }
    for (int i = 0; i < 4; i++) {
        mcpwm_oper_handle_t oper = NULL;
        mcpwm_operator_config_t opcfg = {.group_id = s_group[i]};
        MOTOR_CHECK("mcpwm_new_operator", mcpwm_new_operator(&opcfg, &oper));
        MOTOR_CHECK("mcpwm_operator_connect_timer", mcpwm_operator_connect_timer(oper, timers[s_group[i]]));

        mcpwm_comparator_config_t ccfg = {.flags.update_cmp_on_tez = true};
        MOTOR_CHECK("mcpwm_new_comparator", mcpwm_new_comparator(oper, &ccfg, &s_cmpr[i]));
        MOTOR_CHECK("mcpwm_comparator_set_compare_value", mcpwm_comparator_set_compare_value(s_cmpr[i], 0));

        mcpwm_gen_handle_t gen = NULL;
        mcpwm_generator_config_t gcfg = {.gen_gpio_num = PWM_GPIO[i]};
        MOTOR_CHECK("mcpwm_new_generator", mcpwm_new_generator(oper, &gcfg, &gen));
        MOTOR_CHECK("mcpwm_generator_set_action_on_timer_event",
            mcpwm_generator_set_action_on_timer_event(gen,
            MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
        MOTOR_CHECK("mcpwm_generator_set_action_on_compare_event",
            mcpwm_generator_set_action_on_compare_event(gen,
            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                s_cmpr[i], MCPWM_GEN_ACTION_LOW)));

        gpio_config_t d = {.pin_bit_mask = 1ULL << DIR_GPIO[i], .mode = GPIO_MODE_OUTPUT};
        MOTOR_CHECK("gpio_config(dir)", gpio_config(&d));
        MOTOR_CHECK("gpio_set_level(dir)", gpio_set_level(DIR_GPIO[i], 0));
    }
    for (int g = 0; g < 2; g++) {
        MOTOR_CHECK("mcpwm_timer_enable", mcpwm_timer_enable(timers[g]));
        MOTOR_CHECK("mcpwm_timer_start_stop", mcpwm_timer_start_stop(timers[g], MCPWM_TIMER_START_NO_STOP));
    }
}

void motor_set_duty(int idx, float duty) {
    if (duty >  1.0f) duty =  1.0f;
    if (duty < -1.0f) duty = -1.0f;
    gpio_set_level(DIR_GPIO[idx], duty >= 0.0f ? 1 : 0);
    mcpwm_comparator_set_compare_value(s_cmpr[idx],
        (uint32_t)(fabsf(duty) * PWM_PERIOD));
}

void motor_stop_all(void) { for (int i = 0; i < 4; i++) motor_set_duty(i, 0.0f); }
