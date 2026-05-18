#include "motor.h"
#include "driver/mcpwm_prelude.h"
#include "driver/gpio.h"
#include <math.h>

#define PWM_FREQ_HZ  20000
#define PWM_RES_HZ   10000000UL              /* 10 MHz */
#define PWM_PERIOD   (PWM_RES_HZ / PWM_FREQ_HZ)  /* 500 ticks */

static const int PWM_GPIO[] = {5, 9, 10, 12};    /* avoid C6 SDIO/control GPIO14-19 and GPIO6 */
static const int DIR_GPIO[] = {26, 27, 46, 47};  /* avoid 28-33 camera/display zones */

static mcpwm_cmpr_handle_t s_cmpr[4];

/* motors 0-2 → group 0 (ops 0,1,2); motor 3 → group 1 (op 0) */
static const int s_group[] = {0, 0, 0, 1};

void motor_init(void) {
    mcpwm_timer_handle_t timers[2];
    mcpwm_timer_config_t tcfg = {
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = PWM_RES_HZ,
        .period_ticks  = PWM_PERIOD,
        .count_mode    = MCPWM_TIMER_COUNT_MODE_UP,
    };
    for (int g = 0; g < 2; g++) {
        tcfg.group_id = g;
        mcpwm_new_timer(&tcfg, &timers[g]);
        mcpwm_timer_enable(timers[g]);
        mcpwm_timer_start_stop(timers[g], MCPWM_TIMER_START_NO_STOP);
    }
    for (int i = 0; i < 4; i++) {
        mcpwm_oper_handle_t oper;
        mcpwm_operator_config_t opcfg = {.group_id = s_group[i]};
        mcpwm_new_operator(&opcfg, &oper);
        mcpwm_operator_connect_timer(oper, timers[s_group[i]]);

        mcpwm_comparator_config_t ccfg = {.flags.update_cmp_on_tez = true};
        mcpwm_new_comparator(oper, &ccfg, &s_cmpr[i]);
        mcpwm_comparator_set_compare_value(s_cmpr[i], 0);

        mcpwm_gen_handle_t gen;
        mcpwm_generator_config_t gcfg = {.gen_gpio_num = PWM_GPIO[i]};
        mcpwm_new_generator(oper, &gcfg, &gen);
        mcpwm_generator_set_action_on_timer_event(gen,
            MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
        mcpwm_generator_set_action_on_compare_event(gen,
            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                s_cmpr[i], MCPWM_GEN_ACTION_LOW));

        gpio_config_t d = {.pin_bit_mask = 1ULL << DIR_GPIO[i], .mode = GPIO_MODE_OUTPUT};
        gpio_config(&d);
        gpio_set_level(DIR_GPIO[i], 0);
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
