#include "encoder.h"
#include "driver/pulse_cnt.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

/* GPIO assignments confirmed on Waveshare ESP32-P4-WIFI6 (physical board test).
 * A pins: 48-51 are exposed and free for PCNT input.
 * B pins: 46/47 are exposed and unclaimed; 3 and 7 (SDA label) are free since
 *         IMU/ToF moved to Pi.
 * Previous B pins {52,2,3,4} conflicted with motor RR_PWM/FR_DIR/RR_DIR. */
static const int GPIO_A[] = {48, 49, 50, 51};  /* FL FR RL RR */
static const int GPIO_B[] = {46, 47,  3,  7};  /* FL FR RL RR */

static pcnt_unit_handle_t s_units[4];
static int32_t s_last[4];
static SemaphoreHandle_t s_mutex;

static void init_unit(int idx) {
    /* Match Arduino INPUT_PULLUP — encoder outputs may need pull reference */
    gpio_pullup_en(GPIO_A[idx]);
    gpio_pullup_en(GPIO_B[idx]);

    pcnt_unit_config_t uc = {.low_limit = -32768, .high_limit = 32767};
    pcnt_new_unit(&uc, &s_units[idx]);
    pcnt_chan_config_t ca = {.edge_gpio_num = GPIO_A[idx], .level_gpio_num = GPIO_B[idx]};
    pcnt_chan_config_t cb = {.edge_gpio_num = GPIO_B[idx], .level_gpio_num = GPIO_A[idx]};
    pcnt_channel_handle_t cha, chb;
    pcnt_new_channel(s_units[idx], &ca, &cha);
    pcnt_new_channel(s_units[idx], &cb, &chb);
    pcnt_channel_set_edge_action(cha, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE);
    pcnt_channel_set_level_action(cha, PCNT_CHANNEL_LEVEL_ACTION_KEEP,   PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
    pcnt_channel_set_edge_action(chb, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE);
    pcnt_channel_set_level_action(chb, PCNT_CHANNEL_LEVEL_ACTION_KEEP,   PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
    pcnt_unit_enable(s_units[idx]);
    pcnt_unit_clear_count(s_units[idx]);
    pcnt_unit_start(s_units[idx]);
}

void encoder_init(void) {
    s_mutex = xSemaphoreCreateMutex();
    for (int i = 0; i < 4; i++) { init_unit(i); s_last[i] = 0; }
}

void encoder_get_deltas(int32_t deltas[4]) {
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    for (int i = 0; i < 4; i++) {
        int raw = 0;
        pcnt_unit_get_count(s_units[i], &raw);
        deltas[i] = (int32_t)raw - s_last[i];
        s_last[i] = (int32_t)raw;
    }
    xSemaphoreGive(s_mutex);
}
