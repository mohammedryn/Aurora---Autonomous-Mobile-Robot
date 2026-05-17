#include "vl53l5cx_drv.h"
#include "st_uld/vl53l5cx_api.h"
#include "st_uld/vl53l5cx_platform.h"
#include <string.h>

static VL53L5CX_Configuration s_dev;

bool vl53l5cx_drv_init(void) {
    vl53l5cx_platform_init();
    s_dev.platform.address = VL53L5CX_DEFAULT_I2C_ADDRESS;
    uint8_t alive = 0;
    if (vl53l5cx_is_alive(&s_dev, &alive) || !alive) return false;
    if (vl53l5cx_init(&s_dev))                       return false;
    if (vl53l5cx_set_resolution(&s_dev, VL53L5CX_RESOLUTION_8X8)) return false;
    if (vl53l5cx_set_ranging_frequency_hz(&s_dev, 10)) return false;
    if (vl53l5cx_start_ranging(&s_dev))              return false;
    return true;
}

bool vl53l5cx_drv_read(uint16_t distances[64]) {
    uint8_t ready = 0;
    vl53l5cx_check_data_ready(&s_dev, &ready);
    if (!ready) return false;
    VL53L5CX_ResultsData res;
    if (vl53l5cx_get_ranging_data(&s_dev, &res)) return false;
    for (int i = 0; i < 64; i++)
        distances[i] = (uint16_t)(res.distance_mm[i] < 0 ? 0 : res.distance_mm[i]);
    return true;
}
