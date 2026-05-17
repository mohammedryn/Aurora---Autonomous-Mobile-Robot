#include "vl53l5cx_api.h"
#include "vl53l5cx_platform.h"
#include <string.h>

uint8_t vl53l5cx_is_alive(VL53L5CX_Configuration *dev, uint8_t *alive) {
    (void)dev; *alive = 1; return 0;
}
uint8_t vl53l5cx_init(VL53L5CX_Configuration *dev) { (void)dev; return 0; }
uint8_t vl53l5cx_set_resolution(VL53L5CX_Configuration *dev, uint8_t r) { (void)dev; (void)r; return 0; }
uint8_t vl53l5cx_set_ranging_frequency_hz(VL53L5CX_Configuration *dev, uint8_t hz) { (void)dev; (void)hz; return 0; }
uint8_t vl53l5cx_start_ranging(VL53L5CX_Configuration *dev) { (void)dev; return 0; }
uint8_t vl53l5cx_check_data_ready(VL53L5CX_Configuration *dev, uint8_t *ready) { (void)dev; *ready = 0; return 0; }
uint8_t vl53l5cx_get_ranging_data(VL53L5CX_Configuration *dev, VL53L5CX_ResultsData *res) {
    (void)dev; memset(res, 0, sizeof(*res)); return 0;
}
