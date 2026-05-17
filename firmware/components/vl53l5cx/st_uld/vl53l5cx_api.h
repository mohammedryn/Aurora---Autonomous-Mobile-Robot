#pragma once
#include <stdint.h>

#define VL53L5CX_DEFAULT_I2C_ADDRESS  0x52
#define VL53L5CX_RESOLUTION_8X8       64

typedef struct {
    struct { uint8_t address; } platform;
} VL53L5CX_Configuration;

typedef struct {
    int32_t distance_mm[64];
} VL53L5CX_ResultsData;

/* These return 0 on success, non-zero on error */
uint8_t vl53l5cx_is_alive(VL53L5CX_Configuration *dev, uint8_t *alive);
uint8_t vl53l5cx_init(VL53L5CX_Configuration *dev);
uint8_t vl53l5cx_set_resolution(VL53L5CX_Configuration *dev, uint8_t resolution);
uint8_t vl53l5cx_set_ranging_frequency_hz(VL53L5CX_Configuration *dev, uint8_t freq_hz);
uint8_t vl53l5cx_start_ranging(VL53L5CX_Configuration *dev);
uint8_t vl53l5cx_check_data_ready(VL53L5CX_Configuration *dev, uint8_t *ready);
uint8_t vl53l5cx_get_ranging_data(VL53L5CX_Configuration *dev, VL53L5CX_ResultsData *res);
