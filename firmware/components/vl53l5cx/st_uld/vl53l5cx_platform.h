#pragma once
#include <stdint.h>

typedef struct {
    uint8_t address;
} VL53L5CX_Platform;

void    vl53l5cx_platform_init(void);
uint8_t VL53L5CX_RdMulti(VL53L5CX_Platform *p, uint16_t reg, uint8_t *buf, uint32_t len);
uint8_t VL53L5CX_WrMulti(VL53L5CX_Platform *p, uint16_t reg, uint8_t *buf, uint32_t len);
uint8_t VL53L5CX_RdByte(VL53L5CX_Platform *p, uint16_t addr, uint8_t *val);
uint8_t VL53L5CX_WrByte(VL53L5CX_Platform *p, uint16_t addr, uint8_t val);
void    VL53L5CX_WaitMs(VL53L5CX_Platform *p, uint32_t ms);
