#pragma once
#include <stdint.h>
#include <stdbool.h>
bool     vl53l5cx_drv_init(void);
bool     vl53l5cx_drv_read(uint16_t distances[64]); /* mm, row-major */
