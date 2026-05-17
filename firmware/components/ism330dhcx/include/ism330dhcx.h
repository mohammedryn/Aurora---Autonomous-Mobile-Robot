#pragma once
#include <stdbool.h>

typedef struct { float accel[3]; float gyro[3]; } ism330dhcx_data_t;

bool ism330dhcx_init(void);
bool ism330dhcx_read(ism330dhcx_data_t *out);
void ism330dhcx_calibrate_gyro(void); /* call once with robot stationary */
