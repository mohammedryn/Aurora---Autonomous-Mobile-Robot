#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "serial_protocol.h"
#include <stdbool.h>

typedef struct {
    proto_state_t   state;
    proto_tof_t     tof;
    proto_cmd_vel_t cmd_vel;
    float           omega_meas[4];
    uint8_t         error_flags;
    bool            watchdog_ok;
    SemaphoreHandle_t mutex;
} shared_state_t;

extern shared_state_t g_state;
