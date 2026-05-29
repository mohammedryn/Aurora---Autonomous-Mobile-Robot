#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "serial_protocol.h"
#include <stdbool.h>

typedef struct {
    proto_state_t   state;         /* timestamp (enc_delta filled from enc_accum) */
    proto_cmd_vel_t cmd_vel;       /* wheel velocity setpoints from Pi */
    float           omega_meas[4]; /* instantaneous velocity rad/s (per 1ms) */
    int32_t         enc_accum[4];  /* cumulative counts since last STATE sent */
    uint8_t         error_flags;
    bool            watchdog_ok;
    SemaphoreHandle_t mutex;
} shared_state_t;

extern shared_state_t g_state;
