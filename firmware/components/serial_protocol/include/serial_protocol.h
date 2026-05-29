#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#define PROTO_HEADER_0       0xAA
#define PROTO_HEADER_1       0x55
#define PROTO_TYPE_CMD_VEL   0x01
#define PROTO_TYPE_STATE     0x02
#define PROTO_TYPE_HEARTBEAT 0x04
#define PROTO_TYPE_PARAM_SET 0x05
#define PROTO_TYPE_DIAG      0x06

/* STATE: 20 bytes payload → 26 byte frame (header+type+len+payload+crc) */
typedef struct __attribute__((packed)) {
    uint32_t timestamp_ms;
    int32_t  enc_delta[4];  /* FL FR RL RR — sign corrected in firmware */
} proto_state_t;

typedef struct __attribute__((packed)) {
    float omega[4];         /* FL FR RL RR rad/s setpoints */
} proto_cmd_vel_t;          /* 16 bytes */

typedef struct __attribute__((packed)) {
    uint8_t param_id;
    float   value;
} proto_param_set_t;        /* 5 bytes */

typedef struct __attribute__((packed)) {
    uint16_t batt_mv;
    uint8_t  error_flags;
} proto_diag_t;             /* 3 bytes */

uint16_t protocol_crc16(const uint8_t *data, size_t len);
int      protocol_encode(uint8_t type, const void *payload, uint8_t payload_len,
                         uint8_t *buf, size_t buf_size);
bool     protocol_decode(const uint8_t *buf, size_t len, size_t *consumed,
                         uint8_t *out_type, void *out_payload, uint8_t *out_len);
