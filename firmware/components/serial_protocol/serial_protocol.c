#include "serial_protocol.h"

uint16_t protocol_crc16(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; j++)
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
    }
    return crc;
}

int protocol_encode(uint8_t type, const void *payload, uint8_t payload_len,
                    uint8_t *buf, size_t buf_size) {
    size_t frame_len = 6u + payload_len;
    if (buf_size < frame_len) return -1;
    buf[0] = PROTO_HEADER_0; buf[1] = PROTO_HEADER_1;
    buf[2] = type;           buf[3] = payload_len;
    if (payload_len && payload) memcpy(buf + 4, payload, payload_len);
    uint16_t crc = protocol_crc16(buf + 2, 2 + payload_len);
    buf[4 + payload_len] = (crc >> 8) & 0xFF;
    buf[5 + payload_len] =  crc       & 0xFF;
    return (int)frame_len;
}

bool protocol_decode(const uint8_t *buf, size_t len, size_t *consumed,
                     uint8_t *out_type, void *out_payload, uint8_t *out_len) {
    for (size_t i = 0; i + 5 < len; i++) {
        if (buf[i] != PROTO_HEADER_0 || buf[i+1] != PROTO_HEADER_1) continue;
        uint8_t plen = buf[i+3];
        size_t flen = 6u + plen;
        if (i + flen > len) return false;
        uint16_t exp = protocol_crc16(buf + i + 2, 2 + plen);
        uint16_t got = ((uint16_t)buf[i+4+plen] << 8) | buf[i+5+plen];
        if (exp != got) continue;
        *out_type = buf[i+2]; *out_len = plen;
        if (plen && out_payload) memcpy(out_payload, buf + i + 4, plen);
        *consumed = i + flen;
        return true;
    }
    return false;
}
