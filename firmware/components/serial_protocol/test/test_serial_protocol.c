#include "unity.h"
#include "serial_protocol.h"

TEST_CASE("encode-decode roundtrip CMD_VEL", "[serial_protocol]") {
    proto_cmd_vel_t cmd = {{1.0f, 2.0f, -1.0f, -2.0f}};
    uint8_t buf[32];
    int len = protocol_encode(PROTO_TYPE_CMD_VEL, &cmd, sizeof(cmd), buf, sizeof(buf));
    TEST_ASSERT_EQUAL_INT(22, len);
    uint8_t type, payload[32], plen; size_t consumed;
    TEST_ASSERT_TRUE(protocol_decode(buf, len, &consumed, &type, payload, &plen));
    TEST_ASSERT_EQUAL_UINT8(PROTO_TYPE_CMD_VEL, type);
    TEST_ASSERT_EQUAL_UINT8(sizeof(cmd), plen);
    TEST_ASSERT_EQUAL_MEMORY(&cmd, payload, sizeof(cmd));
    TEST_ASSERT_EQUAL_size_t(22, consumed);
}

TEST_CASE("corrupt CRC is rejected", "[serial_protocol]") {
    proto_cmd_vel_t cmd = {{0.5f, 0.5f, 0.5f, 0.5f}};
    uint8_t buf[32];
    int len = protocol_encode(PROTO_TYPE_CMD_VEL, &cmd, sizeof(cmd), buf, sizeof(buf));
    buf[len - 1] ^= 0xFF;
    uint8_t type, payload[32], plen; size_t consumed;
    TEST_ASSERT_FALSE(protocol_decode(buf, len, &consumed, &type, payload, &plen));
}

TEST_CASE("incomplete packet returns false", "[serial_protocol]") {
    proto_cmd_vel_t cmd = {{1.0f, 0, 0, 0}};
    uint8_t buf[32];
    int len = protocol_encode(PROTO_TYPE_CMD_VEL, &cmd, sizeof(cmd), buf, sizeof(buf));
    uint8_t type, payload[32], plen; size_t consumed;
    TEST_ASSERT_FALSE(protocol_decode(buf, len / 2, &consumed, &type, payload, &plen));
}

TEST_CASE("STATE packet encodes to 50 bytes", "[serial_protocol]") {
    proto_state_t s = {0};
    uint8_t buf[64];
    int len = protocol_encode(PROTO_TYPE_STATE, &s, sizeof(s), buf, sizeof(buf));
    TEST_ASSERT_EQUAL_INT(50, len);
}
