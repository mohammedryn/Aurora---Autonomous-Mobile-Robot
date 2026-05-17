#include "unity.h"

extern void test_encode_decode_roundtrip(void);
extern void test_corrupt_crc_rejected(void);
extern void test_incomplete_packet(void);
extern void test_state_packet_size(void);

void app_main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_encode_decode_roundtrip);
    RUN_TEST(test_corrupt_crc_rejected);
    RUN_TEST(test_incomplete_packet);
    RUN_TEST(test_state_packet_size);
    UNITY_END();
}
