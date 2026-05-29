#pragma once
#include <string>
#include <vector>
#include <cstdint>

namespace amr_hardware {

/* Matches firmware proto_state_t exactly — 20 bytes payload, 26 byte frame */
struct StatePacket {
    uint32_t timestamp_ms;
    int32_t  enc_delta[4];  /* FL FR RL RR — sign-corrected by firmware */
};

/* 16 bytes payload — firmware proto_cmd_vel_t */
struct CmdVelPacket {
    float omega[4];         /* FL FR RL RR rad/s setpoints */
};

class SerialDriver {
public:
    bool open(const std::string & port, int baud_rate);
    void close();
    bool is_open() const { return fd_ >= 0; }

    bool send_cmd_vel(const CmdVelPacket & pkt);
    bool send_heartbeat();

    /* Drains port and parses all complete frames. Returns true if at least
       one valid STATE packet was received; *st holds the most recent one. */
    bool spin_once(StatePacket * st);

private:
    int fd_{-1};
    std::vector<uint8_t> rx_buf_;

    bool write_packet(uint8_t type, const void * pl, uint8_t len);
    static uint16_t crc16(const uint8_t * d, size_t n);

    static constexpr uint8_t H0 = 0xAA;
    static constexpr uint8_t H1 = 0x55;
    static constexpr uint8_t TYPE_CMD_VEL   = 0x01;
    static constexpr uint8_t TYPE_STATE     = 0x02;
    static constexpr uint8_t TYPE_HEARTBEAT = 0x04;
};

}  // namespace amr_hardware
