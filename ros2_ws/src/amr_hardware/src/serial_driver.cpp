#include "amr_hardware/serial_driver.hpp"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>

namespace amr_hardware {

uint16_t SerialDriver::crc16(const uint8_t * d, size_t n)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < n; i++) {
        crc ^= (uint16_t)d[i] << 8;
        for (int j = 0; j < 8; j++)
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
    }
    return crc;
}

bool SerialDriver::open(const std::string & port, int /*baud_rate*/)
{
    fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) return false;
    struct termios tty{};
    tcgetattr(fd_, &tty);
    cfsetispeed(&tty, B921600);
    cfsetospeed(&tty, B921600);
    cfmakeraw(&tty);
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 0;
    tcsetattr(fd_, TCSANOW, &tty);
    return true;
}

void SerialDriver::close()
{
    if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
}

bool SerialDriver::write_packet(uint8_t type, const void * pl, uint8_t len)
{
    uint8_t frame[64];
    size_t flen = 6u + len;
    frame[0] = H0; frame[1] = H1;
    frame[2] = type; frame[3] = len;
    if (len && pl) std::memcpy(frame + 4, pl, len);
    uint16_t crc = crc16(frame + 2, 2 + len);
    frame[4 + len] = (crc >> 8) & 0xFF;
    frame[5 + len] =  crc       & 0xFF;
    return ::write(fd_, frame, flen) == (ssize_t)flen;
}

bool SerialDriver::send_cmd_vel(const CmdVelPacket & p)
{
    return write_packet(TYPE_CMD_VEL, &p, sizeof(p));
}

bool SerialDriver::send_heartbeat()
{
    return write_packet(TYPE_HEARTBEAT, nullptr, 0);
}

bool SerialDriver::spin_once(StatePacket * st)
{
    uint8_t tmp[512];
    ssize_t n = ::read(fd_, tmp, sizeof(tmp));
    if (n > 0) rx_buf_.insert(rx_buf_.end(), tmp, tmp + n);

    bool got = false;

    while (rx_buf_.size() >= 6) {
        /* Find header */
        auto it = rx_buf_.begin();
        while (it + 1 < rx_buf_.end() && !(*it == H0 && *(it + 1) == H1)) ++it;
        if (it + 1 >= rx_buf_.end()) { rx_buf_.clear(); break; }
        rx_buf_.erase(rx_buf_.begin(), it);
        if (rx_buf_.size() < 6) break;

        uint8_t plen = rx_buf_[3];
        size_t flen = 6u + plen;
        if (rx_buf_.size() < flen) break;

        uint16_t exp = crc16(rx_buf_.data() + 2, 2 + plen);
        uint16_t got2 = ((uint16_t)rx_buf_[4 + plen] << 8) | rx_buf_[5 + plen];
        if (exp != got2) {
            rx_buf_.erase(rx_buf_.begin());  /* bad CRC — skip header byte */
            continue;
        }

        if (rx_buf_[2] == TYPE_STATE && plen == sizeof(StatePacket)) {
            std::memcpy(st, rx_buf_.data() + 4, sizeof(StatePacket));
            got = true;
        }
        /* Consume frame (unknown types are silently dropped) */
        rx_buf_.erase(rx_buf_.begin(), rx_buf_.begin() + flen);
    }
    return got;
}

}  // namespace amr_hardware
