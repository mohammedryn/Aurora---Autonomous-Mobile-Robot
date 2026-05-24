#!/usr/bin/env python3
"""
AMR Pi-side teleop — arrow-key control over serial to ESP32 MCU.

Serial framing: [0xAA][0x55][TYPE:1][LEN:1][PAYLOAD:LEN][CRC16_HI][CRC16_LO]
CRC16 is computed over TYPE + LEN + PAYLOAD.
"""

import argparse
import curses
import struct
import time
import serial

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
SPEED = 0.30      # m/s linear speed
ROT   = 1.2       # rad/s rotation speed
WHEEL_RADIUS = 0.030   # metres
WHEELBASE    = 0.30    # lx + ly  (wheelbase sum used in mecanum IK)

LOOP_HZ        = 20          # main loop frequency
LOOP_PERIOD    = 1.0 / LOOP_HZ
HEARTBEAT_HZ   = 1           # must be ≥ 0.5  (ESP32 watchdog = 2 s)

# Packet type IDs
PKT_CMD_VEL   = 0x01
PKT_STATE     = 0x02
PKT_HEARTBEAT = 0x04

# ---------------------------------------------------------------------------
# CRC-16/ARC  — matches C firmware
# ---------------------------------------------------------------------------
def crc16(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


# ---------------------------------------------------------------------------
# Packet encode
# ---------------------------------------------------------------------------
def encode_packet(pkt_type: int, payload: bytes) -> bytes:
    pkt_len  = len(payload)
    crc_data = bytes([pkt_type, pkt_len]) + payload
    crc      = crc16(crc_data)
    return bytes([0xAA, 0x55, pkt_type, pkt_len]) + payload + bytes([crc >> 8, crc & 0xFF])


# ---------------------------------------------------------------------------
# Mecanum inverse kinematics
# ---------------------------------------------------------------------------
def mecanum_ik(vx: float, vy: float, wz: float):
    """Return (FL, FR, RL, RR) wheel angular velocities in rad/s."""
    r = WHEEL_RADIUS
    L = WHEELBASE
    fl = (1.0 / r) * (vx - vy - L * wz)
    fr = (1.0 / r) * (vx + vy + L * wz)
    rl = (1.0 / r) * (vx + vy - L * wz)
    rr = (1.0 / r) * (vx - vy + L * wz)
    return fl, fr, rl, rr


# ---------------------------------------------------------------------------
# Build CMD_VEL packet
# ---------------------------------------------------------------------------
def cmd_vel_packet(vx: float, vy: float, wz: float) -> bytes:
    fl, fr, rl, rr = mecanum_ik(vx, vy, wz)
    payload = struct.pack('<ffff', fl, fr, rl, rr)
    return encode_packet(PKT_CMD_VEL, payload)


HEARTBEAT_PACKET = encode_packet(PKT_HEARTBEAT, b'')
STOP_PACKET      = cmd_vel_packet(0.0, 0.0, 0.0)


# ---------------------------------------------------------------------------
# RX packet parser (streaming, stateless buffer)
# ---------------------------------------------------------------------------
def parse_packets(buf: bytearray):
    """
    Scan buf for complete, valid packets.
    Returns (list_of_parsed_packets, remaining_buf).
    Each parsed packet is a dict: {type, payload}.
    """
    packets = []
    i = 0
    while i < len(buf) - 1:
        if buf[i] == 0xAA and buf[i + 1] == 0x55:
            # Need at least header (4 bytes) + 2 CRC bytes = 6 bytes minimum
            if i + 4 > len(buf):
                break
            pkt_type = buf[i + 2]
            pkt_len  = buf[i + 3]
            frame_end = i + 4 + pkt_len + 2
            if frame_end > len(buf):
                break   # incomplete packet, wait for more data
            payload  = bytes(buf[i + 4 : i + 4 + pkt_len])
            crc_hi   = buf[frame_end - 2]
            crc_lo   = buf[frame_end - 1]
            received_crc  = (crc_hi << 8) | crc_lo
            crc_data      = bytes([pkt_type, pkt_len]) + payload
            computed_crc  = crc16(crc_data)
            if received_crc == computed_crc:
                packets.append({'type': pkt_type, 'payload': payload})
                i = frame_end
            else:
                # Bad CRC — skip this sync byte, keep scanning
                i += 1
        else:
            i += 1
    return packets, bytearray(buf[i:])


# ---------------------------------------------------------------------------
# Parse STATE packet (type 0x02, 44 bytes)
# uint32 ts_ms + 4×int32 enc_delta + 3×float accel + 3×float gyro
# ---------------------------------------------------------------------------
def parse_state(payload: bytes) -> dict:
    if len(payload) < 44:
        return {}
    ts, e0, e1, e2, e3, ax, ay, az, gx, gy, gz = struct.unpack_from('<Iiiiiffffff', payload, 0)
    return {
        'ts': ts,
        'enc': [e0, e1, e2, e3],
        'accel': [ax, ay, az],
        'gyro':  [gx, gy, gz],
    }


# ---------------------------------------------------------------------------
# Key → (vx, vy, wz, label)
# ---------------------------------------------------------------------------
KEY_MAP = {
    curses.KEY_UP:    ( SPEED,  0.0,  0.0,  'FORWARD'),
    curses.KEY_DOWN:  (-SPEED,  0.0,  0.0,  'BACKWARD'),
    curses.KEY_LEFT:  ( 0.0,  SPEED,  0.0,  'STRAFE LEFT'),
    curses.KEY_RIGHT: ( 0.0, -SPEED,  0.0,  'STRAFE RIGHT'),
    ord('q'):         ( 0.0,   0.0,  ROT,   'ROTATE CCW'),
    ord('Q'):         ( 0.0,   0.0,  ROT,   'ROTATE CCW'),
    ord('e'):         ( 0.0,   0.0, -ROT,   'ROTATE CW'),
    ord('E'):         ( 0.0,   0.0, -ROT,   'ROTATE CW'),
    ord(' '):         ( 0.0,   0.0,  0.0,   'STOP'),
}


# ---------------------------------------------------------------------------
# Main curses loop
# ---------------------------------------------------------------------------
def main(stdscr, port: str, baud: int):
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.keypad(True)

    ser = serial.Serial(port, baud, timeout=0)

    rx_buf         = bytearray()
    last_state     = {}
    last_hb_time   = 0.0
    last_cmd_time  = 0.0

    vx, vy, wz = 0.0, 0.0, 0.0
    label       = 'STOP'

    try:
        while True:
            now = time.monotonic()

            # --- Key input -----------------------------------------------
            key = stdscr.getch()
            if key in (27, 3):          # Esc or Ctrl-C
                break
            elif key in KEY_MAP:
                vx, vy, wz, label = KEY_MAP[key]
            elif key != -1:
                # Any other key → stop
                vx, vy, wz, label = 0.0, 0.0, 0.0, 'STOP'

            # --- CMD_VEL at 20 Hz ----------------------------------------
            if now - last_cmd_time >= LOOP_PERIOD:
                ser.write(cmd_vel_packet(vx, vy, wz))
                last_cmd_time = now

            # --- HEARTBEAT at 1 Hz ---------------------------------------
            if now - last_hb_time >= 1.0 / HEARTBEAT_HZ:
                ser.write(HEARTBEAT_PACKET)
                last_hb_time = now

            # --- RX parse ------------------------------------------------
            incoming = ser.read(256)
            if incoming:
                rx_buf.extend(incoming)
                pkts, rx_buf = parse_packets(rx_buf)
                for pkt in pkts:
                    if pkt['type'] == PKT_STATE:
                        last_state = parse_state(pkt['payload'])

            # Keep buffer bounded
            if len(rx_buf) > 4096:
                rx_buf = rx_buf[-2048:]

            # --- Draw UI -------------------------------------------------
            stdscr.erase()
            fl, fr, rl, rr = mecanum_ik(vx, vy, wz)

            stdscr.addstr(0, 0, f"AMR Teleop  |  port: {port}  baud: {baud}")
            stdscr.addstr(1, 0, "-" * 52)
            stdscr.addstr(2, 0, "Controls:")
            stdscr.addstr(3, 0, "  Arrows = drive   Q/E = rotate   Space = stop   Esc = quit")
            stdscr.addstr(4, 0, "")
            stdscr.addstr(5, 0, f"Active command:  {label:<14}  (vx={vx:+.2f}  vy={vy:+.2f}  wz={wz:+.2f})")
            stdscr.addstr(6, 0, f"Wheel w (rad/s): FL={fl:+.2f}  FR={fr:+.2f}  RL={rl:+.2f}  RR={rr:+.2f}")
            stdscr.addstr(7, 0, "")
            if last_state:
                enc  = last_state['enc']
                ts   = last_state['ts']
                stdscr.addstr(8, 0,
                    f"Last STATE  ts={ts}ms  enc=[{enc[0]:+d} {enc[1]:+d} {enc[2]:+d} {enc[3]:+d}]")
            else:
                stdscr.addstr(8, 0, "Last STATE  (waiting...)")
            stdscr.addstr(9, 0, "-" * 52)
            stdscr.refresh()

            # --- Pace the loop -------------------------------------------
            elapsed = time.monotonic() - now
            sleep_for = LOOP_PERIOD - elapsed
            if sleep_for > 0:
                time.sleep(sleep_for)

    finally:
        # Send stop before closing
        try:
            ser.write(STOP_PACKET)
        except Exception:
            pass
        ser.close()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def run():
    parser = argparse.ArgumentParser(description='AMR teleop — arrow-key serial control')
    parser.add_argument('--port', default='/dev/amr_mcu', help='Serial port (default: /dev/amr_mcu)')
    parser.add_argument('--baud', type=int, default=921600, help='Baud rate (default: 921600)')
    args = parser.parse_args()

    curses.wrapper(main, args.port, args.baud)


if __name__ == '__main__':
    run()
