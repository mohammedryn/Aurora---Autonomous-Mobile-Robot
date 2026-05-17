#!/usr/bin/env python3
"""Verify AMR firmware STATE packets streaming at ~100Hz over USB serial."""
import serial, struct, time, argparse

HEADER = bytes([0xAA, 0x55])
TYPE_STATE = 0x02
TYPE_TOF   = 0x03

def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = (crc << 1) ^ 0x1021 if crc & 0x8000 else crc << 1
    return crc & 0xFFFF

def decode_frame(buf):
    """Find and decode one frame from buf. Returns (type, payload, bytes_consumed) or None."""
    i = 0
    while i < len(buf) - 1:
        if buf[i] == 0xAA and buf[i+1] == 0x55:
            if i + 4 > len(buf):
                return None
            ftype = buf[i+2]
            flen  = buf[i+3]
            end   = i + 4 + flen + 2
            if end > len(buf):
                return None
            payload = buf[i+4:i+4+flen]
            hi, lo  = buf[i+4+flen], buf[i+4+flen+1]
            crc_rx  = (hi << 8) | lo
            crc_calc = crc16(bytes([ftype, flen]) + payload)
            if crc_rx != crc_calc:
                i += 1
                continue
            return ftype, payload, end
        i += 1
    return None

def parse_state(payload):
    if len(payload) < 44:
        return None
    ts, = struct.unpack_from('<I', payload, 0)
    enc = struct.unpack_from('<4i', payload, 4)
    acc = struct.unpack_from('<3f', payload, 20)
    gyr = struct.unpack_from('<3f', payload, 32)
    return ts, enc, acc, gyr

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('port', nargs='?', default='/dev/ttyACM0')
    parser.add_argument('-b', '--baud', type=int, default=115200)
    parser.add_argument('-t', '--time', type=int, default=5, help='seconds to run')
    args = parser.parse_args()

    print(f'Opening {args.port} @ {args.baud}...')
    ser = serial.Serial(args.port, args.baud, timeout=0.1)
    time.sleep(0.5)
    ser.reset_input_buffer()

    buf = bytearray()
    state_count = 0
    tof_count   = 0
    t_start = time.time()
    t_report = t_start + 1.0
    last_state = None

    print(f'Reading for {args.time}s...\n')
    while time.time() - t_start < args.time:
        chunk = ser.read(256)
        if chunk:
            buf.extend(chunk)
        while True:
            result = decode_frame(buf)
            if result is None:
                break
            ftype, payload, consumed = result
            buf = buf[consumed:]
            if ftype == TYPE_STATE:
                state_count += 1
                last_state = parse_state(payload)
            elif ftype == TYPE_TOF:
                tof_count += 1

        now = time.time()
        if now >= t_report:
            elapsed = now - t_start
            hz = state_count / elapsed
            print(f't={elapsed:.1f}s  STATE={state_count} ({hz:.1f} Hz)  TOF={tof_count}')
            if last_state:
                ts, enc, acc, gyr = last_state
                print(f'  ts={ts}ms  enc={enc}')
                print(f'  acc=({acc[0]:.3f}, {acc[1]:.3f}, {acc[2]:.3f}) m/s²')
                print(f'  gyr=({gyr[0]:.4f}, {gyr[1]:.4f}, {gyr[2]:.4f}) rad/s')
            t_report = now + 1.0

    elapsed = time.time() - t_start
    hz = state_count / elapsed if elapsed > 0 else 0
    print(f'\n--- Results ---')
    print(f'STATE packets: {state_count} in {elapsed:.1f}s = {hz:.1f} Hz (target 100 Hz)')
    print(f'TOF packets:   {tof_count} in {elapsed:.1f}s = {tof_count/elapsed:.1f} Hz (target 10 Hz)')
    ok = 90 <= hz <= 110
    print(f'Rate check: {"PASS" if ok else "FAIL"}')

if __name__ == '__main__':
    main()
