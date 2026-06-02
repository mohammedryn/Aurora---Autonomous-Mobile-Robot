#!/usr/bin/env python3
"""
ISM330DHCX raw IMU data display — reads accel + gyro at 100 kHz SPI.

Run on the Pi (no ROS2 needed):
    python3 tools/imu_raw_data.py

Press Ctrl-C to stop.

Output columns:
    Ax Ay Az  [m/s²]    — linear acceleration (gravity included)
    Gx Gy Gz  [rad/s]   — angular velocity
    |G|       [rad/s]   — gyro magnitude (should be ~0 when still)

Healthy robot at rest:
    Az ≈ ±9.81 m/s²  (gravity, sign depends on IMU orientation)
    Ax, Ay ≈ 0
    Gx Gy Gz ≈ 0  (< 0.05 rad/s noise floor)
"""
import sys
import time
import struct
import math

try:
    import spidev
except ImportError:
    sys.exit("spidev not found — run: sudo apt install python3-spidev")

# ── Register map ──────────────────────────────────────────────────
WHO_AM_I = 0x0F
EXPECTED  = 0x6B
CTRL1_XL  = 0x10   # accel control
CTRL2_G   = 0x11   # gyro control
STATUS    = 0x1E   # XLDA | GDA ready flags
OUTX_L_G  = 0x22   # gyro  X low (6 bytes: Gx Gy Gz)
OUTX_L_A  = 0x28   # accel X low (6 bytes: Ax Ay Az)

# ── Scale factors matching production driver ───────────────────────
# CTRL1_XL=0x4A → ODR=104Hz, FS=±4g  → 0.122 mg/LSB → 0.001198 m/s²/LSB
# CTRL2_G =0x4C → ODR=104Hz, FS=±2000dps → 70 mdps/LSB → 0.001222 rad/s/LSB
ACCEL_SCALE = 0.122e-3 * 9.80665   # m/s² per LSB
GYRO_SCALE  = 70e-3 * (math.pi / 180.0)  # rad/s per LSB

BUS, DEVICE = 0, 0
RATE_HZ     = 50   # display rate


def make_spi() -> spidev.SpiDev:
    spi = spidev.SpiDev()
    spi.open(BUS, DEVICE)
    spi.max_speed_hz = 100_000
    spi.mode = 0
    return spi


def read_reg(spi, reg: int) -> int:
    return spi.xfer2([reg | 0x80, 0x00])[1]


def write_reg(spi, reg: int, val: int) -> None:
    spi.xfer2([reg & 0x7F, val])


def read_6(spi, reg: int):
    rx = spi.xfer2([reg | 0x80] + [0x00] * 6)
    return struct.unpack_from('<3h', bytes(rx[1:]))


def init(spi) -> None:
    who = read_reg(spi, WHO_AM_I)
    if who != EXPECTED:
        raise RuntimeError(
            f"WHO_AM_I=0x{who:02x}, expected 0x{EXPECTED:02x} — "
            f"check SPI wiring")
    write_reg(spi, CTRL1_XL, 0x4A)   # accel 104Hz ±4g  (matches ROS2 driver)
    write_reg(spi, CTRL2_G,  0x4C)   # gyro  104Hz ±2000dps
    time.sleep(0.02)
    print(f"ISM330DHCX OK  (WHO_AM_I=0x{EXPECTED:02x})\n")


print("=" * 72)
print("  ISM330DHCX raw data  —  Ctrl-C to stop")
print("  Settings: accel ±4g @ 104Hz,  gyro ±2000dps @ 104Hz,  SPI 100kHz")
print("=" * 72)
print(f"  {'Ax':>8} {'Ay':>8} {'Az':>8}  m/s²  |  "
      f"{'Gx':>8} {'Gy':>8} {'Gz':>8}  rad/s  |  |G|")
print("-" * 72)

spi = make_spi()
try:
    init(spi)
except RuntimeError as e:
    spi.close()
    sys.exit(f"\n  FAILED: {e}")

interval = 1.0 / RATE_HZ
n = 0
t_next = time.monotonic()

try:
    while True:
        now = time.monotonic()
        if now < t_next:
            time.sleep(t_next - now)
        t_next += interval

        gx_r, gy_r, gz_r = read_6(spi, OUTX_L_G)
        ax_r, ay_r, az_r = read_6(spi, OUTX_L_A)

        ax = ax_r * ACCEL_SCALE
        ay = ay_r * ACCEL_SCALE
        az = az_r * ACCEL_SCALE
        gx = gx_r * GYRO_SCALE
        gy = gy_r * GYRO_SCALE
        gz = gz_r * GYRO_SCALE
        g_mag = math.sqrt(gx*gx + gy*gy + gz*gz)

        print(f"  {ax:+8.3f} {ay:+8.3f} {az:+8.3f}        "
              f"{gx:+8.4f} {gy:+8.4f} {gz:+8.4f}       {g_mag:.4f}",
              end='\r')
        n += 1
        if n % (RATE_HZ * 5) == 0:   # newline every 5 s so history scrolls
            print()

except KeyboardInterrupt:
    print("\n\nStopped.")
finally:
    spi.close()
