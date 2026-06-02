#!/usr/bin/env python3
"""
IMU SPI health check — tests ISM330DHCX WHO_AM_I at multiple clock speeds.

Run on the Pi (no ROS2 needed):
    python3 tools/imu_spi_check.py

Requirements:
    sudo apt install python3-spidev   (or: pip3 install spidev)

Wiring check:
    Pi SPI0: MOSI=GPIO10/pin19, MISO=GPIO9/pin21,
             SCLK=GPIO11/pin23, CS0=GPIO8/pin24
"""
import sys
import time

try:
    import spidev
except ImportError:
    sys.exit("spidev not found — run: sudo apt install python3-spidev")

WHO_AM_I_REG = 0x0F
EXPECTED_ID  = 0x6B
CTRL1_XL     = 0x10
CTRL2_G      = 0x11
BUS, DEVICE  = 0, 0
REPS         = 30


def make_spi(speed_hz: int) -> spidev.SpiDev:
    spi = spidev.SpiDev()
    spi.open(BUS, DEVICE)
    spi.max_speed_hz = speed_hz
    spi.mode = 0  # CPOL=0 CPHA=0 — ISM330DHCX requirement
    return spi


def read_reg(spi, reg: int) -> int:
    return spi.xfer2([reg | 0x80, 0x00])[1]


def write_reg(spi, reg: int, val: int) -> None:
    spi.xfer2([reg & 0x7F, val])


def test_speed(speed_hz: int, n: int = REPS):
    spi = make_spi(speed_hz)
    ok = 0
    seen = set()
    for _ in range(n):
        v = read_reg(spi, WHO_AM_I_REG)
        seen.add(v)
        if v == EXPECTED_ID:
            ok += 1
        time.sleep(0.005)
    spi.close()
    return ok, n, sorted(seen)


def test_init(speed_hz: int):
    """Try full init at given speed: configure accel + gyro, read WHO_AM_I."""
    spi = make_spi(speed_hz)
    who = read_reg(spi, WHO_AM_I_REG)
    if who != EXPECTED_ID:
        spi.close()
        return False, f"WHO_AM_I=0x{who:02x}"
    write_reg(spi, CTRL1_XL, 0x4A)   # accel 104Hz ±4g
    write_reg(spi, CTRL2_G,  0x4C)   # gyro  104Hz ±2000dps
    time.sleep(0.01)
    who2 = read_reg(spi, WHO_AM_I_REG)
    spi.close()
    if who2 == EXPECTED_ID:
        return True, "init OK"
    return False, f"WHO_AM_I after init=0x{who2:02x}"


print("=" * 55)
print("  ISM330DHCX SPI health check")
print(f"  Device: /dev/spidev{BUS}.{DEVICE}   Expected ID: 0x{EXPECTED_ID:02x}")
print("=" * 55)

print(f"\n[1] WHO_AM_I stability — {REPS} reads per speed\n")
any_passed = False
for speed in [50_000, 100_000, 200_000, 500_000, 1_000_000]:
    ok, n, seen = test_speed(speed)
    ratio = ok / n * 100
    seen_str = ", ".join(f"0x{v:02x}" for v in seen)
    mark = "✓" if ok == n else ("~" if ok > 0 else "✗")
    print(f"  {mark} {speed//1000:5d} kHz : {ok:2d}/{n} ({ratio:5.1f}%)  "
          f"values: {seen_str}")
    if ok == n:
        any_passed = True

print(f"\n[2] Full init test at 100 kHz (production setting)\n")
ok_init, msg = test_init(100_000)
mark = "✓" if ok_init else "✗"
print(f"  {mark} {msg}")

print("\n" + "=" * 55)
if any_passed:
    # find best passing speed
    for speed in [100_000, 50_000, 200_000]:
        ok, n, _ = test_speed(speed, n=10)
        if ok == n:
            print(f"  RESULT: Connection marginal but works at {speed//1000} kHz")
            break
    else:
        print("  RESULT: Connection intermittent — solder the wires")
else:
    print("  RESULT: COMPLETE FAILURE — check wiring, power, and CS pin")
    print("  Most likely cause: loose/broken jumper wire")
print("=" * 55)
