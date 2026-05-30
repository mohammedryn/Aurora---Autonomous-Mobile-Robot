# AMR Project Log

**Complete chronological record of hardware bring-up, confirmed working state, and all ESP-IDF firmware debugging.**  
This document is the single source of truth for what has actually been built and tested on physical hardware.

---

## Table of Contents

1. [Project Overview](#1-project-overview)
2. [Hardware](#2-hardware)
3. [Confirmed Safe GPIO Pin Assignments](#3-confirmed-safe-gpio-pin-assignments)
4. [Wiring](#4-wiring)
5. [Mecanum Wheel Orientation](#5-mecanum-wheel-orientation)
6. [Motor Direction Matrix](#6-motor-direction-matrix)
7. [Currently Flashed Firmware — Arduino Sketch](#7-currently-flashed-firmware--arduino-sketch)
8. [Teleop Script — teleop_simple.py](#8-teleop-script--teleop_simplepy)
9. [Confirmed Working Status](#9-confirmed-working-status)
10. [ESP-IDF Firmware — Background](#10-esp-idf-firmware--background)
11. [ESP-IDF Debugging — Bugs Encountered and Fixes Tried](#11-esp-idf-debugging--bugs-encountered-and-fixes-tried)
    - [Bug #1: LEDC channel ≥ 1 hang (motor_init)](#bug-1-ledc-channel--1-hang-motor_init)
    - [Bug #2: I2C deprecation warning misread as driver install](#bug-2-i2c-deprecation-warning-misread-as-driver-install)
    - [Bug #3: Onboard peripheral GPIO conflicts](#bug-3-onboard-peripheral-gpio-conflicts)
    - [Bug #4: encoder_init hang](#bug-4-encoder_init-hang)
    - [Bug #5 (Root Cause): ESP_LOG path in early app_main](#bug-5-root-cause-esp_log-path-in-early-app_main)
12. [Detailed Chronological Attempt Log](#12-detailed-chronological-attempt-log)
    - [Attempt 0: Constant Symptom](#attempt-0-constant-symptom)
    - [Attempt 1: Change LEDC Channel Number](#attempt-1-change-ledc-channel-number)
    - [Attempt 2: Move PWM GPIO Away From 22/23 to 48/49](#attempt-2-move-pwm-gpio-away-from-2223-to-4849)
    - [Attempt 3: Skip ledc_channel_config for Motors 2 and 3](#attempt-3-skip-ledc_channel_config-for-motors-2-and-3)
    - [Attempt 4: Fix VL53L5CX I2C GPIO Pins](#attempt-4-fix-vl53l5cx-i2c-gpio-pins)
    - [Attempt 5: Full Rewrite from LEDC to MCPWM](#attempt-5-full-rewrite-from-ledc-to-mcpwm)
    - [Attempt 6: Interim Theory — GPIO22 / SDIO / ESP32-C6 Conflict](#attempt-6-interim-theory--gpio22--sdio--esp32-c6-conflict)
    - [Attempt 7: Change PWM Away From GPIO 20–23](#attempt-7-change-pwm-away-from-gpio-2023)
    - [Attempt 8: Confirm Fresh Firmware Was Actually Flashing](#attempt-8-confirm-fresh-firmware-was-actually-flashing)
    - [Attempt 9: Check MCPWM Hardware Capacity](#attempt-9-check-mcpwm-hardware-capacity)
    - [Attempt 10: Add MOTOR_CHECK Error Handling](#attempt-10-add-motor_check-error-handling)
    - [Attempt 11: Add esp_rom_printf Breadcrumbs](#attempt-11-add-esp_rom_printf-breadcrumbs)
    - [Attempt 12: Move Timer Start to End](#attempt-12-move-timer-start-to-end)
    - [Attempt 13: Change DIR Away From GPIO 46/47 to GPIO 20/21](#attempt-13-change-dir-away-from-gpio-4647-to-gpio-2021)
    - [Attempt 14: Suppress ESP-IDF GPIO Driver Logs](#attempt-14-suppress-esp-idf-gpio-driver-logs)
    - [Attempt 15: Use GPIO 11/13 for DIR](#attempt-15-use-gpio-1113-for-dir)
    - [Attempt 16: Avoid Audio GPIO 9–13 Completely](#attempt-16-avoid-audio-gpio-913-completely)
    - [Attempt 17: Remove Normal esp_rom_printf Breadcrumbs](#attempt-17-remove-normal-esp_rom_printf-breadcrumbs)
    - [Attempt 18: Current Pin Mapping and Code State](#attempt-18-current-pin-mapping-and-code-state)
    - [Attempt 19: Git / Workflow Issues](#attempt-19-git--workflow-issues)
    - [Attempt 20: Skip motor_init() entirely (BOOT_DIAG flags)](#attempt-20-skip-motor_init-entirely-boot_diag-flags)
    - [Attempt 21: Skip encoder_init() as well](#attempt-21-skip-encoder_init-as-well)
    - [Attempt 22: Bare app_main with only esp_rom_printf](#attempt-22-bare-app_main-with-only-esp_rom_printf)
    - [Attempt 23: Restore full original app_main](#attempt-23-restore-full-original-app_main)
13. [What Was Definitively Ruled Out](#13-what-was-definitively-ruled-out)
14. [What Was Confirmed as Real Problems](#14-what-was-confirmed-as-real-problems)
15. [What Remained Suspicious (Not Fully Proven)](#15-what-remained-suspicious-not-fully-proven)
16. [Final Combined Conclusion](#16-final-combined-conclusion)
17. [Lessons Learned](#17-lessons-learned)
18. [Architecture Pivot — ESP32-P4 Scope Reduction](#18-architecture-pivot--esp32-p4-scope-reduction)
19. [Critical Discovery — USB CDC vs USB JTAG](#19-critical-discovery--usb-cdc-vs-usb-jtag)
20. [ESP-IDF Motor Test Firmware — Confirmed Working](#20-esp-idf-motor-test-firmware--confirmed-working)
21. [Encoder GPIO Selection — Physical Board Constraints](#21-encoder-gpio-selection--physical-board-constraints)
22. [Encoder Testing — All 4 Wheels Confirmed](#22-encoder-testing--all-4-wheels-confirmed)
23. [Full ESP-IDF Firmware — Motors + Encoders + Serial Protocol](#23-full-esp-idf-firmware--motors--encoders--serial-protocol)
24. [Updated Confirmed Working Status](#24-updated-confirmed-working-status-as-of-2026-05-30)

---

## 1. Project Overview

**Goal:** Autonomous mobile robot (AMR) with 4-wheel mecanum drive. Explores an unknown room, builds a map, returns home, and navigates to user-clicked goals.

**Architecture:**
- ESP32-P4 runs FreeRTOS firmware: quadrature encoder reading, PID wheel velocity control, IMU reading, ToF sensing, binary serial protocol.
- Raspberry Pi 5 runs ROS2 Jazzy: ros2_control hardware interface, EKF localization, slam_toolbox SLAM, Nav2 navigation, frontier exploration.
- Communication: USB-C serial link at 921600 baud (binary framed packets).
- Dev machine: WSL2 Ubuntu 22.04 (code, ESP-IDF toolchain, ROS2 CLI).

---

## 2. Hardware

| Component | Model / Spec | Role |
|---|---|---|
| MCU | Waveshare ESP32-P4-WIFI6 dev board (ESP32-P4 rev v1.3 main RISC-V MCU + ESP32-C6 WiFi co-processor connected via internal SDIO bus) | Real-time motor/sensor control |
| Motor Drivers | 2× Cytron MDD10A (dual H-bridge, 10A/ch continuous, 30A peak, 6–24V, sign-magnitude mode) | Drive 4 motors |
| Motors | 4× PGM45775-19.2K (12V, 19.2:1 gearbox, ~187 RPM output, built-in ME-37 quadrature encoder, 7 PPR on motor shaft → 537.6 counts/output rev) | Wheel actuation |
| IMU | ISM330DHCX (SPI) — SmartElex 9DoF breakout (also has MMC5983MA magnetometer, unused) | Inertial sensing |
| ToF | VL53L5CX 8×8 pixel (I2C) — SmartElex breakout, 4m range, 63°×63° FoV | Low-obstacle detection |
| LiDAR | Slamtec C1M1 R2 (USB, 12m, 360°, 10Hz) | Primary SLAM sensor |
| Battery (motors) | 3S3P LiPo, 11.1V nominal, ~7800 mAh | Motor power rail |
| Battery (compute) | 4S LiPo 1200mAh + XL buck → 5.12V | RPi5 power rail (isolated from motor rail) |
| Compute host | Raspberry Pi 5 8GB, Ubuntu 24.04, ROS2 Jazzy | High-level compute, SLAM, Nav2, teleop |
| Dev machine | WSL2 Ubuntu 22.04 on Windows 11 | Code editing, ESP-IDF builds, ROS2 CLI |

**Encoder math:**
```
7 PPR (motor shaft) × 4 (quadrature) × 19.2 (gear ratio) = 537.6 counts/output revolution
RAD_PER_COUNT = 2π / 537.6 = 0.01169 rad/count
Wheel circumference = π × 0.060m = 0.1885m
Linear resolution = 0.1885 / 537.6 = 0.35mm per count
```

---

## 3. Confirmed Safe GPIO Pin Assignments

These are the **only** pins confirmed working with motor drivers on this Waveshare board, verified by physical motor-response testing. Do not change these without re-testing.

| Signal  | GPIO | Side      | MDD10A Channel |
|---------|------|-----------|----------------|
| FL_PWM  | 5    | Left #1   | PWM1           |
| FL_DIR  | 26   | Left #1   | DIR1           |
| RL_PWM  | 32   | Left #1   | PWM2           |
| RL_DIR  | 27   | Left #1   | DIR2           |
| FR_PWM  | 33   | Right #2  | PWM1           |
| FR_DIR  | 2    | Right #2  | DIR1           |
| RR_PWM  | 52   | Right #2  | PWM2           |
| RR_DIR  | 4    | Right #2  | DIR2           |

**Sensor pins (planned, not yet wired):**

| Signal       | GPIO |
|--------------|------|
| I2C SDA (VL53L5CX) | 7 |
| I2C SCL (VL53L5CX) | 8 |
| SPI (ISM330DHCX)   | 36–39 |

**GPIO zones to avoid entirely on Waveshare ESP32-P4-WIFI6:**

| GPIO Range | Reason |
|---|---|
| 9–13 | Onboard audio codec I2S (DSDIN=9, LRCK=10, ASDOUT=11, SCLK=12, MCLK=13) |
| 18–23 | Suspected SDIO bus to ESP32-C6 WiFi co-processor |
| 24–25 | USB OTG D+/D- |
| 28–31 | Camera / display zone |

---

## 4. Wiring

### MDD10A #1 — Left Side (signal connector)

| MDD10A Pin | Connected To |
|---|---|
| GND | ESP32 GND (common ground rail) |
| VCC | ESP32 3.3V pin |
| PWM1 | GPIO 5 (FL speed) |
| DIR1 | GPIO 26 (FL direction) |
| PWM2 | GPIO 32 (RL speed) |
| DIR2 | GPIO 27 (RL direction) |
| M1A / M1B | Front-Left motor leads |
| M2A / M2B | Rear-Left motor leads |

### MDD10A #2 — Right Side (signal connector)

| MDD10A Pin | Connected To |
|---|---|
| GND | ESP32 GND (common ground rail) |
| VCC | ESP32 3.3V pin |
| PWM1 | GPIO 33 (FR speed) |
| DIR1 | GPIO 2 (FR direction) |
| PWM2 | GPIO 52 (RR speed) |
| DIR2 | GPIO 4 (RR direction) |
| M1A / M1B | Front-Right motor leads |
| M2A / M2B | Rear-Right motor leads |

### Power

| Rail | Source | Voltage | Consumers |
|---|---|---|---|
| Motor power | Main 3S3P LiPo | 11.1–12.8V | VM+ on both MDD10As |
| Logic VCC | ESP32-P4 3V3 pin | 3.3V | VCC on both MDD10As |
| ESP32 power | RPi5 USB-A → USB-C | 5V | ESP32-P4 |
| RPi5 power | 4S LiPo + XL buck | 5.12V | RPi5 only (isolated) |

**Critical rule:** ESP32 GND, both MDD10A GNDs, and main battery negative must all share a single common ground rail. Floating grounds cause erratic motor behavior and can damage GPIO pins.

---

## 5. Mecanum Wheel Orientation

Viewed from above — X pattern:

```
              FRONT
    FL (left-hand)    FR (right-hand)
          \                /
          /                \
    RL (right-hand)   RR (left-hand)
              REAR
```

- **FL and RR** have left-hand (/) rollers
- **FR and RL** have right-hand (\) rollers

This roller orientation enables holonomic motion (forward, strafe, diagonal, rotate) using the motor direction matrix below.

---

## 6. Motor Direction Matrix

`+` = forward spin, `−` = reverse spin, `0` = stopped.

| Command           | FL | FR | RL | RR |
|-------------------|----|----|----|-----|
| Forward           | +  | +  | +  | +   |
| Backward          | −  | −  | −  | −   |
| Rotate left (CCW) | −  | +  | −  | +   |
| Rotate right (CW) | +  | −  | +  | −   |
| Strafe left       | −  | +  | +  | −   |
| Strafe right      | +  | −  | −  | +   |
| Fwd-left diagonal | 0  | +  | +  | 0   |
| Fwd-right diagonal| +  | 0  | 0  | +   |
| Bwd-left diagonal | −  | 0  | 0  | −   |
| Bwd-right diagonal| 0  | −  | −  | 0   |

---

## 7. Currently Flashed Firmware — Arduino Sketch

**Status:** This is what is running on the ESP32 right now. NOT the ESP-IDF firmware.

Listens on USB serial at **115200 baud**. Accepts single-character commands. `SPEED = 80` out of 255 (chosen for safe indoor testing).

```cpp
#define FL_PWM 5
#define FL_DIR 26
#define RL_PWM 32
#define RL_DIR 27
#define FR_PWM 33
#define FR_DIR 2
#define RR_PWM 52
#define RR_DIR 4
#define SPEED 80

void setMotor(int pwmPin, int dirPin, int speed) {
    if (speed > 0)      { digitalWrite(dirPin, HIGH); analogWrite(pwmPin,  speed); }
    else if (speed < 0) { digitalWrite(dirPin, LOW);  analogWrite(pwmPin, -speed); }
    else                { digitalWrite(dirPin, LOW);  analogWrite(pwmPin, 0); }
}

void stopAll() {
    setMotor(FL_PWM, FL_DIR, 0); setMotor(RL_PWM, RL_DIR, 0);
    setMotor(FR_PWM, FR_DIR, 0); setMotor(RR_PWM, RR_DIR, 0);
}

void setup() {
    Serial.begin(115200);
    pinMode(FL_PWM, OUTPUT); pinMode(FL_DIR, OUTPUT);
    pinMode(RL_PWM, OUTPUT); pinMode(RL_DIR, OUTPUT);
    pinMode(FR_PWM, OUTPUT); pinMode(FR_DIR, OUTPUT);
    pinMode(RR_PWM, OUTPUT); pinMode(RR_DIR, OUTPUT);
    stopAll();
    Serial.println("Ready");
}

void loop() {
    if (Serial.available()) {
        char cmd = tolower(Serial.read());
        switch (cmd) {
            case 'f': setMotor(FL_PWM, FL_DIR,  SPEED); setMotor(RL_PWM, RL_DIR,  SPEED);
                      setMotor(FR_PWM, FR_DIR,  SPEED); setMotor(RR_PWM, RR_DIR,  SPEED); break;
            case 'b': setMotor(FL_PWM, FL_DIR, -SPEED); setMotor(RL_PWM, RL_DIR, -SPEED);
                      setMotor(FR_PWM, FR_DIR, -SPEED); setMotor(RR_PWM, RR_DIR, -SPEED); break;
            case 'l': setMotor(FL_PWM, FL_DIR, -SPEED); setMotor(RL_PWM, RL_DIR, -SPEED);
                      setMotor(FR_PWM, FR_DIR,  SPEED); setMotor(RR_PWM, RR_DIR,  SPEED); break;
            case 'r': setMotor(FL_PWM, FL_DIR,  SPEED); setMotor(RL_PWM, RL_DIR,  SPEED);
                      setMotor(FR_PWM, FR_DIR, -SPEED); setMotor(RR_PWM, RR_DIR, -SPEED); break;
            case 'q': setMotor(FL_PWM, FL_DIR,      0); setMotor(RL_PWM, RL_DIR,  SPEED);
                      setMotor(FR_PWM, FR_DIR,  SPEED); setMotor(RR_PWM, RR_DIR,      0); break;
            case 'e': setMotor(FL_PWM, FL_DIR,  SPEED); setMotor(RL_PWM, RL_DIR,      0);
                      setMotor(FR_PWM, FR_DIR,      0); setMotor(RR_PWM, RR_DIR,  SPEED); break;
            case 'z': setMotor(FL_PWM, FL_DIR, -SPEED); setMotor(RL_PWM, RL_DIR,      0);
                      setMotor(FR_PWM, FR_DIR,      0); setMotor(RR_PWM, RR_DIR, -SPEED); break;
            case 'x': setMotor(FL_PWM, FL_DIR,      0); setMotor(RL_PWM, RL_DIR, -SPEED);
                      setMotor(FR_PWM, FR_DIR, -SPEED); setMotor(RR_PWM, RR_DIR,      0); break;
            case 's': case ' ': stopAll(); break;
        }
    }
}
```

**Command reference:**

| Char | Motion             |
|------|--------------------|
| `f`  | Forward            |
| `b`  | Backward           |
| `l`  | Rotate left (CCW)  |
| `r`  | Rotate right (CW)  |
| `q`  | Fwd-left diagonal  |
| `e`  | Fwd-right diagonal |
| `z`  | Bwd-left diagonal  |
| `x`  | Bwd-right diagonal |
| `s` / Space | Stop      |

---

## 8. Teleop Script — teleop_simple.py

Runs on the Raspberry Pi. Latch mode: press a key → robot moves and keeps moving until Space is pressed. Esc to quit.

```python
#!/usr/bin/env python3
import curses, serial, argparse, time

KEY_TO_CMD = {
    curses.KEY_UP:    b'f', curses.KEY_DOWN:  b'b',
    curses.KEY_LEFT:  b'l', curses.KEY_RIGHT: b'r',
    ord('q'): b'q', ord('Q'): b'q',
    ord('e'): b'e', ord('E'): b'e',
    ord('z'): b'z', ord('Z'): b'z',
    ord('x'): b'x', ord('X'): b'x',
    ord(' '): b's',
}
LABELS = {
    b'f': 'FORWARD',    b'b': 'BACKWARD',
    b'l': 'ROTATE LEFT', b'r': 'ROTATE RIGHT',
    b'q': 'FWD-LEFT',   b'e': 'FWD-RIGHT',
    b'z': 'BWD-LEFT',   b'x': 'BWD-RIGHT',
    b's': 'STOP',
}

def main(stdscr, port, baud):
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.keypad(True)
    ser = serial.Serial(port, baud, timeout=0)
    cmd = b's'
    try:
        while True:
            key = stdscr.getch()
            if key in (27, 3):
                break
            elif key in KEY_TO_CMD:
                cmd = KEY_TO_CMD[key]
                ser.write(cmd)
            stdscr.erase()
            stdscr.addstr(0, 0, "AMR Teleop")
            stdscr.addstr(6, 0, f"Command:  {LABELS.get(cmd, '?')}")
            stdscr.refresh()
            time.sleep(0.05)
    finally:
        try:
            ser.write(b's')
        except Exception:
            pass
        ser.close()

def run():
    p = argparse.ArgumentParser()
    p.add_argument('--port', default='/dev/amr_mcu')
    p.add_argument('--baud', type=int, default=115200)
    a = p.parse_args()
    curses.wrapper(main, a.port, a.baud)

if __name__ == '__main__':
    run()
```

**Run on Pi:**
```bash
python3 ~/AMR/teleop/teleop_simple.py --port /dev/ttyACM0
```

---

## 9. Confirmed Working Status

Everything below has been physically tested and confirmed functional as of this log.

- All 4 motors respond correctly to all 9 commands
- Forward and backward confirmed
- Rotate left and rotate right confirmed
- All 4 diagonal movements confirmed (q, e, z, x)
- Strafe left and strafe right confirmed (l, r)
- Pi → ESP32 serial control via `teleop_simple.py` confirmed
- Latch mode confirmed (robot holds motion until Space is pressed)
- SPEED = 80 is comfortable and safe for indoor testing

---

## 10. ESP-IDF Firmware — Background

The repository also contains a full ESP-IDF v5.4.1 firmware under `firmware/` with FreeRTOS tasks for:
- Encoder reading (PCNT quadrature decoder, 4 motors)
- PID velocity control (1 kHz, 4 independent controllers)
- IMU reading (ISM330DHCX over SPI, 100 Hz)
- ToF reading (VL53L5CX over I2C, 10 Hz)
- Serial communications (binary framed protocol at 921600 baud)

**This firmware is NOT currently flashed.** It is on hold while the Arduino sketch handles manual teleop.

We spent 2 days debugging a boot hang in this firmware. The root cause was eventually traced to a transient corrupted firmware state accumulated during rapid flash-test-modify cycles — **not** a hardware or driver bug. A bare `app_main` using only `esp_rom_printf` booted clean and ran forever (proving board hardware is healthy). After a full `idf.py fullclean` and clean rebuild of the original `app_main`, the firmware worked perfectly.

The ESP-IDF path is the long-term plan for closed-loop PID + ROS2 hardware interface (Phase 2 / Phase 3 of the implementation plan).

---

## 11. ESP-IDF Debugging — Bugs Encountered and Fixes Tried

The following section documents every theory, fix, and conclusion from the two-day debugging session. The appendix that follows (Section 12) provides the detailed chronological attempt-by-attempt log.

---

### Bug #1: LEDC Channel ≥ 1 Hang (motor_init)

**Symptom:** `motor_init()` called `ledc_channel_config()` for 4 motor PWM channels. The board hung mid-print of `"Pullup:"` from `gpio_config()` on channel 2's DIR pin. No panic, no backtrace, no watchdog message, no abort — just a silent freeze.

Original offending pin state:
```
PWM_GPIO[] = {20, 21, 22, 23}
DIR_GPIO[] = {26, 27, 46, 47}
```

**Theory:** ESP32-P4 rev v1.3 + ESP-IDF v5.4.1 has a bug where `ledc_channel_config()` for channel ≥ 1 writes to LEDC GAMMA_RAM at an address that triggers a deferred AHB bus fault. The RISC-V store buffer holds the faulting write and delivers the exception at the NEXT AHB access (which was the `gpio_config` call for DIR_GPIO[2]). The panic handler cannot print while UART is mid-operation, causing a silent hang.

**Fixes tried:**
- Switched LEDC channel from 2 → 3 → 0 → 4. Same hang every time.
- Skipped `ledc_channel_config()` for channels ≥ 2. Still hung (fault from ch=1's GAMMA_RAM write surfaced at ch=2's `gpio_config`).
- Final fix attempted: rewrote entire `motor.c` to use MCPWM instead of LEDC (commit d64b98e). 4 motors across 2 MCPWM groups, 20 kHz, 10 MHz resolution.

**Result: MCPWM rewrite did NOT solve the hang.** Identical `"GPIO Pullup:"` partial-print symptom. **Theory was WRONG** — LEDC GAMMA_RAM was not the root cause.

---

### Bug #2: I2C Deprecation Warning Misread as Driver Install

**Symptom:** Boot log showed:
```
W (278) i2c: This driver is an old driver, please migrate your application
            code to adapt 'driver/i2c_master.h'
```

**Theory:** An I2C driver was being installed early and conflicting with motor GPIO pins (motor PWM was on GPIO 22/23 which are also I2C SDA/SCL on some boards).

**Fixes tried:**
- Changed VL53L5CX I2C pins from SDA=22/SCL=23 → SDA=7/SCL=8.

**Result:** Same boot hang. Investigation revealed the W(278) message comes from `check_i2c_driver_conflict()` in `esp-idf/components/driver/i2c/i2c.c` — it is an `__attribute__((constructor))` function that logs at C-runtime initialization, BEFORE `main_task` ever starts, regardless of whether any I2C driver is actually installed. **It was a red herring.** No I2C driver was running early. The SDA/SCL=7/8 change was kept as a permanent improvement.

---

### Bug #3: Onboard Peripheral GPIO Conflicts

**Symptom:** Hang at random GPIO config calls. The hang appeared to follow whichever GPIO was being configured at the time, not a specific GPIO number.

**Theory:** The Waveshare ESP32-P4-WIFI6 board has many GPIO pins reserved for onboard peripherals that are not clearly documented in the schematic:
- GPIO 9–13: onboard audio codec (DSDIN, LRCK, ASDOUT, SCLK, MCLK)
- GPIO 18–23: suspected SDIO bus to onboard ESP32-C6 WiFi chip
- GPIO 48–52: suspected SDIO or C6 control
- GPIO 2, 3, 4: low-number strapping pins / possible C6 control
- GPIO 24–25: USB D+/D-
- GPIO 28–33: camera/display zone (later proved partially safe)
- GPIO 36–39: SPI bus for ISM330DHCX IMU

**Fixes tried:**
- Tried motor DIR pins on GPIO 11, 13: froze (GPIO 11 = ASDOUT, codec was driving the pin)
- Tried motor PWM on GPIO 20, 21, 48, 49: froze (GPIO 48/49 conflicted with encoder PCNT, which ran first)
- Tried encoder GPIO_A[]={48,49,50,51} + GPIO_B[]={52,2,3,4}: encoder_init hung at partial "I"
- Final safe set discovered through physical motor-test: PWM={5,32,33,52} DIR={26,27,2,4}

**Result:** Partially correct. The board does have restricted GPIO zones, and identifying them was necessary. But the GPIO conflicts were not the primary root cause of the boot crash.

---

### Bug #4: encoder_init Hang

**Symptom:** With `motor_init()` entirely skipped via `BOOT_DIAG_SKIP_MOTOR_INIT=1`, the board still froze. Print sequence: `"Init encoders..."` then partial `"I"` then silence.

**Theory:** Encoder uses PCNT driver on GPIO 48–52, 2–4, which overlap with the reserved C6 SDIO / strapping zones.

**Fixes tried:**
- Added `BOOT_DIAG_SKIP_ENCODER_INIT=1` to also skip `encoder_init()`.

**Result:** With BOTH skipped, the board STILL rebooted in a loop at ~290ms, printing partial `"BOOT_DIAG_SKIP_ENCODER_INIT"` then software-resetting and repeating. This proved the bug was NOT in `encoder_init` OR `motor_init` — it was in `app_main` itself, somewhere in the ESP_LOG calls or the surrounding setup.

---

### Bug #5 (Root Cause): ESP_LOG Path in Early app_main

**Symptom:** Even with all hardware init skipped, `app_main` rebooted at ~290ms. Reset reason: `0xc = SW_CPU_RESET`. Both cores had saved PC pointing at `esp_cpu_reset` and `esp_cpu_wait_for_intr` — useless for identifying the trigger.

**Theory:** Something in the `ESP_LOGI/ESP_LOGW` path (logging VFS, UART driver, or interaction with the `shared_state` global) triggered a panic or reset that did not print.

**Diagnostic step that proved board health:**

Stripped `app_main` down to bare bones — only `esp_rom_printf()` (no logging subsystem, no globals, no includes). Added a heartbeat loop:

```c
void app_main(void) {
    esp_rom_printf("[diag] step1: app_main entered\n");
    esp_rom_printf("[diag] step2: reset_reason=%d\n",
                   (int)esp_reset_reason());
    for (int i = 0; i < 10; i++) {
        esp_rom_printf("[diag] step3.%d: alive at %dms\n", i,
                       (int)(xTaskGetTickCount() * portTICK_PERIOD_MS));
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    while (1) vTaskDelay(pdMS_TO_TICKS(1000));
}
```

**Result: Booted CLEAN.** `reset_reason=1 (POWERON)`. All 10 heartbeats fired at 500ms intervals. Idle loop reached and held. THE BOARD HARDWARE IS CONFIRMED HEALTHY.

**Next bisect step (commit 44b5ae2):** Restored full original `app_main` (includes, `g_state`, `ESP_LOG`) but kept all hardware init/task creation calls.

**Result: "It worked perfectly."**

**Final conclusion:** The ~290ms reboot loop was likely caused by one or more of:
- Transient corrupted state from a bad firmware image accumulated during rapid flash-test-modify-reflash cycles
- Interaction between multiple `#if BOOT_DIAG_SKIP_*` preprocessor blocks and partial-skip builds producing an inconsistent binary
- A build cache artifact from `idf.py` serving a stale `.o` file

The exact triggering mechanism was never fully isolated because `idf.py fullclean && idf.py build flash` resolved it entirely. No hardware fault. No driver bug. No GPIO conflict caused the reboot loop.

---

## 12. Detailed Chronological Attempt Log

This section records every individual fix attempt in order, with exact changes, observations, and conclusions.

---

### Attempt 0: Constant Symptom

The baseline symptom observed across most early tests:

```
I (...) main: Init hardware...
I (...) gpio: GPIO[26] ... Pullup: 0| Pulldown: 0| Intr:0
I (...) gpio: GPIO[27] ... Pullup: 0| Pulldown: 0| Intr:0
I (...) gpio: GPIO[46] ... Pullup:
[silence]
```

Characteristics:
- No panic
- No reboot (at this stage)
- No backtrace
- No watchdog message
- No abort
- No `ESP_ERROR_CHECK` failure
- Hard silent freeze, always at the same visual point in the gpio_config INFO log
- Always appeared to be: `motor_init()` → 3rd motor iteration → DIR_GPIO[2]=GPIO46 → mid-print of `gpio_config` INFO line

Original pin state at this point:
```c
PWM_GPIO[] = {20, 21, 22, 23}
DIR_GPIO[] = {26, 27, 46, 47}
```

---

### Attempt 1: Change LEDC Channel Number

**Hypothesis:** LEDC channel 2 itself is broken on ESP32-P4.

**Change:**
```c
PWM_CH[] = {
    LEDC_CHANNEL_0,
    LEDC_CHANNEL_1,
    LEDC_CHANNEL_3,   // skip channel 2
    LEDC_CHANNEL_4,
};
```

**Result:** Identical freeze. Still stopped at `GPIO[46] Pullup:`. Same visible timing.

**Conclusion:** LEDC channel number was not the cause. The specific channel index did not matter.

**Status:** Ruled out.

---

### Attempt 2: Move PWM GPIO Away From 22/23 to 48/49

**Hypothesis:** GPIO22/GPIO23 may be internal WiFi co-processor or board-reserved pins. Move motor PWM off them.

**Change:**
```c
PWM_GPIO[] = {20, 21, 48, 49}
```

**Result:** Still froze. Introduced a new GPIO conflict.

**Why conflict:** `encoder.c` had `GPIO_A[] = {48, 49, 50, 51}`. `encoder_init()` ran before `motor_init()`, so GPIO48/GPIO49 were already claimed by PCNT encoder setup.

**Conclusion:** GPIO48/49 are not available for motor PWM. The original hang did not disappear.

**Status:** GPIO48/49 ruled out for motor PWM. Reverted `PWM_GPIO[]` to `{20, 21, 22, 23}`.

---

### Attempt 3: Skip ledc_channel_config for Motors 2 and 3

**Hypothesis:** The third `ledc_channel_config()` may write into bad LEDC/GAMMA_RAM state. Skipping that call should stop the freeze.

**Change:**
```c
if (i < 2) {
    ledc_channel_config(&ch);
}
gpio_config(&d);
gpio_set_level(DIR_GPIO[i], 0);
```

**Result:** Identical freeze. Still stopped at `GPIO[46] Pullup:`. Same timing, around the same boot point.

**Conclusion:** The freeze was not caused directly by the third `ledc_channel_config()`. The failure appeared time-correlated, not call-correlated — something seemed to happen around the same boot timestamp regardless of what was being executed.

**Important insight gained:** The visible last log line may be a victim, not the cause.

**Status:** LEDC channel config as direct cause ruled out.

---

### Attempt 4: Fix VL53L5CX I2C GPIO Pins

**Hypothesis:** VL53L5CX I2C used GPIO22/GPIO23, same as motor PWM. Maybe legacy I2C initialized early, hit bad pins, timed out, and caused a log deadlock.

**Original:**
```c
#define PIN_SDA 22
#define PIN_SCL 23
```

**Changed to:**
```c
#define PIN_SDA 7
#define PIN_SCL 8
```

Also restored full LEDC setup (removed the `i < 2` diagnostic guard).

**Result:** Identical freeze. `GPIO26` OK, `GPIO27` OK, `GPIO46 Pullup:` freeze.

**Conclusion:** The I2C pin conflict was real and needed fixing as a correctness issue, but it was not the motor_init freeze root cause. Also confirmed: the W(278) i2c deprecation warning appearing before `app_main` is from an `__attribute__((constructor))` function, not an actual installed driver.

**Status:** I2C pin conflict fixed (SDA=7, SCL=8 kept permanently). I2C as freeze cause ruled out.

---

### Attempt 5: Full Rewrite from LEDC to MCPWM

**Hypothesis:** Even if skipping one `ledc_channel_config` did not help, some LEDC timer/channel/GAMMA_RAM interaction may still poison the system. Replace LEDC completely.

**Change:** Removed LEDC motor driver. Implemented MCPWM motor driver.

MCPWM design:
- 20 kHz PWM
- 10 MHz resolution (500 ticks per period)
- Motors 0, 1, 2 → MCPWM Group 0 (operators 0, 1, 2)
- Motor 3 → MCPWM Group 1 (operator 0)

Initial MCPWM pins (unchanged):
```c
PWM_GPIO[] = {20, 21, 22, 23}
DIR_GPIO[] = {26, 27, 46, 47}
```

**Result:** Identical freeze. Still appeared at `GPIO[46] Pullup:` — same symptom as with LEDC.

**Conclusion:** LEDC was not the root cause. The freeze happened identically with both LEDC and MCPWM. The peripheral type was not sufficient to explain the failure.

**Status:** LEDC/GAMMA_RAM theory ruled out as direct root cause. MCPWM kept for its other advantages (no GAMMA_RAM, better suited for motor control).

---

### Attempt 6: Interim Theory — GPIO22 / SDIO / ESP32-C6 Conflict

This was the prevailing strong hypothesis at this stage, based on accumulated evidence.

**Hypothesis:** GPIO22 is part of the ESP32-P4 ↔ ESP32-C6 SDIO bus. Routing PWM to GPIO22 corrupts SDIO, which triggers an ISR storm. UART cannot drain. The `gpio_config` log print freezes mid-line.

**Status at the time:** This was a reasonable hypothesis given the evidence.

**Correction (learned later):** This theory was only partially right. We moved PWM away from GPIO22 in Attempt 7 and the freeze continued. So GPIO22/SDIO may have been a real conflict but was not the sole or root cause. The actual root cause was broader.

---

### Attempt 7: Change PWM Away From GPIO 20–23

**Hypothesis:** Avoid the suspected SDIO/control range entirely.

**Change:**
```c
PWM_GPIO[] = {5, 9, 10, 12}
DIR_GPIO[] = {26, 27, 46, 47}
```

**Result:** Build passed. Fresh binary flashed. App version confirmed changed. Still froze.

**Important caveat:** GPIO9/10/12 are onboard audio codec I2S pins. This test accidentally entered another bad pin zone, so the result is contaminated.

**Conclusion:** This weakened the GPIO22-only theory but was not a clean test because audio codec pins were accidentally used.

**Status:** Contaminated test. Useful signal, not definitive proof.

---

### Attempt 8: Confirm Fresh Firmware Was Actually Flashing

**Problem to rule out:** Maybe the Pi was flashing stale binaries, or the build cache was lying.

**Evidence that proved new firmware was flashing:**
- App versions changed between flashes: `90b620c`, `7385843`, `56da050`, `506dea5`
- Compile timestamps changed with each build
- Commands used included `idf.py fullclean` between some builds

**Conclusion:** The board was receiving new firmware. Stale code was not the explanation.

**Status:** Stale firmware ruled out.

---

### Attempt 9: Check MCPWM Hardware Capacity

**Hypothesis:** Maybe ESP32-P4 MCPWM Group 0 cannot support 3 motors.

**Checked ESP-IDF SoC caps:**
```c
SOC_MCPWM_GROUPS = 2
SOC_MCPWM_OPERATORS_PER_GROUP = 3
SOC_MCPWM_TIMERS_PER_GROUP = 3
```

Our mapping:
```c
s_group[] = {0, 0, 0, 1}
```

**Conclusion:** This is legal. Group 0 can support motors 0, 1, 2. Group 1 supports motor 3.

**Status:** MCPWM operator exhaustion ruled out.

---

### Attempt 10: Add MOTOR_CHECK Error Handling

**Problem:** `motor.c` ignored return values from all ESP-IDF driver calls. A hidden `ESP_ERR_*` could poison later calls silently.

**Change:** Added `MOTOR_CHECK` macro:
```c
#define MOTOR_CHECK(label, expr) do {                                          \
    esp_err_t err__ = (expr);                                                  \
    if (err__ != ESP_OK) {                                                     \
        esp_rom_printf("[motor] %s failed: %s (%d)\n",                         \
            (label), esp_err_to_name(err__), err__);                           \
        ESP_ERROR_CHECK(err__);                                                \
    }                                                                          \
} while (0)
```

Wrapped calls: `mcpwm_new_timer()`, `mcpwm_new_operator()`, `mcpwm_operator_connect_timer()`, `mcpwm_new_comparator()`, `mcpwm_comparator_set_compare_value()`, `mcpwm_new_generator()`, `mcpwm_generator_set_action_on_timer_event()`, `mcpwm_generator_set_action_on_compare_event()`, `gpio_config()`, `gpio_set_level()`, `mcpwm_timer_enable()`, `mcpwm_timer_start_stop()`.

**Result:** No `MOTOR_CHECK` failure appeared before the freeze. The calls completed successfully for multiple channels before it froze.

**Conclusion:** No simple unchecked `ESP_ERR_INVALID_ARG` / `ESP_ERR_NOT_FOUND` was being silently ignored before the freeze.

**Status:** `MOTOR_CHECK` macro retained as permanent good practice. Unchecked ESP-IDF error as simple root cause weakened.

---

### Attempt 11: Add esp_rom_printf Breadcrumbs

**Hypothesis:** `ESP_LOGI` / `gpio` logs are misleading. Use ROM-level prints to see exact progress.

**Added prints:**
```c
esp_rom_printf("[motor] init begin\n");
esp_rom_printf("[motor] timer group=%d new\n", g);
esp_rom_printf("[motor] channel=%d pwm=%d dir=%d begin\n", i, PWM_GPIO[i], DIR_GPIO[i]);
esp_rom_printf("[motor] channel=%d done\n", i);
esp_rom_printf("[motor] timer group=%d start\n", g);
esp_rom_printf("[motor] init done\n");
```

**Result:** Much better visibility. MCPWM calls and GPIO configs completed for several channels. But later the freeze itself moved into `esp_rom_printf` output — the visible freeze appeared inside the next debug print.

**Conclusion:** The original `GPIO[46] Pullup:` location was not a reliable root-cause marker. The "last printed line" is a victim of where the fault surfaces, not necessarily where it originates.

**Status:** Useful diagnostic. Later removed from normal path.

---

### Attempt 12: Move Timer Start to End

**Hypothesis:** Starting MCPWM timers before all generators/DIR pins are configured could produce output glitches causing the hang.

**Change:**
```
Before: create timer → enable/start timer → create operators/generators
After:  create all timers → create all operators/generators/DIR pins → enable/start timers
```

**Result:** Cleaner sequencing. Did not by itself solve the freeze.

**Conclusion:** Good code hygiene and safer motor startup sequence, but not the root cause.

**Status:** Kept. Motor init now starts timers only after all channels are fully configured.

---

### Attempt 13: Change DIR Away From GPIO 46/47 to GPIO 20/21

**Hypothesis:** Maybe GPIO46/47 are the only problem pins.

**Change:**
```c
PWM_GPIO[] = {5, 9, 10, 12}
DIR_GPIO[] = {26, 27, 20, 21}
```

**Result:**
```
[motor] channel=0 pwm=5 dir=26 begin → done
[motor] channel=1 pwm=9 dir=27 begin → done
[motor] channel=2 pwm=10 dir=20 begin → done
[motor] channel=3 → freeze around GPIO21
```

**Conclusion:** GPIO46 alone was not the only issue — the freeze moved to GPIO21. However, this test still used GPIO9/10/12 (audio codec pins), so the test was contaminated.

**Status:** GPIO46-only theory weakened. Test contaminated by audio pins.

---

### Attempt 14: Suppress ESP-IDF GPIO Driver Logs

**Hypothesis:** The `gpio_config` INFO log may itself be part of the visible hang. Suppress it.

**Change:**
```c
void motor_init(void) {
    esp_log_level_set("gpio", ESP_LOG_WARN);
    // ...
}
```

**Result:** The green `gpio: GPIO[...] INFO` lines disappeared from the output. Freeze still happened later.

**Conclusion:** `gpio_config` INFO logging was a symptom amplifier (making the hang look pin-specific) but not the cause. Suppressing it improved diagnostic clarity.

**Status:** Kept. GPIO INFO logs muted in normal motor init path.

---

### Attempt 15: Use GPIO 11/13 for DIR

**Hypothesis:** Avoid GPIO46/47 and GPIO20/21. Try GPIO11/13 for DIR.

**Change:**
```c
PWM_GPIO[] = {5, 9, 10, 12}
DIR_GPIO[] = {26, 27, 11, 13}
```

**Result:**
```
[motor] channel=0 pwm=5 dir=26 begin → done
[motor] channel=1 pwm=9 dir=27 begin → done
[motor] channel=2 pwm=10 dir=11 begin → done
[motor] channel=
[silence]
```

**Discovery during this attempt:** GPIO9–13 are onboard audio codec I2S pins on the Waveshare ESP32-P4-WIFI6 board:
- GPIO9 = DSDIN
- GPIO10 = LRCK
- GPIO11 = ASDOUT (codec OUTPUT — codec was driving this pin)
- GPIO12 = SCLK
- GPIO13 = MCLK

**Conclusion:** GPIO9–13 must not be used for motor PWM or DIR. GPIO11 is especially bad because the audio codec is actively outputting on it.

**Status:** Audio pin conflict confirmed. GPIO9–13 permanently banned for motor signals.

---

### Attempt 16: Avoid Audio GPIO 9–13 Completely

**Hypothesis:** The audio pins were causing the freeze. Move PWM away from GPIO9–13.

**Change:**
```c
PWM_GPIO[] = {5, 34, 35, 45}
DIR_GPIO[] = {26, 27, 46, 47}
```

**Result:**
```
[motor] channel=0 pwm=5 dir=26 begin → done
[motor] channel=1 pwm=34 dir=27 begin → done
[motor] channel=2 pwm=35 dir=46 begin → done   ← GPIO46 configured successfully here
[motor] channel=
[silence]
```

**Conclusion:** Avoiding audio pins moved the failure — GPIO46 was configured successfully in this run (contradicting the earlier theory that GPIO46 itself was always the problem). Freeze happened after channel 2, before channel 3. The visible hang was inside the next debug print, suggesting the debug printing / log path was part of the failure surface.

**Status:** Audio conflict confirmed real. Still not full boot.

---

### Attempt 17: Remove Normal esp_rom_printf Breadcrumbs

**Hypothesis:** The debug prints themselves were causing or exposing a stall.

**Change:** Removed all normal per-channel `esp_rom_printf` calls (init begin, timer new, channel begin/done, timer start, init done). Kept only `MOTOR_CHECK` output (prints only on actual ESP-IDF error).

**Result:** Local build passed. Quiet version became the current intended test path.

**Conclusion:** Normal per-channel debug printing should not be in the boot path.

**Status:** Kept. Motor init now runs silently unless a driver call fails.

---

### Attempt 18: Current Pin Mapping and Code State

At this point in the debugging, the intended motor.c mapping was:

```c
PWM_GPIO[] = {5, 34, 35, 45}
DIR_GPIO[] = {26, 27, 20, 21}
```

Motor init behavior:
- MCPWM driver
- MOTOR_CHECK on all driver calls
- GPIO driver INFO logs muted (`esp_log_level_set("gpio", ESP_LOG_WARN)`)
- Timers started after all channels configured
- No normal per-channel `esp_rom_printf` spam

This was the best available test image but the reboot loop was still occurring.

---

### Attempt 19: Git / Workflow Issues

**Issue encountered:** Pi had local edits to `firmware/main/motor.c`. `git pull` refused to overwrite them.

```
Your local changes to the following files would be overwritten by merge:
    firmware/main/motor.c
```

**Fix:**
```bash
git restore firmware/main/motor.c
git pull
```

**Result:** Pi synced. Separate error seen: `nothing added to commit / Everything up-to-date` — meaning the dev machine already had the commit. Not a firmware issue.

**Status:** Git sync issue resolved. Not related to hardware debugging.

---

### Attempt 20: Skip motor_init() entirely (BOOT_DIAG flags)

**Hypothesis:** Removing all motor initialization should let the rest of the firmware boot.

**Change:** Added `BOOT_DIAG_SKIP_MOTOR_INIT=1` preprocessor flag to skip `motor_init()` in `app_main`.

**Result:** Board still rebooted in a loop at ~290ms. Reset reason 0xc (SW_CPU_RESET).

**Conclusion:** The problem was not in `motor_init()`. It was somewhere in `app_main` itself — the logging calls, the `g_state` global, or the surrounding setup.

**Status:** Motor init ruled out as the sole cause.

---

### Attempt 21: Skip encoder_init() as Well

**Hypothesis:** Maybe `encoder_init()` is the culprit, not `motor_init()`.

**Change:** Added `BOOT_DIAG_SKIP_ENCODER_INIT=1` alongside `BOOT_DIAG_SKIP_MOTOR_INIT=1`.

**Result:** With both skipped, the board STILL rebooted in a loop at ~290ms. It printed a partial `"BOOT_DIAG_SKIP_ENCODER_INIT"` log then software-reset and repeated.

**Conclusion:** The bug was in `app_main` itself — not in any hardware init function. The ESP_LOG calls or `g_state` global were suspect.

**Status:** Both motor_init and encoder_init ruled out as the cause.

---

### Attempt 22: Bare app_main with Only esp_rom_printf

**The decisive diagnostic.** Stripped `app_main` to absolute minimum — no ESP_LOG, no `g_state`, no includes. Only `esp_rom_printf` (which writes directly via ROM, bypassing all UART/VFS/logging subsystems).

```c
void app_main(void) {
    esp_rom_printf("[diag] step1: app_main entered\n");
    esp_rom_printf("[diag] step2: reset_reason=%d\n",
                   (int)esp_reset_reason());
    for (int i = 0; i < 10; i++) {
        esp_rom_printf("[diag] step3.%d: alive at %dms\n", i,
                       (int)(xTaskGetTickCount() * portTICK_PERIOD_MS));
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    while (1) vTaskDelay(pdMS_TO_TICKS(1000));
}
```

**Result:**
```
[diag] step1: app_main entered
[diag] step2: reset_reason=1
[diag] step3.0: alive at 12ms
[diag] step3.1: alive at 512ms
[diag] step3.2: alive at 1012ms
... (all 10 heartbeats fired)
```

**reset_reason=1 = POWERON_RESET.** All 10 heartbeats fired at 500ms intervals. Idle loop held indefinitely.

**Conclusion: THE BOARD HARDWARE IS CONFIRMED HEALTHY.** The reboot loop was not a hardware fault, not a GPIO fault, not a driver fault — it was in the firmware software state (logging subsystem, `g_state`, or corrupted build artifacts).

---

### Attempt 23: Restore Full Original app_main

**Bisect step (commit 44b5ae2):** Restored the full original `app_main` — all includes, `g_state` initialization, `ESP_LOGI` calls, all hardware init calls, all `xTaskCreatePinnedToCore` calls. No skip flags. No stripped-down version. Exactly the original firmware.

**After `idf.py fullclean && idf.py build flash`:**

**Result:** "It worked perfectly."

Boot completed. All tasks started. Firmware ran as intended.

**Final conclusion:** The ~290ms reboot loop that persisted for two days was caused by corrupted firmware state accumulated during rapid flash-test-modify cycles (or build cache artifacts), not by any real hardware or driver bug. `idf.py fullclean` resolved it entirely.

---

## 13. What Was Definitively Ruled Out

These are confirmed to NOT be the root cause:

- LEDC channel 2 specifically
- LEDC channel numbering in general
- LEDC peripheral as a whole (MCPWM had identical symptoms)
- LEDC GAMMA_RAM deferred bus fault as the direct crash trigger
- I2C SDA/SCL on GPIO22/23 as the freeze cause
- Legacy I2C startup deprecation warning as a cause (it's a C-constructor, not an installed driver)
- MCPWM operator exhaustion (SOC supports 3 per group, our use is legal)
- Stale firmware / wrong binary (app versions changed with each flash)
- A simple unchecked ESP-IDF error silently ignored before the freeze
- GPIO46 as the only or primary problem pin
- GPIO22 as the only or primary problem pin
- motor_init() itself (freezes continued with it skipped)
- encoder_init() itself (freezes continued with it skipped)
- Board hardware (bare app_main with esp_rom_printf ran cleanly and indefinitely)

---

## 14. What Was Confirmed as Real Problems

These are real issues discovered during debugging (even though they were not the root cause):

- **GPIO48/49 conflict with encoder PCNT.** Using these pins for motor PWM while encoder also uses them causes silent init conflicts. GPIO48/49 are reserved for encoder channel A (FL, FR).
- **GPIO9–13 are onboard audio codec I2S pins.** They must not be used for motor signals. GPIO11 (ASDOUT) is especially dangerous — the codec actively drives it.
- **`gpio_config` INFO logs are misleading in fault diagnosis.** The apparent last-printed pin is a victim of where a deferred fault surfaces, not necessarily the cause.
- **Normal debug printing can become the visible freeze point.** Excessive `esp_rom_printf` or `ESP_LOGI` calls during init can look like the freeze is inside a print call when the actual fault is elsewhere.
- **Corrupted build artifacts accumulate from rapid flash-test-modify cycles.** `idf.py fullclean` should be run when the firmware behavior becomes unexplainable.
- **The W(278) i2c deprecation warning is cosmetic.** It runs from a C constructor before main_task and does not indicate an installed driver.

---

## 15. What Remained Suspicious (Not Fully Proven)

These remain as theories that were not conclusively confirmed or ruled out:

- **GPIO20/21** — one test moved the freeze to GPIO21, but that test also used audio codec pins, so it was contaminated.
- **GPIO46/47** — early failures centered on GPIO46, but later runs configured it successfully, and further tests showed the freeze was not pin-specific.
- **GPIO18–23 as ESP32-C6 SDIO zone** — the GPIO22/SDIO theory was reasonable but never cleanly tested in isolation.
- **External motor driver backfeed** — motor wires connected to MDD10A during debugging; never tested with drivers fully disconnected.

---

## 16. Final Combined Conclusion

The Waveshare ESP32-P4-WIFI6 is a heavily multiplexed development board. Many GPIO pins are silently tied to onboard hardware: audio codec, ESP32-C6 WiFi SDIO/control, USB OTG, camera/display, SD/MMC. This project uses enough GPIOs (8 motor + 8 encoder + 6 sensor) that it operates near the practical GPIO limit of this specific board.

The ESP32-P4 chip itself is not defective. Evidence:
- It boots cleanly
- It flashes reliably
- It runs `app_main`
- It executes MCPWM driver calls successfully
- It configures multiple GPIO channels successfully
- It ran the full original firmware after a clean rebuild

The two-day boot hang was resolved by `idf.py fullclean` + clean rebuild. The specific firmware state that triggered the ~290ms `SW_CPU_RESET` loop was never fully isolated, but its resolution proves it was a software/build artifact, not a hardware fault.

---

## 17. Lessons Learned

**1. `esp_rom_printf` is the only reliable debug output during early boot.**  
It bypasses the VFS/UART driver entirely and writes directly via ROM functions. Use it whenever the logging subsystem itself is suspect.

**2. Bisect downward first.**  
Strip `app_main` to a bare heartbeat loop before blaming hardware, drivers, or GPIO. Proving board health takes 30 minutes with this approach. Not doing it cost two days.

**3. The apparent last-printed line is a victim, not necessarily the cause.**  
A deferred bus fault from an earlier register write surfaces at the next AHB access. The visible `"GPIO Pullup:"` partial print indicated WHERE the exception delivered, not WHERE it originated.

**4. `reset_reason=0xc (SW_CPU_RESET)` with saved PC at `esp_cpu_reset` gives no useful info.**  
When you see this, add `esp_rom_printf` checkpoint prints to find where the reset is actually triggered, rather than trusting the saved PC.

**5. C-runtime constructors run BEFORE `main_task`.**  
The `W(278) i2c:` deprecation warning comes from `check_i2c_driver_conflict()` marked with `__attribute__((constructor))`. It logs before FreeRTOS ever starts. Never assume a pre-`app_main` log message indicates a driver is installed or running.

**6. `idf.py fullclean` before assuming hardware fault.**  
After many rapid build-flash-test-modify cycles, stale `.o` files and build cache artifacts can produce binaries that behave unpredictably. Run `fullclean` as the first step when behavior becomes unexplainable.

**7. GPIO safety on Waveshare ESP32-P4-WIFI6.**  
Many pins are silently reserved. Confirmed safe for motor/sensor signals: 2, 4, 5, 7, 8, 26, 27, 32, 33, 36–39, 52.  
Confirmed unsafe: 9–13 (audio codec), 18–23 (suspected C6 SDIO), 24–25 (USB), 28–31 (camera/display), 48–49 (encoder PCNT Ch A).

**8. `MOTOR_CHECK` macro using `esp_rom_printf` is the right pattern for early-init drivers.**  
`ESP_LOG` may not be safe during early init. The macro pattern:
```c
#define MOTOR_CHECK(label, expr) do {                                  \
    esp_err_t err__ = (expr);                                          \
    if (err__ != ESP_OK) {                                             \
        esp_rom_printf("[motor] %s failed: %s (%d)\n",                 \
            (label), esp_err_to_name(err__), err__);                   \
        ESP_ERROR_CHECK(err__);                                        \
    }                                                                  \
} while (0)
```
provides error visibility even when the logging subsystem is the suspected fault.

**9. Start MCPWM timers only after all channels are fully configured.**  
Starting timers mid-init while some generators/DIR pins are not yet set can produce transient output glitches. Configure everything first, then start timers.

**10. Onboard peripheral GPIO maps for Waveshare boards are under-documented.**  
Do not rely on the ESP32-P4 bare-chip GPIO capability list alone. Always validate against physical testing on the specific development board, and cross-reference the board schematic (where available) before assuming a pin is free.

---

## 18. Architecture Pivot — ESP32-P4 Scope Reduction

**Decision:** IMU (ISM330DHCX) and ToF (VL53L5CX) moved off the ESP32-P4 entirely. Both sensors will connect directly to the Raspberry Pi 5 via its SPI/I2C header.

**ESP32-P4 final scope:**
- Motor PWM (MCPWM, 4 channels)
- Encoder reading (PCNT quadrature, 4 channels)
- Wheel velocity PID (1kHz, 4 independent controllers)
- Binary serial protocol to Pi (STATE packets 100Hz, CMD_VEL reception)
- Watchdog E-stop (motors zeroed if no HEARTBEAT for 2s)

**Raspberry Pi 5 scope:**
- All sensors: LiDAR (USB), IMU (SPI), ToF (I2C)
- Full ROS2 Jazzy stack: imu_filter_madgwick, robot_localization EKF, slam_toolbox, Nav2, explore_lite
- ros2_control hardware interface (amr_hardware package, not yet written)

**Rationale:**
- Removes SPI/I2C sensor drivers from ESP32 entirely — far fewer GPIO conflicts on the Waveshare board
- Eliminates a serial transport hop: sensors go directly into ROS2 topics instead of being tunnelled through binary packets
- ESP32 stays focused on hard real-time drivetrain only

**Serial protocol simplified:**
- STATE packet dropped from 44 bytes to 26 bytes (removed accel[3] and gyro[3])
- TOF_DATA packet type removed entirely
- Wire bandwidth dropped from 6,340 B/s to 2,600 B/s (2.8% of capacity)

**Spec document updated:** `docs/superpowers/specs/2026-05-17-amr-system-design.md` reflects this split.

---

## 19. Critical Discovery — USB CDC vs USB JTAG

**Problem encountered:** First ESP-IDF motor test firmware booted correctly (logs visible in `idf.py monitor`), teleop panel opened on Pi, but motors did not respond to any commands.

**Root cause:** The Waveshare ESP32-P4-WIFI6 board has **two separate USB interfaces**:
- **USB OTG (CDC-ACM):** Connected to the USB-C physical port. Appears as `/dev/ttyACM0` on the Pi. This is what `CONFIG_ESP_CONSOLE_USB_CDC=y` routes `ESP_LOG` output to. It is also what the Arduino sketch's `Serial` uses.
- **USB JTAG Serial:** A separate hardware block. The `usb_serial_jtag_driver_install()` / `usb_serial_jtag_read_bytes()` / `usb_serial_jtag_write_bytes()` APIs target THIS interface — NOT the USB-C port the Pi sees.

**Consequence:** The original `task_serial_comms.c` called `usb_serial_jtag_driver_install()` and read from `usb_serial_jtag_read_bytes()`. Commands sent by the Pi to `/dev/ttyACM0` never reached the firmware because the firmware was listening on the wrong hardware block.

**Fix:** Replace all `usb_serial_jtag_*` calls with `fgetc(stdin)` / `fwrite(stdout)` / `fflush(stdout)`. When `CONFIG_ESP_CONSOLE_USB_CDC=y`, `stdin` and `stdout` are connected to the USB OTG CDC interface — the same `/dev/ttyACM0` the Pi communicates on.

**Rule for all future firmware on this board:**
- Pi → ESP32 communication: always use `stdin` / `stdout` (USB CDC OTG)
- Never use `usb_serial_jtag_driver_install()` for Pi communication on this board
- `ESP_LOG` shares this same interface — silence all logs with `esp_log_level_set("*", ESP_LOG_NONE)` before starting the binary protocol to prevent log output corrupting binary packets

---

## 20. ESP-IDF Motor Test Firmware — Confirmed Working

After the USB CDC vs JTAG fix, a minimal motor test firmware was flashed:
- `main.c` + `motor.c` only (no encoder, PID, or serial protocol tasks)
- Commands received via `fgetc(stdin)` — same char interface as the Arduino sketch
- SPEED = 35% duty cycle

**Result:** All 4 motors confirmed responding to all 8 commands via `teleop_simple.py --port /dev/ttyACM0`.

This confirms:
- ESP-IDF MCPWM driver works on this board
- Pi → ESP32 USB CDC communication works
- GPIO assignments are correct end-to-end in ESP-IDF (not just Arduino)

**Confirmed working GPIO map (locked):**

| Signal | GPIO | Notes |
|--------|------|-------|
| FL_PWM | 5 | MCPWM group 0 |
| FL_DIR | 26 | GPIO output |
| FR_PWM | 33 | MCPWM group 0 |
| FR_DIR | 2 | GPIO output |
| RL_PWM | 32 | MCPWM group 0 |
| RL_DIR | 27 | GPIO output |
| RR_PWM | 52 | MCPWM group 1 |
| RR_DIR | 4 | GPIO output |

---

## 21. Encoder GPIO Selection — Physical Board Constraints

**Problem:** GPIO pins 14, 15, 16, 17 (originally planned for encoder B-channels in the spec) are **not exposed** on the Waveshare ESP32-P4-WIFI6 dev board. They are internal or tied to unpopulated headers.

**Additional constraint discovered:** The original encoder B-channel assignment `{52, 2, 3, 4}` conflicted directly with motor pins:
- GPIO 52 = RR_PWM (MCPWM output)
- GPIO 2 = FR_DIR (GPIO output)
- GPIO 4 = RR_DIR (GPIO output)

**Final encoder GPIO assignment (confirmed working):**

| Wheel | A pin | B pin | Notes |
|-------|-------|-------|-------|
| FL | 48 | 46 | A: free for PCNT input; B: exposed, no conflicts |
| FR | 49 | 47 | Same |
| RL | 50 | 3 | B: GPIO3, no conflicts |
| RR | 51 | 7 | B: GPIO7 (SDA label but free — ToF moved to Pi) |

**Power:** Encoder VCC → ESP32 3.3V pin. GND → common ground rail (shared with MDD10A GNDs and battery negative). Encoder outputs are 3.3V compatible — do not use 5V supply or GPIO pins will be damaged.

**Key finding:** PCNT driver does not automatically enable GPIO pullups. Must call `gpio_pullup_en()` explicitly on both A and B pins before initialising each PCNT unit. Without this, encoder reads may stay at zero.

---

## 22. Encoder Testing — All 4 Wheels Confirmed

**Method:** Arduino sketch with interrupt-based quadrature decoding (`attachInterrupt` on A pin, read B for direction). Tested each wheel individually using the MDD10A built-in channel test button (runs each motor in both directions at full speed).

**Results:**

| Wheel | Raw sign on forward | Counts per 10ms at full speed |
|-------|--------------------|-----------------------------|
| FL | negative | ~±46 |
| FR | positive | ~±46 |
| RL | negative | ~±46 |
| RR | negative (manually tested first) | ~±46 |

**Sign convention:** FL and RL spin negative in the raw PCNT count when the robot moves forward. This is a physical mounting artefact — left-side and right-side motors face opposite directions on the frame. The firmware corrects this with:

```c
static const int SIGN[] = {-1, 1, -1, 1};  /* FL FR RL RR */
```

After correction, all four wheels report positive velocity for forward motion.

**Speed sanity check:**
- 46 counts/10ms = 4,600 counts/s
- 4,600 / 537.6 counts/rev = 8.56 rev/s = ~514 RPM output shaft at no load
- Rated loaded speed is 187 RPM — free-running unloaded is expected to be significantly higher

---

## 23. Full ESP-IDF Firmware — Motors + Encoders + Serial Protocol

**Status: Confirmed working as of 2026-05-30.**

### FreeRTOS Task Layout

| Task | Core | Priority | Rate | Function |
|------|------|----------|------|----------|
| task_encoder_read | 0 | 9 | 1kHz | PCNT quadrature read → enc_accum, omega_meas |
| task_pid_control | 0 | 10 | 1kHz | 4× velocity PID → MCPWM duty |
| task_serial_comms | 1 | 8 | 100Hz TX | Send STATE; spawn task_serial_rx |
| task_serial_rx | 1 | 7 | blocking | fgetc(stdin) → rx_queue for CMD_VEL/HEARTBEAT |

### Serial Protocol (confirmed)

```
Frame: [0xAA][0x55][TYPE][LEN][PAYLOAD][CRC16_HI][CRC16_LO]
```

| Packet | Direction | Rate | Payload | Frame size |
|--------|-----------|------|---------|-----------|
| STATE (0x02) | ESP→Pi | 100Hz | timestamp_ms(4) + enc_delta[4×int32](16) | 26 bytes |
| CMD_VEL (0x01) | Pi→ESP | on demand | 4×float32 omega rad/s | 22 bytes |
| HEARTBEAT (0x04) | Pi→ESP | 1Hz | — | 6 bytes |
| PARAM_SET (0x05) | Pi→ESP | on demand | param_id(1) + value(4) | 11 bytes |
| DIAGNOSTICS (0x06) | ESP→Pi | 1Hz | batt_mv(2) + error_flags(1) | 9 bytes |

### Encoder accumulation pattern

`enc_delta` in the STATE packet is a **cumulative count over the 10ms period**, not an instantaneous 1ms snapshot. Implementation:
- `task_encoder_read` accumulates into `g_state.enc_accum[]` using `+=`
- `task_serial_comms` snapshots `enc_accum`, copies to `sc.enc_delta`, then resets `enc_accum` to zero under mutex
- `omega_meas[]` (used by PID) uses the per-1ms instantaneous delta separately

### ESP_LOG silenced for binary protocol

`esp_log_level_set("*", ESP_LOG_NONE)` is called in `app_main` after init messages and before tasks start. This prevents log text from corrupting the binary STATE packet stream on stdout.

### Verification

STATE packets confirmed streaming at 10ms intervals (100Hz) via Python:

```python
python3 -c "
import serial, struct
s = serial.Serial('/dev/ttyACM0', 115200, timeout=2)
buf = b''
while True:
    buf += s.read(64)
    while len(buf) >= 26:
        i = buf.find(b'\xaa\x55')
        if i < 0: buf = b''; break
        buf = buf[i:]
        if len(buf) < 26: break
        pkt = buf[:26]; buf = buf[26:]
        ts,fl,fr,rl,rr = struct.unpack_from('<Iiiii', pkt, 4)
        if ts < 1000000:
            print(f'ts={ts}ms  FL={fl}  FR={fr}  RL={rl}  RR={rr}')
"
```

Encoder counts respond correctly to wheel motion with sign correction applied.

---

## 24. Updated Confirmed Working Status (as of 2026-05-30)

### Hardware

- ✅ All 4 motors: MCPWM PWM + sign-magnitude direction, all 8 commands confirmed
- ✅ All 4 encoders: PCNT quadrature, all 4 wheels respond, sign convention confirmed
- ✅ Common ground rail: ESP32 + both MDD10As + battery negative
- ✅ USB-C serial link: Pi → ESP32 CDC communication working

### Firmware (ESP-IDF, currently flashed)

- ✅ MCPWM motor driver — 20kHz, 4 channels across 2 groups
- ✅ PCNT encoder driver — quadrature 4x counting, pullups enabled, 537.6 counts/rev
- ✅ PID controller — 1kHz, 4 independent, anti-windup, Kp=2.0 Ki=5.0 Kd=0.01
- ✅ Binary serial protocol — CRC16, framed packets, STATE at 100Hz
- ✅ Watchdog E-stop — motors zeroed if no HEARTBEAT for 2 seconds
- ✅ Encoder accumulation — cumulative counts per 10ms period sent in STATE packet

### What is NOT yet done (next phases)

- ⬜ `amr_hardware` ROS2 package — ros2_control SystemInterface reading STATE packets
- ⬜ `mecanum_drive_controller` config — odometry + cmd_vel forwarding
- ⬜ IMU (ISM330DHCX) bring-up on Pi 5 SPI
- ⬜ ToF (VL53L5CX) bring-up on Pi 5 I2C
- ⬜ slam_toolbox, Nav2, explore_lite integration
- ⬜ Physical measurements: wheel_separation_x, wheel_separation_y, sensor offsets for URDF

---

## 25. Phase 3 — ROS2 Hardware Interface (2026-05-30 → 2026-05-31)

**Objective:** Build the `amr_hardware` ros2_control package, get `mecanum_drive_controller` and `joint_state_broadcaster` loaded, and achieve full closed-loop ROS2 → ESP32 → motors → encoders → ROS2.

**Outcome: FULLY ACHIEVED.** All 4 wheels run under closed-loop PID velocity control from ROS2 with encoder feedback. Motion is smooth, stops cleanly on command removal, drivers stay cool.

---

### 25.1 ROS2 Package Creation

**Files created:**
- `ros2_ws/src/amr_hardware/` — full ros2_control SystemInterface plugin
  - `serial_driver.hpp/cpp` — 26-byte STATE framing with CRC16, CMD_VEL + HEARTBEAT send, non-blocking `::read()` draining RX buffer
  - `amr_hardware_interface.hpp/cpp` — exports 4×velocity+position state interfaces and 4×velocity command interfaces; heartbeat sent inline in `write()` every 1s; encoder velocity = `enc_delta * RAD_PER_COUNT * 100.0` (RAD_PER_COUNT = 2π/537.6 from 19.2:1 gearbox)
  - `amr_hardware.xml` — pluginlib registration
- `ros2_ws/src/amr_bringup/config/controllers.yaml` — updated with full mecanum_drive_controller params
- `ros2_ws/src/amr_bringup/launch/hardware.launch.py` — minimal launch for Phase 3 testing without LiDAR/foxglove
- `ros2_ws/src/amr_bringup/launch/amr.launch.py` — full launch with sllidar_ros2 + foxglove_bridge

**Architecture decision:** Heartbeat sent inline in `write()` (same thread as CMD_VEL) rather than from a `rclcpp::TimerBase` on a separate thread. This eliminates concurrent `::write()` calls on the same file descriptor which caused packet interleaving.

---

### 25.2 Build Bugs and Fixes

#### Bug B1: ESP-IDF Python picked up by colcon CMake

**Symptom:** `colcon build` failed with `ModuleNotFoundError: No module named 'catkin_pkg'`. CMake was using `/home/miniproj/.espressif/python_env/idf5.4_py3.12_env/bin/python3` instead of the system Python.

**Root cause:** The Pi's `.bashrc` auto-sources ESP-IDF, putting its Python virtualenv first in `$PATH`. Every shell inherits this, including the colcon build shell.

**Fix:** Strip espressif from PATH before every ROS2 build:
```bash
export PATH=$(echo $PATH | tr ':' '\n' | grep -v espressif | tr '\n' ':')
```
This must be run in every terminal session before `colcon build`. The underlying fix is to NOT auto-source `get_idf` in `.bashrc` and only activate it on demand.

#### Bug B2: CMake cached wrong Python in build directory

**Symptom:** Even after fixing PATH, the build still used the ESP-IDF Python — CMake had cached the wrong interpreter from the first (failed) run.

**Fix:** Delete the build directories before rebuilding:
```bash
rm -rf ros2_ws/build/amr_hardware ros2_ws/build/amr_bringup ros2_ws/build/amr_description
```

#### Bug B3: `on_init` deprecation in Jazzy hardware_interface

**Symptom:** Build warning: `'on_init(const HardwareInfo &)' is deprecated: Use on_init(const HardwareComponentInterfaceParams & params) instead`.

**Fix:** Updated signature from `on_init(const hardware_interface::HardwareInfo & info)` to `on_init(const hardware_interface::HardwareComponentInterfaceParams & params)`. After the parent call, `info_` is populated as before.

#### Bug B4: mecanum_drive_controller parameter names wrong

**Symptom:** Controller failed to load: `Invalid value set during initialization for parameter 'front_left_wheel_command_joint_name': Parameter cannot be empty`. Our YAML used the old `front_left_wheel_name` key.

**Fix:** In Jazzy's ros2_controllers 4.39.0, the correct parameter names are:
```yaml
front_left_wheel_command_joint_name:  wheel_FL_joint
front_right_wheel_command_joint_name: wheel_FR_joint
rear_left_wheel_command_joint_name:   wheel_RL_joint
rear_right_wheel_command_joint_name:  wheel_RR_joint
front_left_wheel_state_joint_name:    wheel_FL_joint
...
```
Separate `_command_joint_name` and `_state_joint_name` for each wheel.

#### Bug B5: mecanum_drive_controller kinematics namespace

**Symptom:** Next error after B4: `Invalid value set during initialization for parameter 'kinematics.wheels_radius': value '0' must be greater than '0'`. Old flat `wheel_radius: 0.030` was ignored.

**Root cause:** Jazzy's mecanum_drive_controller uses a `kinematics` nested namespace. `wheel_separation_x`/`wheel_separation_y` no longer exist; the geometry is expressed as a single `sum_of_robot_center_projection_on_X_Y_axis` = lx + ly.

**Fix:**
```yaml
kinematics:
  wheels_radius: 0.030
  sum_of_robot_center_projection_on_X_Y_axis: 0.495  # lx(0.275) + ly(0.220)
```

#### Bug B6: odom relay wrong source topic

**Symptom:** `/odom/wheel` never appeared in topic list. The relay node was targeting `/mecanum_drive_controller/odom`.

**Root cause:** Jazzy's mecanum_drive_controller publishes on `/mecanum_drive_controller/odometry`, not `/odom`.

**Fix:** Updated relay in both launch files:
```python
arguments=['/mecanum_drive_controller/odometry', '/odom/wheel'],
```

#### Bug B7: udev rule wrong VID/PID

**Symptom:** `/dev/amr_mcu` symlink not created after applying udev rules. `lsusb` showed `1a86:55d3 QinHeng Electronics USB Single Serial`, not the Espressif `303a:1001` the rule was written for.

**Root cause:** The Waveshare ESP32-P4-WIFI6 board uses a QinHeng CH343P USB-to-UART bridge chip, not Espressif's native USB OTG. The CH343P appears as `1a86:55d3`.

**Fix:** Updated `/etc/udev/rules.d/99-amr.rules`:
```
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d3", \
  SYMLINK+="amr_mcu", MODE="0666"
```

#### Bug B8: sllidar_ros2 not in Jazzy apt

**Symptom:** `E: Unable to locate package ros-jazzy-sllidar-ros2`. The full `amr.launch.py` would crash at startup trying to find the sllidar package.

**Fix:** Cloned from source: `git clone https://github.com/Slamtec/sllidar_ros2.git`. Added `hardware.launch.py` as a minimal test launch that omits sllidar and foxglove_bridge entirely.

---

### 25.3 Critical Bug — CMD_VEL Never Reached ESP32 (Multi-Day Investigation)

This was the most complex debug of Phase 3. The ROS2 side showed `CMD_VEL FL=3.33 FR=3.33 RL=3.33 RR=3.33` in the log (confirmed correct), but wheels never moved and `fread(stdin)` on the ESP32 always returned 0 bytes.

#### Discovery sequence

**Step 1 — Confirmed Pi side is correct.** Added RCLCPP_INFO_THROTTLE to `write()` in the hardware interface. Log showed `CMD_VEL FL=3.33 FL=3.33 RL=3.33 RR=3.33` at 1Hz. The mecanum controller, command interfaces, and serial send were all working.

**Step 2 — Ruled out watchdog.** If the watchdog (no HEARTBEAT for 2s) was blocking the PID, wheels would still twitch briefly. They never moved at all.

**Step 3 — Python bypass test.** Stopped ROS2, ran a Python script directly sending HEARTBEAT + CMD_VEL to `/dev/amr_mcu`. No STATE packets returned, no wheel motion. **The ESP32 was not receiving any bytes at all.**

**Step 4 — Identified the baud rate mismatch.** Checked `firmware/sdkconfig`:
```
CONFIG_ESP_CONSOLE_UART_DEFAULT=y
CONFIG_ESP_CONSOLE_UART_BAUDRATE=115200
```
The sdkconfig.defaults had `CONFIG_ESP_CONSOLE_USB_CDC=y` but the actual sdkconfig used at build time had UART at 115200. The Pi's serial_driver.cpp was opening at `B921600`. The CH343P configured its UART to 921600 baud but the ESP32 UART0 was at 115200 — complete mismatch in both directions.

**Fix:** Changed `serial_driver.cpp` to use `B115200`:
```cpp
cfsetispeed(&tty, B115200);
cfsetospeed(&tty, B115200);
```
Also updated URDF `baud_rate` param to 115200 for consistency.

**After this fix:** STATE packets were confirmed flowing correctly (they had been garbled all along — joint_states at 100Hz was the joint_state_broadcaster publishing zeros, not actual encoder data).

**Step 5 — fread still returned 0.** After fixing baud rate, baud matched but fread(stdin) in `task_serial_comms` still returned 0 bytes on every call. Motors still did not move.

**Step 6 — Identified VFS shared lock deadlock.** In ESP-IDF, when `CONFIG_ESP_CONSOLE_UART_DEFAULT=y`, stdin and stdout both map to the same UART0 VFS device and share the same VFS-level lock. The original design had `task_serial_rx` calling `fread(stdin)` in one FreeRTOS task while `task_serial_comms` called `fwrite(stdout)` + `fflush(stdout)` at 100Hz in another task.

When `fwrite` acquired the VFS lock, `fread` blocked. When `fwrite` then waited for the UART TX FIFO to drain (blocking write), `fread` was permanently locked out. This caused `fread` to always return 0 (timeout with no data).

**Fix attempts:**
- Merged `task_serial_rx` into `task_serial_comms` (single task for all IO). `fread` still returned 0 — because in non-blocking mode `fread` on the ESP-IDF UART console returns 0 immediately when the FIFO is empty.
- Switching from `fgetc` to `fread` to read larger chunks — still 0, same problem.

**Step 7 — Root fix: bypass VFS entirely using UART driver directly.**

Added `uart_driver_install(CONFIG_ESP_CONSOLE_UART_NUM, 4096, 0, 0, NULL, 0)` in `app_main` before tasks start. This installs the ESP-IDF UART driver with a 4KB RX ring buffer, completely bypassing the VFS layer.

Changed `task_serial_comms` to use:
```c
uart_read_bytes(CONSOLE_UART, chunk, sizeof(chunk), pdMS_TO_TICKS(1));  // RX
uart_write_bytes(CONSOLE_UART, (const char *)frame, flen);               // TX
```

This eliminates the VFS lock entirely. RX and TX use separate driver queues internally.

**Result:** First test after this fix — `CMD_VEL FL=3.33 FR=3.33 RL=3.33 RR=3.33` sent and **all 4 wheels spun forward**. Phase 3 confirmed working.

---

### 25.4 Power Wiring Incident

During the first successful ROS2 motor test, smoke appeared from the MDD10A power terminals.

**Root cause:**
1. Battery wires were single-stranded (solid-core) rated for low current. The MDD10A at 100% duty with all 4 motors draws 20-40A. Solid-core wire cannot handle this — it heats, melts insulation, and can cause shorts.
2. MDD10A #2 was daisy-chained off MDD10A #1's terminals, doubling the current through one set of connectors.

**Fix:** Replaced all motor power wiring with **16AWG 3kV silicone stranded wire**. Both MDD10As now connect directly (in parallel) to the battery terminals. No daisy-chaining.

After rewiring, tested at SPEED=60 (Arduino sketch) and then under ROS2 PID control — no heating, no smoke.

---

### 25.5 PID Tuning — Overheating, Overshoot, Jerkiness

The first PID parameters (Kp=2.0, Ki=5.0, Kd=0.01, max_duty=±1.0) caused severe overheating.

#### Issue: 100% duty saturation

**Symptom:** MDD10A drivers extremely hot within 30 seconds.

**Root cause:** Kp=2.0 with setpoint=3.33 rad/s gives initial PID output = 2.0 × 3.33 = 6.66, clamped to 1.0 = 100% duty. All 4 motors at full duty = ~480W draw. The battery drained fast and the drivers reached thermal shutdown.

**Fix 1 (conservative reduction):** Kp=0.3, Ki=0.8, max_duty=±0.6.

**Result:** Motors ran but still overshot significantly. Measured velocity ~4.67 rad/s for 3.33 rad/s setpoint. Drivers warm but not overheating.

#### Issue: Integral windup — motors kept running after stop

**Symptom:** When the ROS2 command was stopped, motors continued running slowly for several seconds before stopping.

**Root cause:** During the ramp-up phase (setpoint=3.33, measured=0), the integrator accumulated a large positive value. When the setpoint dropped to 0, the integral value kept the duty positive even though the proportional term was negative. The integral had to "unwind" before the motor stopped.

**Fix:** Added a deadband and integral reset on zero setpoint:
```c
if (fabsf(sp[i]) < SP_DEADBAND) {   // SP_DEADBAND = 0.1 rad/s
    pid_reset(&s_pid[i]);
    motor_set_duty(i, 0.0f);
} else {
    motor_set_duty(i, pid_update(&s_pid[i], sp[i], meas[i]));
}
```
Also reset integral when watchdog fires.

#### Issue: Jerkiness at low speed (encoder quantization noise)

**Symptom:** At 0.05 m/s = 1.67 rad/s, motors ran in a jerky, pulsing pattern rather than smoothly.

**Root cause:** At 1.67 rad/s the encoder generates ~0.14 counts per 1ms PID cycle. Every PID iteration sees either 0 counts (omega_meas = 0 → PID drives duty up) or 1 count (omega_meas = 11.7 rad/s → PID drives duty down). The PID oscillates violently against this binary signal.

**Also:** Kp=0.05 (used in one iteration) gave initial duty = 0.05 × 1.67 = 8.35%, which is below the motor stiction threshold. Motors barely moved.

**Fix:** Two-part:
1. **IIR low-pass filter on omega_meas** in `task_encoder_read.c`:
   ```c
   #define VEL_ALPHA 0.08f
   float raw = (float)signed_delta * RAD_PER_COUNT * 1000.0f;
   s_omega_filtered[i] = VEL_ALPHA * raw + (1.0f - VEL_ALPHA) * s_omega_filtered[i];
   g_state.omega_meas[i] = s_omega_filtered[i];
   ```
   alpha=0.08 gives time constant ~11ms — smooths 0/11.7 binary noise without killing step response.

2. **Raise Kp to clear stiction:** Kp=0.12 gives initial duty ~20% for 1.67 rad/s error — sufficient to start motors without significant overshoot.

**Final PID parameters:**
```c
pid_init(&s_pid[i], 0.12f, 0.3f, 0.0f, 0.001f, -0.45f, 0.45f);
// Kp=0.12, Ki=0.3, Kd=0 (noise), max_duty=±0.45 (45%)
```

**Result:** Motors run smoothly at 0.1 m/s, velocity settles around 3.5 rad/s (within 1 encoder count of the 3.33 rad/s setpoint at this resolution), stops immediately and cleanly on command removal.

---

### 25.6 Architecture Decision — ToF Sensor Removed

**Decision:** VL53L5CX ToF sensor dropped entirely from the project.

**Rationale:** The Slamtec C1M1 R2 LiDAR (360°, 12m range, 10Hz) provides complete obstacle detection at all heights relevant to indoor AMR operation. The ToF added wiring complexity (Pi I2C header) and a custom ROS2 driver (`tof_pointcloud_node`) for marginal benefit.

**Changes made:**
- Removed `tof_link` and `tof_joint` from `ros2_ws/src/amr_description/urdf/sensors.urdf.xacro`
- No launch files referenced `/tof/points` (the node was never written)
- Future Nav2 costmaps and collision_monitor will use `/scan` (LiDAR) only

---

### 25.7 Confirmed Working Status (as of 2026-05-31)

#### Hardware

- ✅ All 4 motors: smooth closed-loop velocity control via ROS2
- ✅ All 4 encoders: PCNT quadrature feeding back to firmware PID + ROS2 joint_states
- ✅ Motor power wiring: 16AWG 3kV stranded, parallel connection to both MDD10As
- ✅ USB-C serial link: Pi ↔ ESP32 bidirectional at 115200 baud (uart_driver + uart_read_bytes)

#### Firmware (ESP-IDF, currently flashed — commit d1358a4)

- ✅ MCPWM motor driver — 20kHz, 4 channels
- ✅ PCNT encoder driver — 537.6 counts/rev, sign-corrected, IIR filtered
- ✅ PID controller — 1kHz, Kp=0.12, Ki=0.3, Kd=0, max_duty=±0.45, deadband, integral reset
- ✅ Binary serial protocol — uart_driver_install + uart_read_bytes/uart_write_bytes (bypasses VFS)
- ✅ Watchdog E-stop — motors + integral reset if no HEARTBEAT for 2s
- ✅ uart_driver_install called in app_main before tasks start (4KB RX ring buffer)

#### ROS2 Stack (Pi, ros2_ws — built from source)

- ✅ `amr_hardware` package — ros2_control SystemInterface, 115200 baud, heartbeat inline
- ✅ `joint_state_broadcaster` — publishing /joint_states at 100Hz with real encoder data
- ✅ `mecanum_drive_controller` — configured with Jazzy 4.39.0 kinematics params
- ✅ `/odom/wheel` — relayed from `/mecanum_drive_controller/odometry` via topic_tools relay
- ✅ `hardware.launch.py` — minimal test launch (no LiDAR/foxglove needed)
- ✅ `amr.launch.py` — full launch with sllidar_ros2 (cloned from source) + foxglove_bridge

#### Key serial configuration facts (do not change without re-testing)

- **Baud rate:** 115200 (sdkconfig `CONFIG_ESP_CONSOLE_UART_BAUDRATE=115200`; Pi opens at B115200)
- **Device:** `/dev/amr_mcu → ttyACM1` (QinHeng 1a86:55d3, may appear as ttyACM0 or ttyACM1 depending on plug order)
- **ESP32 UART:** UART0 via CH343P bridge (NOT USB OTG CDC; sdkconfig uses `CONFIG_ESP_CONSOLE_UART_DEFAULT=y`)
- **Serial API:** uart_driver_install + uart_read_bytes/uart_write_bytes — NOT fread/fwrite/fgetc

### What is NOT yet done (next phases)

- ⬜ Physical measurements: wheel_separation_x, wheel_separation_y, LiDAR/IMU offsets for URDF
- ⬜ slam_toolbox, Nav2 (SmacPlannerLattice + MPPI), explore_lite integration

---

## 26. Phase 4 — Sensor Fusion (2026-05-31)

**Objective:** Wire ISM330DHCX IMU to Pi 5 SPI, publish `/imu/data_raw`, configure Madgwick + EKF, verify `/odom` accumulates correctly.

**Outcome: FULLY ACHIEVED.** Full pipeline from wheels → encoders → `/odom/wheel` → EKF (fused with IMU) → `/odom` confirmed working. `/odom` accumulated ~1.03 m over 3 s at 0.3 m/s setpoint.

---

### 26.1 IMU Hardware Bring-up

**Wiring (Pi 5 40-pin header → SmartElex 9DoF breakout):**

| Breakout pin | Pi 5 pin | Signal |
|---|---|---|
| GND | Pin 20 | GND |
| 3V3 | Pin 17 | 3.3V |
| SDA | Pin 19 | SPI0 MOSI (GPIO10) |
| SCL | Pin 23 | SPI0 SCLK (GPIO11) |
| ACS | Pin 24 | SPI0 CE0 (GPIO8) — chip select |
| POCI | Pin 21 | SPI0 MISO (GPIO9) |

Enable SPI: `dtparam=spi=on` in `/boot/firmware/config.txt`.

**SPI debugging — three bugs found:**
1. `struct.unpack_from('<6h', ...)` on 6-byte buffer — must be `'<3h'` (3 × int16 = 6 bytes)
2. SPI **mode 3 gave 0x7f** (WHO_AM_I wrong) — switched to **mode 0** → got 0x6f (1 bit wrong)
3. Speed 8 MHz too fast for jumper wires — **capped at 500 kHz** (1 MHz+ causes bit errors). Confirmed by speed sweep: 100/500 kHz → OK, ≥1 MHz → WRONG.

**Final driver config:** `spi.mode = 0`, `spi.max_speed_hz = 500_000`.

**Result:** `/imu/data_raw` at exactly **100 Hz**. `linear_acceleration.y ≈ 9.56 m/s²` (gravity on Y — IMU mounted with Y up). Angular velocity near zero at rest.

---

### 26.2 New ROS2 Packages

**`amr_imu`** (ament_python):
- `imu_sensor_node` — ISM330DHCX SPI driver, 100 Hz, publishes `/imu/data_raw`
- `twist_to_reference` — converts `/cmd_vel` (Twist) → `/mecanum_drive_controller/reference` (TwistStamped)

**`amr_sensor_fusion`** (ament_cmake, config only):
- `config/imu_filter.yaml` — Madgwick, `use_mag: false`, `publish_tf: false`
- `config/ekf.yaml` — robot_localization EKF, 50 Hz, `two_d_mode: true`
- `launch/sensor_fusion.launch.py` — starts imu_filter_madgwick + ekf_node

---

### 26.3 Bugs and Fixes

#### Bug P4-1: EKF YAML type error
**Symptom:** `Sequence should be of same type. Value type 'integer'` — EKF crashed immediately.
**Root cause:** Covariance matrices had bare `0` (integer) mixed with `0.05` (float). RCL YAML parser is strict about homogeneous sequences.
**Fix:** Replace all `0` with `0.0` in both `process_noise_covariance` and `initial_estimate_covariance`.

#### Bug P4-2: mecanum_drive_controller not receiving cmd_vel
**Symptom:** `/cmd_vel` topic didn't appear in `ros2 topic list`. Publishing 30 messages had zero effect. CMD_VEL stayed 0.00.
**Root cause:** In Jazzy ros2_controllers 4.x, mecanum_drive_controller renamed its command interface from `cmd_vel` to `reference`. The topic is `/mecanum_drive_controller/reference` and the message type is **`geometry_msgs/TwistStamped`** (not `Twist`). The `use_stamped_vel` parameter does NOT exist in this version.
**Fix:** `twist_to_reference` Python node in `amr_imu` package subscribes to `/cmd_vel` (Twist) and republishes to `/mecanum_drive_controller/reference` (TwistStamped) with current timestamp. Node starts immediately (no topic-existence requirement at startup unlike `topic_tools transform`).

**Key fact for Nav2 Phase 5:** Nav2 in Jazzy also outputs TwistStamped on `/cmd_vel`. The chain will be:
`/cmd_vel (TwistStamped) → collision_monitor → /cmd_vel_safe (TwistStamped) → relay → /mecanum_drive_controller/reference`
The `twist_to_reference` bridge may be replaced by a direct relay at that point.

#### Bug P4-3: `topic_tools transform` crashes on missing input topic
**Symptom:** transform node died with `ERROR: Wrong input topic: /cmd_vel` — `/cmd_vel` has no publisher at startup.
**Root cause:** This version of `topic_tools transform` requires the input topic to already have a publisher when it starts. The bridge was moved to a proper rclpy node to avoid this.

#### Bug P4-4: `install(PROGRAMS)` in ament_cmake not reliable with `--symlink-install`
**Symptom:** `executable 'twist_to_reference.py' not found on the libexec directory`.
**Fix:** Moved `twist_to_reference.py` to `amr_imu` (ament_python), added it as a `console_scripts` entry point in `setup.py`. ament_python always installs executables correctly.

---

### 26.4 Confirmed Working Status (as of 2026-05-31)

| Topic | Hz | Source | Status |
|---|---|---|---|
| `/imu/data_raw` | 100 | ISM330DHCX SPI driver | ✅ |
| `/imu/data` | 100 | imu_filter_madgwick | ✅ |
| `/odom/wheel` | 50 | mecanum_drive_controller relay | ✅ |
| `/odom` | 50 | robot_localization EKF | ✅ |
| `/cmd_vel` → motors | — | twist_to_reference bridge | ✅ |

**Drive test result:** Published `/cmd_vel` vx=0.3 m/s for 3 s. `/odom` x accumulated from ~0.94 m to ~1.97 m (≈1.03 m, within 15% of 0.9 m target). Position held stable after stop. IMU orientation fused correctly. EKF odom→base_link TF live.

#### What is NOT yet done (Phase 5+)

- ⬜ Physical measurements: wheel_separation_x, wheel_separation_y, LiDAR/IMU offsets for URDF
- ⬜ slam_toolbox (Phase 5)
- ⬜ Nav2 SmacPlannerLattice + MPPI (Phase 6)
- ⬜ explore_lite + amr_home_manager (Phase 7)
- ⬜ amr_home_manager, foxglove Studio goal-click interface
