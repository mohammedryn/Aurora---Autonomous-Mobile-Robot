# AMR Teleoperation

Control the AMR via keyboard from a Raspberry Pi connected to the ESP32-P4 MCU over USB-C serial.

## Quick Start

1. Flash firmware from WSL2:
   ```bash
   cd firmware
   idf.py -p /dev/ttyACM0 flash
   ```

2. Install Python serial dependency on Pi:
   ```bash
   sudo apt install python3-serial
   ```

3. Run teleop script from AMR repo root:
   ```bash
   python3 teleop/teleop.py
   ```

4. Optional: override serial port if not `/dev/amr_mcu`:
   ```bash
   python3 teleop/teleop.py --port /dev/ttyUSB0
   ```

## Controls

| Key | Action |
|-----|--------|
| ↑ | Forward |
| ↓ | Backward |
| ← | Strafe left |
| → | Strafe right |
| Q | Rotate CCW |
| E | Rotate CW |
| Space | Stop |
| Esc | Quit |

## Tuning

Speed and rotation parameters are at the top of `teleop.py`: adjust `SPEED`, `ROT`, and `WHEELBASE` as needed. The `WHEELBASE` constant (in meters) must match your physical frame geometry—it is the sum of the center-to-center front/rear axle distance and the center-to-center left/right wheel distance. The default 0.30 m is a starting estimate; measure and update for your build.

## Safety

The ESP32 watchdog stops motors if no heartbeat is received for 2 seconds. The script sends a heartbeat every 1 second. Do not add artificial delays to the control loop.

## Troubleshooting

**Port not found:** Verify the udev rule is applied on the Pi with:
```bash
ls /dev/amr_mcu
```
If missing, reload udev:
```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
```

**Robot moves wrong direction:** Invert the `vy` sign in the strafe entries of `KEY_MAP` in `teleop.py`.

**No serial data:** Confirm the baud rate matches firmware (921600 baud) and the USB cable is data-capable, not power-only.

## Hardware

- **MCU:** ESP32-P4 connected to Pi via USB-C serial
- **Baud rate:** 921600
- **udev symlink:** `/dev/amr_mcu` (set via `/etc/udev/rules.d/99-amr.rules`)
