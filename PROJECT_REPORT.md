# Project Report: Aurora — Autonomous Mecanum-Wheel Mobile Robot

**Author:** Mohammed Rayan
**Domain:** Embedded Systems, Robotics, ROS 2 Navigation
**Stack:** ESP32-P4 (ESP-IDF / FreeRTOS, C) · Raspberry Pi 5 (ROS 2 Jazzy, Python/C++)
**Repository:** `mohammedryn/AMR`

---

## Abstract

Aurora is a ground-up autonomous mobile robot built on a four-wheel mecanum
chassis with a split-compute architecture: a real-time **ESP32-P4** drivetrain
microcontroller and a **Raspberry Pi 5** running the full **ROS 2 Jazzy**
navigation stack. The system was designed, wired, programmed, and debugged
entirely from first principles — firmware, sensor fusion, SLAM, path planning,
and autonomous exploration — with no pre-built robot platform or simulation
shortcuts. The finished system powers on in an unfamiliar room, autonomously
explores and maps the entire reachable area using LiDAR-based SLAM and
frontier exploration, avoids obstacles in real time, recovers from stalls and
dead ends, and can retrace its own path home on command. The mission-control
logic was developed test-first and is backed by a 58-test unit suite (58/58
passing) validated against a mocked ROS 2 environment before being deployed
and confirmed on real hardware.

This report documents the system architecture, hardware design, firmware
implementation, ROS 2 software stack, the testing methodology, and — in
detail — the major engineering problems encountered and how each was
diagnosed and resolved. Several of these bugs are non-obvious and instructive:
a 2× odometry scaling error that masqueraded as a dozen unrelated navigation
problems, an IMU signal-integrity issue traceable to jumper-wire inductance,
and a three-layer "premature stop" bug in the autonomous exploration pipeline
where each fix revealed the next hidden failure underneath it.

---

## 1. Introduction

### 1.1 Problem Statement

Build a wheeled robot that can be placed in an unknown indoor space, powered
on, and — without any human input — explore the entire reachable area while
avoiding obstacles, build an occupancy-grid map of the space, save that map to
disk, and return to its starting location. The robot should also accept
ad-hoc navigation goals from an operator (click-to-navigate) once a map
exists.

### 1.2 Objectives

1. Design and wire a 4-wheel mecanum platform with a dedicated real-time
   motor/encoder controller.
2. Implement closed-loop wheel velocity control on the microcontroller.
3. Bridge the microcontroller to ROS 2 via `ros2_control` and a mecanum drive
   kinematic model.
4. Fuse wheel odometry and IMU data into a stable 50 Hz localization estimate.
5. Run live SLAM (`slam_toolbox`) to build an occupancy grid from LiDAR data.
6. Use Nav2 (planner + controller + costmaps + collision monitor) for safe,
   obstacle-aware motion.
7. Drive autonomous frontier exploration (`explore_lite`) under a custom
   mission-control state machine that is resilient to stalls, dead frontiers,
   and operator commands (`stop`, `go_home`, `resume`).
8. Validate the mission-control logic with a thorough automated test suite,
   then validate live on hardware.

### 1.3 Scope and Constraints

- **Holonomic hardware, differential-drive motion model.** The chassis is
  mechanically holonomic (mecanum wheels can strafe and rotate in place), but
  Nav2 is configured with a `DiffDrive` motion model. This was a deliberate,
  validated trade-off after a holonomic (`Omni`) configuration produced
  oscillating, fighting behavior between the MPPI controller's critics — see
  Section 7.5.
- **Compute budget.** The Raspberry Pi 5 must run LiDAR processing, IMU
  fusion, SLAM, Nav2's planner and controller, frontier exploration, and
  mission control simultaneously, at a real 20 Hz control rate. Several design
  decisions (Section 7.5, 7.6) exist specifically to fit this CPU budget.
- **No simulation shortcuts for final validation.** A Gazebo Harmonic
  simulation exists for fast iteration, but every major subsystem was also
  validated on the physical robot — encoder scale, IMU calibration, SLAM map
  quality, and the full autonomous exploration pipeline.

---

## 2. System Architecture

The system is organized into three layers:

| Layer | Hardware | Responsibility |
|---|---|---|
| **Firmware (real-time)** | ESP32-P4 (ESP-IDF, FreeRTOS) | Encoder capture, wheel velocity PID, motor PWM, serial protocol, hardware watchdog |
| **Compute (ROS 2)** | Raspberry Pi 5, Ubuntu 24.04, ROS 2 Jazzy | Sensor drivers, localization, SLAM, navigation, exploration, mission control |
| **Development** | WSL2 Ubuntu 22.04 | Firmware builds, ROS 2 workspace edits, Docker simulation, deployment |

### 2.1 High-Level Data Flow

```
LiDAR (Slamtec C1M1) ──► /scan ──► slam_toolbox ──► /map ──► explore_lite ──► goal
                                │                              │
IMU (ISM330DHCX, SPI) ──► imu_filter_madgwick ──► /imu/data ──┤
                                                               ▼
ESP32 Encoders ──► amr_hardware ──► mecanum_drive_controller ──► /odom/wheel ──► EKF (50Hz) ──► /odom
                                                                                         │
                                                                                         ▼
                                                          Nav2 (SmacPlanner2D + MPPI DiffDrive)
                                                                                         │
                                                                                         ▼
                                            nav2_collision_monitor ──► cmd_vel_safe_relay ──► mecanum_drive_controller
                                                                                         │
                                                                                         ▼
                                                              amr_hardware ──► ESP32 (CMD_VEL @ 115200 baud)
```

### 2.2 The "Split Brain" Rationale

The Raspberry Pi 5, despite being a capable single-board computer, does not
provide hard real-time guarantees under Linux. Motor PWM generation and
quadrature encoder counting are timing-critical at the microsecond scale and
cannot tolerate scheduling jitter from a general-purpose OS. The ESP32-P4
therefore owns everything timing-critical — encoders, PID, motor output, and
a hardware watchdog — while the Pi owns everything that benefits from a rich
software ecosystem: ROS 2 drivers, `tf2`, SLAM, and Nav2.

The original design also placed the IMU and a time-of-flight (ToF) sensor on
the ESP32. Both were relocated during bring-up: the IMU now talks to the Pi
directly over SPI0 (ROS-native IMU drivers and filters live on the Pi anyway),
and the ToF sensor was dropped entirely once LiDAR proved sufficient for all
obstacle detection. This kept the firmware's scope narrow and testable.

---

## 3. Hardware Design

| Component | Model | Key Specs |
|---|---|---|
| Frame | Aluminium extrusion | 71.5 × 56.5 cm, ~8 kg |
| Wheels | 4× mecanum, 60 mm | Wheel radius 0.030 m, X-pattern roller layout |
| Motors | 4× PGM45775-19.2K | 12 V, 19.2:1 gearbox, 7 PPR quadrature encoder on motor shaft |
| Motor Drivers | 2× Cytron MDD10A | Dual-channel, sign-magnitude PWM, 10 A/channel continuous |
| Drivetrain MCU | ESP32-P4 (Waveshare ESP32-P4-WIFI6) | FreeRTOS, ESP-IDF v5.3.x |
| Main Compute | Raspberry Pi 5, 8 GB | Ubuntu 24.04, ROS 2 Jazzy |
| LiDAR | Slamtec C1M1 R2 | USB, 460800 baud, 360°, 10 Hz, DenseBoost mode |
| IMU | ISM330DHCX | 6-DoF accel + gyro, SPI0 @ 500 kHz |
| Main Battery | 3S3P LiPo | 11.1 V nominal / 12.8 V max, ~7800 mAh — powers motors + ESP32 logic |
| Compute Battery | 4S 1200 mAh → buck converter | 5.12 V / 5.1 A — powers Pi 5 (and ESP32 via USB) |

### 3.1 Encoder Resolution

The encoder math is the single most consequential number in the entire
system, because every downstream estimate — odometry, SLAM, exploration —
depends on it:

```
7 PPR (motor shaft) × 2 (X2 quadrature, as counted by ESP32 PCNT) × 19.2 (gear ratio)
  = 268.8 counts per output-shaft revolution

RAD_PER_COUNT = 2π / 268.8
```

The firmware's PCNT peripheral counts in **X2 mode**, not the originally
assumed X4 mode — a distinction that, when wrong, silently halves every
odometry measurement (see Section 7.2 for the full debugging story).

### 3.2 Mecanum Geometry (Measured)

| Parameter | Value | Used in |
|---|---|---|
| `wheel_separation_x` (front↔rear axle) | 0.462 m | `amr.urdf.xacro` (`lx = 0.231`) |
| `wheel_separation_y` (left↔right track) | 0.510 m | `amr.urdf.xacro` (`ly = 0.255`) |
| Sum of center projections (lx + ly) | 0.486 | `controllers.yaml` |
| Wheel radius | 0.030 m | `controllers.yaml` |
| Wheel velocity limit | ±37.6 rad/s (≈ ±1.13 m/s rim speed) | `amr.urdf.xacro` |

These values were measured directly off the assembled chassis rather than
taken from a spec sheet — the difference between a placeholder and a measured
value is the difference between the robot driving straight and driving in a
curve.

### 3.3 Power Architecture

The motor drivers and ESP32 logic share a common ground with the main 3S3P
LiPo battery. The Raspberry Pi 5 is powered from an independent 4S 1200 mAh
pack through a 5.12 V / 5.1 A buck converter, and in turn powers the ESP32
over USB (5 V power + serial data on the same cable). Early bring-up surfaced
that isolated grounds between the ESP32 and the motor drivers produced
undefined logic levels on the PWM/DIR lines — all grounds were tied to the
main battery's negative terminal to resolve this.

### 3.4 GPIO Map (Physically Verified)

The Waveshare ESP32-P4-WIFI6 board has several GPIO ranges that are internally
reserved (an onboard audio codec on GPIO 9–13, an internal SDIO link to the
ESP32-C6 WiFi co-processor on GPIO 18–23, USB OTG D+/D- on GPIO 24–25, and a
camera/display zone on GPIO 28–31), none of which is documented up front. The
final, physically-verified pin map:

| Wheel | PWM GPIO | DIR GPIO | Encoder A | Encoder B |
|---|---|---|---|---|
| FL | 5  | 26 | 48 | 46 |
| FR | 33 | 2  | 49 | 47 |
| RL | 32 | 27 | 50 | 3  |
| RR | 52 | 4  | 51 | 7  |

---

## 4. Firmware Architecture (ESP32-P4)

The firmware is deliberately narrow in scope: encoder capture, wheel velocity
PID, the serial protocol, and a hardware watchdog. It runs three FreeRTOS
tasks:

| Task | Core | Priority | Rate | Responsibility |
|---|---|---|---|---|
| `task_encoder_read` | 0 | 9 | 1 kHz | Atomically snapshot 4× PCNT quadrature counters |
| `task_pid_control` | 0 | 10 | 1 kHz | 4× independent wheel velocity PID loops → MCPWM duty |
| `task_serial_comms` | 1 | 8 | 100 Hz | Parse RX commands, transmit STATE packets, run the watchdog |

The two real-time tasks are pinned to Core 0 and never block on I/O. Shared
state between tasks is a mutex-protected struct (`shared_state.h`).

### 4.1 Serial Protocol

All communication uses a single framed packet format validated with a CRC16
checksum over the type, length, and payload:

```
[0xAA][0x55][TYPE:1][LEN:1][PAYLOAD:LEN bytes][CRC16_HI][CRC16_LO]
```

| Type | Code | Direction | Payload |
|---|---|---|---|
| `CMD_VEL` | `0x01` | Host → MCU | 4× `float32` wheel angular velocities (rad/s), FL/FR/RL/RR |
| `STATE` | `0x02` | MCU → Host, 100 Hz | `timestamp_ms` (u32) + 4× `int32` encoder deltas |
| `HEARTBEAT` | `0x04` | Host → MCU | — |
| `PARAM_SET` | `0x05` | reserved | `param_id` (u8) + `value` (f32) |
| `DIAGNOSTICS` | `0x06` | reserved | `batt_mv` (u16) + `error_flags` (u8) |

Transport is USB CDC-ACM at 115200 baud. The driver bypasses the ESP-IDF VFS
layer entirely (`uart_driver_install` + `uart_read_bytes`/`uart_write_bytes`)
— using `fread(stdin)`/`fwrite(stdout)` concurrently on the console UART
deadlocks.

**Watchdog:** if no `HEARTBEAT` arrives within 2 seconds, the firmware zeroes
all wheel setpoints, resets the PID integrators, stops every motor, and sets
an error flag — entirely independent of the ROS 2 side's health.

### 4.2 Wheel Velocity PID

Each of the four wheels runs an independent PID loop at 1 kHz:

```c
pid_init(&s_pid[i], /*Kp=*/0.12f, /*Ki=*/0.3f, /*Kd=*/0.0f,
         /*dt=*/0.001f, /*out_min=*/-0.45f, /*out_max=*/0.45f);
```

- **Kp = 0.12** clears static friction reliably without saturating the driver.
- **Ki = 0.3** compensates for load variation; the integrator is clamped
  (anti-windup) whenever the output saturates.
- **Kd = 0** — the measured angular velocity signal is too noisy at 1 kHz for
  a useful derivative term.
- **Output clamp ±0.45** (45% PWM duty) keeps the drivers within a safe
  sustained-operation range.
- A **0.1 rad/s setpoint deadband** commands the wheel to zero below this
  threshold, preventing drift and unnecessary heating.

---

## 5. ROS 2 Software Architecture (Jazzy Jalisco)

The ROS 2 workspace is organized into nine packages, each with a single
responsibility:

| Package | Description |
|---|---|
| `amr_description` | URDF/xacro robot model, sensor frames, Gazebo simulation plugins |
| `amr_hardware` | `ros2_control` `SystemInterface` — serial bridge to the ESP32-P4 |
| `amr_imu` | ISM330DHCX SPI driver + `cmd_vel_safe_relay` / `twist_to_reference` bridge nodes |
| `amr_sensor_fusion` | `imu_filter_madgwick` + `robot_localization` EKF → `/odom` |
| `amr_slam` | `slam_toolbox` online-async mapping configuration |
| `amr_nav` | Nav2: `SmacPlanner2D` planner, MPPI controller, costmaps, collision monitor |
| `amr_explore` | `explore_lite` (m-explore-ros2) frontier-exploration configuration |
| `amr_home_manager` | Mission-control state machine: explore → stall watchdog → save → return home |
| `amr_bringup` | Top-level launch files and shared runtime configuration |

The sim/real boundary is a single xacro switch (`gz_ros2_control` vs.
`amr_hardware/AMRHardwareInterface`) — everything above `ros2_control`
(Nav2, SLAM, EKF, exploration, mission control) runs identically in
simulation and on hardware.

### 5.1 Sensor Fusion and Localization

`robot_localization`'s EKF fuses two sources at 50 Hz:

- **`/odom/wheel`** — position only (x, y, vx, vy). Yaw and angular velocity
  from wheel odometry are **disabled**, because mecanum roller slip makes
  wheel-derived heading too noisy to trust.
- **`/imu/data`** — yaw and angular velocity only, from the
  Madgwick-filtered, gyro-bias-calibrated IMU.

This "IMU is the sole yaw source" split was the fix for a map-smearing bug
where the two yaw estimates fought each other (Section 7.3).

The EKF publishes `odom → base_footprint`, leaving `robot_state_publisher`'s
static `base_footprint → base_link` chain intact — resolving a TF tree
conflict where `base_link` previously had two competing parents.

### 5.2 SLAM

`slam_toolbox` runs in permanent **online_async mapping mode** (it never
transitions to a localization-only mode, so the map stays live for the entire
session). It is tuned for small indoor rooms: `resolution: 0.05`,
`minimum_travel_distance: 0.1`, `minimum_travel_heading: 0.1`,
`map_update_interval: 1.0`, with `do_loop_closing: true` using the Ceres
solver.

### 5.3 Navigation (Nav2)

| Component | Choice | Rationale |
|---|---|---|
| Global planner | `SmacPlanner2D` | Plain 8-connected grid search with zero kinematic assumptions — always trackable by the controller |
| Local controller | MPPI, `motion_model: DiffDrive` | Predictable rotate-then-drive-straight motion; lateral velocity zeroed |
| Costmap inflation | 0.45 m, `cost_scaling_factor: 3.5` | Tuned for small-room navigation |
| `robot_radius` | 0.30 m (virtual circle) | Avoids false "start occupied" states vs. a full footprint polygon |
| Safety layer | `nav2_collision_monitor` (`FootprintApproach`, raw `/scan`) | Runs independently of the costmap update cycle |
| Lifecycle | `bond_timeout: 0.0`, `autostart: True` | The Pi 5's startup CPU load caused false bond failures with the 4 s default |

### 5.4 Autonomous Exploration and Mission Control

`explore_lite` (m-explore-ros2) performs frontier selection and starts itself
autonomously the moment it launches — it does not wait for a command.
`amr_home_manager` is a Python state machine
(`IDLE → EXPLORING ⇄ STUCK`, plus `RETURNING_HOME` and `RESUMING`) that
mirrors `explore_lite`'s behavior by listening to its `/explore/status`
announcements:

- **`IDLE → EXPLORING`** is triggered by `explore_lite`'s own
  `EXPLORATION_STARTED` / `EXPLORATION_IN_PROGRESS` status, or by
  `/amr/command = "explore"`. The current `/odom` pose is recorded as "home."
- **Persistent retry on "no frontiers found."** Rather than giving up,
  `home_manager` waits 2 seconds (debounce) and republishes
  `/explore/resume = True`. The debounce prevents a busy-loop against
  `explore_lite`'s synchronous `makePlan()` when the costmap hasn't changed
  (e.g., the robot is stopped against a wall).
- **Continuous path recording.** While `EXPLORING` or `STUCK`, the robot
  samples `(x, y, yaw)` from `/odom` every 0.5 m into an in-memory path.
- **`STUCK` state.** A stall watchdog (`|vx|, |vy|, |wz| < 0.02` for 15 s)
  nudges `/explore/resume`. After 15 consecutive nudges with no motion, the
  state escalates to `STUCK` and the operator is prompted to send `stop` or
  `go_home`.
- **Operator commands (`/amr/command`):**
  - `stop` — saves the recorded path (`<map_save_path>_path.json`) and calls
    `slam_toolbox/save_map`, then transitions to `IDLE`.
  - `go_home` — saves progress, then retraces the recorded path **in reverse**
    via sequential `NavigateToPose` waypoints back to the home pose.
  - `resume` — pauses exploration, retraces the path **forward** from home to
    the saved breakpoint, resets stall-watchdog bookkeeping, then resumes
    exploring.
- **Reentrancy guards** make repeated `go_home`/`resume` calls during an
  active retrace safe no-ops rather than errors.

---

## 6. Testing and Validation Methodology

### 6.1 Unit Testing (Test-Driven Development)

The mission-control state machine (`amr_home_manager`) was built test-first
against a mocked `rclpy`/ROS 2 environment, with **58/58 tests passing**.
Coverage includes:

- State transitions (`IDLE ↔ EXPLORING ↔ STUCK ↔ RETURNING_HOME ↔ RESUMING`)
- The 2-second no-frontiers debounce and busy-loop prevention
- Stall-watchdog nudge counting and `STUCK` escalation after 15 nudges
- Path recording at 0.5 m intervals and JSON serialization
- `go_home` / `resume` waypoint generation, including the ≤1-point fallback
- Reentrancy guards for overlapping commands
- `~`-expansion of `map_save_path` before any file I/O (a regression test
  added after a live bug — see Section 7.7)

Tests are run locally with:
```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest ros2_ws/src/amr_home_manager -q
```

### 6.2 Hardware Validation

- **Encoder/odometry calibration:** a motor-driven push of 0.5 m reported
  ~0.51 m via `/odom` after the encoder-scale fix (Section 7.2). Hand-pushing
  the robot was explicitly ruled invalid for this test, since mecanum rollers
  skid sideways under lateral hand pressure.
- **IMU SPI signal integrity:** a dedicated diagnostic tool
  (`tools/imu_spi_check.py`) ran a speed ladder (50/100/200/500/1000 kHz)
  reading the IMU's `WHO_AM_I` register 30 times per speed, used to validate
  both the original jumper-wire connection and the later soldered connection.
- **Full-stack live run:** `ros2 launch amr_bringup explore_map.launch.py` on
  the Pi — global costmap grew from 203×217 to 391×315 cells
  (~19.5 m × 15.75 m at 5 cm/pixel) during a single autonomous exploration
  run, with the robot driving through multiple frontier goals without manual
  intervention.
- **`stop` command:** validated live — transitions to `IDLE`, saves both the
  occupancy-grid map and the recorded-path JSON to disk.
- **Debounce fix:** validated live — eliminated a ~20 ms busy-loop that
  previously occurred when the robot stopped against a wall.

---

## 7. Engineering Challenges and Debugging Case Studies

Several bugs in this project were notable not for their fix (often a single
line) but for how indirect and misleading their symptoms were. Each is
documented in full chronological detail across 33 sections of
`docs/project_log.md`; the most instructive are summarized here.

### 7.1 The Two-Day Silent Boot Hang

Before any ROS 2 code existed, early ESP-IDF firmware would silently hang
mid-boot with no panic, no backtrace, and no watchdog message — just a frozen
UART. The Waveshare ESP32-P4-WIFI6 board has several undocumented reserved
GPIO ranges (an onboard audio codec, an internal SDIO link to its ESP32-C6
WiFi co-processor, USB OTG lines, and a camera/display zone), which made
GPIO conflicts the obvious first suspect.

Two days of investigation followed: LEDC PWM channel reconfiguration, a full
rewrite from LEDC to MCPWM, chasing an apparent I2C driver "conflict" that
turned out to be a harmless constructor-time log message, and finally a
bare-bones `app_main()` using only `esp_rom_printf` to prove the board itself
was healthy. The actual root cause was neither GPIO nor silicon: a clean
`idf.py fullclean` and rebuild fixed it outright. The firmware build state had
become corrupted from rapid flash-test-modify cycles — a stale build artifact
had been masquerading as a hardware-level mystery.

**Lesson:** when a board behaves impossibly and every targeted fix fails, wipe
the build before going deeper into hardware theories.

### 7.2 The 2× Under-Scaled Odometry Bug

This single bug, in retrospect, explains nearly every "unrelated" navigation
problem the robot exhibited for weeks.

The encoder math looked correct on paper:

```
7 PPR × 4 (assumed X4 quadrature) × 19.2 gear ratio = 537.6 counts/output rev
RAD_PER_COUNT = 2π / 537.6 = 0.01169 rad/count
```

But the ESP32's PCNT peripheral was actually counting in **X2 mode**, not X4
— the real figure was **268.8 counts/rev**, exactly half. Every wheel-distance
measurement reaching the EKF was therefore halved: pushing the robot 0.5 m
produced an `/odom/wheel` reading of roughly 0.25 m.

The downstream effects looked like a collection of unrelated problems:

- SLAM fought the LiDAR scans because odometry disagreed with reality, and the
  map smeared and ballooned.
- `explore_lite` chased frontiers that did not actually exist.
- A dozen costmap "band-aid" tuning passes were applied to symptoms, none of
  which addressed the actual cause.

The fix was a one-constant change — `RAD_PER_COUNT = 2π / 268.8` — confirmed
by a hand-push test showing 0.514 m measured for a 0.5 m motor-driven push.

### 7.3 IMU Signal Integrity, Yaw Fusion, and Gyro Bias

Once odometry was corrected, three further localization issues surfaced in
sequence:

1. **SPI signal integrity.** The ISM330DHCX intermittently returned garbage
   `WHO_AM_I` values at 500 kHz over jumper wires — a direct test showed
   18/30 good reads at 500 kHz versus 30/30 at 100 kHz. Dropping to 100 kHz
   fixed it immediately (the IMU only needs ~100 Hz of data, so bandwidth was
   never the constraint — signal quality was). After later soldering the
   jumpers directly to the PCB, the bus ran cleanly at 1 MHz (30/30 reads),
   and was set to a conservative 500 kHz for thermal/vibration headroom.

2. **EKF yaw fight.** With both wheel odometry and the IMU feeding yaw into
   the EKF, the fused heading jittered, visible in SLAM as a radial
   "star/fan" smear pattern. Because mecanum wheels slip on their rollers,
   wheel-derived yaw is fundamentally unreliable. The fix: disable yaw and
   angular-velocity fusion from wheel odometry entirely, making the IMU the
   sole yaw source — standard practice for mecanum platforms.

3. **Gyro bias drift.** Even with the IMU as the sole yaw source, a small but
   persistent ~0.0077 rad/s Z-axis gyro bias caused slow heading drift. This
   was initially misdiagnosed as a Madgwick `zeta` tuning problem — it was
   not, because in 6-DoF mode without a magnetometer, the Madgwick filter is
   *structurally incapable* of observing yaw bias (it has no yaw-referenced
   measurement to correct against). The real fix was a boot-time calibration
   routine: average 200 raw gyro samples (~2 s) at rest, compute the per-axis
   bias, and subtract it from every subsequent reading before it reaches
   either Madgwick or the EKF.

A fourth, related issue was a **TF tree conflict**: `robot_state_publisher`
broadcast a static `base_footprint → base_link` chain from the URDF, while the
EKF was *also* configured to publish `odom → base_link` — giving `base_link`
two parents, which `tf2` cannot resolve. The fix set the EKF's
`base_link_frame` to `base_footprint`, so the EKF publishes
`odom → base_footprint` and the URDF's static chain extends cleanly from
there.

The end state: `/odom` at a clean 50 Hz, built on correctly-scaled wheel
encoders and a calibrated, signal-clean IMU serving as the single source of
truth for yaw.

### 7.4 LiDAR Mounting and TF Offsets

The LiDAR's TF offset relative to `base_link` (x = 0.327 m, the center of the
scan disc) had to be measured directly off the mounted hardware. In a later
session, the LiDAR was discovered to be physically mounted **180° backwards**,
requiring a `rpy` correction in the URDF. Once both the offset and orientation
were correct, RViz2 showed a coherent room map with clean, straight wall
lines.

### 7.5 Teaching a Holonomic Robot to Plan with Nav2

Several Nav2 configuration issues were specific to the mecanum platform:

- A missing `inflation_layer` on the global costmap caused `planner_server`
  to spam warnings on every planning cycle and degraded path quality — the
  costmap "worked" without it, just badly, which made the missing layer easy
  to overlook.
- An initial configuration combined `SmacPlannerLattice` (which assumes
  holonomic motion primitives) with a `DiffDrive` controller, producing
  geometrically valid paths the controller could not track ("no valid path
  found" failures). Switching the global planner to `SmacPlanner2D` — a plain
  8-connected grid search with no kinematic assumptions — fixed this, since
  any path it produces is always trackable.
- An `Omni` MPPI motion model was tried to use the mecanum drive's full
  strafing capability, but its `PathAngleCritic` (which penalizes
  non-forward orientation) directly fought the `Omni` model's encouragement of
  lateral strafing, producing oscillating, diagonal motion. `PathAngleCritic`
  is only coherent with a `DiffDrive` motion model, which was adopted as the
  final configuration — a deliberate trade-off that sacrifices strafing for
  predictable, trackable motion.

### 7.6 Fitting MPPI Into the Pi 5's CPU Budget

MPPI is computationally expensive, and a 20 Hz control loop allows only 50 ms
per cycle. The initial configuration (`batch_size=2000`, `time_steps=56`) cost
62–125 ms per cycle, dropping the effective control rate to 8–16 Hz and
producing visible lurching and jerking — independent of any localization
issue.

The fix was a deliberate trade: `batch_size` 2000 → 1000, `time_steps`
56 → 40, `model_dt` 0.05 → 0.07. The new cost (1000 × 40 = 40,000
sample-steps) is about 0.36× the original load — comfortably inside the 20 Hz
budget — while preserving the same 2.8 s planning horizon (40 × 0.07 s). The
per-step displacement at the new `model_dt` (0.10 m/s × 0.07 s = 0.007 m)
remained well under the 0.05 m costmap cell size, so no collision-check
resolution was lost in the trade.

### 7.7 The Premature-Stop Saga: Three Root Causes, Stacked

The single most time-consuming debugging effort of the project was a symptom
that looked simple — "the robot explores a little and then just stops" — but
had **three independent root causes**, each one masking the next.

**Root cause 1 — wrong global planner** (Section 7.5): `SmacPlannerLattice` +
`DiffDrive` produced unreachable paths, so the robot stopped almost
immediately. Fixed by switching to `SmacPlanner2D`.

**Root cause 2 — a costmap QoS race `explore_lite` never recovered from.**
`nav2_costmap_2d`'s `Costmap2DPublisher` sends a full occupancy grid on
activation using **TRANSIENT_LOCAL** QoS, then switches to incremental
`costmap_updates` only. `explore_lite`'s costmap subscriber uses the default
**VOLATILE** QoS. In the staggered launch sequence, the global costmap
activates around T+19 s but `explore_lite` doesn't start until T+23 s — by the
time its VOLATILE subscription joins, the one full-grid message is long gone,
and TRANSIENT_LOCAL replay does not help VOLATILE subscribers. The result:
`explore_lite` sat forever on "Waiting for costmap to become available," and
**every prior "autonomous explore" run had silently been doing nothing**. The
fix was one parameter: `always_send_full_costmap: true` on the global
costmap, forcing a full-grid republish every cycle regardless of subscriber
timing.

**Root cause 3 — the mission-control state machine was never actually
"exploring."** Even after the costmap fix got the robot moving,
`amr_home_manager`'s stall watchdog and recovery logic never fired. The
reason: `explore_lite` starts itself autonomously on launch — it does not wait
for a command — but `explore_map.launch.py` never published
`/amr/command = "explore"`, so `home_manager`'s internal state remained `IDLE`
forever. Every piece of `EXPLORING`-gated recovery logic, including everything
meant to recover from "no frontiers found," had been completely inert on every
previous run, even though the robot *appeared* to be exploring autonomously
(because `explore_lite` genuinely was). The fix: `home_manager` now listens to
`/explore/status` directly and transitions `IDLE → EXPLORING` upon seeing
`EXPLORATION_STARTED` or `EXPLORATION_IN_PROGRESS` — mirroring
`explore_lite`'s actual behavior instead of waiting for a command that would
never arrive.

With all three fixes in place, a full run completed end-to-end: the global
costmap grew from 203×217 to 391×315 cells (~19.5 m × 15.75 m at 5 cm/pixel),
and a recovery spin (a full 360° Nav2 `Spin` action) correctly fired when the
first frontier search came up empty.

### 7.8 From Recovery Spins to a Resilient State Machine

The recovery-spin design worked but had a hard ceiling: after two spins with
no new frontiers, the robot permanently gave up and declared itself "fully
explored," even with most of the room unmapped. Live testing also surfaced a
sharper problem: when the robot drove into a wall, `explore_lite`'s
synchronous `makePlan()` inside `resume()` would immediately re-evaluate the
same unchanged costmap, immediately report "no frontiers" again, and
`home_manager` would immediately re-nudge it — a tight ~20 ms busy-loop that
spammed logs and burned CPU on a Pi with a documented brownout history under
load.

The redesign (Section 5.4) replaced "give up after N spins" with persistent
retry behind a 2-second debounce, a `STUCK` escalation path after 15 failed
stall-watchdog nudges, continuous path recording, and a real operator
vocabulary (`stop` / `go_home` / `resume`) with reentrancy guards. This was
built test-first (58/58 tests passing against a mocked ROS 2 environment),
then validated live — which immediately caught one more real bug: a launch
file passed the literal string `~/AMR/maps/explore_map` (with an unexpanded
`~`) as the map-save path, and Python's `open()` does not expand `~`. A small
`_resolve_map_save_path()` helper calling `os.path.expanduser()` before any
file I/O fixed it, with a regression test added to prevent recurrence.

---

## 8. Results

| Capability | Status |
|---|---|
| ESP32 firmware: encoders, MCPWM motor control, SPI IMU, binary serial protocol | ✅ Working |
| `ros2_control` mecanum drive with measured kinematics | ✅ Working |
| EKF localization at 50 Hz, calibrated gyro, signal-clean SPI IMU | ✅ Working |
| `slam_toolbox` live mapping, coherent non-smeared map | ✅ Working |
| Nav2 (`SmacPlanner2D` + MPPI `DiffDrive`) within Pi 5 CPU budget | ✅ Working |
| Autonomous frontier exploration that doesn't give up prematurely | ✅ Working |
| Resilient mission control: persistent retry, `STUCK` escalation, path recording | ✅ 58/58 tests, validated live |
| `go_home` / `resume` path retrace | ✅ Implemented + unit-tested; full live E2E pending |
| Single-command full autonomous launch | ✅ `ros2 launch amr_bringup explore_map.launch.py` |

**Quantitative highlights:**

- 50 Hz fused localization (`/odom`) from a corrected, properly-scaled
  encoder model and a soldered, signal-clean IMU.
- A single autonomous exploration run grew the occupancy grid from 203×217 to
  391×315 cells — approximately 19.5 m × 15.75 m of real floor space mapped
  at 5 cm resolution.
- 58/58 unit tests passing for the mission-control state machine, covering
  state transitions, debounce timing, stall escalation, path recording, and
  command handling.
- A 2-second debounce eliminated a measured ~20 ms busy-loop against
  `explore_lite`'s synchronous planner.
- MPPI re-tuned from 62–125 ms/cycle (8–16 Hz effective) to within the 50 ms
  (20 Hz) budget while preserving a 2.8 s planning horizon.

---

## 9. Lessons Learned

1. **Measure the physical robot — never trust the placeholder.** Wheel
   separation, LiDAR TF offset, encoder counts-per-revolution, and GPIO safety
   zones all began as documented "best guesses" and had to be replaced with
   values taken directly off the hardware before the system worked correctly.

2. **One root cause can wear many masks.** The 2× odometry scaling bug
   produced SLAM smearing, phantom frontiers, and a dozen unrelated costmap
   tuning passes. The premature-stop bug had three independent root causes
   stacked on top of each other, each hiding the next until the previous was
   fixed.

3. **"It looks like it's working" is not validation.** The recovery-spin
   logic, stall watchdog, and the entire mission-control state machine sat
   completely inert for multiple sessions because `_state` never left `IDLE`
   — the robot looked autonomous because `explore_lite` genuinely was, but
   none of the home_manager's safety nets were actually armed.

4. **Resource-constrained compute changes the right answer, not just the
   tuning.** MPPI's textbook-default parameters were correct for a desktop;
   on a Pi 5 they overran the control-loop budget by 2–3×. The fix preserved
   the planning horizon and collision-check resolution while fitting the real
   CPU budget — a different optimum, not a worse one.

5. **When a build behaves impossibly, suspect the build before the silicon.**
   Two days of GPIO-conflict theories were ultimately moot — a clean rebuild
   fixed a "hardware bug" that was, in fact, stale build state.

6. **Hand-testing can invalidate a measurement.** Hand-pushing the mecanum
   robot to test odometry produces invalid data, because the rollers skid
   sideways under lateral hand pressure — only motor-driven motion gives valid
   calibration data.

---

## 10. Future Work

- Complete a live end-to-end validation of `go_home` and `resume` path
  retrace on hardware (currently unit-tested only).
- Investigate the occasional `slam_toolbox/save_map` "Failed to spin map
  subscription" error under load and add a retry-when-idle path.
- Run a full open-space exploration pass to assess long-run map quality
  (wall straightness, loop-closure accuracy) at scale.
- Remove the confirmed-no-op `start_with_rotations: true` setting from
  `explore.yaml`.
- Wire up the reserved `PARAM_SET` (runtime PID tuning) and `DIAGNOSTICS`
  (battery voltage, error flags) packet types, currently defined in the
  serial protocol but unused.
- Investigate whether further MPPI tuning can recover partial mecanum
  strafing without reintroducing the `PathAngleCritic`/`Omni` conflict.

---

## 11. Conclusion

Aurora demonstrates a complete, real-hardware autonomous mobile robot stack
built from first principles: a real-time embedded drivetrain controller, a
calibrated sensor-fusion pipeline, live SLAM, a CPU-budget-aware Nav2
configuration, and a test-driven, resilient mission-control state machine that
can explore, recognize when it is stuck, and retrace its own path home. The
project's defining engineering lesson is that most of the hardest bugs were
not failures of any single component, but failures of the *assumptions
connecting* components — an encoder mode, a QoS policy, a state machine that
was never entered. Each was found not by guessing harder, but by measuring the
real system and tracing each symptom back through the data flow until the
actual point of disagreement between assumption and reality was found.

Full chronological detail for every debugging session referenced in this
report — 33 sections in total — is recorded in `docs/project_log.md`.
