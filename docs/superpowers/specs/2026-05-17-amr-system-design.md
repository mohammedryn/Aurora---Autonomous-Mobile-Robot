# AMR System Design
**Single Source of Truth — Autonomous Mobile Robot**
*Created: 2026-05-17*

---

## Table of Contents

1. [Project Goal](#1-project-goal)
2. [Hardware Inventory](#2-hardware-inventory)
3. [System Architecture](#3-system-architecture)
4. [Hardware Wiring](#4-hardware-wiring)
5. [Firmware Architecture — ESP32-P4](#5-firmware-architecture--esp32-p4)
6. [ROS2 Package Structure & Node Graph](#6-ros2-package-structure--node-graph)
7. [Sensor Fusion & Localization](#7-sensor-fusion--localization)
8. [SLAM](#8-slam)
9. [Navigation Stack](#9-navigation-stack)
10. [Autonomous Exploration & Post-Exploration](#10-autonomous-exploration--post-exploration)
11. [Simulation](#11-simulation)
12. [Development Workflow](#12-development-workflow)
13. [Key Architectural Decisions](#13-key-architectural-decisions)
14. [Open Measurements Required](#14-open-measurements-required)
15. [Phase 10 — Industry-Grade Deployment Roadmap](#15-phase-10--industry-grade-deployment-roadmap)

---

## 1. Project Goal

Power on the robot in an unknown location. The robot must:

1. Autonomously explore the entire reachable area, building a complete map
2. Avoid all obstacles throughout exploration and navigation
3. Return to its starting position when exploration is complete
4. Save the map and make it available for inspection
5. Accept a user-defined navigation goal (click on map) and drive there, avoiding obstacles

No human intervention between power-on and goal arrival.

---

## 2. Hardware Inventory

| Component | Model | Role |
|---|---|---|
| Frame | Aluminium extrusion, 75 × 58.5 cm | Chassis |
| Wheels | 60mm aluminium mecanum wheels ×4 | Holonomic motion |
| Motors | PG36M555-13.7K, 12V, 359RPM, 4.5 N-cm ×4 | Wheel actuation |
| Encoders | ME-37, 7PPR (on motor shaft) ×4 | Wheel odometry |
| Motor Drivers | Cytron MDD10A ×2 (dual channel each) | PWM motor driving |
| LiDAR | Slamtec C1M1 R2 | Primary SLAM sensor |
| Compute | Raspberry Pi 5 8GB, Ubuntu 24.04 | High-level compute |
| MCU | ESP32-P4 (Wi-Fi 6 / BT5), Waveshare dev board | Real-time control |
| IMU | SmartElex 9DoF breakout — ISM330DHCX + MMC5983MA | Inertial sensing |
| ToF | SmartElex VL53L5CX, 8×8 pixel, 4m range | Low-obstacle detection |
| Main Battery | 3S3P LiPo, 2600mAh cells → 7800mAh, 11.1V nom / 12.8V max | Motor + system power |
| RPi5 Battery | 4S 1200mAh → XL buck converter → 5.12V / 5.1A | RPi5 power |

### Encoder Resolution

```
ME-37: 7 PPR on motor shaft
Quadrature (4-edge): 7 × 4 = 28 counts per motor shaft revolution
Output shaft:        28 × 13.7 (gear ratio) = 384 counts per output revolution
Wheel circumference: π × 60mm = 188.5mm
Linear resolution:   188.5mm / 384 = 0.49mm per count
Max pulse rate:      (359/13.7 RPM) × 384 / 60 ≈ 168 counts/sec/motor
```

---

## 3. System Architecture

### Three-Layer Model

```
┌─────────────────────────── DEVELOPER LAYER ───────────────────────────────────┐
│  WSL2 Ubuntu 22.04 (Windows 11)                                               │
│  • Write code (VSCode Remote WSL)      • Flash ESP32-P4 firmware              │
│  • Git commits / push to remote        • Run simulation in Docker             │
└───────────────────────────────────────────────────────────────────────────────┘
                                  │ SSH + rsync → deploy.sh
┌─────────────────────────── COMPUTE LAYER ─────────────────────────────────────┐
│  Raspberry Pi 5 8GB — Ubuntu 24.04 — ROS2 Jazzy                              │
│                                                                               │
│  sllidar_ros2 ──► slam_toolbox ──────────────────────► /map                  │
│  imu_sensor_node ──► imu_filter_madgwick ────────────► /imu/data            │
│  tof_pointcloud_node ────────────────────────────────► /tof/points          │
│                        │ map→odom TF                                          │
│  amr_hardware ──► robot_localization (EKF) ──────────► /odom                 │
│       │                │ odom→base_link TF                                    │
│  mecanum_drive_controller                                                     │
│                                                                               │
│  nav2_collision_monitor (/cmd_vel → /cmd_vel_safe)                           │
│                                                                               │
│  nav2_planner (SmacPlannerLattice) ◄── nav2_controller (MPPI)               │
│                    ▲                           ▲                              │
│             nav2_bt_navigator ◄── explore_lite / Foxglove goal               │
│                                                                               │
│  amr_home_manager ──────────────────────────────────── lifecycle node        │
│  foxglove_bridge  ──────────────────────────────────── WebSocket :8765       │
└───────────────────────────────────── │ USB-C serial @ 921600 baud ───────────┘
                                       │
┌─────────────────────────── FIRMWARE LAYER ────────────────────────────────────┐
│  ESP32-P4 — FreeRTOS                                                          │
│                                                                               │
│  task_encoder_read  (1kHz)  → 4× quadrature PCNT counts                     │
│  task_pid_control   (1kHz)  → 4× wheel velocity PID → PWM                   │
│  task_serial_comms  (100Hz) → binary packet protocol ↔ RPi5                 │
│                                                                               │
│  Cytron MDD10A #1: FL + RL motors                                            │
│  Cytron MDD10A #2: FR + RR motors                                            │
└───────────────────────────────────────────────────────────────────────────────┘
```

### End-to-End Data Flow

```
LiDAR /scan ──────────────────────────────────────┐
                                                   ▼
Encoders → mecanum fwd kinematics → /odom/wheel → EKF → /odom → slam_toolbox → /map
                                                   ▼                    ▼
Pi-side IMU → madgwick filter → /imu/data ────────┘           global costmap
                                                                    ▼
Pi-side ToF → PointCloud2 /tof/points ──► local costmap     SmacPlannerLattice
                                                   ▼                    ▼
                              collision_monitor → MPPI controller → /cmd_vel_safe
                                                                    ▼
                                              mecanum_drive_controller → 4× ω (rad/s)
                                                                    ▼
                                                    serial CMD_VEL packet
                                                                    ▼
                                                         ESP32 PID → MDD10A PWM → motors
```

---

## 4. Hardware Wiring

### Power Rails

```
Main Battery (3S3P LiPo — 11.1V nominal, 12.8V max, ~7800mAh)
├── VM+ → Cytron MDD10A #1 motor power input
├── VM+ → Cytron MDD10A #2 motor power input
└── GND → MDD10A #1 GND ┐
                         ├── COMMON GROUND RAIL ← must share with ESP32 GND
                         └── MDD10A #2 GND

RPi5 Battery (4S 1200mAh → XL buck converter → 5.12V / 5.1A)
└── USB-C → RPi5 power input
    └── RPi5 USB-A → ESP32-P4 USB-C   [5V power + serial data, single cable]
```

> **Rule:** ESP32-P4 GND and both MDD10A GNDs must share the same negative terminal
> as the main battery. Isolated grounds = undefined logic levels on PWM/DIR pins.

### Cytron MDD10A Wiring

Both boards operate in **Sign-Magnitude mode** (PWM = speed, DIR = direction).

```
MDD10A #1  (Left-side motors)
  VCC  ← ESP32-P4 3V3 pin     [logic power]
  GND  ← Common ground rail
  PWM1 ← ESP32 GPIO_FL_PWM    → drives FL motor (M1A / M1B terminals)
  DIR1 ← ESP32 GPIO_FL_DIR
  PWM2 ← ESP32 GPIO_RL_PWM    → drives RL motor (M2A / M2B terminals)
  DIR2 ← ESP32 GPIO_RL_DIR

MDD10A #2  (Right-side motors)
  VCC  ← ESP32-P4 3V3 pin
  GND  ← Common ground rail
  PWM1 ← ESP32 GPIO_FR_PWM    → drives FR motor (M1A / M1B terminals)
  DIR1 ← ESP32 GPIO_FR_DIR
  PWM2 ← ESP32 GPIO_RR_PWM    → drives RR motor (M2A / M2B terminals)
  DIR2 ← ESP32 GPIO_RR_DIR
```

### ESP32-P4 GPIO Assignment

| GPIO | Function | Peripheral | Notes |
|---|---|---|---|
| 4 | FL_PWM | LEDC ch0 | 20kHz carrier |
| 5 | FL_DIR | GPIO out | H=forward |
| 6 | FR_PWM | LEDC ch1 | 20kHz carrier |
| 7 | FR_DIR | GPIO out | |
| 8 | RL_PWM | LEDC ch2 | 20kHz carrier |
| 9 | RL_DIR | GPIO out | |
| 10 | RR_PWM | LEDC ch3 | 20kHz carrier |
| 11 | RR_DIR | GPIO out | |
| 12 | FL_ENC_A | PCNT unit0 | Quadrature A |
| 13 | FL_ENC_B | PCNT unit0 | Quadrature B |
| 14 | FR_ENC_A | PCNT unit1 | |
| 15 | FR_ENC_B | PCNT unit1 | |
| 16 | RL_ENC_A | PCNT unit2 | |
| 17 | RL_ENC_B | PCNT unit2 | |
| 18 | RR_ENC_A | PCNT unit3 | |
| 19 | RR_ENC_B | PCNT unit3 | |
| USB D+/D− | Serial to RPi5 | USB OTG | CDC-ACM device |

> Verify all GPIO numbers against the Waveshare ESP32-P4-WIFI6 board silkscreen
> before soldering. The P4 chip can route most peripherals to most pins but the
> dev board may have some pins tied to onboard peripherals.

### RPi5 Sensor Connections

In the baseline hardware split, the Raspberry Pi 5 owns all non-LiDAR sensors:

```
ISM330DHCX IMU   → RPi5 SPI or I2C header   [final bus/pin choice validated during bring-up]
VL53L5CX ToF     → RPi5 I2C header          [publish as /tof/points on Pi]
Slamtec C1M1 R2  → RPi5 USB-A               [/dev/lidar via udev]
ESP32-P4         → RPi5 USB-A               [/dev/amr_mcu via udev]
```

This keeps the ESP32-P4 focused on hard real-time drivetrain control while the
RPi5 handles ROS-native sensor drivers, filtering, and perception.

### Direct USB Connections to RPi5

```
Slamtec C1M1 R2 LiDAR → USB-A port on RPi5  (/dev/lidar  via udev)
ESP32-P4 USB-C         → USB-A port on RPi5  (/dev/amr_mcu via udev)
```

### udev Rules (RPi5 — stable device names)

```
# /etc/udev/rules.d/99-amr.rules
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", \
  SYMLINK+="lidar", MODE="0666"

SUBSYSTEM=="tty", ATTRS{idVendor}=="303a", ATTRS{idProduct}=="1001", \
  SYMLINK+="amr_mcu", MODE="0666"
```

> Verify vendor/product IDs with `lsusb` on first connection and update if different.

---

## 5. Firmware Architecture — ESP32-P4

### FreeRTOS Task Layout

| Task | Core | Priority | Rate | Responsibility |
|---|---|---|---|---|
| task_encoder_read | 0 | 9 | 1kHz | Snapshot 4× PCNT counters atomically |
| task_pid_control | 0 | 10 | 1kHz | 4× wheel velocity PID → PWM output |
| task_serial_comms | 1 | 8 | 100Hz TX | Assemble + send packets; parse RX CMD |

Real-time tasks (encoder + PID) are pinned to Core 0 and never blocked by I/O.
The serial task runs on Core 1. Shared data is protected by FreeRTOS mutexes.

### Serial Packet Protocol

All packets follow this frame:

```
[0xAA][0x55][TYPE:1][LEN:1][PAYLOAD:LEN bytes][CRC16_HI][CRC16_LO]
```

CRC16 computed over TYPE + LEN + PAYLOAD. Receiver validates CRC before processing.

| Type | Dir | Rate | Payload | Total Size |
|---|---|---|---|---|
| `0x01` CMD_VEL | Host→MCU | on demand | 4× float32 wheel ω rad/s | 22 B |
| `0x02` STATE | MCU→Host | 100Hz | timestamp(4) + 4×enc_delta int32(16) | 26 B |
| `0x04` HEARTBEAT | Host→MCU | 1Hz | — | 6 B |
| `0x05` PARAM_SET | Host→MCU | on demand | param_id(1) + value f32(4) | 11 B |
| `0x06` DIAGNOSTICS | MCU→Host | 1Hz | batt_mv uint16(2) + error_flags uint8(1) | 9 B |

**Wire bandwidth:** STATE@100Hz = 2,600 B/s total.
921600 baud ≈ 92,160 B/s capacity. Usage: **2.8%**. Ample headroom.

**Watchdog:** No HEARTBEAT received for 2 seconds → firmware zeros all wheel
velocity setpoints and sets `error_flags` bit 0. Hardware-level E-stop independent
of ROS2 state.

### Wheel PID

Each of the 4 wheels has an independent velocity PID controller running at 1kHz.

```
setpoint (rad/s) ──► [PID Kp=2.0, Ki=5.0, Kd=0.01] ──► duty [-1.0, 1.0]
        ▲                                                        │
        │                                              ┌─────────▼──────────┐
encoder_delta / dt (rad/s)                             │ Sign-Magnitude map │
                                                       │ duty>0: DIR=H,PWM=duty│
                                                       │ duty<0: DIR=L,PWM=|d| │
                                                       └─────────────────────┘
```

Anti-windup: integrator clamped when output is saturated.
Initial gains are starting estimates — tune on real hardware with step response.
Gains can be updated at runtime via `0x05 PARAM_SET` packet without reflashing.

### Firmware Scope Boundary

The ESP32-P4 firmware is intentionally limited to drivetrain responsibilities:
encoder capture, wheel PID, serial command handling, and watchdog-backed E-stop.
IMU and ToF sensing are handled directly on the RPi5 so ROS2 can consume those
topics without an extra MCU-side sensor transport layer.

### Firmware Project Layout

```
firmware/
├── CMakeLists.txt
├── sdkconfig                          # WiFi + BT disabled to save RAM
├── components/
│   └── serial_protocol/               # Packet framing, CRC16, encode/decode
└── main/
    ├── main.c                         # Task creation, hardware init
    ├── encoder.c / .h                 # PCNT quadrature decoder, 4 units
    ├── motor.c / .h                   # PWM init, set_duty()
    ├── pid.c / .h                     # Generic PID, anti-windup
    └── tasks/
        ├── task_encoder_read.c
        ├── task_pid_control.c
        └── task_serial_comms.c
```

---

## 6. ROS2 Package Structure & Node Graph

### Environment

- **OS:** Ubuntu 24.04 (RPi5) / Ubuntu 24.04 Docker (sim)
- **ROS2:** Jazzy Jalisco (LTS, pairs with Gazebo Harmonic)
- **Build:** colcon with `--symlink-install` for fast parameter iteration

### Workspace

```
ros2_ws/src/
├── amr_description/      # URDF, sensor frames, Gazebo plugins
├── amr_hardware/         # ros2_control hardware interface, serial ↔ ESP32-P4
├── amr_bringup/          # All launch files, top-level configs, EKF config
├── amr_navigation/       # Nav2 params, costmaps, lattice file
├── amr_slam/             # slam_toolbox params
├── amr_exploration/      # amr_home_manager node + explore_lite configs
├── sllidar_ros2/         # cloned — Slamtec LiDAR ROS2 driver
└── m-explore-ros2/       # cloned — explore_lite for ROS2 (robo-friends/m-explore-ros2)
                          # NOT available as binary apt package for Jazzy — must build from source
```

### Full Node Graph

```
[sllidar_ros2]
    → /scan  (LaserScan, 10Hz)

[amr_hardware]  — ros2_control SystemInterface, serial ↔ /dev/amr_mcu
    → /joint_states         (JointState, 100Hz)

[mecanum_drive_controller]  — ros2_controllers standard plugin
    ← /cmd_vel_safe         (Twist)
    → /odom/wheel           (Odometry, 50Hz)
    commands joint velocity interfaces → amr_hardware → serial CMD_VEL

[imu_sensor_node]
    → /imu/data_raw         (Imu, 100Hz)

[imu_filter_madgwick]  — imu_tools package
    ← /imu/data_raw
    → /imu/data             (Imu, 100Hz, stable orientation quaternion)

[tof_pointcloud_node]
    → /tof/points           (PointCloud2, 10Hz)

[robot_localization — EKF node]
    ← /odom/wheel
    ← /imu/data
    → /odom                 (Odometry, fused, 100Hz)
    → TF: odom → base_link

[slam_toolbox — online_async]
    ← /scan
    ← TF: odom → base_link
    → /map                  (OccupancyGrid, 1Hz)
    → TF: map → odom
    [runs in mapping mode permanently — map stays live and updates]

[nav2_collision_monitor]
    ← /cmd_vel              (from MPPI controller)
    ← /scan
    ← /tof/points
    → /cmd_vel_safe         (to mecanum_drive_controller)
    [SlowdownZone: 0.80m → 40% speed | StopZone: 0.42m → full stop]

[nav2_costmap_2d — global]
    ← /map  (static layer)
    ← /scan (obstacle layer)
    → /global_costmap/costmap

[nav2_costmap_2d — local]
    ← /scan        (obstacle layer — lidar)
    ← /tof/points  (obstacle layer — low obstacles, glass)
    ← /odom
    → /local_costmap/costmap

[nav2_planner — SmacPlannerLattice]
    ← /global_costmap/costmap
    → /plan
    [uses precomputed lattice/output.json for holonomic motion primitives]

[nav2_controller — MPPI, motion_model: Omni]
    ← /local_costmap/costmap
    ← /odom
    ← /plan
    → /cmd_vel

[nav2_bt_navigator]
    ← /goal_pose            (PoseStamped — from Foxglove click)
    ← NavigateToPose action (from explore_lite / amr_home_manager)
    orchestrates planner + controller + recoveries

[nav2_behavior_server]
    plugins: spin, backup, drive_on_heading, wait

[explore_lite — m-explore-ros2]
    ← /global_costmap/costmap + costmap_updates
    ← TF
    → NavigateToPose action goals
    → /explore/frontiers    (MarkerArray — visible in Foxglove)

[amr_home_manager]  — custom Python node
    saves home pose at T_init
    starts /explore action → monitors completion
    on complete: saves map, navigates home
    exposes /return_home service (std_srvs/Trigger)
    exposes /save_map service   (std_srvs/Trigger)

[foxglove_bridge]
    ↔ WebSocket :8765       → Foxglove Studio on Windows 11
```

### Key Topic Reference

| Topic | Type | Hz | Producer | Key Consumers |
|---|---|---|---|---|
| `/scan` | LaserScan | 10 | sllidar_ros2 | slam_toolbox, costmaps, collision_monitor |
| `/imu/data_raw` | Imu | 100 | imu_sensor_node | imu_filter_madgwick |
| `/imu/data` | Imu | 100 | imu_filter_madgwick | robot_localization |
| `/odom/wheel` | Odometry | 50 | mecanum_drive_controller | robot_localization |
| `/odom` | Odometry | 100 | robot_localization | Nav2, MPPI |
| `/tof/points` | PointCloud2 | 10 | tof_pointcloud_node | local costmap, collision_monitor |
| `/map` | OccupancyGrid | 1 | slam_toolbox | global costmap, explore_lite |
| `/cmd_vel` | Twist | 20 | MPPI controller | collision_monitor |
| `/cmd_vel_safe` | Twist | 20 | collision_monitor | mecanum_drive_controller |
| `/goal_pose` | PoseStamped | event | Foxglove user | bt_navigator |
| `/explore/frontiers` | MarkerArray | 0.33 | explore_lite | Foxglove (visual) |
| `/exploration_complete` | Bool | event | amr_home_manager | Foxglove (status) |
| `/home_pose` | PoseStamped | 1 | amr_home_manager | Foxglove (visual marker) |

### TF Tree

```
map                      ← slam_toolbox publishes
└── odom                 ← robot_localization EKF publishes
    └── base_link
        ├── base_laser   ← static (URDF) — LiDAR mount offset
        ├── imu_link     ← static (URDF) — IMU mount offset
        ├── tof_link     ← static (URDF) — ToF sensor, forward-facing
        ├── wheel_FL     ← mecanum_drive_controller (joint state)
        ├── wheel_FR     ← mecanum_drive_controller
        ├── wheel_RL     ← mecanum_drive_controller
        └── wheel_RR     ← mecanum_drive_controller
```

Each TF edge has exactly one publisher. No conflicts.

---

## 7. Sensor Fusion & Localization

### Two-Stage Fusion Pipeline

```
Stage 1 — Orientation
  Pi-side IMU raw accel + gyro (100Hz)
      → imu_filter_madgwick
      → /imu/data  (stable quaternion, gyro-drift corrected by gravity reference)

Stage 2 — Full Pose
  /odom/wheel (mecanum forward kinematics from encoder deltas)  ─┐
  /imu/data (orientation + angular velocity)                     ├─► EKF ──► /odom
                                                                ─┘
  slam_toolbox scan-match correction ──► TF map→odom
  (final ground truth, corrects accumulated drift globally)
```

### robot_localization EKF Config

```yaml
ekf_node:
  ros__parameters:
    frequency: 100.0
    sensor_timeout: 0.1
    two_d_mode: true              # planar robot — no Z/roll/pitch in state vector

    odom_frame:      odom
    base_link_frame: base_link
    world_frame:     odom

    odom0: /odom/wheel
    odom0_config: [true,  true,  false,    # x, y, z
                   false, false, true,     # roll, pitch, yaw
                   true,  true,  false,    # vx, vy, vz
                   false, false, true,     # vroll, vpitch, vyaw
                   false, false, false]    # ax, ay, az
    odom0_differential: false

    imu0: /imu/data
    imu0_config: [false, false, false,
                  false, false, true,      # yaw
                  false, false, false,
                  false, false, true,      # vyaw
                  false, false, false]
    imu0_differential: false
    imu0_remove_gravitational_acceleration: true
```

### Mecanum Forward Kinematics (in mecanum_drive_controller)

```
Given: ωFL, ωFR, ωRL, ωRR  [rad/s, signed]
       r  = 0.030 m          [wheel radius]
       lx = half-wheelbase   [measure on physical frame — see §14]
       ly = half-track        [measure on physical frame — see §14]

vx = (r/4) × ( ωFL + ωFR + ωRL + ωRR)
vy = (r/4) × (-ωFL + ωFR + ωRL - ωRR)
ωz = (r / (4×(lx+ly))) × (-ωFL + ωFR - ωRL + ωRR)
```

`lx` and `ly` are YAML parameters — measured once from the physical frame and set
in `amr_bringup/config/mecanum_drive_controller.yaml`. No code changes needed.

---

## 8. SLAM

**Package:** slam_toolbox — online_async mode
**Stays in mapping mode permanently.** The map is always live, always updating.
No mode transition after exploration. Nav2 always uses the current live map.

```yaml
# amr_slam/config/slam_toolbox.yaml
slam_toolbox:
  ros__parameters:
    mode: mapping
    use_sim_time: false
    solver_plugin: solver_plugins::CeresSolver
    ceres_linear_solver: SPARSE_NORMAL_CHOLESKY

    scan_topic:  /scan
    odom_frame:  odom
    map_frame:   map
    base_frame:  base_link

    resolution:               0.05    # 5cm/cell — resolves doorways clearly
    max_laser_range:          12.0    # C1M1 R2 rated range
    minimum_travel_distance:  0.10    # add pose node after 10cm movement
    minimum_travel_heading:   0.10    # or ~6° rotation

    do_loop_closing:                       true
    loop_search_maximum_distance:          3.0
    loop_match_minimum_chain_size:         10
    loop_match_minimum_response_fine:      0.45

    map_update_interval:      1.0     # publish /map at 1Hz
    use_multithread_scheduler: true   # use RPi5 cores for background solving
    tf_buffer_duration:       30.0
```

### Map Saving

`amr_home_manager` calls this when exploration is declared complete:

```bash
ros2 run nav2_map_server map_saver_cli \
  -f ~/maps/$(date +%Y%m%d_%H%M%S) \
  --ros-args -p save_map_timeout:=5.0
```

Outputs: `<timestamp>.pgm` (bitmap) + `<timestamp>.yaml` (metadata).
The live `/map` topic continues in Foxglove throughout and after exploration.

---

## 9. Navigation Stack

### Global Planner — SmacPlannerLattice

Selected over NavFn and SmacPlanner2D because it is specifically designed for
holonomic robots. Plans globally-optimal paths using precomputed motion primitives
(forward, strafe, diagonal, spin-in-place) that fully exploit mecanum capability.

```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_smac_planner::SmacPlannerLattice"
      allow_unknown: true
      tolerance: 0.25
      max_iterations: 4000
      max_planning_time: 5.0
      reverse_penalty: 2.0
      cost_penalty: 2.0
      rotation_penalty: 5.0
      lattice_filepath: "$(find amr_navigation)/config/lattice/output.json"
      cache_obstacle_heuristic: true
      allow_reverse_expansion: true
      smooth_path: true
```

**Lattice generation (run once, commit output.json):**
```bash
# Verify exact script name and flags against Nav2 Jazzy docs for SmacPlannerLattice.
# The script ships with the nav2_smac_planner package:
ros2 run nav2_smac_planner generate_lattice \
  --config-filepath amr_navigation/config/lattice_gen.yaml \
  --output-filepath amr_navigation/config/lattice/output.json
```
```yaml
# lattice_gen.yaml
minimum_turning_radius: 0.0
allow_reverse_motion: true
n_orientations: 16              # 22.5° heading resolution
motion_model: "omni"
turning_radii: [0.0, 0.4, 0.8, 1.2, 1.8]
```

### Local Controller — MPPI

Selected as the most capable Nav2 controller for holonomic motion. Samples 2000
trajectory rollouts per cycle, weights by multi-critic cost, executes optimal mean.
`motion_model: "Omni"` is essential — enables full (vx, vy, ωz) control.

```yaml
controller_server:
  ros__parameters:
    controller_frequency: 20.0
    FollowPath:
      plugin: "nav2_mppi_controller::MPPIController"
      time_steps: 56
      model_dt: 0.05
      batch_size: 2000
      vx_std: 0.2
      vy_std: 0.2               # non-zero — holonomic
      wz_std: 0.4
      vx_max:  0.50
      vx_min: -0.50
      vy_max:  0.50
      vy_min: -0.50
      wz_max:  1.50
      motion_model: "Omni"      # ← critical for mecanum
      critics:
        - ConstraintCritic
        - GoalCritic
        - GoalAngleCritic
        - PathAlignCritic
        - PathFollowCritic
        - ObstaclesCritic
        - VelocityDeadbandCritic
      ObstaclesCritic:
        consider_footprint: true
        collision_cost: 10000.0
        collision_margin_distance: 0.15
      VelocityDeadbandCritic:
        deadband_velocities: [0.05, 0.05, 0.05]  # prevents goal jitter
```

### mecanum_drive_controller Config

```yaml
# amr_bringup/config/controllers.yaml
controller_manager:
  ros__parameters:
    update_rate: 100

mecanum_drive_controller:
  ros__parameters:
    front_left_wheel_name:  wheel_FL_joint
    front_right_wheel_name: wheel_FR_joint
    rear_left_wheel_name:   wheel_RL_joint
    rear_right_wheel_name:  wheel_RR_joint
    wheel_separation_x: !!MEASURE!!   # front-to-rear axle distance (m)
    wheel_separation_y: !!MEASURE!!   # left-to-right axle distance (m)
    wheel_radius: 0.030
    open_loop: false
    enable_odom_tf: false             # robot_localization owns odom→base_link TF
    cmd_vel_timeout: 0.5
    publish_rate: 50.0
    velocity_rolling_window_size: 10
```

### Costmaps

**Robot footprint (75 × 58.5cm + 5cm padding):**
```
footprint: "[[0.375, 0.2925], [0.375, -0.2925], [-0.375, -0.2925], [-0.375, 0.2925]]"
footprint_padding: 0.05
inflation_radius: 0.55    # half-diagonal ~0.47m + 8cm clearance
```

**Local costmap:** 4.0m × 4.0m rolling window, 0.05m resolution
- obstacle_layer_lidar: `/scan` — primary obstacle source
- obstacle_layer_tof: `/tof/points` — obstacles 2–50cm tall (chair legs, cables)
- inflation_layer: 0.55m radius, cost_scaling_factor 3.5

**Global costmap:** full map size, 0.05m resolution
- static_layer: `/map` from slam_toolbox
- obstacle_layer: `/scan` for dynamic updates
- inflation_layer: same parameters as local

### Collision Monitor (Safety Layer)

Sits between MPPI output `/cmd_vel` and hardware `/cmd_vel_safe`.
Operates at 20Hz from raw sensor data — faster than costmap update cycle.
Works even if Nav2 produces a bad velocity command.

```
StopZone:     Circle r=0.42m  → full stop if /scan or /tof/points inside
SlowdownZone: Circle r=0.80m  → reduce to 40% speed if sensor inside
```

### Recovery Behaviors

Default Nav2 BT: navigate → obstacle → backup(0.3m) → spin(1.57rad) → retry.
After 3 retries: wait(3s) → retry. Standard plugins: spin, backup, drive_on_heading, wait.

### Velocity Limits Summary

| Parameter | Value | Rationale |
|---|---|---|
| vx_max / vx_min | ±0.50 m/s | Safe indoor AMR speed |
| vy_max / vy_min | ±0.50 m/s | Full holonomic lateral |
| wz_max | 1.50 rad/s | ~86°/s rotation |
| Motor max speed | ~1.13 m/s | 359/13.7 RPM × 2π × 0.030m |

---

## 10. Autonomous Exploration & Post-Exploration

### VL53L5CX → PointCloud2 Conversion

Implemented in a dedicated Pi-side ROS2 node (`tof_pointcloud_node`).

```
Sensor: 8×8 pixel grid, 63° × 63° FoV
Pixel (row, col):
  θ_h = (col - 3.5) × 7.875°   [horizontal]
  θ_v = (row - 3.5) × 7.875°   [vertical]
  d   = distance_mm / 1000.0    [meters]
  x   = d × cos(θ_v) × cos(θ_h)
  y   = d × cos(θ_v) × sin(θ_h)
  z   = d × sin(θ_v)
```

Unit vectors precomputed at node init — zero runtime trig cost.

**Filters applied before publishing:**
1. Drop pixels where d = 0 (no target) or d > 3.5m (unreliable)
2. Nav2 costmap applies min/max height filter in `tof_link` → `base_link` TF space

Published as `sensor_msgs/PointCloud2`, frame_id `tof_link`, at 10Hz.

### Explore Lite Config

```yaml
explore:
  ros__parameters:
    robot_base_frame: base_link
    costmap_topic: /global_costmap/costmap
    costmap_updates_topic: /global_costmap/costmap_updates
    visualize: true               # /explore/frontiers → Foxglove markers
    planner_frequency: 0.33       # recompute best frontier every 3 seconds
    progress_timeout: 45.0        # abandon stuck frontier after 45s
    potential_scale: 3.0          # prefer closer frontiers
    orientation_scale: 0.0        # holonomic — arrival orientation irrelevant
    gain_scale: 1.0               # prefer larger unknown regions
    min_frontier_size: 0.25       # ignore micro-frontiers near walls
    use_nav2_api: true
    transform_tolerance: 0.3
```

### amr_home_manager State Machine

```
WAITING_FOR_MAP
  └─ map→base_link TF valid + stable 3s ──► RECORDING_HOME

RECORDING_HOME
  └─ save home_pose (PoseStamped in map frame)
     publish /home_pose (visible in Foxglove)
     start /explore action ──► EXPLORING

EXPLORING
  └─ /explore action succeeded ──► SAVING_MAP
     /explore action aborted  ──► retry ×3, then SAVING_MAP

SAVING_MAP
  └─ call map_saver_cli → ~/maps/<timestamp>.pgm/.yaml
     publish /exploration_complete = true
     ──► RETURNING_HOME

RETURNING_HOME
  └─ NavigateToPose → home_pose
     succeeded or failed → IDLE

IDLE
  ├─ /return_home service (std_srvs/Trigger) → NavigateToPose to home_pose
  ├─ /save_map service    (std_srvs/Trigger) → map_saver_cli
  └─ slam_toolbox continues mapping — /goal_pose from Foxglove accepted
```

### Full Operational Lifecycle

```
T+0s     Power on — ESP32 drivetrain firmware streams STATE packets within 2s
T+0s     RPi5 systemd amr.service starts all ROS2 nodes
T+2s     sllidar_ros2 publishing /scan
T+2s     amr_hardware serial link established, /odom/wheel source live
T+2s     imu_sensor_node + tof_pointcloud_node publishing sensor topics
T+8–12s  slam_toolbox has enough scans → first /map + map→odom TF published
T+12s    amr_home_manager: home pose saved → /explore action started
T+12s    Foxglove on Windows: connect → full live view available

T+12s→completion:
  explore_lite selects frontier (closest high-gain unknown cell)
  SmacPlannerLattice plans holonomic path
  MPPI controller executes smooth trajectory
  collision_monitor watches 0.42m stop zone and 0.80m slowdown zone
  slam_toolbox closes loops → map sharpens
  New frontiers every 3s → robot advances
  ↑ Repeat until no reachable frontiers

Exploration complete:
  amr_home_manager saves map snapshot
  publishes /exploration_complete = true
  Navigates back to home_pose

IDLE phase (indefinite):
  slam_toolbox keeps map live
  User opens Foxglove → sees complete map
  User clicks 3D panel → /goal_pose published
  bt_navigator → SmacPlannerLattice + MPPI → robot drives to goal
  collision_monitor active throughout
  Repeat for any number of goals
```

### Foxglove Studio Setup

**Connection:** `ws://amr.local:8765`

**Recommended panel layout:**
```
┌──────────────────────────────────┬─────────────────────────┐
│  3D Panel                        │  Plot Panel             │
│  • /map         OccupancyGrid    │  • /odom vx, vy         │
│  • /odom        path trail       │  • /imu/data angular_z  │
│  • /scan        LaserScan        ├─────────────────────────┤
│  • /tof/points  PointCloud2      │  Raw Messages           │
│  • /explore/frontiers markers    │  • /exploration_complete│
│  • /home_pose   marker           │  • /collision_monitor   │
│  Publish click → /goal_pose      │    state                │
└──────────────────────────────────┴─────────────────────────┘
```

**Goal setting:** 3D panel → settings → Add Publish Action →
topic: `/goal_pose`, type: `geometry_msgs/PoseStamped`. Click + drag on map.

---

## 11. Simulation

### Architecture

Docker container with Ubuntu 24.04 + ROS2 Jazzy + Gazebo Harmonic — identical
stack to the RPi5. Ensures sim-to-real transfer with zero translation.

The sim-to-real boundary is a single line in the URDF:

```xml
<!-- amr_description/urdf/amr.urdf.xacro -->
<ros2_control name="AMRSystem" type="system">
  <hardware>
    <xacro:if value="$(arg use_sim)">
      <plugin>gz_ros2_control/GazeboSimSystem</plugin>
    </xacro:if>
    <xacro:unless value="$(arg use_sim)">
      <plugin>amr_hardware/AMRHardwareInterface</plugin>
      <param name="serial_port">/dev/amr_mcu</param>
      <param name="baud_rate">921600</param>
    </xacro:unless>
  </hardware>
  <!-- joints identical in both modes -->
</ros2_control>
```

Everything above ros2_control (Nav2, SLAM, explore_lite, EKF) is identical.
`mecanum_drive_controller` runs identically — it sees joint interfaces regardless
of whether they come from Gazebo or from an ESP32 over serial.

### Gazebo Sensor Plugins

IMU noise parameters match ISM330DHCX datasheet — EKF gains tuned in simulation
carry over to real hardware accurately.

```xml
<!-- LiDAR: 720 samples/scan, 12m range, 10Hz -->
<!-- IMU: accel stddev=0.021, gyro stddev=0.009 (datasheet values) -->
```

### Docker Setup

```dockerfile
# docker/Dockerfile
FROM osrf/ros:jazzy-desktop
RUN apt-get update && apt-get install -y \
    ros-jazzy-gz-ros2-control ros-jazzy-ros-gz \
    ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox \
    ros-jazzy-robot-localization ros-jazzy-imu-tools \
    ros-jazzy-ros2-controllers ros-jazzy-foxglove-bridge \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*
# Note: explore_lite (m-explore-ros2) is NOT in Jazzy apt — it is built from
# source inside the workspace (ros2_ws/src/m-explore-ros2/)
```

```bash
# docker/run_sim.sh
docker run -it --rm \
  --env DISPLAY=$DISPLAY \
  --env ROS_DOMAIN_ID=42 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v $HOME/amr/ros2_ws:/amr_ws \
  --network host \
  amr_sim:latest \
  ros2 launch amr_bringup sim.launch.py
```

`--network host` allows Foxglove on Windows to connect to port 8765 identically
to connecting to the real RPi5.

---

## 12. Development Workflow

### Three Environments

| Environment | OS | ROS2 | Role |
|---|---|---|---|
| WSL2 (native) | Ubuntu 22.04 | Humble | Edit code, flash ESP32, git |
| Docker on WSL2 | Ubuntu 24.04 | Jazzy + Gazebo Harmonic | Simulate, validate, tune |
| RPi5 | Ubuntu 24.04 | Jazzy | Real hardware |

### Daily Iteration Loop

```
1. Edit code in VSCode (Remote WSL extension → WSL2 Ubuntu 22.04)
2. ./docker/run_sim.sh  →  test in Gazebo Harmonic
3. Open Foxglove → ws://localhost:8765  →  inspect topics live
4. ./scripts/deploy.sh  →  rsync + remote colcon build + restart service
5. Connect Foxglove → ws://amr.local:8765  →  test on real hardware
```

### deploy.sh

```bash
#!/bin/bash
set -e
RPI="ubuntu@amr.local"
rsync -avz --delete ~/amr/ros2_ws/src/ $RPI:~/amr_ws/src/
ssh $RPI "
  source /opt/ros/jazzy/setup.bash &&
  cd ~/amr_ws &&
  colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1 | tail -30
"
ssh $RPI "sudo systemctl restart amr.service"
```

### RPi5 Auto-Start (systemd)

```ini
# /etc/systemd/system/amr.service
[Unit]
Description=AMR ROS2 Stack
After=network.target

[Service]
Type=simple
User=ubuntu
ExecStart=/bin/bash -c \
  "source /opt/ros/jazzy/setup.bash && \
   source /home/ubuntu/amr_ws/install/setup.bash && \
   ros2 launch amr_bringup amr.launch.py"
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
```

Power on → 15 seconds → full stack running → Foxglove ready. No SSH required.

### Firmware Flash

```bash
# From WSL2, ESP32-P4 connected via USB-C
cd ~/amr/firmware
idf.py set-target esp32p4
idf.py build
idf.py -p /dev/ttyACM0 flash monitor
```

PID gains can be updated at runtime via `0x05 PARAM_SET` packets — no reflash needed
for tuning. Reflash only for driver updates or protocol changes.

### Network Config

| Mode | Setup |
|---|---|
| Development | Ethernet cable RPi5 ↔ router, static IP `192.168.1.100`, hostname `amr.local` |
| Deployment | RPi5 connects to home WiFi, same static IP, same hostname |

`ROS_DOMAIN_ID=42` set on both RPi5 and Foxglove to isolate from other ROS2 traffic.

---

## 13. Key Architectural Decisions

| Decision | Choice | Rationale |
|---|---|---|
| MCU↔RPi5 protocol | ros2_control + custom serial (Approach B) | Industry standard; debuggable; firmware-agnostic to ROS2 |
| MCU↔RPi5 transport | USB-C serial @ 921600 baud | Deterministic; no WiFi dependency on control path |
| Sensor placement | LiDAR + IMU + ToF on RPi5; motors + encoders on ESP32-P4 | Keeps MCU focused on hard real-time control while staying within Pi 5 8GB budget |
| ROS2 version | Jazzy Jalisco | LTS for Ubuntu 24.04 — deployment target |
| SLAM | slam_toolbox online_async, permanent mapping mode | No mode transition complexity; live map always available |
| Global planner | SmacPlannerLattice | Only Nav2 planner that exploits holonomic motion primitives |
| Local controller | MPPI (Omni mode) | Best smooth holonomic trajectory tracking in Nav2 Jazzy |
| Drive controller | mecanum_drive_controller (ros2_controllers) | Official package; handles kinematics + odometry |
| Sensor fusion | imu_filter_madgwick → robot_localization EKF | Two-stage: orientation stability then full pose fusion |
| Magnetometer | Disabled (MMC5983MA ignored) | DC motor fields corrupt indoor magnetic readings |
| Safety layer | nav2_collision_monitor | Operates on raw sensor data, faster than costmap cycle |
| ToF integration | Dedicated Pi-side pointcloud node | Keeps sensor processing ROS-native and decoupled from drivetrain firmware |
| Exploration | m-explore-ros2 (frontier-based) | Lightweight, Nav2-native, well-maintained ROS2 port |
| Post-exploration | slam_toolbox stays in mapping mode | Map stays live; no restart; handles environment changes |
| User interface | Foxglove Studio via foxglove_bridge | Professional visualization; click-to-navigate built-in |
| Simulation | Docker (Ubuntu 24.04 + Jazzy + Gazebo Harmonic) | Exact parity with deployment — zero sim-to-real translation |
| RPi5 deployment | Native colcon build via SSH | RPi5 8GB handles it; no cross-compilation complexity |

---

## 14. Open Measurements Required

These values cannot be determined from specs — measure from the physical robot frame:

| Parameter | Where Used | How to Measure |
|---|---|---|
| `wheel_separation_x` | mecanum_drive_controller, URDF | Distance between front and rear wheel axle centers |
| `wheel_separation_y` | mecanum_drive_controller, URDF | Distance between left and right wheel centerlines |
| LiDAR mount height | URDF `base_laser` TF | Height of LiDAR scan plane from floor |
| LiDAR x/y offset | URDF `base_laser` TF | Forward/lateral offset of LiDAR from robot center |
| IMU mount position | URDF `imu_link` TF | x/y/z offset of IMU from base_link origin |
| ToF mount height | URDF `tof_link` TF | Height of sensor from floor — sets height filter reference |
| ToF x offset | URDF `tof_link` TF | Forward distance from robot center |
| MDD10A vendor/product ID | udev rules | Run `lsusb` with each device connected and check |

All of these go into URDF xacro parameters and controller YAML files.
No source code changes needed — only config values.

---

## 15. Phase 10 — Industry-Grade Deployment Roadmap

### 15.0 Goal & Scope

Phases 1-7 (sections 1-14, complete and validated on hardware) deliver: power-on
→ autonomous frontier exploration → map save → return home → click-to-navigate.
Phase 10 turns this from "a robot that can do an impressive demo while supervised"
into "a robot that can be deployed and run unattended, with the operational
tooling expected of a real product."

Priority order (highest-impact first; each is independently shippable):

| Priority | Section | What it adds | New packages |
|---|---|---|---|
| 1 | §15.1 Localization mode | Map once, deploy many — load saved map + AMCL instead of re-mapping every boot | `amr_localization` |
| 2 | §15.3 Battery monitoring | Auto-return before the robot dies mid-mission | `amr_battery` |
| 3 | §15.5 Diagnostics aggregator | Single `/diagnostics_agg` topic reporting health of every subsystem | `amr_diagnostics` |
| 4 | §15.5 Autostart hardening | Robot is fully operational on power-on, no SSH, self-restarts on hang | amr_bringup (systemd only) |
| 5 | §15.2 Localization watchdog | Auto-relocalize if AMCL diverges, no human intervention | amr_home_manager (extension) |
| 6 | §15.4 Person detection | Camera-based safety stop, independent of LiDAR | `amr_perception` |
| 7 | §15.4 Floor hazard ToF | VL53L5CX repurposed for cliff/low-obstacle detection | `amr_perception` |
| 8 | §15.6 Web dashboard | Non-technical operator UI | `dashboard/` (Next.js, outside ros2_ws) |
| 9 | §15.7 Validation | Quantified benchmarks + demo video + technical writeup | `tools/benchmark_run.py` |

Each subsection below is written so it can become its own implementation-plan
task list (`docs/superpowers/plans/2026-06-1X-phase8-*.md`) without further
design work.

---

### 15.1 Localization Mode — Map Once, Deploy Many (AMCL)

**Problem:** Every boot currently re-runs SLAM from a blank map (§8). A deployed
robot should map an environment once, save it, and on every subsequent boot load
that map and localize against it — no re-mapping, no re-exploration.

**New package:** `amr_localization`

```
ros2_ws/src/amr_localization/
├── config/
│   └── amcl.yaml
└── launch/
    └── localize_map.launch.py   # mirrors explore_map.launch.py
```

**Mode switch (launch-time, not runtime):**

```bash
ros2 launch amr_bringup explore_map.launch.py     # Phase 1-7: SLAM + explore
ros2 launch amr_bringup localize_map.launch.py \
  map:=/home/ubuntu/AMR/maps/explore_map.yaml      # Phase 10: load map + AMCL + Nav2 only
```

`localize_map.launch.py` replaces `slam_toolbox` + `amr_explore` with
`nav2_map_server` + `nav2_amcl`, both lifecycle-managed. Everything below the
localization layer (EKF, Nav2 planner/controller, collision_monitor,
mecanum_drive_controller) is **unchanged** — same as the sim-to-real boundary
in §11, this is a single swappable layer.

```yaml
# amr_localization/config/amcl.yaml
amcl:
  ros__parameters:
    use_sim_time: false
    base_frame_id: "base_footprint"     # matches ekf.yaml base_link_frame
    global_frame_id: "map"
    odom_frame_id: "odom"
    scan_topic: scan
    set_initial_pose: true
    initial_pose: {x: 0.0, y: 0.0, z: 0.0, yaw: 0.0}   # = home_pose at last mapping run
    # DifferentialMotionModel matches the current MPPI motion_model: DiffDrive
    # (amr_nav/config/nav2_params.yaml) -- the robot drives forward/rotate,
    # never strafes, so AMCL's motion model should assume the same. Switch to
    # OmniMotionModel only if MPPI reverts to motion_model: Omni.
    robot_model_type: "nav2_amcl::DifferentialMotionModel"
    min_particles: 500
    max_particles: 2000
    update_min_d: 0.1
    update_min_a: 0.1
    resample_interval: 1
    transform_tolerance: 1.0
    laser_model_type: "likelihood_field"
    laser_max_range: 12.0
    laser_min_range: -1.0
    z_hit: 0.5
    z_short: 0.05
    z_max: 0.05
    z_rand: 0.5
    sigma_hit: 0.2
    recovery_alpha_slow: 0.001
    recovery_alpha_fast: 0.1

map_server:
  ros__parameters:
    use_sim_time: false
    yaml_filename: ""    # supplied via launch arg `map:=`

lifecycle_manager_localization:
  ros__parameters:
    use_sim_time: false
    autostart: true
    bond_timeout: 4.0
    node_names: ["map_server", "amcl"]
```

**`amr_home_manager` extension** — new states bracket the existing `IDLE` state
(current states: `IDLE`, `EXPLORING`, `RETURNING_HOME`, driven by `/amr/command`):

```
LOADING_MAP
  └─ map_server activates, /map published (transient_local, one-shot) ──► LOCALIZING

LOCALIZING
  └─ AMCL converged (covariance trace < threshold for 3s, see §15.2)
     ──► publish /localization_ready = true ──► IDLE
  [robot does NOT accept /amr/command "explore" or NavigateToPose goals
   until this transition -- prevents driving on a bad pose estimate]

IDLE  (existing -- unchanged)
  ├─ /amr/command "explore"            -> existing EXPLORING flow (§10, mapping mode only)
  ├─ /amr/command "go_home"            -> existing RETURNING_HOME flow
  └─ /amr/command "goto:<x>,<y>,<yaw>"  -> NEW: waypoint mission, NavigateToPose
                                            to an arbitrary map-frame pose
```

The `"goto:<x>,<y>,<yaw>"` command is the minimum viable **mission API** — a
single string command is consistent with the existing `/amr/command` interface
and avoids introducing a new message type. A proper mission queue (multiple
ordered waypoints) is a natural Phase 9 extension once this primitive exists.

---

### 15.2 Robustness — Localization Health Watchdog & Relocalization

**Problem:** AMCL can silently diverge (e.g. robot picked up and moved, or a
long featureless corridor). A deployed robot must detect this and recover
without a human resetting its pose in RViz.

Extends `amr_home_manager` — same module that already owns the stall watchdog
(`docs/project_log.md` §33.5), same "detect bad state from data already on the
bus → self-correct → resume" philosophy:

```python
# New subscription
self._amcl_pose_sub = self.create_subscription(
    PoseWithCovarianceStamped, '/amcl_pose', self._on_amcl_pose, 10)

# New timer, same pattern as _check_stall
self.create_timer(2.0, self._check_localization_health)
```

- `_on_amcl_pose`: track the trace of the position covariance (xx + yy).
- `_check_localization_health`: if the trace exceeds `_loc_covariance_threshold`
  (start at `0.5 m²`, tune empirically) for longer than
  `_loc_degraded_timeout_s` (e.g. 10s):
  1. Set state `LOCALIZATION_DEGRADED`, cancel any active `NavigateToPose` goal
     (via `_nav_client`).
  2. Call AMCL's `/reinitialize_global_localization` (`std_srvs/srv/Empty`) to
     spread particles globally.
  3. Issue a single in-place `Spin` behavior (reuse `nav2_behavior_server`'s
     `spin` plugin, already configured in §9) to gather scan diversity from
     multiple headings.
  4. Resume normal monitoring — once covariance drops back below threshold,
     return to `IDLE` and resume the interrupted mission if any.

~40 lines added to an already-tested node — no new sensors, no new dependencies.

---

### 15.3 Battery Monitoring & Auto-Return

**Problem:** Nothing currently measures battery voltage. A deployed robot that
runs the main 3S3P LiPo flat mid-mission requires manual retrieval and risks
over-discharging the cells.

**New hardware:** INA219 current/voltage sensor breakout (~$3-5, I2C, 0-26V
range — comfortably covers the 3S pack's 9.0-12.6V range), wired across the
main battery's output **before** the MDD10A inputs. I2C to RPi5 (default
address `0x40`, configurable via address pins if it conflicts with the IMU bus).

**New package:** `amr_battery`

```
ros2_ws/src/amr_battery/
├── config/
│   └── battery_monitor.yaml
└── amr_battery/
    └── battery_monitor_node.py
```

```python
# battery_monitor_node.py -- outline
# Reads INA219 via smbus2 @ 1Hz, publishes sensor_msgs/msg/BatteryState
# on /battery_state. Voltage thresholds for 3S LiPo (3.0-4.2V/cell):
#   12.6V (4.2V/cell)  = 100% -- full charge
#   10.5V (3.5V/cell)  = LOW_BATTERY threshold  -> trigger auto-return
#   9.9V  (3.3V/cell)  = CRITICAL threshold     -> immediate stop, no further driving
# percentage = clamp((voltage - 9.9) / (12.6 - 9.9), 0.0, 1.0)
```

**`amr_home_manager` extension:**

```
(any state)
  └─ /battery_state.voltage < LOW_BATTERY_V
     AND state not already RETURNING_HOME/CHARGING
     ──► cancel current goal ──► RETURNING_HOME (existing flow, §10) ──► CHARGING

CHARGING
  └─ publish /battery_alert = "LOW_BATTERY: returned home, awaiting charge"
     [robot remains in CHARGING -- ignores /amr/command "explore"/"goto:" until
      voltage recovers above (LOW_BATTERY_V + hysteresis), then ──► IDLE]

(any state)
  └─ /battery_state.voltage < CRITICAL_V
     ──► immediate /cmd_vel_safe override = zero (independent of collision_monitor)
     ──► state CRITICAL (terminal until power-cycle / manual intervention)
```

The `CRITICAL` override publishes directly, bypassing Nav2 entirely — same
"independent safety layer" principle as `nav2_collision_monitor` in §9.

---

### 15.4 Safety Perception — Person Detection & Floor Hazard Detection

Two independent additions, both in a new package **`amr_perception`**, feeding
a single new safety-gating node so `/cmd_vel_safe`'s consumer-side semantics
don't change for `mecanum_drive_controller`:

```
nav2_controller (MPPI) --/cmd_vel--> nav2_collision_monitor --/cmd_vel_monitored-->
                                                                                   |
                                                             perception_safety_node
                                                                                   |
                                                                       /cmd_vel_safe
                                                                                   v
                                                               mecanum_drive_controller
```

Only `nav2_collision_monitor`'s output topic remap changes (`/cmd_vel_safe` →
`/cmd_vel_monitored`); `mecanum_drive_controller`'s subscription stays
`/cmd_vel_safe`, now sourced from `perception_safety_node`. One remap line
changes in `amr_bringup`'s launch file.

#### 15.4.1 Person Detection (Pi HQ Camera)

```
ros2_ws/src/amr_perception/
├── config/
│   └── person_detector.yaml
├── models/
│   └── yolov8n.onnx           # exported once, downloaded at build time
└── amr_perception/
    ├── camera_node.py          # picamera2 -> /camera/image_raw (sensor_msgs/Image), 10-15Hz
    ├── person_detector_node.py # YOLOv8n (ONNX Runtime, ARM CPU) -> /perception/person_detected (Bool)
    └── perception_safety_node.py
```

- `camera_node`: `picamera2` capture at 640×480, ~10-15fps — sufficient for a
  safety trip-wire, not for SLAM.
- `person_detector_node`: YOLOv8n exported to ONNX, `person` class only,
  confidence > 0.5. On Pi 5 CPU, expect ~5-8fps at 640×480 — adequate for a
  safety stop (not a control-loop-rate requirement).
- `perception_safety_node`: if `/perception/person_detected` is `true`,
  publish zero `Twist` on `/cmd_vel_safe` regardless of `/cmd_vel_monitored`
  (latched for `person_clear_hold_s` ≈ 1.0s after the last detection, to avoid
  flicker-driving). Otherwise pass `/cmd_vel_monitored` straight through.

This is **independent of LiDAR** — its value is covering blind spots the LiDAR
can't (e.g. a person crouching below the scan plane, or standing where the
chassis partially occludes the LiDAR's field of view).

#### 15.4.2 Floor Hazard Detection (VL53L5CX, repurposed)

> **Note on history:** the VL53L5CX was previously removed from this project
> entirely (forward-facing low-obstacle role — `docs/project_log.md` §25.6,
> "Architecture Decision — ToF Sensor Removed") because its 8×8/64-zone, 45°
> FOV resolution made it redundant with the 2D LiDAR's obstacle coverage.
> **This is a deliberately different, narrower use case**: mounted at the front
> edge of the chassis, angled ~30° downward, it covers the one blind spot no
> other sensor on this robot can see — the floor immediately in front of the
> wheels, below the LiDAR's fixed horizontal scan plane.

- New URDF frame `floor_tof_link` (amr_description) — distinct mount pose from
  the old (removed) `tof_link`. New entries in §15.9's measurement table:
  mount height + downward angle.
- New node `floor_hazard_node.py` (in `amr_perception`): reads the 8×8 depth
  grid, computes expected floor distance from mount geometry (precomputed
  per-pixel unit vectors, same trig approach as the original §10
  `tof_pointcloud_node` design). If any zone reads **significantly shorter**
  than expected floor distance (low obstacle/cable) **or significantly
  longer / no return** (drop-off/stairs edge), publish
  `/perception/floor_hazard` (`Bool`).
- `perception_safety_node` treats `/perception/floor_hazard` identically to
  `/perception/person_detected` — immediate stop, same latch.

This gives the floor-hazard sensor a single, well-justified job it's actually
good at, instead of the general-purpose obstacle-detection role it was
correctly removed from.

---

### 15.5 Operations Tooling — Diagnostics, Autostart Hardening, CI

#### 15.5.1 Diagnostics Aggregator

**New package:** `amr_diagnostics` — wraps the standard `diagnostic_aggregator`
node, no custom code beyond an `analyzers.yaml`:

```yaml
# amr_diagnostics/config/analyzers.yaml
analyzers:
  ros__parameters:
    sensors:
      type: diagnostic_aggregator/AnalyzerGroup
      path: Sensors
      analyzers:
        lidar:     {type: diagnostic_aggregator/GenericAnalyzer, path: LiDAR,    contains: ['sllidar']}
        imu:       {type: diagnostic_aggregator/GenericAnalyzer, path: IMU,      contains: ['imu_sensor_node']}
        camera:    {type: diagnostic_aggregator/GenericAnalyzer, path: Camera,   contains: ['camera_node', 'person_detector']}
        floor_tof: {type: diagnostic_aggregator/GenericAnalyzer, path: FloorToF, contains: ['floor_hazard_node']}
    localization:
      type: diagnostic_aggregator/AnalyzerGroup
      path: Localization
      analyzers:
        ekf:  {type: diagnostic_aggregator/GenericAnalyzer, path: EKF,  contains: ['ekf_filter_node']}
        amcl: {type: diagnostic_aggregator/GenericAnalyzer, path: AMCL, contains: ['amcl']}
    power:
      type: diagnostic_aggregator/AnalyzerGroup
      path: Power
      analyzers:
        battery: {type: diagnostic_aggregator/GenericAnalyzer, path: Battery, contains: ['battery_monitor_node']}
    actuators:
      type: diagnostic_aggregator/AnalyzerGroup
      path: Actuators
      analyzers:
        firmware: {type: diagnostic_aggregator/GenericAnalyzer, path: ESP32, contains: ['amr_hardware']}
```

`amr_hardware`'s existing serial protocol already defines a `0x06 DIAGNOSTICS`
packet (`batt_mv` + `error_flags`, §5) — wire its handler to publish
`diagnostic_msgs/DiagnosticStatus` so firmware-level health (motor driver
faults, watchdog trips) surfaces on the same `/diagnostics_agg` topic as
everything else. One topic, one place to look.

#### 15.5.2 Autostart Hardening

Extends the existing `amr.service` (§12) — already starts the full stack on
boot. Hardening additions:

```ini
# /etc/systemd/system/amr.service -- additions to existing unit
[Service]
...
WatchdogSec=30
NotifyAccess=all
StandardOutput=journal
StandardError=journal
```

A small `heartbeat_node` (in `amr_diagnostics`) calls `sd_notify(WATCHDOG=1)`
on a timer as long as `/diagnostics_agg` shows no `ERROR`-level entries — if
the stack hangs or a critical node dies, systemd kills and restarts the whole
launch within `WatchdogSec`.

```ini
# /etc/systemd/journald.conf.d/amr.conf
[Journal]
SystemMaxUse=200M     # bound log growth on the Pi's SD card / SSD
```

#### 15.5.3 CI Pipeline

```yaml
# .github/workflows/ci.yml
name: CI
on: [push, pull_request]
jobs:
  build-and-test:
    runs-on: ubuntu-24.04
    container: osrf/ros:jazzy-desktop
    steps:
      - uses: actions/checkout@v4
      - name: colcon build
        run: |
          source /opt/ros/jazzy/setup.bash
          colcon build --symlink-install
      - name: Run tests
        run: |
          source /opt/ros/jazzy/setup.bash
          source install/setup.bash
          PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 colcon test --packages-select \
            amr_home_manager amr_battery amr_perception amr_diagnostics
          colcon test-result --verbose
```

The `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` prefix matches the workaround already
required for local test runs (broken `anyio` pytest plugin in the dev
environment) — keeping CI and local runs consistent avoids "works on CI, fails
locally" or vice versa.

---

### 15.6 Web Dashboard

A non-ROS, non-Python addition outside `ros2_ws/` — does not affect the build
graph above:

```
AMR/
└── dashboard/                  # Next.js app, separate from ros2_ws
    ├── package.json
    └── src/
        ├── app/page.tsx          # main layout
        ├── components/
        │   ├── MapView.tsx          # renders /map OccupancyGrid on <canvas>
        │   ├── RobotPose.tsx        # /amcl_pose or /odom marker over MapView
        │   ├── BatteryWidget.tsx    # /battery_state
        │   ├── HealthPanel.tsx      # /diagnostics_agg, color-coded by level
        │   └── MissionControls.tsx  # buttons -> /amr/command (explore, go_home, goto:x,y,yaw)
        └── lib/
            └── foxgloveClient.ts    # @foxglove/ws-protocol -> ws://amr.local:8765
```

Reuses the **existing `foxglove_bridge`** (§6, already running on `:8765`) as
the sole transport — no `rosbridge_suite`, no second WebSocket server. Served
from the Pi via `npm run build && npm start -- -p 3000`, accessible at
`http://amr.local:3000` from any device on the same network.

---

### 15.7 Validation — Benchmarks, Demo, Technical Writeup

**New script:** `tools/benchmark_run.py` — run during/after a full
`explore_map.launch.py` session, computes from a recorded `ros2 bag`:

| Metric | Source | Computation |
|---|---|---|
| Coverage % | final `/map` | (free cells) / (free + unknown reachable cells) |
| Total path length | `/odom` | sum of consecutive position deltas |
| Exploration time | bag start/end timestamps | `t_exploration_complete - t_start` |
| Localization error (post-§15.1) | `/amcl_pose` vs. known reference points | Euclidean distance at marked checkpoints |
| Battery consumed | `/battery_state` | `voltage_start - voltage_end`, converted via discharge curve |

**Demo video checklist** (single take, ~3-5 min):
1. Power-on → autonomous boot (no SSH) — show `amr.service` status
2. Full exploration run, sped up, with live Foxglove/dashboard overlay
3. Map save + return-home
4. Power-cycle → `localize_map.launch.py` → AMCL converges → `goto:` waypoint mission
5. Person walks into frame → safety stop (§15.4.1)
6. On-screen metrics overlay from `benchmark_run.py`

**Technical writeup** (4-6 pages, arXiv-style): Abstract → System Architecture
(reuse §3 diagram) → Hardware → Key Engineering Challenges (table referencing
`docs/project_log.md` sections — encoder 2x scale, IMU zero-rate drift, TF tree
split, planner/controller kinematic mismatch, costmap race condition) →
Quantitative Results (§15.7 benchmark table) → Limitations & Future Work
(Phase 9: multi-waypoint missions, Gazebo digital twin, semantic mapping).

---

### 15.8 Phase 10 Package & Topic Summary

```
ros2_ws/src/
├── amr_localization/   # NEW -- AMCL + map_server, localize_map.launch.py
├── amr_battery/        # NEW -- INA219 driver, /battery_state
├── amr_perception/     # NEW -- camera, YOLOv8n person detector, floor-hazard ToF, safety gate
└── amr_diagnostics/    # NEW -- diagnostic_aggregator analyzers, heartbeat_node
dashboard/               # NEW -- Next.js, outside ros2_ws, talks to existing foxglove_bridge
```

| Topic | Type | Producer | Consumer | Purpose |
|---|---|---|---|---|
| `/amcl_pose` | PoseWithCovarianceStamped | amcl | amr_home_manager | localization health (§15.2), dashboard pose |
| `/localization_ready` | Bool | amr_home_manager | Foxglove/dashboard | gates mission acceptance |
| `/battery_state` | sensor_msgs/BatteryState | amr_battery | amr_home_manager, dashboard | low-battery auto-return (§15.3) |
| `/battery_alert` | String | amr_home_manager | dashboard | human-readable status |
| `/camera/image_raw` | sensor_msgs/Image | amr_perception | person_detector_node | input to YOLOv8n |
| `/perception/person_detected` | Bool | person_detector_node | perception_safety_node | safety stop trigger |
| `/perception/floor_hazard` | Bool | floor_hazard_node | perception_safety_node | safety stop trigger |
| `/cmd_vel_monitored` | Twist | nav2_collision_monitor | perception_safety_node | renamed from current `/cmd_vel_safe` |
| `/cmd_vel_safe` | Twist | perception_safety_node | mecanum_drive_controller | unchanged consumer-side topic name |
| `/diagnostics_agg` | DiagnosticArray | diagnostic_aggregator | dashboard | single health summary |

---

### 15.9 New Open Measurements (extends §14)

| Parameter | Where Used | How to Measure |
|---|---|---|
| `floor_tof_link` mount height | URDF, floor_hazard_node geometry | Height of VL53L5CX above floor when chassis-mounted |
| `floor_tof_link` downward angle | URDF, floor_hazard_node geometry | Angle from horizontal (~30° target) — measure with protractor/inclinometer |
| `floor_tof_link` x offset | URDF | Forward distance from `base_link` origin to sensor |
| Camera mount position + angle | URDF `camera_link` TF | x/y/z offset and pitch of Pi HQ Camera from `base_link` |
| INA219 I2C address | amr_battery config | `i2cdetect -y 1` on Pi (default `0x40`; check for conflicts with existing IMU bus) |
| 3S LiPo discharge curve | amr_battery thresholds | Either use a generic LiPo curve, or log voltage vs. runtime once for this specific pack |

---

*End of Phase 10 roadmap. Implementation should proceed as a separate plan
(`docs/superpowers/plans/2026-06-1X-phase8-deployment-plan.md`), prioritized
per the table in §15.0, via the writing-plans skill.*

---

*End of design document.*
*Next step: implementation plan via writing-plans skill.*
