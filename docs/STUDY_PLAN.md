# AMR Mastery Study Plan

**Goal:** Go from "knows nothing about embedded/robotics" to "can explain this entire
project's architecture, code, and design decisions in an interview, including
*why* each choice was made and what went wrong along the way."

## How to Use This Plan

- **One session = one sitting (~2 hours).** Don't combine sessions, don't split one
  across two days if avoidable — each is sized to be a complete, satisfying unit.
- **Part A first, in order.** These are general robotics/embedded concepts with
  zero assumed prior knowledge. Each session ends with a "you should now be able
  to explain..." checklist — don't move on until you can answer those out loud,
  from memory, without notes.
- **Part B second, in order.** Each session maps directly onto a real package/file
  in this repo and tells you exactly what to read. By now you have the vocabulary
  from Part A — Part B is "here's how those concepts were actually used, and here's
  the messy reality of why it changed."
- **Format per session:**
  - *Core questions* — what you should be able to answer by the end (the "what is
    X, why does it exist, what problem does it solve" framing you asked for)
  - *Files to read* (Part B only) — real paths in this repo
  - *Why this matters for interviews* — the specific talking point this session
    gives you
  - *Checklist* — explain-out-loud test before moving on

- **Tracking:** check off `[ ]` → `[x]` as you complete sessions. This file is
  yours to mark up.

---

# PART A — Foundations (12 sessions, ~24 hours)

No project-specific content yet. This is the vocabulary and mental models that
make Part B make sense instead of feeling like memorization.

## [ ] A1. Electricity, GPIO & Embedded Basics

**Core questions:**
- What are voltage, current, and power, and why does "common ground" matter when
  two boards share signals? (This project has a literal rule about this — Main
  Battery / MDD10A / ESP32 must share one ground rail.)
- What is a GPIO pin? Digital vs analog I/O. What does "pull-up/pull-down" mean
  and why do floating inputs cause garbage readings?
- What's the difference between a microcontroller (ESP32-P4), an SBC
  (Raspberry Pi 5), and why does a robot use *both*? (real-time vs general-purpose
  compute — this is the single most important architectural idea in this project)
- What does "3.3V logic" mean and why can't you wire 3.3V logic directly to a 12V
  motor driver input without care?

**Why this matters for interviews:** "Why two computers?" is one of the first
questions anyone asks about this robot. You need the real-time vs general-purpose
framing cold.

**Checklist:** Explain why a robot can't run its motor PID loop *inside* ROS2 on
Linux, and why a separate MCU exists for that.

---

## [ ] A2. PWM, Motor Drivers & the LEDC vs MCPWM Question

**Core questions:**
- What is PWM (Pulse Width Modulation)? What do "duty cycle" and "carrier
  frequency" mean physically (what does the motor actually "feel")?
- What is an H-bridge, and what is "sign-magnitude" mode vs "locked anti-phase"
  mode for driving a DC motor (PWM = speed, DIR = direction vs both PWM lines
  active)?
- On the ESP32 family specifically: what is the **LEDC peripheral** (LED Control —
  general-purpose PWM timer/channel hardware) vs the **MCPWM peripheral** (Motor
  Control PWM — has built-in deadtime, fault handling, sync features for motor
  control)? Why would a project pick one over the other, and what's the practical
  difference if you "just want a PWM signal on a pin"?
- What is a "20kHz carrier frequency" choice about — why not 1kHz? (hint: audible
  whine vs switching losses)

**Why this matters for interviews:** This project's firmware debugging history
(`docs/project_log.md` §11-17, "Bug #1: LEDC Channel ≥ 1 Hang") is a real war
story about exactly this peripheral choice. You can't tell that story without
knowing what LEDC *is* first.

**Checklist:** Explain, to someone who's never seen a motor driver, how a
microcontroller makes a DC motor spin at half speed in reverse.

---

## [ ] A3. Encoders, Quadrature Decoding & the PCNT Peripheral

**Core questions:**
- What is an incremental rotary encoder? What does "PPR" (pulses per revolution)
  mean?
- What is quadrature encoding (two channels A/B, 90° out of phase) and why does it
  let you detect *direction*, not just speed? What does "4x decoding" / "x4
  counting" mean and why does it multiply your resolution by 4?
- Given motor PPR + gearbox ratio + wheel diameter, how do you compute
  "millimeters per encoder count"? (This is pure dimensional-analysis math you'll
  redo for this project.)
- What is the ESP32 **PCNT** (Pulse Counter) peripheral, and why is dedicated
  counting hardware better than reading the pins in software at 1kHz?
- How do you go from "raw encoder counts since last tick" to "wheel angular
  velocity in rad/s"?

**Why this matters for interviews:** This project had a real bug where wheel
odometry was 2x under-scaled because of an encoder counting-mode mismatch
(`x2` vs `x4`). You need the quadrature math to even understand what that bug *was*.

**Checklist:** Given PPR, gear ratio, and wheel diameter, derive "mm per count"
from scratch on paper.

---

## [ ] A4. Control Theory — PID From Scratch

**Core questions:**
- What is the difference between open-loop and closed-loop control? Why is "set
  PWM to 50% and hope the wheel goes 1 m/s" open-loop and why is that bad?
- What do the P, I, and D terms in PID *individually* do, intuitively? (P = react
  to current error, I = eliminate steady-state error / "catch up" over time, D =
  dampen oscillation / react to rate of change)
- What is "integral windup" and why does it happen when the output saturates
  (e.g., commanded duty > 1.0)? What is anti-windup / integrator clamping?
- What's the difference between a **velocity control loop** (setpoint = rad/s,
  this project's wheel PID) and a **position control loop**?
- How do you tune PID practically — what is a "step response" and what do you look
  for (overshoot, settling time, oscillation)?

**Why this matters for interviews:** Every wheel in this robot has its own 1kHz
PID loop (`Kp=2.0, Ki=5.0, Kd=0.01`). "Walk me through your control loop" is a
guaranteed embedded/robotics interview question.

**Checklist:** Explain what happens to a wheel's behavior if `Ki` is set to 0, and
separately if `Kp` is set way too high.

---

## [ ] A5. RTOS Concepts — FreeRTOS

**Core questions:**
- What is a "task" in an RTOS, and how is it different from a function call or a
  thread on Linux?
- What does "task priority" mean, and what does "pinned to Core 0" mean on a
  dual-core chip — why would you pin your real-time tasks to one core and leave
  the other core for I/O?
- What is a mutex, and what specific problem does it solve when two tasks
  (encoder-read task and serial-comms task) touch the same shared variable?
- What is a "watchdog timer" and why does "no heartbeat for 2 seconds → zero all
  motor commands" need to live in firmware, independent of ROS2?

**Why this matters for interviews:** This project's firmware layout
(`task_encoder_read` @1kHz/Core0, `task_pid_control` @1kHz/Core0,
`task_serial_comms` @100Hz/Core1) is a textbook RTOS task-partitioning example.

**Checklist:** Explain why putting the serial communication task on the same core
as the PID loop, at the same priority, would be a bad idea.

---

## [ ] A6. Serial Protocols — UART, SPI, I2C & Packet Design

**Core questions:**
- What is UART/serial communication? What does "baud rate" mean, and what limits
  how much data you can send per second?
- What are SPI and I2C, how do they differ from UART (clock line, addressing,
  speed), and when would a sensor use one vs the other?
- What is "USB CDC" (Communications Device Class) vs "USB JTAG" on a chip like the
  ESP32-P4 — why does it matter which one your code talks to?
- What is a packet framing scheme? Why do protocols use header bytes (`0xAA 0x55`),
  a type/length field, and a CRC16 checksum — what specific failure does each part
  protect against?
- What is a "binary protocol" vs sending human-readable text (e.g., JSON) over
  serial, and why would a 1kHz control loop prefer binary?

**Why this matters for interviews:** This project designed a custom binary serial
protocol from scratch (CMD_VEL, STATE, HEARTBEAT, PARAM_SET, DIAGNOSTICS packet
types) and also hit a real USB CDC vs JTAG bug. Protocol design questions are
common in embedded interviews.

**Checklist:** Design (on paper) a minimal packet format for sending "4 floats" of
data reliably over a noisy serial link, and explain what each byte is for.

---

## [ ] A7. Robot Kinematics — Differential Drive vs Mecanum/Holonomic

**Core questions:**
- What is a coordinate frame, and what do `x`, `y`, `yaw (θ)` mean for a 2D ground
  robot's pose?
- What is "holonomic" motion vs "non-holonomic" (differential drive)? Why can a
  car not strafe sideways but a mecanum robot can?
- How does a mecanum wheel physically work (the angled rollers) to produce a
  sideways force component?
- What is **forward kinematics** here: given 4 wheel angular velocities
  (ωFL, ωFR, ωRL, ωRR), how do you compute robot-frame `vx, vy, ωz`? (You'll see
  this exact formula in the project — work through *why* it has those +/- signs.)
- What is odometry, and why does it "drift" over time (integration of small
  errors)?

**Why this matters for interviews:** Mecanum kinematics is a clean, derivable
formula — interviewers love asking you to derive or explain it, and it's the root
of why this project picked the specific planner/controller it did.

**Checklist:** Given 4 wheel speeds, compute `vx, vy, ωz` using the forward
kinematics formula, and explain in words why `ωFL` has the signs it has in each of
the three equations.

---

## [ ] A8. ROS2 Fundamentals Part 1 — Nodes, Topics, Services, Actions

**Core questions:**
- What is a ROS2 **node**? What is a **topic** (pub/sub, many-to-many, fire and
  forget)? What is a **service** (request/response, like a function call)? What is
  an **action** (long-running, with feedback + cancel — e.g., "navigate to this
  pose")?
- What is a **message type** and why does everything need a defined schema
  (`geometry_msgs/Twist`, `sensor_msgs/Imu`, etc.)?
- What is a **launch file**, and why does a real robot need dozens of nodes started
  together with specific parameters?
- What is a **parameter**, and what's a YAML params file for?
- What is a **colcon workspace**, and what does `--symlink-install` do and why is
  it useful during development?

**Why this matters for interviews:** This is baseline ROS2 literacy — you cannot
discuss the node graph without these terms being automatic.

**Checklist:** Without looking anything up, explain why `/cmd_vel` is a topic but
"navigate to this goal pose" is an action, not a topic or service.

---

## [ ] A9. ROS2 Fundamentals Part 2 — TF2, URDF, and ros2_control

**Core questions:**
- What is **TF2** (the transform tree)? What does it mean that `map → odom →
  base_link` is a chain, and why must "each TF edge have exactly one publisher"?
- What is **URDF/xacro**? What is it used for — robot geometry, sensor mount
  offsets, joint definitions?
- What is **ros2_control**? At a high level: what is a **hardware interface**
  (talks to real/simulated actuators), a **controller** (e.g.
  `mecanum_drive_controller` — converts a velocity command into joint commands),
  and the **controller_manager** (orchestrates them)?
- Why does separating "controller" from "hardware interface" let the *exact same*
  controller code run against a real ESP32 over serial *or* a Gazebo simulation?

**Why this matters for interviews:** ros2_control's hardware-abstraction pattern
is exactly the "sim-to-real" story this project tells — and it's a genuinely
important industry pattern, not just project trivia.

**Checklist:** Explain what would need to change (and what would NOT need to
change) to port this robot's *software stack* onto a robot with completely
different motors and a different MCU.

---

## [ ] A10. State Estimation & Sensor Fusion — IMU + EKF

**Core questions:**
- What does an IMU measure — accelerometer (linear acceleration) vs gyroscope
  (angular velocity)? What is "gyro drift" / "zero-rate offset" and why does it
  accumulate into yaw error over time?
- What is a magnetometer, and why might it be *unreliable* near DC motors
  (magnetic interference)?
- At a conceptual level, what does a **Kalman Filter / EKF** do — combining
  multiple noisy, complementary sensor estimates into one better estimate, each
  weighted by how much you trust it?
- What is `imu_filter_madgwick` doing (orientation-only filter using gravity as a
  reference) vs what `robot_localization`'s EKF does (full pose: combines wheel
  odometry + IMU into one `/odom` estimate)? Why two stages instead of one big
  filter?
- What does "two_d_mode" mean for a ground robot's state vector, and why exclude
  Z/roll/pitch entirely instead of just letting them be ~0?

**Why this matters for interviews:** Sensor fusion architecture questions are
extremely common, and this project's "why two stages" + "why disable the
magnetometer" decisions are concrete, defensible answers — not textbook
regurgitation.

**Checklist:** Explain, in plain language, why fusing wheel odometry with an IMU
gives a better yaw estimate than either sensor alone.

---

## [ ] A11. SLAM Fundamentals

**Core questions:**
- What is an **occupancy grid map**? What do cell values (free / occupied /
  unknown) mean?
- What is **SLAM** (Simultaneous Localization and Mapping) — why is it a
  chicken-and-egg problem (you need a map to localize, and a pose to build a map)?
- What is **scan matching** — how does comparing a new LiDAR scan against the
  existing map correct accumulated odometry drift?
- What is **loop closure**, and why does revisiting a previously-mapped area let
  SLAM "snap" the map into better global consistency?
- What's the difference between "mapping mode" (build a new map) and
  "localization mode" (use a fixed pre-built map + estimate pose only, e.g. AMCL /
  particle filter)? (You won't need AMCL details yet — that's Phase 10 — just the
  concept.)

**Why this matters for interviews:** SLAM is often treated as a black box by
people who've only run `slam_toolbox` without understanding it. Being able to
explain *why* it works is a differentiator.

**Checklist:** Explain why a robot's map can "sharpen" or visibly correct itself
when it drives back through a hallway it already mapped.

---

## [ ] A12. Path Planning, Costmaps & Sampling-Based Control

**Core questions:**
- What is a **costmap**, and what are "layers" (static layer from the map,
  obstacle layer from live sensors, inflation layer = safety buffer around
  obstacles)?
- What's the difference between a **global planner** (plans a path across the
  whole map, doesn't need to be fast) and a **local controller** (re-plans/tracks
  in real time, must be fast, reacts to dynamic obstacles)?
- What is a "motion primitive" / "lattice" planner, and why would a planner for a
  holonomic robot need *different* primitives than one for a car-like robot?
- At a conceptual level, what does **MPPI** (Model Predictive Path Integral
  control) do — sample many candidate trajectories, score each with weighted
  "critics" (cost functions for goal distance, obstacles, path following, etc.),
  and execute the best one, every cycle?
- What is **frontier-based exploration** — what is a "frontier" (boundary between
  known-free and unknown space) and why does picking the "best" frontier drive
  autonomous mapping?
- Why have a separate, simple "collision monitor" safety layer that's independent
  of the planner/controller entirely?

**Why this matters for interviews:** This is the conceptual foundation for
*every* Nav2 config decision in Part B — without it, the YAML files are just
magic numbers.

**Checklist:** Explain, without naming any ROS2 package, what "the robot picks the
nearest big unknown area, plans a path to it, drives there while avoiding
obstacles, and repeats until no unknown areas are left" requires as separate
pieces of software.

---

# PART B — This Project, End to End (14 sessions, ~28 hours)

Now we connect every concept from Part A to actual files, configs, and decisions
in this repo — including the places where the real build diverged from the
original design, and why.

## [ ] B1. System Architecture Overview & Key Decisions (Review)

**Files to read:**
- `docs/superpowers/specs/2026-05-17-amr-system-design.md` — sections 1-3, 13
- `docs/project_log.md` — §1-2 (Project Overview, Hardware)

**Core questions:**
- Walk through the three-layer architecture (Dev/Compute/Firmware) and the
  end-to-end data flow diagram (§3) end to end, out loud, from LiDAR scan to motor
  PWM.
- For each row in the "Key Architectural Decisions" table (§13), explain the
  decision *and* the rationale in your own words (not the table's words).
- Note the **spec vs. as-built gaps**: ToF removed, IMU bus/placement, mecanum
  reference topic, measured wheel separation values. We'll resolve each of these
  with real code in later sessions — for now just know they exist.

**Why this matters for interviews:** This is your "tell me about the system"
30-second-to-5-minute answer at every fidelity level.

**Checklist:** Draw the three-layer diagram from memory, with the protocol and
rate annotations (USB serial @921600 baud, 1kHz PID, 100Hz STATE, etc.)

---

## [ ] B2. Hardware, Wiring & Power System

**Files to read:**
- `docs/superpowers/specs/2026-05-17-amr-system-design.md` — §2 (Hardware
  Inventory), §4 (Wiring)
- `docs/project_log.md` — §2-7 (Hardware, GPIO pins, Wiring, Mecanum orientation,
  Motor direction matrix)
- Memory: confirmed safe GPIO pins, wheel separation measurements

**Core questions:**
- Trace the power tree: main LiPo → MDD10A drivers / RPi5 battery → RPi5 → ESP32
  (via USB). Where does each voltage rail come from and what does it power?
- Why must ESP32 GND and both MDD10A GNDs share the main battery's negative
  terminal — what goes wrong electrically if they don't (floating/undefined logic
  levels)?
- Look at the full GPIO assignment table. Cross-reference with
  `docs/project_log.md` §3 (Confirmed Safe GPIO Pins) and §21 (Encoder GPIO
  Selection — Physical Board Constraints) — what GPIOs had to change from the
  original plan, and why (onboard peripheral conflicts on the dev board)?
- What is the mecanum wheel orientation / motor direction matrix (§5-6 of
  project_log) actually encoding, and why does getting this wrong make the robot
  drive diagonally instead of straight?

**Why this matters for interviews:** "Walk me through your wiring/power design and
a constraint you had to work around" — real hardware bring-up stories are gold.

**Checklist:** Explain why a seemingly arbitrary GPIO reassignment (away from the
spec's original pin table) happened, using the actual board-constraint reasoning
from the project log.

---

## [ ] B3. Robot Description — URDF, Xacro & TF Tree

**Files to read:**
- `ros2_ws/src/amr_description/urdf/amr.urdf.xacro`
- `ros2_ws/src/amr_description/urdf/wheels.urdf.xacro`
- `ros2_ws/src/amr_description/urdf/sensors.urdf.xacro`
- `ros2_ws/src/amr_description/launch/view_robot.launch.py`

**Core questions:**
- Identify every `<link>` and `<joint>` — which are fixed (sensor mounts) vs
  continuous (wheels)?
- Find the `<ros2_control>` block — identify the `use_sim` xacro conditional that
  switches between `GazeboSimSystem` and `AMRHardwareInterface`. This is the
  literal sim-to-real boundary from §11 of the spec — find it in real XML.
- How do `base_laser`, `imu_link`, and (originally) `tof_link` frames relate to
  `base_link`? What numeric offsets are used, and where did those numbers come
  from (§14 "Open Measurements")?
- Reconcile with memory: the LiDAR was found to be mounted 180° backwards relative
  to its URDF declaration. Find where that rotation (`rpy`) lives in the xacro and
  understand what value was changed.

**Why this matters for interviews:** URDF/TF correctness is foundational — almost
every downstream bug (SLAM, costmaps, navigation) traces back to a wrong frame
definition, and you have a *real* example of that.

**Checklist:** Given the TF tree diagram from the spec (§6), point to the exact
xacro line that defines each static transform.

---

## [ ] B4. ESP32 Firmware Part 1 — Motor Driver, PID & Encoders

**Files to read:**
- `firmware/main/motor.c` / `motor.h`
- `firmware/main/encoder.c` / `encoder.h`
- `firmware/main/pid.c` / `pid.h`

**Core questions:**
- Open `motor.c` and determine: does this firmware actually use **LEDC** or
  **MCPWM**? (The spec says LEDC; the project log mentions an MCPWM rewrite
  attempt during debugging — find out what's *actually* in the current code and
  why.) What carrier frequency and resolution (`PWM_RES`, `PWM_MAX`) are
  configured?
- In `encoder.c`, find the PCNT setup for the 4 wheels. What counting mode is used
  — and does it match the "x4 decoding" concept from A3, or is this the place the
  2x-scale bug lived? Compute `RAD_PER_COUNT` by hand from the encoder spec
  (A3-style math) and compare it to the constant in the code.
- In `pid.c`, identify the anti-windup mechanism. Match it to the conceptual
  explanation from A4.
- What are `Kp=2.0, Ki=5.0, Kd=0.01` actually doing for a wheel velocity loop —
  plug through a worked example (encoder says wheel is 0.2 rad/s slow — what does
  each PID term contribute?).

**Why this matters for interviews:** This is "show me the code that controls a
motor" — the most concrete embedded question you'll get.

**Checklist:** Trace one full control cycle: encoder count → velocity → PID error
→ PID output → PWM duty + DIR pin state, citing the actual function names.

---

## [ ] B5. ESP32 Firmware Part 2 — FreeRTOS Tasks, Serial Protocol & the Boot-Hang Saga

**Files to read:**
- `firmware/main/main.c`
- `firmware/main/shared_state.h`
- `firmware/components/serial_protocol/serial_protocol.c` (+ header)
- `docs/project_log.md` — §10-17 (ESP-IDF Firmware background, Bugs #1-5,
  Attempts 0-23, Lessons Learned)
- Memory: `project_ledc_ch2_bug`, `project_usb_cdc_vs_jtag`

**Core questions:**
- In `main.c`, identify the 3 tasks (encoder_read, pid_control, serial_comms) —
  what core and priority is each pinned to, matching A5? What's protected by
  `shared_state.h`'s mutex(es)?
- Read the serial packet format in `serial_protocol.c` — identify the header
  bytes, type field, length field, and CRC16. Match each packet type (CMD_VEL,
  STATE, HEARTBEAT, PARAM_SET, DIAGNOSTICS) to A6's framing concepts.
- Now read the **2-day boot-hang debugging saga** (§11-17) as a story, not a
  checklist. By the end you should be able to summarize: what the symptom was,
  which 2-3 theories were seriously investigated, which were ruled out, and what
  the actual root cause + fix turned out to be (per memory: resolved by a clean
  rebuild; LEDC/MCPWM theories were ultimately ruled out).
- Separately: what is the USB CDC vs USB JTAG distinction (memory:
  `project_usb_cdc_vs_jtag`) and why does `task_serial_comms` need to use stdin,
  not `usb_serial_jtag`, for Pi communication?

**Why this matters for interviews:** This is your single best "describe a hard bug
you debugged" story — multi-day, systematic, with ruled-out hypotheses. Practice
telling it in under 3 minutes.

**Checklist:** Tell the boot-hang story out loud, unscripted, in under 3 minutes,
ending with the actual root cause.

---

## [ ] B6. ROS2 Hardware Interface — `amr_hardware`

**Files to read:**
- `ros2_ws/src/amr_hardware/` — find and read the `SerialDriver` and
  `AMRHardwareInterface` source (header + cpp)
- `ros2_ws/src/amr_bringup/` — find the launch file that brings up
  `controller_manager` + `mecanum_drive_controller` for real hardware

**Core questions:**
- How does `AMRHardwareInterface` implement the `SystemInterface` contract from
  A9 — what are `read()` and `write()` doing in terms of the serial protocol from
  B5 (sending CMD_VEL packets, receiving STATE packets)?
- How does `SerialDriver` open and configure `/dev/amr_mcu` (baud rate, framing)?
  Relate this to the udev rule that creates that stable device name.
- Where do `/joint_states` come from, and how does `mecanum_drive_controller`
  consume them to produce `/odom/wheel`?
- This is the literal sim/real swap point from A9 — find the
  `use_sim`/`AMRHardwareInterface` vs `GazeboSimSystem` selection again, now from
  the *controller_manager config* side rather than the URDF side (B3).

**Why this matters for interviews:** This package is the textbook answer to "how
does ROS2 talk to your custom hardware" — a fully concrete ros2_control
SystemInterface implementation.

**Checklist:** Explain the full path of one velocity command: `/cmd_vel_safe` →
`mecanum_drive_controller` → joint velocity interfaces → `AMRHardwareInterface::
write()` → serial CMD_VEL packet → ESP32.

---

## [ ] B7. Sensor Fusion & Localization — IMU, EKF & the Odometry Scale Bug

**Files to read:**
- `ros2_ws/src/amr_imu/amr_imu/imu_sensor_node.py`
- `ros2_ws/src/amr_imu/amr_imu/twist_to_reference.py`
- `ros2_ws/src/amr_imu/amr_imu/cmd_vel_safe_relay.py`
- `ros2_ws/src/amr_sensor_fusion/config/ekf.yaml`
- `ros2_ws/src/amr_sensor_fusion/config/imu_filter.yaml`
- `docs/project_log.md` — §26 (IMU integration + Bug B3 WHO_AM_I), §27.4 (IMU
  replacement), Section 32.3 (yaw drift root cause)
- Memory: `project_odom_scale_fix`, `project_imu_yaw_drift_found`,
  `project_imu_spi_poci_wire_fix`, `project_phase4_complete`

**Core questions:**
- `imu_sensor_node.py`: confirm the IMU is read over **SPI** (mode=0, 500kHz per
  memory) — find where these SPI parameters are set, and relate `WHO_AM_I`
  register checks (B3 bug, §26.6) to "how do you know your sensor wiring is
  correct" debugging.
- `ekf.yaml`: map every field back to A10 — `two_d_mode`, `odom0_config` /
  `imu0_config` boolean masks (which state variables come from which sensor),
  `imu0_remove_gravitational_acceleration`. Why is `base_link_frame` set to
  `base_footprint` (memory: TF tree split fix)?
- The **odometry scale bug** (memory `project_odom_scale_fix`): wheel odometry was
  2x under-scaled due to an encoder x2-vs-x4 counting mismatch, root-caused via
  `RAD_PER_COUNT` (connects back to B4!). Walk through: what symptom would 2x-wrong
  odometry actually produce in a running robot (hint: SLAM/EKF disagreement,
  "nav chaos")?
- The **yaw drift saga** (memory `project_imu_yaw_drift_found`): a wall collision
  was traced to raw gyro zero-rate offset (~0.0077 rad/s Z), fixed via boot
  calibration. Relate this to A10's "gyro drift" concept — why does a *constant*
  small offset cause an *unbounded* yaw error over time (integration)?
- What is `twist_to_reference.py` / `cmd_vel_safe_relay.py` doing, and how does
  this relate to the spec-vs-as-built note "mecanum reference topic =
  TwistStamped" from your memory? What problem does a type-conversion relay node
  solve?

**Why this matters for interviews:** Two real, root-caused sensor bugs with clean
before/after — this is exactly the depth interviewers want on "describe a subtle
bug."

**Checklist:** Explain both the odometry-scale bug and the yaw-drift bug, each in
under 2 minutes, covering symptom → investigation → root cause → fix → validation.

---

## [ ] B8. SLAM — `amr_slam`, slam_toolbox & the TF/LiDAR Fixes

**Files to read:**
- `ros2_ws/src/amr_slam/` — config and launch files
- `docs/superpowers/specs/2026-05-17-amr-system-design.md` — §8
- `docs/project_log.md` — §27 (Phase 5 SLAM), §32.4 (LiDAR 180° bug), §32.5 (TF
  tree split)
- Memory: `project_lidar_tf_fixes_2026-06-08`

**Core questions:**
- Map every `slam_toolbox` parameter to A11 concepts: `resolution`,
  `minimum_travel_distance`/`heading` (when is a new pose node added?),
  `do_loop_closing` + `loop_search_maximum_distance` (loop closure params),
  `map_update_interval`.
- Why does this project run slam_toolbox in **permanent mapping mode** rather than
  switching to localization mode after exploration (§13 decision)? What's the
  tradeoff being accepted, and how does Phase 10 (`amr_localization` + AMCL) plan
  to address it later?
- **The LiDAR 180° bug**: the LiDAR was physically mounted backwards relative to
  its URDF declaration. What does "backwards" actually do to `/scan` data and to
  the resulting map, and why was the fix an `rpy` change in the URDF (B3) rather
  than a code change?
- **The TF tree split bug**: `robot_localization`'s `base_link_frame` conflicted
  with `robot_state_publisher`'s parent for `base_link`, creating two
  disconnected TF trees. Using A9's "each TF edge has exactly one publisher" rule,
  explain *why* two nodes both trying to publish `odom→base_link` (or similar)
  breaks everything downstream, and how renaming to `base_footprint` fixed it.

**Why this matters for interviews:** TF tree bugs are notoriously confusing for
beginners and a great "I understand ROS2 internals" signal when you can explain
one cleanly.

**Checklist:** Draw the *broken* TF tree (two disconnected trees) and the *fixed*
TF tree, and explain the single config change that connected them.

---

## [ ] B9. Navigation Stack — Costmaps, SmacPlannerLattice, MPPI & Collision Monitor

**Files to read:**
- `ros2_ws/src/amr_nav/config/nav2_params.yaml`
- `ros2_ws/src/amr_nav/config/collision_monitor.yaml`
- `ros2_ws/src/amr_nav/config/lattice/output.json` (just skim — it's generated)
- `ros2_ws/src/amr_nav/launch/nav2.launch.py`
- `docs/superpowers/specs/2026-05-17-amr-system-design.md` — §9
- `docs/project_log.md` — §28 (Phase 6 Navigation), §30 (MPPI tuning for Pi 5)

**Core questions:**
- In `nav2_params.yaml`, find the global costmap and local costmap layer configs —
  map each layer (static, obstacle, inflation) to A12. What's the
  `inflation_radius` and how was it derived from the robot's footprint
  dimensions?
- Find the planner config — is it `SmacPlannerLattice` as originally specced, or
  did this change (recall B7's note about `DifferentialMotionModel` appearing in
  the Phase 10 doc — has the motion model changed from `Omni` since the original
  design)? Read the actual current `motion_model` value and reconcile it with
  what A7/A12 told you about holonomic planning. This is an important
  spec-vs-as-built check.
- Find the MPPI critics list — pick 2-3 and explain in your own words what cost
  each one penalizes (e.g., `ObstaclesCritic`, `PathFollowCritic`,
  `VelocityDeadbandCritic`).
- §30.1 "MPPI lightened for Pi 5" — what got reduced, and why (Pi 5 CPU budget vs
  desktop-grade defaults)?
- `collision_monitor.yaml`: find the StopZone and SlowdownZone radii. Why does
  this run independent of, and faster than, the costmap-based planner/controller
  (A12's "separate simple safety layer" idea)?

**Why this matters for interviews:** This session is where "I know what MPPI is"
becomes "I tuned MPPI for a resource-constrained embedded computer and here's
what I changed and why."

**Checklist:** Explain one concrete parameter change made for Pi 5 performance,
what it trades off, and why that tradeoff was acceptable for this robot.

---

## [ ] B10. Autonomous Exploration — `amr_explore` & `amr_home_manager`

**Files to read:**
- `ros2_ws/src/amr_explore/config/explore.yaml`
- `ros2_ws/src/amr_home_manager/amr_home_manager/home_manager_node.py`
- `ros2_ws/src/amr_home_manager/test/test_home_manager.py`
- `docs/project_log.md` — §29 (Phase 7 + Great Localization Debug), §32.6
  (exploration tuning), §33 (premature-stop saga), §34 (recovery-spin + costmap
  QoS bug)
- Memory: `project_explore_premature_stop`, `project_explore_resilience_resume`

**Core questions:**
- In `explore.yaml`, map `potential_scale`, `gain_scale`, `min_frontier_size`,
  `progress_timeout` back to A12's frontier-exploration concept — what does each
  one bias the robot toward or away from?
- Read `home_manager_node.py`'s state machine. List every state and transition.
  How does it map onto the spec's described state machine (§10)? Has it grown new
  states/transitions since the spec was written (memory mentions `/amr/command`
  string-based interface, `go_home`, `resume` — is that in the spec's diagram?)?
- **The premature-stop saga** (§33, memory `project_explore_premature_stop`):
  what were the root causes (motion model mismatch, frontier race condition), and
  what was the fix (`always_send_full_costmap`, `EXPLORATION_STARTED` state
  tracking, stall watchdog in `home_manager_node.py` — find this function in the
  code)?
- **The resilience/resume work** (memory `project_explore_resilience_resume`):
  persistent retry, STUCK escalation, go_home/resume path retrace — find these in
  `home_manager_node.py` and `test_home_manager.py`. Why does a 53/53 passing test
  suite matter for a state machine specifically (state machines have many
  edge-case transitions that are easy to miss by manual testing alone)?
- §34.2 "costmap never reaches explore_lite" — what was the actual bug (a QoS
  setting), and why would a QoS mismatch cause data to silently never arrive
  rather than erroring loudly? (Good general ROS2 lesson.)

**Why this matters for interviews:** This is the most "software engineering" part
of the whole project — state machines, watchdogs, retries, and a real test suite.
Great for "how do you make autonomous systems robust to failure" questions.

**Checklist:** Draw the full `amr_home_manager` state machine from current code
(not just the spec), including the stall-watchdog and resume paths.

---

## [ ] B11. Bringup, Dev Workflow & Deployment

**Files to read:**
- `ros2_ws/src/amr_bringup/launch/` — all launch files, especially
  `explore_map.launch.py` (or current equivalent)
- `scripts/deploy.sh` (or wherever the deploy script lives — find it)
- `docs/superpowers/specs/2026-05-17-amr-system-design.md` — §12
- `docs/project_log.md` — §32.1 (consolidated launch), memory
  `project_pi_power_cable_fix`

**Core questions:**
- Walk the top-level launch file: what nodes/launch-includes does it bring up, and
  in what order? Memory notes a *staggered* startup (Nav2 +15s, explore +23s) —
  find this in the launch file and explain why a fixed delay was used instead of
  an event-based "wait for X to be ready" approach (tradeoffs: simplicity vs
  robustness).
- What does the systemd `amr.service` do (§12), and how does "power on → 15
  seconds → full stack running, no SSH" actually happen mechanically?
- Trace the dev loop (§12 "Daily Iteration Loop"): WSL2 edit → Docker sim → deploy
  → Pi. What does `deploy.sh` actually do (rsync, remote colcon build, restart
  service)?
- **The power cable saga** (memory `project_pi_power_cable_fix`): a Pi 5
  brownout/reboot was traced to the USB cable, not the battery or buck converter.
  Why would a cable cause a *reboot* rather than just "not working" — relate to
  A1 (voltage sag under current draw).

**Why this matters for interviews:** Deployment/ops questions ("how do you get
your code from your laptop onto the robot, and how does the robot start itself
on power-on") come up often for robotics roles with real hardware.

**Checklist:** Explain the full deploy pipeline and the full power-on sequence,
each in under 2 minutes.

---

## [ ] B12. Simulation — Docker, Gazebo & Sim-to-Real Parity

**Files to read:**
- `docker/` — Dockerfile and run script
- `ros2_ws/src/amr_description/` — Gazebo-related xacro (sensor plugin
  definitions)
- `docs/superpowers/specs/2026-05-17-amr-system-design.md` — §11

**Core questions:**
- Why Docker + Gazebo Harmonic + Ubuntu 24.04 specifically — what does "identical
  stack to the RPi5" buy you (A9's sim/real swap, made concrete)?
- How are IMU noise parameters in the Gazebo plugin matched to the real
  ISM330DHCX datasheet — why does this matter for "EKF gains tuned in sim transfer
  to real hardware"?
- What is `m-explore-ros2` and why did it need to be built from source instead of
  installed via apt for Jazzy? (Minor but real — packaging/dependency-availability
  issues are common in ROS2.)

**Why this matters for interviews:** "How do you test robot software without
breaking real hardware" is a practical, frequently-asked question — and
sim-to-real parity is the *correct* answer, not just "I used Gazebo."

**Checklist:** Explain what would need to change if this robot's mecanum wheels
were swapped for differential-drive wheels — which layers (sim, ros2_control,
Nav2 config, EKF) are affected and which are untouched.

---

## [ ] B13. The Debugging Sagas, End to End (Synthesis Session)

**Files to read:** This is a re-read/synthesis session, not new files. Re-skim:
- `docs/project_log.md` §11-17 (boot hang), §26.6 (IMU WHO_AM_I), §29.4-29.5
  (localization debug + refuted hypotheses), §32.2-32.3 (runaway wheel + yaw
  drift), §33 (premature stop)
- Memory: `project_localization_debug`, `project_runaway_wheel_resolved`,
  `project_imu_spi_poci_wire_fix`

**Core questions:**
- For each of the ~6 major bugs in this project's history, write (mentally or on
  paper) one line each: **Symptom → Wrong theory ruled out → Real root cause →
  Fix → How it was validated.**
- Look specifically at the *refuted hypotheses* sections (§29.5, §33.7,
  `project_runaway_wheel_resolved`'s "not hardware, was transient"). Why is
  documenting what *didn't* work valuable — both for this study session and in a
  real engineering team?
- Pick the **3 best stories** for interviews — ones with: a clear symptom, a
  non-obvious root cause, and a validated fix. Practice each as a 90-second
  "tell me about a bug" answer.

**Why this matters for interviews:** "Tell me about a difficult bug" is asked in
almost every technical interview. Having 3 polished, true, technically deep
stories ready — each from a different layer of the stack (firmware, sensor
fusion, navigation) — is one of the highest-leverage things you can walk in with.

**Checklist:** Deliver all 3 chosen bug stories, each under 90 seconds, to an
imaginary interviewer, without reading from notes.

---

## [ ] B14. Phase 10 Roadmap — What's Next, and Why

**Files to read:**
- `docs/superpowers/specs/2026-05-17-amr-system-design.md` — §15 (Phase 10, all
  subsections)

**Core questions:**
- §15.1 AMCL/localization mode: how does this connect back to A11's
  mapping-vs-localization distinction, and to B8's "why permanent mapping mode"
  discussion? What's the "single swappable layer" framing, and how does it mirror
  the sim-to-real boundary from B12?
- §15.2 Localization watchdog: how does this reuse the "detect bad state from data
  already on the bus → self-correct → resume" philosophy from B10's stall
  watchdog?
- §15.3 Battery monitoring: how does the `CRITICAL` override relate to the
  "independent safety layer" pattern you've now seen 3 times (firmware watchdog,
  collision monitor, battery critical)?
- §15.4 Floor hazard ToF: why is this a *deliberately different* use case from the
  originally-removed ToF sensor — what changed about the sensor's role/geometry
  that makes it useful again?
- §15.5-15.7: diagnostics aggregation, autostart hardening, CI, dashboard,
  benchmarking — for each, what gap does it close between "impressive supervised
  demo" and "deployable product"?

**Why this matters for interviews:** "What would you do next / what are the
limitations of your current system" is a near-universal closing question. Having
a *real, written, prioritized* roadmap — with engineering rationale for the
ordering — is a strong signal of product thinking, not just "make it work once."

**Checklist:** Explain the Phase 10 priority ordering (§15.0's table) and justify
*why* localization mode is #1 and the web dashboard is #8 — what's the underlying
prioritization principle (independently shippable, highest-impact-first, safety
before nice-to-have)?

---

# Appendix — Package Map (for quick reference)

| Package | Role |
|---|---|
| `amr_description` | URDF/xacro, sensor frames, ros2_control tags, sim plugins |
| `amr_hardware` | ros2_control SystemInterface, serial driver ↔ ESP32 |
| `amr_bringup` | Top-level launch files, configs |
| `amr_sensor_fusion` | EKF config, IMU filter (Madgwick) config |
| `amr_imu` | IMU sensor node (SPI), cmd_vel relay/conversion nodes |
| `amr_slam` | slam_toolbox config/launch |
| `amr_nav` | Nav2 params, costmaps, collision monitor, lattice |
| `amr_explore` | explore_lite (frontier exploration) config |
| `amr_home_manager` | Custom state-machine node (home pose, explore lifecycle, watchdogs) |
| `firmware/` | ESP-IDF FreeRTOS firmware: encoder, motor, PID, serial protocol |

---

*Total: 26 sessions × ~2 hours = ~52 hours. At one session/day, that's roughly
5-6 weeks to full mastery. Mark sessions complete as you go — and if a session
uncovers something that contradicts this plan (e.g., a file has moved, or a
config value differs from what's described here), that's expected — note it and
keep going; the plan is a map, the code is the territory.*
