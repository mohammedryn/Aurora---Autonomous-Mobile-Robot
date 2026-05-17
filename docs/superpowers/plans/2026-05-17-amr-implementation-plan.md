# AMR Full-Stack Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a complete autonomous mobile robot stack — firmware through navigation — that explores an unknown room, maps it, returns home, and navigates to user-clicked goals.

**Architecture:** ESP32-P4 runs FreeRTOS firmware with a binary serial protocol. RPi5 runs ROS2 Jazzy with ros2_control, slam_toolbox, Nav2 MPPI, and m-explore-ros2. The sim-to-real boundary is a single URDF xacro switch between `gz_ros2_control/GazeboSimSystem` and `amr_hardware/AMRHardwareInterface`.

**Tech Stack:** ESP-IDF v5.x, FreeRTOS, ROS2 Jazzy, ros2_control, slam_toolbox, Nav2 (SmacPlannerLattice + MPPI), robot_localization EKF, m-explore-ros2, Gazebo Harmonic, Foxglove Studio, Docker, Ubuntu 24.04 (RPi5), Ubuntu 22.04 WSL2 (dev).

**Spec:** `docs/superpowers/specs/2026-05-17-amr-system-design.md`

---

## Phase 0 — Repository & Environment
**Milestone:** Git repo initialised, Docker sim launches, RPi5 reachable over SSH, ESP-IDF builds a hello-world.

---

### Task 1: Initialise Git repo and directory structure

**Files:**
- Create: `.gitignore`
- Create: `README.md`

- [ ] **Step 1: Create root directories**

```bash
cd ~/amr
mkdir -p firmware/components firmware/main/tasks
mkdir -p ros2_ws/src
mkdir -p docker scripts
mkdir -p docs/superpowers/specs docs/superpowers/plans
```

- [ ] **Step 2: Write `.gitignore`**

```
# ESP-IDF
firmware/build/
firmware/managed_components/
firmware/sdkconfig.old
firmware/.cache/

# ROS2
ros2_ws/build/
ros2_ws/install/
ros2_ws/log/

# Docker
docker/*.tar

# OS
.DS_Store
__pycache__/
*.pyc
*.egg-info/
```

- [ ] **Step 3: Initialise git and make first commit**

```bash
cd ~/amr
git init
git add .
git commit -m "chore: initialise AMR monorepo"
```

Expected: `[main (root-commit) xxxxxxx] chore: initialise AMR monorepo`

---

### Task 2: Docker simulation environment

**Files:**
- Create: `docker/Dockerfile`
- Create: `docker/run_sim.sh`

- [ ] **Step 1: Write `docker/Dockerfile`**

```dockerfile
FROM osrf/ros:jazzy-desktop

RUN apt-get update && apt-get install -y \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-ros-gz \
    ros-jazzy-nav2-bringup \
    ros-jazzy-slam-toolbox \
    ros-jazzy-robot-localization \
    ros-jazzy-imu-tools \
    ros-jazzy-ros2-controllers \
    ros-jazzy-foxglove-bridge \
    python3-colcon-common-extensions \
    python3-pytest \
    && rm -rf /var/lib/apt/lists/*

# explore_lite is NOT in Jazzy apt — built from source in the workspace
ENV ROS_DOMAIN_ID=42
WORKDIR /amr_ws
```

- [ ] **Step 2: Write `docker/run_sim.sh`**

```bash
#!/bin/bash
set -e
docker run -it --rm \
  --env DISPLAY=$DISPLAY \
  --env ROS_DOMAIN_ID=42 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "$HOME/amr/ros2_ws":/amr_ws \
  --network host \
  amr_sim:latest \
  ros2 launch amr_bringup sim.launch.py
```

```bash
chmod +x docker/run_sim.sh
```

- [ ] **Step 3: Build Docker image and verify**

```bash
cd ~/amr
docker build -t amr_sim:latest -f docker/Dockerfile .
```

Expected: `Successfully tagged amr_sim:latest`

- [ ] **Step 4: Verify ROS2 Jazzy inside container**

```bash
docker run --rm amr_sim:latest ros2 --version
```

Expected: `ros2 1.x.x (jazzy ...)`

- [ ] **Step 5: Commit**

```bash
git add docker/
git commit -m "chore: add Docker sim environment (Jazzy + Gazebo Harmonic)"
```

---

### Task 3: ESP-IDF setup on WSL2

**Files:**
- Create: `scripts/setup_espidf.sh`

- [ ] **Step 1: Write `scripts/setup_espidf.sh`**

```bash
#!/bin/bash
set -e
# Run once on WSL2 Ubuntu 22.04
sudo apt-get update && sudo apt-get install -y \
    git wget flex bison gperf python3 python3-pip python3-venv \
    cmake ninja-build ccache libffi-dev libssl-dev dfu-util libusb-1.0-0

cd ~
git clone --recursive https://github.com/espressif/esp-idf.git --branch v5.3.1
cd esp-idf
./install.sh esp32p4
echo 'alias get_idf=". $HOME/esp-idf/export.sh"' >> ~/.bashrc
source ~/.bashrc
```

- [ ] **Step 2: Run setup and verify**

```bash
chmod +x scripts/setup_espidf.sh
./scripts/setup_espidf.sh
get_idf
idf.py --version
```

Expected: `ESP-IDF v5.3.1`

- [ ] **Step 3: Commit**

```bash
git add scripts/setup_espidf.sh
git commit -m "chore: add ESP-IDF setup script for WSL2"
```

---

### Task 4: RPi5 setup script + udev rules

**Files:**
- Create: `scripts/setup_rpi5.sh`
- Create: `scripts/udev/99-amr.rules`

- [ ] **Step 1: Write `scripts/udev/99-amr.rules`**

```
# Slamtec C1M1 R2 LiDAR (Silicon Labs CP210x)
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", \
  SYMLINK+="lidar", MODE="0666"

# ESP32-P4 USB CDC
SUBSYSTEM=="tty", ATTRS{idVendor}=="303a", ATTRS{idProduct}=="1001", \
  SYMLINK+="amr_mcu", MODE="0666"
```

> Verify vendor/product IDs with `lsusb` on first connection. Update if different.

- [ ] **Step 2: Write `scripts/setup_rpi5.sh`**

```bash
#!/bin/bash
# Run once on RPi5 Ubuntu 24.04 as ubuntu user
set -e

echo "--- Installing ROS2 Jazzy ---"
sudo apt-get update && sudo apt-get install -y software-properties-common curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list

sudo apt-get update && sudo apt-get install -y \
    ros-jazzy-ros-base \
    ros-jazzy-nav2-bringup \
    ros-jazzy-slam-toolbox \
    ros-jazzy-robot-localization \
    ros-jazzy-imu-tools \
    ros-jazzy-ros2-controllers \
    ros-jazzy-ros2-control \
    ros-jazzy-foxglove-bridge \
    python3-colcon-common-extensions \
    python3-rosdep

echo "--- Workspace setup ---"
mkdir -p ~/amr_ws/src
cd ~/amr_ws
rosdep init 2>/dev/null || true
rosdep update

echo "--- udev rules ---"
sudo cp ~/amr/scripts/udev/99-amr.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger

echo "--- ROS2 sourcing ---"
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "source ~/amr_ws/install/setup.bash 2>/dev/null || true" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc

echo "Setup complete. Reboot recommended."
```

- [ ] **Step 3: Write `scripts/deploy.sh`**

```bash
#!/bin/bash
set -e
RPI="ubuntu@amr.local"

echo "--- Syncing source ---"
rsync -avz --delete ~/amr/ros2_ws/src/ $RPI:~/amr_ws/src/

echo "--- Building on RPi5 ---"
ssh $RPI "
  source /opt/ros/jazzy/setup.bash &&
  cd ~/amr_ws &&
  colcon build --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    2>&1 | tail -30
"

echo "--- Restarting service ---"
ssh $RPI "sudo systemctl restart amr.service 2>/dev/null || echo 'Service not yet installed'"
echo "Deploy complete."
```

```bash
chmod +x scripts/setup_rpi5.sh scripts/deploy.sh
```

- [ ] **Step 4: Transfer and run on RPi5**

```bash
# From WSL2
scp scripts/setup_rpi5.sh ubuntu@amr.local:~/
ssh ubuntu@amr.local "bash ~/setup_rpi5.sh"
```

Expected: `Setup complete. Reboot recommended.`

- [ ] **Step 5: Commit**

```bash
git add scripts/
git commit -m "chore: add RPi5 setup script, udev rules, and deploy script"
```

---

## Phase 1 — Robot Description (URDF & TF)
**Milestone:** Full robot TF tree visualised in RViz2/Foxglove with correct sensor frame positions. Works in both sim and real mode via `use_sim:=true/false`.

---

### Task 5: Create `amr_description` package

**Files:**
- Create: `ros2_ws/src/amr_description/package.xml`
- Create: `ros2_ws/src/amr_description/CMakeLists.txt`

- [ ] **Step 1: Create package**

```bash
cd ~/amr/ros2_ws/src
ros2 pkg create amr_description --build-type ament_cmake
```

- [ ] **Step 2: Replace `CMakeLists.txt`**

```cmake
cmake_minimum_required(VERSION 3.8)
project(amr_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf meshes worlds config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()
```

- [ ] **Step 3: Replace `package.xml`**

```xml
<?xml version="1.0"?>
<package format="3">
  <name>amr_description</name>
  <version>0.1.0</version>
  <description>AMR URDF, sensor frames, and Gazebo world</description>
  <maintainer email="khatrishubham030@gmail.com">Shubham</maintainer>
  <license>MIT</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>xacro</exec_depend>
  <exec_depend>joint_state_publisher_gui</exec_depend>
  <ament_cmake>
    <export/>
  </ament_cmake>
</package>
```

- [ ] **Step 4: Create URDF directories**

```bash
mkdir -p ros2_ws/src/amr_description/urdf
mkdir -p ros2_ws/src/amr_description/worlds
```

- [ ] **Step 5: Commit skeleton**

```bash
git add ros2_ws/src/amr_description/
git commit -m "feat: add amr_description package skeleton"
```

---

### Task 6: Write base + wheels URDF

**Files:**
- Create: `ros2_ws/src/amr_description/urdf/amr.urdf.xacro`
- Create: `ros2_ws/src/amr_description/urdf/wheels.urdf.xacro`

- [ ] **Step 1: Write `urdf/wheels.urdf.xacro`**

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

  <xacro:macro name="mecanum_wheel" params="name x y z reflect_y">
    <link name="${name}">
      <visual>
        <geometry><cylinder length="0.04" radius="0.030"/></geometry>
        <origin xyz="0 0 0" rpy="${pi/2} 0 0"/>
        <material name="black"><color rgba="0.1 0.1 0.1 1"/></material>
      </visual>
      <collision>
        <geometry><cylinder length="0.04" radius="0.030"/></geometry>
        <origin xyz="0 0 0" rpy="${pi/2} 0 0"/>
      </collision>
      <inertial>
        <mass value="0.300"/>
        <inertia ixx="0.000900" iyy="0.000900" izz="0.000135"
                 ixy="0" ixz="0" iyz="0"/>
      </inertial>
    </link>

    <joint name="${name}_joint" type="continuous">
      <parent link="base_link"/>
      <child  link="${name}"/>
      <origin xyz="${x} ${y} ${z}" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
      <dynamics damping="0.01" friction="0.1"/>
    </joint>
  </xacro:macro>

</robot>
```

- [ ] **Step 2: Write `urdf/amr.urdf.xacro`**

```xml
<?xml version="1.0"?>
<robot name="amr" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <xacro:arg name="use_sim" default="false"/>
  <xacro:include filename="$(find amr_description)/urdf/wheels.urdf.xacro"/>
  <xacro:include filename="$(find amr_description)/urdf/sensors.urdf.xacro"/>

  <!-- Robot dimensions: 75cm long, 58.5cm wide -->
  <!-- MEASURE lx (half-wheelbase) and ly (half-track) from physical frame -->
  <xacro:property name="lx" value="0.275"/>  <!-- half front-to-rear axle distance — UPDATE -->
  <xacro:property name="ly" value="0.220"/>  <!-- half left-to-right axle distance — UPDATE -->
  <xacro:property name="wheel_z" value="-0.060"/>  <!-- wheel centre height from base_link -->

  <!-- Base footprint -->
  <link name="base_footprint"/>
  <joint name="base_footprint_joint" type="fixed">
    <parent link="base_footprint"/>
    <child  link="base_link"/>
    <origin xyz="0 0 0.080" rpy="0 0 0"/>
  </joint>

  <!-- Main chassis -->
  <link name="base_link">
    <visual>
      <geometry><box size="0.750 0.585 0.060"/></geometry>
      <material name="grey"><color rgba="0.6 0.6 0.6 1"/></material>
    </visual>
    <collision>
      <geometry><box size="0.750 0.585 0.060"/></geometry>
    </collision>
    <inertial>
      <mass value="8.000"/>
      <inertia ixx="0.2282" iyy="0.3750" izz="0.5844"
               ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>

  <!-- Four mecanum wheels: FL, FR, RL, RR -->
  <xacro:mecanum_wheel name="wheel_FL" x=" ${lx}" y=" ${ly}" z="${wheel_z}" reflect_y="1"/>
  <xacro:mecanum_wheel name="wheel_FR" x=" ${lx}" y="-${ly}" z="${wheel_z}" reflect_y="-1"/>
  <xacro:mecanum_wheel name="wheel_RL" x="-${lx}" y=" ${ly}" z="${wheel_z}" reflect_y="1"/>
  <xacro:mecanum_wheel name="wheel_RR" x="-${lx}" y="-${ly}" z="${wheel_z}" reflect_y="-1"/>

  <!-- Sensors -->
  <xacro:include filename="$(find amr_description)/urdf/sensors.urdf.xacro"/>

  <!-- ros2_control hardware interface — switches between sim and real -->
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

    <xacro:macro name="wheel_joint_interface" params="name">
      <joint name="${name}_joint">
        <command_interface name="velocity">
          <param name="min">-37.6</param>
          <param name="max"> 37.6</param>
        </command_interface>
        <state_interface name="velocity"/>
        <state_interface name="position"/>
      </joint>
    </xacro:macro>

    <xacro:wheel_joint_interface name="wheel_FL"/>
    <xacro:wheel_joint_interface name="wheel_FR"/>
    <xacro:wheel_joint_interface name="wheel_RL"/>
    <xacro:wheel_joint_interface name="wheel_RR"/>
  </ros2_control>

</robot>
```

- [ ] **Step 3: Commit**

```bash
git add ros2_ws/src/amr_description/urdf/
git commit -m "feat: add base_link, wheel joints, and ros2_control xacro"
```

---

### Task 7: Write sensor frames URDF

**Files:**
- Create: `ros2_ws/src/amr_description/urdf/sensors.urdf.xacro`

- [ ] **Step 1: Write `urdf/sensors.urdf.xacro`**

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- LiDAR mount — MEASURE actual offsets from frame -->
  <link name="base_laser">
    <visual>
      <geometry><cylinder length="0.04" radius="0.04"/></geometry>
      <material name="black"><color rgba="0.1 0.1 0.1 1"/></material>
    </visual>
  </link>
  <joint name="base_laser_joint" type="fixed">
    <parent link="base_link"/>
    <child  link="base_laser"/>
    <origin xyz="0.0 0.0 0.080" rpy="0 0 0"/>  <!-- UPDATE with measured offset -->
  </joint>

  <!-- IMU mount -->
  <link name="imu_link"/>
  <joint name="imu_joint" type="fixed">
    <parent link="base_link"/>
    <child  link="imu_link"/>
    <origin xyz="0.0 0.0 0.020" rpy="0 0 0"/>  <!-- UPDATE with measured offset -->
  </joint>

  <!-- ToF sensor — forward-facing at front of robot -->
  <link name="tof_link"/>
  <joint name="tof_joint" type="fixed">
    <parent link="base_link"/>
    <child  link="tof_link"/>
    <origin xyz="0.370 0.0 0.080" rpy="0 0 0"/>  <!-- UPDATE with measured offset -->
  </joint>

  <!-- Gazebo sensor plugins (only active when use_sim:=true) -->
  <xacro:if value="$(arg use_sim)">

    <!-- LiDAR -->
    <gazebo reference="base_laser">
      <sensor name="lidar" type="gpu_lidar">
        <gz_frame_id>base_laser</gz_frame_id>
        <topic>/scan</topic>
        <update_rate>10</update_rate>
        <lidar>
          <scan><horizontal>
            <samples>720</samples>
            <resolution>1</resolution>
            <min_angle>-3.14159</min_angle>
            <max_angle>3.14159</max_angle>
          </horizontal></scan>
          <range>
            <min>0.15</min><max>12.0</max><resolution>0.01</resolution>
          </range>
          <noise type="gaussian"><mean>0</mean><stddev>0.01</stddev></noise>
        </lidar>
        <always_on>true</always_on>
        <visualize>false</visualize>
      </sensor>
    </gazebo>

    <!-- IMU -->
    <gazebo reference="imu_link">
      <sensor name="imu_sensor" type="imu">
        <topic>/imu/data_raw</topic>
        <update_rate>100</update_rate>
        <imu>
          <angular_velocity>
            <x><noise type="gaussian"><mean>0</mean><stddev>0.009</stddev></noise></x>
            <y><noise type="gaussian"><mean>0</mean><stddev>0.009</stddev></noise></y>
            <z><noise type="gaussian"><mean>0</mean><stddev>0.009</stddev></noise></z>
          </angular_velocity>
          <linear_acceleration>
            <x><noise type="gaussian"><mean>0</mean><stddev>0.021</stddev></noise></x>
            <y><noise type="gaussian"><mean>0</mean><stddev>0.021</stddev></noise></y>
            <z><noise type="gaussian"><mean>0</mean><stddev>0.021</stddev></noise></z>
          </linear_acceleration>
        </imu>
        <always_on>true</always_on>
      </sensor>
    </gazebo>

    <!-- gz_ros2_control plugin -->
    <gazebo>
      <plugin filename="gz_ros2_control-system"
              name="gz_ros2_control::GazeboSimROS2ControlPlugin">
        <robot_param>robot_description</robot_param>
        <robot_param_node>robot_state_publisher</robot_param_node>
        <parameters>$(find amr_bringup)/config/controllers.yaml</parameters>
      </plugin>
    </gazebo>

  </xacro:if>

</robot>
```

- [ ] **Step 2: Commit**

```bash
git add ros2_ws/src/amr_description/urdf/sensors.urdf.xacro
git commit -m "feat: add sensor frames and Gazebo plugins to URDF"
```

---

### Task 8: Verify TF tree with robot_state_publisher

**Files:**
- Create: `ros2_ws/src/amr_description/launch/view_robot.launch.py`

- [ ] **Step 1: Write `launch/view_robot.launch.py`**

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    urdf_file = PathJoinSubstitution([
        FindPackageShare('amr_description'), 'urdf', 'amr.urdf.xacro'
    ])
    robot_description = Command([
        FindExecutable(name='xacro'), ' ', urdf_file,
        ' use_sim:=false'
    ])

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
        ),
    ])
```

- [ ] **Step 2: Build and launch**

On RPi5 (or Docker):
```bash
cd ~/amr_ws
colcon build --packages-select amr_description --symlink-install
source install/setup.bash
ros2 launch amr_description view_robot.launch.py
```

- [ ] **Step 3: Verify TF tree**

In a second terminal:
```bash
ros2 run tf2_tools view_frames
```

Expected output file `frames.pdf` must show:
```
base_footprint → base_link → wheel_FL, wheel_FR, wheel_RL, wheel_RR
                           → base_laser
                           → imu_link
                           → tof_link
```

If any frame is missing, check joint names in the URDF.

- [ ] **Step 4: Commit**

```bash
git add ros2_ws/src/amr_description/launch/
git commit -m "feat: add view_robot launch and verify TF tree"
```

---

## Phase 2 — ESP32-P4 Firmware
**Milestone:** ESP32-P4 flashed, streaming valid STATE + TOF_DATA packets over USB serial, wheels spin at commanded velocities.

---

### Task 9: ESP-IDF project init + sdkconfig

**Files:**
- Create: `firmware/CMakeLists.txt`
- Create: `firmware/sdkconfig.defaults`
- Create: `firmware/main/CMakeLists.txt`

- [ ] **Step 1: Write `firmware/CMakeLists.txt`**

```cmake
cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(amr_firmware)
```

- [ ] **Step 2: Write `firmware/sdkconfig.defaults`**

```
CONFIG_IDF_TARGET="esp32p4"
CONFIG_ESP_WIFI_ENABLED=n
CONFIG_BT_ENABLED=n
CONFIG_ESP_CONSOLE_USB_CDC=y
CONFIG_FREERTOS_HZ=1000
CONFIG_FREERTOS_UNICORE=n
CONFIG_ESP_MAIN_TASK_STACK_SIZE=4096
CONFIG_LOG_DEFAULT_LEVEL_INFO=y
```

- [ ] **Step 3: Write `firmware/main/CMakeLists.txt`**

```cmake
idf_component_register(
    SRCS
        "main.c"
        "encoder.c"
        "motor.c"
        "pid.c"
        "shared_state.c"
        "tasks/task_encoder_read.c"
        "tasks/task_pid_control.c"
        "tasks/task_imu_read.c"
        "tasks/task_tof_read.c"
        "tasks/task_serial_comms.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_timer freertos serial_protocol ism330dhcx vl53l5cx
)
```

- [ ] **Step 4: Verify project configures**

```bash
cd ~/amr/firmware && get_idf
idf.py set-target esp32p4
idf.py menuconfig   # open and exit — confirms sdkconfig is valid
```

Expected: menuconfig opens without errors, target shows esp32p4.

- [ ] **Step 5: Commit**

```bash
git add firmware/CMakeLists.txt firmware/sdkconfig.defaults firmware/main/CMakeLists.txt
git commit -m "feat: init ESP-IDF project for ESP32-P4"
```

---

### Task 10: `serial_protocol` component + unit tests

**Files:**
- Create: `firmware/components/serial_protocol/CMakeLists.txt`
- Create: `firmware/components/serial_protocol/include/serial_protocol.h`
- Create: `firmware/components/serial_protocol/serial_protocol.c`
- Create: `firmware/components/serial_protocol/test/test_serial_protocol.c`

- [ ] **Step 1: Write `CMakeLists.txt`**

```cmake
idf_component_register(SRCS "serial_protocol.c" INCLUDE_DIRS "include")
```

- [ ] **Step 2: Write `include/serial_protocol.h`**

```c
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#define PROTO_HEADER_0       0xAA
#define PROTO_HEADER_1       0x55
#define PROTO_TYPE_CMD_VEL   0x01
#define PROTO_TYPE_STATE     0x02
#define PROTO_TYPE_TOF_DATA  0x03
#define PROTO_TYPE_HEARTBEAT 0x04
#define PROTO_TYPE_PARAM_SET 0x05
#define PROTO_TYPE_DIAG      0x06

typedef struct __attribute__((packed)) {
    uint32_t timestamp_ms;
    int32_t  enc_delta[4];  /* FL FR RL RR counts */
    float    accel[3];      /* m/s2 */
    float    gyro[3];       /* rad/s */
} proto_state_t;            /* 44 bytes */

typedef struct __attribute__((packed)) {
    uint16_t distances[64]; /* mm, row-major 8x8 */
} proto_tof_t;              /* 128 bytes */

typedef struct __attribute__((packed)) {
    float omega[4];         /* FL FR RL RR rad/s */
} proto_cmd_vel_t;          /* 16 bytes */

typedef struct __attribute__((packed)) {
    uint8_t param_id;
    float   value;
} proto_param_set_t;

typedef struct __attribute__((packed)) {
    uint16_t batt_mv;
    uint8_t  error_flags;
} proto_diag_t;

uint16_t protocol_crc16(const uint8_t *data, size_t len);
int      protocol_encode(uint8_t type, const void *payload, uint8_t payload_len,
                         uint8_t *buf, size_t buf_size);
bool     protocol_decode(const uint8_t *buf, size_t len, size_t *consumed,
                         uint8_t *out_type, void *out_payload, uint8_t *out_len);
```

- [ ] **Step 3: Write `serial_protocol.c`**

```c
#include "serial_protocol.h"

uint16_t protocol_crc16(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; j++)
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
    }
    return crc;
}

int protocol_encode(uint8_t type, const void *payload, uint8_t payload_len,
                    uint8_t *buf, size_t buf_size) {
    size_t frame_len = 6u + payload_len;
    if (buf_size < frame_len) return -1;
    buf[0] = PROTO_HEADER_0; buf[1] = PROTO_HEADER_1;
    buf[2] = type;           buf[3] = payload_len;
    if (payload_len && payload) memcpy(buf + 4, payload, payload_len);
    uint16_t crc = protocol_crc16(buf + 2, 2 + payload_len);
    buf[4 + payload_len] = (crc >> 8) & 0xFF;
    buf[5 + payload_len] =  crc       & 0xFF;
    return (int)frame_len;
}

bool protocol_decode(const uint8_t *buf, size_t len, size_t *consumed,
                     uint8_t *out_type, void *out_payload, uint8_t *out_len) {
    for (size_t i = 0; i + 5 < len; i++) {
        if (buf[i] != PROTO_HEADER_0 || buf[i+1] != PROTO_HEADER_1) continue;
        uint8_t plen = buf[i+3];
        size_t flen = 6u + plen;
        if (i + flen > len) return false;
        uint16_t exp = protocol_crc16(buf + i + 2, 2 + plen);
        uint16_t got = ((uint16_t)buf[i+4+plen] << 8) | buf[i+5+plen];
        if (exp != got) continue;
        *out_type = buf[i+2]; *out_len = plen;
        if (plen && out_payload) memcpy(out_payload, buf + i + 4, plen);
        *consumed = i + flen;
        return true;
    }
    return false;
}
```

- [ ] **Step 4: Write `test/test_serial_protocol.c`**

```c
#include "unity.h"
#include "serial_protocol.h"

TEST_CASE("encode-decode roundtrip CMD_VEL", "[serial_protocol]") {
    proto_cmd_vel_t cmd = {{1.0f, 2.0f, -1.0f, -2.0f}};
    uint8_t buf[32];
    int len = protocol_encode(PROTO_TYPE_CMD_VEL, &cmd, sizeof(cmd), buf, sizeof(buf));
    TEST_ASSERT_EQUAL_INT(22, len);
    uint8_t type, payload[32], plen; size_t consumed;
    TEST_ASSERT_TRUE(protocol_decode(buf, len, &consumed, &type, payload, &plen));
    TEST_ASSERT_EQUAL_UINT8(PROTO_TYPE_CMD_VEL, type);
    TEST_ASSERT_EQUAL_UINT8(sizeof(cmd), plen);
    TEST_ASSERT_EQUAL_MEMORY(&cmd, payload, sizeof(cmd));
    TEST_ASSERT_EQUAL_size_t(22, consumed);
}

TEST_CASE("corrupt CRC is rejected", "[serial_protocol]") {
    proto_cmd_vel_t cmd = {{0.5f, 0.5f, 0.5f, 0.5f}};
    uint8_t buf[32];
    int len = protocol_encode(PROTO_TYPE_CMD_VEL, &cmd, sizeof(cmd), buf, sizeof(buf));
    buf[len - 1] ^= 0xFF;
    uint8_t type, payload[32], plen; size_t consumed;
    TEST_ASSERT_FALSE(protocol_decode(buf, len, &consumed, &type, payload, &plen));
}

TEST_CASE("incomplete packet returns false", "[serial_protocol]") {
    proto_cmd_vel_t cmd = {{1.0f, 0, 0, 0}};
    uint8_t buf[32];
    int len = protocol_encode(PROTO_TYPE_CMD_VEL, &cmd, sizeof(cmd), buf, sizeof(buf));
    uint8_t type, payload[32], plen; size_t consumed;
    TEST_ASSERT_FALSE(protocol_decode(buf, len / 2, &consumed, &type, payload, &plen));
}

TEST_CASE("STATE packet encodes to 50 bytes", "[serial_protocol]") {
    proto_state_t s = {0};
    uint8_t buf[64];
    int len = protocol_encode(PROTO_TYPE_STATE, &s, sizeof(s), buf, sizeof(buf));
    TEST_ASSERT_EQUAL_INT(50, len);
}
```

- [ ] **Step 5: Run tests**

```bash
cd ~/amr/firmware
idf.py -C components/serial_protocol/test build flash monitor -p /dev/ttyACM0
```

Expected: `4 Tests 0 Failures 0 Ignored`

- [ ] **Step 6: Commit**

```bash
git add firmware/components/serial_protocol/
git commit -m "feat: serial_protocol component with CRC16 encode/decode and tests"
```

---

### Task 11: Encoder PCNT + Motor PWM + PID

**Files:**
- Create: `firmware/main/encoder.h` + `encoder.c`
- Create: `firmware/main/motor.h` + `motor.c`
- Create: `firmware/main/pid.h` + `pid.c`

- [ ] **Step 1: Write `encoder.h`**

```c
#pragma once
#include <stdint.h>

#define MOT_FL 0
#define MOT_FR 1
#define MOT_RL 2
#define MOT_RR 3

void encoder_init(void);
void encoder_get_deltas(int32_t deltas[4]); /* thread-safe, resets on read */
```

- [ ] **Step 2: Write `encoder.c`**

```c
#include "encoder.h"
#include "driver/pulse_cnt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

/* Verify GPIO numbers against Waveshare ESP32-P4-WIFI6 silkscreen */
static const int GPIO_A[] = {12, 14, 16, 18};
static const int GPIO_B[] = {13, 15, 17, 19};
static pcnt_unit_handle_t s_units[4];
static int32_t s_last[4];
static SemaphoreHandle_t s_mutex;

static void init_unit(int idx) {
    pcnt_unit_config_t uc = {.low_limit = -32768, .high_limit = 32767};
    pcnt_new_unit(&uc, &s_units[idx]);
    pcnt_chan_config_t ca = {.edge_gpio_num = GPIO_A[idx], .level_gpio_num = GPIO_B[idx]};
    pcnt_chan_config_t cb = {.edge_gpio_num = GPIO_B[idx], .level_gpio_num = GPIO_A[idx]};
    pcnt_channel_handle_t cha, chb;
    pcnt_new_channel(s_units[idx], &ca, &cha);
    pcnt_new_channel(s_units[idx], &cb, &chb);
    pcnt_channel_set_edge_action(cha, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE);
    pcnt_channel_set_level_action(cha, PCNT_CHANNEL_LEVEL_ACTION_KEEP,   PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
    pcnt_channel_set_edge_action(chb, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE);
    pcnt_channel_set_level_action(chb, PCNT_CHANNEL_LEVEL_ACTION_KEEP,   PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
    pcnt_unit_enable(s_units[idx]);
    pcnt_unit_clear_count(s_units[idx]);
    pcnt_unit_start(s_units[idx]);
}

void encoder_init(void) {
    s_mutex = xSemaphoreCreateMutex();
    for (int i = 0; i < 4; i++) { init_unit(i); s_last[i] = 0; }
}

void encoder_get_deltas(int32_t deltas[4]) {
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    for (int i = 0; i < 4; i++) {
        int raw = 0;
        pcnt_unit_get_count(s_units[i], &raw);
        deltas[i] = (int32_t)raw - s_last[i];
        s_last[i] = (int32_t)raw;
    }
    xSemaphoreGive(s_mutex);
}
```

- [ ] **Step 3: Write `motor.h`**

```c
#pragma once
void motor_init(void);
void motor_set_duty(int motor_idx, float duty); /* duty in [-1.0, 1.0] */
void motor_stop_all(void);
```

- [ ] **Step 4: Write `motor.c`**

```c
#include "motor.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include <math.h>

#define PWM_FREQ   20000
#define PWM_RES    LEDC_TIMER_12_BIT
#define PWM_MAX    4095u

static const int PWM_GPIO[] = {4,  6,  8,  10};  /* Verify against board */
static const int DIR_GPIO[] = {5,  7,  9,  11};

void motor_init(void) {
    ledc_timer_config_t t = {.speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0, .duty_resolution = PWM_RES,
        .freq_hz = PWM_FREQ, .clk_cfg = LEDC_AUTO_CLK};
    ledc_timer_config(&t);
    for (int i = 0; i < 4; i++) {
        ledc_channel_config_t ch = {.speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = (ledc_channel_t)i, .timer_sel = LEDC_TIMER_0,
            .gpio_num = PWM_GPIO[i], .duty = 0, .hpoint = 0};
        ledc_channel_config(&ch);
        gpio_config_t d = {.pin_bit_mask = 1ULL << DIR_GPIO[i], .mode = GPIO_MODE_OUTPUT};
        gpio_config(&d);
        gpio_set_level(DIR_GPIO[i], 0);
    }
}

void motor_set_duty(int idx, float duty) {
    if (duty >  1.0f) duty =  1.0f;
    if (duty < -1.0f) duty = -1.0f;
    gpio_set_level(DIR_GPIO[idx], duty >= 0.0f ? 1 : 0);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)idx, (uint32_t)(fabsf(duty)*PWM_MAX));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)idx);
}

void motor_stop_all(void) { for (int i=0;i<4;i++) motor_set_duty(i,0.0f); }
```

- [ ] **Step 5: Write `pid.h` + `pid.c`**

```c
/* pid.h */
#pragma once
typedef struct { float kp,ki,kd,integral,prev_error,out_min,out_max,dt; } pid_t;
void  pid_init(pid_t *p,float kp,float ki,float kd,float dt,float mn,float mx);
void  pid_reset(pid_t *p);
float pid_update(pid_t *p,float setpoint,float measured);
```

```c
/* pid.c */
#include "pid.h"
void pid_init(pid_t *p,float kp,float ki,float kd,float dt,float mn,float mx){
    p->kp=kp;p->ki=ki;p->kd=kd;p->dt=dt;p->out_min=mn;p->out_max=mx;
    p->integral=0;p->prev_error=0;
}
void pid_reset(pid_t *p){p->integral=0;p->prev_error=0;}
float pid_update(pid_t *p,float sp,float meas){
    float e=sp-meas, d=(e-p->prev_error)/p->dt;
    p->prev_error=e;
    float out=p->kp*e+p->ki*p->integral+p->kd*d;
    if(out>p->out_min&&out<p->out_max) p->integral+=e*p->dt;
    if(out>p->out_max) out=p->out_max;
    if(out<p->out_min) out=p->out_min;
    return out;
}
```

- [ ] **Step 6: Commit**

```bash
git add firmware/main/encoder.h firmware/main/encoder.c \
        firmware/main/motor.h firmware/main/motor.c \
        firmware/main/pid.h firmware/main/pid.c
git commit -m "feat: PCNT encoder, LEDC motor driver, and PID controller"
```

---

### Task 12: ISM330DHCX SPI driver

**Files:**
- Create: `firmware/components/ism330dhcx/CMakeLists.txt`
- Create: `firmware/components/ism330dhcx/include/ism330dhcx.h`
- Create: `firmware/components/ism330dhcx/ism330dhcx.c`

- [ ] **Step 1: Write `CMakeLists.txt`**

```cmake
idf_component_register(SRCS "ism330dhcx.c" INCLUDE_DIRS "include" REQUIRES driver)
```

- [ ] **Step 2: Write `include/ism330dhcx.h`**

```c
#pragma once
#include <stdbool.h>

typedef struct { float accel[3]; float gyro[3]; } ism330dhcx_data_t;

bool ism330dhcx_init(void);
bool ism330dhcx_read(ism330dhcx_data_t *out);
void ism330dhcx_calibrate_gyro(void); /* call once with robot stationary */
```

- [ ] **Step 3: Write `ism330dhcx.c`**

```c
#include "ism330dhcx.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <string.h>

#define PIN_MOSI 36
#define PIN_MISO 37
#define PIN_SCLK 38
#define PIN_CS   39

#define REG_WHO_AM_I 0x0F  /* expect 0x6B */
#define REG_CTRL1_XL 0x10
#define REG_CTRL2_G  0x11
#define REG_OUTX_L_G 0x22  /* gyro X low (6 bytes) */
#define REG_OUTX_L_A 0x28  /* accel X low (6 bytes) */
#define READ_FLAG    0x80

#define ACCEL_SENS (0.000122f * 9.80665f)      /* +-4g, LSB -> m/s2 */
#define GYRO_SENS  (0.070f * 3.14159265f/180.f) /* +-2000dps, LSB -> rad/s */

static spi_device_handle_t s_spi;
static float s_gyro_bias[3];

static uint8_t reg_read(uint8_t addr) {
    uint8_t tx[2] = {addr|READ_FLAG, 0}, rx[2];
    spi_transaction_t t = {.length=16,.tx_buffer=tx,.rx_buffer=rx};
    spi_device_transmit(s_spi, &t);
    return rx[1];
}
static void reg_write(uint8_t addr, uint8_t val) {
    uint8_t tx[2] = {addr&0x7F, val};
    spi_transaction_t t = {.length=16,.tx_buffer=tx};
    spi_device_transmit(s_spi, &t);
}
static void read_6(uint8_t addr, int16_t out[3]) {
    uint8_t tx[7]={addr|READ_FLAG}, rx[7];
    spi_transaction_t t={.length=56,.tx_buffer=tx,.rx_buffer=rx};
    spi_device_transmit(s_spi, &t);
    for(int i=0;i<3;i++) out[i]=(int16_t)((rx[2*i+2]<<8)|rx[2*i+1]);
}

bool ism330dhcx_init(void) {
    spi_bus_config_t bc={.mosi_io_num=PIN_MOSI,.miso_io_num=PIN_MISO,
        .sclk_io_num=PIN_SCLK,.quadwp_io_num=-1,.quadhd_io_num=-1};
    spi_device_interface_config_t dc={.clock_speed_hz=8000000,.mode=3,
        .spics_io_num=PIN_CS,.queue_size=1};
    spi_bus_initialize(SPI2_HOST, &bc, SPI_DMA_CH_AUTO);
    spi_bus_add_device(SPI2_HOST, &dc, &s_spi);
    if(reg_read(REG_WHO_AM_I)!=0x6B) return false;
    reg_write(REG_CTRL1_XL, 0x4A); /* 104Hz, +-4g */
    reg_write(REG_CTRL2_G,  0x4C); /* 104Hz, +-2000dps */
    vTaskDelay(pdMS_TO_TICKS(20));
    return true;
}

bool ism330dhcx_read(ism330dhcx_data_t *out) {
    int16_t rg[3],ra[3];
    read_6(REG_OUTX_L_G,rg); read_6(REG_OUTX_L_A,ra);
    for(int i=0;i<3;i++){
        out->gyro[i]  = rg[i]*GYRO_SENS  - s_gyro_bias[i];
        out->accel[i] = ra[i]*ACCEL_SENS;
    }
    return true;
}

void ism330dhcx_calibrate_gyro(void) {
    double sum[3]={0}; const int N=500;
    for(int n=0;n<N;n++){
        ism330dhcx_data_t d; ism330dhcx_read(&d);
        for(int i=0;i<3;i++) sum[i]+=d.gyro[i];
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    for(int i=0;i<3;i++) s_gyro_bias[i]=(float)(sum[i]/N);
}
```

- [ ] **Step 4: Commit**

```bash
git add firmware/components/ism330dhcx/
git commit -m "feat: ISM330DHCX SPI driver with WHO_AM_I check and gyro calibration"
```

---

### Task 13: VL53L5CX I2C driver

**Files:**
- Create: `firmware/components/vl53l5cx/CMakeLists.txt`
- Create: `firmware/components/vl53l5cx/include/vl53l5cx_drv.h`
- Create: `firmware/components/vl53l5cx/vl53l5cx_drv.c`
- Create: `firmware/components/vl53l5cx/vl53l5cx_platform.c`

- [ ] **Step 1: Clone ST ULD**

```bash
cd ~/amr/firmware/components/vl53l5cx
git clone https://github.com/stm32duino/VL53L5CX st_uld --depth=1
cp st_uld/src/vl53l5cx_api.c .
cp st_uld/src/vl53l5cx_plugin_detection_thresholds.c .
cp -r st_uld/src/include .
```

- [ ] **Step 2: Write `include/vl53l5cx_drv.h`**

```c
#pragma once
#include <stdint.h>
#include <stdbool.h>
bool     vl53l5cx_drv_init(void);
bool     vl53l5cx_drv_read(uint16_t distances[64]); /* mm, row-major */
```

- [ ] **Step 3: Write `vl53l5cx_platform.c`** (ESP-IDF I2C shim for ST ULD)

```c
#include "include/vl53l5cx_platform.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define I2C_PORT  I2C_NUM_0
#define PIN_SDA   22
#define PIN_SCL   23
#define I2C_FREQ  400000

void vl53l5cx_platform_init(void) {
    i2c_config_t cfg = {.mode=I2C_MODE_MASTER,.sda_io_num=PIN_SDA,
        .scl_io_num=PIN_SCL,.sda_pullup_en=GPIO_PULLUP_ENABLE,
        .scl_pullup_en=GPIO_PULLUP_ENABLE,.master.clk_speed=I2C_FREQ};
    i2c_param_config(I2C_PORT, &cfg);
    i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
}

uint8_t VL53L5CX_RdMulti(VL53L5CX_Platform *p,uint16_t reg,uint8_t *buf,uint32_t len){
    uint8_t rb[2]={reg>>8,reg&0xFF};
    i2c_cmd_handle_t c=i2c_cmd_link_create();
    i2c_master_start(c);
    i2c_master_write_byte(c,(p->address<<1)|I2C_MASTER_WRITE,true);
    i2c_master_write(c,rb,2,true);
    i2c_master_start(c);
    i2c_master_write_byte(c,(p->address<<1)|I2C_MASTER_READ,true);
    i2c_master_read(c,buf,len,I2C_MASTER_LAST_NACK);
    i2c_master_stop(c);
    esp_err_t r=i2c_master_cmd_begin(I2C_PORT,c,pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(c);
    return r==ESP_OK?0:1;
}
uint8_t VL53L5CX_WrMulti(VL53L5CX_Platform *p,uint16_t reg,uint8_t *buf,uint32_t len){
    i2c_cmd_handle_t c=i2c_cmd_link_create();
    i2c_master_start(c);
    i2c_master_write_byte(c,(p->address<<1)|I2C_MASTER_WRITE,true);
    uint8_t rb[2]={reg>>8,reg&0xFF};
    i2c_master_write(c,rb,2,true);
    i2c_master_write(c,buf,len,true);
    i2c_master_stop(c);
    esp_err_t r=i2c_master_cmd_begin(I2C_PORT,c,pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(c);
    return r==ESP_OK?0:1;
}
uint8_t VL53L5CX_RdByte(VL53L5CX_Platform *p,uint16_t a,uint8_t *v){return VL53L5CX_RdMulti(p,a,v,1);}
uint8_t VL53L5CX_WrByte(VL53L5CX_Platform *p,uint16_t a,uint8_t v){return VL53L5CX_WrMulti(p,a,&v,1);}
void    VL53L5CX_WaitMs(VL53L5CX_Platform *p,uint32_t ms){(void)p;vTaskDelay(pdMS_TO_TICKS(ms));}
```

- [ ] **Step 4: Write `vl53l5cx_drv.c`**

```c
#include "vl53l5cx_drv.h"
#include "include/vl53l5cx_api.h"
#include "vl53l5cx_platform.h"
#include <string.h>

static VL53L5CX_Configuration s_dev;

bool vl53l5cx_drv_init(void) {
    vl53l5cx_platform_init();
    s_dev.platform.address = VL53L5CX_DEFAULT_I2C_ADDRESS;
    uint8_t alive = 0;
    if (vl53l5cx_is_alive(&s_dev, &alive) || !alive) return false;
    if (vl53l5cx_init(&s_dev))                       return false;
    if (vl53l5cx_set_resolution(&s_dev, VL53L5CX_RESOLUTION_8X8)) return false;
    if (vl53l5cx_set_ranging_frequency_hz(&s_dev, 10)) return false;
    if (vl53l5cx_start_ranging(&s_dev))              return false;
    return true;
}

bool vl53l5cx_drv_read(uint16_t distances[64]) {
    uint8_t ready = 0;
    vl53l5cx_check_data_ready(&s_dev, &ready);
    if (!ready) return false;
    VL53L5CX_ResultsData res;
    if (vl53l5cx_get_ranging_data(&s_dev, &res)) return false;
    for (int i = 0; i < 64; i++)
        distances[i] = (uint16_t)(res.distance_mm[i] < 0 ? 0 : res.distance_mm[i]);
    return true;
}
```

- [ ] **Step 5: Write `CMakeLists.txt`**

```cmake
idf_component_register(
    SRCS "vl53l5cx_api.c"
         "vl53l5cx_plugin_detection_thresholds.c"
         "vl53l5cx_platform.c"
         "vl53l5cx_drv.c"
    INCLUDE_DIRS "include"
    REQUIRES driver
)
```

- [ ] **Step 6: Commit**

```bash
git add firmware/components/vl53l5cx/
git commit -m "feat: VL53L5CX I2C driver with ST ULD and ESP-IDF platform shim"
```

---

### Task 14: FreeRTOS tasks + main.c — complete firmware

**Files:**
- Create: `firmware/main/shared_state.h`
- Create: `firmware/main/tasks/task_*.c` + `.h` (5 files)
- Create: `firmware/main/main.c`

- [ ] **Step 1: Write `shared_state.h`**

```c
#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "serial_protocol.h"
#include <stdbool.h>

typedef struct {
    proto_state_t   state;
    proto_tof_t     tof;
    proto_cmd_vel_t cmd_vel;
    float           omega_meas[4];
    uint8_t         error_flags;
    bool            watchdog_ok;
    SemaphoreHandle_t mutex;
} shared_state_t;

extern shared_state_t g_state;
```

- [ ] **Step 2: Write `tasks/task_encoder_read.c`**

```c
#include "tasks/task_encoder_read.h"
#include "shared_state.h"
#include "encoder.h"
#include "freertos/task.h"
#include "esp_timer.h"

#define RAD_PER_COUNT (2.0f*3.14159265f/384.0f)

void task_encoder_read(void *arg) {
    TickType_t last = xTaskGetTickCount();
    while (1) {
        int32_t d[4]; encoder_get_deltas(d);
        xSemaphoreTake(g_state.mutex, portMAX_DELAY);
        for (int i=0;i<4;i++) {
            g_state.state.enc_delta[i] = d[i];
            g_state.omega_meas[i] = d[i]*RAD_PER_COUNT*1000.0f; /* 1kHz -> rad/s */
        }
        g_state.state.timestamp_ms = (uint32_t)(esp_timer_get_time()/1000);
        xSemaphoreGive(g_state.mutex);
        vTaskDelayUntil(&last, pdMS_TO_TICKS(1));
    }
}
```

- [ ] **Step 3: Write `tasks/task_pid_control.c`**

```c
#include "tasks/task_pid_control.h"
#include "shared_state.h"
#include "pid.h"
#include "motor.h"
#include "freertos/task.h"

static pid_t s_pid[4];

void task_pid_control(void *arg) {
    for (int i=0;i<4;i++) pid_init(&s_pid[i],2.0f,5.0f,0.01f,0.001f,-1.0f,1.0f);
    TickType_t last = xTaskGetTickCount();
    while (1) {
        xSemaphoreTake(g_state.mutex, portMAX_DELAY);
        float sp[4], meas[4]; bool ok=g_state.watchdog_ok;
        for(int i=0;i<4;i++){sp[i]=g_state.cmd_vel.omega[i];meas[i]=g_state.omega_meas[i];}
        xSemaphoreGive(g_state.mutex);
        if (!ok) { motor_stop_all(); vTaskDelayUntil(&last,pdMS_TO_TICKS(1)); continue; }
        for (int i=0;i<4;i++) motor_set_duty(i,pid_update(&s_pid[i],sp[i],meas[i]));
        vTaskDelayUntil(&last, pdMS_TO_TICKS(1));
    }
}
```

- [ ] **Step 4: Write `tasks/task_imu_read.c`**

```c
#include "tasks/task_imu_read.h"
#include "shared_state.h"
#include "ism330dhcx.h"
#include "freertos/task.h"

void task_imu_read(void *arg) {
    TickType_t last = xTaskGetTickCount();
    while (1) {
        ism330dhcx_data_t d; ism330dhcx_read(&d);
        xSemaphoreTake(g_state.mutex, portMAX_DELAY);
        for(int i=0;i<3;i++){g_state.state.accel[i]=d.accel[i];g_state.state.gyro[i]=d.gyro[i];}
        xSemaphoreGive(g_state.mutex);
        vTaskDelayUntil(&last, pdMS_TO_TICKS(10));
    }
}
```

- [ ] **Step 5: Write `tasks/task_tof_read.c`**

```c
#include "tasks/task_tof_read.h"
#include "shared_state.h"
#include "vl53l5cx_drv.h"
#include "freertos/task.h"

void task_tof_read(void *arg) {
    TickType_t last = xTaskGetTickCount();
    while (1) {
        uint16_t dist[64];
        if (vl53l5cx_drv_read(dist)) {
            xSemaphoreTake(g_state.mutex, portMAX_DELAY);
            for(int i=0;i<64;i++) g_state.tof.distances[i]=dist[i];
            xSemaphoreGive(g_state.mutex);
        }
        vTaskDelayUntil(&last, pdMS_TO_TICKS(100));
    }
}
```

- [ ] **Step 6: Write `tasks/task_serial_comms.c`**

```c
#include "tasks/task_serial_comms.h"
#include "shared_state.h"
#include "serial_protocol.h"
#include "freertos/task.h"
#include "driver/usb_serial_jtag.h"
#include <string.h>

#define RX_BUF 256
static uint8_t s_rx[RX_BUF]; static size_t s_rx_len=0;
static uint32_t s_last_hb_ms=0; static int s_tof_div=0;

static void tx(uint8_t type, const void *pl, uint8_t len){
    uint8_t frame[256];
    int flen=protocol_encode(type,pl,len,frame,sizeof(frame));
    if(flen>0) usb_serial_jtag_write_bytes(frame,flen,pdMS_TO_TICKS(5));
}

static void rx_process(void){
    int n=usb_serial_jtag_read_bytes(s_rx+s_rx_len,RX_BUF-s_rx_len,0);
    if(n<=0) return; s_rx_len+=n;
    size_t consumed; uint8_t type,payload[32],plen;
    while(protocol_decode(s_rx,s_rx_len,&consumed,&type,payload,&plen)){
        if(type==PROTO_TYPE_CMD_VEL&&plen==sizeof(proto_cmd_vel_t)){
            xSemaphoreTake(g_state.mutex,portMAX_DELAY);
            memcpy(&g_state.cmd_vel,payload,sizeof(proto_cmd_vel_t));
            xSemaphoreGive(g_state.mutex);
        } else if(type==PROTO_TYPE_HEARTBEAT){
            s_last_hb_ms=xTaskGetTickCount()*portTICK_PERIOD_MS;
        }
        memmove(s_rx,s_rx+consumed,s_rx_len-consumed);
        s_rx_len-=consumed;
    }
}

void task_serial_comms(void *arg){
    usb_serial_jtag_driver_config_t cfg={.rx_buffer_size=512,.tx_buffer_size=512};
    usb_serial_jtag_driver_install(&cfg);
    TickType_t last=xTaskGetTickCount();
    while(1){
        rx_process();
        uint32_t now=xTaskGetTickCount()*portTICK_PERIOD_MS;
        bool ok=(now-s_last_hb_ms)<2000;
        xSemaphoreTake(g_state.mutex,portMAX_DELAY);
        g_state.watchdog_ok=ok;
        g_state.error_flags=ok?g_state.error_flags&~0x01:g_state.error_flags|0x01;
        proto_state_t sc=g_state.state; proto_tof_t tc=g_state.tof;
        xSemaphoreGive(g_state.mutex);
        tx(PROTO_TYPE_STATE,&sc,sizeof(sc));
        if(++s_tof_div>=10){s_tof_div=0;tx(PROTO_TYPE_TOF_DATA,&tc,sizeof(tc));}
        vTaskDelayUntil(&last,pdMS_TO_TICKS(10));
    }
}
```

- [ ] **Step 7: Write `main.c`**

```c
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "encoder.h"
#include "motor.h"
#include "ism330dhcx.h"
#include "vl53l5cx_drv.h"
#include "shared_state.h"
#include "tasks/task_encoder_read.h"
#include "tasks/task_pid_control.h"
#include "tasks/task_imu_read.h"
#include "tasks/task_tof_read.h"
#include "tasks/task_serial_comms.h"
#include "esp_log.h"
#include <string.h>

shared_state_t g_state;
static const char *TAG="main";

void app_main(void){
    memset(&g_state,0,sizeof(g_state));
    g_state.mutex=xSemaphoreCreateMutex();
    ESP_LOGI(TAG,"Init hardware...");
    encoder_init(); motor_init();
    if(!ism330dhcx_init()) ESP_LOGE(TAG,"IMU init FAILED");
    if(!vl53l5cx_drv_init()) ESP_LOGE(TAG,"ToF init FAILED");
    ESP_LOGI(TAG,"Calibrating gyro — hold still 5s...");
    ism330dhcx_calibrate_gyro();
    ESP_LOGI(TAG,"Calibration done. Starting tasks.");
    xTaskCreatePinnedToCore(task_encoder_read,"enc",  4096,NULL, 9,NULL,0);
    xTaskCreatePinnedToCore(task_pid_control, "pid",  4096,NULL,10,NULL,0);
    xTaskCreatePinnedToCore(task_imu_read,    "imu",  4096,NULL, 7,NULL,1);
    xTaskCreatePinnedToCore(task_tof_read,    "tof",  8192,NULL, 6,NULL,1);
    xTaskCreatePinnedToCore(task_serial_comms,"ser",  8192,NULL, 8,NULL,1);
}
```

- [ ] **Step 8: Build, flash, verify**

```bash
cd ~/amr/firmware && idf.py build
idf.py -p /dev/ttyACM0 flash monitor
```

Expected in monitor:
```
I (xxx) main: Init hardware...
I (xxx) main: Calibrating gyro — hold still 5s...
I (xxx) main: Calibration done. Starting tasks.
```

- [ ] **Step 9: Verify STATE packets with Python script**

```bash
python3 scripts/verify_serial.py
```

Expected: 10 lines of `STATE ts=NNNNms enc=(0, 0, 0, 0)` — confirms packet framing, CRC, and 100Hz rate.

- [ ] **Step 10: Commit**

```bash
git add firmware/main/
git commit -m "feat: complete FreeRTOS firmware — all tasks, shared state, main.c"
```

---

## Phase 3 — ROS2 Hardware Interface
**Milestone:** `/joint_states`, `/imu/data_raw`, `/tof/points` live on RPi5. `mecanum_drive_controller` loaded. Robot moves when `/cmd_vel_safe` is published.

---

### Task 15: `amr_hardware` package + SerialDriver

**Files:**
- Create: `ros2_ws/src/amr_hardware/package.xml`
- Create: `ros2_ws/src/amr_hardware/CMakeLists.txt`
- Create: `ros2_ws/src/amr_hardware/include/amr_hardware/serial_driver.hpp`
- Create: `ros2_ws/src/amr_hardware/src/serial_driver.cpp`

- [ ] **Step 1: Create package**

```bash
cd ~/amr/ros2_ws/src
ros2 pkg create amr_hardware --build-type ament_cmake \
  --dependencies hardware_interface rclcpp sensor_msgs pluginlib
```

- [ ] **Step 2: Write `include/amr_hardware/serial_driver.hpp`**

```cpp
#pragma once
#include <string>
#include <vector>
#include <cstdint>

namespace amr_hardware {

struct StatePacket {
    uint32_t timestamp_ms;
    int32_t  enc_delta[4];  /* FL FR RL RR */
    float    accel[3];
    float    gyro[3];
};                          /* 44 bytes — matches firmware proto_state_t */

struct TofPacket   { uint16_t distances[64]; };   /* 128 bytes */
struct CmdVelPacket{ float    omega[4];       };   /* 16  bytes */

class SerialDriver {
public:
    bool open(const std::string & port, int baud_rate);
    void close();
    bool is_open() const { return fd_ >= 0; }
    bool send_cmd_vel(const CmdVelPacket & pkt);
    bool send_heartbeat();
    /* Drains port, parses. Returns true if new STATE received. */
    bool spin_once(StatePacket * st, TofPacket * tof, bool * new_tof);

private:
    int fd_{-1};
    std::vector<uint8_t> rx_buf_;
    bool write_packet(uint8_t type, const void * pl, uint8_t len);
    static uint16_t crc16(const uint8_t * d, size_t n);
    static constexpr uint8_t H0=0xAA,H1=0x55,TC=0x01,TS=0x02,TT=0x03,TH=0x04;
};

}  // namespace amr_hardware
```

- [ ] **Step 3: Write `src/serial_driver.cpp`**

```cpp
#include "amr_hardware/serial_driver.hpp"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>

namespace amr_hardware {

uint16_t SerialDriver::crc16(const uint8_t *d, size_t n){
    uint16_t crc=0xFFFF;
    for(size_t i=0;i<n;i++){crc^=(uint16_t)d[i]<<8;for(int j=0;j<8;j++)crc=(crc&0x8000)?(crc<<1)^0x1021:crc<<1;}
    return crc;
}

bool SerialDriver::open(const std::string &port, int /*baud_rate*/) {
    fd_=::open(port.c_str(),O_RDWR|O_NOCTTY|O_NONBLOCK);
    if(fd_<0) return false;
    struct termios tty{}; tcgetattr(fd_,&tty);
    cfsetispeed(&tty,B921600); cfsetospeed(&tty,B921600);
    cfmakeraw(&tty); tty.c_cc[VMIN]=0; tty.c_cc[VTIME]=0;
    tcsetattr(fd_,TCSANOW,&tty); return true;
}

void SerialDriver::close(){if(fd_>=0){::close(fd_);fd_=-1;}}

bool SerialDriver::write_packet(uint8_t type,const void *pl,uint8_t len){
    uint8_t frame[256]; size_t flen=6u+len;
    frame[0]=H0;frame[1]=H1;frame[2]=type;frame[3]=len;
    if(len&&pl) std::memcpy(frame+4,pl,len);
    uint16_t crc=crc16(frame+2,2+len);
    frame[4+len]=(crc>>8)&0xFF; frame[5+len]=crc&0xFF;
    return ::write(fd_,frame,flen)==(ssize_t)flen;
}

bool SerialDriver::send_cmd_vel(const CmdVelPacket &p){return write_packet(TC,&p,sizeof(p));}
bool SerialDriver::send_heartbeat(){return write_packet(TH,nullptr,0);}

bool SerialDriver::spin_once(StatePacket *st,TofPacket *tof,bool *new_tof){
    uint8_t tmp[512]; ssize_t n=::read(fd_,tmp,sizeof(tmp));
    if(n>0) rx_buf_.insert(rx_buf_.end(),tmp,tmp+n);
    bool got=false; *new_tof=false;
    while(rx_buf_.size()>=6){
        auto it=rx_buf_.begin();
        while(it+1<rx_buf_.end()&&!(*it==H0&&*(it+1)==H1)) ++it;
        if(it+1>=rx_buf_.end()){rx_buf_.clear();break;}
        rx_buf_.erase(rx_buf_.begin(),it);
        if(rx_buf_.size()<6) break;
        uint8_t plen=rx_buf_[3]; size_t flen=6u+plen;
        if(rx_buf_.size()<flen) break;
        uint16_t exp=crc16(rx_buf_.data()+2,2+plen);
        uint16_t got2=((uint16_t)rx_buf_[4+plen]<<8)|rx_buf_[5+plen];
        if(exp!=got2){rx_buf_.erase(rx_buf_.begin());continue;}
        uint8_t type=rx_buf_[2];
        if(type==TS&&plen==sizeof(StatePacket)){std::memcpy(st,rx_buf_.data()+4,sizeof(StatePacket));got=true;}
        else if(type==TT&&plen==sizeof(TofPacket)){std::memcpy(tof,rx_buf_.data()+4,sizeof(TofPacket));*new_tof=true;}
        rx_buf_.erase(rx_buf_.begin(),rx_buf_.begin()+flen);
    }
    return got;
}

}  // namespace amr_hardware
```

- [ ] **Step 4: Commit**

```bash
git add ros2_ws/src/amr_hardware/
git commit -m "feat: amr_hardware package with SerialDriver"
```

---

### Task 16: TofConverter + AMRHardwareInterface

**Files:**
- Create: `ros2_ws/src/amr_hardware/include/amr_hardware/tof_converter.hpp`
- Create: `ros2_ws/src/amr_hardware/src/tof_converter.cpp`
- Create: `ros2_ws/src/amr_hardware/include/amr_hardware/amr_hardware_interface.hpp`
- Create: `ros2_ws/src/amr_hardware/src/amr_hardware_interface.cpp`
- Create: `ros2_ws/src/amr_hardware/amr_hardware.xml`

- [ ] **Step 1: Write `include/amr_hardware/tof_converter.hpp`**

```cpp
#pragma once
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rclcpp/rclcpp.hpp>
#include <array>
#include <cmath>

namespace amr_hardware {
class TofConverter {
public:
    TofConverter();
    sensor_msgs::msg::PointCloud2 convert(const uint16_t distances[64],
        const std::string & frame_id, const rclcpp::Time & stamp);
private:
    static constexpr float HFOV=63.0f*M_PI/180.0f, VFOV=63.0f*M_PI/180.0f;
    std::array<float,64> ux_,uy_,uz_;
};
}
```

- [ ] **Step 2: Write `src/tof_converter.cpp`**

```cpp
#include "amr_hardware/tof_converter.hpp"
#include <sensor_msgs/msg/point_field.hpp>
#include <cstring>

namespace amr_hardware {

TofConverter::TofConverter(){
    for(int r=0;r<8;r++) for(int c=0;c<8;c++){
        int i=r*8+c;
        float th=(c-3.5f)/8.0f*HFOV, tv=(r-3.5f)/8.0f*VFOV;
        ux_[i]=std::cos(tv)*std::cos(th);
        uy_[i]=std::cos(tv)*std::sin(th);
        uz_[i]=std::sin(tv);
    }
}

sensor_msgs::msg::PointCloud2 TofConverter::convert(
    const uint16_t distances[64], const std::string &fid, const rclcpp::Time &stamp)
{
    sensor_msgs::msg::PointCloud2 msg;
    msg.header.frame_id=fid; msg.header.stamp=stamp;
    msg.height=1; msg.is_dense=false; msg.is_bigendian=false;
    auto mf=[](const char*n,uint32_t o){sensor_msgs::msg::PointField f;f.name=n;f.offset=o;f.datatype=sensor_msgs::msg::PointField::FLOAT32;f.count=1;return f;};
    msg.fields={mf("x",0),mf("y",4),mf("z",8)};
    msg.point_step=12;
    std::vector<float> pts; pts.reserve(192);
    for(int i=0;i<64;i++){
        if(!distances[i]||distances[i]>3500) continue;
        float d=distances[i]*0.001f;
        pts.push_back(d*ux_[i]); pts.push_back(d*uy_[i]); pts.push_back(d*uz_[i]);
    }
    msg.width=pts.size()/3; msg.row_step=msg.point_step*msg.width;
    msg.data.resize(msg.row_step);
    std::memcpy(msg.data.data(),pts.data(),msg.data.size());
    return msg;
}

}  // namespace amr_hardware
```

- [ ] **Step 3: Write `include/amr_hardware/amr_hardware_interface.hpp`**

```cpp
#pragma once
#include <hardware_interface/system_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "amr_hardware/serial_driver.hpp"
#include "amr_hardware/tof_converter.hpp"
#include <array>

namespace amr_hardware {

class AMRHardwareInterface : public hardware_interface::SystemInterface {
public:
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &) override;
    std::vector<hardware_interface::StateInterface>   export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
    hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;
    hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
    SerialDriver serial_; TofConverter tof_conv_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr tof_pub_;
    rclcpp::TimerBase::SharedPtr hb_timer_;
    /* FL=0 FR=1 RL=2 RR=3 */
    std::array<double,4> hw_cmd_{}, hw_vel_{}, hw_pos_{};
    static constexpr double RAD_PER_COUNT=2.0*M_PI/384.0;
    StatePacket st_{}; TofPacket tof_{};
    std::string port_; int baud_{921600};
};

}  // namespace amr_hardware
```

- [ ] **Step 4: Write `src/amr_hardware_interface.cpp`**

```cpp
#include "amr_hardware/amr_hardware_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace amr_hardware {

hardware_interface::CallbackReturn
AMRHardwareInterface::on_init(const hardware_interface::HardwareInfo &info){
    if(hardware_interface::SystemInterface::on_init(info)!=hardware_interface::CallbackReturn::SUCCESS)
        return hardware_interface::CallbackReturn::ERROR;
    port_=info_.hardware_parameters.at("serial_port");
    if(info_.hardware_parameters.count("baud_rate"))
        baud_=std::stoi(info_.hardware_parameters.at("baud_rate"));
    hw_cmd_.fill(0); hw_vel_.fill(0); hw_pos_.fill(0);
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
AMRHardwareInterface::export_state_interfaces(){
    const std::array<std::string,4> n={"wheel_FL_joint","wheel_FR_joint","wheel_RL_joint","wheel_RR_joint"};
    std::vector<hardware_interface::StateInterface> si;
    for(size_t i=0;i<4;i++){
        si.emplace_back(n[i],hardware_interface::HW_IF_VELOCITY,&hw_vel_[i]);
        si.emplace_back(n[i],hardware_interface::HW_IF_POSITION,&hw_pos_[i]);
    }
    return si;
}

std::vector<hardware_interface::CommandInterface>
AMRHardwareInterface::export_command_interfaces(){
    const std::array<std::string,4> n={"wheel_FL_joint","wheel_FR_joint","wheel_RL_joint","wheel_RR_joint"};
    std::vector<hardware_interface::CommandInterface> ci;
    for(size_t i=0;i<4;i++) ci.emplace_back(n[i],hardware_interface::HW_IF_VELOCITY,&hw_cmd_[i]);
    return ci;
}

hardware_interface::CallbackReturn
AMRHardwareInterface::on_activate(const rclcpp_lifecycle::State &){
    if(!serial_.open(port_,baud_)){
        RCLCPP_ERROR(rclcpp::get_logger("amr_hw"),"Cannot open %s",port_.c_str());
        return hardware_interface::CallbackReturn::ERROR;
    }
    auto node=get_node();
    imu_pub_=node->create_publisher<sensor_msgs::msg::Imu>("/imu/data_raw",10);
    tof_pub_=node->create_publisher<sensor_msgs::msg::PointCloud2>("/tof/points",10);
    hb_timer_=node->create_wall_timer(std::chrono::seconds(1),[this]{serial_.send_heartbeat();});
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
AMRHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &){
    hb_timer_.reset(); serial_.close();
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type
AMRHardwareInterface::read(const rclcpp::Time &t, const rclcpp::Duration &){
    bool new_tof=false;
    if(serial_.spin_once(&st_,&tof_,&new_tof)){
        for(int i=0;i<4;i++){
            hw_vel_[i]=st_.enc_delta[i]*RAD_PER_COUNT*100.0; /* 100Hz */
            hw_pos_[i]+=st_.enc_delta[i]*RAD_PER_COUNT;
        }
        sensor_msgs::msg::Imu imu;
        imu.header.stamp=t; imu.header.frame_id="imu_link";
        imu.linear_acceleration.x=st_.accel[0]; imu.linear_acceleration.y=st_.accel[1]; imu.linear_acceleration.z=st_.accel[2];
        imu.angular_velocity.x=st_.gyro[0];     imu.angular_velocity.y=st_.gyro[1];     imu.angular_velocity.z=st_.gyro[2];
        imu.orientation_covariance[0]=-1;
        imu_pub_->publish(imu);
    }
    if(new_tof) tof_pub_->publish(tof_conv_.convert(tof_.distances,"tof_link",t));
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type
AMRHardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration &){
    CmdVelPacket p; for(int i=0;i<4;i++) p.omega[i]=(float)hw_cmd_[i];
    serial_.send_cmd_vel(p);
    return hardware_interface::return_type::OK;
}

}  // namespace amr_hardware

PLUGINLIB_EXPORT_CLASS(amr_hardware::AMRHardwareInterface,hardware_interface::SystemInterface)
```

- [ ] **Step 5: Write `amr_hardware.xml`**

```xml
<library path="amr_hardware">
  <class name="amr_hardware/AMRHardwareInterface"
         type="amr_hardware::AMRHardwareInterface"
         base_class_type="hardware_interface::SystemInterface">
    <description>AMR serial hardware interface — ESP32-P4 over USB CDC</description>
  </class>
</library>
```

- [ ] **Step 6: Write final `CMakeLists.txt`**

```cmake
cmake_minimum_required(VERSION 3.8)
project(amr_hardware)
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(amr_hardware SHARED
    src/serial_driver.cpp
    src/tof_converter.cpp
    src/amr_hardware_interface.cpp)
target_include_directories(amr_hardware PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
ament_target_dependencies(amr_hardware hardware_interface pluginlib rclcpp sensor_msgs)
pluginlib_export_plugin_description_file(hardware_interface amr_hardware.xml)
install(TARGETS amr_hardware EXPORT export_amr_hardware
    LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(FILES amr_hardware.xml DESTINATION share/${PROJECT_NAME})
ament_export_include_directories(include)
ament_export_libraries(amr_hardware)
ament_export_targets(export_amr_hardware)
ament_package()
```

- [ ] **Step 7: Build on RPi5**

```bash
./scripts/deploy.sh
ssh ubuntu@amr.local "cd ~/amr_ws && source /opt/ros/jazzy/setup.bash && \
  colcon build --packages-select amr_hardware amr_description --symlink-install 2>&1 | tail -20"
```

Expected: `Finished <<< amr_hardware` with no errors.

- [ ] **Step 8: Commit**

```bash
git add ros2_ws/src/amr_hardware/
git commit -m "feat: TofConverter + AMRHardwareInterface + plugin registration"
```

---

### Task 17: `amr_bringup` — controllers + real hardware launch

**Files:**
- Create: `ros2_ws/src/amr_bringup/CMakeLists.txt`
- Create: `ros2_ws/src/amr_bringup/package.xml`
- Create: `ros2_ws/src/amr_bringup/config/controllers.yaml`
- Create: `ros2_ws/src/amr_bringup/launch/amr.launch.py`

- [ ] **Step 1: Create package**

```bash
cd ~/amr/ros2_ws/src
ros2 pkg create amr_bringup --build-type ament_cmake
```

- [ ] **Step 2: Write `config/controllers.yaml`**

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100
    mecanum_drive_controller:
      type: mecanum_drive_controller/MecanumDriveController

mecanum_drive_controller:
  ros__parameters:
    front_left_wheel_name:  wheel_FL_joint
    front_right_wheel_name: wheel_FR_joint
    rear_left_wheel_name:   wheel_RL_joint
    rear_right_wheel_name:  wheel_RR_joint
    wheel_separation_x: 0.275  # MEASURE and update
    wheel_separation_y: 0.220  # MEASURE and update
    wheel_radius: 0.030
    open_loop: false
    enable_odom_tf: false      # robot_localization owns this TF edge
    cmd_vel_timeout: 0.5
    publish_rate: 50.0
    velocity_rolling_window_size: 10
    base_frame_id: base_link
    odom_frame_id: odom
```

- [ ] **Step 3: Write `launch/amr.launch.py`**

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim = LaunchConfiguration('use_sim', default='false')
    robot_description = {'robot_description': Command([
        FindExecutable(name='xacro'), ' ',
        PathJoinSubstitution([FindPackageShare('amr_description'), 'urdf', 'amr.urdf.xacro']),
        ' use_sim:=', use_sim
    ])}
    ctrl_params = PathJoinSubstitution([FindPackageShare('amr_bringup'), 'config', 'controllers.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim', default_value='false'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[robot_description]),
        Node(package='controller_manager', executable='ros2_control_node',
             parameters=[robot_description, ctrl_params]),
        TimerAction(period=2.0, actions=[
            Node(package='controller_manager', executable='spawner',
                 arguments=['mecanum_drive_controller']),
        ]),
        Node(package='sllidar_ros2', executable='sllidar_node',
             parameters=[{'serial_port': '/dev/lidar', 'frame_id': 'base_laser',
                          'angle_compensate': True}]),
        Node(package='foxglove_bridge', executable='foxglove_bridge',
             parameters=[{'port': 8765}]),
    ])
```

- [ ] **Step 4: Deploy and launch on RPi5**

```bash
./scripts/deploy.sh
ssh ubuntu@amr.local "source /opt/ros/jazzy/setup.bash && \
  source ~/amr_ws/install/setup.bash && \
  ros2 launch amr_bringup amr.launch.py"
```

- [ ] **Step 5: Verify all Phase 3 topics**

```bash
ssh ubuntu@amr.local "source /opt/ros/jazzy/setup.bash && \
  source ~/amr_ws/install/setup.bash && \
  ros2 topic list"
```

Expected topics:
```
/imu/data_raw           — 100 Hz
/joint_states           — 100 Hz
/scan                   — 10 Hz
/tof/points             — 10 Hz
/odom/wheel             — 50 Hz
```

- [ ] **Step 6: Test wheel motion**

```bash
ssh ubuntu@amr.local "source /opt/ros/jazzy/setup.bash && \
  source ~/amr_ws/install/setup.bash && \
  ros2 topic pub --once /cmd_vel_safe geometry_msgs/msg/Twist \
  '{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'"
```

Expected: robot moves forward briefly, then stops after 500ms cmd_vel timeout.

> If wheels do not spin: check udev `/dev/amr_mcu` exists, check MDD10A power, check common GND.

- [ ] **Step 7: Commit**

```bash
git add ros2_ws/src/amr_bringup/
git commit -m "feat: amr_bringup with controllers config and real hardware launch"
```

---

---

## Phase 4: Sensor Fusion & Base Motion

**Goal:** Fuse wheel odometry + IMU into a reliable `/odom` estimate. Robot drives in a straight line and `/odom` tracks it accurately.

**Milestone:** `ros2 topic echo /odom` shows consistent position updates. `robot_localization` EKF running. `imu_filter_madgwick` publishing orientation. Robot drives forward 1 m and `/odom` reports ~1 m.

---

### Task 18: Create `amr_sensor_fusion` package

**Files:**
- Create: `ros2_ws/src/amr_sensor_fusion/package.xml`
- Create: `ros2_ws/src/amr_sensor_fusion/CMakeLists.txt`
- Create: `ros2_ws/src/amr_sensor_fusion/config/ekf.yaml`
- Create: `ros2_ws/src/amr_sensor_fusion/config/imu_filter.yaml`
- Create: `ros2_ws/src/amr_sensor_fusion/launch/sensor_fusion.launch.py`

- [ ] **Step 1: Create package.xml**

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>amr_sensor_fusion</name>
  <version>0.1.0</version>
  <description>EKF-based sensor fusion for wheel odometry and IMU</description>
  <maintainer email="khatrishubham030@gmail.com">Shubham Khatri</maintainer>
  <license>Apache-2.0</license>

  <exec_depend>robot_localization</exec_depend>
  <exec_depend>imu_filter_madgwick</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

- [ ] **Step 2: Create CMakeLists.txt**

```cmake
cmake_minimum_required(VERSION 3.8)
project(amr_sensor_fusion)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
```

- [ ] **Step 3: Create `config/imu_filter.yaml`**

imu_filter_madgwick fuses accelerometer + gyroscope to produce stable IMU orientation. We disable magnetometer (motor current corruption).

```yaml
imu_filter_madgwick:
  ros__parameters:
    use_mag: false
    publish_tf: false
    world_frame: "enu"
    gain: 0.1
    zeta: 0.0
    orientation_stddev: 0.005
    remove_gravity_vector: false
```

- [ ] **Step 4: Create `config/ekf.yaml`**

EKF fuses `/odom/wheel` (mecanum_drive_controller) and `/imu/data` (post-Madgwick). State vector: [x, y, yaw, vx, vy, vyaw, ax]. two_d_mode: true — robot is planar.

```yaml
ekf_filter_node:
  ros__parameters:
    frequency: 50.0
    sensor_timeout: 0.1
    two_d_mode: true
    transform_time_offset: 0.0
    transform_timeout: 0.0
    print_diagnostics: true
    debug: false

    map_frame: map
    odom_frame: odom
    base_link_frame: base_link
    world_frame: odom

    odom0: /odom/wheel
    odom0_config: [true, true, false,
                   false, false, true,
                   true,  true,  false,
                   false, false, true,
                   false, false, false]
    odom0_differential: false
    odom0_relative: false
    odom0_queue_size: 10

    imu0: /imu/data
    imu0_config: [false, false, false,
                  true,  true,  true,
                  false, false, false,
                  true,  true,  true,
                  true,  true,  false]
    imu0_differential: false
    imu0_relative: false
    imu0_remove_gravitational_acceleration: true
    imu0_queue_size: 10

    process_noise_covariance: [0.05, 0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
                               0,    0.05, 0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
                               0,    0,    0.06, 0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
                               0,    0,    0,    0.03, 0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
                               0,    0,    0,    0,    0.03, 0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
                               0,    0,    0,    0,    0,    0.06, 0,    0,    0,    0,    0,    0,    0,    0,    0,
                               0,    0,    0,    0,    0,    0,    0.025,0,    0,    0,    0,    0,    0,    0,    0,
                               0,    0,    0,    0,    0,    0,    0,    0.025,0,    0,    0,    0,    0,    0,    0,
                               0,    0,    0,    0,    0,    0,    0,    0,    0.04, 0,    0,    0,    0,    0,    0,
                               0,    0,    0,    0,    0,    0,    0,    0,    0,    0.01, 0,    0,    0,    0,    0,
                               0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0.01, 0,    0,    0,    0,
                               0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0.02, 0,    0,    0,
                               0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0.01, 0,    0,
                               0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0.01, 0,
                               0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0.015]

    initial_estimate_covariance: [1e-9, 0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
                                  0,    1e-9, 0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
                                  0,    0,    1e-9, 0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
                                  0,    0,    0,    1e-9, 0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
                                  0,    0,    0,    0,    1e-9, 0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
                                  0,    0,    0,    0,    0,    1e-9, 0,    0,    0,    0,    0,    0,    0,    0,    0,
                                  0,    0,    0,    0,    0,    0,    0.5,  0,    0,    0,    0,    0,    0,    0,    0,
                                  0,    0,    0,    0,    0,    0,    0,    0.5,  0,    0,    0,    0,    0,    0,    0,
                                  0,    0,    0,    0,    0,    0,    0,    0,    0.5,  0,    0,    0,    0,    0,    0,
                                  0,    0,    0,    0,    0,    0,    0,    0,    0,    0.1,  0,    0,    0,    0,    0,
                                  0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0.1,  0,    0,    0,    0,
                                  0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0.1,  0,    0,    0,
                                  0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0.1,  0,    0,
                                  0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0.1,  0,
                                  0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0.1]
```

- [ ] **Step 5: Create `launch/sensor_fusion.launch.py`**

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('amr_sensor_fusion')

    imu_filter = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter_madgwick',
        parameters=[os.path.join(pkg, 'config', 'imu_filter.yaml')],
        remappings=[
            ('imu/data_raw', '/imu/data_raw'),
            ('imu/data',     '/imu/data'),
        ],
    )

    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(pkg, 'config', 'ekf.yaml')],
        remappings=[
            ('odometry/filtered', '/odom'),
        ],
    )

    return LaunchDescription([imu_filter, ekf])
```

- [ ] **Step 6: Install dependencies on RPi5**

```bash
sudo apt install -y ros-jazzy-robot-localization ros-jazzy-imu-filter-madgwick
```

- [ ] **Step 7: Commit**

```bash
cd ~/ros2_ws
git add src/amr_sensor_fusion/
git commit -m "feat: add amr_sensor_fusion package (EKF + Madgwick)"
```

---

### Task 19: Wire mecanum odometry topic rename

The `mecanum_drive_controller` publishes on `/mecanum_drive_controller/odom`. EKF needs it on `/odom/wheel` to avoid confusion with filtered odom.

**Files:**
- Modify: `ros2_ws/src/amr_bringup/config/controllers.yaml`
- Modify: `ros2_ws/src/amr_bringup/launch/amr.launch.py`

- [ ] **Step 1: Add topic remap to controllers.yaml**

Open `ros2_ws/src/amr_bringup/config/controllers.yaml` and add inside the `mecanum_drive_controller` section:

```yaml
mecanum_drive_controller:
  ros__parameters:
    # ... existing params ...
    enable_odom_tf: false
```

The topic rename is handled via launch file remapping (next step).

- [ ] **Step 2: Add remap in amr.launch.py**

In the `mecanum_drive_controller` spawner section of `amr.launch.py`, add a static topic relay. After the spawner, add:

```python
from launch_ros.actions import Node as RosNode

odom_relay = RosNode(
    package='topic_tools',
    executable='relay',
    name='odom_relay',
    arguments=['/mecanum_drive_controller/odom', '/odom/wheel'],
)
```

Add `odom_relay` to the `LaunchDescription` list.

- [ ] **Step 3: Install topic_tools on RPi5**

```bash
sudo apt install -y ros-jazzy-topic-tools
```

- [ ] **Step 4: Commit**

```bash
git add src/amr_bringup/
git commit -m "feat: relay mecanum odom to /odom/wheel for EKF input"
```

---

### Task 20: Include sensor_fusion in main bringup launch

**Files:**
- Modify: `ros2_ws/src/amr_bringup/launch/amr.launch.py`

- [ ] **Step 1: Add sensor_fusion include**

Add to `amr.launch.py`:

```python
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

# inside generate_launch_description():
sensor_fusion_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        get_package_share_directory('amr_sensor_fusion'),
        '/launch/sensor_fusion.launch.py'
    ])
)
```

Add `sensor_fusion_launch` to the `LaunchDescription` list.

- [ ] **Step 2: Build and test on RPi5**

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select amr_sensor_fusion amr_bringup
source install/setup.bash
ros2 launch amr_bringup amr.launch.py
```

In a second terminal:
```bash
ros2 topic echo /odom --once
```

Expected output: a `nav_msgs/Odometry` message with `header.frame_id: "odom"`, `child_frame_id: "base_link"`, non-zero covariance.

- [ ] **Step 3: Drive test**

Publish a forward velocity and verify odom updates:
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
  --rate 10 --times 30
```

Watch `/odom`: x should increase by ~0.9 m over 1 second at 0.3 m/s × 3 s.

Expected: `pose.pose.position.x` near `0.9` (±0.15 m acceptable).

- [ ] **Step 4: Commit**

```bash
git add src/amr_bringup/
git commit -m "feat: integrate sensor_fusion into main bringup launch"
```

---

## Phase 5: SLAM — Map Building

**Goal:** Robot builds a 2D occupancy map of its environment in real time using slam_toolbox in online_async mode. Map visible in Foxglove.

**Milestone:** Launch full stack. Push robot around manually. Open Foxglove on the dev machine, connect to RPi5, see `/map` being built live. Save map with `ros2 run nav2_map_server map_saver_cli -f ~/map`.

---

### Task 21: Create `amr_slam` package

**Files:**
- Create: `ros2_ws/src/amr_slam/package.xml`
- Create: `ros2_ws/src/amr_slam/CMakeLists.txt`
- Create: `ros2_ws/src/amr_slam/config/slam_toolbox.yaml`
- Create: `ros2_ws/src/amr_slam/launch/slam.launch.py`

- [ ] **Step 1: Create package.xml**

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>amr_slam</name>
  <version>0.1.0</version>
  <description>SLAM configuration using slam_toolbox online_async mode</description>
  <maintainer email="khatrishubham030@gmail.com">Shubham Khatri</maintainer>
  <license>Apache-2.0</license>

  <exec_depend>slam_toolbox</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

- [ ] **Step 2: Create CMakeLists.txt**

```cmake
cmake_minimum_required(VERSION 3.8)
project(amr_slam)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
```

- [ ] **Step 3: Create `config/slam_toolbox.yaml`**

online_async mode: SLAM runs continuously during both exploration and normal navigation. No mode transition needed. Permanent mapping.

```yaml
slam_toolbox:
  ros__parameters:
    # Plugin/mode
    solver_plugin: solver_plugins::CeresSolver
    ceres_linear_solver: SPARSE_NORMAL_CHOLESKY
    ceres_preconditioner: SCHUR_JACOBI
    ceres_trust_strategy: LEVENBERG_MARQUARDT
    ceres_dogleg_type: TRADITIONAL_DOGLEG
    ceres_loss_function: None

    # Frames
    odom_frame: odom
    map_frame: map
    base_frame: base_link
    scan_topic: /scan
    use_map_saver: true
    mode: mapping

    # Map update
    map_update_interval: 5.0
    resolution: 0.05
    max_laser_range: 8.0
    minimum_travel_distance: 0.5
    minimum_travel_heading: 0.5

    # Loop closure
    loop_search_maximum_distance: 3.0
    do_loop_closing: true
    loop_match_minimum_chain_size: 10
    loop_match_maximum_variance_coarse: 3.0
    loop_match_minimum_response_coarse: 0.35
    loop_match_minimum_response_fine: 0.45

    # Scan matcher
    correlation_search_space_dimension: 0.5
    correlation_search_space_resolution: 0.01
    correlation_search_space_smear_deviation: 0.1
    loop_search_space_dimension: 8.0
    loop_search_space_resolution: 0.05
    loop_search_space_smear_deviation: 0.03
    distance_variance_penalty: 0.5
    angle_variance_penalty: 1.0
    fine_search_angle_offset: 0.00349
    coarse_search_angle_offset: 0.349
    coarse_angle_resolution: 0.0349
    minimum_angle_penalty: 0.9
    minimum_distance_penalty: 0.5
    use_response_expansion: true

    # TF / timing
    tf_buffer_duration: 30.0
    transform_publish_period: 0.02
    map_publish_interval: 5.0
    use_multithread_lookup: false

    # Serialization (save/restore sessions)
    map_file_name: /home/ubuntu/maps/amr_map
    map_start_at_dock: false
```

- [ ] **Step 4: Create `launch/slam.launch.py`**

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('amr_slam')

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[os.path.join(pkg, 'config', 'slam_toolbox.yaml')],
    )

    return LaunchDescription([slam_node])
```

- [ ] **Step 5: Install slam_toolbox on RPi5**

```bash
sudo apt install -y ros-jazzy-slam-toolbox ros-jazzy-nav2-map-server
```

- [ ] **Step 6: Create maps directory on RPi5**

```bash
mkdir -p ~/maps
```

- [ ] **Step 7: Commit**

```bash
cd ~/ros2_ws
git add src/amr_slam/
git commit -m "feat: add amr_slam package (slam_toolbox online_async)"
```

---

### Task 22: Integrate SLAM into main bringup

**Files:**
- Modify: `ros2_ws/src/amr_bringup/launch/amr.launch.py`

- [ ] **Step 1: Add SLAM include to amr.launch.py**

```python
slam_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        get_package_share_directory('amr_slam'),
        '/launch/slam.launch.py'
    ])
)
```

Add `slam_launch` to the `LaunchDescription` list.

- [ ] **Step 2: Build everything**

```bash
cd ~/ros2_ws
colcon build --symlink-install \
  --packages-select amr_slam amr_bringup
source install/setup.bash
```

Expected: no errors. Both packages installed.

- [ ] **Step 3: Launch full stack on RPi5**

```bash
ros2 launch amr_bringup amr.launch.py
```

Watch for slam_toolbox to log:
```
[slam_toolbox]: Message Filter dropping message: frame 'base_link' ...
```
This warning is normal for the first ~5 s while TF tree settles.

After 10 s with the LiDAR spinning, it should log:
```
[slam_toolbox]: Registering sensor: [LidarSensor]
```

- [ ] **Step 4: Verify map topic**

```bash
ros2 topic echo /map --once
```

Expected: `nav_msgs/OccupancyGrid` with `info.resolution: 0.05`, `info.width` and `info.height` non-zero.

- [ ] **Step 5: View in Foxglove**

On the dev machine, open Foxglove Studio. Connect to `ws://<rpi5-ip>:8765`. Add a Map panel subscribed to `/map`. Add a 3D panel showing `/scan`. Push the robot manually (via keyboard teleop):

```bash
# On RPi5 or dev machine with ROS_DOMAIN_ID=42
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args --remap cmd_vel:=/cmd_vel
```

Drive around the room. Verify map grows correctly in Foxglove.

- [ ] **Step 6: Save map**

```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/amr_map
```

Expected: two files created — `~/maps/amr_map.pgm` and `~/maps/amr_map.yaml`.

- [ ] **Step 7: Commit**

```bash
git add src/amr_bringup/
git commit -m "feat: integrate slam_toolbox into main bringup, map building verified"
```


---

## Phase 6: Navigation — Point-to-Point

**Goal:** Robot navigates to a clicked goal in Foxglove using SmacPlannerLattice (holonomic-aware global planner) + MPPI controller (holonomic local planner) with nav2_collision_monitor as safety layer. No exploration yet — just navigate to a pose goal.

**Milestone:** Publish a goal to `/goal_pose`. Robot plans a path and drives to the goal, stopping within 0.15 m. Collision monitor halts motion when an obstacle enters the 0.3 m safety bubble.

---

### Task 23: Generate SmacPlannerLattice motion primitives

SmacPlannerLattice requires precomputed lattice `.mprim` files specific to the robot's kinematics. Generate them once and commit.

**Files:**
- Create: `ros2_ws/src/amr_nav/config/lattice/mecanum_5cm.mprim` (generated)
- Create: `ros2_ws/src/amr_nav/scripts/generate_lattice.py`

- [ ] **Step 1: Install nav2-smac-planner on dev machine**

```bash
sudo apt install -y ros-humble-nav2-smac-planner
```

- [ ] **Step 2: Create generate_lattice.py**

```python
#!/usr/bin/env python3
"""Generate lattice primitives for a mecanum-drive (holonomic) robot."""
import subprocess, os, pathlib

OUTPUT = pathlib.Path(__file__).parent.parent / "config" / "lattice"
OUTPUT.mkdir(parents=True, exist_ok=True)

# nav2_smac_planner ships a standalone lattice generator binary
cmd = [
    "ros2", "run", "nav2_smac_planner", "lattice_primitives",
    "--output", str(OUTPUT / "mecanum_5cm.mprim"),
    "--motion_model", "OMNI",
    "--turning_radius", "0.0",      # holonomic: 0.0 = spin in place
    "--min_turning_radius", "0.0",
    "--discretization_angle", "0.3927",   # pi/8 → 8 headings
    "--number_of_headings", "8",
    "--grid_resolution", "0.05",
    "--stopping_threshold", "0.01",
    "--max_iter", "10000",
]
print("Running:", " ".join(cmd))
subprocess.run(cmd, check=True)
print("Lattice written to:", OUTPUT / "mecanum_5cm.mprim")
```

- [ ] **Step 3: Run generator on dev machine**

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws/src/amr_nav/scripts
python3 generate_lattice.py
```

Expected: file `ros2_ws/src/amr_nav/config/lattice/mecanum_5cm.mprim` created, ~50–200 KB.

> **Note:** If the `lattice_primitives` binary is not found, check with `ros2 pkg executables nav2_smac_planner`. On some builds the tool is `nav2_lattice_primitive_generator`. Adjust `cmd` accordingly.

- [ ] **Step 4: Commit lattice file**

```bash
git add src/amr_nav/config/lattice/mecanum_5cm.mprim src/amr_nav/scripts/generate_lattice.py
git commit -m "feat: add OMNI mecanum lattice primitives for SmacPlannerLattice"
```

---

### Task 24: Create `amr_nav` package — costmap and planner config

**Files:**
- Create: `ros2_ws/src/amr_nav/package.xml`
- Create: `ros2_ws/src/amr_nav/CMakeLists.txt`
- Create: `ros2_ws/src/amr_nav/config/nav2_params.yaml`
- Create: `ros2_ws/src/amr_nav/config/collision_monitor.yaml`
- Create: `ros2_ws/src/amr_nav/launch/nav2.launch.py`

- [ ] **Step 1: Create package.xml**

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>amr_nav</name>
  <version>0.1.0</version>
  <description>Nav2 navigation stack config for mecanum AMR</description>
  <maintainer email="khatrishubham030@gmail.com">Shubham Khatri</maintainer>
  <license>Apache-2.0</license>

  <exec_depend>nav2_bringup</exec_depend>
  <exec_depend>nav2_smac_planner</exec_depend>
  <exec_depend>nav2_mppi_controller</exec_depend>
  <exec_depend>nav2_collision_monitor</exec_depend>
  <exec_depend>nav2_costmap_2d</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

- [ ] **Step 2: Create CMakeLists.txt**

```cmake
cmake_minimum_required(VERSION 3.8)
project(amr_nav)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
```

- [ ] **Step 3: Create `config/nav2_params.yaml`**

Full Nav2 config — SmacPlannerLattice global planner, MPPI Omni local controller, costmaps with inflation, VoxelLayer for ToF, lifecycle manager.

```yaml
amcl:
  ros__parameters:
    use_sim_time: false

bt_navigator:
  ros__parameters:
    use_sim_time: false
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    bt_loop_duration: 10
    default_server_timeout: 20
    default_nav_to_pose_bt_xml: ""
    default_nav_through_poses_bt_xml: ""
    plugin_lib_names:
      - nav2_compute_path_to_pose_action_bt_node
      - nav2_follow_path_action_bt_node
      - nav2_goal_reached_condition_bt_node
      - nav2_is_path_valid_condition_bt_node
      - nav2_recovery_node_bt_node
      - nav2_spin_action_bt_node
      - nav2_wait_action_bt_node
      - nav2_back_up_action_bt_node
      - nav2_clear_costmap_service_bt_node

controller_server:
  ros__parameters:
    use_sim_time: false
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.001
    min_theta_velocity_threshold: 0.001
    failure_tolerance: 0.3
    progress_checker_plugins: ["progress_checker"]
    goal_checker_plugins: ["goal_checker"]
    controller_plugins: ["FollowPath"]

    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0

    goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.15
      yaw_goal_tolerance: 0.15
      stateful: true

    FollowPath:
      plugin: "nav2_mppi_controller::MPPIController"
      time_steps: 56
      model_dt: 0.05
      batch_size: 2000
      vx_std: 0.2
      vy_std: 0.2
      wz_std: 0.4
      vx_max: 0.5
      vx_min: -0.5
      vy_max: 0.5
      wz_max: 1.9
      iteration_count: 1
      prune_distance: 1.7
      transform_tolerance: 0.1
      temperature: 0.3
      gamma: 0.015
      motion_model: "Omni"
      visualize: false
      TrajectoryVisualizer:
        trajectory_step: 5
        time_step: 3
      AckermannConstraints:
        min_turning_r: 0.0
      critics: ["ConstraintCritic", "GoalCritic", "GoalAngleCritic", "PathAlignCritic",
                "PathFollowCritic", "PathAngleCritic", "PreferForwardCritic"]
      ConstraintCritic:
        enabled: true
        cost_power: 1
        cost_weight: 4.0
      GoalCritic:
        enabled: true
        cost_power: 1
        cost_weight: 5.0
        threshold_to_consider: 1.4
      GoalAngleCritic:
        enabled: true
        cost_power: 1
        cost_weight: 3.0
        threshold_to_consider: 0.5
      PathAlignCritic:
        enabled: true
        cost_power: 1
        cost_weight: 14.0
        max_path_occupancy_ratio: 0.05
        trajectory_point_step: 3
        threshold_to_consider: 0.5
        offset_from_furthest: 20
        use_path_orientations: false
      PathFollowCritic:
        enabled: true
        cost_power: 1
        cost_weight: 5.0
        offset_from_furthest: 5
        threshold_to_consider: 1.4
      PathAngleCritic:
        enabled: true
        cost_power: 1
        cost_weight: 2.0
        offset_from_furthest: 4
        threshold_to_consider: 0.5
        max_angle_to_furthest: 1.0
        mode: 0
      PreferForwardCritic:
        enabled: true
        cost_power: 1
        cost_weight: 5.0
        threshold_to_consider: 0.5

planner_server:
  ros__parameters:
    use_sim_time: false
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_smac_planner::SmacPlannerLattice"
      allow_unknown: true
      tolerance: 0.25
      max_iterations: 1000000
      max_on_approach_iterations: 1000
      max_planning_time: 5.0
      analytic_expansion_ratio: 3.5
      analytic_expansion_max_length: 3.0
      reverse_penalty: 2.0
      change_penalty: 0.05
      non_straight_penalty: 1.05
      cost_penalty: 2.0
      retrospective_penalty: 0.015
      rotation_penalty: 5.0
      lattice_filepath: ""  # set in launch file via full path

local_costmap:
  local_costmap:
    ros__parameters:
      use_sim_time: false
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      rolling_window: true
      width: 3
      height: 3
      resolution: 0.05
      robot_radius: 0.50
      plugins: ["voxel_layer", "inflation_layer"]
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: true
        publish_voxel_map: false
        origin_z: 0.0
        z_resolution: 0.05
        z_voxels: 16
        max_obstacle_height: 2.0
        observation_sources: scan tof
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: true
          marking: true
          data_type: "LaserScan"
          raytrace_max_range: 8.0
          raytrace_min_range: 0.0
          obstacle_max_range: 7.5
          obstacle_min_range: 0.0
        tof:
          topic: /tof/points
          max_obstacle_height: 1.5
          clearing: false
          marking: true
          data_type: "PointCloud2"
          raytrace_max_range: 3.5
          raytrace_min_range: 0.0
          obstacle_max_range: 3.0
          obstacle_min_range: 0.1
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55

global_costmap:
  global_costmap:
    ros__parameters:
      use_sim_time: false
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      robot_radius: 0.50
      resolution: 0.05
      track_unknown_space: true
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: true
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: true
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: true
          marking: true
          data_type: "LaserScan"
          raytrace_max_range: 8.0
          obstacle_max_range: 7.5
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55

map_server:
  ros__parameters:
    use_sim_time: false
    yaml_filename: ""

map_saver:
  ros__parameters:
    use_sim_time: false
    save_map_timeout: 5.0
    free_thresh_default: 0.25
    occupied_thresh_default: 0.65

recoveries_server:
  ros__parameters:
    use_sim_time: false
    costmap_topic: local_costmap/costmap_raw
    footprint_topic: local_costmap/published_footprint
    cycle_frequency: 10.0
    recovery_plugins: ["spin", "backup", "wait"]
    spin:
      plugin: "nav2_recoveries::Spin"
    backup:
      plugin: "nav2_recoveries::BackUp"
    wait:
      plugin: "nav2_recoveries::Wait"
    global_frame: odom
    robot_base_frame: base_link
    transform_timeout: 0.1
    use_sim_time: false
    simulate_ahead_time: 2.0
    max_rotational_vel: 1.0
    min_rotational_vel: 0.4
    rotational_acc_lim: 3.2

lifecycle_manager:
  ros__parameters:
    use_sim_time: false
    autostart: true
    node_names:
      - map_server
      - controller_server
      - planner_server
      - recoveries_server
      - bt_navigator
```

- [ ] **Step 4: Create `config/collision_monitor.yaml`**

nav2_collision_monitor sits between controller `/cmd_vel` output and the hardware `/cmd_vel_safe`. It reads `/scan` + `/tof/points` directly — faster than waiting for costmap updates.

```yaml
collision_monitor:
  ros__parameters:
    use_sim_time: false
    base_frame_id: "base_link"
    odom_frame_id: "odom"
    cmd_vel_in_topic: "/cmd_vel"
    cmd_vel_out_topic: "/cmd_vel_safe"
    state_topic: "/collision_monitor_state"
    transform_tolerance: 0.2
    source_timeout: 1.0
    base_shift_correction: true
    stop_pub_timeout: 2.0
    polygons: ["FootprintApproach"]
    FootprintApproach:
      type: "polygon"
      action_type: "slowdown"
      footprint_topic: "/local_costmap/published_footprint"
      time_before_collision: 1.2
      simulation_time_step: 0.02
      min_points: 6
      visualize: false
      enabled: true
    observation_sources: ["scan"]
    scan:
      type: "scan"
      topic: "/scan"
      min_height: 0.05
      max_height: 0.5
      enabled: true
```

- [ ] **Step 5: Create `launch/nav2.launch.py`**

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('amr_nav')
    nav2_bringup_pkg = get_package_share_directory('nav2_bringup')

    lattice_path = os.path.join(pkg, 'config', 'lattice', 'mecanum_5cm.mprim')
    nav2_params = os.path.join(pkg, 'config', 'nav2_params.yaml')

    # Patch lattice_filepath at runtime (can't use substitution in YAML string)
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_pkg, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': nav2_params,
        }.items(),
    )

    collision_monitor = Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        output='screen',
        parameters=[os.path.join(pkg, 'config', 'collision_monitor.yaml')],
    )

    return LaunchDescription([nav2, collision_monitor])
```

- [ ] **Step 6: Patch lattice_filepath at launch time**

The lattice path can't be embedded in YAML because it's install-location-dependent. After the nav2 bringup launches, override the param:

Add to `launch/nav2.launch.py` after `collision_monitor`:

```python
from launch.actions import ExecuteProcess

patch_lattice = ExecuteProcess(
    cmd=['ros2', 'param', 'set', '/planner_server',
         'GridBased.lattice_filepath', lattice_path],
    output='screen',
)
```

Add `patch_lattice` to `LaunchDescription`. (Nav2 lifecycle manager restarts the planner server after this.)

- [ ] **Step 7: Install Nav2 on RPi5**

```bash
sudo apt install -y \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-nav2-mppi-controller \
  ros-jazzy-nav2-smac-planner \
  ros-jazzy-nav2-collision-monitor
```

- [ ] **Step 8: Commit**

```bash
git add src/amr_nav/
git commit -m "feat: add amr_nav package (SmacPlannerLattice + MPPI Omni + collision_monitor)"
```

---

### Task 25: Wire navigation into main bringup and test

**Files:**
- Modify: `ros2_ws/src/amr_bringup/launch/amr.launch.py`

- [ ] **Step 1: Add nav2 include to amr.launch.py**

```python
nav2_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        get_package_share_directory('amr_nav'),
        '/launch/nav2.launch.py'
    ])
)
```

Add `nav2_launch` to `LaunchDescription`.

Also change the hardware interface `cmd_vel` remap so the robot obeys the collision-monitored output:

```python
# In ros2_control_node or mecanum_drive_controller spawner, confirm it listens on /cmd_vel_safe
# The collision monitor already outputs to /cmd_vel_safe
# mecanum_drive_controller subscribes to /cmd_vel by default — remap it:
Node(
    package='ros2_control',
    executable='ros2_control_node',
    ...
    remappings=[('/mecanum_drive_controller/cmd_vel_unstamped', '/cmd_vel_safe')],
)
```

- [ ] **Step 2: Build and deploy**

```bash
cd ~/ros2_ws
colcon build --symlink-install \
  --packages-select amr_nav amr_bringup
source install/setup.bash
```

Copy lattice file to RPi5 (run from dev machine):
```bash
scp ros2_ws/src/amr_nav/config/lattice/mecanum_5cm.mprim \
  ubuntu@<rpi5-ip>:~/ros2_ws/src/amr_nav/config/lattice/
```

- [ ] **Step 3: Launch and send navigation goal**

```bash
# On RPi5
ros2 launch amr_bringup amr.launch.py
```

In Foxglove: add a **Publish** panel, topic `/goal_pose`, message type `geometry_msgs/PoseStamped`. Set `header.frame_id = "map"`, pick a reachable pose. Click Publish.

Expected in terminal:
```
[bt_navigator]: Begin navigating from current location...
[planner_server]: Planning... Found path with X waypoints
[controller_server]: MPPI: tracking path
```

Robot should drive to the goal and stop within 0.15 m.

- [ ] **Step 4: Test collision stop**

While robot is driving to a goal, place an obstacle (box, foot) within 0.5 m of the robot's front.

Expected: robot slows/stops. Remove obstacle. Robot resumes.

- [ ] **Step 5: Commit**

```bash
git add src/amr_bringup/
git commit -m "feat: integrate nav2 into main bringup, collision monitor wired to cmd_vel_safe"
```

---

## Phase 7: Autonomous Exploration + Home Management

**Goal:** Robot autonomously explores unknown space using frontier-based exploration (m-explore-ros2), builds a full map, then returns to its start pose on command. A state machine node (`amr_home_manager`) orchestrates exploration start/stop, home recording, and return-home.

**Milestone:** `ros2 topic pub /amr/command std_msgs/String '"explore"'` → robot explores until map is complete. `ros2 topic pub /amr/command std_msgs/String '"go_home"'` → robot navigates back to start. Map saved automatically on exploration end.

---

### Task 26: Clone and build m-explore-ros2 from source

m-explore-ros2 is not in the Jazzy apt repository and must be built from source.

**Files:**
- Clone: `ros2_ws/src/m-explore-ros2/` (external repo, not committed)
- Create: `.gitignore` entry

- [ ] **Step 1: Clone on RPi5**

```bash
cd ~/ros2_ws/src
git clone https://github.com/robo-friends/m-explore-ros2.git
```

- [ ] **Step 2: Install dependencies**

```bash
cd ~/ros2_ws
rosdep install --from-paths src/m-explore-ros2 --ignore-src -r -y
```

- [ ] **Step 3: Build**

```bash
colcon build --symlink-install --packages-select explore_lite
source install/setup.bash
```

Expected: `explore_lite` package installed. Verify:
```bash
ros2 pkg executables explore_lite
```
Expected output: `explore_lite explore`

- [ ] **Step 4: Add to .gitignore**

```bash
echo "ros2_ws/src/m-explore-ros2/" >> .gitignore
git add .gitignore
git commit -m "chore: ignore m-explore-ros2 source clone (built from source)"
```

---

### Task 27: Create `amr_explore` package — exploration config

**Files:**
- Create: `ros2_ws/src/amr_explore/package.xml`
- Create: `ros2_ws/src/amr_explore/CMakeLists.txt`
- Create: `ros2_ws/src/amr_explore/config/explore.yaml`
- Create: `ros2_ws/src/amr_explore/launch/explore.launch.py`

- [ ] **Step 1: Create package.xml**

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>amr_explore</name>
  <version>0.1.0</version>
  <description>Frontier-based autonomous exploration config</description>
  <maintainer email="khatrishubham030@gmail.com">Shubham Khatri</maintainer>
  <license>Apache-2.0</license>

  <exec_depend>explore_lite</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

- [ ] **Step 2: Create CMakeLists.txt**

```cmake
cmake_minimum_required(VERSION 3.8)
project(amr_explore)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
```

- [ ] **Step 3: Create `config/explore.yaml`**

```yaml
explore:
  ros__parameters:
    robot_base_frame: base_link
    costmap_topic: /global_costmap/costmap
    costmap_updates_topic: /global_costmap/costmap_updates
    visualize: true
    planner_frequency: 0.33
    progress_timeout: 30.0
    potential_scale: 3.0
    gain_scale: 1.0
    transform_tolerance: 0.3
    min_frontier_size: 0.75
```

- [ ] **Step 4: Create `launch/explore.launch.py`**

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('amr_explore')

    explore_node = Node(
        package='explore_lite',
        executable='explore',
        name='explore_node',
        output='screen',
        parameters=[os.path.join(pkg, 'config', 'explore.yaml')],
    )

    return LaunchDescription([explore_node])
```

- [ ] **Step 5: Commit**

```bash
cd ~/ros2_ws
git add src/amr_explore/
git commit -m "feat: add amr_explore package (frontier exploration config)"
```

---

### Task 28: Create `amr_home_manager` — state machine node

This Python ROS2 node is the behavioral brain of the robot. It orchestrates exploration, home recording, and return-home via a clean state machine.

**Files:**
- Create: `ros2_ws/src/amr_home_manager/package.xml`
- Create: `ros2_ws/src/amr_home_manager/CMakeLists.txt`
- Create: `ros2_ws/src/amr_home_manager/amr_home_manager/home_manager_node.py`
- Create: `ros2_ws/src/amr_home_manager/amr_home_manager/__init__.py`
- Create: `ros2_ws/src/amr_home_manager/launch/home_manager.launch.py`
- Create: `ros2_ws/src/amr_home_manager/test/test_home_manager.py`

- [ ] **Step 1: Create package.xml**

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>amr_home_manager</name>
  <version>0.1.0</version>
  <description>State machine for autonomous exploration and home return</description>
  <maintainer email="khatrishubham030@gmail.com">Shubham Khatri</maintainer>
  <license>Apache-2.0</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>nav2_msgs</depend>
  <depend>action_msgs</depend>

  <test_depend>pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

- [ ] **Step 2: Create CMakeLists.txt (setup.py for ament_python)**

Actually for `ament_python`, we need `setup.py` and `setup.cfg`, not CMakeLists.txt.

Create `ros2_ws/src/amr_home_manager/setup.py`:
```python
from setuptools import setup

package_name = 'amr_home_manager'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/home_manager.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'home_manager = amr_home_manager.home_manager_node:main',
        ],
    },
)
```

Create `ros2_ws/src/amr_home_manager/setup.cfg`:
```ini
[develop]
script_dir=$base/lib/amr_home_manager
[install]
install_scripts=$base/lib/amr_home_manager
```

Create `ros2_ws/src/amr_home_manager/resource/amr_home_manager` (empty file):
```bash
mkdir -p ros2_ws/src/amr_home_manager/resource
touch ros2_ws/src/amr_home_manager/resource/amr_home_manager
```

- [ ] **Step 3: Create `amr_home_manager/__init__.py`**

```python
```
(empty file)

- [ ] **Step 4: Write failing test first**

Create `ros2_ws/src/amr_home_manager/test/test_home_manager.py`:

```python
import pytest
from unittest.mock import MagicMock, patch
import sys

# Stub out ROS2 imports so tests run without a ROS2 installation
sys.modules['rclpy'] = MagicMock()
sys.modules['rclpy.node'] = MagicMock()
sys.modules['rclpy.action'] = MagicMock()
sys.modules['rclpy.action.client'] = MagicMock()
sys.modules['nav2_msgs'] = MagicMock()
sys.modules['nav2_msgs.action'] = MagicMock()
sys.modules['geometry_msgs'] = MagicMock()
sys.modules['geometry_msgs.msg'] = MagicMock()
sys.modules['std_msgs'] = MagicMock()
sys.modules['std_msgs.msg'] = MagicMock()
sys.modules['nav_msgs'] = MagicMock()
sys.modules['nav_msgs.msg'] = MagicMock()
sys.modules['action_msgs'] = MagicMock()
sys.modules['action_msgs.msg'] = MagicMock()

from amr_home_manager.home_manager_node import HomeManagerNode, State


def make_node():
    with patch('amr_home_manager.home_manager_node.Node.__init__', return_value=None):
        node = HomeManagerNode.__new__(HomeManagerNode)
        node._state = State.IDLE
        node._home_pose = None
        node._logger = MagicMock()
        node._nav_client = MagicMock()
        node._explore_pub = MagicMock()
        node._map_saver_pub = MagicMock()
        return node


def test_initial_state_is_idle():
    node = make_node()
    assert node._state == State.IDLE


def test_command_explore_transitions_to_exploring():
    node = make_node()
    node._home_pose = MagicMock()
    msg = MagicMock()
    msg.data = "explore"
    node._on_command(msg)
    assert node._state == State.EXPLORING


def test_command_go_home_from_idle_does_nothing_without_home():
    node = make_node()
    node._home_pose = None
    msg = MagicMock()
    msg.data = "go_home"
    node._on_command(msg)
    assert node._state == State.IDLE


def test_command_go_home_transitions_to_returning():
    node = make_node()
    node._home_pose = MagicMock()
    msg = MagicMock()
    msg.data = "go_home"
    node._on_command(msg)
    assert node._state == State.RETURNING_HOME


def test_record_home_saves_pose():
    node = make_node()
    odom_msg = MagicMock()
    odom_msg.pose.pose.position.x = 1.5
    odom_msg.pose.pose.position.y = 2.3
    node._on_odom(odom_msg)
    assert node._home_pose is not None


def test_exploration_done_saves_map_and_transitions():
    node = make_node()
    node._state = State.EXPLORING
    node._on_exploration_done()
    node._map_saver_pub.publish.assert_called_once()
    assert node._state == State.IDLE
```

- [ ] **Step 5: Run test — expect failure**

```bash
cd ros2_ws/src/amr_home_manager
python3 -m pytest test/test_home_manager.py -v
```

Expected: `ImportError` or `ModuleNotFoundError` for `home_manager_node` (it doesn't exist yet).

- [ ] **Step 6: Implement `home_manager_node.py`**

```python
import enum
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose


class State(enum.Enum):
    IDLE = "idle"
    EXPLORING = "exploring"
    RETURNING_HOME = "returning_home"


class HomeManagerNode(Node):
    def __init__(self):
        super().__init__('amr_home_manager')

        self._state = State.IDLE
        self._home_pose: PoseStamped | None = None

        # Command subscriber: accepts "explore", "go_home", "stop"
        self.create_subscription(String, '/amr/command', self._on_command, 10)

        # Odom subscriber — record home on first message
        self._odom_sub = self.create_subscription(
            Odometry, '/odom', self._on_odom, 10)

        # Publish True/False to start/stop explore_lite
        self._explore_pub = self.create_publisher(Bool, '/explore/resume', 10)

        # Trigger map save
        self._map_saver_pub = self.create_publisher(
            String, '/amr/save_map', 10)

        # Nav2 action client
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.get_logger().info('HomeManagerNode ready. State: IDLE')

    def _on_odom(self, msg: Odometry) -> None:
        if self._home_pose is None:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose = msg.pose.pose
            self._home_pose = pose
            self.get_logger().info(
                f'Home pose recorded: ({msg.pose.pose.position.x:.2f}, '
                f'{msg.pose.pose.position.y:.2f})')
        # Only need first message to record home
        self.destroy_subscription(self._odom_sub)

    def _on_command(self, msg: String) -> None:
        cmd = msg.data.strip().lower()
        self.get_logger().info(f'Command received: {cmd} (state={self._state.value})')

        if cmd == 'explore':
            if self._home_pose is None:
                self.get_logger().warn('Home pose not yet recorded — waiting for /odom')
                return
            self._state = State.EXPLORING
            resume = Bool()
            resume.data = True
            self._explore_pub.publish(resume)
            self.get_logger().info('Exploration started')

        elif cmd == 'stop':
            if self._state == State.EXPLORING:
                self._on_exploration_done()

        elif cmd == 'go_home':
            if self._home_pose is None:
                self.get_logger().warn('No home pose recorded — cannot return home')
                return
            self._state = State.RETURNING_HOME
            self._navigate_to_home()

    def _on_exploration_done(self) -> None:
        stop = Bool()
        stop.data = False
        self._explore_pub.publish(stop)
        save = String()
        save.data = 'save'
        self._map_saver_pub.publish(save)
        self._state = State.IDLE
        self.get_logger().info('Exploration complete. Map save triggered.')

    def _navigate_to_home(self) -> None:
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('NavigateToPose action server not available')
            self._state = State.IDLE
            return
        goal = NavigateToPose.Goal()
        goal.pose = self._home_pose
        self.get_logger().info('Sending navigate_to_pose goal (home)')
        send_future = self._nav_client.send_goal_async(goal)
        send_future.add_done_callback(self._on_nav_goal_accepted)

    def _on_nav_goal_accepted(self, future) -> None:
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Navigation goal rejected')
            self._state = State.IDLE
            return
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._on_nav_done)

    def _on_nav_done(self, future) -> None:
        self._state = State.IDLE
        self.get_logger().info('Returned home. State: IDLE')


def main(args=None):
    rclpy.init(args=args)
    node = HomeManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

- [ ] **Step 7: Run tests — expect pass**

```bash
cd ros2_ws/src/amr_home_manager
python3 -m pytest test/test_home_manager.py -v
```

Expected output:
```
PASSED test/test_home_manager.py::test_initial_state_is_idle
PASSED test/test_home_manager.py::test_command_explore_transitions_to_exploring
PASSED test/test_home_manager.py::test_command_go_home_from_idle_does_nothing_without_home
PASSED test/test_home_manager.py::test_command_go_home_transitions_to_returning
PASSED test/test_home_manager.py::test_record_home_saves_pose
PASSED test/test_home_manager.py::test_exploration_done_saves_map_and_transitions
6 passed in 0.XXs
```

- [ ] **Step 8: Create `launch/home_manager.launch.py`**

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='amr_home_manager',
            executable='home_manager',
            name='amr_home_manager',
            output='screen',
        )
    ])
```

- [ ] **Step 9: Build**

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select amr_home_manager amr_explore
source install/setup.bash
```

- [ ] **Step 10: Commit**

```bash
git add src/amr_home_manager/ src/amr_explore/
git commit -m "feat: add amr_home_manager state machine and amr_explore config"
```

---

### Task 29: Integrate exploration into main bringup and E2E test

**Files:**
- Modify: `ros2_ws/src/amr_bringup/launch/amr.launch.py`

- [ ] **Step 1: Add home_manager include (always running)**

```python
home_manager_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        get_package_share_directory('amr_home_manager'),
        '/launch/home_manager.launch.py'
    ])
)
```

Add `home_manager_launch` to `LaunchDescription`. Do NOT include explore.launch.py in bringup — it is started/stopped dynamically by home_manager via `/explore/resume`.

Actually explore_lite must be running but paused. Update explore.yaml to add:

```yaml
explore:
  ros__parameters:
    # ... existing params ...
    start_with_rotations: false
```

And add explore_lite to bringup (it will wait for the `/explore/resume` topic):

```python
explore_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        get_package_share_directory('amr_explore'),
        '/launch/explore.launch.py'
    ])
)
```

- [ ] **Step 2: Build final bringup**

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select amr_bringup
source install/setup.bash
```

- [ ] **Step 3: E2E exploration test on RPi5**

Start everything:
```bash
ros2 launch amr_bringup amr.launch.py
```

Wait 10 s for TF tree to settle, then start exploration:
```bash
ros2 topic pub /amr/command std_msgs/msg/String '{"data": "explore"}' --once
```

Expected: robot starts moving toward frontiers. Watch `/map` in Foxglove growing. Monitor `/explore/resume` echoing `True`.

When the map looks complete (no more frontiers), stop exploration:
```bash
ros2 topic pub /amr/command std_msgs/msg/String '{"data": "stop"}' --once
```

Expected: robot stops, map save triggered (check `~/maps/` for new `.pgm` file).

Return home:
```bash
ros2 topic pub /amr/command std_msgs/msg/String '{"data": "go_home"}' --once
```

Expected: robot navigates back to its recorded start pose and stops.

- [ ] **Step 4: Commit**

```bash
git add src/amr_bringup/
git commit -m "feat: integrate exploration and home_manager into full bringup stack"
```


---

## Phase 8: Simulation — Gazebo Harmonic

**Goal:** Full stack runs inside Gazebo Harmonic (Docker container on WSL2 dev machine), with exact Ubuntu 24.04 + ROS2 Jazzy parity with the RPi5. Simulate the mecanum AMR in a room, run SLAM, and navigate to a goal — all in sim before touching the real robot.

**Milestone:** `docker compose up sim` → Gazebo opens with AMR model in a room. SLAM builds a map. `ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose` → robot drives to goal in sim.

---

### Task 30: Create Dockerfile and docker-compose for simulation

**Files:**
- Create: `docker/Dockerfile.sim`
- Create: `docker/docker-compose.yaml`
- Create: `docker/.env`

- [ ] **Step 1: Create `docker/Dockerfile.sim`**

```dockerfile
FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive

# Install Gazebo Harmonic and ROS-GZ bridge
RUN apt-get update && apt-get install -y \
  gz-harmonic \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-ros-gz-image \
  ros-jazzy-slam-toolbox \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-nav2-mppi-controller \
  ros-jazzy-nav2-smac-planner \
  ros-jazzy-nav2-collision-monitor \
  ros-jazzy-robot-localization \
  ros-jazzy-imu-filter-madgwick \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-topic-tools \
  python3-colcon-common-extensions \
  && rm -rf /var/lib/apt/lists/*

# Copy workspace source
COPY ros2_ws/src /ros2_ws/src

WORKDIR /ros2_ws

# Build workspace
RUN bash -c "source /opt/ros/jazzy/setup.bash && \
  colcon build --symlink-install \
  --packages-ignore amr_hardware"

# Entrypoint
COPY docker/ros_entrypoint.sh /
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
```

- [ ] **Step 2: Create `docker/ros_entrypoint.sh`**

```bash
#!/bin/bash
set -e
source /opt/ros/jazzy/setup.bash
if [ -f /ros2_ws/install/setup.bash ]; then
  source /ros2_ws/install/setup.bash
fi
exec "$@"
```

- [ ] **Step 3: Create `docker/docker-compose.yaml`**

```yaml
version: "3.9"

services:
  sim:
    build:
      context: ..
      dockerfile: docker/Dockerfile.sim
    image: amr-sim:latest
    environment:
      - DISPLAY=${DISPLAY}
      - ROS_DOMAIN_ID=42
      - GZ_VERSION=harmonic
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix:rw
      - ../ros2_ws/src:/ros2_ws/src:ro
      - ~/.gazebo:/root/.gazebo
    network_mode: host
    privileged: false
    command: >
      ros2 launch amr_bringup sim.launch.py
```

- [ ] **Step 4: Create `docker/.env`**

```bash
DISPLAY=:0
```

- [ ] **Step 5: Enable X11 forwarding on WSL2 host**

```bash
# Run on WSL2 host (one time per session)
export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0
xhost +local:docker
```

- [ ] **Step 6: Commit**

```bash
git add docker/
git commit -m "feat: add Gazebo Harmonic simulation Docker setup"
```

---

### Task 31: Create Gazebo world and AMR sim model

**Files:**
- Create: `ros2_ws/src/amr_description/worlds/room.sdf`
- Modify: `ros2_ws/src/amr_description/urdf/amr.urdf.xacro` (add Gazebo plugins)

- [ ] **Step 1: Create `worlds/room.sdf`**

A 6×6 m room with 4 walls and 3 box obstacles. Enough to test SLAM loop closure.

```xml
<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="room">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin filename="gz-sim-physics-system"
            name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-scene-broadcaster-system"
            name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system"
            name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system"
            name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-user-commands-system"
            name="gz::sim::systems::UserCommands"/>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry>
          <material><ambient>0.8 0.8 0.8 1</ambient></material>
        </visual>
      </link>
    </model>

    <!-- North wall -->
    <model name="wall_north">
      <static>true</static>
      <pose>0 3 0.5 0 0 0</pose>
      <link name="link">
        <collision name="col"><geometry><box><size>6 0.1 1</size></box></geometry></collision>
        <visual name="vis"><geometry><box><size>6 0.1 1</size></box></geometry>
          <material><ambient>0.5 0.5 0.5 1</ambient></material></visual>
      </link>
    </model>

    <!-- South wall -->
    <model name="wall_south">
      <static>true</static>
      <pose>0 -3 0.5 0 0 0</pose>
      <link name="link">
        <collision name="col"><geometry><box><size>6 0.1 1</size></box></geometry></collision>
        <visual name="vis"><geometry><box><size>6 0.1 1</size></box></geometry>
          <material><ambient>0.5 0.5 0.5 1</ambient></material></visual>
      </link>
    </model>

    <!-- East wall -->
    <model name="wall_east">
      <static>true</static>
      <pose>3 0 0.5 0 0 1.5708</pose>
      <link name="link">
        <collision name="col"><geometry><box><size>6 0.1 1</size></box></geometry></collision>
        <visual name="vis"><geometry><box><size>6 0.1 1</size></box></geometry>
          <material><ambient>0.5 0.5 0.5 1</ambient></material></visual>
      </link>
    </model>

    <!-- West wall -->
    <model name="wall_west">
      <static>true</static>
      <pose>-3 0 0.5 0 0 1.5708</pose>
      <link name="link">
        <collision name="col"><geometry><box><size>6 0.1 1</size></box></geometry></collision>
        <visual name="vis"><geometry><box><size>6 0.1 1</size></box></geometry>
          <material><ambient>0.5 0.5 0.5 1</ambient></material></visual>
      </link>
    </model>

    <!-- Obstacle 1 -->
    <model name="box1">
      <static>true</static>
      <pose>1.5 1.0 0.25 0 0 0</pose>
      <link name="link">
        <collision name="col"><geometry><box><size>0.5 0.5 0.5</size></box></geometry></collision>
        <visual name="vis"><geometry><box><size>0.5 0.5 0.5</size></box></geometry>
          <material><ambient>0.8 0.2 0.2 1</ambient></material></visual>
      </link>
    </model>

    <!-- Obstacle 2 -->
    <model name="box2">
      <static>true</static>
      <pose>-1.0 -1.5 0.25 0 0 0</pose>
      <link name="link">
        <collision name="col"><geometry><box><size>0.4 0.8 0.5</size></box></geometry></collision>
        <visual name="vis"><geometry><box><size>0.4 0.8 0.5</size></box></geometry>
          <material><ambient>0.2 0.2 0.8 1</ambient></material></visual>
      </link>
    </model>

    <!-- Obstacle 3 -->
    <model name="box3">
      <static>true</static>
      <pose>-1.5 1.5 0.25 0 0 0.7854</pose>
      <link name="link">
        <collision name="col"><geometry><box><size>0.6 0.3 0.5</size></box></geometry></collision>
        <visual name="vis"><geometry><box><size>0.6 0.3 0.5</size></box></geometry>
          <material><ambient>0.2 0.8 0.2 1</ambient></material></visual>
      </link>
    </model>

  </world>
</sdf>
```

- [ ] **Step 2: Add install rule for worlds in amr_description CMakeLists.txt**

Open `ros2_ws/src/amr_description/CMakeLists.txt` and add:

```cmake
install(DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}
)
```

- [ ] **Step 3: Commit**

```bash
git add src/amr_description/worlds/ src/amr_description/CMakeLists.txt
git commit -m "feat: add 6x6m Gazebo Harmonic room world with 3 obstacles"
```

---

### Task 32: Create `sim.launch.py`

This launch file replaces `amr.launch.py` in simulation — it starts Gazebo instead of real hardware, bridges topics, and still runs the full navigation stack.

**Files:**
- Create: `ros2_ws/src/amr_bringup/launch/sim.launch.py`

- [ ] **Step 1: Create `launch/sim.launch.py`**

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    amr_desc_pkg = get_package_share_directory('amr_description')
    amr_nav_pkg  = get_package_share_directory('amr_nav')
    amr_slam_pkg = get_package_share_directory('amr_slam')
    amr_sf_pkg   = get_package_share_directory('amr_sensor_fusion')
    amr_hm_pkg   = get_package_share_directory('amr_home_manager')

    world_path = os.path.join(amr_desc_pkg, 'worlds', 'room.sdf')
    urdf_path  = os.path.join(amr_desc_pkg, 'urdf', 'amr.urdf.xacro')

    # 1. Gazebo Harmonic
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_path],
        output='screen',
    )

    # 2. Spawn robot into Gazebo
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'amr',
            '-topic', '/robot_description',
            '-x', '0', '-y', '0', '-z', '0.05',
        ],
        output='screen',
    )

    # 3. robot_state_publisher
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': open(urdf_path).read() if os.path.exists(urdf_path) else '',
            'use_sim_time': True,
        }],
    )

    # 4. ros_gz_bridge — bridge Gazebo topics to ROS2
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/imu/data_raw@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/odom/wheel@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/cmd_vel_safe@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        output='screen',
    )

    # 5. Foxglove bridge for visualization
    foxglove = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        parameters=[{'port': 8765, 'use_sim_time': True}],
    )

    # 6. Sensor fusion, SLAM, Nav2, home manager — all with use_sim_time:true
    sensor_fusion = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(amr_sf_pkg, 'launch', 'sensor_fusion.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(amr_slam_pkg, 'launch', 'slam.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(amr_nav_pkg, 'launch', 'nav2.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )

    home_manager = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(amr_hm_pkg, 'launch', 'home_manager.launch.py')),
    )

    return LaunchDescription([
        gazebo,
        rsp,
        TimerAction(period=2.0, actions=[spawn]),
        TimerAction(period=3.0, actions=[bridge]),
        TimerAction(period=4.0, actions=[sensor_fusion, slam]),
        TimerAction(period=6.0, actions=[nav2]),
        TimerAction(period=8.0, actions=[foxglove, home_manager]),
    ])
```

- [ ] **Step 2: Update all launch configs to accept `use_sim_time` arg**

In `amr_sensor_fusion/launch/sensor_fusion.launch.py`, `amr_slam/launch/slam.launch.py`, `amr_nav/launch/nav2.launch.py` — add `use_sim_time` as a `DeclareLaunchArgument` so it can be overridden:

```python
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    # pass use_sim_time to each node's parameters dict
```

- [ ] **Step 3: Build in Docker**

```bash
cd /path/to/AMR
docker compose -f docker/docker-compose.yaml build sim
```

Expected: Docker image `amr-sim:latest` built, ~3-5 minutes.

- [ ] **Step 4: Run simulation**

```bash
# Enable X11 on WSL2 host first
export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0
xhost +local:docker

docker compose -f docker/docker-compose.yaml up sim
```

Expected: Gazebo window opens with the 6×6 m room. AMR model spawns at origin. LiDAR scan visible.

- [ ] **Step 5: Verify topics**

In a second terminal inside the container:
```bash
docker exec -it amr-sim-1 bash
source /ros2_ws/install/setup.bash
ros2 topic list
```

Expected topics present:
- `/scan`
- `/imu/data_raw`
- `/odom/wheel`
- `/map`
- `/odom`

- [ ] **Step 6: Navigate to goal in sim**

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 1.5, z: 0.0}, orientation: {w: 1.0}}}}"
```

Expected: robot in Gazebo drives toward (2.0, 1.5), avoiding obstacles.

- [ ] **Step 7: Commit**

```bash
git add src/amr_bringup/launch/sim.launch.py src/amr_sensor_fusion/ src/amr_slam/ src/amr_nav/
git commit -m "feat: add sim.launch.py and Gazebo Harmonic full-stack simulation"
```

---

## Phase 9: Deployment & Production

**Goal:** The AMR stack starts automatically on RPi5 boot via systemd, connects to WiFi, and is ready to accept commands within 30 s. Foxglove accessible from any device on the LAN. Full end-to-end test: power on → explore → go home → navigate to user-clicked goal.

**Milestone:** Unplug and replug RPi5. After 30 s, Foxglove on the dev machine shows `/scan` data and a growing map. All AMR functionality works with no manual SSH required.

---

### Task 33: Finalize udev rules and verify serial device

**Files:**
- Create: `deploy/udev/99-amr.rules`
- Create: `deploy/scripts/install_udev.sh`

- [ ] **Step 1: Create `deploy/udev/99-amr.rules`**

These rules create stable device paths that survive USB re-enumeration.

```bash
# LiDAR — Slamtec C1M1 R2
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", \
  SYMLINK+="lidar", MODE="0666"

# AMR MCU — ESP32-P4 (Silicon Labs CP2102N or similar)
# Find idVendor/idProduct with: udevadm info -a -n /dev/ttyUSB1 | grep idVendor
SUBSYSTEM=="tty", ATTRS{idVendor}=="303a", ATTRS{idProduct}=="1001", \
  SYMLINK+="amr_mcu", MODE="0666"
```

> **Note:** The ESP32-P4 USB-JTAG/Serial idVendor is `303a`, idProduct `1001` for the built-in USB-Serial. Verify with:
> ```bash
> udevadm info -a -n /dev/ttyUSB0 | grep -E 'idVendor|idProduct'
> ```
> Update the rule if different.

- [ ] **Step 2: Create `deploy/scripts/install_udev.sh`**

```bash
#!/bin/bash
set -e
echo "Installing udev rules..."
sudo cp "$(dirname "$0")/../udev/99-amr.rules" /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
echo "Udev rules installed. Reconnect USB devices."
ls -la /dev/lidar /dev/amr_mcu 2>/dev/null || echo "Devices not yet detected — reconnect USB."
```

```bash
chmod +x deploy/scripts/install_udev.sh
```

- [ ] **Step 3: Run on RPi5**

Copy deploy/ to RPi5:
```bash
scp -r deploy/ ubuntu@<rpi5-ip>:~/AMR/
ssh ubuntu@<rpi5-ip> "bash ~/AMR/deploy/scripts/install_udev.sh"
```

- [ ] **Step 4: Verify device paths**

```bash
ssh ubuntu@<rpi5-ip>
ls -la /dev/lidar /dev/amr_mcu
```

Expected:
```
lrwxrwxrwx 1 root root 7 ...  /dev/amr_mcu -> ttyUSB0
lrwxrwxrwx 1 root root 7 ...  /dev/lidar   -> ttyUSB1
```

- [ ] **Step 5: Commit**

```bash
git add deploy/
git commit -m "feat: add udev rules for stable /dev/lidar and /dev/amr_mcu paths"
```

---

### Task 34: Configure static WiFi IP on RPi5

The RPi5 needs a predictable LAN IP so Foxglove can connect without checking DHCP leases.

- [ ] **Step 1: Find the WiFi interface name**

```bash
ssh ubuntu@<rpi5-ip> "ip link show | grep -E 'wlan|wlp'"
```

Note the interface name (typically `wlan0` or `wlp3s0`).

- [ ] **Step 2: Create netplan config**

```bash
ssh ubuntu@<rpi5-ip>
sudo tee /etc/netplan/10-amr-wifi.yaml << 'EOF'
network:
  version: 2
  wifis:
    wlan0:                    # Replace with actual interface name
      dhcp4: no
      addresses: [192.168.1.100/24]
      gateway4: 192.168.1.1
      nameservers:
        addresses: [8.8.8.8, 8.8.4.4]
      access-points:
        "YOUR_SSID":          # Replace with your WiFi SSID
          password: "YOUR_PASSWORD"
EOF
sudo chmod 600 /etc/netplan/10-amr-wifi.yaml
sudo netplan apply
```

- [ ] **Step 3: Verify static IP**

```bash
ip addr show wlan0
```

Expected: `inet 192.168.1.100/24` listed.

- [ ] **Step 4: Update Foxglove connection on dev machine**

In Foxglove Studio: set WebSocket URL to `ws://192.168.1.100:8765`.

---

### Task 35: Create systemd service for auto-start

**Files:**
- Create: `deploy/systemd/amr.service`
- Create: `deploy/scripts/install_service.sh`

- [ ] **Step 1: Create `deploy/systemd/amr.service`**

```ini
[Unit]
Description=AMR Navigation Stack
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=ubuntu
Environment="ROS_DOMAIN_ID=42"
Environment="HOME=/home/ubuntu"
ExecStartPre=/bin/bash -c "source /opt/ros/jazzy/setup.bash && \
  source /home/ubuntu/ros2_ws/install/setup.bash"
ExecStart=/bin/bash -c "source /opt/ros/jazzy/setup.bash && \
  source /home/ubuntu/ros2_ws/install/setup.bash && \
  ros2 launch amr_bringup amr.launch.py"
Restart=on-failure
RestartSec=5s
StandardOutput=journal
StandardError=journal
SyslogIdentifier=amr

[Install]
WantedBy=multi-user.target
```

- [ ] **Step 2: Create `deploy/scripts/install_service.sh`**

```bash
#!/bin/bash
set -e
echo "Installing AMR systemd service..."
sudo cp "$(dirname "$0")/../systemd/amr.service" /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable amr.service
echo "Service installed and enabled."
echo "Start now with: sudo systemctl start amr.service"
echo "View logs with: journalctl -u amr -f"
```

```bash
chmod +x deploy/scripts/install_service.sh
```

- [ ] **Step 3: Deploy and install on RPi5**

```bash
scp -r deploy/ ubuntu@192.168.1.100:~/AMR/
ssh ubuntu@192.168.1.100 "bash ~/AMR/deploy/scripts/install_service.sh"
```

- [ ] **Step 4: Start and verify**

```bash
ssh ubuntu@192.168.1.100
sudo systemctl start amr.service
sudo systemctl status amr.service
```

Expected: `Active: active (running)`.

Check logs:
```bash
journalctl -u amr -f
```

Expected within 30 s:
```
[robot_state_publisher]: publishing...
[slam_toolbox]: Registering sensor: [LidarSensor]
[ekf_filter_node]: Estimator initialized
```

- [ ] **Step 5: Commit**

```bash
git add deploy/
git commit -m "feat: add systemd amr.service for auto-start on RPi5 boot"
```

---

### Task 36: Full end-to-end boot test

This task has no code — it is the final integration verification checklist.

- [ ] **Step 1: Cold boot test**

Power off RPi5 completely. Wait 5 s. Power on.

SSH in after 30 s:
```bash
ssh ubuntu@192.168.1.100
sudo systemctl status amr.service
```

Expected: `Active: active (running)` without any manual intervention.

- [ ] **Step 2: Verify Foxglove connects**

Open Foxglove Studio on dev machine. Connect to `ws://192.168.1.100:8765`.

Expected topics visible:
- `/scan` — LiDAR spinning
- `/map` — occupancy grid initializing
- `/odom` — EKF output updating at 50 Hz
- `/imu/data` — IMU data flowing
- `/tof/points` — ToF point cloud

- [ ] **Step 3: Explore the room**

```bash
ros2 topic pub /amr/command std_msgs/msg/String '{"data": "explore"}' --once
```

Watch map build in Foxglove. Wait until frontiers are exhausted.

Stop exploration:
```bash
ros2 topic pub /amr/command std_msgs/msg/String '{"data": "stop"}' --once
```

Verify `~/maps/` has a new `.pgm` file on RPi5.

- [ ] **Step 4: Return home**

```bash
ros2 topic pub /amr/command std_msgs/msg/String '{"data": "go_home"}' --once
```

Expected: robot navigates back to its boot position and stops.

- [ ] **Step 5: Goal navigation**

In Foxglove, add a **Publish** panel for `/goal_pose` (`geometry_msgs/PoseStamped`, frame `map`). Click a point on the map. Publish.

Expected: robot plans path and drives to goal within 0.15 m.

- [ ] **Step 6: Obstacle avoidance stress test**

While robot is en route to a goal, place a box in its path. Expected: robot replans around the obstacle. Remove box. Robot continues to original goal.

- [ ] **Step 7: Final commit and tag**

```bash
cd ~/AMR
git add -A
git commit -m "chore: final integration verified — full AMR stack operational"
git tag v1.0.0
```

