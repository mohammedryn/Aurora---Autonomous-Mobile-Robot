# Aurora Gazebo Simulation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a visually polished Gazebo Harmonic warehouse simulation of Aurora that runs a full autonomous explore → map → home → nav-goal mission with one command.

**Architecture:** Gazebo Harmonic provides physics, LiDAR, and IMU via sim plugins; `gz_ros2_control` (already in the URDF) drives the mecanum controller; all existing ROS2 nodes (SLAM, Nav2, amr_explore, amr_home_manager) run unchanged with `use_sim_time: true` injected via patched YAML at launch time.

**Tech Stack:** Gazebo Harmonic, gz_ros2_control, ros_gz_bridge, ROS2 Jazzy, slam_toolbox, nav2, robot_localization EKF, explore_lite, Docker

**Spec:** `docs/superpowers/specs/2026-06-17-gazebo-simulation-design.md`

---

## File Map

| File | Action | Purpose |
|---|---|---|
| `ros2_ws/src/amr_bringup/CMakeLists.txt` | Modify | Add `rviz` to install dirs |
| `ros2_ws/src/amr_description/urdf/amr.urdf.xacro` | Modify | Visual upgrade: charcoal chassis, blue accent, LEDs, sensor tower |
| `ros2_ws/src/amr_description/urdf/wheels.urdf.xacro` | Modify | Orange-amber wheels |
| `ros2_ws/src/amr_description/urdf/sensors.urdf.xacro` | Modify | Enable LiDAR `<visualize>true</visualize>` |
| `ros2_ws/src/amr_description/worlds/warehouse.sdf` | Create | 24×18m warehouse world |
| `ros2_ws/src/amr_bringup/launch/sim.launch.py` | Create | Full simulation launch |
| `ros2_ws/src/amr_bringup/rviz/sim.rviz` | Create | Dark-theme RViz2 config |
| `docker/Dockerfile` | Modify | Add sim deps + bags/maps dirs |
| `docker/run_sim.sh` | Modify | Fix volume path + add --device /dev/dri |
| `scripts/demo_sim.sh` | Create | One-command launcher |
| `scripts/record_demo.sh` | Create | ffmpeg screen capture to MP4 |

---

## Task 1: CMakeLists patch — add rviz install dir

**Files:**
- Modify: `ros2_ws/src/amr_bringup/CMakeLists.txt`

- [ ] **Step 1: Add `rviz` to the install directive**

In `ros2_ws/src/amr_bringup/CMakeLists.txt`, change:
```cmake
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)
```
to:
```cmake
install(DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)
```

- [ ] **Step 2: Create the rviz directory**

```bash
mkdir -p ros2_ws/src/amr_bringup/rviz
```

- [ ] **Step 3: Verify build passes**

```bash
cd ros2_ws && colcon build --packages-select amr_bringup 2>&1 | tail -5
```
Expected: `Summary: 1 package finished`

- [ ] **Step 4: Commit**

```bash
git add ros2_ws/src/amr_bringup/CMakeLists.txt
git commit -m "build(amr_bringup): add rviz to install dirs"
```

---

## Task 2: Robot URDF visual upgrade

**Files:**
- Modify: `ros2_ws/src/amr_description/urdf/amr.urdf.xacro`
- Modify: `ros2_ws/src/amr_description/urdf/wheels.urdf.xacro`
- Modify: `ros2_ws/src/amr_description/urdf/sensors.urdf.xacro`

### 2a — Chassis, accent strip, sensor tower, corner LEDs in `amr.urdf.xacro`

- [ ] **Step 1: Replace the chassis visual material and add visual elements**

In `ros2_ws/src/amr_description/urdf/amr.urdf.xacro`, replace:
```xml
  <!-- Main chassis -->
  <link name="base_link">
    <visual>
      <geometry><box size="0.715 0.565 0.060"/></geometry>
      <material name="grey"><color rgba="0.6 0.6 0.6 1"/></material>
    </visual>
```
with:
```xml
  <!-- Main chassis -->
  <link name="base_link">
    <!-- Charcoal body -->
    <visual name="chassis_body">
      <geometry><box size="0.715 0.565 0.060"/></geometry>
      <material name="charcoal"><color rgba="0.15 0.15 0.18 1"/></material>
    </visual>
    <!-- Bright blue perimeter accent strip -->
    <visual name="chassis_accent">
      <origin xyz="0 0 0.032" rpy="0 0 0"/>
      <geometry><box size="0.720 0.570 0.004"/></geometry>
      <material name="blue_accent"><color rgba="0.2 0.5 1.0 1"/></material>
    </visual>
```

- [ ] **Step 2: Add sensor tower link + joint after the four wheel macros**

After the four `<xacro:mecanum_wheel .../>` lines and before `<ros2_control ...>`, insert:
```xml
  <!-- Sensor tower — matte black cylinder stack below LiDAR -->
  <link name="sensor_tower">
    <visual>
      <geometry><cylinder radius="0.042" length="0.065"/></geometry>
      <material name="matte_black"><color rgba="0.08 0.08 0.08 1"/></material>
    </visual>
  </link>
  <joint name="sensor_tower_joint" type="fixed">
    <parent link="base_link"/>
    <child  link="sensor_tower"/>
    <origin xyz="0.327 0.0 0.063" rpy="0 0 0"/>
  </joint>

  <!-- Corner LED indicator spheres -->
  <xacro:macro name="corner_led" params="name x y">
    <link name="${name}">
      <visual>
        <geometry><sphere radius="0.013"/></geometry>
        <material name="led_green"><color rgba="0.1 0.9 0.3 1"/></material>
      </visual>
    </link>
    <joint name="${name}_joint" type="fixed">
      <parent link="base_link"/>
      <child  link="${name}"/>
      <origin xyz="${x} ${y} 0.032" rpy="0 0 0"/>
    </joint>
  </xacro:macro>

  <xacro:corner_led name="led_FL" x=" 0.340" y=" 0.265"/>
  <xacro:corner_led name="led_FR" x=" 0.340" y="-0.265"/>
  <xacro:corner_led name="led_RL" x="-0.340" y=" 0.265"/>
  <xacro:corner_led name="led_RR" x="-0.340" y="-0.265"/>
```

### 2b — Orange wheels in `wheels.urdf.xacro`

- [ ] **Step 3: Change wheel material to orange-amber**

In `ros2_ws/src/amr_description/urdf/wheels.urdf.xacro`, replace:
```xml
        <material name="black"><color rgba="0.1 0.1 0.1 1"/></material>
```
with:
```xml
        <material name="orange_wheel"><color rgba="0.95 0.55 0.1 1"/></material>
```

### 2c — Enable LiDAR sweep visualisation in `sensors.urdf.xacro`

- [ ] **Step 4: Enable LiDAR visualize**

In `ros2_ws/src/amr_description/urdf/sensors.urdf.xacro`, replace:
```xml
        <always_on>true</always_on>
        <visualize>false</visualize>
```
with:
```xml
        <always_on>true</always_on>
        <visualize>true</visualize>
```

- [ ] **Step 5: Build and verify no xacro errors**

```bash
cd ros2_ws && colcon build --packages-select amr_description 2>&1 | tail -5
```
Expected: `Summary: 1 package finished`

Then verify xacro processes cleanly:
```bash
source ros2_ws/install/setup.bash
ros2 run xacro xacro ros2_ws/src/amr_description/urdf/amr.urdf.xacro use_sim:=true > /tmp/amr_sim.urdf && echo "URDF OK"
```
Expected: `URDF OK`

- [ ] **Step 6: Commit**

```bash
git add ros2_ws/src/amr_description/urdf/
git commit -m "feat(description): visual upgrade — charcoal chassis, blue accent, orange wheels, LEDs, sensor tower"
```

---

## Task 3: warehouse.sdf — world skeleton

**Files:**
- Create: `ros2_ws/src/amr_description/worlds/warehouse.sdf`

The warehouse is 24 × 18 m centred at the origin. Aurora spawns at (0, −7, 0.1). Shelves fill the north. The loading dock (south) is clear.

- [ ] **Step 1: Create the world skeleton**

Create `ros2_ws/src/amr_description/worlds/warehouse.sdf`:

```xml
<?xml version="1.0" ?>
<sdf version="1.10">
<world name="warehouse">

  <!-- ── Physics ─────────────────────────────────────────────────────── -->
  <physics name="default_physics" default="true" type="ode">
    <max_step_size>0.001</max_step_size>
    <real_time_factor>1.0</real_time_factor>
    <real_time_update_rate>1000</real_time_update_rate>
  </physics>

  <!-- ── Required Gazebo Harmonic plugins ───────────────────────────── -->
  <plugin filename="gz-sim-physics-system"
          name="gz::sim::systems::Physics"/>
  <plugin filename="gz-sim-sensors-system"
          name="gz::sim::systems::Sensors">
    <render_engine>ogre2</render_engine>
  </plugin>
  <plugin filename="gz-sim-scene-broadcaster-system"
          name="gz::sim::systems::SceneBroadcaster"/>
  <plugin filename="gz-sim-user-commands-system"
          name="gz::sim::systems::UserCommands"/>
  <plugin filename="gz-sim-imu-system"
          name="gz::sim::systems::Imu"/>

  <!-- ── Scene: cool warehouse ambient, shadows on ──────────────────── -->
  <scene>
    <ambient>0.3 0.3 0.35 1</ambient>
    <background>0.55 0.58 0.62 1</background>
    <shadows>true</shadows>
    <sky>
      <clouds>
        <speed>10</speed>
      </clouds>
    </sky>
  </scene>

  <!-- ── Directional sun (cast shadows, 45° azimuth) ───────────────── -->
  <light name="sun" type="directional">
    <cast_shadows>true</cast_shadows>
    <pose>0 0 20 0 0.6 0.8</pose>
    <diffuse>0.85 0.85 0.80 1</diffuse>
    <specular>0.15 0.15 0.15 1</specular>
    <direction>-0.5 0.2 -0.9</direction>
    <attenuation>
      <range>1000</range>
      <constant>0.9</constant>
      <linear>0.01</linear>
      <quadratic>0.001</quadratic>
    </attenuation>
  </light>

  <!-- ── Concrete floor ─────────────────────────────────────────────── -->
  <model name="ground_plane">
    <static>true</static>
    <pose>0 0 0 0 0 0</pose>
    <link name="link">
      <collision name="collision">
        <geometry>
          <plane><normal>0 0 1</normal><size>30 25</size></plane>
        </geometry>
        <surface>
          <friction><ode><mu>0.8</mu><mu2>0.8</mu2></ode></friction>
        </surface>
      </collision>
      <visual name="visual">
        <geometry>
          <plane><normal>0 0 1</normal><size>30 25</size></plane>
        </geometry>
        <material>
          <ambient>0.45 0.45 0.45 1</ambient>
          <diffuse>0.55 0.55 0.55 1</diffuse>
          <specular>0.05 0.05 0.05 1</specular>
        </material>
      </visual>
    </link>
  </model>

  <!-- ── Perimeter walls (0.2 m thick, 5 m high) ───────────────────── -->
  <model name="wall_north">
    <static>true</static>
    <pose>0 9.1 2.5 0 0 0</pose>
    <link name="link">
      <collision name="col"><geometry><box><size>24.4 0.2 5.0</size></box></geometry></collision>
      <visual name="vis">
        <geometry><box><size>24.4 0.2 5.0</size></box></geometry>
        <material><ambient>0.5 0.48 0.45 1</ambient><diffuse>0.65 0.62 0.58 1</diffuse></material>
      </visual>
    </link>
  </model>

  <model name="wall_south">
    <static>true</static>
    <pose>0 -9.1 2.5 0 0 0</pose>
    <link name="link">
      <collision name="col"><geometry><box><size>24.4 0.2 5.0</size></box></geometry></collision>
      <visual name="vis">
        <geometry><box><size>24.4 0.2 5.0</size></box></geometry>
        <material><ambient>0.5 0.48 0.45 1</ambient><diffuse>0.65 0.62 0.58 1</diffuse></material>
      </visual>
    </link>
  </model>

  <model name="wall_east">
    <static>true</static>
    <pose>12.1 0 2.5 0 0 0</pose>
    <link name="link">
      <collision name="col"><geometry><box><size>0.2 18.0 5.0</size></box></geometry></collision>
      <visual name="vis">
        <geometry><box><size>0.2 18.0 5.0</size></box></geometry>
        <material><ambient>0.5 0.48 0.45 1</ambient><diffuse>0.65 0.62 0.58 1</diffuse></material>
      </visual>
    </link>
  </model>

  <model name="wall_west">
    <static>true</static>
    <pose>-12.1 0 2.5 0 0 0</pose>
    <link name="link">
      <collision name="col"><geometry><box><size>0.2 18.0 5.0</size></box></geometry></collision>
      <visual name="vis">
        <geometry><box><size>0.2 18.0 5.0</size></box></geometry>
        <material><ambient>0.5 0.48 0.45 1</ambient><diffuse>0.65 0.62 0.58 1</diffuse></material>
      </visual>
    </link>
  </model>

  <!-- ── Ceiling (visual only, no collision — keeps light in) ──────── -->
  <model name="ceiling">
    <static>true</static>
    <pose>0 0 5.02 0 0 0</pose>
    <link name="link">
      <visual name="vis">
        <geometry><box><size>24.4 18.4 0.04</size></box></geometry>
        <material><ambient>0.7 0.7 0.7 1</ambient><diffuse>0.8 0.8 0.8 1</diffuse></material>
      </visual>
    </link>
  </model>

```

- [ ] **Step 2: Verify skeleton loads in Gazebo headless**

```bash
gz sim --headless-rendering -r \
  ros2_ws/src/amr_description/worlds/warehouse.sdf \
  --iterations 200 -v 3 2>&1 | grep -E "Error|error|Warning|Loaded" | head -20
```
Expected: no `Error` lines, see `Loaded [warehouse]`.

---

## Task 4: warehouse.sdf — static props

**Files:**
- Modify: `ros2_ws/src/amr_description/worlds/warehouse.sdf`

Append all models before the closing `</world>` tag.

### Shelf layout

Shelves are 2.0 × 0.4 × 2.0 m (X × Y × Z), metal grey. Three rows of 5:

| Row | Y centre | X centres |
|---|---|---|
| Row 1 (north) | 4.0 | −9, −5, −1, 3, 7 |
| Row 2 (mid) | 0.0 | −9, −5, −1, 3, 7 |
| Row 3 (south) | −4.0 | −9, −5, −1, 3, 7 |

Shelf z-centre = 1.0 m (sitting on floor).

- [ ] **Step 1: Add shelf macro helper comment then all 15 shelf models**

Append to `warehouse.sdf` (before `</world>`):

```xml
  <!-- ════════════════════════════════════════════════════════════════
       SHELVES — 3 rows × 5, metal grey, 2.0×0.4×2.0 m
       Aisles run N-S between columns; cross-aisles run E-W between rows
       ════════════════════════════════════════════════════════════════ -->

  <!-- Row 1 (y=4.0) -->
  <model name="shelf_r1_0"><static>true</static><pose>-9 4.0 1.0 0 0 0</pose>
    <link name="l"><collision name="c"><geometry><box><size>2.0 0.4 2.0</size></box></geometry></collision>
    <visual name="v"><geometry><box><size>2.0 0.4 2.0</size></box></geometry>
    <material><ambient>0.35 0.35 0.38 1</ambient><diffuse>0.50 0.50 0.55 1</diffuse><specular>0.3 0.3 0.3 1</specular></material></visual></link></model>

  <model name="shelf_r1_1"><static>true</static><pose>-5 4.0 1.0 0 0 0</pose>
    <link name="l"><collision name="c"><geometry><box><size>2.0 0.4 2.0</size></box></geometry></collision>
    <visual name="v"><geometry><box><size>2.0 0.4 2.0</size></box></geometry>
    <material><ambient>0.35 0.35 0.38 1</ambient><diffuse>0.50 0.50 0.55 1</diffuse><specular>0.3 0.3 0.3 1</specular></material></visual></link></model>

  <model name="shelf_r1_2"><static>true</static><pose>-1 4.0 1.0 0 0 0</pose>
    <link name="l"><collision name="c"><geometry><box><size>2.0 0.4 2.0</size></box></geometry></collision>
    <visual name="v"><geometry><box><size>2.0 0.4 2.0</size></box></geometry>
    <material><ambient>0.35 0.35 0.38 1</ambient><diffuse>0.50 0.50 0.55 1</diffuse><specular>0.3 0.3 0.3 1</specular></material></visual></link></model>

  <model name="shelf_r1_3"><static>true</static><pose>3 4.0 1.0 0 0 0</pose>
    <link name="l"><collision name="c"><geometry><box><size>2.0 0.4 2.0</size></box></geometry></collision>
    <visual name="v"><geometry><box><size>2.0 0.4 2.0</size></box></geometry>
    <material><ambient>0.35 0.35 0.38 1</ambient><diffuse>0.50 0.50 0.55 1</diffuse><specular>0.3 0.3 0.3 1</specular></material></visual></link></model>

  <model name="shelf_r1_4"><static>true</static><pose>7 4.0 1.0 0 0 0</pose>
    <link name="l"><collision name="c"><geometry><box><size>2.0 0.4 2.0</size></box></geometry></collision>
    <visual name="v"><geometry><box><size>2.0 0.4 2.0</size></box></geometry>
    <material><ambient>0.35 0.35 0.38 1</ambient><diffuse>0.50 0.50 0.55 1</diffuse><specular>0.3 0.3 0.3 1</specular></material></visual></link></model>

  <!-- Row 2 (y=0.0) -->
  <model name="shelf_r2_0"><static>true</static><pose>-9 0.0 1.0 0 0 0</pose>
    <link name="l"><collision name="c"><geometry><box><size>2.0 0.4 2.0</size></box></geometry></collision>
    <visual name="v"><geometry><box><size>2.0 0.4 2.0</size></box></geometry>
    <material><ambient>0.35 0.35 0.38 1</ambient><diffuse>0.50 0.50 0.55 1</diffuse><specular>0.3 0.3 0.3 1</specular></material></visual></link></model>

  <model name="shelf_r2_1"><static>true</static><pose>-5 0.0 1.0 0 0 0</pose>
    <link name="l"><collision name="c"><geometry><box><size>2.0 0.4 2.0</size></box></geometry></collision>
    <visual name="v"><geometry><box><size>2.0 0.4 2.0</size></box></geometry>
    <material><ambient>0.35 0.35 0.38 1</ambient><diffuse>0.50 0.50 0.55 1</diffuse><specular>0.3 0.3 0.3 1</specular></material></visual></link></model>

  <model name="shelf_r2_2"><static>true</static><pose>-1 0.0 1.0 0 0 0</pose>
    <link name="l"><collision name="c"><geometry><box><size>2.0 0.4 2.0</size></box></geometry></collision>
    <visual name="v"><geometry><box><size>2.0 0.4 2.0</size></box></geometry>
    <material><ambient>0.35 0.35 0.38 1</ambient><diffuse>0.50 0.50 0.55 1</diffuse><specular>0.3 0.3 0.3 1</specular></material></visual></link></model>

  <model name="shelf_r2_3"><static>true</static><pose>3 0.0 1.0 0 0 0</pose>
    <link name="l"><collision name="c"><geometry><box><size>2.0 0.4 2.0</size></box></geometry></collision>
    <visual name="v"><geometry><box><size>2.0 0.4 2.0</size></box></geometry>
    <material><ambient>0.35 0.35 0.38 1</ambient><diffuse>0.50 0.50 0.55 1</diffuse><specular>0.3 0.3 0.3 1</specular></material></visual></link></model>

  <model name="shelf_r2_4"><static>true</static><pose>7 0.0 1.0 0 0 0</pose>
    <link name="l"><collision name="c"><geometry><box><size>2.0 0.4 2.0</size></box></geometry></collision>
    <visual name="v"><geometry><box><size>2.0 0.4 2.0</size></box></geometry>
    <material><ambient>0.35 0.35 0.38 1</ambient><diffuse>0.50 0.50 0.55 1</diffuse><specular>0.3 0.3 0.3 1</specular></material></visual></link></model>

  <!-- Row 3 (y=-4.0) -->
  <model name="shelf_r3_0"><static>true</static><pose>-9 -4.0 1.0 0 0 0</pose>
    <link name="l"><collision name="c"><geometry><box><size>2.0 0.4 2.0</size></box></geometry></collision>
    <visual name="v"><geometry><box><size>2.0 0.4 2.0</size></box></geometry>
    <material><ambient>0.35 0.35 0.38 1</ambient><diffuse>0.50 0.50 0.55 1</diffuse><specular>0.3 0.3 0.3 1</specular></material></visual></link></model>

  <model name="shelf_r3_1"><static>true</static><pose>-5 -4.0 1.0 0 0 0</pose>
    <link name="l"><collision name="c"><geometry><box><size>2.0 0.4 2.0</size></box></geometry></collision>
    <visual name="v"><geometry><box><size>2.0 0.4 2.0</size></box></geometry>
    <material><ambient>0.35 0.35 0.38 1</ambient><diffuse>0.50 0.50 0.55 1</diffuse><specular>0.3 0.3 0.3 1</specular></material></visual></link></model>

  <model name="shelf_r3_2"><static>true</static><pose>-1 -4.0 1.0 0 0 0</pose>
    <link name="l"><collision name="c"><geometry><box><size>2.0 0.4 2.0</size></box></geometry></collision>
    <visual name="v"><geometry><box><size>2.0 0.4 2.0</size></box></geometry>
    <material><ambient>0.35 0.35 0.38 1</ambient><diffuse>0.50 0.50 0.55 1</diffuse><specular>0.3 0.3 0.3 1</specular></material></visual></link></model>

  <model name="shelf_r3_3"><static>true</static><pose>3 -4.0 1.0 0 0 0</pose>
    <link name="l"><collision name="c"><geometry><box><size>2.0 0.4 2.0</size></box></geometry></collision>
    <visual name="v"><geometry><box><size>2.0 0.4 2.0</size></box></geometry>
    <material><ambient>0.35 0.35 0.38 1</ambient><diffuse>0.50 0.50 0.55 1</diffuse><specular>0.3 0.3 0.3 1</specular></material></visual></link></model>

  <model name="shelf_r3_4"><static>true</static><pose>7 -4.0 1.0 0 0 0</pose>
    <link name="l"><collision name="c"><geometry><box><size>2.0 0.4 2.0</size></box></geometry></collision>
    <visual name="v"><geometry><box><size>2.0 0.4 2.0</size></box></geometry>
    <material><ambient>0.35 0.35 0.38 1</ambient><diffuse>0.50 0.50 0.55 1</diffuse><specular>0.3 0.3 0.3 1</specular></material></visual></link></model>

  <!-- ── Inventory boxes on shelves (cardboard brown, 0.3×0.3×0.3 m) ─ -->
  <!-- 6 representative boxes visible on top shelf level -->
  <model name="box_s1"><static>true</static><pose>-9 4.0 2.15 0 0 0</pose>
    <link name="l"><collision name="c"><geometry><box><size>0.30 0.28 0.30</size></box></geometry></collision>
    <visual name="v"><geometry><box><size>0.30 0.28 0.30</size></box></geometry>
    <material><ambient>0.55 0.38 0.18 1</ambient><diffuse>0.70 0.48 0.22 1</diffuse></material></visual></link></model>
  <model name="box_s2"><static>true</static><pose>-8.5 4.0 2.15 0 0 0</pose>
    <link name="l"><collision name="c"><geometry><box><size>0.30 0.28 0.30</size></box></geometry></collision>
    <visual name="v"><geometry><box><size>0.30 0.28 0.30</size></box></geometry>
    <material><ambient>0.55 0.38 0.18 1</ambient><diffuse>0.70 0.48 0.22 1</diffuse></material></visual></link></model>
  <model name="box_s3"><static>true</static><pose>-5 0.0 2.15 0 0 0</pose>
    <link name="l"><collision name="c"><geometry><box><size>0.30 0.28 0.30</size></box></geometry></collision>
    <visual name="v"><geometry><box><size>0.30 0.28 0.30</size></box></geometry>
    <material><ambient>0.55 0.38 0.18 1</ambient><diffuse>0.70 0.48 0.22 1</diffuse></material></visual></link></model>
  <model name="box_s4"><static>true</static><pose>3 -4.0 2.15 0 0 0</pose>
    <link name="l"><collision name="c"><geometry><box><size>0.30 0.28 0.30</size></box></geometry></collision>
    <visual name="v"><geometry><box><size>0.30 0.28 0.30</size></box></geometry>
    <material><ambient>0.55 0.38 0.18 1</ambient><diffuse>0.70 0.48 0.22 1</diffuse></material></visual></link></model>
  <model name="box_s5"><static>true</static><pose>7 4.0 2.15 0 0 0</pose>
    <link name="l"><collision name="c"><geometry><box><size>0.30 0.28 0.30</size></box></geometry></collision>
    <visual name="v"><geometry><box><size>0.30 0.28 0.30</size></box></geometry>
    <material><ambient>0.55 0.38 0.18 1</ambient><diffuse>0.70 0.48 0.22 1</diffuse></material></visual></link></model>

  <!-- ── Floor pallets (wood brown, 1.2×0.8×0.15 m) ─────────────────── -->
  <model name="pallet_0"><static>true</static><pose>1.0 -6.5 0.075 0 0 0</pose>
    <link name="l"><collision name="c"><geometry><box><size>1.2 0.8 0.15</size></box></geometry></collision>
    <visual name="v"><geometry><box><size>1.2 0.8 0.15</size></box></geometry>
    <material><ambient>0.42 0.28 0.12 1</ambient><diffuse>0.55 0.38 0.18 1</diffuse></material></visual></link></model>
  <model name="pallet_1"><static>true</static><pose>-2.0 -7.0 0.075 0 0 0.3</pose>
    <link name="l"><collision name="c"><geometry><box><size>1.2 0.8 0.15</size></box></geometry></collision>
    <visual name="v"><geometry><box><size>1.2 0.8 0.15</size></box></geometry>
    <material><ambient>0.42 0.28 0.12 1</ambient><diffuse>0.55 0.38 0.18 1</diffuse></material></visual></link></model>
  <model name="pallet_2"><static>true</static><pose>3.5 -7.5 0.075 0 0 -0.2</pose>
    <link name="l"><collision name="c"><geometry><box><size>1.2 0.8 0.15</size></box></geometry></collision>
    <visual name="v"><geometry><box><size>1.2 0.8 0.15</size></box></geometry>
    <material><ambient>0.42 0.28 0.12 1</ambient><diffuse>0.55 0.38 0.18 1</diffuse></material></visual></link></model>
  <model name="pallet_3"><static>true</static><pose>-4.5 -6.0 0.075 0 0 1.57</pose>
    <link name="l"><collision name="c"><geometry><box><size>1.2 0.8 0.15</size></box></geometry></collision>
    <visual name="v"><geometry><box><size>1.2 0.8 0.15</size></box></geometry>
    <material><ambient>0.42 0.28 0.12 1</ambient><diffuse>0.55 0.38 0.18 1</diffuse></material></visual></link></model>
  <model name="pallet_4"><static>true</static><pose>5.0 -8.0 0.075 0 0 0</pose>
    <link name="l"><collision name="c"><geometry><box><size>1.2 0.8 0.15</size></box></geometry></collision>
    <visual name="v"><geometry><box><size>1.2 0.8 0.15</size></box></geometry>
    <material><ambient>0.42 0.28 0.12 1</ambient><diffuse>0.55 0.38 0.18 1</diffuse></material></visual></link></model>
  <model name="pallet_5"><static>true</static><pose>-3.0 -8.5 0.075 0 0 0.4</pose>
    <link name="l"><collision name="c"><geometry><box><size>1.2 0.8 0.15</size></box></geometry></collision>
    <visual name="v"><geometry><box><size>1.2 0.8 0.15</size></box></geometry>
    <material><ambient>0.42 0.28 0.12 1</ambient><diffuse>0.55 0.38 0.18 1</diffuse></material></visual></link></model>

  <!-- ── Structural pillars (concrete, 0.3×0.3×3.0 m) ────────────────── -->
  <model name="pillar_0"><static>true</static><pose>9.5 -5.0 1.5 0 0 0</pose>
    <link name="l"><collision name="c"><geometry><box><size>0.3 0.3 3.0</size></box></geometry></collision>
    <visual name="v"><geometry><box><size>0.3 0.3 3.0</size></box></geometry>
    <material><ambient>0.48 0.48 0.46 1</ambient><diffuse>0.60 0.60 0.58 1</diffuse></material></visual></link></model>
  <model name="pillar_1"><static>true</static><pose>9.5 -2.0 1.5 0 0 0</pose>
    <link name="l"><collision name="c"><geometry><box><size>0.3 0.3 3.0</size></box></geometry></collision>
    <visual name="v"><geometry><box><size>0.3 0.3 3.0</size></box></geometry>
    <material><ambient>0.48 0.48 0.46 1</ambient><diffuse>0.60 0.60 0.58 1</diffuse></material></visual></link></model>
  <model name="pillar_2"><static>true</static><pose>-9.5 -5.0 1.5 0 0 0</pose>
    <link name="l"><collision name="c"><geometry><box><size>0.3 0.3 3.0</size></box></geometry></collision>
    <visual name="v"><geometry><box><size>0.3 0.3 3.0</size></box></geometry>
    <material><ambient>0.48 0.48 0.46 1</ambient><diffuse>0.60 0.60 0.58 1</diffuse></material></visual></link></model>
  <model name="pillar_3"><static>true</static><pose>-9.5 -2.0 1.5 0 0 0</pose>
    <link name="l"><collision name="c"><geometry><box><size>0.3 0.3 3.0</size></box></geometry></collision>
    <visual name="v"><geometry><box><size>0.3 0.3 3.0</size></box></geometry>
    <material><ambient>0.48 0.48 0.46 1</ambient><diffuse>0.60 0.60 0.58 1</diffuse></material></visual></link></model>

  <!-- ── Forklift (static, yellow) ────────────────────────────────────── -->
  <model name="forklift">
    <static>true</static>
    <pose>10.5 -7.5 0 0 0 1.57</pose>
    <link name="base">
      <collision name="c"><geometry><box><size>1.8 0.9 0.5</size></box></geometry><pose>0 0 0.25 0 0 0</pose></collision>
      <visual name="v_base">
        <pose>0 0 0.25 0 0 0</pose>
        <geometry><box><size>1.8 0.9 0.5</size></box></geometry>
        <material><ambient>0.8 0.65 0.0 1</ambient><diffuse>0.95 0.78 0.0 1</diffuse></material>
      </visual>
      <visual name="v_mast">
        <pose>0.85 0 1.2 0 0 0</pose>
        <geometry><box><size>0.1 0.15 2.0</size></box></geometry>
        <material><ambient>0.3 0.3 0.3 1</ambient><diffuse>0.4 0.4 0.4 1</diffuse></material>
      </visual>
      <visual name="v_fork_l">
        <pose>1.1 0.2 0.15 0 0 0</pose>
        <geometry><box><size>0.9 0.06 0.04</size></box></geometry>
        <material><ambient>0.25 0.25 0.25 1</ambient><diffuse>0.35 0.35 0.35 1</diffuse></material>
      </visual>
      <visual name="v_fork_r">
        <pose>1.1 -0.2 0.15 0 0 0</pose>
        <geometry><box><size>0.9 0.06 0.04</size></box></geometry>
        <material><ambient>0.25 0.25 0.25 1</ambient><diffuse>0.35 0.35 0.35 1</diffuse></material>
      </visual>
    </link>
  </model>

  <!-- ── Overhead light fixtures + point lights ────────────────────── -->
  <!-- 6 fixtures at (x, y, z=4.9): (0,4), (-6,4), (6,4), (0,-1), (-6,-1), (6,-1) -->
  <model name="fixture_0"><static>true</static><pose>0 4.0 4.9 0 0 0</pose>
    <link name="l"><visual name="v"><geometry><box><size>0.6 0.15 0.05</size></box></geometry>
    <material><ambient>0.95 0.95 0.92 1</ambient><diffuse>1.0 1.0 0.95 1</diffuse>
    <emissive>0.8 0.8 0.75 1</emissive></material></visual></link></model>
  <model name="fixture_1"><static>true</static><pose>-6 4.0 4.9 0 0 0</pose>
    <link name="l"><visual name="v"><geometry><box><size>0.6 0.15 0.05</size></box></geometry>
    <material><ambient>0.95 0.95 0.92 1</ambient><diffuse>1.0 1.0 0.95 1</diffuse>
    <emissive>0.8 0.8 0.75 1</emissive></material></visual></link></model>
  <model name="fixture_2"><static>true</static><pose>6 4.0 4.9 0 0 0</pose>
    <link name="l"><visual name="v"><geometry><box><size>0.6 0.15 0.05</size></box></geometry>
    <material><ambient>0.95 0.95 0.92 1</ambient><diffuse>1.0 1.0 0.95 1</diffuse>
    <emissive>0.8 0.8 0.75 1</emissive></material></visual></link></model>
  <model name="fixture_3"><static>true</static><pose>0 -1.0 4.9 0 0 0</pose>
    <link name="l"><visual name="v"><geometry><box><size>0.6 0.15 0.05</size></box></geometry>
    <material><ambient>0.95 0.95 0.92 1</ambient><diffuse>1.0 1.0 0.95 1</diffuse>
    <emissive>0.8 0.8 0.75 1</emissive></material></visual></link></model>
  <model name="fixture_4"><static>true</static><pose>-6 -1.0 4.9 0 0 0</pose>
    <link name="l"><visual name="v"><geometry><box><size>0.6 0.15 0.05</size></box></geometry>
    <material><ambient>0.95 0.95 0.92 1</ambient><diffuse>1.0 1.0 0.95 1</diffuse>
    <emissive>0.8 0.8 0.75 1</emissive></material></visual></link></model>
  <model name="fixture_5"><static>true</static><pose>6 -1.0 4.9 0 0 0</pose>
    <link name="l"><visual name="v"><geometry><box><size>0.6 0.15 0.05</size></box></geometry>
    <material><ambient>0.95 0.95 0.92 1</ambient><diffuse>1.0 1.0 0.95 1</diffuse>
    <emissive>0.8 0.8 0.75 1</emissive></material></visual></link></model>

  <light name="pl_0" type="point"><pose>0  4.0 4.6 0 0 0</pose><diffuse>0.9 0.9 0.85 1</diffuse><specular>0.1 0.1 0.1 1</specular><attenuation><range>12</range><constant>0.4</constant><linear>0.02</linear><quadratic>0.002</quadratic></attenuation><cast_shadows>false</cast_shadows></light>
  <light name="pl_1" type="point"><pose>-6 4.0 4.6 0 0 0</pose><diffuse>0.9 0.9 0.85 1</diffuse><specular>0.1 0.1 0.1 1</specular><attenuation><range>12</range><constant>0.4</constant><linear>0.02</linear><quadratic>0.002</quadratic></attenuation><cast_shadows>false</cast_shadows></light>
  <light name="pl_2" type="point"><pose>6  4.0 4.6 0 0 0</pose><diffuse>0.9 0.9 0.85 1</diffuse><specular>0.1 0.1 0.1 1</specular><attenuation><range>12</range><constant>0.4</constant><linear>0.02</linear><quadratic>0.002</quadratic></attenuation><cast_shadows>false</cast_shadows></light>
  <light name="pl_3" type="point"><pose>0  -1.0 4.6 0 0 0</pose><diffuse>0.9 0.9 0.85 1</diffuse><specular>0.1 0.1 0.1 1</specular><attenuation><range>12</range><constant>0.4</constant><linear>0.02</linear><quadratic>0.002</quadratic></attenuation><cast_shadows>false</cast_shadows></light>
  <light name="pl_4" type="point"><pose>-6 -1.0 4.6 0 0 0</pose><diffuse>0.9 0.9 0.85 1</diffuse><specular>0.1 0.1 0.1 1</specular><attenuation><range>12</range><constant>0.4</constant><linear>0.02</linear><quadratic>0.002</quadratic></attenuation><cast_shadows>false</cast_shadows></light>
  <light name="pl_5" type="point"><pose>6  -1.0 4.6 0 0 0</pose><diffuse>0.9 0.9 0.85 1</diffuse><specular>0.1 0.1 0.1 1</specular><attenuation><range>12</range><constant>0.4</constant><linear>0.02</linear><quadratic>0.002</quadratic></attenuation><cast_shadows>false</cast_shadows></light>

  <!-- ── Safety floor stripes at loading dock entry (y≈-4.3) ──────── -->
  <!-- Alternating yellow/black stripes, 3m total width -->
  <model name="stripe_y0"><static>true</static><pose>-1.35 -4.3 0.001 0 0 0</pose>
    <link name="l"><visual name="v"><geometry><box><size>0.3 0.8 0.002</size></box></geometry>
    <material><ambient>0.9 0.8 0.0 1</ambient><diffuse>1.0 0.9 0.0 1</diffuse></material></visual></link></model>
  <model name="stripe_b0"><static>true</static><pose>-0.9 -4.3 0.001 0 0 0</pose>
    <link name="l"><visual name="v"><geometry><box><size>0.3 0.8 0.002</size></box></geometry>
    <material><ambient>0.05 0.05 0.05 1</ambient><diffuse>0.08 0.08 0.08 1</diffuse></material></visual></link></model>
  <model name="stripe_y1"><static>true</static><pose>-0.45 -4.3 0.001 0 0 0</pose>
    <link name="l"><visual name="v"><geometry><box><size>0.3 0.8 0.002</size></box></geometry>
    <material><ambient>0.9 0.8 0.0 1</ambient><diffuse>1.0 0.9 0.0 1</diffuse></material></visual></link></model>
  <model name="stripe_b1"><static>true</static><pose>0.0 -4.3 0.001 0 0 0</pose>
    <link name="l"><visual name="v"><geometry><box><size>0.3 0.8 0.002</size></box></geometry>
    <material><ambient>0.05 0.05 0.05 1</ambient><diffuse>0.08 0.08 0.08 1</diffuse></material></visual></link></model>
  <model name="stripe_y2"><static>true</static><pose>0.45 -4.3 0.001 0 0 0</pose>
    <link name="l"><visual name="v"><geometry><box><size>0.3 0.8 0.002</size></box></geometry>
    <material><ambient>0.9 0.8 0.0 1</ambient><diffuse>1.0 0.9 0.0 1</diffuse></material></visual></link></model>
  <model name="stripe_b2"><static>true</static><pose>0.9 -4.3 0.001 0 0 0</pose>
    <link name="l"><visual name="v"><geometry><box><size>0.3 0.8 0.002</size></box></geometry>
    <material><ambient>0.05 0.05 0.05 1</ambient><diffuse>0.08 0.08 0.08 1</diffuse></material></visual></link></model>
  <model name="stripe_y3"><static>true</static><pose>1.35 -4.3 0.001 0 0 0</pose>
    <link name="l"><visual name="v"><geometry><box><size>0.3 0.8 0.002</size></box></geometry>
    <material><ambient>0.9 0.8 0.0 1</ambient><diffuse>1.0 0.9 0.0 1</diffuse></material></visual></link></model>
```

- [ ] **Step 2: Verify world still loads cleanly**

```bash
gz sim --headless-rendering -r \
  ros2_ws/src/amr_description/worlds/warehouse.sdf \
  --iterations 200 -v 3 2>&1 | grep -E "^\\[Err\\]|^\\[Fatal\\]" | head -10
```
Expected: no output (no errors).

- [ ] **Step 3: Commit**

```bash
git add ros2_ws/src/amr_description/worlds/warehouse.sdf
git commit -m "feat(worlds): warehouse static props — shelves, pallets, pillars, forklift, lights, stripes"
```

---

## Task 5: warehouse.sdf — dynamic actors

**Files:**
- Modify: `ros2_ws/src/amr_description/worlds/warehouse.sdf`

Actors use the Gazebo Fuel `walk.dae` mesh. **Internet access required on first Docker run** to download from Fuel. Gazebo caches to `~/.gz/fuel/` automatically.

Actor positions are in aisles where Aurora will navigate — forcing real MPPI avoidance.

| Actor | Path | Direction |
|---|---|---|
| actor_0 | y=−2 cross-aisle, x: −10 ↔ 8 | East–West |
| actor_1 | x=−3 N-S aisle, y: −7 ↔ 7 | South–North |
| actor_2 | Loading dock loop | Oval |

- [ ] **Step 1: Append actors to warehouse.sdf before `</world>`**

```xml
  <!-- ════════════════════════════════════════════════════════════════
       ACTORS — walking humans from Gazebo Fuel (walk.dae)
       Downloaded automatically on first run → ~/.gz/fuel/
       ════════════════════════════════════════════════════════════════ -->

  <actor name="actor_0">
    <pose>-9 -2 1.0 0 0 0</pose>
    <skin>
      <filename>https://fuel.gazebosim.org/1.0/Mingfei/models/actor/tip/files/meshes/walk.dae</filename>
      <scale>1.0</scale>
    </skin>
    <animation name="walking">
      <filename>https://fuel.gazebosim.org/1.0/Mingfei/models/actor/tip/files/meshes/walk.dae</filename>
      <interpolate_x>true</interpolate_x>
    </animation>
    <script>
      <loop>true</loop>
      <delay_start>2.0</delay_start>
      <auto_start>true</auto_start>
      <trajectory id="0" type="walking" tension="0.6">
        <waypoint><time>0</time><pose>-9 -2 1.0 0 0 0</pose></waypoint>
        <waypoint><time>22</time><pose>8 -2 1.0 0 0 0</pose></waypoint>
        <waypoint><time>23</time><pose>8 -2 1.0 0 0 3.14159</pose></waypoint>
        <waypoint><time>45</time><pose>-9 -2 1.0 0 0 3.14159</pose></waypoint>
        <waypoint><time>46</time><pose>-9 -2 1.0 0 0 0</pose></waypoint>
      </trajectory>
    </script>
  </actor>

  <actor name="actor_1">
    <pose>-3 -7 1.0 0 0 1.5708</pose>
    <skin>
      <filename>https://fuel.gazebosim.org/1.0/Mingfei/models/actor/tip/files/meshes/walk.dae</filename>
      <scale>1.0</scale>
    </skin>
    <animation name="walking">
      <filename>https://fuel.gazebosim.org/1.0/Mingfei/models/actor/tip/files/meshes/walk.dae</filename>
      <interpolate_x>true</interpolate_x>
    </animation>
    <script>
      <loop>true</loop>
      <delay_start>8.0</delay_start>
      <auto_start>true</auto_start>
      <trajectory id="0" type="walking" tension="0.6">
        <waypoint><time>0</time><pose>-3 -7 1.0 0 0 1.5708</pose></waypoint>
        <waypoint><time>20</time><pose>-3 7 1.0 0 0 1.5708</pose></waypoint>
        <waypoint><time>21</time><pose>-3 7 1.0 0 0 4.7124</pose></waypoint>
        <waypoint><time>41</time><pose>-3 -7 1.0 0 0 4.7124</pose></waypoint>
        <waypoint><time>42</time><pose>-3 -7 1.0 0 0 1.5708</pose></waypoint>
      </trajectory>
    </script>
  </actor>

  <actor name="actor_2">
    <pose>4 -7 1.0 0 0 0</pose>
    <skin>
      <filename>https://fuel.gazebosim.org/1.0/Mingfei/models/actor/tip/files/meshes/walk.dae</filename>
      <scale>1.0</scale>
    </skin>
    <animation name="walking">
      <filename>https://fuel.gazebosim.org/1.0/Mingfei/models/actor/tip/files/meshes/walk.dae</filename>
      <interpolate_x>true</interpolate_x>
    </animation>
    <script>
      <loop>true</loop>
      <delay_start>15.0</delay_start>
      <auto_start>true</auto_start>
      <trajectory id="0" type="walking" tension="0.6">
        <waypoint><time>0</time><pose>4 -7 1.0 0 0 0</pose></waypoint>
        <waypoint><time>6</time><pose>7 -7 1.0 0 0 0</pose></waypoint>
        <waypoint><time>7</time><pose>7 -7 1.0 0 0 1.5708</pose></waypoint>
        <waypoint><time>12</time><pose>7 -5 1.0 0 0 1.5708</pose></waypoint>
        <waypoint><time>13</time><pose>7 -5 1.0 0 0 3.14159</pose></waypoint>
        <waypoint><time>19</time><pose>4 -5 1.0 0 0 3.14159</pose></waypoint>
        <waypoint><time>20</time><pose>4 -5 1.0 0 0 4.7124</pose></waypoint>
        <waypoint><time>25</time><pose>4 -7 1.0 0 0 4.7124</pose></waypoint>
        <waypoint><time>26</time><pose>4 -7 1.0 0 0 0</pose></waypoint>
      </trajectory>
    </script>
  </actor>

</world>
</sdf>
```

- [ ] **Step 2: Verify world + actors load**

```bash
gz sim --headless-rendering -r \
  ros2_ws/src/amr_description/worlds/warehouse.sdf \
  --iterations 500 -v 3 2>&1 | grep -E "^\\[Err\\]|^\\[Fatal\\]|actor" | head -20
```
Expected: lines mentioning actors loaded, no `[Err]` or `[Fatal]`.

> **Note:** If the Fuel URL fails (no internet), replace the three `https://fuel.gazebosim.org/...walk.dae` URLs with `model://actor/meshes/walk.dae` — this uses the local Gazebo model cache if the actor model was previously downloaded.

- [ ] **Step 3: Commit**

```bash
git add ros2_ws/src/amr_description/worlds/warehouse.sdf
git commit -m "feat(worlds): add 3 walking actors to warehouse world"
```

---

## Task 6: sim.launch.py

**Files:**
- Create: `ros2_ws/src/amr_bringup/launch/sim.launch.py`

This launch file starts Gazebo, spawns the robot, bridges topics, and launches the full navigation + exploration stack with `use_sim_time: true` injected into every node.

- [ ] **Step 1: Create `sim.launch.py`**

```python
import os
import yaml
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo, OpaqueFunction, TimerAction
from launch_ros.actions import Node


def _patch_yaml(path: str, extra: dict) -> str:
    """Read YAML, inject extra into every node's ros__parameters, return temp path."""
    with open(path) as f:
        data = yaml.safe_load(f)
    for _node, cfg in (data or {}).items():
        if isinstance(cfg, dict) and 'ros__parameters' in cfg:
            cfg['ros__parameters'].update(extra)
    tmp = tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False)
    yaml.dump(data, tmp, default_flow_style=False)
    tmp.close()
    return tmp.name


SIM = {'use_sim_time': True}


def launch_setup(context, *args, **kwargs):
    bringup   = get_package_share_directory('amr_bringup')
    desc      = get_package_share_directory('amr_description')
    nav_pkg   = get_package_share_directory('amr_nav')
    slam_pkg  = get_package_share_directory('amr_slam')
    fusion    = get_package_share_directory('amr_sensor_fusion')
    explore   = get_package_share_directory('amr_explore')
    bt_pkg    = get_package_share_directory('nav2_bt_navigator')

    world_file = os.path.join(desc, 'worlds', 'warehouse.sdf')

    # ── Robot description (use_sim:=true activates Gazebo plugins) ────────────
    import xacro
    robot_description_content = xacro.process_file(
        os.path.join(desc, 'urdf', 'amr.urdf.xacro'),
        mappings={'use_sim': 'true'}
    ).toxml()

    # ── Patch nav2 params: inject BT XML path + use_sim_time ─────────────────
    nav2_raw = os.path.join(nav_pkg, 'config', 'nav2_params.yaml')
    with open(nav2_raw) as f:
        nav2_params = yaml.safe_load(f)
    bt_xml = os.path.join(bt_pkg, 'behavior_trees',
                          'navigate_to_pose_w_replanning_and_recovery.xml')
    nav2_params['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml'] = bt_xml
    for cfg in nav2_params.values():
        if isinstance(cfg, dict) and 'ros__parameters' in cfg:
            cfg['ros__parameters']['use_sim_time'] = True
    nav2_tmp = tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False)
    yaml.dump(nav2_params, nav2_tmp, default_flow_style=False)
    nav2_tmp.close()
    nav2_patched = nav2_tmp.name

    collision_patched = _patch_yaml(
        os.path.join(nav_pkg, 'config', 'collision_monitor.yaml'), SIM)
    slam_patched  = _patch_yaml(os.path.join(slam_pkg, 'config', 'slam_toolbox.yaml'), SIM)
    ekf_patched   = _patch_yaml(os.path.join(fusion, 'config', 'ekf.yaml'), SIM)
    imu_patched   = _patch_yaml(os.path.join(fusion, 'config', 'imu_filter.yaml'), SIM)
    expl_patched  = _patch_yaml(os.path.join(explore, 'config', 'explore.yaml'), SIM)

    os.makedirs('/amr_ws/bags', exist_ok=True)
    os.makedirs('/amr_ws/maps', exist_ok=True)

    # ─────────────────────────────────────────────────────────────────────────
    #  Node definitions
    # ─────────────────────────────────────────────────────────────────────────

    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file],
        output='screen',
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_content,
                     'use_sim_time': True}],
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'amr', '-topic', '/robot_description',
                   '-x', '0.0', '-y', '-7.0', '-z', '0.1'],
        output='screen',
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/imu/data_raw@sensor_msgs/msg/Imu[gz.msgs.IMU',
        ],
        output='screen',
    )

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )
    mecanum_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['mecanum_drive_controller'],
    )

    odom_relay = Node(
        package='topic_tools',
        executable='relay',
        name='odom_relay',
        arguments=['/mecanum_drive_controller/odometry', '/odom/wheel'],
    )
    cmd_vel_relay = Node(
        package='amr_imu',
        executable='cmd_vel_safe_relay',
        name='cmd_vel_safe_relay',
        output='screen',
        parameters=[SIM],
    )

    imu_filter = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter_madgwick',
        output='screen',
        parameters=[imu_patched],
        remappings=[('imu/data_raw', '/imu/data_raw'),
                    ('imu/data',     '/imu/data')],
    )

    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_patched],
        remappings=[('odometry/filtered', '/odom')],
    )

    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_patched],
    )
    slam_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_slam',
        output='screen',
        parameters=[{'autostart': True, 'bond_timeout': 0.0,
                     'node_names': ['slam_toolbox'], **SIM}],
    )

    controller_server = Node(
        package='nav2_controller', executable='controller_server',
        name='controller_server', output='screen',
        parameters=[nav2_patched], remappings=[('cmd_vel', '/cmd_vel')],
    )
    planner_server = Node(
        package='nav2_planner', executable='planner_server',
        name='planner_server', output='screen',
        parameters=[nav2_patched],
    )
    bt_navigator = Node(
        package='nav2_bt_navigator', executable='bt_navigator',
        name='bt_navigator', output='screen',
        parameters=[nav2_patched],
    )
    behavior_server = Node(
        package='nav2_behaviors', executable='behavior_server',
        name='behavior_server', output='screen',
        parameters=[nav2_patched],
    )
    collision_monitor = Node(
        package='nav2_collision_monitor', executable='collision_monitor',
        name='collision_monitor', output='screen',
        parameters=[collision_patched],
    )
    nav2_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'autostart': True, 'bond_timeout': 0.0,
                     'node_names': ['controller_server', 'planner_server',
                                    'bt_navigator', 'behavior_server',
                                    'collision_monitor'], **SIM}],
    )

    explore_node = Node(
        package='explore_lite', executable='explore',
        name='explore', output='screen',
        parameters=[expl_patched],
    )
    home_manager = Node(
        package='amr_home_manager', executable='home_manager',
        name='amr_home_manager', output='screen',
        parameters=[{'map_save_path': '/amr_ws/maps/sim_explore_map', **SIM}],
    )

    rviz = Node(
        package='rviz2', executable='rviz2',
        arguments=['-d', os.path.join(bringup, 'rviz', 'sim.rviz')],
        parameters=[SIM],
        output='screen',
    )

    bag_record = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-a', '-o', '/amr_ws/bags/sim_demo'],
        output='screen',
    )

    return [
        gz_sim,
        robot_state_publisher,
        rviz,
        bag_record,
        TimerAction(period=2.0,  actions=[spawn]),
        TimerAction(period=3.0,  actions=[bridge]),
        TimerAction(period=4.0,  actions=[joint_state_broadcaster,
                                          mecanum_controller]),
        TimerAction(period=5.0,  actions=[odom_relay, cmd_vel_relay,
                                          imu_filter, ekf]),
        TimerAction(period=7.0,  actions=[slam, slam_lifecycle]),
        TimerAction(period=10.0, actions=[controller_server, planner_server,
                                          bt_navigator, behavior_server,
                                          collision_monitor, nav2_lifecycle]),
        TimerAction(period=20.0, actions=[explore_node, home_manager]),
        TimerAction(period=25.0, actions=[
            LogInfo(msg=(
                '\n'
                '══════════════════════════════════════════\n'
                '  AURORA SIM READY — exploring autonomously\n'
                '  Use RViz2 "2D Nav Goal" after exploration\n'
                '══════════════════════════════════════════\n'
            )),
        ]),
    ]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])
```

- [ ] **Step 2: Build and verify import**

```bash
cd ros2_ws && colcon build --packages-select amr_bringup 2>&1 | tail -5
```
Expected: `Summary: 1 package finished`

```bash
source ros2_ws/install/setup.bash
python3 -c "
from launch import LaunchDescription
import launch
import sys; sys.path.insert(0, 'ros2_ws/src/amr_bringup/launch')
import sim
print('import OK')
"
```
Expected: `import OK`

- [ ] **Step 3: Commit**

```bash
git add ros2_ws/src/amr_bringup/launch/sim.launch.py
git commit -m "feat(bringup): add sim.launch.py — full Gazebo simulation launch"
```

---

## Task 7: RViz2 dark-theme config

**Files:**
- Create: `ros2_ws/src/amr_bringup/rviz/sim.rviz`

- [ ] **Step 1: Create `sim.rviz`**

```yaml
Panels:
  - Class: rviz_common/Displays
    Name: Displays
    Property Tree Widget:
      Splitter Ratio: 0.26
  - Class: rviz_common/Views
    Name: Views

Visualization Manager:
  Class: ""
  Displays:

    - Alpha: 0.4
      Class: rviz_default_plugins/Grid
      Color: 80; 80; 100
      Enabled: true
      Name: Grid
      Plane: XY
      Plane Cell Count: 20
      Cell Size: 1
      Value: true

    - Class: rviz_default_plugins/RobotModel
      Enabled: true
      Name: RobotModel
      Description Topic:
        Value: /robot_description
      Value: true

    - Autocompute Intensity Bounds: true
      Class: rviz_default_plugins/LaserScan
      Color: 0; 255; 255
      Enabled: true
      Name: LiDAR Scan
      Point Step: 1
      Size (m): 0.03
      Topic:
        Value: /scan
        Depth: 5
        Durability Policy: Volatile
        QoS Profile: ""
        Reliability Policy: Best Effort
      Value: true

    - Alpha: 0.7
      Class: rviz_default_plugins/Map
      Color Scheme: map
      Draw Behind: false
      Enabled: true
      Name: SLAM Map
      Topic:
        Value: /map
        Depth: 1
        Durability Policy: Transient Local
        QoS Profile: ""
        Reliability Policy: Reliable
      Value: true
      Use Timestamp: false

    - Alpha: 0.4
      Class: rviz_default_plugins/Map
      Color Scheme: costmap
      Draw Behind: false
      Enabled: true
      Name: Global Costmap
      Topic:
        Value: /global_costmap/costmap
        Depth: 1
        Durability Policy: Transient Local
        QoS Profile: ""
        Reliability Policy: Reliable
      Value: true

    - Alpha: 0.55
      Class: rviz_default_plugins/Map
      Color Scheme: costmap
      Draw Behind: false
      Enabled: true
      Name: Local Costmap
      Topic:
        Value: /local_costmap/costmap
        Depth: 1
        Durability Policy: Volatile
        QoS Profile: ""
        Reliability Policy: Reliable
      Value: true

    - Class: rviz_default_plugins/Path
      Color: 57; 255; 20
      Enabled: true
      Name: Global Path
      Topic:
        Value: /plan
        Depth: 5
      Value: true
      Line Style: Lines
      Line Width: 0.03

    - Class: rviz_default_plugins/Path
      Color: 255; 255; 255
      Enabled: true
      Name: Local Path (MPPI)
      Topic:
        Value: /local_plan
        Depth: 5
      Value: true
      Line Style: Lines
      Line Width: 0.02

    - Class: rviz_default_plugins/MarkerArray
      Enabled: true
      Name: Frontiers
      Topic:
        Value: /explore/frontiers
        Depth: 5
      Value: true

    - Class: rviz_default_plugins/Odometry
      Angle Tolerance: 0.1
      Color: 255; 255; 255
      Color Style: Unique
      Enabled: true
      Keep: 200
      Name: EKF Odom (fused)
      Shape: Arrow
      Topic:
        Value: /odom
        Depth: 5
        Reliability Policy: Best Effort
      Value: true

    - Class: rviz_default_plugins/Odometry
      Angle Tolerance: 0.1
      Color: 180; 30; 30
      Color Style: Unique
      Enabled: true
      Keep: 200
      Name: Wheel Odom (raw)
      Shape: Arrow
      Topic:
        Value: /odom/wheel
        Depth: 5
        Reliability Policy: Best Effort
      Value: true

    - Class: rviz_default_plugins/TF
      Enabled: false
      Name: TF
      Value: false

  Enabled: true
  Global Options:
    Background Color: 28; 28; 46
    Fixed Frame: map
    Frame Rate: 30

  Tools:
    - Class: rviz_default_plugins/Interact
      Hide Inactive: false
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
    - Class: rviz_default_plugins/FocusCamera
    - Class: rviz_default_plugins/Measure
      Line color: 128; 128; 0
    - Class: nav2_rviz_plugins/GoalTool

  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 22
      Enable Stereo Rendering:
        Value: false
      Focal Point:
        X: 0
        Y: 0
        Z: 0
      Focal Shape Fixed Size: true
      Focal Shape Size: 0.05
      Invert Z Axis: false
      Name: Current View
      Near Clip Distance: 0.01
      Pitch: 1.05
      Target Frame: <Fixed Frame>
      Value: Orbit (rviz)
      Yaw: 3.14
    Saved: ~

Window Geometry:
  Displays:
    collapsed: false
  Height: 1080
  Width: 1920
  X: 0
  Y: 0
```

- [ ] **Step 2: Build**

```bash
cd ros2_ws && colcon build --packages-select amr_bringup 2>&1 | tail -3
```
Expected: `Summary: 1 package finished`

- [ ] **Step 3: Commit**

```bash
git add ros2_ws/src/amr_bringup/rviz/sim.rviz
git commit -m "feat(bringup): add dark-theme RViz2 config for simulation"
```

---

## Task 8: Docker update

**Files:**
- Modify: `docker/Dockerfile`
- Modify: `docker/run_sim.sh`

- [ ] **Step 1: Update `docker/Dockerfile`**

Replace the existing `RUN apt-get update` block with:

```dockerfile
RUN apt-get update && apt-get install -y \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-nav2-bringup \
    ros-jazzy-slam-toolbox \
    ros-jazzy-robot-localization \
    ros-jazzy-imu-tools \
    ros-jazzy-ros2-controllers \
    ros-jazzy-foxglove-bridge \
    ros-jazzy-topic-tools \
    python3-colcon-common-extensions \
    python3-pytest \
    ffmpeg \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /amr_ws/bags /amr_ws/maps /root/.gz/fuel
```

- [ ] **Step 2: Update `docker/run_sim.sh`**

Replace entire file content with:

```bash
#!/bin/bash
set -e

xhost +local:docker 2>/dev/null || true

docker run -it --rm \
  --env DISPLAY="${DISPLAY}" \
  --env ROS_DOMAIN_ID=42 \
  --env GZ_SIM_RESOURCE_PATH="/root/.gz/fuel/models" \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  --device /dev/dri \
  -v "${HOME}/AMR/ros2_ws":/amr_ws \
  -v "${HOME}/.gz/fuel":/root/.gz/fuel \
  --network host \
  amr_sim:latest \
  bash -c "
    source /opt/ros/jazzy/setup.bash && \
    source /amr_ws/install/setup.bash && \
    ros2 launch amr_bringup sim.launch.py
  "
```

Key fixes vs original:
- `$HOME/amr/ros2_ws` → `$HOME/AMR/ros2_ws` (actual path)
- Added `--device /dev/dri` for GPU-accelerated Gazebo rendering
- Mount `~/.gz/fuel` so Fuel actor mesh is cached across runs
- Added `GZ_SIM_RESOURCE_PATH` for model resolution
- Explicit `source` chain inside container

- [ ] **Step 3: Make run_sim.sh executable**

```bash
chmod +x docker/run_sim.sh
```

- [ ] **Step 4: Commit**

```bash
git add docker/Dockerfile docker/run_sim.sh
git commit -m "feat(docker): add sim deps, GPU device, fix AMR workspace volume path"
```

---

## Task 9: Demo and recording scripts

**Files:**
- Create: `scripts/demo_sim.sh`
- Create: `scripts/record_demo.sh`

- [ ] **Step 1: Create `scripts/demo_sim.sh`**

```bash
#!/bin/bash
# One-command Aurora simulation launcher.
# Builds the Docker image if not present, then launches the sim.
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

echo "======================================"
echo "  Aurora Simulation Launcher"
echo "======================================"

# Build image if needed
if ! docker image inspect amr_sim:latest >/dev/null 2>&1; then
    echo "[INFO] Building amr_sim:latest Docker image..."
    docker build -t amr_sim:latest "${PROJECT_ROOT}/docker"
fi

# First, build the ROS2 workspace inside the container
echo "[INFO] Building ROS2 workspace..."
docker run --rm \
  -v "${HOME}/AMR/ros2_ws":/amr_ws \
  amr_sim:latest \
  bash -c "
    source /opt/ros/jazzy/setup.bash && \
    cd /amr_ws && \
    colcon build --symlink-install 2>&1 | tail -5
  "

echo "[INFO] Launching Aurora simulation..."
echo "[INFO] Gazebo + RViz2 will open shortly."
echo "[INFO] Aurora will start exploring automatically after ~25s."
echo ""
"${PROJECT_ROOT}/docker/run_sim.sh"
```

- [ ] **Step 2: Create `scripts/record_demo.sh`**

```bash
#!/bin/bash
# Record the full X display to an MP4 demo video.
# Run this in a second terminal AFTER launching demo_sim.sh.
# Press Ctrl+C to stop recording.
set -e

OUTPUT="aurora_demo_$(date +%Y%m%d_%H%M%S).mp4"
RESOLUTION=$(xdpyinfo 2>/dev/null | grep dimensions | awk '{print $2}' || echo "1920x1080")

echo "======================================"
echo "  Aurora Demo Recorder"
echo "======================================"
echo "  Output : ${OUTPUT}"
echo "  Display: ${DISPLAY}"
echo "  Res    : ${RESOLUTION}"
echo "  Press Ctrl+C to stop."
echo "======================================"

ffmpeg \
  -f x11grab \
  -r 30 \
  -s "${RESOLUTION}" \
  -i "${DISPLAY}" \
  -c:v libx264 \
  -preset fast \
  -crf 18 \
  -pix_fmt yuv420p \
  "${OUTPUT}"

echo "Saved to: ${OUTPUT}"
```

- [ ] **Step 3: Make scripts executable**

```bash
chmod +x scripts/demo_sim.sh scripts/record_demo.sh
```

- [ ] **Step 4: Commit**

```bash
git add scripts/demo_sim.sh scripts/record_demo.sh
git commit -m "feat(scripts): add demo_sim.sh one-command launcher and record_demo.sh ffmpeg capture"
```

---

## Task 10: Integration smoke test

This task verifies the full sim stack launches and produces the expected topics.

- [ ] **Step 1: Build the Docker image**

```bash
docker build -t amr_sim:latest docker/ 2>&1 | tail -10
```
Expected: `Successfully tagged amr_sim:latest`

- [ ] **Step 2: Build the ROS2 workspace inside Docker**

```bash
docker run --rm \
  -v "${HOME}/AMR/ros2_ws":/amr_ws \
  amr_sim:latest \
  bash -c "
    source /opt/ros/jazzy/setup.bash && \
    cd /amr_ws && \
    colcon build --symlink-install 2>&1 | tail -8
  "
```
Expected: `Summary: N packages finished` with no errors.

- [ ] **Step 3: Headless Gazebo world smoke test (no display needed)**

```bash
docker run --rm \
  -v "${HOME}/AMR/ros2_ws":/amr_ws \
  amr_sim:latest \
  bash -c "
    gz sim --headless-rendering -r \
      /amr_ws/install/amr_description/share/amr_description/worlds/warehouse.sdf \
      --iterations 300 -v 3 2>&1 | grep -E 'Loaded|Error|Fatal' | head -10
  "
```
Expected: `Loaded [warehouse]`, no `Error` or `Fatal`.

- [ ] **Step 4: URDF renders cleanly in sim mode**

```bash
docker run --rm \
  -v "${HOME}/AMR/ros2_ws":/amr_ws \
  amr_sim:latest \
  bash -c "
    source /opt/ros/jazzy/setup.bash && \
    source /amr_ws/install/setup.bash && \
    ros2 run xacro xacro \
      /amr_ws/install/amr_description/share/amr_description/urdf/amr.urdf.xacro \
      use_sim:=true > /tmp/out.urdf && echo 'URDF OK' && wc -l /tmp/out.urdf
  "
```
Expected: `URDF OK`, line count > 100.

- [ ] **Step 5: Full launch topic check (requires display)**

On the host with an X server running:
```bash
./scripts/demo_sim.sh &
sleep 40
```
Then in a new terminal:
```bash
ros2 topic list | grep -E "/clock|/scan|/map|/odom|/cmd_vel"
```
Expected output includes:
```
/clock
/scan
/map
/odom
/odom/wheel
/cmd_vel
/cmd_vel_safe
```

- [ ] **Step 6: Verify Aurora starts exploring (frontier markers appear)**

```bash
ros2 topic hz /explore/frontiers --window 5
```
Expected: messages arriving at ~1 Hz within 30s of Nav2 coming up.

- [ ] **Step 7: Final commit**

```bash
git add .
git commit -m "feat(sim): full Gazebo Harmonic warehouse simulation complete"
```

---

## Troubleshooting Reference

| Symptom | Likely cause | Fix |
|---|---|---|
| `[Err] Actor mesh not found` | No internet to fetch Fuel model | Run inside container: `gz fuel download -u "https://fuel.gazebosim.org/1.0/Mingfei/models/actor"` |
| Gazebo blank window / slow | No GPU passthrough | Verify `--device /dev/dri` and that `libgl1-mesa-dri` is installed |
| `use_sim_time` warnings in EKF | yaml patch not applied | Check `_patch_yaml` return path is valid before launch |
| Robot spawns underground | z too low at spawn | Increase spawn z from `0.1` to `0.15` in sim.launch.py |
| Controller fails to start | gz_ros2_control not loaded | Check `use_sim:=true` was passed to xacro; look for plugin in urdf output |
| `/scan` not bridged | Bridge node crashed | Check `/imu/data_raw@sensor_msgs/msg/Imu[gz.msgs.IMU` — message type must match exactly |
