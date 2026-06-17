# Aurora Gazebo Simulation — Design Spec
**Created: 2026-06-17**

---

## 1. Goal

Build a visually polished Gazebo Harmonic simulation of Aurora in a warehouse
environment that:

1. Runs with a single command (`./scripts/demo_sim.sh`)
2. Auto-executes the full autonomous mission: explore → map → return home → accept nav goal
3. Can be recorded as a 3–4 minute MP4 for a CV/portfolio video
4. Can be run live by an interviewer with `docker run`

The existing `amr_explore` and `amr_home_manager` packages run **unchanged** in
sim — only `use_sim_time:=true` is toggled.

---

## 2. World — `warehouse.sdf`

### Dimensions
24 × 18 m floor, 5 m ceiling.

### Layout
```
┌────────────────────────────────────────────────────┐
│  [SHELF]  [SHELF]  [SHELF]  [SHELF]  [SHELF]       │
│                                                    │
│  [SHELF]  [SHELF]  [SHELF]  [SHELF]  [SHELF]       │
│                                                    │
│  [SHELF]  [SHELF]  [SHELF]  [SHELF]  [SHELF]       │
│                                                    │
│  [LOADING DOCK]    ★ spawn   [PILLAR] [PILLAR]     │
└────────────────────────────────────────────────────┘
```

### Static Elements
| Element | Count | Size (m) | Material |
|---|---|---|---|
| Shelf units | 15 | 1.0 × 0.4 × 2.0 | Metal grey (`Gazebo/Grey`) |
| Inventory boxes on shelves | 30 | 0.3 × 0.3 × 0.3 | Cardboard brown |
| Floor pallets | 8 | 1.2 × 0.8 × 0.15 | Wood (`Gazebo/Wood`) |
| Structural pillars | 4 | 0.3 × 0.3 × 3.0 | Concrete grey |
| Forklift (static prop) | 1 | 1.8 × 0.9 × 2.2 | Yellow (`Gazebo/Yellow`) |
| Overhead light fixtures | 6 | 0.6 × 0.15 × 0.05 | White emissive |
| Safety stripe markers | 4 | 3.0 × 0.15 × 0.01 | Yellow/black |
| Perimeter walls | 4 | — | Brick (`Gazebo/Bricks`) |
| Floor | 1 | 24 × 18 | Concrete (`Gazebo/Concrete`) |

### Lighting
- 1 directional sun light: azimuth 45°, elevation 60°, cast shadows enabled
- 6 point lights above each fixture: intensity 2.0, range 8 m
- Ambient: `0.3 0.3 0.35 1` (cool warehouse feel, not flat)

### Dynamic Actors
3 walking actors using Gazebo SDF `<actor>` with the built-in `walk.bvh` animation:

| Actor | Loop path | Speed |
|---|---|---|
| Actor 1 | Aisle 1 end-to-end | 0.8 m/s |
| Actor 2 | Cross-aisle (perpendicular) | 0.7 m/s |
| Actor 3 | Loading dock area | 0.6 m/s |

Actors use the `meshes/walk.dae` animation from Gazebo's built-in actor mesh
library — no external assets required.

---

## 3. Robot Visual Upgrade

The current URDF is functional but visually flat (all grey boxes). The following
visual-only changes are made to `amr.urdf.xacro` and `wheels.urdf.xacro`.
Collision geometry and inertia are **not changed**.

| Component | Current | New |
|---|---|---|
| Chassis | Grey box | Dark charcoal (`0.15 0.15 0.18 1`) |
| Chassis accent | None | Thin bright-blue strip box (`0.2 0.5 1.0 1`) around the perimeter edge |
| Wheels | Grey cylinders | Orange-amber (`0.95 0.55 0.1 1`) |
| Sensor tower | None | Matte black cylinder stack below LiDAR |
| LiDAR | Cylinder | Black cylinder, slightly larger radius |
| Corner LEDs | None | 4× small green spheres (`0.1 0.9 0.3 1`, emissive) on chassis corners |
| LiDAR sensor | `<visualize>false` | `<visualize>true` → cyan rotating sweep in Gazebo |

---

## 4. RViz2 Configuration — `sim.rviz`

Dark professional theme, all Aurora data layers visible simultaneously.

| Layer | Color | Notes |
|---|---|---|
| Background | `#1c1c2e` | Near-black dark blue |
| RobotModel | — | Full URDF render |
| LaserScan | Cyan `#00ffff`, size 3 | LiDAR sweep |
| Map (SLAM) | Default occupancy colors | Builds live |
| Global Costmap | Dark navy + white obstacles | `nav2_costmap_2d` |
| Local Costmap | Orange overlay | Shows planning horizon |
| Global Path | Lime green `#39ff14` | Nav2 global plan |
| Local Path | White | MPPI trajectory |
| Frontier markers | Orange `#ff6b00` arrows | From `amr_explore` |
| EKF path | White trail | `/odometry/filtered` |
| Raw odom path | Dim red `#cc2222` trail | Shows EKF correction |
| TF | Enabled, key frames only | base_link, odom, map |

Layout: single large 3D viewport. RViz2 panels (Displays, Views) docked left.

---

## 5. New Files

```
ros2_ws/src/amr_description/
  worlds/
    warehouse.sdf                  ← new: warehouse world
  urdf/
    amr.urdf.xacro                 ← modify: visual upgrade (chassis, LEDs)
    wheels.urdf.xacro              ← modify: orange wheels, sensor tower

ros2_ws/src/amr_bringup/
  launch/
    sim.launch.py                  ← new: full sim launch
  rviz/
    sim.rviz                       ← new: dark-theme RViz2 config
  config/
    nav2_params_sim.yaml           ← new: sim-specific Nav2 params (use_sim_time)
    slam_params_sim.yaml           ← new: sim-specific SLAM params (use_sim_time)

docker/
  Dockerfile                       ← modify: add sim deps
  run_sim.sh                       ← modify: point to sim.launch.py

scripts/
  demo_sim.sh                      ← new: one-command launcher
  record_demo.sh                   ← new: ffmpeg screen capture to MP4
```

---

## 6. `sim.launch.py` — Node Startup Order

Nodes launched with appropriate delays to avoid race conditions:

| t=0s | Gazebo Harmonic + warehouse world |
|---|---|
| t=2s | robot_state_publisher (`use_sim_time:=true`) |
| t=3s | gz_ros2_bridge (clock, scan, imu/data_raw, tf) |
| t=4s | joint_state_broadcaster + mecanum_drive_controller |
| t=5s | robot_localization EKF |
| t=7s | slam_toolbox |
| t=10s | nav2_bringup |
| t=15s | amr_explore (exploration auto-starts) |
| t=16s | amr_home_manager |
| t=0s | RViz2 with sim.rviz |
| t=0s | ros2 bag record -a (background) |

---

## 7. Docker

### `docker/Dockerfile` additions
```dockerfile
RUN apt-get update && apt-get install -y \
    ros-jazzy-gz-sim \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-nav2-bringup \
    ros-jazzy-slam-toolbox \
    ros-jazzy-robot-localization \
    ros-jazzy-imu-tools \
    ros-jazzy-ros2-controllers \
    ffmpeg \
    && rm -rf /var/lib/apt/lists/*
```

### `docker/run_sim.sh`
```bash
docker run -it --rm \
  --env DISPLAY=$DISPLAY \
  --env ROS_DOMAIN_ID=42 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  --device /dev/dri \
  -v $HOME/AMR/ros2_ws:/amr_ws \
  --network host \
  amr_sim:latest \
  ros2 launch amr_bringup sim.launch.py
```

`--device /dev/dri` enables GPU acceleration for smooth Gazebo rendering.

---

## 8. Demo Scripts

### `scripts/demo_sim.sh`
1. Builds the Docker image if not present (`docker build -t amr_sim:latest ./docker`)
2. Calls `xhost +local:docker` for X11 access
3. Runs `docker/run_sim.sh`
4. Prints: "Aurora simulation running. Connect RViz2 is launching automatically."

### `scripts/record_demo.sh`
Wraps ffmpeg to capture the full X display:
```bash
ffmpeg -f x11grab -r 30 -s $(xdpyinfo | grep dimensions | awk '{print $2}') \
  -i $DISPLAY -c:v libx264 -preset fast -crf 18 \
  aurora_demo_$(date +%Y%m%d_%H%M%S).mp4
```
Produces a 30fps H.264 MP4 suitable for LinkedIn/YouTube.

---

## 9. Demo Video Sequence (Target: ~3.5 min)

| Time | Scene |
|---|---|
| 0:00–0:15 | Gazebo opens: warehouse with overhead lighting, Aurora sitting at loading dock |
| 0:15–0:30 | LiDAR spins up, cyan sweep visible in Gazebo + RViz2, map starts populating |
| 0:30–2:00 | Autonomous exploration: Aurora navigates aisles, frontier arrows guide it, map fills |
| 1:15 | Actor 1 crosses Aurora's path — MPPI local planner steers around smoothly |
| 1:45 | Aurora strafes sideways through tight shelf gap (mecanum money shot) |
| 2:00–2:30 | Exploration complete — `amr_home_manager` kicks in, Aurora drives back to ★ |
| 2:30–3:30 | User clicks 2D Nav Goal in RViz2 — global path drawn, Aurora navigates |
| 3:30 | Aurora arrives at goal, stops. Full map visible. |

---

## 10. Sim-to-Real Boundary

The only change between sim and real is the single `use_sim` xacro arg in the
URDF. Every ROS2 node (`amr_explore`, `amr_home_manager`, Nav2, SLAM, EKF) runs
**identical code** in both modes. This is the strongest technical claim the
simulation makes — it is not a toy demo, it is the exact production stack.
