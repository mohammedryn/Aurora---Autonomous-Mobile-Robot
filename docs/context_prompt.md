# AMR Project — New Chat Context-Load Prompt

Paste everything below into a fresh chat to bring it fully up to speed before
giving it any task.

---

I'm working on **AMR (Aurora)** — a fully custom autonomous mobile robot:
mecanum-wheel chassis, ESP32-P4 firmware for drivetrain control, Raspberry Pi 5
running ROS2 Jazzy for SLAM/navigation/exploration. The project goes power-on →
autonomous frontier exploration → map save → return home → click-to-navigate,
fully autonomous, no human intervention.

**Phases 1–8 are complete and validated.** Phases 1–7 are validated on real
hardware. Phase 8 (Gazebo Harmonic simulation) is complete and working
end-to-end on WSL2. Phase 10 (Industry-Grade Deployment Roadmap) is fully
spec'd but **DEFERRED** — see below.

## Read these files, in this order, before doing anything else

1. **`/home/m0mspagetthi/AMR/docs/project_log.md`** — full project history, 35
   numbered sections, Phase 0 through the most recent session. **Read Section
   35 first** ("Phase 8: Gazebo Harmonic Simulation") — it's the most recent
   work. Then read Section 34 ("Recovery-Spin Validation") for the hardware
   exploration status. Skim §29–33 for the localization debugging arc.

2. **`/home/m0mspagetthi/AMR/docs/superpowers/specs/2026-05-17-amr-system-design.md`**
   — single source of truth for system design. Also see
   `docs/superpowers/specs/2026-06-17-gazebo-simulation-design.md` for the
   sim-specific spec.

3. **`/home/m0mspagetthi/AMR/docs/superpowers/plans/2026-05-17-amr-implementation-plan.md`**
   — Phases 0–9 plan (already executed). Also
   `docs/superpowers/plans/2026-06-17-gazebo-simulation.md` for the sim plan.

4. **`/home/m0mspagetthi/CLAUDE.md`** (project root `CLAUDE.md`) — execution
   rules: execute don't deliberate, no filler text, graphify-first code search,
   token conservation. Follow these for every response.

5. Check memory at
   `/home/m0mspagetthi/.claude/projects/-home-m0mspagetthi-AMR/memory/MEMORY.md`
   — accumulated cross-session knowledge.

## Critical facts to internalize

- **Dev workflow:** code is edited on WSL2 (this machine). Real hardware tests
  happen on the Raspberry Pi 5 — the user pulls, builds, and runs there
  themselves, pasting back output. **Never attempt to SSH to the Pi.**
- **Sim workflow:** `./scripts/demo_sim.sh` on WSL2 launches the full sim
  inside Docker. Gazebo and RViz2 appear as Windows windows via WSLg X11.
- **ROS2 Jazzy is NOT auto-sourced** on the Pi — every terminal needs
  `source /opt/ros/jazzy/setup.bash` before `source install/setup.bash`.
- **Current implementation** (trust `project_log.md` and source over the spec
  where they conflict):
  - Packages: `amr_hardware`, `amr_description`, `amr_bringup`,
    `amr_sensor_fusion`, `amr_imu`, `amr_slam`, `amr_nav`, `amr_explore`,
    `amr_home_manager`.
  - Global planner: `SmacPlanner2D`. Local controller: MPPI `DiffDrive`.
  - EKF `base_link_frame: base_footprint`. TF tree:
    `map → odom → base_footprint → base_link → ...`
  - Real hardware EKF yaw: IMU only (wheel yaw OFF). Sim EKF yaw: wheel odom
    only (IMU yaw OFF — Madgwick 6-DoF drifts in sim without magnetometer).
  - VL53L5CX ToF sensor **removed** — do not suggest re-adding for general
    obstacle detection.
  - `amr_home_manager` states: `IDLE`, `EXPLORING`, `RETURNING_HOME`, driven
    by `/amr/command` and `/explore/status` (§34, §35).
- **Sim-specific Nav2 overrides** are injected at runtime in `sim.launch.py`
  via `OpaqueFunction` — real `nav2_params.yaml` and `ekf.yaml` are never
  modified. See §35.7 for the full parameter table.
- **Local pytest runs need** `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` prefix.
- **Code search:** use `graphify query "<question>"` before grepping.

## Where things stand right now

**Phase 8 (Simulation) — COMPLETE (2026-06-20):**
- Single-command sim: `./scripts/demo_sim.sh` launches Gazebo Harmonic warehouse
  + full ROS 2 Nav stack inside Docker on WSL2.
- Aurora autonomously explores, builds a live SLAM map, and navigates with MPPI.
- Three CV portfolio videos recorded and embedded in README (GitHub CDN).
- All sim fixes are runtime-only — real hardware config unchanged.

**Phase 7 hardware status (from §34):**
- Exploration validated end-to-end on the Pi: costmap 391×315 cells, both
  recovery spins fired correctly, map saved.
- User wants to re-run in a more open space ("full open court") to assess map
  quality (wall straightness, loop closure, coverage) before deciding on
  Phase 10.

**THE IMMEDIATE NEXT TASK (hardware):** open-court mapping-quality test using
commands in `project_log.md §34.7`. Analyze the resulting `.pgm` map for
coverage vs. early stop, jagged/doubled walls (SLAM drift), and whether
`/explore/status` transitions match §34.4. Then decide: further SLAM/Nav2
tuning, or proceed to Phase 10.

**Phase 10** (AMCL, battery monitoring, person detection, diagnostics,
autostart hardening, CI, web dashboard) is spec'd in `docs/superpowers/specs/
2026-05-17-amr-system-design.md §15`. **Do not start Phase 10 until the
open-court mapping-quality question is resolved.**

Once you've read the files above, confirm the current state in 1–2 sentences
and wait for the actual data/task.
