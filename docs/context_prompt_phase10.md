# AMR Project — New Chat Context-Load Prompt

Paste everything below into a fresh chat to bring it fully up to speed before
giving it any task.

---

I'm working on **AMR (Aurora)** — a fully custom autonomous mobile robot:
mecanum-wheel chassis, ESP32-P4 firmware for drivetrain control, Raspberry Pi 5
running ROS2 Jazzy for SLAM/navigation/exploration, controlled via Foxglove
Studio. The project goes power-on → autonomous frontier exploration → map save
→ return home → click-to-navigate, fully autonomous, no human intervention.

**Phases 1-7 are complete and validated on real hardware.** The robot maps,
explores, avoids obstacles, returns home, and accepts click-to-navigate goals.
We are now starting **Phase 10: Industry-Grade Deployment Roadmap** — adding
AMCL localization mode, battery monitoring, safety perception, diagnostics,
and operational tooling on top of the working core.

## Read these files, in this order, before doing anything else

1. **`/home/m0mspagetthi/AMR/docs/project_log.md`** — the full project history.
   This is the most important file. It has ~33 numbered sections covering
   every bug found, root cause, fix, and commit, from Phase 0 through the most
   recent session (explore-lite stall watchdog, IMU yaw drift fix, LiDAR/TF
   fixes, MPPI tuning). **Read the whole thing** — it's long but it is the
   definitive record of what's been tried, what worked, and what's a dead end.
   Pay special attention to the last 3-4 sections for the most recent state.

2. **`/home/m0mspagetthi/AMR/docs/superpowers/specs/2026-05-17-amr-system-design.md`**
   — the single source of truth for system design: hardware inventory, wiring,
   firmware architecture, ROS2 node graph, sensor fusion, SLAM, Nav2 config,
   exploration state machine, and **§15 "Phase 10 — Industry-Grade Deployment
   Roadmap"** which is the spec for the work we're starting now (AMCL
   localization, battery monitoring, person detection, floor-hazard ToF,
   diagnostics aggregator, autostart hardening, CI, web dashboard, validation).

3. **`/home/m0mspagetthi/AMR/docs/superpowers/plans/2026-05-17-amr-implementation-plan.md`**
   — the task-by-task implementation plan for Phases 0-9 (already executed).
   Skim this to understand the granularity/format expected — Phase 10's plan
   (not yet written) should follow the same style: numbered tasks, file
   lists, step-by-step instructions, checkboxes.

4. **`/home/m0mspagetthi/CLAUDE.md`** (project root `CLAUDE.md`) — execution
   rules for this project: execute don't deliberate, no filler text, no
   trailing summaries, plan-execution discipline, graphify-first code search,
   token conservation rules. Follow these for every response.

5. Check memory at
   `/home/m0mspagetthi/.claude/projects/-home-m0mspagetthi-AMR/memory/MEMORY.md`
   — accumulated cross-session knowledge (confirmed GPIO pins, dead-end fixes
   not to retry, dev workflow, hardware quirks). Read any memory file whose
   description looks relevant before asking me something it might already
   answer.

## Critical facts to internalize

- **Dev workflow:** I write/edit code on WSL2 (this machine). I pull, build,
  and run on the Raspberry Pi 5 myself and paste back terminal output/errors.
  **Never attempt to SSH to the Pi.**
- **ROS2 Jazzy is NOT auto-sourced** on the Pi — every terminal needs
  `source /opt/ros/jazzy/setup.bash` before `source install/setup.bash` and
  before any `ros2`/`colcon` command.
- **Current actual implementation** (the spec doc has some intentionally-kept
  historical detail that's been superseded — trust `project_log.md` and the
  actual source under `ros2_ws/src/` over the spec where they conflict):
  - Packages: `amr_hardware`, `amr_description`, `amr_bringup`, `amr_sensor_fusion`,
    `amr_imu`, `amr_slam`, `amr_nav`, `amr_explore`, `amr_home_manager`.
  - Global planner: `SmacPlanner2D` (not Lattice). Local controller: MPPI with
    `motion_model: DiffDrive` (not Omni) — both changed from the original spec
    during exploration debugging (project_log.md §32-33).
  - EKF `base_link_frame: base_footprint` (not `base_link`) — TF tree is
    `map -> odom -> base_footprint -> base_link -> ...`.
  - The VL53L5CX ToF sensor was **removed** from the project (project_log.md
    §25.6) — do not suggest re-adding it for general obstacle detection.
    Phase 10 reintroduces it for ONE narrow purpose only (downward-angled
    floor-hazard/cliff detection) — see spec §15.4.2.
  - `amr_home_manager` currently has states `IDLE`, `EXPLORING`,
    `RETURNING_HOME`, driven by `/amr/command` (String topic: "explore",
    "go_home", "stop"), plus a stall watchdog (project_log.md §33.5).
- **Local pytest runs need** `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` prefix (broken
  `anyio` plugin in this WSL2 environment) — not a project bug, just an env
  quirk. Use it for every local test run and in any CI config.
- **Code search:** use `graphify query "<question>"` from the project root
  before grepping — it's pre-indexed and far cheaper than scanning files.

## Where things stand right now

Phase 10 has been **spec'd** (§15 of the system design doc) but **no
implementation plan or code exists yet**. The next step is writing
`docs/superpowers/plans/2026-06-XX-phase10-deployment-plan.md` (via the
writing-plans skill) breaking §15.1-§15.9 into the same task/checkbox format
as the existing implementation plan, prioritized per the table in spec §15.0:

1. AMCL localization mode (`amr_localization`)
2. Battery monitoring + auto-return (`amr_battery`)
3. Diagnostics aggregator (`amr_diagnostics`)
4. Autostart hardening (systemd)
5. Localization health watchdog (extends `amr_home_manager`)
6. Person detection (`amr_perception`)
7. Floor hazard ToF (`amr_perception`)
8. Web dashboard (`dashboard/`, Next.js)
9. Validation — benchmarks, demo video, technical writeup

Once you've read the files above, confirm you understand the current state in
1-2 sentences and wait for the actual task — don't start implementing yet.
