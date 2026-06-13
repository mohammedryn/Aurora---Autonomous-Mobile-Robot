# AMR Project — New Chat Context-Load Prompt

Paste everything below into a fresh chat to bring it fully up to speed before
giving it any task.

---

I'm working on **AMR (Aurora)** — a fully custom autonomous mobile robot:
mecanum-wheel chassis, ESP32-P4 firmware for drivetrain control, Raspberry Pi 5
running ROS2 Jazzy for SLAM/navigation/exploration, controlled via Foxglove
Studio. The project goes power-on → autonomous frontier exploration → map save
→ return home → click-to-navigate, fully autonomous, no human intervention.

**Phases 1-9 are complete and validated on real hardware.** The robot maps,
explores, avoids obstacles, returns home, and accepts click-to-navigate goals.
**Phase 10 (Industry-Grade Deployment Roadmap) is fully spec'd but DEFERRED** —
see "Where things stand right now" below for what comes first.

## Read these files, in this order, before doing anything else

1. **`/home/m0mspagetthi/AMR/docs/project_log.md`** — the full project history,
   ~34 numbered sections, Phase 0 through the most recent session. **Read
   Section 34 first** ("Recovery-Spin Validation: Costmap QoS Bug, Autonomous-
   Start State Tracking, and a Successful Room-Scale Explore Run") — it's the
   most recent work and ends with the exact next-step plan (§34.7). Then skim
   earlier sections as needed for background; pay attention to §29-33
   (exploration/localization debugging arc that §34 concludes).

2. **`/home/m0mspagetthi/AMR/docs/superpowers/specs/2026-05-17-amr-system-design.md`**
   — single source of truth for system design: hardware inventory, wiring,
   firmware architecture, ROS2 node graph, sensor fusion, SLAM, Nav2 config,
   exploration state machine, and **§15 "Phase 10 — Industry-Grade Deployment
   Roadmap"** (AMCL localization, battery monitoring, person detection,
   floor-hazard ToF, diagnostics aggregator, autostart hardening, CI, web
   dashboard, validation) — relevant once Phase 10 actually starts, not yet.

3. **`/home/m0mspagetthi/AMR/docs/superpowers/plans/2026-05-17-amr-implementation-plan.md`**
   — task-by-task implementation plan for Phases 0-9 (already executed). Skim
   for the granularity/format expected if/when a Phase 10 plan is written.

4. **`/home/m0mspagetthi/CLAUDE.md`** (project root `CLAUDE.md`) — execution
   rules for this project: execute don't deliberate, no filler text, no
   trailing summaries, plan-execution discipline, graphify-first code search,
   token conservation rules. Follow these for every response.

5. Check memory at
   `/home/m0mspagetthi/.claude/projects/-home-m0mspagetthi-AMR/memory/MEMORY.md`
   — accumulated cross-session knowledge. Read
   **`project_explore_premature_stop.md`** in full (the just-finished bug, all
   root causes and validation results) before asking me anything about it.

## Critical facts to internalize

- **Dev workflow:** I write/edit code on WSL2 (this machine). I pull, build,
  and run on the Raspberry Pi 5 myself and paste back terminal output/errors,
  log files, and map data. **Never attempt to SSH to the Pi.**
- **ROS2 Jazzy is NOT auto-sourced** on the Pi — every terminal needs
  `source /opt/ros/jazzy/setup.bash` before `source install/setup.bash` and
  before any `ros2`/`colcon` command.
- **Current actual implementation** (the spec doc has some intentionally-kept
  historical detail that's been superseded — trust `project_log.md` and the
  actual source under `ros2_ws/src/` over the spec where they conflict):
  - Packages: `amr_hardware`, `amr_description`, `amr_bringup`, `amr_sensor_fusion`,
    `amr_imu`, `amr_slam`, `amr_nav`, `amr_explore`, `amr_home_manager`.
  - Global planner: `SmacPlanner2D` (not Lattice). Local controller: MPPI with
    `motion_model: DiffDrive` (not Omni).
  - EKF `base_link_frame: base_footprint` (not `base_link`) — TF tree is
    `map -> odom -> base_footprint -> base_link -> ...`.
  - The VL53L5CX ToF sensor was **removed** from the project — do not suggest
    re-adding it for general obstacle detection. Phase 10 reintroduces it for
    ONE narrow purpose only (downward-angled floor-hazard/cliff detection).
  - `amr_home_manager` states: `IDLE`, `EXPLORING`, `RETURNING_HOME`, driven by
    `/amr/command` (String: "explore", "go_home", "stop") and `/explore/status`
    (explore_lite's own autonomous start/complete announcements — see §34).
- **Local pytest runs need** `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` prefix (broken
  `anyio` plugin in this WSL2 environment).
- **Code search:** use `graphify query "<question>"` from the project root
  before grepping — it's pre-indexed and far cheaper than scanning files.

## Where things stand right now

The "panic rotation"/premature-stop exploration bug (project_log.md §29-34,
the user's top priority for several sessions) is **FIXED and VALIDATED
end-to-end on the Pi (2026-06-11)**: the robot drove through a corridor,
costmap grew to 391×315 cells, both recovery spins fired correctly (one
succeeded, one gracefully aborted on collision), and the map saved.

However, the user isn't satisfied a corridor run proves "clean mapping" —
they want to re-run the same launch in a more open space ("full open court")
and judge the resulting `.pgm` map for wall straightness, loop closure, and
coverage before deciding what's next.

**THE IMMEDIATE NEXT TASK**: I (the user) will run the open-court test using
the commands in `project_log.md` §34.7 (launch+log in one terminal, a small
`ros2 bag record` in another, then tar up `~/AMR/maps/explore_map.{pgm,yaml}` +
the console log + the bag) and hand the resulting data back to you. Analyze:
coverage of open space vs. early stop, jagged/doubled/offset walls (SLAM
drift / loop-closure issues), repeated "Collision Ahead"/stuck behaviour, and
whether `/explore/status` transitions match the pattern from §34.4. Convert
the `.pgm` to PNG for visual inspection if needed.

**Based on that analysis**, decide: further SLAM/Nav2 tuning, or proceed to
Phase 10 (spec §15, plan not yet written). **Do not start Phase 10 work until
the open-court mapping-quality question is resolved.**

Once you've read the files above, confirm you understand the current state in
1-2 sentences and wait for the actual data/task — don't start implementing yet.
