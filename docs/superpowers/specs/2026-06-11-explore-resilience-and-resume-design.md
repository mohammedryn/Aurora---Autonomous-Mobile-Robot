# Exploration Resilience: Persistent Retry, Stuck-Recovery, and Path-Based Resume

Status: approved design, not yet implemented.
Owner component: `amr_home_manager` (`home_manager_node.py`).

## 1. Goal & Scope

The recovery-spin mechanism added in session 34 (commit `56ec090`) was a
testing-only stopgap: on `EXPLORATION_COMPLETE` it does up to 2x (360° spin +
resume) before giving up, saving the map, and going `IDLE`. In small/congested
spaces the 360° spin itself reliably aborts ("Collision Ahead - Exiting Spin"),
and 2 retries is far too few for a robot that should keep mapping until the
user says otherwise.

This spec replaces that mechanism with three pieces of new behavior, all
scoped to `amr_home_manager`:

1. **Persistent retry on "no frontiers found"** — never auto-stop; keep
   nudging `explore_lite` to search again, indefinitely. Only `stop`/`go_home`
   end an exploration session.
2. **Stuck detection** — if the robot is physically unable to make any
   progress (no motion despite repeated resume-nudges) for an extended
   period, stop nudging, prompt the operator via the log, and wait for a
   `stop`/`go_home` command instead of looping forever.
3. **Path recording + clean retrace** — continuously record the robot's
   travelled path during exploration. On `stop`, save the map *and* the
   path. `go_home` retraces that path in reverse (instead of relying on a
   fresh Nav2 plan through a possibly-cluttered explored costmap, which was
   causing "going here and there"). A new `resume` command retraces the path
   forward from home back to the exact point exploration was stopped, then
   automatically resumes exploration from there.

Out of scope: changes to `explore_lite`, Nav2 planner/controller tuning,
costmap inflation tuning (tracked separately), and any new Nav2 nodes
(no `waypoint_follower` — see §4.6).

## 2. Current Behavior (for reference)

- `State` enum: `IDLE`, `EXPLORING`, `RETURNING_HOME`.
- `_check_stall` (5s timer): if no `/odom` motion for 15s while `EXPLORING`,
  publish `/explore/resume=True` once, latch (`_stall_nudge_sent`) until
  motion resumes.
- `_on_explore_status`: on `EXPLORATION_STARTED`/`EXPLORATION_IN_PROGRESS`
  while `IDLE`, transition to `EXPLORING`. On `EXPLORATION_COMPLETE` while
  `EXPLORING`: up to `_max_recovery_spins=2` rounds of (Nav2 `Spin` 360° +
  resume); after that, `_on_exploration_done()` (resume=False, save map,
  → `IDLE`).
- `_on_command`: `explore` → resets spin counter, resume=True, → `EXPLORING`.
  `stop` → `_on_exploration_done()`. `go_home` → single `NavigateToPose` to
  `_home_pose`, → `RETURNING_HOME` → `IDLE`. **Neither `stop` nor `go_home`
  currently publishes `/explore/resume=False`**, so `explore_lite` keeps
  issuing its own frontier `NavigateToPose` goals that compete with the
  return-home goal.
- `_home_pose`: recorded once, from the first `/odom` message, frame_id
  `'map'` (valid because `map`↔`odom` are coincident at t=0).

## 3. New State Machine

```
States: IDLE, EXPLORING, STUCK, RETURNING_HOME, RESUMING

IDLE --explore cmd / EXPLORATION_STARTED--> EXPLORING
  (reset _recorded_path = [home_pose], _breakpoint_pose = None,
   _stall_nudge_count = 0)

EXPLORING --EXPLORATION_COMPLETE--> EXPLORING
  (always: log + /explore/resume=True, never stops)

EXPLORING --stall, no motion despite N nudges--> STUCK
  (log operator prompt, stop nudging)

EXPLORING or STUCK --stop cmd--> IDLE
  (save progress: map + path + breakpoint pose, resume=False)

EXPLORING or STUCK --go_home cmd--> RETURNING_HOME --> IDLE
  (save progress first, then retrace _recorded_path reversed)

IDLE --resume cmd (requires saved breakpoint)--> RESUMING --> EXPLORING
  (retrace _recorded_path forward to breakpoint, then resume=True)
```

## 4. Detailed Design

### 4.1 Remove recovery-spin; persistent retry on `EXPLORATION_COMPLETE`

Delete: `Spin`/`Duration` imports, `_spin_client`, `_send_recovery_spin`,
`_on_spin_goal_accepted`, `_on_spin_done`, `_recovery_spins_attempted`,
`_max_recovery_spins`, `_spin_time_allowance_s`.

`_on_explore_status`, `EXPLORATION_COMPLETE` branch (while `EXPLORING`):
log `'explore reported "no frontiers found" -- resuming, will keep trying'`
and call `_resume_explore()`. No counter, no `_on_exploration_done()`. While
`STUCK`/`IDLE`/`RESUMING`/`RETURNING_HOME`, ignore (no-op).

Note this doesn't mean the robot can spin in this loop forever unattended:
if `EXPLORATION_COMPLETE` keeps firing because the robot genuinely has
nowhere left to go, it's also not moving — and the stall watchdog (§4.3)
escalates to `STUCK` independently (based on `/odom`, not on explore status)
after `_max_stall_retries`. The two mechanisms are deliberately decoupled but
converge: "no frontiers + no motion for ~3.75 min" → operator prompt either
way.

### 4.2 `stop`/`go_home` stop `explore_lite` first

Both commands begin by publishing `/explore/resume=False` (when current
state is `EXPLORING` or `STUCK`) before doing anything else. This prevents
`explore_lite` from issuing competing frontier goals while the robot is
retracing/idle.

### 4.3 Stall-watchdog escalation → `STUCK`

New parameters (instance attributes, not ROS params — consistent with
existing `_stall_timeout_s`):
- `_max_stall_retries = 15` (≈3.75 min of being fully stuck before giving up)
- `_stuck_prompt_delay_s = 3.0`

New state: `_last_nudge_time` (init = node start time, alongside
`_last_motion_time`), `_stall_nudge_count = 0`.

`_check_stall` (every 5s, only while `EXPLORING`):
- `stalled_for = now - _last_motion_time`
- if `stalled_for > _stall_timeout_s` and `(now - _last_nudge_time) >= _stall_timeout_s`:
  - if `_stall_nudge_count < _max_stall_retries`:
    publish `/explore/resume=True`, `_stall_nudge_count += 1`,
    `_last_nudge_time = now`, log
    `f'No motion for {stalled_for:.0f}s -- nudging resume ({_stall_nudge_count}/{_max_stall_retries})'`.
  - else:
    start a one-shot `_stuck_prompt_delay_s` timer → `_enter_stuck()` (guard
    with `_stuck_pending` flag so it isn't scheduled twice).

`_enter_stuck()`: if motion resumed during the delay, abort (reset counters,
stay `EXPLORING`). Otherwise: `_state = STUCK`, log (WARN, visible in the
launch terminal):
```
Robot stuck -- cannot make further progress after 15 retries.
Send /amr/command 'stop' (save map+path, stay here) or 'go_home'
(save + retrace path home).
```

`_on_odom`: whenever real motion is detected, reset `_last_motion_time`,
`_stall_nudge_sent`-equivalent (`_stall_nudge_count = 0`,
`_last_nudge_time = now`), and `_stuck_pending = False`.

### 4.4 Path recording

New attributes: `_recorded_path: list[tuple[float,float,float]]` (x, y, yaw),
`_path_sample_distance_m = 0.5`.

In `_on_odom`, while `_state in (EXPLORING, STUCK)`: if Euclidean distance
from `_recorded_path[-1]` (or no entries yet) ≥ `_path_sample_distance_m`,
append `(x, y, yaw)` extracted from `msg.pose.pose`.

`_recorded_path` is reset to `[home_pose_xyz]` whenever `_state` transitions
`IDLE → EXPLORING` (both the `explore` command path and the
`EXPLORATION_STARTED` auto-transition path in `_on_explore_status`).

### 4.5 `stop` command — `_save_progress()`

New helper, called from both `stop` and `go_home` (§4.6) when leaving
`EXPLORING`/`STUCK`:

1. Publish `/explore/resume=False`.
2. `_breakpoint_pose` = last entry of `_recorded_path` (current position).
3. `_trigger_map_save()` (existing — unchanged).
4. Save `_recorded_path` as JSON to `<map_save_path>_path.json`
   (e.g. `~/AMR/maps/explore_map_path.json`), format:
   `{"path": [[x, y, yaw], ...]}`.

`stop` command handler: if `_state in (EXPLORING, STUCK)`, call
`_save_progress()`, then `_state = IDLE`. Robot does not move. If already
`IDLE`, no-op (log info).

### 4.6 `go_home` command — retrace reversed

If `_state in (EXPLORING, STUCK)`: call `_save_progress()` first.

Then:
- If `len(_recorded_path) <= 1`: already at/near home — fall back to the
  existing single `_navigate_to_home()` (direct `NavigateToPose` to
  `_home_pose`).
- Else: `_state = RETURNING_HOME`; call
  `_navigate_path(reversed(_recorded_path), on_done=lambda: state=IDLE)`.

**`_navigate_path(waypoints, on_done)`** — sequential `NavigateToPose` helper
(no new Nav2 nodes; reuses the existing `_nav_client` →
`bt_navigator`/`navigate_to_pose`, already wired up):
- Store `waypoints` as a queue. For each waypoint *i*, the goal orientation
  (yaw) is computed as the heading toward waypoint *i+1* (`atan2(dy, dx)`);
  the final waypoint uses its own recorded yaw (home's recorded yaw for
  `go_home`, breakpoint's recorded yaw for `resume`).
- Send `NavigateToPose` for the first waypoint. On result (success or
  failure — don't get stuck if one leg fails, just proceed, matching the
  "keep trying, don't permanently stop" spirit), send the next waypoint. When
  the queue is empty, call `on_done()`.

### 4.7 New `resume` command — retrace forward

Requires `_breakpoint_pose is not None` and `_recorded_path` non-empty
(i.e., a prior `stop`/`go_home` ran this session). If not set: log warning
`'No saved breakpoint -- nothing to resume'`, no-op.

`_state = RESUMING`; call `_navigate_path(_recorded_path, on_done=_on_resume_arrived)`
(forward order, home → breakpoint).

`_on_resume_arrived()`: publish `/explore/resume=True`, `_state = EXPLORING`.
Path recording (§4.4) continues appending from here — `_recorded_path` now
represents home → breakpoint → (new exploration), so a future `stop`/`go_home`
retrace naturally passes back through the old breakpoint to home.

### 4.8 `/amr/command` summary (new full set)

| Command | Precondition | Effect |
|---|---|---|
| `explore` | home pose recorded | reset path/breakpoint, resume=True, → `EXPLORING` |
| `stop` | `EXPLORING`/`STUCK` (else no-op) | save map+path+breakpoint, resume=False, → `IDLE` |
| `go_home` | home pose recorded | save progress (if exploring/stuck), retrace path reversed → home, → `IDLE` |
| `resume` | saved breakpoint exists (else no-op + warn) | retrace path forward → breakpoint, resume=True, → `EXPLORING` |

`explore_map.launch.py`'s operator-instructions `LogInfo` (§ "Operator
instructions after stack is up") gets updated to document `resume` and the
new stuck-prompt message.

## 5. Persistence Format

`<map_save_path>_path.json` (sibling to `.pgm`/`.yaml`):
```json
{"path": [[0.0, 0.0, 0.0], [0.48, 0.02, 0.04], ...]}
```
List of `[x, y, yaw]` in the `map` frame, first entry = home pose. Written on
every `_save_progress()` (overwrites previous file — single most-recent
breakpoint, no history stack, per YAGNI).

In-memory `_recorded_path`/`_breakpoint_pose` are the source of truth for
`go_home`/`resume` within a running session; the JSON file is the durable
record for `stop` (loading it back after a node restart is explicitly out of
scope for this iteration).

## 6. Testing

Update `test_home_manager.py` (currently 24/24 passing):

- **Remove**: the 4 recovery-spin tests from session 34
  (`test_explore_status_started_transitions_idle_to_exploring` etc. that
  reference `_recovery_spins_attempted`/spin client — keep the
  state-transition tests, drop spin-specific assertions/mocks).
- **Update**: `EXPLORATION_COMPLETE` test(s) — assert `_resume_explore()` is
  called and `_state` stays `EXPLORING`, regardless of how many times it
  fires (no `_on_exploration_done()` call).
- **New**:
  - stall escalation: simulate `_max_stall_retries` nudges with no motion →
    assert transition to `STUCK` and the prompt log.
  - `_on_odom` resets stall/stuck counters on motion.
  - path recording: simulate odom messages ≥0.5m apart while `EXPLORING` →
    assert `_recorded_path` grows; < 0.5m → assert it doesn't.
  - `stop`: asserts `/explore/resume=False` published, `_save_progress`
    writes path JSON, `_breakpoint_pose` set, `_trigger_map_save` called,
    → `IDLE`.
  - `go_home` from `EXPLORING`/`STUCK`: asserts `_save_progress` then
    `_navigate_path` called with reversed path; from `IDLE` with no recorded
    path: falls back to direct `_navigate_to_home`.
  - `resume`: with saved breakpoint → `_navigate_path` called with forward
    path, completion → resume=True + `EXPLORING`. Without saved breakpoint →
    no-op + warning logged.
  - `_navigate_path`: sequential goal sending, orientation-toward-next-
    waypoint computation, completion callback fires after last waypoint.

## 7. Open Items / Future Work (explicitly deferred)

- Reloading `_recorded_path`/`_breakpoint_pose` from
  `<map_save_path>_path.json` on node restart.
- Multiple breakpoints / history stack (only the most recent is kept).
- Per-leg failure handling beyond "proceed to next waypoint anyway" (e.g.
  retry a failed leg, abort retrace).
