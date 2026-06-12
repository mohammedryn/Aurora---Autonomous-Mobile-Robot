# Exploration Resilience: Persistent Retry, Stuck-Recovery, and Path-Based Resume — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the testing-only 360°-spin recovery in `amr_home_manager` with (1) persistent retry on "no frontiers found", (2) a `STUCK` state reached by stall-watchdog escalation that prompts the operator, and (3) continuous path recording with clean reverse-retrace for `go_home` and forward-retrace for a new `resume` command.

**Architecture:** All changes are confined to `ros2_ws/src/amr_home_manager/amr_home_manager/home_manager_node.py` and its test file `ros2_ws/src/amr_home_manager/test/test_home_manager.py`, plus a small operator-instructions update in `ros2_ws/src/amr_bringup/launch/explore_map.launch.py` and a dependency cleanup in `ros2_ws/src/amr_home_manager/package.xml`. No new Nav2 nodes — path retrace reuses the existing `navigate_to_pose` action client as a sequence of goals.

**Tech Stack:** ROS2 Jazzy, rclpy, pytest (run with `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`).

**Spec:** `docs/superpowers/specs/2026-06-11-explore-resilience-and-resume-design.md`

**Test command** (from `ros2_ws/src/amr_home_manager/`):
```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/ -q
```

---

## Task 1: Remove recovery-spin; persistent retry on `EXPLORATION_COMPLETE`

**Files:**
- Modify: `ros2_ws/src/amr_home_manager/amr_home_manager/home_manager_node.py`
- Modify: `ros2_ws/src/amr_home_manager/test/test_home_manager.py`

- [ ] **Step 1: Update the test file — remove recovery-spin tests, fix `make_node()`, add the persistent-retry test**

Remove these 7 tests entirely (search for each function and delete it):
`test_explore_status_complete_triggers_recovery_spin_when_exploring`,
`test_explore_status_complete_after_max_spins_finishes_exploration`,
`test_command_explore_resets_recovery_spin_counter`,
`test_recovery_spin_server_unavailable_resumes_explore_directly`,
`test_recovery_spin_goal_rejected_resumes_explore`,
`test_recovery_spin_done_resumes_explore`,
`test_explore_status_started_while_already_exploring_keeps_spin_count`.

In `make_node()`, remove this line:
```python
    node._spin_client = MagicMock()
```

A few lines later in `make_node()`, remove this block:
```python
    # Recovery spin: explore_lite "no frontiers" handling
    node._recovery_spins_attempted = 0
    node._max_recovery_spins = 2
    node._spin_time_allowance_s = 20
```

Replace `test_explore_status_complete_ignored_when_not_exploring` with:
```python
def test_explore_status_complete_ignored_when_not_exploring():
    node = make_node()
    node._state = State.IDLE
    node._on_explore_status(_explore_status(ExploreStatus.EXPLORATION_COMPLETE))
    node._explore_pub.publish.assert_not_called()
```

Replace `test_explore_status_in_progress_is_ignored` with:
```python
def test_explore_status_in_progress_is_ignored():
    node = make_node()
    node._state = State.EXPLORING
    node._on_explore_status(_explore_status(ExploreStatus.EXPLORATION_IN_PROGRESS))
    node._explore_pub.publish.assert_not_called()
```

Replace `test_explore_status_started_transitions_idle_to_exploring` with:
```python
def test_explore_status_started_transitions_idle_to_exploring():
    node = make_node()
    node._state = State.IDLE
    node._on_explore_status(_explore_status(ExploreStatus.EXPLORATION_STARTED))
    assert node._state == State.EXPLORING
```

Add this new test at the end of the file:
```python
def test_explore_status_complete_resumes_indefinitely_while_exploring():
    node = make_node()
    node._state = State.EXPLORING
    for _ in range(5):
        node._on_explore_status(_explore_status(ExploreStatus.EXPLORATION_COMPLETE))
        assert node._state == State.EXPLORING
    assert node._explore_pub.publish.call_count == 5
    for call in node._explore_pub.publish.call_args_list:
        assert call[0][0].data is True
    node._save_map_client.call_async.assert_not_called()
```

- [ ] **Step 2: Run tests to verify the new test fails**

```bash
cd ros2_ws/src/amr_home_manager && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/ -q
```
Expected: `test_explore_status_complete_resumes_indefinitely_while_exploring` FAILS with
`AttributeError: 'HomeManagerNode' object has no attribute '_recovery_spins_attempted'`
(the old `_on_explore_status` still reads it). 17 other tests should still pass.

- [ ] **Step 3: Remove recovery-spin code from `home_manager_node.py`**

Replace the imports block (top of file) with:
```python
import enum
import math
import os
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from explore_lite_msgs.msg import ExploreStatus
from slam_toolbox.srv import SaveMap
```

Delete this whole comment+attribute block from `__init__` (currently right after the stall-watchdog block):
```python
        # Recovery spin: explore_lite's "No frontiers found, stopping" can fire
        # after only a few seconds, with most of the room still unmapped --
        # `start_with_rotations` was supposed to cover this but is a phantom
        # parameter (not declared/read by m-explore-ros2). Instead, on
        # EXPLORATION_COMPLETE while still EXPLORING, do a real 360 deg spin via
        # Nav2's Spin behavior to sweep the LiDAR across new area, then resume.
        # Bounded so a genuinely fully-mapped area still terminates.
        self._recovery_spins_attempted = 0
        self._max_recovery_spins = 2
        self._spin_time_allowance_s = 20
```

In `__init__`, delete this line (where the action clients are created):
```python
        self._spin_client = ActionClient(self, Spin, 'spin')
```

In `_on_command`, in the `cmd == 'explore'` branch, delete this line:
```python
            self._recovery_spins_attempted = 0
```

Replace the entire `_on_explore_status` method with:
```python
    def _on_explore_status(self, msg: ExploreStatus) -> None:
        # explore_lite starts exploring on its own as soon as it launches --
        # explore_map.launch.py never publishes /amr/command "explore", so
        # _state would otherwise stay IDLE forever. Track explore_lite's own
        # start/resume announcements instead.
        if msg.status in (ExploreStatus.EXPLORATION_STARTED,
                          ExploreStatus.EXPLORATION_IN_PROGRESS):
            if self._state == State.IDLE:
                self._state = State.EXPLORING
                self.get_logger().info(
                    f'explore_lite status "{msg.status}" -- state -> EXPLORING')
            return

        if msg.status != ExploreStatus.EXPLORATION_COMPLETE:
            return
        if self._state != State.EXPLORING:
            return

        # Never give up automatically -- keep nudging explore_lite to search
        # again. Only an explicit 'stop'/'go_home' command ends exploration.
        # If the robot is also physically not moving, the stall watchdog
        # (_check_stall) escalates to State.STUCK independently.
        self.get_logger().warn(
            'explore reported "no frontiers found" -- resuming, '
            'will keep trying')
        self._resume_explore()
```

Delete the entire `_send_recovery_spin`, `_on_spin_goal_accepted`, and `_on_spin_done` methods (the three methods immediately following `_on_explore_status`).

- [ ] **Step 4: Run tests to verify they pass**

```bash
cd ros2_ws/src/amr_home_manager && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/ -q
```
Expected: `18 passed`.

- [ ] **Step 5: Commit**

```bash
git add ros2_ws/src/amr_home_manager/amr_home_manager/home_manager_node.py ros2_ws/src/amr_home_manager/test/test_home_manager.py
git commit -m "feat(home_manager): remove testing-only recovery spin, retry forever on no-frontiers"
```

---

## Task 2: Stall-watchdog escalation → `STUCK` state

**Files:**
- Modify: `ros2_ws/src/amr_home_manager/amr_home_manager/home_manager_node.py`
- Modify: `ros2_ws/src/amr_home_manager/test/test_home_manager.py`

- [ ] **Step 1: Add `State.STUCK` and update `make_node()`**

In `home_manager_node.py`, update the `State` enum:
```python
class State(enum.Enum):
    IDLE = "idle"
    EXPLORING = "exploring"
    STUCK = "stuck"
    RETURNING_HOME = "returning_home"
    RESUMING = "resuming"
```
(Adding `RESUMING` now too — it's needed by Task 7 and adding it here avoids touching the enum twice.)

In `test_home_manager.py`, in `make_node()`, replace:
```python
    node._last_motion_time = node._clock.now()
    node._stall_nudge_sent = False
    node._stall_timeout_s = 15.0
    node._motion_threshold = 0.02
```
with:
```python
    node._last_motion_time = node._clock.now()
    node._last_nudge_time = node._clock.now()
    node._stall_nudge_count = 0
    node._stall_timeout_s = 15.0
    node._motion_threshold = 0.02
    node._max_stall_retries = 15
    node._stuck_prompt_delay_s = 3.0
    node._stuck_pending = False
```

- [ ] **Step 2: Rewrite the stall-watchdog tests for the new escalation behavior**

Replace `test_on_odom_moving_resets_stall_clock_and_flag` with:
```python
def test_on_odom_moving_resets_stall_and_stuck_state():
    node = make_node()
    node._stall_nudge_count = 3
    node._stuck_pending = True
    node._clock.seconds = 100.0
    odom_msg = MagicMock()
    odom_msg.pose.pose.position.x = 0.0
    odom_msg.pose.pose.position.y = 0.0
    _set_twist(odom_msg, x=0.1)
    node._on_odom(odom_msg)
    assert node._last_motion_time.seconds == 100.0
    assert node._last_nudge_time.seconds == 100.0
    assert node._stall_nudge_count == 0
    assert node._stuck_pending is False
```

Replace `test_on_odom_still_does_not_touch_stall_clock` with:
```python
def test_on_odom_still_does_not_touch_stall_clock():
    node = make_node()
    node._stall_nudge_count = 2
    node._last_motion_time = node._clock.now()
    node._last_nudge_time = node._clock.now()
    node._clock.seconds = 100.0
    odom_msg = MagicMock()
    odom_msg.pose.pose.position.x = 0.0
    odom_msg.pose.pose.position.y = 0.0
    _set_twist(odom_msg)
    node._on_odom(odom_msg)
    assert node._last_motion_time.seconds == 0.0
    assert node._last_nudge_time.seconds == 0.0
    assert node._stall_nudge_count == 2
```

Replace `test_check_stall_nudges_resume_after_timeout_while_exploring` with:
```python
def test_check_stall_nudges_resume_after_timeout_while_exploring():
    node = make_node()
    node._state = State.EXPLORING
    node._clock.seconds = 20.0
    node._check_stall()
    node._explore_pub.publish.assert_called_once()
    published = node._explore_pub.publish.call_args[0][0]
    assert published.data is True
    assert node._stall_nudge_count == 1
    assert node._last_nudge_time.seconds == 20.0
```

Replace `test_check_stall_does_not_nudge_again_before_motion_resumes` with two tests:
```python
def test_check_stall_does_not_renudge_within_timeout_window():
    node = make_node()
    node._state = State.EXPLORING
    node._clock.seconds = 20.0
    node._check_stall()
    node._clock.seconds = 25.0
    node._check_stall()
    node._explore_pub.publish.assert_called_once()
    assert node._stall_nudge_count == 1


def test_check_stall_renudges_after_another_full_timeout():
    node = make_node()
    node._state = State.EXPLORING
    node._clock.seconds = 20.0
    node._check_stall()
    node._clock.seconds = 35.0
    node._check_stall()
    assert node._explore_pub.publish.call_count == 2
    assert node._stall_nudge_count == 2
    assert node._last_nudge_time.seconds == 35.0
```

`test_check_stall_ignores_non_exploring_state` and `test_check_stall_does_nothing_before_timeout`
are unchanged — leave them as-is.

- [ ] **Step 3: Add tests for STUCK escalation and `_enter_stuck`**

Add these new tests at the end of the file:
```python
def test_check_stall_enters_stuck_pending_after_max_retries():
    node = make_node()
    node._state = State.EXPLORING
    node._stall_nudge_count = node._max_stall_retries
    node._last_nudge_time = _FakeTime(0.0)
    node._last_motion_time = _FakeTime(0.0)
    node._clock.seconds = 20.0
    node._check_stall()
    assert node._stuck_pending is True
    # No further resume nudge once max retries reached.
    node._explore_pub.publish.assert_not_called()


def test_check_stall_does_not_double_schedule_stuck_pending():
    node = make_node()
    node._state = State.EXPLORING
    node._stall_nudge_count = node._max_stall_retries
    node._stuck_pending = True
    node._last_nudge_time = _FakeTime(0.0)
    node._last_motion_time = _FakeTime(0.0)
    node._clock.seconds = 20.0
    node._check_stall()
    assert node._stuck_pending is True


def test_enter_stuck_transitions_state_when_still_stalled():
    node = make_node()
    node._state = State.EXPLORING
    node._stuck_timer = MagicMock()
    node._last_motion_time = _FakeTime(0.0)
    node._clock.seconds = 20.0
    node._enter_stuck()
    assert node._state == State.STUCK
    assert node._stuck_pending is False
    node._stuck_timer.cancel.assert_called_once()


def test_enter_stuck_aborts_if_motion_resumed_during_delay():
    node = make_node()
    node._state = State.EXPLORING
    node._stuck_timer = MagicMock()
    node._last_motion_time = _FakeTime(10.0)
    node._clock.seconds = 12.0
    node._enter_stuck()
    assert node._state == State.EXPLORING
    assert node._stuck_pending is False


def test_check_stall_ignores_stuck_state():
    node = make_node()
    node._state = State.STUCK
    node._clock.seconds = 1000.0
    node._check_stall()
    node._explore_pub.publish.assert_not_called()
```

Note: `_FakeTime` is already defined near the top of the file (used by `_FakeClock`).

- [ ] **Step 4: Run tests to verify the new/changed tests fail**

```bash
cd ros2_ws/src/amr_home_manager && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/ -q
```
Expected: failures referencing `_last_nudge_time`/`_stall_nudge_count`/`_enter_stuck`/`State.STUCK`
not existing yet (AttributeError / AttributeError on enum member).

- [ ] **Step 5: Implement stall escalation in `home_manager_node.py`**

Replace the `_on_odom` motion-detection block:
```python
        v = msg.twist.twist
        moving = (abs(v.linear.x) > self._motion_threshold
                  or abs(v.linear.y) > self._motion_threshold
                  or abs(v.angular.z) > self._motion_threshold)
        if moving:
            self._last_motion_time = self.get_clock().now()
            self._stall_nudge_sent = False
```
with:
```python
        v = msg.twist.twist
        moving = (abs(v.linear.x) > self._motion_threshold
                  or abs(v.linear.y) > self._motion_threshold
                  or abs(v.angular.z) > self._motion_threshold)
        if moving:
            self._last_motion_time = self.get_clock().now()
            self._last_nudge_time = self.get_clock().now()
            self._stall_nudge_count = 0
            self._stuck_pending = False
```

Replace the entire `_check_stall` method with:
```python
    def _check_stall(self) -> None:
        if self._state != State.EXPLORING:
            return
        now = self.get_clock().now()
        stalled_for = (now - self._last_motion_time).nanoseconds / 1e9
        since_nudge = (now - self._last_nudge_time).nanoseconds / 1e9
        if stalled_for <= self._stall_timeout_s or since_nudge < self._stall_timeout_s:
            return
        if self._stall_nudge_count < self._max_stall_retries:
            self._stall_nudge_count += 1
            self._last_nudge_time = now
            self.get_logger().warn(
                f'No motion for {stalled_for:.0f}s -- nudging '
                f'/explore/resume ({self._stall_nudge_count}/'
                f'{self._max_stall_retries})')
            self._resume_explore()
            return
        if not self._stuck_pending:
            self._stuck_pending = True
            self._stuck_timer = self.create_timer(
                self._stuck_prompt_delay_s, self._enter_stuck)

    def _enter_stuck(self) -> None:
        self._stuck_timer.cancel()
        self._stuck_pending = False
        if self._state != State.EXPLORING:
            return
        stalled_for = (self.get_clock().now()
                       - self._last_motion_time).nanoseconds / 1e9
        if stalled_for <= self._stall_timeout_s:
            return
        self._state = State.STUCK
        self.get_logger().warn(
            f'Robot stuck -- cannot make further progress after '
            f'{self._max_stall_retries} retries.\n'
            "Send /amr/command 'stop' (save map+path, stay here) or "
            "'go_home' (save + retrace path home).")
```

Add `_max_stall_retries`, `_stuck_prompt_delay_s`, `_stuck_pending`, and `_last_nudge_time` to
`__init__`. Replace this block:
```python
        self._last_motion_time = self.get_clock().now()
        self._stall_nudge_sent = False
        self._stall_timeout_s = 15.0
        self._motion_threshold = 0.02
```
with:
```python
        self._last_motion_time = self.get_clock().now()
        self._last_nudge_time = self.get_clock().now()
        self._stall_nudge_count = 0
        self._stall_timeout_s = 15.0
        self._motion_threshold = 0.02
        self._max_stall_retries = 15
        self._stuck_prompt_delay_s = 3.0
        self._stuck_pending = False
```

- [ ] **Step 6: Run tests to verify they pass**

```bash
cd ros2_ws/src/amr_home_manager && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/ -q
```
Expected: `24 passed`.

- [ ] **Step 7: Commit**

```bash
git add ros2_ws/src/amr_home_manager/amr_home_manager/home_manager_node.py ros2_ws/src/amr_home_manager/test/test_home_manager.py
git commit -m "feat(home_manager): escalate stall watchdog to STUCK state after 15 nudges"
```

---

## Task 3: Path recording infrastructure

**Files:**
- Modify: `ros2_ws/src/amr_home_manager/amr_home_manager/home_manager_node.py`
- Modify: `ros2_ws/src/amr_home_manager/test/test_home_manager.py`

- [ ] **Step 1: Add test helpers and `make_node()` fields for path recording**

In `test_home_manager.py`, add `import math` near the top (with the other imports).

Add this helper class near `_FakeClock` (it stands in for `PoseStamped` since
`geometry_msgs.msg` is fully mocked — `_pose_to_xyyaw` needs real numeric
`.position.x/y` and `.orientation.z/w`):
```python
class _FakePose:
    """Stand-in for PoseStamped with real numeric position/orientation."""
    def __init__(self, x=0.0, y=0.0, yaw=0.0):
        self.header = MagicMock()
        self.header.frame_id = 'map'
        self.pose = MagicMock()
        self.pose.position.x = x
        self.pose.position.y = y
        self.pose.position.z = 0.0
        self.pose.orientation.x = 0.0
        self.pose.orientation.y = 0.0
        self.pose.orientation.z = math.sin(yaw / 2.0)
        self.pose.orientation.w = math.cos(yaw / 2.0)
```

Add a helper next to `_set_twist` for setting odom position+orientation:
```python
def _set_pose(odom_msg, x=0.0, y=0.0, yaw=0.0):
    odom_msg.pose.pose.position.x = x
    odom_msg.pose.pose.position.y = y
    odom_msg.pose.pose.orientation.x = 0.0
    odom_msg.pose.pose.orientation.y = 0.0
    odom_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
    odom_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
```

In `make_node()`, add after the stuck-related fields:
```python
    node._recorded_path = []
    node._breakpoint_pose = None
    node._path_sample_distance_m = 0.5
```

In `test_command_explore_transitions_to_exploring`, change
`node._home_pose = MagicMock()` to `node._home_pose = _FakePose()`
(this test now goes through `_reset_path_recording`, which calls
`_pose_to_xyyaw(self._home_pose.pose)`).

- [ ] **Step 2: Add path-recording tests**

Add these new tests at the end of the file:
```python
# ---- path recording ----

def test_record_path_point_appends_first_sample():
    node = make_node()
    node._state = State.EXPLORING
    odom_msg = MagicMock()
    _set_pose(odom_msg, x=1.0, y=2.0, yaw=0.5)
    _set_twist(odom_msg)
    node._on_odom(odom_msg)
    assert node._recorded_path == [(1.0, 2.0, 0.5)]


def test_record_path_point_skips_small_movements():
    node = make_node()
    node._state = State.EXPLORING
    node._recorded_path = [(0.0, 0.0, 0.0)]
    odom_msg = MagicMock()
    _set_pose(odom_msg, x=0.1, y=0.0, yaw=0.0)
    _set_twist(odom_msg)
    node._on_odom(odom_msg)
    assert node._recorded_path == [(0.0, 0.0, 0.0)]


def test_record_path_point_appends_after_threshold_distance():
    node = make_node()
    node._state = State.EXPLORING
    node._recorded_path = [(0.0, 0.0, 0.0)]
    odom_msg = MagicMock()
    _set_pose(odom_msg, x=0.6, y=0.0, yaw=0.0)
    _set_twist(odom_msg)
    node._on_odom(odom_msg)
    assert node._recorded_path == [(0.0, 0.0, 0.0), (0.6, 0.0, 0.0)]


def test_record_path_point_ignored_when_idle():
    node = make_node()
    node._state = State.IDLE
    node._home_pose = _FakePose()
    node._recorded_path = []
    odom_msg = MagicMock()
    _set_pose(odom_msg, x=5.0, y=5.0, yaw=0.0)
    _set_twist(odom_msg)
    node._on_odom(odom_msg)
    assert node._recorded_path == []


def test_record_path_point_recorded_while_stuck():
    node = make_node()
    node._state = State.STUCK
    node._recorded_path = [(0.0, 0.0, 0.0)]
    odom_msg = MagicMock()
    _set_pose(odom_msg, x=1.0, y=0.0, yaw=0.0)
    _set_twist(odom_msg)
    node._on_odom(odom_msg)
    assert node._recorded_path == [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)]


# ---- path reset on (re)start ----

def test_command_explore_resets_recorded_path_to_home():
    node = make_node()
    node._home_pose = _FakePose(x=1.0, y=2.0, yaw=0.0)
    node._recorded_path = [(9.0, 9.0, 0.0)]
    node._breakpoint_pose = _FakePose()
    msg = MagicMock()
    msg.data = "explore"
    node._on_command(msg)
    assert node._recorded_path == [(1.0, 2.0, 0.0)]
    assert node._breakpoint_pose is None


def test_explore_status_started_resets_recorded_path_to_home():
    node = make_node()
    node._state = State.IDLE
    node._home_pose = _FakePose(x=3.0, y=4.0, yaw=0.0)
    node._recorded_path = [(9.0, 9.0, 0.0)]
    node._on_explore_status(_explore_status(ExploreStatus.EXPLORATION_STARTED))
    assert node._state == State.EXPLORING
    assert node._recorded_path == [(3.0, 4.0, 0.0)]
```

- [ ] **Step 3: Run tests to verify the new tests fail**

```bash
cd ros2_ws/src/amr_home_manager && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/ -q
```
Expected: failures — `_recorded_path` not updated by `_on_odom` (no `_record_path_point`/
`_pose_to_xyyaw`/`_reset_path_recording` yet).

- [ ] **Step 4: Implement path recording in `home_manager_node.py`**

Add two helper methods anywhere convenient (e.g. just before `_on_odom`):
```python
    def _pose_to_xyyaw(self, pose) -> tuple[float, float, float]:
        x = pose.position.x
        y = pose.position.y
        q = pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return (x, y, yaw)

    def _record_path_point(self, pose) -> None:
        x, y, yaw = self._pose_to_xyyaw(pose)
        if self._recorded_path:
            last_x, last_y, _ = self._recorded_path[-1]
            if math.hypot(x - last_x, y - last_y) < self._path_sample_distance_m:
                return
        self._recorded_path.append((x, y, yaw))

    def _reset_path_recording(self) -> None:
        self._recorded_path = []
        if self._home_pose is not None:
            self._recorded_path.append(self._pose_to_xyyaw(self._home_pose.pose))
        self._breakpoint_pose = None
        self._stall_nudge_count = 0
        self._last_nudge_time = self.get_clock().now()
        self._stuck_pending = False
```

In `_on_odom`, append a call to `_record_path_point` after the motion-detection
block (so it runs every message regardless of motion):
```python
        if moving:
            self._last_motion_time = self.get_clock().now()
            self._last_nudge_time = self.get_clock().now()
            self._stall_nudge_count = 0
            self._stuck_pending = False

        if self._state in (State.EXPLORING, State.STUCK):
            self._record_path_point(msg.pose.pose)
```

In `_on_command`, `cmd == 'explore'` branch, call `_reset_path_recording()`
right after setting `self._state = State.EXPLORING`:
```python
        if cmd == 'explore':
            if self._home_pose is None:
                self.get_logger().warn('Home pose not yet recorded — waiting for /odom')
                return
            self._state = State.EXPLORING
            self._reset_path_recording()
            resume = Bool()
            resume.data = True
            self._explore_pub.publish(resume)
            self.get_logger().info('Exploration started')
```

In `_on_explore_status`, the `EXPLORATION_STARTED`/`EXPLORATION_IN_PROGRESS`
branch, call `_reset_path_recording()` when transitioning from `IDLE`:
```python
        if msg.status in (ExploreStatus.EXPLORATION_STARTED,
                          ExploreStatus.EXPLORATION_IN_PROGRESS):
            if self._state == State.IDLE:
                self._state = State.EXPLORING
                self._reset_path_recording()
                self.get_logger().info(
                    f'explore_lite status "{msg.status}" -- state -> EXPLORING')
            return
```

- [ ] **Step 5: Run tests to verify they pass**

```bash
cd ros2_ws/src/amr_home_manager && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/ -q
```
Expected: `31 passed`.

- [ ] **Step 6: Commit**

```bash
git add ros2_ws/src/amr_home_manager/amr_home_manager/home_manager_node.py ros2_ws/src/amr_home_manager/test/test_home_manager.py
git commit -m "feat(home_manager): record travelled path during exploration"
```

---

## Task 4: `_save_progress` (map + path persistence) and `stop` rewrite

**Files:**
- Modify: `ros2_ws/src/amr_home_manager/amr_home_manager/home_manager_node.py`
- Modify: `ros2_ws/src/amr_home_manager/test/test_home_manager.py`

- [ ] **Step 1: Add tests for `_save_progress`, `_pause_explore`, `_save_path_to_disk`, and the rewritten `stop`**

In `test_home_manager.py`, add `import json` near the top.

Add these new tests at the end of the file:
```python
# ---- _save_progress / stop ----

def test_pause_explore_publishes_false():
    node = make_node()
    node._pause_explore()
    node._explore_pub.publish.assert_called_once()
    assert node._explore_pub.publish.call_args[0][0].data is False


def test_save_progress_records_breakpoint_and_saves_map_and_path(tmp_path):
    node = make_node()
    node._recorded_path = [(0.0, 0.0, 0.0), (1.5, 2.5, 0.7)]
    node._save_map_client.wait_for_service.return_value = True
    node._save_map_client.call_async.return_value = MagicMock()
    save_path = str(tmp_path / "explore_map")
    node.get_parameter = lambda name: MagicMock(
        get_parameter_value=lambda: MagicMock(string_value=save_path))
    node._save_progress()

    assert node._explore_pub.publish.call_args[0][0].data is False
    assert node._breakpoint_pose.pose.position.x == 1.5
    assert node._breakpoint_pose.pose.position.y == 2.5
    node._save_map_client.call_async.assert_called_once()

    with open(save_path + "_path.json") as f:
        data = json.load(f)
    assert data == {"path": [[0.0, 0.0, 0.0], [1.5, 2.5, 0.7]]}


def test_save_progress_with_empty_path_uses_origin_breakpoint(tmp_path):
    node = make_node()
    node._recorded_path = []
    node._save_map_client.wait_for_service.return_value = True
    node._save_map_client.call_async.return_value = MagicMock()
    save_path = str(tmp_path / "explore_map")
    node.get_parameter = lambda name: MagicMock(
        get_parameter_value=lambda: MagicMock(string_value=save_path))
    node._save_progress()
    assert node._breakpoint_pose.pose.position.x == 0.0
    assert node._breakpoint_pose.pose.position.y == 0.0


def test_command_stop_while_exploring_saves_progress_and_goes_idle(tmp_path):
    node = make_node()
    node._state = State.EXPLORING
    node._recorded_path = [(0.0, 0.0, 0.0), (2.0, 0.0, 0.0)]
    node._save_map_client.wait_for_service.return_value = True
    node._save_map_client.call_async.return_value = MagicMock()
    save_path = str(tmp_path / "explore_map")
    node.get_parameter = lambda name: MagicMock(
        get_parameter_value=lambda: MagicMock(string_value=save_path))
    msg = MagicMock()
    msg.data = "stop"
    node._on_command(msg)
    assert node._state == State.IDLE
    assert node._explore_pub.publish.call_args[0][0].data is False
    node._save_map_client.call_async.assert_called_once()
    assert node._breakpoint_pose is not None


def test_command_stop_while_stuck_saves_progress_and_goes_idle(tmp_path):
    node = make_node()
    node._state = State.STUCK
    node._recorded_path = [(0.0, 0.0, 0.0)]
    node._save_map_client.wait_for_service.return_value = True
    node._save_map_client.call_async.return_value = MagicMock()
    save_path = str(tmp_path / "explore_map")
    node.get_parameter = lambda name: MagicMock(
        get_parameter_value=lambda: MagicMock(string_value=save_path))
    msg = MagicMock()
    msg.data = "stop"
    node._on_command(msg)
    assert node._state == State.IDLE


def test_command_stop_while_idle_is_noop():
    node = make_node()
    node._state = State.IDLE
    msg = MagicMock()
    msg.data = "stop"
    node._on_command(msg)
    assert node._state == State.IDLE
    node._explore_pub.publish.assert_not_called()
    node._save_map_client.call_async.assert_not_called()
```

Remove `test_exploration_done_calls_save_map_service_and_transitions` — `_on_exploration_done`
is being removed in favor of `_save_progress` (already covered by the new `stop` tests above).

- [ ] **Step 2: Run tests to verify the new tests fail**

```bash
cd ros2_ws/src/amr_home_manager && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/ -q
```
Expected: failures — `_save_progress`/`_pause_explore`/`_save_path_to_disk` don't exist yet,
and `stop` still calls the old `_on_exploration_done`.

- [ ] **Step 3: Implement `_save_progress` and rewrite `stop` in `home_manager_node.py`**

Add `import json` to the imports block (top of file, alongside `import math`).

Change the `geometry_msgs.msg` import line:
```python
from geometry_msgs.msg import PoseStamped
```
to:
```python
from geometry_msgs.msg import PoseStamped, Quaternion
```

Replace the `_on_exploration_done` method with:
```python
    def _pause_explore(self) -> None:
        msg = Bool()
        msg.data = False
        self._explore_pub.publish(msg)

    def _save_progress(self) -> None:
        self._pause_explore()
        if self._recorded_path:
            x, y, yaw = self._recorded_path[-1]
        else:
            x, y, yaw = 0.0, 0.0, 0.0
        bp = PoseStamped()
        bp.header.frame_id = 'map'
        bp.pose.position.x = x
        bp.pose.position.y = y
        bp.pose.orientation = self._yaw_to_quaternion(yaw)
        self._breakpoint_pose = bp
        self._trigger_map_save()
        self._save_path_to_disk()

    def _yaw_to_quaternion(self, yaw: float):
        q = Quaternion()
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q

    def _save_path_to_disk(self) -> None:
        base = self.get_parameter(
            'map_save_path').get_parameter_value().string_value
        json_path = f'{base}_path.json'
        try:
            with open(json_path, 'w') as f:
                json.dump({'path': [list(p) for p in self._recorded_path]}, f)
            self.get_logger().info(f'Path saved: {json_path}')
        except OSError as e:
            self.get_logger().error(f'Failed to save path: {e}')
```

In `_on_command`, replace the `cmd == 'stop'` branch:
```python
        elif cmd == 'stop':
            if self._state == State.EXPLORING:
                self._on_exploration_done()
```
with:
```python
        elif cmd == 'stop':
            if self._state in (State.EXPLORING, State.STUCK):
                self._save_progress()
                self._state = State.IDLE
                self.get_logger().info('Exploration stopped. Progress saved.')
            else:
                self.get_logger().info('Already idle -- nothing to stop')
```

- [ ] **Step 4: Run tests to verify they pass**

```bash
cd ros2_ws/src/amr_home_manager && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/ -q
```
Expected: `36 passed`.

- [ ] **Step 5: Commit**

```bash
git add ros2_ws/src/amr_home_manager/amr_home_manager/home_manager_node.py ros2_ws/src/amr_home_manager/test/test_home_manager.py
git commit -m "feat(home_manager): save map+path breakpoint on stop, replacing _on_exploration_done"
```

---

## Task 5: `_navigate_path` — sequential `NavigateToPose` waypoint follower

**Files:**
- Modify: `ros2_ws/src/amr_home_manager/amr_home_manager/home_manager_node.py`
- Modify: `ros2_ws/src/amr_home_manager/test/test_home_manager.py`

- [ ] **Step 1: Add tests for `_navigate_path`**

Add these new tests at the end of the file:
```python
# ---- _navigate_path: sequential NavigateToPose waypoint follower ----

def test_navigate_path_sends_first_waypoint_with_heading_to_next():
    node = make_node()
    node._nav_client.wait_for_server.return_value = True
    send_future = MagicMock()
    node._nav_client.send_goal_async.return_value = send_future

    on_done = MagicMock()
    node._navigate_path([(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (1.0, 1.0, 0.0)], on_done)

    node._nav_client.send_goal_async.assert_called_once()
    goal = node._nav_client.send_goal_async.call_args[0][0]
    assert goal.pose.pose.position.x == 0.0
    assert goal.pose.pose.position.y == 0.0
    # heading toward (1.0, 0.0) from (0.0, 0.0) is yaw=0
    assert goal.pose.pose.orientation.z == pytest.approx(math.sin(0.0))
    assert goal.pose.pose.orientation.w == pytest.approx(math.cos(0.0))
    on_done.assert_not_called()


def test_navigate_path_advances_through_all_waypoints_then_calls_done():
    node = make_node()
    node._nav_client.wait_for_server.return_value = True

    send_futures = [MagicMock() for _ in range(3)]
    node._nav_client.send_goal_async.side_effect = send_futures

    on_done = MagicMock()
    waypoints = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (1.0, 1.0, 0.3)]
    node._navigate_path(waypoints, on_done)

    for i in range(3):
        accept_cb = send_futures[i].add_done_callback.call_args[0][0]
        handle = MagicMock()
        handle.accepted = True
        result_future = MagicMock()
        handle.get_result_async.return_value = result_future
        accept_future = MagicMock()
        accept_future.result.return_value = handle
        accept_cb(accept_future)

        result_cb = result_future.add_done_callback.call_args[0][0]
        result_cb(MagicMock())

    assert node._nav_client.send_goal_async.call_count == 3
    # last waypoint keeps its own recorded yaw (0.3), no "next" to face.
    last_goal = node._nav_client.send_goal_async.call_args_list[2][0][0]
    assert last_goal.pose.pose.orientation.z == pytest.approx(math.sin(0.15))
    on_done.assert_called_once()


def test_navigate_path_skips_waypoint_on_rejected_goal_and_finishes():
    node = make_node()
    node._nav_client.wait_for_server.return_value = True

    send_future = MagicMock()
    node._nav_client.send_goal_async.return_value = send_future

    on_done = MagicMock()
    waypoints = [(0.0, 0.0, 0.0)]
    node._navigate_path(waypoints, on_done)

    accept_cb = send_future.add_done_callback.call_args[0][0]
    handle = MagicMock()
    handle.accepted = False
    accept_future = MagicMock()
    accept_future.result.return_value = handle
    accept_cb(accept_future)

    node._nav_client.send_goal_async.assert_called_once()
    on_done.assert_called_once()


def test_navigate_path_empty_calls_done_immediately():
    node = make_node()
    on_done = MagicMock()
    node._navigate_path([], on_done)
    node._nav_client.send_goal_async.assert_not_called()
    on_done.assert_called_once()
```

`pytest` and `math` are already imported at the top of the test file.

- [ ] **Step 2: Run tests to verify the new tests fail**

```bash
cd ros2_ws/src/amr_home_manager && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/ -q
```
Expected: failures — `_navigate_path` doesn't exist.

- [ ] **Step 3: Implement `_navigate_path` in `home_manager_node.py`**

Add these methods near `_navigate_to_home`:
```python
    def _navigate_path(self, waypoints, on_done) -> None:
        """Drive through `waypoints` (list of (x, y, yaw)) one at a time via
        sequential navigate_to_pose goals, calling on_done() once the last
        one's result comes back (success or failure -- a failed leg doesn't
        abort the retrace, matching "keep trying" rather than stop)."""
        self._waypoint_queue = list(waypoints)
        self._waypoint_done_cb = on_done
        self._send_next_waypoint()

    def _send_next_waypoint(self) -> None:
        if not self._waypoint_queue:
            cb = self._waypoint_done_cb
            self._waypoint_done_cb = None
            if cb:
                cb()
            return
        x, y, yaw = self._waypoint_queue.pop(0)
        if self._waypoint_queue:
            nx, ny, _ = self._waypoint_queue[0]
            yaw = math.atan2(ny - y, nx - x)
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                'NavigateToPose action server not available -- skipping waypoint')
            self._send_next_waypoint()
            return
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation = self._yaw_to_quaternion(yaw)
        send_future = self._nav_client.send_goal_async(goal)
        send_future.add_done_callback(self._on_waypoint_goal_accepted)

    def _on_waypoint_goal_accepted(self, future) -> None:
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Waypoint goal rejected -- skipping')
            self._send_next_waypoint()
            return
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._on_waypoint_result)

    def _on_waypoint_result(self, future) -> None:
        self._send_next_waypoint()
```

- [ ] **Step 4: Run tests to verify they pass**

```bash
cd ros2_ws/src/amr_home_manager && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/ -q
```
Expected: `40 passed`.

- [ ] **Step 5: Commit**

```bash
git add ros2_ws/src/amr_home_manager/amr_home_manager/home_manager_node.py ros2_ws/src/amr_home_manager/test/test_home_manager.py
git commit -m "feat(home_manager): add sequential NavigateToPose waypoint follower"
```

---

## Task 6: `go_home` — retrace recorded path in reverse

**Files:**
- Modify: `ros2_ws/src/amr_home_manager/amr_home_manager/home_manager_node.py`
- Modify: `ros2_ws/src/amr_home_manager/test/test_home_manager.py`

- [ ] **Step 1: Update/add `go_home` tests**

Replace `test_command_go_home_transitions_to_returning` with:
```python
def test_command_go_home_with_no_recorded_path_falls_back_to_direct_nav():
    node = make_node()
    node._home_pose = _FakePose()
    node._recorded_path = [(0.0, 0.0, 0.0)]  # only home -- nothing recorded yet
    node._nav_client.wait_for_server.return_value = True
    msg = MagicMock()
    msg.data = "go_home"
    node._on_command(msg)
    assert node._state == State.RETURNING_HOME
    node._nav_client.send_goal_async.assert_called_once()
```

Add these new tests at the end of the file:
```python
def test_command_go_home_while_exploring_saves_progress_then_retraces(tmp_path):
    node = make_node()
    node._state = State.EXPLORING
    node._home_pose = _FakePose(x=0.0, y=0.0, yaw=0.0)
    node._recorded_path = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (2.0, 0.0, 0.0)]
    node._save_map_client.wait_for_service.return_value = True
    node._save_map_client.call_async.return_value = MagicMock()
    node._nav_client.wait_for_server.return_value = True
    save_path = str(tmp_path / "explore_map")
    node.get_parameter = lambda name: MagicMock(
        get_parameter_value=lambda: MagicMock(string_value=save_path))

    msg = MagicMock()
    msg.data = "go_home"
    node._on_command(msg)

    # progress saved (resume=False, map+path) before retracing
    assert node._explore_pub.publish.call_args_list[0][0][0].data is False
    node._save_map_client.call_async.assert_called_once()
    assert node._state == State.RETURNING_HOME

    # first waypoint of the retrace is the most recently recorded point
    goal = node._nav_client.send_goal_async.call_args[0][0]
    assert goal.pose.pose.position.x == 2.0


def test_command_go_home_retrace_completion_sets_idle():
    node = make_node()
    node._home_pose = _FakePose(x=0.0, y=0.0, yaw=0.0)
    node._recorded_path = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)]
    node._nav_client.wait_for_server.return_value = True
    send_futures = [MagicMock(), MagicMock()]
    node._nav_client.send_goal_async.side_effect = send_futures

    msg = MagicMock()
    msg.data = "go_home"
    node._on_command(msg)
    assert node._state == State.RETURNING_HOME

    for sf in send_futures:
        accept_cb = sf.add_done_callback.call_args[0][0]
        handle = MagicMock()
        handle.accepted = True
        result_future = MagicMock()
        handle.get_result_async.return_value = result_future
        accept_future = MagicMock()
        accept_future.result.return_value = handle
        accept_cb(accept_future)
        result_cb = result_future.add_done_callback.call_args[0][0]
        result_cb(MagicMock())

    assert node._state == State.IDLE
```

- [ ] **Step 2: Run tests to verify the new/changed tests fail**

```bash
cd ros2_ws/src/amr_home_manager && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/ -q
```
Expected: failures — `go_home` still does the old single-`NavigateToPose`-only
behavior and never calls `_save_progress`/`_navigate_path`.

- [ ] **Step 3: Rewrite the `go_home` branch and add `_on_returned_home`**

In `_on_command`, replace the `cmd == 'go_home'` branch:
```python
        elif cmd == 'go_home':
            if self._home_pose is None:
                self.get_logger().warn('No home pose recorded — cannot return home')
                return
            self._state = State.RETURNING_HOME
            self._navigate_to_home()
```
with:
```python
        elif cmd == 'go_home':
            if self._home_pose is None:
                self.get_logger().warn('No home pose recorded — cannot return home')
                return
            if self._state in (State.EXPLORING, State.STUCK):
                self._save_progress()
            self._state = State.RETURNING_HOME
            if len(self._recorded_path) <= 1:
                self._navigate_to_home()
            else:
                self._navigate_path(list(reversed(self._recorded_path)),
                                     self._on_returned_home)
```

Add `_on_returned_home` near `_on_nav_done`:
```python
    def _on_returned_home(self) -> None:
        self._state = State.IDLE
        self.get_logger().info('Returned home (retraced path). State: IDLE')
```

- [ ] **Step 4: Run tests to verify they pass**

```bash
cd ros2_ws/src/amr_home_manager && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/ -q
```
Expected: `42 passed`.

- [ ] **Step 5: Commit**

```bash
git add ros2_ws/src/amr_home_manager/amr_home_manager/home_manager_node.py ros2_ws/src/amr_home_manager/test/test_home_manager.py
git commit -m "feat(home_manager): go_home retraces recorded path in reverse"
```

---

## Task 7: `resume` command — retrace forward to breakpoint, then continue exploring

**Files:**
- Modify: `ros2_ws/src/amr_home_manager/amr_home_manager/home_manager_node.py`
- Modify: `ros2_ws/src/amr_home_manager/test/test_home_manager.py`

- [ ] **Step 1: Add `resume` command tests**

Add these new tests at the end of the file:
```python
# ---- resume: retrace forward to breakpoint, then continue exploring ----

def test_command_resume_without_breakpoint_is_noop():
    node = make_node()
    node._state = State.IDLE
    node._breakpoint_pose = None
    node._recorded_path = [(0.0, 0.0, 0.0)]
    msg = MagicMock()
    msg.data = "resume"
    node._on_command(msg)
    assert node._state == State.IDLE
    node._nav_client.send_goal_async.assert_not_called()


def test_command_resume_retraces_forward_to_breakpoint():
    node = make_node()
    node._state = State.IDLE
    node._home_pose = _FakePose(x=0.0, y=0.0, yaw=0.0)
    node._recorded_path = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (2.0, 0.0, 0.0)]
    node._breakpoint_pose = _FakePose(x=2.0, y=0.0, yaw=0.0)
    node._nav_client.wait_for_server.return_value = True

    msg = MagicMock()
    msg.data = "resume"
    node._on_command(msg)

    assert node._state == State.RESUMING
    goal = node._nav_client.send_goal_async.call_args[0][0]
    # forward order: first waypoint sent is home (0,0)
    assert goal.pose.pose.position.x == 0.0


def test_command_resume_completion_resumes_exploring():
    node = make_node()
    node._state = State.IDLE
    node._home_pose = _FakePose(x=0.0, y=0.0, yaw=0.0)
    node._recorded_path = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)]
    node._breakpoint_pose = _FakePose(x=1.0, y=0.0, yaw=0.0)
    node._nav_client.wait_for_server.return_value = True
    send_futures = [MagicMock(), MagicMock()]
    node._nav_client.send_goal_async.side_effect = send_futures

    msg = MagicMock()
    msg.data = "resume"
    node._on_command(msg)
    assert node._state == State.RESUMING

    for sf in send_futures:
        accept_cb = sf.add_done_callback.call_args[0][0]
        handle = MagicMock()
        handle.accepted = True
        result_future = MagicMock()
        handle.get_result_async.return_value = result_future
        accept_future = MagicMock()
        accept_future.result.return_value = handle
        accept_cb(accept_future)
        result_cb = result_future.add_done_callback.call_args[0][0]
        result_cb(MagicMock())

    assert node._state == State.EXPLORING
    assert node._explore_pub.publish.call_args[0][0].data is True
```

- [ ] **Step 2: Run tests to verify the new tests fail**

```bash
cd ros2_ws/src/amr_home_manager && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/ -q
```
Expected: failures — `_on_command` doesn't recognize `"resume"` yet.

- [ ] **Step 3: Implement the `resume` command**

In `_on_command`, add a new branch after `elif cmd == 'go_home': ...`:
```python
        elif cmd == 'resume':
            if self._breakpoint_pose is None or len(self._recorded_path) <= 1:
                self.get_logger().warn('No saved breakpoint -- nothing to resume')
                return
            self._state = State.RESUMING
            self._navigate_path(list(self._recorded_path), self._on_resume_arrived)
```

Add `_on_resume_arrived` near `_on_returned_home`:
```python
    def _on_resume_arrived(self) -> None:
        self._resume_explore()
        self._state = State.EXPLORING
        self.get_logger().info('Resumed at breakpoint -- exploration continuing')
```

- [ ] **Step 4: Run tests to verify they pass**

```bash
cd ros2_ws/src/amr_home_manager && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/ -q
```
Expected: `45 passed`.

- [ ] **Step 5: Commit**

```bash
git add ros2_ws/src/amr_home_manager/amr_home_manager/home_manager_node.py ros2_ws/src/amr_home_manager/test/test_home_manager.py
git commit -m "feat(home_manager): add resume command to retrace path forward and continue exploring"
```

---

## Task 8: Dependency cleanup and operator-instructions update

**Files:**
- Modify: `ros2_ws/src/amr_home_manager/package.xml`
- Modify: `ros2_ws/src/amr_bringup/launch/explore_map.launch.py`

- [ ] **Step 1: Remove the now-unused `builtin_interfaces` dependency**

In `ros2_ws/src/amr_home_manager/package.xml`, remove this line (the `Duration`
import it backed was removed in Task 1; confirmed via
`grep -rn "builtin_interfaces" ros2_ws/src/amr_home_manager/amr_home_manager/*.py`
returns nothing after Task 1):
```xml
  <depend>builtin_interfaces</depend>
```

- [ ] **Step 2: Update the operator-instructions `LogInfo` in `explore_map.launch.py`**

Replace the final `TimerAction` block (the one with the `LogInfo` operator
instructions) with:
```python
        # ── Operator instructions after stack is up ────────────────────────────
        TimerAction(period=28.0, actions=[
            LogInfo(msg=(
                '\n\n'
                '══════════════════════════════════════════════════════\n'
                '  explore_map READY — robot is exploring autonomously\n'
                '══════════════════════════════════════════════════════\n'
                '  Monitor:  ros2 topic hz /cmd_vel\n'
                '            ros2 topic echo /map_metadata --once\n'
                '\n'
                '  If the robot reports being stuck (see this terminal for\n'
                '  "Robot stuck -- cannot make further progress"), send\n'
                '  either of:\n'
                "    ros2 topic pub /amr/command std_msgs/msg/String \\\n"
                "      \"data: 'stop'\" --once      # save map+path, stay put\n"
                "    ros2 topic pub /amr/command std_msgs/msg/String \\\n"
                "      \"data: 'go_home'\" --once   # save + retrace path home\n"
                '\n'
                '  Map saved to:  ~/AMR/maps/explore_map.pgm/.yaml\n'
                '  Path saved to: ~/AMR/maps/explore_map_path.json\n'
                '\n'
                '  To go back to where you stopped and resume mapping:\n'
                "    ros2 topic pub /amr/command std_msgs/msg/String \\\n"
                "      \"data: 'resume'\" --once\n"
                '══════════════════════════════════════════════════════\n'
            )),
        ]),
```

- [ ] **Step 3: Commit**

```bash
git add ros2_ws/src/amr_home_manager/package.xml ros2_ws/src/amr_bringup/launch/explore_map.launch.py
git commit -m "chore: drop unused builtin_interfaces dep, document stuck/resume commands in launch instructions"
```

---

## Task 9: Final full-suite verification

**Files:** none (verification only)

- [ ] **Step 1: Run the full `amr_home_manager` test suite**

```bash
cd ros2_ws/src/amr_home_manager && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/ -v
```
Expected: `45 passed`, with no warnings about removed symbols
(`_recovery_spins_attempted`, `_spin_client`, `_stall_nudge_sent`,
`_on_exploration_done`, `Spin`, `Duration`) remaining anywhere.

- [ ] **Step 2: Grep to confirm removed symbols are fully gone**

```bash
cd /home/m0mspagetthi/AMR && grep -rn "_recovery_spins_attempted\|_spin_client\|_stall_nudge_sent\|_on_exploration_done\|nav2_msgs.action import NavigateToPose, Spin\|builtin_interfaces" ros2_ws/src/amr_home_manager/
```
Expected: no output.

- [ ] **Step 3: Confirm the new `/amr/command` vocabulary is consistent**

```bash
grep -n "cmd ==" ros2_ws/src/amr_home_manager/amr_home_manager/home_manager_node.py
```
Expected: four branches — `'explore'`, `'stop'`, `'go_home'`, `'resume'`.

This task produces no new commit (verification only) — if either check finds a
problem, fix it and commit as `fix(home_manager): <description>`, re-running
Step 1 until it's clean.

---

## Task 10: Guard `go_home`/`resume` against reentrant retraces

**Background:** The final review of Tasks 1-9 confirmed a reentrancy gap: neither
the `go_home` nor `resume` branches in `_on_command` check whether a retrace is
already in progress (`State.RETURNING_HOME`/`State.RESUMING`) before calling
`_navigate_path` again. Re-issuing either command mid-retrace overwrites
`_waypoint_queue`/`_waypoint_done_cb` without cancelling the in-flight
`NavigateToPose` goal, corrupting the sequence. Separately, `resume` never calls
`_pause_explore()`, so if `_breakpoint_pose` is stale (set by an earlier
stop/resume cycle and not yet cleared by `_reset_path_recording`) and the robot is
still `EXPLORING`, a `resume` command starts a manual retrace while explore_lite is
still issuing its own nav goals.

**Files:**
- Modify: `ros2_ws/src/amr_home_manager/amr_home_manager/home_manager_node.py`
- Modify: `ros2_ws/src/amr_home_manager/test/test_home_manager.py`

- [ ] **Step 1: Add tests for the reentrancy guards**

Add these new tests at the end of the file:
```python
# ---- reentrancy guards: go_home/resume while a retrace is already in progress ----

def test_command_go_home_while_returning_home_is_noop():
    node = make_node()
    node._state = State.RETURNING_HOME
    node._home_pose = _FakePose()
    node._recorded_path = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)]
    msg = MagicMock()
    msg.data = "go_home"
    node._on_command(msg)
    assert node._state == State.RETURNING_HOME
    node._nav_client.send_goal_async.assert_not_called()


def test_command_go_home_while_resuming_is_noop():
    node = make_node()
    node._state = State.RESUMING
    node._home_pose = _FakePose()
    node._recorded_path = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)]
    msg = MagicMock()
    msg.data = "go_home"
    node._on_command(msg)
    assert node._state == State.RESUMING
    node._nav_client.send_goal_async.assert_not_called()


def test_command_resume_while_returning_home_is_noop():
    node = make_node()
    node._state = State.RETURNING_HOME
    node._home_pose = _FakePose(x=0.0, y=0.0, yaw=0.0)
    node._recorded_path = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)]
    node._breakpoint_pose = _FakePose(x=1.0, y=0.0, yaw=0.0)
    msg = MagicMock()
    msg.data = "resume"
    node._on_command(msg)
    assert node._state == State.RETURNING_HOME
    node._nav_client.send_goal_async.assert_not_called()


def test_command_resume_while_resuming_is_noop():
    node = make_node()
    node._state = State.RESUMING
    node._home_pose = _FakePose(x=0.0, y=0.0, yaw=0.0)
    node._recorded_path = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)]
    node._breakpoint_pose = _FakePose(x=1.0, y=0.0, yaw=0.0)
    msg = MagicMock()
    msg.data = "resume"
    node._on_command(msg)
    assert node._state == State.RESUMING
    node._nav_client.send_goal_async.assert_not_called()


def test_command_resume_pauses_explore_before_retracing():
    node = make_node()
    node._state = State.EXPLORING
    node._home_pose = _FakePose(x=0.0, y=0.0, yaw=0.0)
    node._recorded_path = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)]
    node._breakpoint_pose = _FakePose(x=1.0, y=0.0, yaw=0.0)
    node._nav_client.wait_for_server.return_value = True

    msg = MagicMock()
    msg.data = "resume"
    node._on_command(msg)

    assert node._state == State.RESUMING
    assert node._explore_pub.publish.call_args_list[0][0][0].data is False
```

- [ ] **Step 2: Run tests to verify the new tests fail**

```bash
cd ros2_ws/src/amr_home_manager && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/ -q
```
Expected: failures — `go_home`/`resume` have no state guard yet, and `resume`
doesn't call `_pause_explore()`.

- [ ] **Step 3: Add the guards to `go_home` and `resume`**

In `_on_command`, replace the `go_home` and `resume` branches:
```python
        elif cmd == 'go_home':
            if self._home_pose is None:
                self.get_logger().warn('No home pose recorded — cannot return home')
                return
            if self._state in (State.EXPLORING, State.STUCK):
                self._save_progress()
            self._state = State.RETURNING_HOME
            if len(self._recorded_path) <= 1:
                self._navigate_to_home()
            else:
                self._navigate_path(list(reversed(self._recorded_path)),
                                     self._on_returned_home)

        elif cmd == 'resume':
            if self._breakpoint_pose is None or len(self._recorded_path) <= 1:
                self.get_logger().warn('No saved breakpoint -- nothing to resume')
                return
            self._state = State.RESUMING
            self._navigate_path(list(self._recorded_path), self._on_resume_arrived)
```
with:
```python
        elif cmd == 'go_home':
            if self._home_pose is None:
                self.get_logger().warn('No home pose recorded — cannot return home')
                return
            if self._state in (State.RETURNING_HOME, State.RESUMING):
                self.get_logger().info(
                    f'Already {self._state.value} -- ignoring go_home')
                return
            if self._state in (State.EXPLORING, State.STUCK):
                self._save_progress()
            self._state = State.RETURNING_HOME
            if len(self._recorded_path) <= 1:
                self._navigate_to_home()
            else:
                self._navigate_path(list(reversed(self._recorded_path)),
                                     self._on_returned_home)

        elif cmd == 'resume':
            if self._state in (State.RETURNING_HOME, State.RESUMING):
                self.get_logger().info(
                    f'Already {self._state.value} -- ignoring resume')
                return
            if self._breakpoint_pose is None or len(self._recorded_path) <= 1:
                self.get_logger().warn('No saved breakpoint -- nothing to resume')
                return
            self._pause_explore()
            self._state = State.RESUMING
            self._navigate_path(list(self._recorded_path), self._on_resume_arrived)
```

- [ ] **Step 4: Run tests to verify they pass**

```bash
cd ros2_ws/src/amr_home_manager && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/ -q
```
Expected: `50 passed`.

- [ ] **Step 5: Commit**

```bash
git add ros2_ws/src/amr_home_manager/amr_home_manager/home_manager_node.py ros2_ws/src/amr_home_manager/test/test_home_manager.py
git commit -m "fix(home_manager): guard go_home/resume against reentrant retraces"
```
