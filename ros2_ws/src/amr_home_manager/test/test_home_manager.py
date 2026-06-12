import pytest
import json
import math
from unittest.mock import MagicMock
import sys


# ---- ROS2 stubs ----
# Use a real Python class for Node (MagicMock-as-base-class causes metaclass
# conflicts when calling object.__new__ on the subclass).

class _FakeNode:
    """Minimal Node stub that supports subclassing."""
    def __init__(self, *a, **kw): pass
    def get_logger(self): return MagicMock()
    def create_subscription(self, *a, **kw): return MagicMock()
    def create_publisher(self, *a, **kw): return MagicMock()
    def create_client(self, *a, **kw): return MagicMock()
    def create_timer(self, *a, **kw): return MagicMock()
    def declare_parameter(self, *a, **kw): return MagicMock()
    def get_parameter(self, *a, **kw): return MagicMock()
    def get_clock(self): return MagicMock()
    def destroy_subscription(self, *a, **kw): pass


# ---- fake clock: lets stall-watchdog tests control elapsed time exactly,
# rather than fighting MagicMock's lack of numeric/subtraction support ----

class _FakeDuration:
    def __init__(self, nanoseconds):
        self.nanoseconds = nanoseconds


class _FakeTime:
    def __init__(self, seconds):
        self.seconds = seconds

    def __sub__(self, other):
        return _FakeDuration((self.seconds - other.seconds) * 1e9)


class _FakeClock:
    def __init__(self):
        self.seconds = 0.0

    def now(self):
        return _FakeTime(self.seconds)


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


_rclpy_node_mod = MagicMock()
_rclpy_node_mod.Node = _FakeNode

sys.modules['rclpy'] = MagicMock()
sys.modules['rclpy.node'] = _rclpy_node_mod
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
sys.modules['slam_toolbox'] = MagicMock()
sys.modules['slam_toolbox.srv'] = MagicMock()
sys.modules['builtin_interfaces'] = MagicMock()
sys.modules['builtin_interfaces.msg'] = MagicMock()
sys.modules['explore_lite_msgs'] = MagicMock()
sys.modules['explore_lite_msgs.msg'] = MagicMock()

from amr_home_manager.home_manager_node import HomeManagerNode, State, ExploreStatus


def make_node():
    node = object.__new__(HomeManagerNode)
    node._state = State.IDLE
    node._home_pose = None
    node._odom_sub = MagicMock()
    node._logger = MagicMock()
    node._nav_client = MagicMock()
    node._explore_pub = MagicMock()
    node._save_map_client = MagicMock()
    # Stall watchdog state -- object.__new__ skips __init__, so set these
    # manually. _clock is a _FakeClock so tests can move time forward by
    # writing node._clock.seconds directly (MagicMock doesn't support the
    # numeric subtraction _check_stall/_on_odom rely on).
    node._clock = _FakeClock()
    node.get_clock = lambda: node._clock
    node._last_motion_time = node._clock.now()
    node._last_nudge_time = node._clock.now()
    node._stall_nudge_count = 0
    node._stall_timeout_s = 15.0
    node._motion_threshold = 0.02
    node._max_stall_retries = 15
    node._stuck_prompt_delay_s = 3.0
    node._stuck_pending = False
    node._recorded_path = []
    node._breakpoint_pose = None
    node._path_sample_distance_m = 0.5
    return node


def _set_twist(odom_msg, x=0.0, y=0.0, z=0.0):
    odom_msg.twist.twist.linear.x = x
    odom_msg.twist.twist.linear.y = y
    odom_msg.twist.twist.angular.z = z


def _set_pose(odom_msg, x=0.0, y=0.0, yaw=0.0):
    odom_msg.pose.pose.position.x = x
    odom_msg.pose.pose.position.y = y
    odom_msg.pose.pose.orientation.x = 0.0
    odom_msg.pose.pose.orientation.y = 0.0
    odom_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
    odom_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)


def test_initial_state_is_idle():
    node = make_node()
    assert node._state == State.IDLE


def test_command_explore_transitions_to_exploring():
    node = make_node()
    node._home_pose = _FakePose()
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
    _set_twist(odom_msg)
    node._on_odom(odom_msg)
    assert node._home_pose is not None


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


def test_check_stall_ignores_non_exploring_state():
    node = make_node()
    node._state = State.IDLE
    node._clock.seconds = 20.0
    node._check_stall()
    node._explore_pub.publish.assert_not_called()


def test_check_stall_does_nothing_before_timeout():
    node = make_node()
    node._state = State.EXPLORING
    node._clock.seconds = 5.0
    node._check_stall()
    node._explore_pub.publish.assert_not_called()


# ---- explore status handling: explore_lite "No frontiers found" ----

def _explore_status(status):
    msg = MagicMock()
    msg.status = status
    return msg


def test_explore_status_complete_ignored_when_not_exploring():
    node = make_node()
    node._state = State.IDLE
    node._on_explore_status(_explore_status(ExploreStatus.EXPLORATION_COMPLETE))
    node._explore_pub.publish.assert_not_called()


def test_explore_status_in_progress_is_ignored():
    node = make_node()
    node._state = State.EXPLORING
    node._on_explore_status(_explore_status(ExploreStatus.EXPLORATION_IN_PROGRESS))
    node._explore_pub.publish.assert_not_called()


# ---- explore_lite auto-starts: track its status to reach EXPLORING ----

def test_explore_status_started_transitions_idle_to_exploring():
    node = make_node()
    node._state = State.IDLE
    node._on_explore_status(_explore_status(ExploreStatus.EXPLORATION_STARTED))
    assert node._state == State.EXPLORING


def test_explore_status_in_progress_transitions_idle_to_exploring():
    node = make_node()
    node._state = State.IDLE
    node._on_explore_status(_explore_status(ExploreStatus.EXPLORATION_IN_PROGRESS))
    assert node._state == State.EXPLORING


def test_explore_status_started_does_not_override_returning_home():
    node = make_node()
    node._state = State.RETURNING_HOME
    node._on_explore_status(_explore_status(ExploreStatus.EXPLORATION_STARTED))
    assert node._state == State.RETURNING_HOME


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


# ---- stall escalation to STUCK ----

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
