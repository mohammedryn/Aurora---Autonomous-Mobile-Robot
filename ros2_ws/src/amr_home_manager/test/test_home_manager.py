import pytest
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

from amr_home_manager.home_manager_node import HomeManagerNode, State


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
    node._stall_nudge_sent = False
    node._stall_timeout_s = 15.0
    node._motion_threshold = 0.02
    return node


def _set_twist(odom_msg, x=0.0, y=0.0, z=0.0):
    odom_msg.twist.twist.linear.x = x
    odom_msg.twist.twist.linear.y = y
    odom_msg.twist.twist.angular.z = z


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
    _set_twist(odom_msg)
    node._on_odom(odom_msg)
    assert node._home_pose is not None


def test_on_odom_moving_resets_stall_clock_and_flag():
    node = make_node()
    node._stall_nudge_sent = True
    node._clock.seconds = 100.0
    odom_msg = MagicMock()
    odom_msg.pose.pose.position.x = 0.0
    odom_msg.pose.pose.position.y = 0.0
    _set_twist(odom_msg, x=0.1)
    node._on_odom(odom_msg)
    assert node._last_motion_time.seconds == 100.0
    assert node._stall_nudge_sent is False


def test_on_odom_still_does_not_touch_stall_clock():
    node = make_node()
    node._stall_nudge_sent = True
    node._last_motion_time = node._clock.now()
    node._clock.seconds = 100.0
    odom_msg = MagicMock()
    odom_msg.pose.pose.position.x = 0.0
    odom_msg.pose.pose.position.y = 0.0
    _set_twist(odom_msg)
    node._on_odom(odom_msg)
    assert node._last_motion_time.seconds == 0.0
    assert node._stall_nudge_sent is True


def test_check_stall_nudges_resume_after_timeout_while_exploring():
    node = make_node()
    node._state = State.EXPLORING
    node._clock.seconds = 20.0
    node._check_stall()
    node._explore_pub.publish.assert_called_once()
    published = node._explore_pub.publish.call_args[0][0]
    assert published.data is True
    assert node._stall_nudge_sent is True


def test_check_stall_does_not_nudge_again_before_motion_resumes():
    node = make_node()
    node._state = State.EXPLORING
    node._clock.seconds = 20.0
    node._check_stall()
    node._clock.seconds = 40.0
    node._check_stall()
    node._explore_pub.publish.assert_called_once()


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


def test_exploration_done_calls_save_map_service_and_transitions():
    node = make_node()
    node._state = State.EXPLORING
    # service is available
    node._save_map_client.wait_for_service.return_value = True
    node._save_map_client.call_async.return_value = MagicMock()
    node._on_exploration_done()
    node._save_map_client.wait_for_service.assert_called_once()
    node._save_map_client.call_async.assert_called_once()
    assert node._state == State.IDLE
