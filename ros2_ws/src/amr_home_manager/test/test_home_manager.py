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
    def destroy_subscription(self, *a, **kw): pass


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

from amr_home_manager.home_manager_node import HomeManagerNode, State


def make_node():
    node = object.__new__(HomeManagerNode)
    node._state = State.IDLE
    node._home_pose = None
    node._odom_sub = MagicMock()
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
