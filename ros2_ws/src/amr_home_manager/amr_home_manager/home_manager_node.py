import enum
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose


class State(enum.Enum):
    IDLE = "idle"
    EXPLORING = "exploring"
    RETURNING_HOME = "returning_home"


class HomeManagerNode(Node):
    def __init__(self):
        super().__init__('amr_home_manager')

        self._state = State.IDLE
        self._home_pose: PoseStamped | None = None

        # Command subscriber: accepts "explore", "go_home", "stop"
        self.create_subscription(String, '/amr/command', self._on_command, 10)

        # Odom subscriber — record home on first message
        self._odom_sub = self.create_subscription(
            Odometry, '/odom', self._on_odom, 10)

        # Publish True/False to start/stop explore_lite
        self._explore_pub = self.create_publisher(Bool, '/explore/resume', 10)

        # Trigger map save
        self._map_saver_pub = self.create_publisher(
            String, '/amr/save_map', 10)

        # Nav2 action client
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.get_logger().info('HomeManagerNode ready. State: IDLE')

    def _on_odom(self, msg: Odometry) -> None:
        if self._home_pose is None:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose = msg.pose.pose
            self._home_pose = pose
            self.get_logger().info(
                f'Home pose recorded: ({msg.pose.pose.position.x:.2f}, '
                f'{msg.pose.pose.position.y:.2f})')
        # Only need first message to record home
        self.destroy_subscription(self._odom_sub)

    def _on_command(self, msg: String) -> None:
        cmd = msg.data.strip().lower()
        self.get_logger().info(f'Command received: {cmd} (state={self._state.value})')

        if cmd == 'explore':
            if self._home_pose is None:
                self.get_logger().warn('Home pose not yet recorded — waiting for /odom')
                return
            self._state = State.EXPLORING
            resume = Bool()
            resume.data = True
            self._explore_pub.publish(resume)
            self.get_logger().info('Exploration started')

        elif cmd == 'stop':
            if self._state == State.EXPLORING:
                self._on_exploration_done()

        elif cmd == 'go_home':
            if self._home_pose is None:
                self.get_logger().warn('No home pose recorded — cannot return home')
                return
            self._state = State.RETURNING_HOME
            self._navigate_to_home()

    def _on_exploration_done(self) -> None:
        stop = Bool()
        stop.data = False
        self._explore_pub.publish(stop)
        save = String()
        save.data = 'save'
        self._map_saver_pub.publish(save)
        self._state = State.IDLE
        self.get_logger().info('Exploration complete. Map save triggered.')

    def _navigate_to_home(self) -> None:
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('NavigateToPose action server not available')
            self._state = State.IDLE
            return
        goal = NavigateToPose.Goal()
        goal.pose = self._home_pose
        self.get_logger().info('Sending navigate_to_pose goal (home)')
        send_future = self._nav_client.send_goal_async(goal)
        send_future.add_done_callback(self._on_nav_goal_accepted)

    def _on_nav_goal_accepted(self, future) -> None:
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Navigation goal rejected')
            self._state = State.IDLE
            return
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._on_nav_done)

    def _on_nav_done(self, future) -> None:
        self._state = State.IDLE
        self.get_logger().info('Returned home. State: IDLE')


def main(args=None):
    rclpy.init(args=args)
    node = HomeManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
