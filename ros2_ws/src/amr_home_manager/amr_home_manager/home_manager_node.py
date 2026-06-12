import enum
import json
import math
import os
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from explore_lite_msgs.msg import ExploreStatus
from slam_toolbox.srv import SaveMap


class State(enum.Enum):
    IDLE = "idle"
    EXPLORING = "exploring"
    STUCK = "stuck"
    RETURNING_HOME = "returning_home"
    RESUMING = "resuming"


class HomeManagerNode(Node):
    def __init__(self):
        super().__init__('amr_home_manager')

        self._state = State.IDLE
        self._home_pose: PoseStamped | None = None

        # Stall watchdog: explore_lite has repeatedly called "No frontiers
        # found, stopping" within ~10s of the first goal -- a transient empty
        # frontier-search result (observed to coincide with global costmap
        # resize events) that it treats as terminal, even with most of the
        # room still unmapped. Rather than patch the third-party node, detect
        # "robot gone fully still while we're supposed to be exploring" from
        # actual /odom motion and re-publish /explore/resume to make it search
        # again -- turning a permanent give-up into a brief pause-and-retry.
        self._last_motion_time = self.get_clock().now()
        self._last_nudge_time = self.get_clock().now()
        self._stall_nudge_count = 0
        self._stall_timeout_s = 15.0
        self._motion_threshold = 0.02
        self._max_stall_retries = 15
        self._stuck_prompt_delay_s = 3.0
        self._stuck_pending = False

        # Path recording: sampled (x, y, yaw) trail of the robot's travel
        # while EXPLORING/STUCK, used by later resume/go-home retrace logic.
        self._recorded_path: list[tuple[float, float, float]] = []
        self._breakpoint_pose = None
        self._path_sample_distance_m = 0.5

        self.declare_parameter('map_save_path',
                               os.path.expanduser('~/AMR/maps/explore_map'))

        # Command subscriber: accepts "explore", "go_home", "stop"
        self.create_subscription(String, '/amr/command', self._on_command, 10)

        # Odom subscriber — record home on first message
        self._odom_sub = self.create_subscription(
            Odometry, '/odom', self._on_odom, 10)

        # Publish True/False to start/stop explore_lite
        self._explore_pub = self.create_publisher(Bool, '/explore/resume', 10)

        # explore_lite status — drives the recovery-spin logic above
        self.create_subscription(
            ExploreStatus, '/explore/status', self._on_explore_status, 10)

        # slam_toolbox service client — saves .pgm + .yaml when exploration stops
        self._save_map_client = self.create_client(
            SaveMap, '/slam_toolbox/save_map')

        # Nav2 action clients
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.create_timer(5.0, self._check_stall)

        self.get_logger().info('HomeManagerNode ready. State: IDLE')

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

    def _on_odom(self, msg: Odometry) -> None:
        if self._home_pose is None:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose = msg.pose.pose
            self._home_pose = pose
            self.get_logger().info(
                f'Home pose recorded: ({msg.pose.pose.position.x:.2f}, '
                f'{msg.pose.pose.position.y:.2f})')

        # Subscription stays alive (no self-destroy) -- the watchdog needs a
        # continuous feed of actual robot motion, not just the first sample.
        v = msg.twist.twist
        moving = (abs(v.linear.x) > self._motion_threshold
                  or abs(v.linear.y) > self._motion_threshold
                  or abs(v.angular.z) > self._motion_threshold)
        if moving:
            self._last_motion_time = self.get_clock().now()
            self._last_nudge_time = self.get_clock().now()
            self._stall_nudge_count = 0
            self._stuck_pending = False

        if self._state in (State.EXPLORING, State.STUCK):
            self._record_path_point(msg.pose.pose)

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

    def _on_command(self, msg: String) -> None:
        cmd = msg.data.strip().lower()
        self.get_logger().info(f'Command received: {cmd} (state={self._state.value})')

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

        elif cmd == 'stop':
            if self._state in (State.EXPLORING, State.STUCK):
                self._save_progress()
                self._state = State.IDLE
                self.get_logger().info('Exploration stopped. Progress saved.')
            else:
                self.get_logger().info('Already idle -- nothing to stop')

        elif cmd == 'go_home':
            if self._home_pose is None:
                self.get_logger().warn('No home pose recorded — cannot return home')
                return
            self._state = State.RETURNING_HOME
            self._navigate_to_home()

    def _on_explore_status(self, msg: ExploreStatus) -> None:
        # explore_lite starts exploring on its own as soon as it launches --
        # explore_map.launch.py never publishes /amr/command "explore", so
        # _state would otherwise stay IDLE forever. Track explore_lite's own
        # start/resume announcements instead.
        if msg.status in (ExploreStatus.EXPLORATION_STARTED,
                          ExploreStatus.EXPLORATION_IN_PROGRESS):
            if self._state == State.IDLE:
                self._state = State.EXPLORING
                self._reset_path_recording()
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

    def _resume_explore(self) -> None:
        resume = Bool()
        resume.data = True
        self._explore_pub.publish(resume)

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

    def _trigger_map_save(self) -> None:
        if not self._save_map_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error(
                '/slam_toolbox/save_map service not available — map NOT saved')
            return
        req = SaveMap.Request()
        req.name.data = self.get_parameter(
            'map_save_path').get_parameter_value().string_value
        future = self._save_map_client.call_async(req)
        future.add_done_callback(self._on_save_map_done)

    def _on_save_map_done(self, future) -> None:
        try:
            result = future.result()
            if result.result == 0:
                path = self.get_parameter(
                    'map_save_path').get_parameter_value().string_value
                self.get_logger().info(f'Map saved: {path}.pgm / {path}.yaml')
            else:
                self.get_logger().error(
                    f'slam_toolbox/save_map returned error code {result.result}')
        except Exception as e:
            self.get_logger().error(f'Map save service call failed: {e}')

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
