#!/usr/bin/env python3
"""
Relays /cmd_vel_safe (Twist) → /mecanum_drive_controller/reference (TwistStamped).

Nav2 Jazzy collision_monitor publishes /cmd_vel_safe as unstamped Twist, but the
chainable mecanum_drive_controller reference topic requires TwistStamped. This node
converts Twist → TwistStamped with a fresh timestamp.

topic_tools relay fails silently in Jazzy with the chainable mecanum_drive_controller
because it uses a GenericPublisher whose QoS doesn't match the controller's subscriber.
This rclpy node uses an explicit typed publisher that the controller accepts reliably.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped


class CmdVelSafeRelay(Node):
    def __init__(self):
        super().__init__('cmd_vel_safe_relay')
        self._pub = self.create_publisher(
            TwistStamped, '/mecanum_drive_controller/reference', 10)
        self.create_subscription(
            Twist, '/cmd_vel_safe', self._cb, 10)
        self.get_logger().info('/cmd_vel_safe → /mecanum_drive_controller/reference relay active')

    def _cb(self, msg: Twist):
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'base_link'
        out.twist = msg
        self._pub.publish(out)


def main():
    rclpy.init()
    node = CmdVelSafeRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
