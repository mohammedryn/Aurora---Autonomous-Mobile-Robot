#!/usr/bin/env python3
"""
Relays /cmd_vel_safe (TwistStamped) → /mecanum_drive_controller/reference (TwistStamped).

topic_tools relay fails silently in Jazzy with the chainable mecanum_drive_controller
because it uses a GenericPublisher whose QoS doesn't match the controller's subscriber.
This rclpy node uses an explicit typed publisher that the controller accepts reliably.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped


class CmdVelSafeRelay(Node):
    def __init__(self):
        super().__init__('cmd_vel_safe_relay')
        self._pub = self.create_publisher(
            TwistStamped, '/mecanum_drive_controller/reference', 10)
        self.create_subscription(
            TwistStamped, '/cmd_vel_safe', self._cb, 10)
        self.get_logger().info('/cmd_vel_safe → /mecanum_drive_controller/reference relay active')

    def _cb(self, msg: TwistStamped):
        # Refresh timestamp so controller doesn't reject stale commands
        msg.header.stamp = self.get_clock().now().to_msg()
        self._pub.publish(msg)


def main():
    rclpy.init()
    node = CmdVelSafeRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
