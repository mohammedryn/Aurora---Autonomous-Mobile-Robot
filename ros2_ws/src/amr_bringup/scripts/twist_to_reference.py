#!/usr/bin/env python3
"""
Converts /cmd_vel (Twist) → /mecanum_drive_controller/reference (TwistStamped).

In Jazzy ros2_controllers 4.x, the mecanum_drive_controller's command interface
is the 'reference' topic and only accepts TwistStamped.  This bridge lets all
upstream publishers (Nav2, manual tests) continue to use plain Twist on /cmd_vel.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped


class TwistToReference(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_reference')
        self._pub = self.create_publisher(
            TwistStamped, '/mecanum_drive_controller/reference', 10)
        self.create_subscription(Twist, '/cmd_vel', self._cb, 10)
        self.get_logger().info('/cmd_vel → /mecanum_drive_controller/reference bridge active')

    def _cb(self, msg: Twist):
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'base_link'
        out.twist = msg
        self._pub.publish(out)


def main():
    rclpy.init()
    node = TwistToReference()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
