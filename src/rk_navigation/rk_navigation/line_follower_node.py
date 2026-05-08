#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from rk_interfaces.msg import LineTrack


class LineFollowerNode(Node):
    """Convert mock line tracking data into a simple velocity command."""

    def __init__(self):
        super().__init__('line_follower_node')
        self.confidence_threshold = 0.50
        self.forward_speed = 0.20
        self.angular_gain = -1.20
        self.max_angular_speed = 0.60

        self.publisher = self.create_publisher(
            Twist,
            '/navigation/cmd_vel',
            10
        )
        self.subscription = self.create_subscription(
            LineTrack,
            '/perception/line_track',
            self.on_line_track,
            10
        )
        self.get_logger().info('Line follower node started')

    def on_line_track(self, msg):
        cmd = Twist()

        if not msg.line_visible or msg.confidence < self.confidence_threshold:
            self.publisher.publish(cmd)
            self.get_logger().warn('Line lost or confidence low, stopping')
            return

        angular = self.angular_gain * msg.lateral_error
        angular = max(-self.max_angular_speed, min(self.max_angular_speed, angular))

        cmd.linear.x = self.forward_speed
        cmd.angular.z = angular
        self.publisher.publish(cmd)

        self.get_logger().info(
            f'Publishing cmd_vel: linear.x={cmd.linear.x:.2f}, '
            f'angular.z={cmd.angular.z:.2f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = LineFollowerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
