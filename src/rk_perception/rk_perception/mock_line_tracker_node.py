#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from rk_interfaces.msg import LineTrack


class MockLineTrackerNode(Node):

    def __init__(self):
        super().__init__('mock_line_tracker_node')

        self.publisher_ = self.create_publisher(
            LineTrack,
            '/perception/line_track',
            10
        )

        self.timer = self.create_timer(
            0.5,
            self.publish_mock_data
        )

        self.get_logger().info(
            'Mock Line Tracker Node Started'
        )

    def publish_mock_data(self):

        msg = LineTrack()

        msg.lateral_error = 0.05
        msg.heading_error = 0.02
        msg.confidence = 0.95
        msg.line_visible = True

        self.publisher_.publish(msg)

        self.get_logger().info(
            f'Publishing line track: '
            f'lateral={msg.lateral_error:.2f}, '
            f'heading={msg.heading_error:.2f}'
        )


def main(args=None):

    rclpy.init(args=args)

    node = MockLineTrackerNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()