#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from rk_interfaces.msg import LineTrack


class MockLineTrackerNode(Node):
    """Publish a stable mock line tracking signal for stage-one tests."""

    def __init__(self):
        super().__init__('mock_line_tracker_node')

        self.publisher = self.create_publisher(
            LineTrack,
            '/perception/line_track',
            10
        )
        self.timer = self.create_timer(0.5, self.publish_mock_data)
        self.sample_index = 0

        self.get_logger().info('Mock line tracker node started')

    def publish_mock_data(self):
        msg = LineTrack()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'd435i_color_optical_frame'

        pattern = [-0.04, -0.02, 0.0, 0.02, 0.04, 0.03, 0.0]
        msg.lateral_error = pattern[self.sample_index % len(pattern)]
        msg.heading_error = 0.02
        msg.confidence = 0.95
        msg.line_visible = True

        self.publisher.publish(msg)
        self.sample_index += 1

        self.get_logger().info(
            'Publishing line track: '
            f'lateral={msg.lateral_error:.2f}, '
            f'heading={msg.heading_error:.2f}, '
            f'confidence={msg.confidence:.2f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = MockLineTrackerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
