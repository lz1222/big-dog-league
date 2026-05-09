#!/usr/bin/env python3

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from rk_interfaces.msg import SignDetection, SignDetectionArray


class MockSignDetectorNode(Node):
    """Publish mock sign detections used by the mission state machine."""

    def __init__(self):
        super().__init__('mock_sign_detector_node')
        self.publisher = self.create_publisher(
            SignDetectionArray,
            '/perception/sign_detections',
            10
        )
        self.timer = self.create_timer(1.0, self.publish_mock_data)
        self.sample_index = 0
        self.get_logger().info('Mock sign detector node started')

    def publish_mock_data(self):
        msg = SignDetectionArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'd435i_color_optical_frame'

        signs = [
            ('direction', 'forward'),
            ('warning', 'caution'),
            ('task', 'pick_item'),
        ]
        sign_type, sign_value = signs[self.sample_index % len(signs)]

        detection = SignDetection()
        detection.header = msg.header
        detection.sign_type = sign_type
        detection.sign_value = sign_value
        detection.confidence = 0.92
        msg.detections.append(detection)

        self.publisher.publish(msg)
        self.sample_index += 1
        self.get_logger().info(
            f'Publishing sign detection: {sign_type}={sign_value}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = MockSignDetectorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
