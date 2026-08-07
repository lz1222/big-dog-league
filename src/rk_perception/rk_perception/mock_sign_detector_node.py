#!/usr/bin/env python3

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from rk_interfaces.msg import SignDetection, SignDetectionArray


class MockSignDetectorNode(Node):
    """Publish mock sign detections used by the mission state machine."""

    def __init__(self):
        super().__init__('mock_sign_detector_node')
        self.declare_parameter('place_marker_value', 'place_1')
        self.declare_parameter('warning_value', 'electric_shock')

        self.publisher = self.create_publisher(
            SignDetectionArray,
            '/perception/sign_detections',
            10
        )
        self.timer = self.create_timer(1.0, self.publish_mock_data)
        self.get_logger().info('Mock sign detector node started')

    def publish_mock_data(self):
        msg = SignDetectionArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'd435i_color_optical_frame'

        place_marker = self._make_detection(
            msg,
            'place_marker',
            str(self.get_parameter('place_marker_value').value)
        )
        warning = self._make_detection(
            msg,
            'warning',
            str(self.get_parameter('warning_value').value)
        )
        msg.detections.extend([place_marker, warning])

        self.publisher.publish(msg)
        self.get_logger().info(
            'Publishing sign detections: '
            f'place_marker={place_marker.sign_value}, '
            f'warning={warning.sign_value}'
        )

    @staticmethod
    def _make_detection(msg, sign_type, sign_value):
        detection = SignDetection()
        detection.header = msg.header
        detection.sign_type = sign_type
        detection.sign_value = sign_value
        detection.confidence = 0.92
        return detection


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
