#!/usr/bin/env python3

import math

import cv2
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image

from rk_interfaces.msg import LineTrack


class RealLineTrackerNode(Node):
    """Track a black floor line from a RealSense RGB image."""

    def __init__(self):
        super().__init__('real_line_tracker_node')

        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('line_track_topic', '/perception/line_track')
        self.declare_parameter('roi_top_fraction', 0.5)
        self.declare_parameter('threshold_value', 80)
        self.declare_parameter('min_contour_area', 300.0)
        self.declare_parameter('frame_id', 'd435i_color_optical_frame')
        self.declare_parameter('debug_log', False)

        self.image_topic = self.get_parameter(
            'image_topic'
        ).get_parameter_value().string_value
        self.line_track_topic = self.get_parameter(
            'line_track_topic'
        ).get_parameter_value().string_value
        self.roi_top_fraction = self.get_parameter(
            'roi_top_fraction'
        ).get_parameter_value().double_value
        self.threshold_value = self.get_parameter(
            'threshold_value'
        ).get_parameter_value().integer_value
        self.min_contour_area = self.get_parameter(
            'min_contour_area'
        ).get_parameter_value().double_value
        self.frame_id = self.get_parameter(
            'frame_id'
        ).get_parameter_value().string_value
        self.debug_log = self.get_parameter(
            'debug_log'
        ).get_parameter_value().bool_value

        self.roi_top_fraction = max(0.0, min(0.95, self.roi_top_fraction))
        self.threshold_value = max(0, min(255, self.threshold_value))
        self.bridge = CvBridge()

        self.publisher = self.create_publisher(
            LineTrack,
            self.line_track_topic,
            10
        )
        self.subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.on_image,
            10
        )

        self.get_logger().info(
            'Real line tracker node started: '
            f'image_topic={self.image_topic}, '
            f'line_track_topic={self.line_track_topic}'
        )

    def on_image(self, image_msg):
        try:
            image = self.bridge.imgmsg_to_cv2(
                image_msg,
                desired_encoding='bgr8'
            )
        except CvBridgeError as exc:
            self.get_logger().warn(f'Failed to convert image: {exc}')
            self.publish_line_lost(image_msg)
            return

        try:
            height, width = image.shape[:2]
            roi_start_y = int(height * self.roi_top_fraction)
            roi = image[roi_start_y:height, :]

            if roi.size == 0:
                self.get_logger().warn('Image ROI is empty')
                self.publish_line_lost(image_msg)
                return

            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            _, binary = cv2.threshold(
                gray,
                self.threshold_value,
                255,
                cv2.THRESH_BINARY_INV
            )

            contours, _ = cv2.findContours(
                binary,
                cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE
            )

            if not contours:
                self.publish_line_lost(image_msg)
                return

            contour = max(contours, key=cv2.contourArea)
            contour_area = float(cv2.contourArea(contour))

            if contour_area < self.min_contour_area:
                self.publish_line_lost(image_msg, contour_area=contour_area)
                return

            moments = cv2.moments(contour)
            if moments['m00'] == 0.0:
                self.publish_line_lost(image_msg, contour_area=contour_area)
                return

            centroid_x = moments['m10'] / moments['m00']
            image_center_x = width / 2.0
            lateral_error = (centroid_x - image_center_x) / image_center_x
            lateral_error = max(-1.0, min(1.0, lateral_error))

            heading_error = self.compute_heading_error(contour)
            confidence = self.compute_confidence(contour_area, roi.shape[:2])

            msg = LineTrack()
            msg.header.stamp = image_msg.header.stamp
            msg.header.frame_id = image_msg.header.frame_id or self.frame_id
            msg.lateral_error = float(lateral_error)
            msg.heading_error = float(heading_error)
            msg.confidence = float(confidence)
            msg.line_visible = True
            self.publisher.publish(msg)

            if self.debug_log:
                self.get_logger().info(
                    'line debug: '
                    f'contour_area={contour_area:.1f}, '
                    f'lateral_error={msg.lateral_error:.3f}, '
                    f'heading_error={msg.heading_error:.3f}, '
                    f'confidence={msg.confidence:.3f}'
                )
        except Exception as exc:
            self.get_logger().warn(f'Line tracking failed: {exc}')
            self.publish_line_lost(image_msg)

    def compute_heading_error(self, contour):
        if len(contour) < 2:
            return 0.0

        try:
            line = cv2.fitLine(
                contour,
                cv2.DIST_L2,
                0,
                0.01,
                0.01
            )
            vx, vy, _, _ = [float(v) for v in line.reshape(-1)]
        except Exception:
            return 0.0

        if abs(vy) < 1.0e-6:
            return math.copysign(math.pi / 2.0, vx)

        heading_error = math.atan2(vx, vy)
        return max(-math.pi / 2.0, min(math.pi / 2.0, heading_error))

    def compute_confidence(self, contour_area, roi_shape):
        roi_height, roi_width = roi_shape
        roi_area = float(roi_height * roi_width)
        if roi_area <= 0.0:
            return 0.0

        area_ratio = contour_area / roi_area
        return max(0.0, min(1.0, area_ratio * 25.0))

    def publish_line_lost(self, image_msg, contour_area=0.0):
        msg = LineTrack()
        msg.header.stamp = image_msg.header.stamp
        msg.header.frame_id = image_msg.header.frame_id or self.frame_id
        msg.lateral_error = 0.0
        msg.heading_error = 0.0
        msg.confidence = 0.0
        msg.line_visible = False
        self.publisher.publish(msg)

        if self.debug_log:
            self.get_logger().info(
                'line debug: '
                f'contour_area={contour_area:.1f}, '
                'lateral_error=0.000, '
                'heading_error=0.000, '
                'confidence=0.000'
            )


def main(args=None):
    rclpy.init(args=args)
    node = RealLineTrackerNode()
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
