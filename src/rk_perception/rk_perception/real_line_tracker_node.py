#!/usr/bin/env python3

import math

import cv2
import numpy as np
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

        self.declare_parameter(
            'image_topic',
            '/camera/camera/color/image_raw'
        )
        self.declare_parameter('line_track_topic', '/perception/line_track')
        self.declare_parameter('enable_debug_image', False)
        self.declare_parameter('debug_log', False)
        self.declare_parameter('roi_top_fraction', 0.5)
        self.declare_parameter('threshold_value', 80)
        self.declare_parameter('min_contour_area', 300.0)
        self.declare_parameter('max_lateral_error', 1.0)
        self.declare_parameter('min_confidence_area', 1500.0)
        self.declare_parameter('frame_id', 'd435i_color_optical_frame')

        self.image_topic = self.get_parameter(
            'image_topic'
        ).get_parameter_value().string_value
        self.line_track_topic = self.get_parameter(
            'line_track_topic'
        ).get_parameter_value().string_value
        self.frame_id = self.get_parameter(
            'frame_id'
        ).get_parameter_value().string_value

        self.bridge = CvBridge()
        self.last_debug_log_ns = 0
        self.debug_log_period_ns = 1_000_000_000
        self.refresh_parameters()

        self.publisher = self.create_publisher(
            LineTrack,
            self.line_track_topic,
            10
        )
        self.mask_publisher = self.create_publisher(
            Image,
            '/perception/debug/line_mask',
            10
        )
        self.overlay_publisher = self.create_publisher(
            Image,
            '/perception/debug/line_overlay',
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

    def refresh_parameters(self):
        """Refresh runtime-tunable parameters from the ROS parameter store."""
        self.enable_debug_image = self.get_parameter(
            'enable_debug_image'
        ).get_parameter_value().bool_value
        self.debug_log = self.get_parameter(
            'debug_log'
        ).get_parameter_value().bool_value
        self.roi_top_fraction = self.get_parameter(
            'roi_top_fraction'
        ).get_parameter_value().double_value
        self.threshold_value = self.get_parameter(
            'threshold_value'
        ).get_parameter_value().integer_value
        self.min_contour_area = self.get_parameter(
            'min_contour_area'
        ).get_parameter_value().double_value
        self.max_lateral_error = self.get_parameter(
            'max_lateral_error'
        ).get_parameter_value().double_value
        self.min_confidence_area = self.get_parameter(
            'min_confidence_area'
        ).get_parameter_value().double_value

        self.roi_top_fraction = self.clamp(self.roi_top_fraction, 0.0, 0.95)
        self.threshold_value = int(self.clamp(self.threshold_value, 0, 255))
        self.min_contour_area = max(0.0, self.min_contour_area)
        self.max_lateral_error = max(0.01, self.max_lateral_error)
        self.min_confidence_area = max(1.0, self.min_confidence_area)

    def on_image(self, image_msg):
        self.refresh_parameters()

        try:
            image = self.bridge.imgmsg_to_cv2(
                image_msg,
                desired_encoding='bgr8'
            )
        except CvBridgeError as exc:
            self.get_logger().warn(f'Failed to convert image: {exc}')
            self.publish_line_lost(image_msg)
            return
        except Exception as exc:
            self.get_logger().warn(f'Unexpected image conversion error: {exc}')
            self.publish_line_lost(image_msg)
            return

        binary = None
        overlay = None
        roi_start_y = 0
        contour_area = 0.0

        try:
            height, width = image.shape[:2]
            roi_start_y = int(height * self.roi_top_fraction)
            roi = image[roi_start_y:height, :]

            if roi.size == 0:
                self.get_logger().warn('Image ROI is empty')
                self.publish_line_lost(image_msg)
                self.publish_debug_images(
                    image_msg,
                    image,
                    binary,
                    overlay,
                    roi_start_y
                )
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
                if self.enable_debug_image:
                    overlay = self.make_overlay(
                        image,
                        roi_start_y,
                        None,
                        None,
                        0.0,
                        0.0,
                        0.0,
                        False
                    )
                self.publish_debug_images(
                    image_msg,
                    image,
                    binary,
                    overlay,
                    roi_start_y
                )
                return

            contour = max(contours, key=cv2.contourArea)
            contour_area = float(cv2.contourArea(contour))

            if contour_area < self.min_contour_area:
                self.publish_line_lost(image_msg, contour_area=contour_area)
                if self.enable_debug_image:
                    overlay = self.make_overlay(
                        image,
                        roi_start_y,
                        contour,
                        None,
                        0.0,
                        0.0,
                        0.0,
                        False
                    )
                self.publish_debug_images(
                    image_msg,
                    image,
                    binary,
                    overlay,
                    roi_start_y
                )
                return

            moments = cv2.moments(contour)
            if moments['m00'] == 0.0:
                self.publish_line_lost(image_msg, contour_area=contour_area)
                if self.enable_debug_image:
                    overlay = self.make_overlay(
                        image,
                        roi_start_y,
                        contour,
                        None,
                        0.0,
                        0.0,
                        0.0,
                        False
                    )
                self.publish_debug_images(
                    image_msg,
                    image,
                    binary,
                    overlay,
                    roi_start_y
                )
                return

            centroid_x = float(moments['m10'] / moments['m00'])
            centroid_y = float(moments['m01'] / moments['m00']) + roi_start_y
            centroid = (int(round(centroid_x)), int(round(centroid_y)))
            image_center_x = width / 2.0
            lateral_error = (
                (centroid_x - image_center_x) / image_center_x
                if image_center_x > 0.0 else 0.0
            )
            lateral_error = self.clamp(
                lateral_error,
                -self.max_lateral_error,
                self.max_lateral_error
            )

            heading_error = self.compute_heading_error(contour)
            if heading_error is None:
                self.publish_line_lost(image_msg, contour_area=contour_area)
                if self.enable_debug_image:
                    overlay = self.make_overlay(
                        image,
                        roi_start_y,
                        contour,
                        centroid,
                        0.0,
                        0.0,
                        0.0,
                        False
                    )
                self.publish_debug_images(
                    image_msg,
                    image,
                    binary,
                    overlay,
                    roi_start_y
                )
                return

            confidence = self.compute_confidence(contour_area)

            if self.enable_debug_image:
                overlay = self.make_overlay(
                    image,
                    roi_start_y,
                    contour,
                    centroid,
                    lateral_error,
                    heading_error,
                    confidence,
                    True
                )

            msg = self.make_line_track_msg(
                image_msg,
                lateral_error,
                heading_error,
                confidence,
                True
            )
            self.publisher.publish(msg)
            self.publish_debug_images(
                image_msg,
                image,
                binary,
                overlay,
                roi_start_y
            )
            self.log_debug(
                contour_area,
                lateral_error,
                heading_error,
                confidence,
                True
            )
        except cv2.error as exc:
            self.get_logger().warn(f'OpenCV line tracking failed: {exc}')
            self.publish_line_lost(image_msg, contour_area=contour_area)
            self.publish_debug_images(
                image_msg,
                image,
                binary,
                overlay,
                roi_start_y
            )
        except Exception as exc:
            self.get_logger().warn(f'Line tracking failed: {exc}')
            self.publish_line_lost(image_msg, contour_area=contour_area)
            self.publish_debug_images(
                image_msg,
                image,
                binary,
                overlay,
                roi_start_y
            )

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
        except cv2.error as exc:
            self.get_logger().warn(f'OpenCV fitLine failed: {exc}')
            return None
        except Exception as exc:
            self.get_logger().warn(f'fitLine failed: {exc}')
            return None

        if not math.isfinite(vx) or not math.isfinite(vy):
            return None

        if abs(vy) < 1.0e-6:
            return math.copysign(math.pi / 2.0, vx)

        heading_error = math.atan2(vx, vy)
        return self.clamp(heading_error, -math.pi / 2.0, math.pi / 2.0)

    def compute_confidence(self, contour_area):
        confidence = contour_area / self.min_confidence_area
        return self.clamp(confidence, 0.0, 1.0)

    def publish_line_lost(self, image_msg, contour_area=0.0):
        msg = self.make_line_track_msg(image_msg, 0.0, 0.0, 0.0, False)
        self.publisher.publish(msg)
        self.log_debug(contour_area, 0.0, 0.0, 0.0, False)

    def make_line_track_msg(
        self,
        image_msg,
        lateral_error,
        heading_error,
        confidence,
        line_visible
    ):
        msg = LineTrack()
        msg.header.stamp = image_msg.header.stamp
        msg.header.frame_id = image_msg.header.frame_id or self.frame_id
        msg.lateral_error = float(lateral_error)
        msg.heading_error = float(heading_error)
        msg.confidence = float(confidence)
        msg.line_visible = bool(line_visible)
        return msg

    def make_overlay(
        self,
        image,
        roi_start_y,
        contour,
        centroid,
        lateral_error,
        heading_error,
        confidence,
        line_visible
    ):
        overlay = image.copy()
        height, width = overlay.shape[:2]
        cv2.rectangle(
            overlay,
            (0, roi_start_y),
            (max(0, width - 1), max(0, height - 1)),
            (255, 255, 0),
            2
        )
        cv2.line(
            overlay,
            (width // 2, 0),
            (width // 2, max(0, height - 1)),
            (255, 0, 255),
            1
        )

        if contour is not None:
            shifted_contour = contour.copy()
            shifted_contour[:, 0, 1] += roi_start_y
            cv2.drawContours(overlay, [shifted_contour], -1, (0, 255, 0), 2)

        if centroid is not None:
            cv2.circle(overlay, centroid, 6, (0, 0, 255), -1)

        text_lines = [
            f'lateral_error: {lateral_error:.3f}',
            f'heading_error: {heading_error:.3f}',
            f'confidence: {confidence:.3f}',
            f'line_visible: {line_visible}',
        ]
        for index, text in enumerate(text_lines):
            y = 24 + index * 24
            cv2.putText(
                overlay,
                text,
                (10, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 255, 255),
                2,
                cv2.LINE_AA
            )

        return overlay

    def publish_debug_images(
        self,
        image_msg,
        image,
        binary,
        overlay,
        roi_start_y
    ):
        if not self.enable_debug_image:
            return

        try:
            if binary is None:
                height, width = image.shape[:2]
                binary = np.zeros(
                    (max(1, height - roi_start_y), width),
                    dtype='uint8'
                )
            if overlay is None:
                overlay = self.make_overlay(
                    image,
                    roi_start_y,
                    None,
                    None,
                    0.0,
                    0.0,
                    0.0,
                    False
                )

            mask_msg = self.bridge.cv2_to_imgmsg(binary, encoding='mono8')
            mask_msg.header.stamp = image_msg.header.stamp
            mask_msg.header.frame_id = (
                image_msg.header.frame_id or self.frame_id
            )
            self.mask_publisher.publish(mask_msg)

            overlay_msg = self.bridge.cv2_to_imgmsg(overlay, encoding='bgr8')
            overlay_msg.header.stamp = image_msg.header.stamp
            overlay_msg.header.frame_id = (
                image_msg.header.frame_id or self.frame_id
            )
            self.overlay_publisher.publish(overlay_msg)
        except CvBridgeError as exc:
            self.get_logger().warn(f'Failed to publish debug image: {exc}')
            self.publish_line_lost(image_msg)
        except cv2.error as exc:
            self.get_logger().warn(f'OpenCV debug image failed: {exc}')
            self.publish_line_lost(image_msg)
        except Exception as exc:
            self.get_logger().warn(f'Debug image publishing failed: {exc}')
            self.publish_line_lost(image_msg)

    def log_debug(
        self,
        contour_area,
        lateral_error,
        heading_error,
        confidence,
        line_visible
    ):
        if not self.debug_log:
            return

        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_debug_log_ns < self.debug_log_period_ns:
            return

        self.last_debug_log_ns = now_ns
        self.get_logger().info(
            'line debug: '
            f'contour_area={contour_area:.1f}, '
            f'lateral_error={lateral_error:.3f}, '
            f'heading_error={heading_error:.3f}, '
            f'confidence={confidence:.3f}, '
            f'line_visible={line_visible}'
        )

    @staticmethod
    def clamp(value, minimum, maximum):
        return max(minimum, min(maximum, value))


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
