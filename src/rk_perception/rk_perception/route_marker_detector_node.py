#!/usr/bin/env python3
"""State-neutral white-line and red-circle route marker detector.

The detector only publishes visual evidence using standard ROS messages.  The
national mission FSM owns start/finish locks and decides when that evidence is
valid, so a start marker cannot trigger the finish route by itself.
"""

import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32


class RouteMarkerDetectorNode(Node):
    """Detect high-value horizontal bars and circular red ground markers."""

    def __init__(self):
        super().__init__('route_marker_detector_node')
        self._declare_parameters()
        self.bridge = CvBridge()
        self.white_streak = 0
        self.red_streak = 0
        self.white_pub = self.create_publisher(
            Bool, self._string('white_line_topic'), 10
        )
        self.white_confidence_pub = self.create_publisher(
            Float32, self._string('white_line_confidence_topic'), 10
        )
        self.red_pub = self.create_publisher(
            PointStamped, self._string('red_circle_topic'), 10
        )
        self.red_visible_pub = self.create_publisher(
            Bool, self._string('red_circle_visible_topic'), 10
        )
        self.debug_pub = self.create_publisher(
            Image, self._string('debug_overlay_topic'), 10
        )
        self.image_sub = self.create_subscription(
            Image, self._string('image_topic'), self._on_image, 10
        )

    def _declare_parameters(self):
        defaults = {
            'image_topic': '/camera/color/image_raw',
            'white_line_topic': '/perception/route_markers/white_line',
            'white_line_confidence_topic': (
                '/perception/route_markers/white_line_confidence'
            ),
            'red_circle_topic': '/perception/route_markers/red_circle',
            'red_circle_visible_topic': (
                '/perception/route_markers/red_circle_visible'
            ),
            'debug_overlay_topic': '/perception/debug/route_markers_overlay',
            'ground_roi_top_fraction': 0.42,
            'white_min_value': 185,
            'white_max_saturation': 70,
            'white_min_width_fraction': 0.28,
            'white_min_thickness_px': 3.0,
            'white_max_thickness_px': 90.0,
            'white_confirm_frames': 3,
            'red_hue_low_1': 0,
            'red_hue_high_1': 10,
            'red_hue_low_2': 170,
            'red_hue_high_2': 180,
            'red_min_saturation': 90,
            'red_min_value': 70,
            'red_min_area_ratio': 0.0015,
            'red_min_circularity': 0.55,
            'red_confirm_frames': 3,
            'publish_debug_overlay': True,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _string(self, name):
        return str(self.get_parameter(name).value)

    def _on_image(self, message):
        try:
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warn('route marker image conversion failed: {}'.format(exc))
            return
        if image is None or image.size == 0:
            return
        white_visible, white_confidence, white_rect = self._detect_white(image)
        red_visible, red_point, red_circle = self._detect_red(image)
        self._publish_white(white_visible, white_confidence)
        self._publish_red(message, red_visible, red_point)
        if bool(self.get_parameter('publish_debug_overlay').value):
            self._publish_overlay(message, image, white_rect, red_circle,
                                  white_visible, red_visible)

    def _ground_roi(self, image):
        height = image.shape[0]
        top_fraction = max(0.0, min(
            0.95, float(self.get_parameter('ground_roi_top_fraction').value)
        ))
        top = int(height * top_fraction)
        return top, image[top:, :]

    def _detect_white(self, image):
        top, roi = self._ground_roi(image)
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        saturation = hsv[:, :, 1]
        value = hsv[:, :, 2]
        mask = cv2.inRange(
            saturation, 0, int(self.get_parameter('white_max_saturation').value)
        )
        mask = cv2.bitwise_and(mask, cv2.inRange(
            value, int(self.get_parameter('white_min_value').value), 255
        ))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        image_width = image.shape[1]
        best = None
        best_confidence = 0.0
        for contour in contours:
            x, y, width, height = cv2.boundingRect(contour)
            if height <= 0:
                continue
            coverage = width / float(max(1, image_width))
            thickness_ok = (
                height >= float(self.get_parameter('white_min_thickness_px').value)
                and height <= float(
                    self.get_parameter('white_max_thickness_px').value
                )
            )
            if not thickness_ok:
                continue
            aspect = width / float(height)
            if coverage < float(self.get_parameter('white_min_width_fraction').value):
                continue
            confidence = min(1.0, 0.70 * coverage + 0.30 * min(1.0, aspect / 8.0))
            if confidence > best_confidence:
                best_confidence = confidence
                best = (x, y + top, width, height)
        if best is None:
            self.white_streak = 0
            return False, 0.0, None
        self.white_streak += 1
        visible = self.white_streak >= int(
            self.get_parameter('white_confirm_frames').value
        )
        return visible, best_confidence, best

    def _detect_red(self, image):
        top, roi = self._ground_roi(image)
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        lower_1 = np.array([
            int(self.get_parameter('red_hue_low_1').value),
            int(self.get_parameter('red_min_saturation').value),
            int(self.get_parameter('red_min_value').value),
        ])
        upper_1 = np.array([
            int(self.get_parameter('red_hue_high_1').value), 255, 255
        ])
        lower_2 = np.array([
            int(self.get_parameter('red_hue_low_2').value),
            int(self.get_parameter('red_min_saturation').value),
            int(self.get_parameter('red_min_value').value),
        ])
        upper_2 = np.array([
            int(self.get_parameter('red_hue_high_2').value), 255, 255
        ])
        mask = cv2.bitwise_or(cv2.inRange(hsv, lower_1, upper_1),
                              cv2.inRange(hsv, lower_2, upper_2))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        roi_area = float(max(1, roi.shape[0] * roi.shape[1]))
        best = None
        best_confidence = 0.0
        for contour in contours:
            area = cv2.contourArea(contour)
            if area <= 0.0 or area / roi_area < float(
                self.get_parameter('red_min_area_ratio').value
            ):
                continue
            perimeter = cv2.arcLength(contour, True)
            if perimeter <= 0.0:
                continue
            circularity = 4.0 * math.pi * area / (perimeter * perimeter)
            if circularity < float(self.get_parameter('red_min_circularity').value):
                continue
            x, y, width, height = cv2.boundingRect(contour)
            if height <= 0 or width <= 0:
                continue
            aspect = min(width, height) / float(max(width, height))
            confidence = min(1.0, 0.55 * circularity + 0.25 * aspect +
                             0.20 * min(1.0, area / (roi_area * 0.02)))
            if confidence > best_confidence:
                best_confidence = confidence
                moments = cv2.moments(contour)
                center_x = moments['m10'] / moments['m00']
                center_y = moments['m01'] / moments['m00'] + top
                best = (center_x, center_y, x, y + top, width, height)
        if best is None:
            self.red_streak = 0
            return False, (0.0, 0.0, 0.0), None
        self.red_streak += 1
        visible = self.red_streak >= int(
            self.get_parameter('red_confirm_frames').value
        )
        point = (
            best[0] / float(max(1, image.shape[1])),
            best[1] / float(max(1, image.shape[0])),
            best_confidence if visible else 0.0,
        )
        return visible, point, best[2:]

    def _publish_white(self, visible, confidence):
        bool_msg = Bool()
        bool_msg.data = bool(visible)
        self.white_pub.publish(bool_msg)
        confidence_msg = Float32()
        confidence_msg.data = float(confidence)
        self.white_confidence_pub.publish(confidence_msg)

    def _publish_red(self, image_message, visible, point):
        point_msg = PointStamped()
        point_msg.header = image_message.header
        point_msg.point.x = float(point[0])
        point_msg.point.y = float(point[1])
        point_msg.point.z = float(point[2]) if visible else 0.0
        self.red_pub.publish(point_msg)
        visible_msg = Bool()
        visible_msg.data = bool(visible)
        self.red_visible_pub.publish(visible_msg)

    def _publish_overlay(self, source, image, white_rect, red_circle,
                         white_visible, red_visible):
        overlay = image.copy()
        if white_rect is not None:
            x, y, width, height = white_rect
            color = (0, 255, 0) if white_visible else (0, 180, 255)
            cv2.rectangle(overlay, (x, y), (x + width, y + height), color, 2)
        if red_circle is not None:
            x, y, width, height = red_circle
            color = (0, 255, 0) if red_visible else (0, 180, 255)
            cv2.ellipse(overlay, (x, y, width, height), color, 2)
        try:
            message = self.bridge.cv2_to_imgmsg(overlay, encoding='bgr8')
            message.header = source.header
            self.debug_pub.publish(message)
        except Exception as exc:
            self.get_logger().debug('route marker overlay publish failed: {}'.format(exc))


def main(args=None):
    rclpy.init(args=args)
    node = RouteMarkerDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
