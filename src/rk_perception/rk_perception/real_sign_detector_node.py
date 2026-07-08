#!/usr/bin/env python3

import json
import math
import time
from dataclasses import dataclass
from typing import Dict, List, Tuple

import cv2
import numpy as np

try:
    import rclpy
    from cv_bridge import CvBridge, CvBridgeError
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image

    from rk_interfaces.msg import SignDetection, SignDetectionArray
except ImportError:
    rclpy = None
    CvBridge = None
    CvBridgeError = Exception
    ExternalShutdownException = Exception
    Image = None
    Node = object
    SignDetection = None
    SignDetectionArray = None
    qos_profile_sensor_data = 10


DEFAULT_QR_VALUE_MAP = {
    '1': {'sign_type': 'place_marker', 'sign_value': 'place_1'},
    'place_1': {'sign_type': 'place_marker', 'sign_value': 'place_1'},
    'platform_1': {'sign_type': 'place_marker', 'sign_value': 'place_1'},
    '2': {'sign_type': 'place_marker', 'sign_value': 'place_2'},
    'place_2': {'sign_type': 'place_marker', 'sign_value': 'place_2'},
    'platform_2': {'sign_type': 'place_marker', 'sign_value': 'place_2'},
    'electric': {'sign_type': 'warning', 'sign_value': 'electric_shock'},
    'electric_shock': {
        'sign_type': 'warning',
        'sign_value': 'electric_shock',
    },
    'shock': {'sign_type': 'warning', 'sign_value': 'electric_shock'},
    'oxidizer': {'sign_type': 'warning', 'sign_value': 'strong_oxidizer'},
    'strong_oxidizer': {
        'sign_type': 'warning',
        'sign_value': 'strong_oxidizer',
    },
    'radiation': {'sign_type': 'warning', 'sign_value': 'radiation'},
    'radioactive': {'sign_type': 'warning', 'sign_value': 'radiation'},
}

DEFAULT_COLOR_RULES = [
    {
        'name': 'red_warning',
        'sign_type': 'warning',
        'sign_value': 'electric_shock',
        'hsv_ranges': [
            [0, 80, 60, 12, 255, 255],
            [170, 80, 60, 180, 255, 255],
        ],
        'min_area_fraction': 0.012,
        'min_confidence': 0.55,
    },
    {
        'name': 'yellow_warning',
        'sign_type': 'warning',
        'sign_value': 'strong_oxidizer',
        'hsv_ranges': [[18, 70, 70, 38, 255, 255]],
        'min_area_fraction': 0.012,
        'min_confidence': 0.55,
    },
    {
        'name': 'blue_warning',
        'sign_type': 'warning',
        'sign_value': 'radiation',
        'hsv_ranges': [[92, 60, 50, 132, 255, 255]],
        'min_area_fraction': 0.012,
        'min_confidence': 0.55,
    },
    {
        'name': 'green_place_1',
        'sign_type': 'place_marker',
        'sign_value': 'place_1',
        'hsv_ranges': [[42, 50, 50, 88, 255, 255]],
        'min_area_fraction': 0.014,
        'min_confidence': 0.55,
    },
    {
        'name': 'purple_place_2',
        'sign_type': 'place_marker',
        'sign_value': 'place_2',
        'hsv_ranges': [[132, 45, 45, 164, 255, 255]],
        'min_area_fraction': 0.014,
        'min_confidence': 0.55,
    },
]


def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


def normalize_label(value):
    normalized = str(value or '').strip().lower()
    normalized = normalized.replace('-', '_').replace(' ', '_')
    while '__' in normalized:
        normalized = normalized.replace('__', '_')
    return normalized


def _as_json_object(raw, fallback):
    if raw is None or raw == '':
        return dict(fallback)
    try:
        data = json.loads(raw)
    except (TypeError, ValueError):
        return dict(fallback)
    if not isinstance(data, dict):
        return dict(fallback)
    return data


def _as_json_list(raw, fallback):
    if raw is None or raw == '':
        return list(fallback)
    try:
        data = json.loads(raw)
    except (TypeError, ValueError):
        return list(fallback)
    if not isinstance(data, list):
        return list(fallback)
    return data


@dataclass(frozen=True)
class SignCandidate:
    sign_type: str
    sign_value: str
    confidence: float
    source: str
    center_x: float = 0.0
    center_y: float = 0.0
    area_fraction: float = 0.0
    contour: object = None


@dataclass(frozen=True)
class ColorRule:
    name: str
    sign_type: str
    sign_value: str
    hsv_ranges: Tuple[Tuple[int, int, int, int, int, int], ...]
    min_area_fraction: float
    min_confidence: float


def parse_color_rules(raw_json):
    rules = []
    for item in _as_json_list(raw_json, DEFAULT_COLOR_RULES):
        if not isinstance(item, dict):
            continue
        hsv_ranges = []
        for hsv_range in item.get('hsv_ranges', []):
            if not isinstance(hsv_range, list) or len(hsv_range) != 6:
                continue
            values = tuple(
                int(clamp(int(value), 0, 255)) for value in hsv_range
            )
            hsv_ranges.append(values)
        if not hsv_ranges:
            continue
        rules.append(ColorRule(
            name=str(item.get('name', 'color_rule')),
            sign_type=normalize_label(item.get('sign_type', 'warning')),
            sign_value=normalize_label(item.get('sign_value', 'unknown')),
            hsv_ranges=tuple(hsv_ranges),
            min_area_fraction=max(
                0.0,
                float(item.get('min_area_fraction', 0.01))
            ),
            min_confidence=float(item.get('min_confidence', 0.55)),
        ))
    return rules


def parse_qr_value_map(raw_json):
    parsed = _as_json_object(raw_json, DEFAULT_QR_VALUE_MAP)
    value_map = {}
    for raw_key, raw_value in parsed.items():
        if isinstance(raw_value, str):
            value_map[normalize_label(raw_key)] = {
                'sign_type': 'warning',
                'sign_value': normalize_label(raw_value),
            }
        elif isinstance(raw_value, dict):
            value_map[normalize_label(raw_key)] = {
                'sign_type': normalize_label(
                    raw_value.get('sign_type', 'warning')
                ),
                'sign_value': normalize_label(
                    raw_value.get('sign_value', raw_key)
                ),
            }
    return value_map


def _find_external_contours(mask):
    result = cv2.findContours(
        mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )
    if len(result) == 2:
        contours, _ = result
    else:
        _, contours, _ = result
    return contours


def detect_color_signs(image_bgr, rules):
    if image_bgr is None or image_bgr.size == 0:
        return []

    height, width = image_bgr.shape[:2]
    image_area = max(1.0, float(height * width))
    hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
    kernel = np.ones((5, 5), np.uint8)
    candidates = []

    for rule in rules:
        mask = np.zeros((height, width), dtype=np.uint8)
        for hsv_range in rule.hsv_ranges:
            lower = np.array(hsv_range[:3], dtype=np.uint8)
            upper = np.array(hsv_range[3:], dtype=np.uint8)
            mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lower, upper))

        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contours = _find_external_contours(mask)
        if not contours:
            continue

        contour = max(contours, key=cv2.contourArea)
        area = float(cv2.contourArea(contour))
        area_fraction = area / image_area
        if area_fraction < rule.min_area_fraction:
            continue

        moments = cv2.moments(contour)
        if abs(moments['m00']) > 1e-6:
            center_x = moments['m10'] / moments['m00']
            center_y = moments['m01'] / moments['m00']
        else:
            x, y, box_w, box_h = cv2.boundingRect(contour)
            center_x = x + box_w * 0.5
            center_y = y + box_h * 0.5

        confidence = clamp(
            rule.min_confidence
            + area_fraction / max(rule.min_area_fraction, 1e-6) * 0.12,
            rule.min_confidence,
            0.92
        )
        candidates.append(SignCandidate(
            sign_type=rule.sign_type,
            sign_value=rule.sign_value,
            confidence=float(confidence),
            source=rule.name,
            center_x=float(center_x),
            center_y=float(center_y),
            area_fraction=float(area_fraction),
            contour=contour,
        ))

    return candidates


def detect_qr_signs(image_bgr, qr_detector, value_map):
    if qr_detector is None:
        return []

    candidates = []
    decoded_items = []
    points_items = []
    try:
        if hasattr(qr_detector, 'detectAndDecodeMulti'):
            ok, decoded_info, points, _ = qr_detector.detectAndDecodeMulti(
                image_bgr
            )
            if ok:
                decoded_items = list(decoded_info or [])
                points_items = list(points) if points is not None else []
        if not decoded_items:
            decoded, points, _ = qr_detector.detectAndDecode(image_bgr)
            if decoded:
                decoded_items = [decoded]
                points_items = [points]
    except cv2.error:
        return []

    for index, decoded in enumerate(decoded_items):
        key = normalize_label(decoded)
        mapped = value_map.get(key)
        if mapped is None:
            mapped = {'sign_type': 'qr', 'sign_value': key}

        center_x = 0.0
        center_y = 0.0
        if index < len(points_items) and points_items[index] is not None:
            pts = np.array(points_items[index], dtype=np.float32).reshape(
                -1,
                2
            )
            if pts.size:
                center_x = float(np.mean(pts[:, 0]))
                center_y = float(np.mean(pts[:, 1]))

        candidates.append(SignCandidate(
            sign_type=normalize_label(mapped.get('sign_type', 'qr')),
            sign_value=normalize_label(mapped.get('sign_value', key)),
            confidence=0.99,
            source='qr',
            center_x=center_x,
            center_y=center_y,
        ))
    return candidates


def merge_candidates(candidates, min_confidence):
    best_by_key: Dict[Tuple[str, str], SignCandidate] = {}
    for candidate in candidates:
        if not math.isfinite(candidate.confidence):
            continue
        if candidate.confidence < min_confidence:
            continue
        key = (candidate.sign_type, candidate.sign_value)
        old = best_by_key.get(key)
        if old is None or candidate.confidence > old.confidence:
            best_by_key[key] = candidate
    return sorted(
        best_by_key.values(),
        key=lambda item: item.confidence,
        reverse=True
    )


class RealSignDetectorNode(Node):
    """Detect simple competition signs from the robot RGB camera."""

    def __init__(self):
        super().__init__('real_sign_detector_node')
        self._declare_parameters()

        self.image_topic = self._string_parameter('image_topic')
        self.sign_detections_topic = self._string_parameter(
            'sign_detections_topic'
        )
        self.debug_image_topic = self._string_parameter('debug_image_topic')
        self.frame_id = self._string_parameter('frame_id')
        self.min_confidence = self._float_parameter('min_confidence', 0.55)
        self.enable_qr = self._bool_parameter('enable_qr')
        self.enable_color = self._bool_parameter('enable_color')
        self.enable_debug_image = self._bool_parameter('enable_debug_image')
        self.debug_log = self._bool_parameter('debug_log')
        self.log_period_sec = self._float_parameter('log_period_sec', 1.0)
        self._last_log_time = 0.0

        self.color_rules = parse_color_rules(
            self._string_parameter('color_rules_json')
        )
        self.qr_value_map = parse_qr_value_map(
            self._string_parameter('qr_value_map_json')
        )
        self.qr_detector = cv2.QRCodeDetector() if self.enable_qr else None
        self.bridge = CvBridge()

        self.publisher = self.create_publisher(
            SignDetectionArray,
            self.sign_detections_topic,
            10
        )
        self.debug_publisher = None
        if self.enable_debug_image:
            self.debug_publisher = self.create_publisher(
                Image,
                self.debug_image_topic,
                2
            )

        self.subscription = self.create_subscription(
            Image,
            self.image_topic,
            self._on_image,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            'Real sign detector ready: '
            f'image_topic={self.image_topic}, '
            f'sign_topic={self.sign_detections_topic}, '
            f'qr={self.enable_qr}, color={self.enable_color}'
        )

    def _declare_parameters(self):
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter(
            'sign_detections_topic',
            '/perception/sign_detections'
        )
        self.declare_parameter(
            'debug_image_topic',
            '/perception/sign_debug_image'
        )
        self.declare_parameter('frame_id', 'd435i_color_optical_frame')
        self.declare_parameter('min_confidence', 0.55)
        self.declare_parameter('enable_qr', True)
        self.declare_parameter('enable_color', True)
        self.declare_parameter('enable_debug_image', False)
        self.declare_parameter('debug_log', False)
        self.declare_parameter('log_period_sec', 1.0)
        self.declare_parameter(
            'color_rules_json',
            json.dumps(DEFAULT_COLOR_RULES, separators=(',', ':'))
        )
        self.declare_parameter(
            'qr_value_map_json',
            json.dumps(DEFAULT_QR_VALUE_MAP, separators=(',', ':'))
        )

    def _on_image(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as error:
            self.get_logger().error(f'cv_bridge failed: {error}')
            return

        candidates: List[SignCandidate] = []
        if self.enable_qr:
            candidates.extend(detect_qr_signs(
                image,
                self.qr_detector,
                self.qr_value_map
            ))
        if self.enable_color:
            candidates.extend(detect_color_signs(image, self.color_rules))

        detections = merge_candidates(candidates, self.min_confidence)
        self._publish_detections(msg, detections)

        if self.debug_publisher is not None:
            self._publish_debug_image(msg, image, detections)
        if self.debug_log:
            self._log_detections(detections)

    def _publish_detections(self, image_msg, detections):
        msg = SignDetectionArray()
        msg.header.stamp = image_msg.header.stamp
        msg.header.frame_id = image_msg.header.frame_id or self.frame_id

        for candidate in detections:
            detection = SignDetection()
            detection.header = msg.header
            detection.sign_type = candidate.sign_type
            detection.sign_value = candidate.sign_value
            detection.confidence = float(candidate.confidence)
            msg.detections.append(detection)

        self.publisher.publish(msg)

    def _publish_debug_image(self, image_msg, image, detections):
        overlay = image.copy()
        for candidate in detections:
            if candidate.contour is not None:
                cv2.drawContours(
                    overlay,
                    [candidate.contour],
                    -1,
                    (0, 255, 0),
                    2
                )
            x = int(candidate.center_x)
            y = int(candidate.center_y)
            cv2.circle(overlay, (x, y), 4, (0, 255, 255), -1)
            label = (
                f'{candidate.sign_type}:{candidate.sign_value} '
                f'{candidate.confidence:.2f}'
            )
            cv2.putText(
                overlay,
                label,
                (max(0, x - 40), max(20, y - 12)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (0, 255, 255),
                1,
                cv2.LINE_AA
            )
        debug_msg = self.bridge.cv2_to_imgmsg(overlay, encoding='bgr8')
        debug_msg.header = image_msg.header
        self.debug_publisher.publish(debug_msg)

    def _log_detections(self, detections):
        now = time.monotonic()
        if now - self._last_log_time < self.log_period_sec:
            return
        self._last_log_time = now
        if not detections:
            self.get_logger().info('No sign detected')
            return
        summary = ', '.join(
            f'{item.sign_type}:{item.sign_value}:{item.confidence:.2f}'
            for item in detections
        )
        self.get_logger().info(f'Sign detections: {summary}')

    def _string_parameter(self, name):
        return str(self.get_parameter(name).value)

    def _bool_parameter(self, name):
        value = self.get_parameter(name).value
        if isinstance(value, str):
            return value.strip().lower() in ('1', 'true', 'yes', 'on')
        return bool(value)

    def _float_parameter(self, name, default):
        try:
            value = float(self.get_parameter(name).value)
        except (TypeError, ValueError):
            return float(default)
        if not math.isfinite(value):
            return float(default)
        return value


def main(args=None):
    if rclpy is None:
        raise RuntimeError('ROS2 Python dependencies are not available')
    rclpy.init(args=args)
    node = RealSignDetectorNode()
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
