#!/usr/bin/env python3

import base64
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

DEFAULT_WARNING_TEMPLATE_SIZE = 48
DEFAULT_WARNING_TEMPLATE_SCORE = 0.34
DEFAULT_WARNING_TEMPLATE_MIN_AREA_FRACTION = 0.010

DEFAULT_WARNING_TEMPLATE_IMAGES = {
    'electric_shock': (
        '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'
        'SuQmCC'
    ),
    'strong_oxidizer': (
        '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'
        'J3ZIwGMcPEAAAAASUVORK5CYII='
    ),
    'radiation': (
        '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'
        'xFvEtfXbjfFxL1QbDyhwdShgxoavUVmMe9DTpTv'
        '22+i2aOcdQk+W0Wr93slSW/hKJQ3q1oOnims08'
        'QHKUX9/XNmJaEoTKALmH28r8q1VN5wGtCI3u2b'
        '/ps3dYGARzxIenKzPsMEVaqwDo4k8W/XG79MO'
        'W77zx+STrXmTMn2ZnPd08wiYcAyRB+/F50+c'
        'AEKGhI1sfFjUWb1rHwPlmrzX8h6z+GQRigI'
        '8jhsj33JlDZLpSP28B9xhXljJRD/fE2jCB'
        'nlSnu7QFKnLzxu9r8pG36gdgfxf5ABRKkY'
        '1FQM01j/1xQfduteXjPmoybplTpTDgBaJ'
        'ailtF3LZe3r5FOTq6rfXeXI89eYjHfLd4'
        'QcJDTeBcnJasvl51JyVbzET0iCcRIj+/7'
        'GEYY9eIh8OFYWGNhROBkCLKZsdPVV07Lm'
        'PL6X+pnLzrR1Cy8Az/3ClHvwsM02WWb9'
        'fjA9IxfYd0I0NPrgw0bIip0CUIxPc/H'
        'tU0v7wEWFHsHYLM4bH3HmpPuwTqLiBQx'
        'UGjSGsnzO/D3Mep7bo95BiO2MBPoOPR'
        'rFvMSPL7FQ/+Pg7QxQPgC/vOlV8hO0I'
        '/pwD82QHD8F4PhcMRo8XjXAAAAAElF'
        'TkSuQmCC'
    ),
}


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


def _largest_contour(mask):
    contours = _find_external_contours(mask)
    if not contours:
        return None
    return max(contours, key=cv2.contourArea)


def _yellow_warning_mask(image_bgr):
    hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
    lower = np.array([15, 55, 70], dtype=np.uint8)
    upper = np.array([45, 255, 255], dtype=np.uint8)
    mask = cv2.inRange(hsv, lower, upper)
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask


def _expanded_rect(x, y, width, height, image_width, image_height, ratio=0.18):
    pad_x = int(width * ratio)
    pad_y = int(height * ratio)
    x0 = max(0, x - pad_x)
    y0 = max(0, y - pad_y)
    x1 = min(image_width, x + width + pad_x)
    y1 = min(image_height, y + height + pad_y)
    return x0, y0, max(1, x1 - x0), max(1, y1 - y0)


def _normalize_warning_symbol(image_bgr, size=DEFAULT_WARNING_TEMPLATE_SIZE):
    if image_bgr is None or image_bgr.size == 0:
        return None

    height, width = image_bgr.shape[:2]
    yellow_mask = _yellow_warning_mask(image_bgr)
    yellow_contour = _largest_contour(yellow_mask)
    if yellow_contour is None or cv2.contourArea(yellow_contour) < 20.0:
        return None

    support = np.zeros((height, width), dtype=np.uint8)
    cv2.drawContours(support, [yellow_contour], -1, 255, thickness=-1)
    kernel_size = max(5, int(min(width, height) * 0.08))
    if kernel_size % 2 == 0:
        kernel_size += 1
    kernel = np.ones((kernel_size, kernel_size), np.uint8)
    support = cv2.dilate(support, kernel, iterations=1)

    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
    dark_mask = cv2.inRange(gray, 0, 105)
    dark_mask = cv2.bitwise_and(dark_mask, support)

    x, y, rect_w, rect_h = cv2.boundingRect(support)
    x, y, rect_w, rect_h = _expanded_rect(
        x,
        y,
        rect_w,
        rect_h,
        width,
        height,
        ratio=0.03
    )
    crop = dark_mask[y:y + rect_h, x:x + rect_w]
    if crop.size == 0:
        return None

    normalized = cv2.resize(
        crop,
        (size, size),
        interpolation=cv2.INTER_AREA
    )
    _, normalized = cv2.threshold(
        normalized,
        60,
        255,
        cv2.THRESH_BINARY
    )
    return normalized


def load_warning_templates(template_images=None):
    templates = {}
    raw_templates = template_images or DEFAULT_WARNING_TEMPLATE_IMAGES
    for sign_value, encoded in raw_templates.items():
        try:
            image_bytes = base64.b64decode(encoded)
        except (TypeError, ValueError):
            continue
        image_array = np.frombuffer(image_bytes, dtype=np.uint8)
        image = cv2.imdecode(image_array, cv2.IMREAD_COLOR)
        normalized = _normalize_warning_symbol(image)
        if normalized is None:
            continue
        templates[normalize_label(sign_value)] = normalized
    return templates


def _template_score(candidate_mask, template_mask):
    candidate = candidate_mask.astype(np.float32).reshape(-1) / 255.0
    template = template_mask.astype(np.float32).reshape(-1) / 255.0
    candidate -= float(np.mean(candidate))
    template -= float(np.mean(template))
    denominator = float(np.linalg.norm(candidate) * np.linalg.norm(template))
    if denominator <= 1e-6:
        return 0.0
    return float(np.dot(candidate, template) / denominator)


def detect_warning_template_signs(
    image_bgr,
    templates,
    min_area_fraction=DEFAULT_WARNING_TEMPLATE_MIN_AREA_FRACTION,
    min_score=DEFAULT_WARNING_TEMPLATE_SCORE
):
    if image_bgr is None or image_bgr.size == 0 or not templates:
        return []

    height, width = image_bgr.shape[:2]
    image_area = max(1.0, float(height * width))
    yellow_mask = _yellow_warning_mask(image_bgr)
    contours = _find_external_contours(yellow_mask)
    candidates = []

    for contour in contours:
        area = float(cv2.contourArea(contour))
        area_fraction = area / image_area
        if area_fraction < min_area_fraction:
            continue

        x, y, rect_w, rect_h = cv2.boundingRect(contour)
        x, y, rect_w, rect_h = _expanded_rect(
            x,
            y,
            rect_w,
            rect_h,
            width,
            height,
            ratio=0.28
        )
        roi = image_bgr[y:y + rect_h, x:x + rect_w]
        symbol = _normalize_warning_symbol(roi)
        if symbol is None:
            continue

        best_value = None
        best_score = -1.0
        for sign_value, template in templates.items():
            score = _template_score(symbol, template)
            if score > best_score:
                best_value = sign_value
                best_score = score

        if best_value is None or best_score < min_score:
            continue

        confidence = clamp(
            0.45 + best_score * 0.55,
            0.0,
            0.96
        )
        candidates.append(SignCandidate(
            sign_type='warning',
            sign_value=best_value,
            confidence=float(confidence),
            source=f'template:{best_score:.2f}',
            center_x=float(x + rect_w * 0.5),
            center_y=float(y + rect_h * 0.5),
            area_fraction=float(area_fraction),
            contour=contour,
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
        self.enable_warning_templates = self._bool_parameter(
            'enable_warning_templates'
        )
        self.enable_color = self._bool_parameter('enable_color')
        self.enable_debug_image = self._bool_parameter('enable_debug_image')
        self.debug_log = self._bool_parameter('debug_log')
        self.log_period_sec = self._float_parameter('log_period_sec', 1.0)
        self.template_min_score = self._float_parameter(
            'template_min_score',
            DEFAULT_WARNING_TEMPLATE_SCORE
        )
        self.template_min_area_fraction = self._float_parameter(
            'template_min_area_fraction',
            DEFAULT_WARNING_TEMPLATE_MIN_AREA_FRACTION
        )
        self._last_log_time = 0.0

        self.color_rules = parse_color_rules(
            self._string_parameter('color_rules_json')
        )
        self.qr_value_map = parse_qr_value_map(
            self._string_parameter('qr_value_map_json')
        )
        self.warning_templates = load_warning_templates()
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
            f'qr={self.enable_qr}, '
            f'templates={self.enable_warning_templates}, '
            f'color={self.enable_color}'
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
        self.declare_parameter('enable_warning_templates', True)
        self.declare_parameter(
            'template_min_score',
            DEFAULT_WARNING_TEMPLATE_SCORE
        )
        self.declare_parameter(
            'template_min_area_fraction',
            DEFAULT_WARNING_TEMPLATE_MIN_AREA_FRACTION
        )
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
        if self.enable_warning_templates:
            candidates.extend(detect_warning_template_signs(
                image,
                self.warning_templates,
                self.template_min_area_fraction,
                self.template_min_score
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
