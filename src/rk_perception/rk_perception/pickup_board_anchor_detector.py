"""抓取平台前置挡板的最小独立检测器。

该节点只发布图像中候选挡板的几何比例，不参与路线选择、底盘控制或机械臂
控制。所有 HSV/灰度阈值均由 YAML 提供，默认值故意保守且标为待标定。
"""

from __future__ import annotations

from dataclasses import dataclass
import json

import cv2
import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from std_msgs.msg import String
    from rk_interfaces.msg import SpecialTargetDetection
    ROS_AVAILABLE = True
except ImportError:  # 允许 OpenCV 算法在无 ROS 的 CI 环境独立测试。
    rclpy = None
    Node = object
    Image = object
    String = object
    SpecialTargetDetection = object
    ROS_AVAILABLE = False

try:
    from cv_bridge import CvBridge
except ImportError:  # 单元测试环境可能不安装 ROS 图像桥接。
    CvBridge = None


@dataclass(frozen=True)
class PickupBoardResult:
    """挡板候选的归一化 bbox 数据，适合直接进入黄金画面统计。"""

    detected: bool = False
    confidence: float = 0.0
    center_x_ratio: float = 0.0
    center_y_ratio: float = 0.0
    bottom_y_ratio: float = 0.0
    width_ratio: float = 0.0
    height_ratio: float = 0.0
    area_ratio: float = 0.0
    reason: str = 'not_detected'


def pickup_board_route_enabled(payload):
    """只允许显式抓取平台 route phase，畸形/普通路线一律关闭。"""
    if isinstance(payload, str):
        try:
            payload = json.loads(payload)
        except (TypeError, ValueError, json.JSONDecodeError):
            return False
    return (
        isinstance(payload, dict)
        and payload.get('platform_route_phase') == (
            'PICKUP_PLATFORM_APPROACH')
    )


def detect_pickup_board(image_bgr, config):
    """从 ROI 的暗色或 HSV 区域提取最大合格矩形，不伪造现场阈值。"""
    if image_bgr is None or getattr(image_bgr, 'ndim', 0) != 3:
        return PickupBoardResult(reason='invalid_bgr_image')
    height, width = image_bgr.shape[:2]
    if height <= 0 or width <= 0:
        return PickupBoardResult(reason='empty_image')
    left = int(max(0.0, min(1.0, config['roi_left_fraction'])) * width)
    right = int(max(0.0, min(1.0, config['roi_right_fraction'])) * width)
    top = int(max(0.0, min(1.0, config['roi_top_fraction'])) * height)
    bottom = int(max(0.0, min(1.0, config['roi_bottom_fraction'])) * height)
    if right <= left or bottom <= top:
        return PickupBoardResult(reason='invalid_roi')
    roi = image_bgr[top:bottom, left:right]
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    # 两个参数化掩码取交集，避免单纯暗影或低饱和背景作为挡板。
    gray_mask = cv2.inRange(gray, 0, int(config['gray_max']))
    hsv_mask = cv2.inRange(
        hsv,
        np.array([int(config['hsv_h_min']), int(config['hsv_s_min']),
                  int(config['hsv_v_min'])]),
        np.array([int(config['hsv_h_max']), int(config['hsv_s_max']),
                  int(config['hsv_v_max'])]),
    )
    mask = cv2.bitwise_and(gray_mask, hsv_mask)
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    contours, _ = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    candidates = []
    image_area = float(width * height)
    for contour in contours:
        x, y, box_width, box_height = cv2.boundingRect(contour)
        if box_width <= 0 or box_height <= 0:
            continue
        area_ratio = (box_width * box_height) / image_area
        aspect_ratio = box_width / float(box_height)
        if not (
            float(
                config['min_area_ratio']) <= area_ratio <= float(
                config['max_area_ratio'])):
            continue
        if not (
            float(
                config['min_aspect_ratio']) <= aspect_ratio <= float(
                config['max_aspect_ratio'])):
            continue
        candidates.append(
            (box_width *
             box_height,
             x,
             y,
             box_width,
             box_height,
             area_ratio))
    if not candidates:
        return PickupBoardResult(reason='no_candidate')
    _, x, y, box_width, box_height, area_ratio = max(candidates)
    full_x, full_y = x + left, y + top
    # confidence 是候选占参数允许范围的简单一致性评分，不能替代现场黄金标定。
    confidence = min(1.0, max(0.0, area_ratio /
                              max(float(config['min_area_ratio']), 1e-9)))
    return PickupBoardResult(
        detected=True, confidence=confidence,
        center_x_ratio=(full_x + box_width / 2.0) / width,
        center_y_ratio=(full_y + box_height / 2.0) / height,
        bottom_y_ratio=(full_y + box_height) / height,
        width_ratio=box_width / width, height_ratio=box_height / height,
        area_ratio=area_ratio, reason='candidate',
    )


class PickupBoardAnchorDetector(Node):
    """将 Go2 前置相机的挡板候选发布为标准 SpecialTargetDetection。"""

    def __init__(self):
        super().__init__('pickup_board_anchor_detector')
        self._declare_parameters()
        self.bridge = CvBridge() if CvBridge is not None else None
        self._route_enabled = False
        self._confirm_count = 0
        self.publisher = self.create_publisher(
            SpecialTargetDetection,
            self.get_parameter('output_topic').value,
            10)
        self.metrics_publisher = self.create_publisher(
            String, self.get_parameter('metrics_topic').value, 10)
        self.subscription = self.create_subscription(
            Image, self.get_parameter('image_topic').value, self._on_image, 10)
        self.route_subscription = self.create_subscription(
            String,
            self.get_parameter('route_state_topic').value,
            self._on_route_state,
            10)

    def _declare_parameters(self):
        defaults = {
            'image_topic': '/go2/front_camera/image_raw',
            'output_topic': '/perception/pickup_board_anchor',
            'metrics_topic': '/perception/pickup_board_anchor_metrics',
            'route_state_topic': '/mission/line_course_state',
            'calibration_mode': False,
            'pickup_board_roi_left_fraction': 0.0,
            'pickup_board_roi_right_fraction': 1.0,
            'pickup_board_roi_top_fraction': 0.0,
            'pickup_board_roi_bottom_fraction': 1.0,
            # CALIBRATION_REQUIRED：以下仅为 detector scaffold 的安全入口。
            'pickup_board_threshold': 0, 'pickup_board_hsv_h_min': 0,
            'pickup_board_hsv_h_max': 180, 'pickup_board_hsv_s_min': 0,
            'pickup_board_hsv_s_max': 255, 'pickup_board_hsv_v_min': 0,
            'pickup_board_hsv_v_max': 255, 'pickup_board_min_area_ratio': 1.0,
            'pickup_board_max_area_ratio': 0.0,
            'pickup_board_min_aspect_ratio': 1.0,
            'pickup_board_max_aspect_ratio': 0.0,
            'pickup_board_confirm_frames': 3,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _config(self):
        names = ('roi_left_fraction', 'roi_right_fraction', 'roi_top_fraction',
                 'roi_bottom_fraction', 'gray_max', 'hsv_h_min', 'hsv_h_max',
                 'hsv_s_min', 'hsv_s_max', 'hsv_v_min', 'hsv_v_max',
                 'min_area_ratio', 'max_area_ratio', 'min_aspect_ratio',
                 'max_aspect_ratio')
        config = {
            name: self.get_parameter('pickup_board_' + name).value
            for name in names
            if name != 'gray_max'
        }
        config['gray_max'] = self.get_parameter(
            'pickup_board_threshold').value
        return config

    def _on_route_state(self, message):
        """只有显式抓取平台 route phase 才启用挡板图像计算。"""
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            self._route_enabled = False
            self._confirm_count = 0
            return
        self._route_enabled = pickup_board_route_enabled(payload)
        if not self._route_enabled:
            self._confirm_count = 0

    def _on_image(self, message):
        """图像桥接失败时也发布明确的不可见帧，避免下游沿用旧检测。"""
        enabled = self._route_enabled or bool(
            self.get_parameter('calibration_mode').value)
        result = PickupBoardResult(reason='pickup_route_gate_closed')
        if enabled and self.bridge is None:
            result = PickupBoardResult(reason='cv_bridge_unavailable')
        elif enabled:
            try:
                result = detect_pickup_board(self.bridge.imgmsg_to_cv2(
                    message, desired_encoding='bgr8'), self._config())
            except Exception as error:  # 相机编码异常不得中断检测器回调。
                result = PickupBoardResult(
                    reason='image_decode_failed:{}'.format(
                        type(error).__name__))
        self._confirm_count = (
            self._confirm_count + 1 if result.detected else 0)
        confirmed = self._confirm_count >= int(
            self.get_parameter('pickup_board_confirm_frames').value)
        output = SpecialTargetDetection()
        output.header = message.header
        output.target_type = 'pickup_board_anchor'
        output.visible = confirmed
        output.confidence = float(result.confidence)
        output.center_x = float(result.center_x_ratio)
        output.center_y = float(result.center_y_ratio)
        output.area_ratio = float(result.area_ratio)
        output.width_ratio = float(result.width_ratio)
        output.height_ratio = float(result.height_ratio)
        output.inside_candidate = False
        output.direction_hint = ''
        output.reason = result.reason
        self.publisher.publish(output)
        metrics = String()
        metrics.data = json.dumps({
            'detected': confirmed,
            'raw_detected': result.detected,
            'confidence': result.confidence,
            'bbox_center_x_ratio': result.center_x_ratio,
            'bbox_center_y_ratio': result.center_y_ratio,
            'bbox_bottom_y_ratio': result.bottom_y_ratio,
            'bbox_width_ratio': result.width_ratio,
            'bbox_height_ratio': result.height_ratio,
            'area_ratio': result.area_ratio,
            'stamp_sec': int(message.header.stamp.sec),
            'stamp_nanosec': int(message.header.stamp.nanosec),
            'frame_id': str(message.header.frame_id),
            'confirm_count': self._confirm_count,
            'reason': result.reason,
        }, sort_keys=True)
        self.metrics_publisher.publish(metrics)


def main(args=None):
    """ROS 入口。"""
    if not ROS_AVAILABLE:
        raise RuntimeError('rclpy and rk_interfaces are required for ROS node')
    rclpy.init(args=args)
    node = PickupBoardAnchorDetector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
