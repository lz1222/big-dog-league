#!/usr/bin/env python3

from dataclasses import dataclass, replace
import math
from typing import Optional, Sequence, Tuple

import cv2
import numpy as np

try:
    import rclpy
    from cv_bridge import CvBridge, CvBridgeError
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    from std_msgs.msg import Header

    from rk_interfaces.msg import LineTrack, SpecialTargetDetection
except ImportError:
    rclpy = None
    CvBridge = None
    CvBridgeError = Exception
    ExternalShutdownException = Exception
    Header = None
    Image = None
    LineTrack = None
    SpecialTargetDetection = None
    Node = object
    qos_profile_sensor_data = 10


def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


PENDING_SWITCH_STABLE_FRAMES = 3


@dataclass(frozen=True)
class LineTrackerConfig:
    # 可调参数主要在 src/rk_bringup/config/line_nav_params.yaml 中改。
    # 这里是程序内部默认值和参数名定义；只有新增参数时才建议改这里。
    #
    # ROI 参数：控制使用画面的哪一部分做巡线检测。
    use_full_frame_roi: bool = True
    roi_top_fraction: float = 0.05
    roi_bottom_margin_fraction: float = 0.03
    roi_left_margin_fraction: float = 0.03
    roi_right_margin_fraction: float = 0.03

    # 黑线提取参数：threshold_value 越大，越容易把灰色阴影也识别为黑线。
    threshold_value: int = 80
    max_lateral_error: float = 1.0
    line_width_cm: float = 10.0

    # 扫描带参数：要求越多扫描带连续命中，越不容易被零散黑物体干扰。
    num_scan_bands: int = 11
    min_path_bands: int = 3
    min_valid_bands: Optional[int] = None
    require_bottom_band: bool = False

    # 线宽参数：透视开启后，画面底部允许更宽，画面顶部允许更窄。
    min_line_width_fraction: float = 0.015
    max_line_width_fraction: float = 0.20
    perspective_width_enabled: bool = False
    min_line_width_top_fraction: float = 0.006
    min_line_width_bottom_fraction: float = 0.015
    max_line_width_top_fraction: float = 0.10
    max_line_width_bottom_fraction: float = 0.30
    max_dark_fraction: float = 0.35
    visible_min_confidence: float = 0.45

    # 选线/锁线参数：用于防止巡线突然跳到旁边黑色物体。
    bottom_band_preference_weight: float = 2.0
    previous_center_weight: float = 2.0
    bottom_start_max_center_error_fraction: float = 1.0
    max_band_center_jump_fraction: float = 0.30
    max_band_gap: int = 2

    def normalized(self):
        min_width = clamp(float(self.min_line_width_fraction), 0.001, 1.0)
        max_width = clamp(float(self.max_line_width_fraction), min_width, 1.0)
        min_top_width = clamp(
            float(self.min_line_width_top_fraction),
            0.001,
            1.0
        )
        min_bottom_width = clamp(
            float(self.min_line_width_bottom_fraction),
            0.001,
            1.0
        )
        max_top_width = clamp(
            float(self.max_line_width_top_fraction),
            min_top_width,
            1.0
        )
        max_bottom_width = clamp(
            float(self.max_line_width_bottom_fraction),
            min_bottom_width,
            1.0
        )
        num_scan_bands = max(1, int(self.num_scan_bands))
        min_path_bands = self.min_path_bands
        if self.min_valid_bands is not None:
            min_path_bands = self.min_valid_bands
        min_path_bands = clamp(int(min_path_bands), 1, num_scan_bands)

        return LineTrackerConfig(
            use_full_frame_roi=bool(self.use_full_frame_roi),
            roi_top_fraction=clamp(float(self.roi_top_fraction), 0.0, 0.95),
            roi_bottom_margin_fraction=clamp(
                float(self.roi_bottom_margin_fraction),
                0.0,
                0.45
            ),
            roi_left_margin_fraction=clamp(
                float(self.roi_left_margin_fraction),
                0.0,
                0.45
            ),
            roi_right_margin_fraction=clamp(
                float(self.roi_right_margin_fraction),
                0.0,
                0.45
            ),
            threshold_value=int(clamp(int(self.threshold_value), 0, 255)),
            max_lateral_error=max(0.01, float(self.max_lateral_error)),
            line_width_cm=max(0.1, float(self.line_width_cm)),
            num_scan_bands=num_scan_bands,
            min_path_bands=min_path_bands,
            min_valid_bands=min_path_bands,
            require_bottom_band=bool(self.require_bottom_band),
            min_line_width_fraction=min_width,
            max_line_width_fraction=max_width,
            perspective_width_enabled=bool(self.perspective_width_enabled),
            min_line_width_top_fraction=min_top_width,
            min_line_width_bottom_fraction=min_bottom_width,
            max_line_width_top_fraction=max_top_width,
            max_line_width_bottom_fraction=max_bottom_width,
            max_dark_fraction=clamp(float(self.max_dark_fraction), 0.01, 1.0),
            visible_min_confidence=clamp(
                float(self.visible_min_confidence),
                0.0,
                1.0
            ),
            bottom_band_preference_weight=max(
                0.0,
                float(self.bottom_band_preference_weight)
            ),
            previous_center_weight=max(
                0.0,
                float(self.previous_center_weight)
            ),
            bottom_start_max_center_error_fraction=clamp(
                float(self.bottom_start_max_center_error_fraction),
                0.0,
                1.0
            ),
            max_band_center_jump_fraction=clamp(
                float(self.max_band_center_jump_fraction),
                0.0,
                1.0
            ),
            max_band_gap=max(1, int(self.max_band_gap)),
        )


@dataclass(frozen=True)
class ScanBandRow:
    index: int
    y: int
    y_min: int
    y_max: int


@dataclass(frozen=True)
class ScanCandidate:
    band_index: int
    y: int
    y_min: int
    y_max: int
    x_start: int
    x_end: int
    center_x: float
    width_px: int
    accepted: bool
    reason: str


@dataclass(frozen=True)
class LineDetectionResult:
    binary: np.ndarray
    roi_start_y: int
    line_visible: bool
    lateral_error: float
    heading_error: float
    confidence: float
    reason: str
    dark_fraction: float
    band_rows: Sequence[ScanBandRow]
    candidates: Sequence[ScanCandidate]
    selected_bands: Sequence[ScanCandidate]
    fitted_line: Optional[Tuple[int, int, int, int]]
    roi_start_x: int = 0
    roi_end_x: Optional[int] = None
    roi_end_y: Optional[int] = None
    preferred_center_x: Optional[float] = None
    robot_center_x: Optional[float] = None
    tracking_anchor_x: Optional[float] = None
    current_bottom_x: Optional[float] = None
    last_bottom_x: Optional[float] = None
    pending_bottom_x: Optional[float] = None
    pending_stable_count: int = 0
    track_lock_enabled: bool = False
    lost_frame_count: int = 0
    bottom_band_valid: bool = False
    candidate_rejected: bool = False
    candidate_rejection_reason: str = 'none'
    track_jump_rejected: bool = False


@dataclass(frozen=True)
class SpecialDetectionResult:
    target_type: str
    visible: bool = False
    confidence: float = 0.0
    center_x: float = 0.0
    center_y: float = 0.0
    area_ratio: float = 0.0
    width_ratio: float = 0.0
    height_ratio: float = 0.0
    inside_candidate: bool = False
    direction_hint: str = 'unknown'
    reason: str = 'not_detected'


def detect_red_circle(
    image,
    min_area_ratio=0.002,
    min_circularity=0.55
):
    """Return visual evidence for the largest red circular region."""
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    low_red = cv2.inRange(hsv, (0, 80, 60), (12, 255, 255))
    high_red = cv2.inRange(hsv, (170, 80, 60), (180, 255, 255))
    mask = cv2.bitwise_or(low_red, high_red)
    mask = _clean_special_mask(mask)
    return _best_special_contour(
        mask,
        image.shape,
        'red_circle',
        min_area_ratio,
        shape_filter='circle',
        min_circularity=min_circularity
    )


def detect_blue_stop_zone(
    image,
    robot_center_x,
    blue_h_min=90,
    blue_h_max=130,
    blue_s_min=60,
    blue_v_min=40,
    min_area_ratio=0.05,
    reference_y_ratio=0.85
):
    """Return visual evidence for a blue floor stop zone."""
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower = (
        int(clamp(blue_h_min, 0, 180)),
        int(clamp(blue_s_min, 0, 255)),
        int(clamp(blue_v_min, 0, 255)),
    )
    upper = (
        int(clamp(blue_h_max, lower[0], 180)),
        255,
        255,
    )
    mask = _clean_special_mask(cv2.inRange(hsv, lower, upper), 7)
    return _best_special_contour(
        mask,
        image.shape,
        'blue_stop_zone',
        min_area_ratio,
        reference_point=(
            float(robot_center_x),
            float(image.shape[0]) * clamp(reference_y_ratio, 0.0, 1.0),
        )
    )


def detect_white_bar(
    image,
    white_v_min=180,
    white_s_max=80,
    min_width_ratio=0.20,
    max_height_ratio=0.15,
    min_area_ratio=0.002
):
    """Return visual evidence for a bright horizontal route marker."""
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(
        hsv,
        (0, 0, int(clamp(white_v_min, 0, 255))),
        (180, int(clamp(white_s_max, 0, 255)), 255)
    )
    mask = _clean_special_mask(mask, 5)
    return _best_special_contour(
        mask,
        image.shape,
        'white_bar',
        min_area_ratio,
        shape_filter='horizontal',
        min_width_ratio=min_width_ratio,
        max_height_ratio=max_height_ratio
    )


def detect_corner_candidate(
    line_result,
    image_width,
    min_heading_error=0.30,
    edge_fraction=0.28
):
    """Publish a geometric corner hint; mission decides whether to turn."""
    if line_result is None:
        return SpecialDetectionResult(
            target_type='corner_candidate',
            reason='line_result_missing'
        )

    valid_ratio = (
        float(len(line_result.selected_bands))
        / max(1.0, float(len(line_result.band_rows)))
    )
    heading_score = clamp(
        abs(float(line_result.heading_error))
        / max(0.01, float(min_heading_error)),
        0.0,
        1.0
    )
    sparse_upper_path = (
        bool(line_result.bottom_band_valid)
        and valid_ratio < 0.55
    )
    rejection_hint = (
        'too_wide' in str(line_result.reason)
        or 'too_wide' in str(line_result.candidate_rejection_reason)
        or 'obstacle_like_dark_block' in str(line_result.reason)
    )
    anchor = line_result.tracking_anchor_x
    direction_hint = 'unknown'
    edge_score = 0.0
    if anchor is not None and image_width > 0:
        normalized_x = float(anchor) / float(image_width)
        if normalized_x < edge_fraction:
            direction_hint = 'left'
            edge_score = clamp(
                (edge_fraction - normalized_x) / max(edge_fraction, 0.01),
                0.0,
                1.0
            )
        elif normalized_x > 1.0 - edge_fraction:
            direction_hint = 'right'
            edge_score = clamp(
                (normalized_x - (1.0 - edge_fraction))
                / max(edge_fraction, 0.01),
                0.0,
                1.0
            )
        elif float(line_result.heading_error) > min_heading_error:
            direction_hint = 'right'
        elif float(line_result.heading_error) < -min_heading_error:
            direction_hint = 'left'

    visible = bool(
        line_result.bottom_band_valid
        and (
            abs(float(line_result.heading_error)) >= min_heading_error
            or (sparse_upper_path and edge_score > 0.0)
            or rejection_hint
        )
    )
    confidence = clamp(
        0.45 * heading_score
        + 0.35 * edge_score
        + (0.20 if sparse_upper_path or rejection_hint else 0.0),
        0.0,
        1.0
    )
    reason_parts = []
    if sparse_upper_path:
        reason_parts.append('upper_path_sparse')
    if rejection_hint:
        reason_parts.append('wide_or_block_rejection')
    if heading_score >= 1.0:
        reason_parts.append('large_heading')
    if edge_score > 0.0:
        reason_parts.append('anchor_near_edge')
    return SpecialDetectionResult(
        target_type='corner_candidate',
        visible=visible,
        confidence=confidence if visible else 0.0,
        center_x=(
            clamp(float(anchor) / float(image_width), 0.0, 1.0)
            if anchor is not None and image_width > 0 else 0.0
        ),
        direction_hint=direction_hint,
        reason=','.join(reason_parts) if reason_parts else 'not_candidate'
    )


def _clean_special_mask(mask, kernel_size=5):
    kernel_size = max(3, int(kernel_size) | 1)
    kernel = np.ones((kernel_size, kernel_size), dtype=np.uint8)
    opened = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    return cv2.morphologyEx(opened, cv2.MORPH_CLOSE, kernel)


def _best_special_contour(
    mask,
    image_shape,
    target_type,
    min_area_ratio,
    shape_filter=None,
    min_circularity=0.0,
    min_width_ratio=0.0,
    max_height_ratio=1.0,
    reference_point=None
):
    height, width = image_shape[:2]
    image_area = max(1.0, float(height * width))
    contours, _ = cv2.findContours(
        mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )
    best = None
    best_score = -1.0
    rejection_reason = 'not_detected'
    for contour in contours:
        area = float(cv2.contourArea(contour))
        area_ratio = area / image_area
        if area_ratio < max(0.0, float(min_area_ratio)):
            rejection_reason = 'area_too_small'
            continue
        x, y, box_width, box_height = cv2.boundingRect(contour)
        width_ratio = float(box_width) / max(1.0, float(width))
        height_ratio = float(box_height) / max(1.0, float(height))
        shape_score = 1.0
        if shape_filter == 'circle':
            perimeter = float(cv2.arcLength(contour, True))
            circularity = (
                4.0 * math.pi * area / (perimeter * perimeter)
                if perimeter > 1.0 else 0.0
            )
            aspect = (
                float(min(box_width, box_height))
                / max(1.0, float(max(box_width, box_height)))
            )
            if circularity < min_circularity or aspect < 0.65:
                rejection_reason = 'not_circular'
                continue
            shape_score = 0.65 * circularity + 0.35 * aspect
        elif shape_filter == 'horizontal':
            if (
                width_ratio < min_width_ratio
                or height_ratio > max_height_ratio
                or box_width <= box_height
            ):
                rejection_reason = 'not_horizontal_bar'
                continue
            shape_score = clamp(
                float(box_width) / max(1.0, float(box_height)) / 8.0,
                0.0,
                1.0
            )

        score = area_ratio + 0.1 * shape_score
        if score > best_score:
            best_score = score
            best = (
                contour,
                area_ratio,
                width_ratio,
                height_ratio,
                x,
                y,
                box_width,
                box_height,
                shape_score,
            )

    if best is None:
        return SpecialDetectionResult(
            target_type=target_type,
            reason=rejection_reason
        )

    (
        contour,
        area_ratio,
        width_ratio,
        height_ratio,
        x,
        y,
        box_width,
        box_height,
        shape_score,
    ) = best
    inside_candidate = False
    if reference_point is not None:
        inside_candidate = cv2.pointPolygonTest(
            contour,
            reference_point,
            False
        ) >= 0.0
    confidence = clamp(
        0.55 * shape_score
        + 0.45 * clamp(
            area_ratio / max(float(min_area_ratio), 0.001),
            0.0,
            1.0
        ),
        0.0,
        1.0
    )
    return SpecialDetectionResult(
        target_type=target_type,
        visible=True,
        confidence=confidence,
        center_x=clamp(
            (float(x) + float(box_width) / 2.0) / max(1.0, float(width)),
            0.0,
            1.0
        ),
        center_y=clamp(
            (float(y) + float(box_height) / 2.0)
            / max(1.0, float(height)),
            0.0,
            1.0
        ),
        area_ratio=area_ratio,
        width_ratio=width_ratio,
        height_ratio=height_ratio,
        inside_candidate=inside_candidate,
        reason='ok'
    )


def detect_line_in_image(
    image,
    config,
    preferred_center_x=None,
    robot_center_x=None,
    max_track_jump_fraction=0.30
):
    config = config.normalized()
    height, width = image.shape[:2]
    robot_center_x = _normalize_preferred_center(
        robot_center_x,
        width
    )
    preferred_center_x = _normalize_preferred_center(
        preferred_center_x,
        width
    )
    roi_start_x, roi_start_y, roi_end_x, roi_end_y = _effective_roi_bounds(
        height,
        width,
        config
    )
    roi_bounds = {
        'roi_start_x': roi_start_x,
        'roi_end_x': roi_end_x,
        'roi_end_y': roi_end_y,
    }
    roi = image[roi_start_y:roi_end_y, roi_start_x:roi_end_x]

    if roi.size == 0:
        return _lost_result(
            np.zeros((1, max(1, width)), dtype='uint8'),
            roi_start_y,
            'empty_roi',
            preferred_center_x=preferred_center_x,
            robot_center_x=robot_center_x,
            **roi_bounds
        )

    binary = _make_binary_mask(roi, config.threshold_value)
    dark_fraction = float(cv2.countNonZero(binary)) / float(binary.size)
    band_rows, candidates_by_band, candidates = _scan_line_candidates(
        binary,
        config,
        roi_start_x
    )
    bottom_band_valid = _bottom_band_valid(candidates)

    if dark_fraction > config.max_dark_fraction:
        return _lost_result(
            binary,
            roi_start_y,
            'too_dark',
            dark_fraction,
            band_rows,
            candidates,
            preferred_center_x=preferred_center_x,
            robot_center_x=robot_center_x,
            **roi_bounds
        )

    selected_bands = _select_best_band_path(
        candidates_by_band,
        width,
        config.require_bottom_band,
        preferred_center_x,
        max_track_jump_fraction,
        config.bottom_band_preference_weight,
        config.previous_center_weight,
        config.bottom_start_max_center_error_fraction,
        config.max_band_center_jump_fraction,
        config.max_band_gap
    )

    if config.require_bottom_band and not candidates_by_band[0]:
        reason = _candidate_rejection_reason(
            candidates,
            fallback='bottom_missing'
        )
        return _lost_result(
            binary,
            roi_start_y,
            reason,
            dark_fraction,
            band_rows,
            candidates,
            preferred_center_x=preferred_center_x,
            robot_center_x=robot_center_x,
            **roi_bounds
        )

    if len(selected_bands) < config.min_path_bands:
        reason = _candidate_rejection_reason(
            candidates,
            fallback='not_enough_bands'
        )
        return _lost_result(
            binary,
            roi_start_y,
            reason,
            dark_fraction,
            band_rows,
            candidates,
            selected_bands,
            preferred_center_x=preferred_center_x,
            robot_center_x=robot_center_x,
            **roi_bounds
        )

    fitted_line = _fit_line_to_bands(selected_bands, binary.shape[0])
    if fitted_line is None:
        return _lost_result(
            binary,
            roi_start_y,
            'fit_failed',
            dark_fraction,
            band_rows,
            candidates,
            selected_bands,
            preferred_center_x=preferred_center_x,
            robot_center_x=robot_center_x,
            **roi_bounds
        )

    x_top, y_top, x_bottom, y_bottom, slope = fitted_line
    tracking_anchor_x = _select_lateral_anchor_x(
        selected_bands,
        preferred_center_x,
        width
    )
    if tracking_anchor_x is None:
        tracking_anchor_x = float(x_bottom)
    image_center_x = width / 2.0
    error_scale_x = max(1.0, image_center_x)
    lateral_error = (
        (float(tracking_anchor_x) - robot_center_x) / error_scale_x
        if error_scale_x > 0.0 else 0.0
    )
    lateral_error = clamp(
        lateral_error,
        -config.max_lateral_error,
        config.max_lateral_error
    )
    heading_error = _heading_error_from_path(selected_bands, slope)
    confidence = _compute_confidence(len(selected_bands), config)

    if confidence < config.visible_min_confidence:
        return _lost_result(
            binary,
            roi_start_y,
            'confidence_low',
            dark_fraction,
            band_rows,
            candidates,
            selected_bands,
            (x_top, y_top, x_bottom, y_bottom),
            preferred_center_x=preferred_center_x,
            robot_center_x=robot_center_x,
            tracking_anchor_x=tracking_anchor_x,
            current_bottom_x=tracking_anchor_x,
            **roi_bounds
        )

    return LineDetectionResult(
        binary=binary,
        roi_start_y=roi_start_y,
        line_visible=True,
        lateral_error=lateral_error,
        heading_error=heading_error,
        confidence=confidence,
        reason='ok',
        dark_fraction=dark_fraction,
        band_rows=band_rows,
        candidates=candidates,
        selected_bands=selected_bands,
        fitted_line=(x_top, y_top, x_bottom, y_bottom),
        roi_start_x=roi_start_x,
        roi_end_x=roi_end_x,
        roi_end_y=roi_end_y,
        preferred_center_x=preferred_center_x,
        robot_center_x=robot_center_x,
        tracking_anchor_x=tracking_anchor_x,
        current_bottom_x=tracking_anchor_x,
        bottom_band_valid=bottom_band_valid,
        candidate_rejected=False,
        candidate_rejection_reason='none'
    )


def _bottom_band_valid(candidates):
    return any(
        candidate.accepted and candidate.band_index == 0
        for candidate in candidates
    )


def _effective_roi_bounds(height, width, config):
    height = max(0, int(height))
    width = max(0, int(width))
    if height <= 0 or width <= 0:
        return 0, 0, max(0, width), max(0, height)

    roi_start_y = int(round(height * config.roi_top_fraction))
    roi_start_x = 0
    roi_end_x = width
    roi_end_y = height

    if config.use_full_frame_roi:
        roi_start_x = int(round(width * config.roi_left_margin_fraction))
        roi_end_x = width - int(
            round(width * config.roi_right_margin_fraction)
        )
        roi_end_y = height - int(
            round(height * config.roi_bottom_margin_fraction)
        )

    roi_start_y = int(clamp(roi_start_y, 0, max(0, height - 1)))
    roi_end_y = int(clamp(roi_end_y, roi_start_y + 1, height))
    roi_start_x = int(clamp(roi_start_x, 0, max(0, width - 1)))
    roi_end_x = int(clamp(roi_end_x, roi_start_x + 1, width))
    return roi_start_x, roi_start_y, roi_end_x, roi_end_y


def _normalize_preferred_center(preferred_center_x, image_width):
    if image_width <= 0:
        return 0.0
    if preferred_center_x is None:
        return image_width / 2.0
    try:
        center_x = float(preferred_center_x)
    except (TypeError, ValueError):
        center_x = image_width / 2.0
    if not math.isfinite(center_x):
        center_x = image_width / 2.0
    return clamp(center_x, 0.0, max(0.0, float(image_width - 1)))


def _make_binary_mask(roi, threshold_value):
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    _, binary = cv2.threshold(
        blurred,
        threshold_value,
        255,
        cv2.THRESH_BINARY_INV
    )
    kernel = np.ones((3, 3), dtype='uint8')
    binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
    binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)
    return binary


def _scan_line_candidates(binary, config, x_offset=0):
    roi_height, width = binary.shape[:2]
    x_offset = int(x_offset)
    band_height = max(3, roi_height // max(1, config.num_scan_bands * 4))
    if band_height % 2 == 0:
        band_height += 1

    bottom_y = max(0, roi_height - 1 - band_height // 2)
    top_y = min(roi_height - 1, band_height // 2)
    if config.num_scan_bands == 1:
        y_values = [roi_height // 2]
    else:
        y_values = np.linspace(bottom_y, top_y, config.num_scan_bands)

    band_rows = []
    candidates_by_band = []
    candidates = []

    for index, y_value in enumerate(y_values):
        y = int(round(float(y_value)))
        y_min = max(0, y - band_height // 2)
        y_max = min(roi_height, y + band_height // 2 + 1)
        row = ScanBandRow(index=index, y=y, y_min=y_min, y_max=y_max)
        band_rows.append(row)

        strip = binary[y_min:y_max, :]
        min_dark_rows = max(1, int(math.ceil(strip.shape[0] * 0.35)))
        dark_columns = np.count_nonzero(strip, axis=0) >= min_dark_rows
        segments = _segments_from_mask(dark_columns)
        band_candidates = []

        for x_start, x_end in segments:
            width_px = x_end - x_start
            min_width_px, max_width_px = _allowed_width_px(
                index,
                config,
                width
            )
            accepted = min_width_px <= width_px <= max_width_px
            reason = 'ok'
            if width_px < min_width_px:
                reason = 'too_narrow'
            elif width_px > max_width_px:
                reason = 'too_wide'

            candidate = ScanCandidate(
                band_index=index,
                y=y,
                y_min=y_min,
                y_max=y_max,
                x_start=x_offset + x_start,
                x_end=x_offset + x_end,
                center_x=x_offset + (x_start + x_end - 1) / 2.0,
                width_px=width_px,
                accepted=accepted,
                reason=reason
            )
            candidates.append(candidate)
            if accepted:
                band_candidates.append(candidate)

        candidates_by_band.append(band_candidates)

    return band_rows, candidates_by_band, candidates


def _allowed_width_px(band_index, config, image_width):
    if not config.perspective_width_enabled:
        min_fraction = config.min_line_width_fraction
        max_fraction = config.max_line_width_fraction
    else:
        max_index = max(1, config.num_scan_bands - 1)
        top_weight = clamp(float(band_index) / float(max_index), 0.0, 1.0)
        bottom_weight = 1.0 - top_weight
        min_fraction = (
            config.min_line_width_bottom_fraction * bottom_weight
            + config.min_line_width_top_fraction * top_weight
        )
        max_fraction = (
            config.max_line_width_bottom_fraction * bottom_weight
            + config.max_line_width_top_fraction * top_weight
        )

    min_width_px = max(1, int(round(image_width * min_fraction)))
    max_width_px = max(min_width_px, int(round(image_width * max_fraction)))
    return min_width_px, max_width_px


def _segments_from_mask(mask):
    segments = []
    start = None

    for index, value in enumerate(mask):
        if bool(value) and start is None:
            start = index
        elif not bool(value) and start is not None:
            segments.append((start, index))
            start = None

    if start is not None:
        segments.append((start, len(mask)))

    return segments


def _select_best_band_path(
    candidates_by_band,
    image_width,
    require_bottom_band,
    preferred_center_x=None,
    max_track_jump_fraction=0.30,
    bottom_band_preference_weight=2.0,
    previous_center_weight=2.0,
    bottom_start_max_center_error_fraction=1.0,
    max_band_center_jump_fraction=0.30,
    max_band_gap=2
):
    start_candidates = []
    center_x = _normalize_preferred_center(preferred_center_x, image_width)
    bottom_start_limit = (
        image_width * max(0.0, float(bottom_start_max_center_error_fraction))
    )
    if require_bottom_band:
        if candidates_by_band:
            start_candidates = [
                (0, candidate)
                for candidate in candidates_by_band[0]
                if abs(candidate.center_x - center_x) <= bottom_start_limit
            ]
    else:
        for band_index in range(len(candidates_by_band)):
            candidates = candidates_by_band[band_index]
            start_candidates.extend(
                (band_index, candidate)
                for candidate in candidates
            )

    if not start_candidates:
        return []

    best_path = []
    best_cost = float('inf')
    max_track_jump_fraction = max(0.0, float(max_track_jump_fraction))
    max_band_center_jump_fraction = max(
        0.0,
        float(max_band_center_jump_fraction)
    )
    max_band_gap = max(1, int(max_band_gap))
    bottom_band_preference_weight = max(
        0.0,
        float(bottom_band_preference_weight)
    )
    previous_center_weight = max(0.0, float(previous_center_weight))

    for start_index, start_candidate in start_candidates:
        path = [start_candidate]
        cost = (
            abs(start_candidate.center_x - center_x)
            * previous_center_weight
        )
        cost += (
            float(start_index)
            * float(image_width)
            * bottom_band_preference_weight
        )
        last_candidate = start_candidate

        for candidates in candidates_by_band[start_index + 1:]:
            if not candidates:
                continue

            max_center_jump = max(
                image_width * max_band_center_jump_fraction,
                last_candidate.width_px * 2.0
            )
            max_center_jump = min(
                max_center_jump,
                image_width * max_track_jump_fraction
            )
            eligible = []
            for candidate in candidates:
                band_gap = candidate.band_index - last_candidate.band_index
                if band_gap > max_band_gap:
                    continue
                center_delta = abs(
                    candidate.center_x - last_candidate.center_x
                )
                if center_delta <= max_center_jump:
                    eligible.append(candidate)
            if not eligible:
                continue

            next_candidate = min(
                eligible,
                key=lambda candidate: (
                    abs(candidate.center_x - last_candidate.center_x),
                    abs(candidate.center_x - center_x)
                )
            )
            cost += abs(next_candidate.center_x - last_candidate.center_x)
            cost += (
                abs(next_candidate.center_x - center_x)
                * previous_center_weight
            )
            path.append(next_candidate)
            last_candidate = next_candidate

        if len(path) > len(best_path) or (
            len(path) == len(best_path) and cost < best_cost
        ):
            best_path = path
            best_cost = cost

    return best_path


def _select_lateral_anchor_x(selected_bands, preferred_center_x, image_width):
    if not selected_bands:
        return None

    center_x = _normalize_preferred_center(preferred_center_x, image_width)
    top_y = min(candidate.y for candidate in selected_bands)
    bottom_y = max(candidate.y for candidate in selected_bands)
    path_height = max(1, bottom_y - top_y)
    bottom_window_min_y = bottom_y - max(1, int(round(path_height * 0.25)))
    anchor_candidates = [
        candidate
        for candidate in selected_bands
        if candidate.y >= bottom_window_min_y
    ]
    if not anchor_candidates:
        anchor_candidates = list(selected_bands)

    anchor = min(
        anchor_candidates,
        key=lambda candidate: (
            abs(candidate.center_x - center_x),
            -candidate.y
        )
    )
    return float(anchor.center_x)


def _heading_error_from_path(selected_bands, fallback_slope):
    if selected_bands:
        top_candidate = min(selected_bands, key=lambda candidate: candidate.y)
        bottom_candidate = max(
            selected_bands,
            key=lambda candidate: candidate.y
        )
        delta_y = float(bottom_candidate.y - top_candidate.y)
        if abs(delta_y) >= 1.0:
            slope = (
                float(bottom_candidate.center_x - top_candidate.center_x)
                / delta_y
            )
            return clamp(math.atan(slope), -math.pi / 2.0, math.pi / 2.0)

    return clamp(
        math.atan(float(fallback_slope)),
        -math.pi / 2.0,
        math.pi / 2.0
    )


def _fit_line_to_bands(selected_bands, roi_height):
    if len(selected_bands) < 2:
        return None

    points_y = np.array(
        [float(candidate.y) for candidate in selected_bands],
        dtype=np.float64
    )
    points_x = np.array(
        [float(candidate.center_x) for candidate in selected_bands],
        dtype=np.float64
    )

    try:
        slope, intercept = np.polyfit(points_y, points_x, 1)
    except (TypeError, ValueError, np.linalg.LinAlgError):
        return None

    if not math.isfinite(float(slope)) or not math.isfinite(float(intercept)):
        return None

    y_top = 0
    y_bottom = max(0, roi_height - 1)
    x_top = int(round(slope * y_top + intercept))
    x_bottom = int(round(slope * y_bottom + intercept))
    return x_top, y_top, x_bottom, y_bottom, float(slope)


def _compute_confidence(valid_band_count, config):
    band_score = float(valid_band_count) / float(config.num_scan_bands)
    return clamp(0.25 + 0.75 * band_score, 0.0, 1.0)


def _candidate_rejection_reason(candidates, fallback):
    if any(candidate.reason == 'too_wide' for candidate in candidates):
        return 'too_wide'
    if any(candidate.reason == 'too_narrow' for candidate in candidates):
        return 'too_narrow'
    return fallback


def _lost_result(
    binary,
    roi_start_y,
    reason,
    dark_fraction=0.0,
    band_rows=(),
    candidates=(),
    selected_bands=(),
    fitted_line=None,
    roi_start_x=0,
    roi_end_x=None,
    roi_end_y=None,
    preferred_center_x=None,
    robot_center_x=None,
    tracking_anchor_x=None,
    current_bottom_x=None,
    last_bottom_x=None,
    pending_bottom_x=None,
    pending_stable_count=0,
    track_lock_enabled=False,
    lost_frame_count=0,
    bottom_band_valid=None,
    candidate_rejected=True,
    candidate_rejection_reason=None,
    track_jump_rejected=False
):
    if bottom_band_valid is None:
        bottom_band_valid = _bottom_band_valid(candidates)
    if candidate_rejection_reason is None:
        candidate_rejection_reason = reason
    return LineDetectionResult(
        binary=binary,
        roi_start_y=roi_start_y,
        line_visible=False,
        lateral_error=0.0,
        heading_error=0.0,
        confidence=0.0,
        reason=reason,
        dark_fraction=dark_fraction,
        band_rows=band_rows,
        candidates=candidates,
        selected_bands=selected_bands,
        fitted_line=fitted_line,
        roi_start_x=roi_start_x,
        roi_end_x=roi_end_x,
        roi_end_y=roi_end_y,
        preferred_center_x=preferred_center_x,
        robot_center_x=robot_center_x,
        tracking_anchor_x=tracking_anchor_x,
        current_bottom_x=current_bottom_x,
        last_bottom_x=last_bottom_x,
        pending_bottom_x=pending_bottom_x,
        pending_stable_count=pending_stable_count,
        track_lock_enabled=track_lock_enabled,
        lost_frame_count=lost_frame_count,
        bottom_band_valid=bool(bottom_band_valid),
        candidate_rejected=bool(candidate_rejected),
        candidate_rejection_reason=str(candidate_rejection_reason),
        track_jump_rejected=track_jump_rejected
    )


class RealLineTrackerNode(Node):
    """Track a 10 cm black floor line from a RealSense RGB image."""

    def __init__(self):
        super().__init__('real_line_tracker_node')

        self.declare_parameter(
            'image_topic',
            '/camera/color/image_raw'
        )
        self.declare_parameter('line_track_topic', '/perception/line_track')
        self.declare_parameter(
            'red_circle_topic',
            '/perception/red_circle_detection'
        )
        self.declare_parameter(
            'stop_zone_topic',
            '/perception/stop_zone_detection'
        )
        self.declare_parameter(
            'white_bar_topic',
            '/perception/white_bar_detection'
        )
        self.declare_parameter(
            'corner_candidate_topic',
            '/perception/corner_candidate'
        )
        self.declare_parameter('enable_debug_image', True)
        self.declare_parameter('debug_log', True)
        self.declare_parameter('use_full_frame_roi', True)
        self.declare_parameter('roi_top_fraction', 0.05)
        self.declare_parameter('roi_bottom_margin_fraction', 0.03)
        self.declare_parameter('roi_left_margin_fraction', 0.03)
        self.declare_parameter('roi_right_margin_fraction', 0.03)
        self.declare_parameter('threshold_value', 80)
        self.declare_parameter('max_lateral_error', 1.0)
        self.declare_parameter('robot_center_x_offset_fraction', 0.0)
        self.declare_parameter('robot_center_x_offset_px', 0.0)
        self.declare_parameter('line_width_cm', 10.0)
        self.declare_parameter('num_scan_bands', 11)
        self.declare_parameter('min_path_bands', 3)
        self.declare_parameter('min_valid_bands', 3)
        self.declare_parameter('require_bottom_band', False)
        self.declare_parameter('min_line_width_fraction', 0.015)
        self.declare_parameter('max_line_width_fraction', 0.20)
        self.declare_parameter('perspective_width_enabled', False)
        self.declare_parameter('min_line_width_top_fraction', 0.006)
        self.declare_parameter('min_line_width_bottom_fraction', 0.015)
        self.declare_parameter('max_line_width_top_fraction', 0.10)
        self.declare_parameter('max_line_width_bottom_fraction', 0.30)
        self.declare_parameter('max_dark_fraction', 0.35)
        self.declare_parameter('visible_min_confidence', 0.45)
        self.declare_parameter('track_lock_enabled', True)
        self.declare_parameter('max_track_jump_fraction', 0.30)
        self.declare_parameter('max_reacquire_jump_fraction', 0.20)
        self.declare_parameter('bottom_band_preference_weight', 2.0)
        self.declare_parameter('previous_center_weight', 2.0)
        self.declare_parameter('bottom_start_max_center_error_fraction', 1.0)
        self.declare_parameter('max_band_center_jump_fraction', 0.30)
        self.declare_parameter('max_band_gap', 2)
        self.declare_parameter('frame_id', 'd435i_color_optical_frame')
        self.declare_parameter('red_circle_min_area_ratio', 0.002)
        self.declare_parameter('red_circle_min_circularity', 0.55)
        self.declare_parameter('blue_h_min', 90)
        self.declare_parameter('blue_h_max', 130)
        self.declare_parameter('blue_s_min', 60)
        self.declare_parameter('blue_v_min', 40)
        self.declare_parameter('stop_zone_min_area_ratio', 0.05)
        self.declare_parameter('stop_zone_reference_y_ratio', 0.85)
        self.declare_parameter('white_bar_v_min', 180)
        self.declare_parameter('white_bar_s_max', 80)
        self.declare_parameter('white_bar_min_width_ratio', 0.20)
        self.declare_parameter('white_bar_max_height_ratio', 0.15)
        self.declare_parameter('white_bar_min_area_ratio', 0.002)
        self.declare_parameter('corner_min_heading_error', 0.30)
        self.declare_parameter('corner_edge_fraction', 0.28)

        self.image_topic = self.get_parameter('image_topic').value
        self.enable_debug_image = self._get_bool_parameter(
            'enable_debug_image',
            True
        )
        self.debug_log = self._get_bool_parameter('debug_log', True)
        self.line_track_topic = self.get_parameter(
            'line_track_topic'
        ).get_parameter_value().string_value
        self.red_circle_topic = str(
            self.get_parameter('red_circle_topic').value
        )
        self.stop_zone_topic = str(
            self.get_parameter('stop_zone_topic').value
        )
        self.white_bar_topic = str(
            self.get_parameter('white_bar_topic').value
        )
        self.corner_candidate_topic = str(
            self.get_parameter('corner_candidate_topic').value
        )
        self.frame_id = self.get_parameter(
            'frame_id'
        ).get_parameter_value().string_value

        self.bridge = CvBridge()
        self.last_debug_log_ns = 0
        self.debug_log_period_ns = 1_000_000_000
        self.last_line_visible = False
        self.last_bottom_x = None
        self.last_slope = None
        self.lost_frame_count = 0
        self.last_result = None
        self.pending_bottom_x = None
        self.pending_stable_count = 0
        self.refresh_parameters()

        self.publisher = self.create_publisher(
            LineTrack,
            self.line_track_topic,
            10
        )
        self.red_circle_publisher = self.create_publisher(
            SpecialTargetDetection,
            self.red_circle_topic,
            10
        )
        self.stop_zone_publisher = self.create_publisher(
            SpecialTargetDetection,
            self.stop_zone_topic,
            10
        )
        self.white_bar_publisher = self.create_publisher(
            SpecialTargetDetection,
            self.white_bar_topic,
            10
        )
        self.corner_candidate_publisher = self.create_publisher(
            SpecialTargetDetection,
            self.corner_candidate_topic,
            10
        )
        self.mask_pub = self.create_publisher(
            Image,
            '/perception/debug/line_mask',
            1
        )
        self.overlay_pub = self.create_publisher(
            Image,
            '/perception/debug/line_overlay',
            1
        )
        self.mask_publisher = self.mask_pub
        self.overlay_publisher = self.overlay_pub
        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            qos_profile_sensor_data
        )
        self.subscription = self.image_sub

        self.get_logger().info(
            'Real line tracker node started: '
            f'image_topic={self.image_topic}, '
            f'line_track_topic={self.line_track_topic}, '
            f'special_topics=[{self.red_circle_topic}, '
            f'{self.stop_zone_topic}, {self.white_bar_topic}, '
            f'{self.corner_candidate_topic}], '
            f'enable_debug_image={self.enable_debug_image}, '
            f'debug_log={self.debug_log}, '
            'robot_center_offset='
            f'{self.robot_center_x_offset_fraction:.3f}+'
            f'{self.robot_center_x_offset_px:.1f}px, '
            'line_mask_topic=/perception/debug/line_mask, '
            'line_overlay_topic=/perception/debug/line_overlay'
        )
        raw_enable_debug_image = self.get_parameter(
            'enable_debug_image'
        ).value
        raw_debug_log = self.get_parameter('debug_log').value
        self.get_logger().info(
            'debug params: '
            f'enable_debug_image={self.enable_debug_image}, '
            f'raw_enable_debug_image={raw_enable_debug_image!r}, '
            f'type={type(raw_enable_debug_image).__name__}, '
            f'debug_log={self.debug_log}, '
            f'raw_debug_log={raw_debug_log!r}, '
            f'type={type(raw_debug_log).__name__}'
        )

    def refresh_parameters(self):
        """Refresh runtime-tunable parameters from the ROS parameter store."""
        self.enable_debug_image = self._get_bool_parameter(
            'enable_debug_image',
            True
        )
        self.debug_log = self._get_bool_parameter('debug_log', True)
        self.track_lock_enabled = self.get_parameter(
            'track_lock_enabled'
        ).get_parameter_value().bool_value
        self.max_track_jump_fraction = clamp(
            self.get_parameter(
                'max_track_jump_fraction'
            ).get_parameter_value().double_value,
            0.0,
            1.0
        )
        self.max_reacquire_jump_fraction = clamp(
            self.get_parameter(
                'max_reacquire_jump_fraction'
            ).get_parameter_value().double_value,
            0.0,
            1.0
        )
        self.robot_center_x_offset_fraction = clamp(
            self.get_parameter(
                'robot_center_x_offset_fraction'
            ).get_parameter_value().double_value,
            -0.45,
            0.45
        )
        self.robot_center_x_offset_px = self.get_parameter(
            'robot_center_x_offset_px'
        ).get_parameter_value().double_value
        self.red_circle_min_area_ratio = max(
            0.0,
            float(self.get_parameter('red_circle_min_area_ratio').value)
        )
        self.red_circle_min_circularity = clamp(
            float(self.get_parameter('red_circle_min_circularity').value),
            0.0,
            1.0
        )
        self.blue_h_min = int(self.get_parameter('blue_h_min').value)
        self.blue_h_max = int(self.get_parameter('blue_h_max').value)
        self.blue_s_min = int(self.get_parameter('blue_s_min').value)
        self.blue_v_min = int(self.get_parameter('blue_v_min').value)
        self.stop_zone_min_area_ratio = max(
            0.0,
            float(self.get_parameter('stop_zone_min_area_ratio').value)
        )
        self.stop_zone_reference_y_ratio = clamp(
            float(self.get_parameter('stop_zone_reference_y_ratio').value),
            0.0,
            1.0
        )
        self.white_bar_v_min = int(
            self.get_parameter('white_bar_v_min').value
        )
        self.white_bar_s_max = int(
            self.get_parameter('white_bar_s_max').value
        )
        self.white_bar_min_width_ratio = clamp(
            float(self.get_parameter('white_bar_min_width_ratio').value),
            0.0,
            1.0
        )
        self.white_bar_max_height_ratio = clamp(
            float(self.get_parameter('white_bar_max_height_ratio').value),
            0.0,
            1.0
        )
        self.white_bar_min_area_ratio = max(
            0.0,
            float(self.get_parameter('white_bar_min_area_ratio').value)
        )
        self.corner_min_heading_error = max(
            0.0,
            float(self.get_parameter('corner_min_heading_error').value)
        )
        self.corner_edge_fraction = clamp(
            float(self.get_parameter('corner_edge_fraction').value),
            0.0,
            0.49
        )
        self.bottom_band_preference_weight = max(
            0.0,
            self.get_parameter(
                'bottom_band_preference_weight'
            ).get_parameter_value().double_value
        )
        self.previous_center_weight = max(
            0.0,
            self.get_parameter(
                'previous_center_weight'
            ).get_parameter_value().double_value
        )
        self.tracker_config = LineTrackerConfig(
            use_full_frame_roi=self._get_bool_parameter(
                'use_full_frame_roi',
                True
            ),
            roi_top_fraction=self.get_parameter(
                'roi_top_fraction'
            ).get_parameter_value().double_value,
            roi_bottom_margin_fraction=self.get_parameter(
                'roi_bottom_margin_fraction'
            ).get_parameter_value().double_value,
            roi_left_margin_fraction=self.get_parameter(
                'roi_left_margin_fraction'
            ).get_parameter_value().double_value,
            roi_right_margin_fraction=self.get_parameter(
                'roi_right_margin_fraction'
            ).get_parameter_value().double_value,
            threshold_value=self.get_parameter(
                'threshold_value'
            ).get_parameter_value().integer_value,
            max_lateral_error=self.get_parameter(
                'max_lateral_error'
            ).get_parameter_value().double_value,
            line_width_cm=self.get_parameter(
                'line_width_cm'
            ).get_parameter_value().double_value,
            num_scan_bands=self.get_parameter(
                'num_scan_bands'
            ).get_parameter_value().integer_value,
            min_path_bands=self.effective_min_path_bands(),
            require_bottom_band=self.get_parameter(
                'require_bottom_band'
            ).get_parameter_value().bool_value,
            min_line_width_fraction=self.get_parameter(
                'min_line_width_fraction'
            ).get_parameter_value().double_value,
            max_line_width_fraction=self.get_parameter(
                'max_line_width_fraction'
            ).get_parameter_value().double_value,
            perspective_width_enabled=self._get_bool_parameter(
                'perspective_width_enabled',
                False
            ),
            min_line_width_top_fraction=self.get_parameter(
                'min_line_width_top_fraction'
            ).get_parameter_value().double_value,
            min_line_width_bottom_fraction=self.get_parameter(
                'min_line_width_bottom_fraction'
            ).get_parameter_value().double_value,
            max_line_width_top_fraction=self.get_parameter(
                'max_line_width_top_fraction'
            ).get_parameter_value().double_value,
            max_line_width_bottom_fraction=self.get_parameter(
                'max_line_width_bottom_fraction'
            ).get_parameter_value().double_value,
            max_dark_fraction=self.get_parameter(
                'max_dark_fraction'
            ).get_parameter_value().double_value,
            visible_min_confidence=self.get_parameter(
                'visible_min_confidence'
            ).get_parameter_value().double_value,
            bottom_band_preference_weight=self.bottom_band_preference_weight,
            previous_center_weight=self.previous_center_weight,
            bottom_start_max_center_error_fraction=self.get_parameter(
                'bottom_start_max_center_error_fraction'
            ).get_parameter_value().double_value,
            max_band_center_jump_fraction=self.get_parameter(
                'max_band_center_jump_fraction'
            ).get_parameter_value().double_value,
            max_band_gap=self.get_parameter(
                'max_band_gap'
            ).get_parameter_value().integer_value,
        ).normalized()

    def _get_bool_parameter(self, name: str, default: bool = True) -> bool:
        value = self.get_parameter(name).value
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            normalized = value.strip().lower()
            if normalized in ('true', '1', 'yes', 'on'):
                return True
            if normalized in ('false', '0', 'no', 'off'):
                return False
        return bool(default)

    def effective_min_path_bands(self):
        min_path_bands = self.get_parameter(
            'min_path_bands'
        ).get_parameter_value().integer_value
        legacy_min_valid_bands = self.get_parameter(
            'min_valid_bands'
        ).get_parameter_value().integer_value

        if min_path_bands == 3 and legacy_min_valid_bands != 3:
            return legacy_min_valid_bands
        return min_path_bands

    def on_image(self, image_msg):
        self.image_callback(image_msg)

    def image_callback(self, image_msg):
        self.refresh_parameters()
        stage = 'cv_bridge'
        image = None
        result = None

        try:
            image = self.bridge.imgmsg_to_cv2(
                image_msg,
                desired_encoding='bgr8'
            )
        except CvBridgeError as exc:
            self.log_image_callback_exception(
                'Failed to convert image',
                exc,
                image_msg,
                stage,
                image=image,
                result=result
            )
            self.publish_line_lost(image_msg, 'convert_failed')
            self.publish_empty_special_detections(
                image_msg,
                'convert_failed'
            )
            self.publish_fallback_debug(
                image_msg,
                image,
                result,
                reason='convert_failed',
                stage=stage
            )
            return
        except Exception as exc:
            self.log_image_callback_exception(
                'Unexpected image conversion error',
                exc,
                image_msg,
                stage,
                image=image,
                result=result
            )
            self.publish_line_lost(image_msg, 'convert_failed')
            self.publish_empty_special_detections(
                image_msg,
                'convert_failed'
            )
            self.publish_fallback_debug(
                image_msg,
                image,
                result,
                reason='convert_failed',
                stage=stage
            )
            return

        try:
            stage = 'detect_line_in_image'
            image_width = image.shape[1]
            robot_center_x = self.robot_center_x(image_width)
            preferred_center_x = self.preferred_center_x(
                image_width,
                robot_center_x
            )
            current_result = detect_line_in_image(
                image,
                self.tracker_config,
                preferred_center_x=preferred_center_x,
                robot_center_x=robot_center_x,
                max_track_jump_fraction=self.max_track_jump_fraction
            )
            stage = 'apply_route_lock'
            result = self.apply_route_lock(
                current_result,
                image_width,
                preferred_center_x
            )
            stage = 'publish_line_track'
            msg = self.make_line_track_msg(
                image_msg,
                result.lateral_error,
                result.heading_error,
                result.confidence,
                result.line_visible
            )
            self.publisher.publish(msg)
        except cv2.error as exc:
            self.log_image_callback_exception(
                'OpenCV line tracking failed',
                exc,
                image_msg,
                stage,
                image=image,
                result=result
            )
            self.publish_line_lost(image_msg, 'opencv_failed')
            self.publish_empty_special_detections(
                image_msg,
                'opencv_failed'
            )
            self.publish_fallback_debug(
                image_msg,
                image,
                result,
                reason='opencv_failed',
                stage=stage
            )
            return
        except Exception as exc:
            self.log_image_callback_exception(
                'Line tracking failed',
                exc,
                image_msg,
                stage,
                image=image,
                result=result
            )
            self.publish_line_lost(image_msg, 'tracking_failed')
            self.publish_empty_special_detections(
                image_msg,
                'tracking_failed'
            )
            self.publish_fallback_debug(
                image_msg,
                image,
                result,
                reason='tracking_failed',
                stage=stage
            )
            return

        try:
            self.publish_special_detections(image_msg, image, result)
        except (cv2.error, ValueError, TypeError) as exc:
            self.get_logger().error(
                'Special target detection failed: '
                f'{type(exc).__name__}: {exc}'
            )
            self.publish_empty_special_detections(
                image_msg,
                'special_detection_failed'
            )

        debug_status = self.publish_debug_images(
            image_msg,
            image,
            result.binary,
            None,
            result.roi_start_y,
            result=result,
            stage='debug_publish'
        )
        self.log_debug(
            result,
            image_msg=image_msg,
            image=image,
            mask=result.binary,
            debug_status=debug_status
        )

    def publish_line_lost(self, image_msg, reason='line lost'):
        self.reset_pending_candidate()
        self.last_line_visible = False
        self.lost_frame_count += 1
        msg = self.make_line_track_msg(image_msg, 0.0, 0.0, 0.0, False)
        self.publisher.publish(msg)
        self.log_debug_reason(reason)

    def publish_special_detections(self, image_msg, image, line_result):
        image_width = image.shape[1]
        robot_center_x = self.robot_center_x(image_width)
        detections = (
            (
                self.red_circle_publisher,
                detect_red_circle(
                    image,
                    self.red_circle_min_area_ratio,
                    self.red_circle_min_circularity
                ),
            ),
            (
                self.stop_zone_publisher,
                detect_blue_stop_zone(
                    image,
                    robot_center_x,
                    self.blue_h_min,
                    self.blue_h_max,
                    self.blue_s_min,
                    self.blue_v_min,
                    self.stop_zone_min_area_ratio,
                    self.stop_zone_reference_y_ratio
                ),
            ),
            (
                self.white_bar_publisher,
                detect_white_bar(
                    image,
                    self.white_bar_v_min,
                    self.white_bar_s_max,
                    self.white_bar_min_width_ratio,
                    self.white_bar_max_height_ratio,
                    self.white_bar_min_area_ratio
                ),
            ),
            (
                self.corner_candidate_publisher,
                detect_corner_candidate(
                    line_result,
                    image_width,
                    self.corner_min_heading_error,
                    self.corner_edge_fraction
                ),
            ),
        )
        for publisher, result in detections:
            publisher.publish(
                self.make_special_detection_msg(image_msg, result)
            )

    def publish_empty_special_detections(self, image_msg, reason):
        publishers = (
            (self.red_circle_publisher, 'red_circle'),
            (self.stop_zone_publisher, 'blue_stop_zone'),
            (self.white_bar_publisher, 'white_bar'),
            (self.corner_candidate_publisher, 'corner_candidate'),
        )
        for publisher, target_type in publishers:
            result = SpecialDetectionResult(
                target_type=target_type,
                reason=reason
            )
            publisher.publish(
                self.make_special_detection_msg(image_msg, result)
            )

    def make_special_detection_msg(self, image_msg, result):
        msg = SpecialTargetDetection()
        header = getattr(image_msg, 'header', None)
        if header is not None:
            msg.header.stamp = header.stamp
            msg.header.frame_id = header.frame_id or self.frame_id
        else:
            msg.header.frame_id = self.frame_id
        msg.target_type = str(result.target_type)
        msg.visible = bool(result.visible)
        msg.confidence = clamp(
            self.safe_float(result.confidence),
            0.0,
            1.0
        )
        msg.center_x = clamp(self.safe_float(result.center_x), 0.0, 1.0)
        msg.center_y = clamp(self.safe_float(result.center_y), 0.0, 1.0)
        msg.area_ratio = clamp(
            self.safe_float(result.area_ratio),
            0.0,
            1.0
        )
        msg.width_ratio = clamp(
            self.safe_float(result.width_ratio),
            0.0,
            1.0
        )
        msg.height_ratio = clamp(
            self.safe_float(result.height_ratio),
            0.0,
            1.0
        )
        msg.inside_candidate = bool(result.inside_candidate)
        msg.direction_hint = str(result.direction_hint)
        msg.reason = str(result.reason)
        return msg

    def robot_center_x(self, image_width):
        image_center_x = float(image_width) / 2.0
        center_x = (
            image_center_x
            + float(image_width) * self.robot_center_x_offset_fraction
            + self.robot_center_x_offset_px
        )
        return _normalize_preferred_center(center_x, image_width)

    def preferred_center_x(self, image_width, robot_center_x):
        if (
            self.track_lock_enabled
            and self.last_bottom_x is not None
        ):
            return _normalize_preferred_center(
                self.last_bottom_x,
                image_width
            )
        return _normalize_preferred_center(robot_center_x, image_width)

    def apply_route_lock(self, result, image_width, preferred_center_x):
        current_bottom_x = self.result_bottom_x(result)

        if not self.track_lock_enabled:
            self.reset_pending_candidate()
            self.record_published_result(result)
            return self.annotate_result(
                result,
                preferred_center_x,
                current_bottom_x,
                track_jump_rejected=False
            )

        if not result.line_visible:
            self.reset_pending_candidate()
            self.record_published_result(result)
            return self.annotate_result(
                result,
                preferred_center_x,
                current_bottom_x,
                track_jump_rejected=False
            )

        if (
            not self.last_line_visible
            and self.last_bottom_x is not None
            and current_bottom_x is not None
        ):
            reacquire_jump_threshold = (
                image_width * self.max_reacquire_jump_fraction
            )
            if abs(current_bottom_x - self.last_bottom_x) > (
                reacquire_jump_threshold
            ):
                return self.handle_reacquire_jump(
                    result,
                    preferred_center_x,
                    current_bottom_x,
                    reacquire_jump_threshold
                )

        if self.last_bottom_x is None or current_bottom_x is None:
            self.reset_pending_candidate()
            self.record_published_result(result)
            return self.annotate_result(
                result,
                preferred_center_x,
                current_bottom_x,
                track_jump_rejected=False
            )

        jump_threshold = image_width * self.max_track_jump_fraction
        if abs(current_bottom_x - self.last_bottom_x) > jump_threshold:
            return self.handle_track_jump(
                result,
                preferred_center_x,
                current_bottom_x,
                jump_threshold
            )

        self.reset_pending_candidate()
        self.record_published_result(result)
        return self.annotate_result(
            result,
            preferred_center_x,
            current_bottom_x,
            track_jump_rejected=False
        )

    def handle_reacquire_jump(
        self,
        result,
        preferred_center_x,
        current_bottom_x,
        jump_threshold
    ):
        self.reset_pending_candidate()
        lost_result = _lost_result(
            result.binary,
            result.roi_start_y,
            'reacquire_jump_rejected',
            result.dark_fraction,
            result.band_rows,
            result.candidates,
            result.selected_bands,
            result.fitted_line,
            roi_start_x=result.roi_start_x,
            roi_end_x=result.roi_end_x,
            roi_end_y=result.roi_end_y,
            preferred_center_x=preferred_center_x,
            robot_center_x=result.robot_center_x,
            tracking_anchor_x=result.tracking_anchor_x,
            current_bottom_x=current_bottom_x,
            candidate_rejection_reason=(
                f'reacquire_jump>{jump_threshold:.1f}px'
            )
        )
        self.record_published_result(lost_result)
        return self.annotate_result(
            lost_result,
            preferred_center_x,
            current_bottom_x,
            track_jump_rejected=True
        )

    def handle_track_jump(
        self,
        result,
        preferred_center_x,
        current_bottom_x,
        jump_threshold
    ):
        self.update_pending_candidate(current_bottom_x, jump_threshold)

        if self.pending_stable_count >= PENDING_SWITCH_STABLE_FRAMES:
            self.record_published_result(result)
            self.reset_pending_candidate()
            return self.annotate_result(
                result,
                preferred_center_x,
                current_bottom_x,
                track_jump_rejected=False
            )

        if self.last_result is not None and self.last_result.line_visible:
            held_result = self.make_held_result(result, current_bottom_x)
            return self.annotate_result(
                held_result,
                preferred_center_x,
                current_bottom_x,
                track_jump_rejected=True
            )

        self.lost_frame_count += 1
        lost_result = _lost_result(
            result.binary,
            result.roi_start_y,
            'track_jump_rejected',
            result.dark_fraction,
            result.band_rows,
            result.candidates,
            result.selected_bands,
            result.fitted_line,
            roi_start_x=result.roi_start_x,
            roi_end_x=result.roi_end_x,
            roi_end_y=result.roi_end_y,
            preferred_center_x=preferred_center_x,
            robot_center_x=result.robot_center_x,
            tracking_anchor_x=result.tracking_anchor_x,
            current_bottom_x=current_bottom_x
        )
        return self.annotate_result(
            lost_result,
            preferred_center_x,
            current_bottom_x,
            track_jump_rejected=True
        )

    def make_held_result(self, current_result, current_bottom_x):
        return replace(
            current_result,
            line_visible=True,
            lateral_error=self.last_result.lateral_error,
            heading_error=self.last_result.heading_error,
            confidence=self.last_result.confidence,
            reason='track_jump_rejected_hold_last',
            roi_start_x=self.last_result.roi_start_x,
            roi_end_x=self.last_result.roi_end_x,
            roi_end_y=self.last_result.roi_end_y,
            selected_bands=self.last_result.selected_bands,
            fitted_line=self.last_result.fitted_line,
            robot_center_x=current_result.robot_center_x,
            tracking_anchor_x=self.last_result.tracking_anchor_x,
            current_bottom_x=current_bottom_x,
            candidate_rejected=True,
            candidate_rejection_reason='track_jump_rejected_hold_last'
        )

    def record_published_result(self, result):
        if result.line_visible:
            self.last_result = result
            self.last_line_visible = True
            self.last_bottom_x = self.result_bottom_x(result)
            self.last_slope = self.result_slope(result)
            self.lost_frame_count = 0
        else:
            self.last_line_visible = False
            self.lost_frame_count += 1

    def reset_pending_candidate(self):
        self.pending_bottom_x = None
        self.pending_stable_count = 0

    def update_pending_candidate(self, current_bottom_x, jump_threshold):
        if (
            self.pending_bottom_x is None
            or abs(current_bottom_x - self.pending_bottom_x) > jump_threshold
        ):
            self.pending_bottom_x = current_bottom_x
            self.pending_stable_count = 1
            return

        self.pending_stable_count += 1

    def annotate_result(
        self,
        result,
        preferred_center_x,
        current_bottom_x,
        track_jump_rejected
    ):
        return replace(
            result,
            preferred_center_x=preferred_center_x,
            current_bottom_x=current_bottom_x,
            last_bottom_x=self.last_bottom_x,
            pending_bottom_x=self.pending_bottom_x,
            pending_stable_count=self.pending_stable_count,
            track_lock_enabled=self.track_lock_enabled,
            lost_frame_count=self.lost_frame_count,
            track_jump_rejected=track_jump_rejected
        )

    @staticmethod
    def result_bottom_x(result):
        if result is None:
            return None
        if result.tracking_anchor_x is not None:
            bottom_x = float(result.tracking_anchor_x)
        elif result.current_bottom_x is not None:
            bottom_x = float(result.current_bottom_x)
        elif result.fitted_line is not None:
            bottom_x = float(result.fitted_line[2])
        else:
            return None
        if not math.isfinite(bottom_x):
            return None
        return bottom_x

    @staticmethod
    def result_slope(result):
        if result is None or result.fitted_line is None:
            return None
        x_top, y_top, x_bottom, y_bottom = result.fitted_line
        delta_y = float(y_bottom - y_top)
        if abs(delta_y) < 1.0:
            return None
        return float(x_bottom - x_top) / delta_y

    def make_line_track_msg(
        self,
        image_msg,
        lateral_error,
        heading_error,
        confidence,
        line_visible
    ):
        msg = LineTrack()
        header = getattr(image_msg, 'header', None)
        if header is not None:
            msg.header.stamp = header.stamp
            msg.header.frame_id = header.frame_id or self.frame_id
        elif Header is not None:
            msg.header = Header()
            msg.header.frame_id = self.frame_id
        else:
            msg.header.frame_id = self.frame_id

        msg.lateral_error = self.safe_float(lateral_error)
        msg.heading_error = self.safe_float(heading_error)
        msg.confidence = clamp(self.safe_float(confidence), 0.0, 1.0)
        msg.line_visible = bool(line_visible)
        return msg

    @staticmethod
    def safe_float(value):
        try:
            number = float(value)
        except (TypeError, ValueError):
            return 0.0

        if not math.isfinite(number):
            return 0.0
        return number

    def make_overlay(self, image, result):
        overlay = image.copy()
        height, width = overlay.shape[:2]
        roi_start_y = result.roi_start_y
        roi_start_x = int(clamp(result.roi_start_x, 0, max(0, width - 1)))
        roi_end_x = result.roi_end_x
        if roi_end_x is None:
            roi_end_x = width
        roi_end_y = result.roi_end_y
        if roi_end_y is None:
            roi_end_y = min(height, roi_start_y + result.binary.shape[0])
        roi_end_x = int(clamp(roi_end_x, roi_start_x + 1, width))
        roi_end_y = int(clamp(roi_end_y, roi_start_y + 1, height))

        cv2.rectangle(
            overlay,
            (roi_start_x, roi_start_y),
            (roi_end_x - 1, roi_end_y - 1),
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
        self.draw_optional_vertical_line(
            overlay,
            result.robot_center_x,
            (0, 165, 255),
            2
        )
        self.draw_optional_vertical_line(
            overlay,
            result.preferred_center_x,
            (0, 255, 255),
            1
        )
        self.draw_optional_vertical_line(
            overlay,
            result.last_bottom_x,
            (0, 255, 0)
        )
        self.draw_optional_vertical_line(
            overlay,
            result.pending_bottom_x,
            (255, 0, 0)
        )

        for row in result.band_rows:
            y = roi_start_y + row.y
            color = (80, 80, 80)
            thickness = 1
            if row.index == 0:
                color = (
                    (0, 180, 0)
                    if result.bottom_band_valid
                    else (0, 0, 255)
                )
                thickness = 2
                cv2.rectangle(
                    overlay,
                    (roi_start_x, roi_start_y + row.y_min),
                    (
                        roi_end_x - 1,
                        roi_start_y + max(row.y_min, row.y_max - 1)
                    ),
                    color,
                    2
                )
            cv2.line(
                overlay,
                (roi_start_x, y),
                (roi_end_x - 1, y),
                color,
                thickness
            )

        for candidate in result.candidates:
            color = (0, 180, 0) if candidate.accepted else (0, 0, 255)
            if candidate.reason == 'too_narrow':
                color = (255, 0, 0)
            cv2.rectangle(
                overlay,
                (candidate.x_start, roi_start_y + candidate.y_min),
                (max(candidate.x_start, candidate.x_end - 1),
                 roi_start_y + candidate.y_max - 1),
                color,
                1
            )

        selected_points = [
            (int(round(candidate.center_x)), roi_start_y + candidate.y)
            for candidate in result.selected_bands
        ]
        for point in selected_points:
            cv2.circle(overlay, point, 5, (0, 255, 255), -1)
        if len(selected_points) >= 2:
            points = np.array(
                selected_points,
                dtype=np.int32
            ).reshape(-1, 1, 2)
            cv2.polylines(overlay, [points], False, (0, 255, 255), 2)

        if result.fitted_line is not None:
            x_top, y_top, x_bottom, y_bottom = result.fitted_line
            cv2.line(
                overlay,
                (x_top, roi_start_y + y_top),
                (x_bottom, roi_start_y + y_bottom),
                (0, 0, 255),
                2
            )

        text_lines = [
            f'reason={result.reason}',
            f'line_width_cm={self.tracker_config.line_width_cm:.1f}',
            f'dark_fraction={result.dark_fraction:.3f}',
            f'roi=({roi_start_x},{roi_start_y})-'
            f'({roi_end_x - 1},{roi_end_y - 1})',
            f'valid_bands={len(result.selected_bands)}/'
            f'{self.tracker_config.num_scan_bands}',
            f'lateral_error={result.lateral_error:.3f}',
            f'heading_error={result.heading_error:.3f}',
            f'confidence={result.confidence:.3f}',
            f'line_visible={self.format_bool(result.line_visible)}',
            'bottom_band_valid='
            f'{self.format_bool(result.bottom_band_valid)}',
            'candidate_rejected='
            f'{self.format_bool(result.candidate_rejected)}',
            f'reject_reason={result.candidate_rejection_reason}',
            f'current_bottom_x='
            f'{self.format_optional_float(result.current_bottom_x)}',
            f'tracking_anchor_x='
            f'{self.format_optional_float(result.tracking_anchor_x)}',
            f'last_bottom_x='
            f'{self.format_optional_float(result.last_bottom_x)}',
            f'pending_bottom_x='
            f'{self.format_optional_float(result.pending_bottom_x)}',
            f'pending_stable_count={result.pending_stable_count}',
            f'image_center_x={width / 2.0:.1f}',
            f'robot_center_x='
            f'{self.format_optional_float(result.robot_center_x)}',
            'robot_center_offset='
            f'{self.robot_center_x_offset_fraction:.3f}+'
            f'{self.robot_center_x_offset_px:.1f}px',
            f'preferred_center_x='
            f'{self.format_optional_float(result.preferred_center_x)}',
            'track_lock_enabled='
            f'{self.format_bool(result.track_lock_enabled)}',
            f'lost_frame_count={result.lost_frame_count}',
            'track_jump_rejected='
            f'{self.format_bool(result.track_jump_rejected)}',
        ]
        self.draw_overlay_text(overlay, text_lines)

        return overlay

    def make_failure_overlay(self, image_msg, image, reason, stage):
        overlay = self.make_overlay_base(image_msg, image)
        text_lines = [
            'line_visible=false',
            'confidence=0.000',
            'bottom_band_valid=false',
            f'reject_reason={reason}',
            f'stage={stage}',
            f'input_encoding={self.image_msg_encoding(image_msg)}',
            f'input_size={self.image_msg_size_text(image_msg)}',
            f'cv_image_shape={self.shape_text(image)}',
        ]
        self.draw_overlay_text(overlay, text_lines)
        return overlay

    def make_overlay_base(self, image_msg, image):
        if image is not None:
            return image.copy()

        height, width = self.debug_image_size(image_msg, image)
        return np.zeros((height, width, 3), dtype='uint8')

    @staticmethod
    def draw_overlay_text(overlay, text_lines):
        height = overlay.shape[0]
        for index, text in enumerate(text_lines):
            y = 18 + index * 18
            if y >= height - 4:
                break
            cv2.putText(
                overlay,
                text,
                (10, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (0, 255, 255),
                1,
                cv2.LINE_AA
            )

    @staticmethod
    def draw_optional_vertical_line(image, x_value, color, thickness=1):
        if x_value is None:
            return
        height, width = image.shape[:2]
        x = int(round(float(x_value)))
        x = int(clamp(x, 0, max(0, width - 1)))
        cv2.line(
            image,
            (x, 0),
            (x, max(0, height - 1)),
            color,
            thickness
        )

    @staticmethod
    def format_optional_float(value):
        if value is None:
            return 'None'
        return f'{float(value):.1f}'

    @staticmethod
    def format_optional_int(value):
        if value is None:
            return 'None'
        return str(int(value))

    @staticmethod
    def format_bool(value):
        return 'true' if bool(value) else 'false'

    @staticmethod
    def shape_text(array):
        if array is None:
            return 'None'
        shape = getattr(array, 'shape', None)
        if shape is None:
            return 'unknown'
        return 'x'.join(str(part) for part in shape)

    @staticmethod
    def image_msg_encoding(image_msg):
        return str(getattr(image_msg, 'encoding', 'unknown'))

    @staticmethod
    def image_msg_size_text(image_msg):
        width = int(getattr(image_msg, 'width', 0) or 0)
        height = int(getattr(image_msg, 'height', 0) or 0)
        return f'{width}x{height}'

    @staticmethod
    def debug_image_size(image_msg, image):
        if image is not None:
            height, width = image.shape[:2]
            return max(1, int(height)), max(1, int(width))

        width = int(getattr(image_msg, 'width', 0) or 0)
        height = int(getattr(image_msg, 'height', 0) or 0)
        return max(1, height), max(1, width)

    def make_debug_mask(self, image_msg, image, binary):
        if binary is None:
            height, width = self.debug_image_size(image_msg, image)
            return np.zeros((height, width), dtype='uint8')

        if len(binary.shape) == 3:
            binary = cv2.cvtColor(binary, cv2.COLOR_BGR2GRAY)
        if binary.dtype != np.uint8:
            binary = np.clip(binary, 0, 255).astype('uint8')
        return binary

    def make_debug_overlay(
        self,
        image_msg,
        image,
        overlay,
        result,
        reason,
        stage
    ):
        if overlay is not None:
            return overlay
        if result is None:
            return self.make_failure_overlay(image_msg, image, reason, stage)

        try:
            return self.make_overlay(image, result)
        except Exception as exc:
            self.get_logger().warn(
                'Failed to draw debug overlay: '
                f'{exc}; stage={stage}, '
                f'input_encoding={self.image_msg_encoding(image_msg)}, '
                f'input_size={self.image_msg_size_text(image_msg)}, '
                f'cv_image_shape={self.shape_text(image)}'
            )
            return self.make_failure_overlay(
                image_msg,
                image,
                f'overlay_failed:{type(exc).__name__}',
                stage
            )

    def publish_debug_images(
        self,
        image_msg,
        image,
        binary,
        overlay,
        roi_start_y,
        result=None,
        stage='debug_publish',
        failure_reason='line_not_visible'
    ):
        status = {
            'enabled': self.enable_debug_image,
            'mask_published': False,
            'overlay_published': False,
            'mask_shape': 'None',
            'overlay_shape': 'None',
            'stage': stage,
        }
        if not self.enable_debug_image:
            return status

        try:
            mask = self.make_debug_mask(image_msg, image, binary)
            overlay = self.make_debug_overlay(
                image_msg,
                image,
                overlay,
                result,
                failure_reason,
                stage
            )
            status['mask_shape'] = self.shape_text(mask)
            status['overlay_shape'] = self.shape_text(overlay)

            mask_msg = self.bridge.cv2_to_imgmsg(mask, encoding='mono8')
            overlay_msg = self.bridge.cv2_to_imgmsg(overlay, encoding='bgr8')
            mask_msg.header = image_msg.header
            overlay_msg.header = image_msg.header

            self.mask_pub.publish(mask_msg)
            status['mask_published'] = True
            self.overlay_pub.publish(overlay_msg)
            status['overlay_published'] = True
        except Exception as exc:
            self.get_logger().warn(
                f'Failed to publish debug images: {exc}; '
                f'stage={stage}, '
                f'input_encoding={self.image_msg_encoding(image_msg)}, '
                f'input_size={self.image_msg_size_text(image_msg)}, '
                f'cv_image_shape={self.shape_text(image)}, '
                f'mask_shape={status["mask_shape"]}, '
                f'overlay_shape={status["overlay_shape"]}'
            )
        return status

    def publish_fallback_debug(
        self,
        image_msg,
        image,
        result,
        reason,
        stage
    ):
        binary = result.binary if result is not None else None
        roi_start_y = result.roi_start_y if result is not None else 0
        self.publish_debug_images(
            image_msg,
            image,
            binary,
            None,
            roi_start_y,
            result=result,
            stage=stage,
            failure_reason=reason
        )

    def log_image_callback_exception(
        self,
        message,
        exc,
        image_msg,
        stage,
        image=None,
        result=None,
        overlay=None
    ):
        mask = result.binary if result is not None else None
        self.get_logger().warn(
            f'{message}: {exc}; '
            f'stage={stage}, '
            f'input_encoding={self.image_msg_encoding(image_msg)}, '
            f'input_size={self.image_msg_size_text(image_msg)}, '
            f'cv_image_shape={self.shape_text(image)}, '
            f'mask_shape={self.shape_text(mask)}, '
            f'overlay_shape={self.shape_text(overlay)}'
        )

    def log_debug(
        self,
        result,
        image_msg=None,
        image=None,
        mask=None,
        debug_status=None
    ):
        if not self.debug_log:
            return

        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_debug_log_ns < self.debug_log_period_ns:
            return

        debug_status = debug_status or {}
        mask_shape = debug_status.get('mask_shape') or self.shape_text(mask)
        overlay_shape = debug_status.get('overlay_shape') or 'None'
        mask_published = self.format_bool(
            debug_status.get('mask_published', False)
        )
        overlay_published = self.format_bool(
            debug_status.get('overlay_published', False)
        )
        self.last_debug_log_ns = now_ns
        self.get_logger().info(
            'line debug: '
            f'enable_debug_image={self.format_bool(self.enable_debug_image)}, '
            'image_callback_receiving_frames=true, '
            f'publishing_line_mask={mask_published}, '
            f'publishing_line_overlay={overlay_published}, '
            f'input_encoding={self.image_msg_encoding(image_msg)}, '
            f'input_size={self.image_msg_size_text(image_msg)}, '
            f'cv_image_shape={self.shape_text(image)}, '
            f'mask_shape={mask_shape}, '
            f'overlay_shape={overlay_shape}, '
            f'reason={result.reason}, '
            f'use_full_frame_roi='
            f'{self.format_bool(self.tracker_config.use_full_frame_roi)}, '
            f'roi=({result.roi_start_x},{result.roi_start_y})-'
            f'({self.format_optional_int(result.roi_end_x)},'
            f'{self.format_optional_int(result.roi_end_y)}), '
            f'valid_bands={len(result.selected_bands)}/'
            f'{self.tracker_config.num_scan_bands}, '
            f'dark_fraction={result.dark_fraction:.3f}, '
            f'lateral_error={result.lateral_error:.3f}, '
            f'heading_error={result.heading_error:.3f}, '
            f'confidence={result.confidence:.3f}, '
            f'line_visible={self.format_bool(result.line_visible)}, '
            'bottom_band_valid='
            f'{self.format_bool(result.bottom_band_valid)}, '
            'candidate_rejected='
            f'{self.format_bool(result.candidate_rejected)}, '
            f'reject_reason={result.candidate_rejection_reason}, '
            f'current_bottom_x='
            f'{self.format_optional_float(result.current_bottom_x)}, '
            f'tracking_anchor_x='
            f'{self.format_optional_float(result.tracking_anchor_x)}, '
            f'last_bottom_x='
            f'{self.format_optional_float(result.last_bottom_x)}, '
            f'pending_bottom_x='
            f'{self.format_optional_float(result.pending_bottom_x)}, '
            f'pending_stable_count={result.pending_stable_count}, '
            f'robot_center_x='
            f'{self.format_optional_float(result.robot_center_x)}, '
            'robot_center_offset='
            f'{self.robot_center_x_offset_fraction:.3f}+'
            f'{self.robot_center_x_offset_px:.1f}px, '
            f'preferred_center_x='
            f'{self.format_optional_float(result.preferred_center_x)}, '
            'track_lock_enabled='
            f'{self.format_bool(result.track_lock_enabled)}, '
            f'lost_frame_count={result.lost_frame_count}, '
            'track_jump_rejected='
            f'{self.format_bool(result.track_jump_rejected)}'
        )

    def log_debug_reason(self, reason):
        if not self.debug_log:
            return

        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_debug_log_ns < self.debug_log_period_ns:
            return

        self.last_debug_log_ns = now_ns
        self.get_logger().info(
            'line debug: '
            f'reason={reason}, '
            'valid_bands=0, '
            'dark_fraction=0.000, '
            'lateral_error=0.000, '
            'heading_error=0.000, '
            'confidence=0.000, '
            'line_visible=false, '
            'bottom_band_valid=false, '
            'candidate_rejected=true, '
            f'reject_reason={reason}, '
            f'last_bottom_x={self.format_optional_float(self.last_bottom_x)}, '
            f'pending_bottom_x='
            f'{self.format_optional_float(self.pending_bottom_x)}, '
            f'pending_stable_count={self.pending_stable_count}, '
            f'track_lock_enabled={self.format_bool(self.track_lock_enabled)}, '
            f'lost_frame_count={self.lost_frame_count}'
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
