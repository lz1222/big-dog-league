"""独立挡板检测器的最小图像合同测试。"""

import numpy as np

from rk_perception.pickup_board_anchor_detector import (
    detect_pickup_board, pickup_board_route_enabled,
)


def _config():
    return {
        'roi_left_fraction': 0.0, 'roi_right_fraction': 1.0,
        'roi_top_fraction': 0.0, 'roi_bottom_fraction': 1.0,
        'gray_max': 40, 'hsv_h_min': 0, 'hsv_h_max': 180,
        'hsv_s_min': 0, 'hsv_s_max': 255, 'hsv_v_min': 0, 'hsv_v_max': 40,
        'min_area_ratio': 0.05, 'max_area_ratio': 0.30,
        'min_aspect_ratio': 1.0, 'max_aspect_ratio': 3.0,
    }


def test_dark_board_reports_normalized_bbox_metrics():
    image = np.full((100, 200, 3), 255, dtype=np.uint8)
    image[30:70, 50:130] = 0
    result = detect_pickup_board(image, _config())
    assert result.detected
    assert result.area_ratio == 0.16
    assert result.center_x_ratio == 0.45
    assert result.bottom_y_ratio == 0.70


def test_invalid_area_range_keeps_calibration_scaffold_invisible():
    config = _config()
    config['min_area_ratio'] = 1.0
    config['max_area_ratio'] = 0.0
    result = detect_pickup_board(np.zeros((20, 20, 3), dtype=np.uint8), config)
    assert not result.detected


def test_board_detector_is_enabled_only_by_explicit_pickup_route():
    assert not pickup_board_route_enabled('{}')
    assert not pickup_board_route_enabled({
        'platform_route_phase': 'TRANSFER_PLATFORM_APPROACH'})
    assert pickup_board_route_enabled({
        'platform_route_phase': 'PICKUP_PLATFORM_APPROACH'})
