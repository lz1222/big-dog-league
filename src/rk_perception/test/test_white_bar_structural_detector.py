"""白横杆结构检测的离线契约：验证相对边缘/路径约束，不依赖绝对白度。"""

from types import SimpleNamespace

import cv2
import numpy as np

from rk_perception.real_line_tracker_node import (
    SpecialDetectionResult,
    StructuralWhiteBarCandidate,
    StructuralWhiteBarConfig,
    StructuralWhiteBarTemporalFilter,
    _detect_white_bar_structural_candidate,
    detect_white_bar_structural,
)


WIDTH = 320
HEIGHT = 240
ROUTE_X = 174.0


def make_scene():
    """构造灰色赛道与黑线路径；横杆使用 170 灰度，证明不需要 V=180。"""
    image = np.full((HEIGHT, WIDTH, 3), 110, dtype=np.uint8)
    cv2.rectangle(image, (158, 0), (190, HEIGHT - 1), (20, 20, 20), -1)
    return image


def draw_fragmented_bar(image, top=120, height=16, left=35, right=295):
    """以左右片段模拟白杆被黑线路径遮挡后的真实结构。"""
    cv2.rectangle(image, (left, top), (157, top + height), (170, 170, 170), -1)
    cv2.rectangle(image, (191, top), (right, top + height), (170, 170, 170), -1)
    return image


def default_config(**kwargs):
    return StructuralWhiteBarConfig(**kwargs).normalized()


def test_start_like_edge_pair_detects_fragmented_bar_without_absolute_white_gate():
    image = draw_fragmented_bar(make_scene(), top=136, height=16)

    result = detect_white_bar_structural(
        image, robot_center_x=ROUTE_X, config=default_config()
    )

    assert result.visible is True
    assert result.confidence > 0.55
    assert 0.55 < result.center_y < 0.70
    assert result.width_ratio > 0.65


def test_finish_like_long_fragmented_bar_detects_with_route_crossing():
    image = draw_fragmented_bar(make_scene(), top=104, height=22, left=12, right=308)

    raw = _detect_white_bar_structural_candidate(
        image, None, ROUTE_X, default_config()
    )

    assert raw.result.visible is True
    assert raw.left_x < ROUTE_X < raw.right_x
    assert raw.top_y < raw.bottom_y


def test_horizontal_structure_not_crossing_route_is_rejected():
    image = make_scene()
    cv2.rectangle(image, (5, 120), (125, 136), (170, 170, 170), -1)

    result = detect_white_bar_structural(
        image, robot_center_x=ROUTE_X, config=default_config()
    )

    assert result.visible is False
    assert result.reason == 'structural_not_detected'


def test_single_edge_wrong_polarity_too_tall_and_too_narrow_are_rejected():
    # 单边：亮区域延续到图像底部，没有可配对的下边缘。
    single_edge = make_scene()
    cv2.rectangle(single_edge, (20, 130), (300, HEIGHT - 1), (170, 170, 170), -1)
    assert not detect_white_bar_structural(
        single_edge, robot_center_x=ROUTE_X, config=default_config()
    ).visible

    # 错误极性：暗条在亮背景中产生的边缘方向与白横杆相反。
    wrong_polarity = np.full((HEIGHT, WIDTH, 3), 170, dtype=np.uint8)
    cv2.rectangle(wrong_polarity, (158, 0), (190, HEIGHT - 1), (20, 20, 20), -1)
    cv2.rectangle(wrong_polarity, (20, 120), (300, 136), (50, 50, 50), -1)
    assert not detect_white_bar_structural(
        wrong_polarity, robot_center_x=ROUTE_X, config=default_config()
    ).visible

    too_tall = draw_fragmented_bar(make_scene(), top=100, height=42)
    assert not detect_white_bar_structural(
        too_tall, robot_center_x=ROUTE_X, config=default_config()
    ).visible

    too_narrow = make_scene()
    cv2.rectangle(too_narrow, (145, 120), (205, 136), (170, 170, 170), -1)
    assert not detect_white_bar_structural(
        too_narrow, robot_center_x=ROUTE_X, config=default_config()
    ).visible


def test_reliable_tracking_anchor_overrides_robot_center_reference():
    image = make_scene()
    # 横杆只覆盖左侧路径；宽度足以保留局部对比，但不覆盖默认机身中心。
    cv2.rectangle(image, (0, 120), (165, 136), (170, 170, 170), -1)
    line_result = SimpleNamespace(line_visible=True, tracking_anchor_x=80.0)

    raw = _detect_white_bar_structural_candidate(
        image, line_result, ROUTE_X, default_config()
    )

    assert raw.result.visible is True
    assert raw.reference_x == 80.0


def raw_candidate(center_y, visible=True):
    """构造时序门输入，隔离验证 y 跳变和有限 miss 的安全行为。"""
    result = SpecialDetectionResult(
        target_type='white_bar',
        visible=visible,
        confidence=0.8 if visible else 0.0,
        center_y=center_y if visible else 0.0,
        center_x=0.5,
        width_ratio=0.7 if visible else 0.0,
        height_ratio=0.06 if visible else 0.0,
        reason='structural_raw' if visible else 'structural_not_detected',
    )
    return StructuralWhiteBarCandidate(
        result=result,
        reference_x=ROUTE_X,
        roi_top_y=36,
        roi_bottom_y=235,
        top_y=120 if visible else None,
        bottom_y=136 if visible else None,
        left_x=35 if visible else None,
        right_x=295 if visible else None,
    )


def test_temporal_filter_rejects_y_jump_and_expires_short_miss_hold():
    gate = StructuralWhiteBarTemporalFilter(default_config())

    assert gate.update(raw_candidate(0.60)).visible is False
    assert gate.update(raw_candidate(0.602)).visible is False
    assert gate.update(raw_candidate(0.604)).reason == 'structural_stable'

    jumped = gate.update(raw_candidate(0.72))
    assert jumped.visible is True
    assert jumped.reason == 'structural_miss_hold'
    assert abs(jumped.center_y - 0.604) < 1e-6

    assert gate.update(raw_candidate(0.0, visible=False)).visible is True
    expired = gate.update(raw_candidate(0.0, visible=False))
    assert expired.visible is False
    assert expired.reason == 'structural_miss'
