"""PICKUP V1.2 弧顶 top_y 视觉签名的软件回归测试。"""

import math

import pytest

from rk_mission.platform_positioning_core import (
    BoardObservation, TaskPlatformPositioningCore,
)


def _core():
    """构造只验证视觉签名的核心，不启用任何实体运动参数。"""
    return TaskPlatformPositioningCore({
        'pickup_board_signature_valid': True,
        'pickup_target_top_y_ratio': 0.338,
        'pickup_top_y_tolerance': 0.012,
    })


def _board(*, detected=True, top_y=0.338, bottom=1.0,
           width=0.7, center_x=0.65):
    """由 top_y 反推 bbox，故意固定所有已知边界饱和字段。"""
    height = 0.2
    return BoardObservation(
        detected=detected,
        center_x_ratio=center_x,
        center_y_ratio=top_y + height / 2.0,
        bottom_y_ratio=bottom,
        width_ratio=width,
        height_ratio=height,
        area_ratio=0.46,
    )


@pytest.mark.parametrize('top_y', (0.333333, 0.337762, 0.340741))
def test_pickup_v12_accepts_measured_target_top_y_interval(top_y):
    assert _core()._pickup_target_matches(_board(top_y=top_y))


@pytest.mark.parametrize('top_y', (0.370370, 0.371219))
def test_pickup_v12_rejects_measured_post_apex_top_y_interval(top_y):
    assert not _core()._pickup_target_matches(_board(top_y=top_y))


@pytest.mark.parametrize('top_y', (math.nan, math.inf, -math.inf))
def test_pickup_v12_rejects_nonfinite_top_y(top_y):
    assert not _core()._pickup_target_matches(_board(top_y=top_y))


def test_pickup_v12_rejects_undetected_board():
    assert not _core()._pickup_target_matches(_board(detected=False))


def test_pickup_v12_ignores_boundary_saturated_legacy_features():
    assert _core()._pickup_target_matches(_board(
        top_y=0.338, bottom=1.0, width=0.7, center_x=0.65))


def test_pickup_v12_preflight_requires_only_top_y_visual_signature():
    """旧四组边界裁切 target/tolerance 为默认零值也不得阻塞预检。"""
    core = TaskPlatformPositioningCore({
        'pickup_board_signature_valid': True,
        'pickup_target_top_y_ratio': 0.338,
        'pickup_top_y_tolerance': 0.012,
        'pickup_turn_speed_radps': 0.4,
        'pickup_yaw_tolerance_deg': 1.0,
        'pickup_yaw_timeout_sec': 2.0,
        'pickup_view_reverse_required': False,
        'pickup_side_forward_required': False,
    })
    assert core._pickup_preflight_ready()
