# -*- coding: utf-8 -*-
"""三类任务平台定位状态机的软件安全合同测试。"""

import math

import pytest

from rk_mission.platform_positioning_core import (
    BoardObservation, PlatformCommand, TaskPlatformPositioningCore,
    WhiteBarObservation,
)


def _transfer_params(**overrides):
    params = {
        'transfer_platform_calibrated': True,
        'transfer_anchor_signature_valid': True,
        'transfer_anchor_confirm_frames': 2,
        'transfer_anchor_heading_tolerance': 0.1,
        'transfer_anchor_lateral_tolerance': 0.1,
    }
    params.update(overrides)
    return params


def _pickup_params(**overrides):
    params = {
        'pickup_platform_calibrated': True,
        'pickup_board_signature_valid': True,
        'pickup_target_area_ratio': 0.20,
        'pickup_area_tolerance': 0.02,
        'pickup_target_bottom_y_ratio': 0.80,
        'pickup_bottom_y_tolerance': 0.02,
        'pickup_target_center_x_ratio': 0.50,
        'pickup_center_x_tolerance': 0.02,
        'pickup_target_width_ratio': 0.40,
        'pickup_width_tolerance': 0.02,
        'pickup_position_confirm_frames': 2,
        'pickup_zero_confirm_samples': 2,
        'pickup_zero_epsilon': 0.001,
        'pickup_yaw_tolerance_deg': 1.0,
        'pickup_yaw_timeout_sec': 2.0,
    }
    params.update(overrides)
    return params


def _place_params(**overrides):
    params = {
        'place_platform_calibrated': True,
        'place_white_bar_signature_valid': True,
        'place_white_bar_confirm_frames': 2,
        'place_zero_confirm_samples': 2,
        'place_white_bar_target_y_ratio': 0.80,
        'place_white_bar_y_tolerance': 0.02,
        'place_white_bar_target_span_ratio': 0.60,
        'place_white_bar_span_tolerance': 0.02,
        'place_line_max_lateral_error': 0.10,
        'place_line_max_heading_error': 0.10,
    }
    params.update(overrides)
    return params


def _board():
    return BoardObservation(True, 0.9, 0.50, 0.70, 0.80, 0.40, 0.20, 0.20)


def _bar():
    return WhiteBarObservation(True, 0.9, 0.80, 0.60, 0.04)


def _enter_pickup(core):
    core.set_route_phase('PICKUP_PLATFORM_APPROACH')
    core.tick(0.0)


def _lock_pickup(core):
    core.observe_pickup_board(_board())
    assert core.state == 'PICKUP_VISUAL_APPROACH'
    core.observe_pickup_board(_board())
    assert core.state == 'PICKUP_VISUAL_POSITION_LOCKED'
    core.tick(0.1)
    assert core.state == 'PICKUP_STOP_CONFIRM'


def _confirm_zero(core):
    core.observe_final_command(PlatformCommand())
    core.observe_final_command(PlatformCommand())


def _start_left_turn(core):
    _lock_pickup(core)
    _confirm_zero(core)
    core.tick(0.2)
    assert core.state == 'PICKUP_STOP_CONFIRMED'
    core.observe_odom_yaw(0.0)
    core.tick(0.3)
    assert core.state == 'PICKUP_TURN_LEFT'


def test_transfer_requires_route_and_valid_anchor():
    core = TaskPlatformPositioningCore(_transfer_params())
    core.observe_transfer_anchor(
        detected=True, confidence=1.0,
        heading_error=0.0, lateral_error=0.0)
    assert core.state == 'IDLE'

    core.set_route_phase('TRANSFER_PLATFORM_APPROACH')
    core.tick(0.0)
    assert core.state == 'TRANSFER_ANCHOR_WAIT'
    core.observe_transfer_anchor(
        detected=False, confidence=1.0,
        heading_error=0.0, lateral_error=0.0)
    assert core.state == 'TRANSFER_ANCHOR_WAIT'
    core.observe_transfer_anchor(
        detected=True, confidence=1.0,
        heading_error=0.0, lateral_error=0.0)
    core.observe_transfer_anchor(
        detected=True, confidence=1.0,
        heading_error=0.0, lateral_error=0.0)
    assert core.state == 'TRANSFER_FINE_OFFSET'


def test_transfer_offset_flag_distinguishes_disabled_and_uncalibrated():
    core = TaskPlatformPositioningCore(_transfer_params())
    core.set_route_phase('TRANSFER_PLATFORM_APPROACH')
    core.tick(0.0)
    for _ in range(2):
        core.observe_transfer_anchor(
            detected=True, confidence=1.0,
            heading_error=0.0, lateral_error=0.0)
    assert core.tick(0.1) == PlatformCommand()
    assert core.state == 'TRANSFER_OFFSET_NOT_CALIBRATED'

    core = TaskPlatformPositioningCore(_transfer_params(
        transfer_offset_required=False))
    core.set_route_phase('TRANSFER_PLATFORM_APPROACH')
    core.tick(0.0)
    for _ in range(2):
        core.observe_transfer_anchor(
            detected=True, confidence=1.0,
            heading_error=0.0, lateral_error=0.0)
    core.tick(0.1)
    assert core.state == 'TRANSFER_STOP_CONFIRM'
    assert not core.gait_lock_requested()
    for _ in range(3):
        core.observe_final_command(PlatformCommand())
    core.tick(0.2)
    assert core.state == 'TRANSFER_TASK'
    assert core.gait_lock_requested()


def test_transfer_offset_counts_only_matching_final_command():
    core = TaskPlatformPositioningCore(_transfer_params(
        allow_motion_execution=True,
        transfer_offset_speed_mps=0.10,
        transfer_offset_duration_sec=0.5,
    ))
    core.set_route_phase('TRANSFER_PLATFORM_APPROACH')
    core.tick(0.0)
    for _ in range(2):
        core.observe_transfer_anchor(
            detected=True, confidence=1.0,
            heading_error=0.0, lateral_error=0.0)
    core.observe_final_command(PlatformCommand())
    core.tick(0.2)
    assert core.snapshot()['active_motion_elapsed_sec'] == 0.0
    core.observe_final_command(PlatformCommand(vx=0.10))
    core.tick(0.71)
    assert core.state == 'TRANSFER_STOP_CONFIRM'


@pytest.mark.parametrize(
    'phase, expected',
    [
        ('TRANSFER_PLATFORM_APPROACH', 'TRANSFER_PLATFORM_NOT_CALIBRATED'),
        ('PICKUP_PLATFORM_APPROACH', 'PICKUP_PLATFORM_NOT_CALIBRATED'),
        ('PLACE_PLATFORM_APPROACH', 'PLACE_PLATFORM_NOT_CALIBRATED'),
    ],
)
def test_default_platforms_fail_closed_without_motion(phase, expected):
    core = TaskPlatformPositioningCore()
    core.set_route_phase(phase)
    assert core.state == expected
    assert core.tick(10.0) == PlatformCommand()
    assert core.snapshot()['failure_reason'] == expected


def test_pickup_target_requires_consecutive_frames_and_final_zero():
    core = TaskPlatformPositioningCore(_pickup_params())
    core.observe_pickup_board(_board())
    assert core.state == 'IDLE'
    _enter_pickup(core)
    core.observe_pickup_board(_board())
    core.observe_pickup_board(BoardObservation())
    assert core.state == 'PICKUP_VISUAL_APPROACH'
    _lock_pickup(core)
    core.observe_final_command(PlatformCommand(vx=0.01))
    core.tick(0.2)
    assert core.state == 'PICKUP_STOP_CONFIRM'
    _confirm_zero(core)
    core.tick(0.3)
    assert core.state == 'PICKUP_STOP_CONFIRMED'


def test_pickup_yaw_is_closed_loop_and_times_out_fail_closed():
    core = TaskPlatformPositioningCore(_pickup_params())
    _enter_pickup(core)
    _start_left_turn(core)
    core.observe_odom_yaw(math.radians(45.0))
    assert core.tick(0.4) == PlatformCommand()
    assert core.state == 'PICKUP_TURN_LEFT'
    core.observe_odom_yaw(math.pi / 2.0)
    core.tick(0.5)
    assert core.state == 'PICKUP_VIEW_REVERSE'

    core = TaskPlatformPositioningCore(_pickup_params())
    _enter_pickup(core)
    _start_left_turn(core)
    core.tick(2.31)
    assert core.state == 'PICKUP_YAW_TIMEOUT'
    assert core.tick(2.4) == PlatformCommand()


def test_pickup_reverse_uses_final_active_motion_time_and_freezes_on_zero():
    core = TaskPlatformPositioningCore(_pickup_params(
        allow_motion_execution=True,
        pickup_timing_calibrated=True,
        pickup_view_reverse_speed_mps=0.10,
        pickup_view_reverse_duration_sec=1.0,
    ))
    _enter_pickup(core)
    _start_left_turn(core)
    core.observe_odom_yaw(math.pi / 2.0)
    core.tick(0.4)
    core.observe_final_command(PlatformCommand(vx=-0.10))
    core.tick(0.9)
    assert core.snapshot()['active_motion_elapsed_sec'] == pytest.approx(0.5)
    core.observe_final_command(PlatformCommand())
    core.tick(1.41)
    assert core.snapshot()['active_motion_elapsed_sec'] == pytest.approx(0.5)
    core.observe_final_command(PlatformCommand(vx=-0.10))
    core.tick(1.92)
    assert core.state == 'PICKUP_REVERSE_ZERO_CONFIRM'


def test_pickup_timing_and_recognition_fail_closed():
    core = TaskPlatformPositioningCore(_pickup_params())
    _enter_pickup(core)
    _start_left_turn(core)
    core.observe_odom_yaw(math.pi / 2.0)
    core.tick(0.4)
    core.tick(0.5)
    assert core.state == 'PICKUP_REVERSE_NOT_CALIBRATED'
    assert core.tick(0.6) == PlatformCommand()

    core = TaskPlatformPositioningCore(_pickup_params(
        pickup_view_reverse_required=False))
    _enter_pickup(core)
    _start_left_turn(core)
    core.observe_odom_yaw(math.pi / 2.0)
    core.tick(0.4)
    core.tick(0.5)
    _confirm_zero(core)
    core.tick(0.6)
    assert core.state == 'PICKUP_OBJECT_RECOGNITION'
    core.recognition_result(False)
    core.tick(0.7)
    assert core.state == 'PICKUP_RECOGNITION_FAILED'
    assert core.tick(0.8) == PlatformCommand()


def test_pickup_forward_right_turn_and_arm_handoff_sequence():
    core = TaskPlatformPositioningCore(_pickup_params(
        allow_motion_execution=True,
        pickup_timing_calibrated=True,
        pickup_view_reverse_required=False,
        pickup_side_forward_speed_mps=0.10,
        pickup_side_forward_duration_sec=0.5,
    ))
    _enter_pickup(core)
    _start_left_turn(core)
    core.observe_odom_yaw(math.pi / 2.0)
    core.tick(0.4)
    core.tick(0.5)
    _confirm_zero(core)
    core.tick(0.6)
    core.recognition_result(True)
    core.tick(0.7)
    assert core.state == 'PICKUP_SIDE_FORWARD'
    core.observe_final_command(PlatformCommand())
    core.tick(0.9)
    assert core.snapshot()['active_motion_elapsed_sec'] == 0.0
    core.observe_final_command(PlatformCommand(vx=0.10))
    core.tick(1.41)
    assert core.state == 'PICKUP_FORWARD_ZERO_CONFIRM'
    _confirm_zero(core)
    core.tick(1.5)
    assert core.state == 'PICKUP_TURN_RIGHT'
    core.observe_odom_yaw(0.0)
    core.tick(1.6)
    assert core.state == 'PICKUP_SIDE_POSITION_READY'
    assert not core.gait_lock_requested()
    _confirm_zero(core)
    core.tick(1.7)
    assert core.state == 'PICKUP_ARM_HANDOFF_READY'
    assert core.gait_lock_requested()


def test_place_requires_white_and_line_gates_for_consecutive_frames():
    core = TaskPlatformPositioningCore(_place_params())
    core.observe_place_white_bar(
        _bar(), line_lateral_error=0.0, line_heading_error=0.0)
    assert core.state == 'IDLE'
    core.set_route_phase('PLACE_PLATFORM_APPROACH')
    core.tick(0.0)
    core.observe_place_white_bar(
        _bar(), line_lateral_error=0.5, line_heading_error=0.0)
    core.observe_place_white_bar(
        _bar(), line_lateral_error=0.0, line_heading_error=0.5)
    core.observe_place_white_bar(
        _bar(), line_lateral_error=0.0, line_heading_error=0.0)
    assert core.state == 'PLACE_WHITE_BAR_APPROACH'
    core.observe_place_white_bar(
        _bar(), line_lateral_error=0.0, line_heading_error=0.0)
    assert core.state == 'PLACE_STOP_CONFIRM'
    assert not core.snapshot()['finish_white_bar_armed']
    core.tick(0.1)
    assert core.state == 'PLACE_STOP_CONFIRM'
    _confirm_zero(core)
    core.tick(0.2)
    assert core.state == 'PLACE_POSITION_LOCKED'


def test_place_done_still_requires_explicit_finish_rearm():
    core = TaskPlatformPositioningCore(_place_params())
    core.set_route_phase('PLACE_PLATFORM_APPROACH')
    core.tick(0.0)
    for _ in range(2):
        core.observe_place_white_bar(
            _bar(), line_lateral_error=0.0, line_heading_error=0.0)
    _confirm_zero(core)
    core.tick(0.1)
    core.place_done()
    assert core.state == 'PLACE_DONE'
    assert not core.snapshot()['finish_white_bar_armed']
    core.rearm_finish()
    assert core.snapshot()['finish_white_bar_armed']


def test_place_optional_offset_uses_active_motion_time():
    core = TaskPlatformPositioningCore(_place_params(
        allow_motion_execution=True,
        place_final_offset_enabled=True,
        place_final_offset_speed_mps=0.05,
        place_final_offset_duration_sec=0.5,
    ))
    core.set_route_phase('PLACE_PLATFORM_APPROACH')
    core.tick(0.0)
    for _ in range(2):
        core.observe_place_white_bar(
            _bar(), line_lateral_error=0.0, line_heading_error=0.0)
    assert core.state == 'PLACE_FINAL_OFFSET'
    core.observe_final_command(PlatformCommand())
    core.tick(0.2)
    assert core.snapshot()['active_motion_elapsed_sec'] == 0.0
    core.observe_final_command(PlatformCommand(vx=0.05))
    core.tick(0.71)
    assert core.state == 'PLACE_STOP_CONFIRM'


def test_status_contains_field_calibration_observability():
    core = TaskPlatformPositioningCore()
    core.set_route_phase('PICKUP_PLATFORM_APPROACH')
    status = core.snapshot()
    for key in (
        'platform', 'state', 'calibrated', 'signature_valid',
        'anchor_detected', 'current_values', 'target_values', 'errors',
        'confirm_count', 'active_motion_elapsed_sec',
        'active_motion_remaining_sec', 'yaw_start_rad', 'yaw_current_rad',
        'yaw_delta_rad', 'zero_confirm_count', 'failure_reason',
    ):
        assert key in status
