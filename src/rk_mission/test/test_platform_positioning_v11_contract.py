# -*- coding: utf-8 -*-
"""V1.1 平台定位路由、时效、mux 与标定合同测试。"""

from rk_mission.platform_calibration_contract import (
    pickup_capture_result, place_capture_result,
)
from rk_mission.platform_positioning_contract import (
    PlatformFinishInterlockContract, parse_platform_route_state,
)
from rk_mission.platform_positioning_core import (
    BoardObservation, PlatformCommand, TaskPlatformPositioningCore,
)


def _pickup_params(**overrides):
    params = {
        'pickup_platform_calibrated': True,
        'pickup_board_signature_valid': True,
        'pickup_timing_calibrated': True,
        'pickup_target_area_ratio': .2, 'pickup_area_tolerance': .02,
        'pickup_target_bottom_y_ratio': .8, 'pickup_bottom_y_tolerance': .02,
        'pickup_target_center_x_ratio': .5, 'pickup_center_x_tolerance': .02,
        'pickup_target_width_ratio': .4, 'pickup_width_tolerance': .02,
        'pickup_target_top_y_ratio': .6, 'pickup_top_y_tolerance': .02,
        'pickup_turn_speed_radps': .4, 'pickup_yaw_tolerance_deg': 1.0,
        'pickup_yaw_timeout_sec': 2.0, 'pickup_view_reverse_required': False,
        'pickup_side_forward_required': False,
        'pickup_position_confirm_frames': 2, 'pickup_zero_confirm_samples': 2,
    }
    params.update(overrides)
    return params


def _place_params(**overrides):
    params = {
        'place_platform_calibrated': True,
        'place_white_bar_signature_valid': True,
        'place1_white_bar_target_y_ratio': .8,
        'place1_white_bar_y_tolerance': .02,
        'place1_white_bar_target_span_ratio': .6,
        'place1_white_bar_span_tolerance': .02,
        'place1_line_max_lateral_error': .1,
        'place1_line_max_heading_error': .1,
        'place2_white_bar_target_y_ratio': .7,
        'place2_white_bar_y_tolerance': .03,
        'place2_white_bar_target_span_ratio': .5,
        'place2_white_bar_span_tolerance': .03,
        'place2_line_max_lateral_error': .2,
        'place2_line_max_heading_error': .2,
    }
    params.update(overrides)
    return params


def _board():
    return BoardObservation(True, .9, .5, .7, .8, .4, .2, .2)


def _mux(core, command, source='mission', fresh=True):
    core.observe_cmd_mux_status({
        'active_source': source, 'final_vx': command.vx,
        'final_vy': command.vy, 'final_wz': command.wz,
    }, fresh=fresh)


def test_route_none_and_malformed_contract_clear_previous_platform_state():
    core = TaskPlatformPositioningCore(_pickup_params())
    core.set_route_phase('PICKUP_PLATFORM_APPROACH')
    assert core.state == 'PICKUP_PLATFORM_APPROACH'
    core.set_route_phase('NONE')
    assert core.state == 'IDLE'
    bad = parse_platform_route_state('{not json')
    assert not bad.valid and bad.platform_route_phase == 'NONE'
    core.set_route_phase(bad.platform_route_phase)
    assert core.state == 'IDLE'


def test_route_contract_requires_explicit_allowed_platform_phase():
    assert parse_platform_route_state(
        '{"route_phase":"MID_ROUTE"}').reason == (
        'platform_route_phase_missing')
    assert parse_platform_route_state(
        '{"platform_route_phase":"PICKUP_PLATFORM_APPROACH"}').valid
    assert not parse_platform_route_state(
        '{"platform_route_phase":"UNSAFE"}').valid


def test_stale_line_rejects_transfer_and_place_confirmation():
    transfer = TaskPlatformPositioningCore({
        'transfer_platform_calibrated': True,
        'transfer_anchor_signature_valid': True,
        'transfer_offset_required': False,
        'transfer_anchor_heading_tolerance': .1,
        'transfer_anchor_lateral_tolerance': .1,
    })
    transfer.set_route_phase('TRANSFER_PLATFORM_APPROACH')
    transfer.tick(0.0)
    transfer.update_sensor_freshness(corner=True, line=False)
    transfer.tick(.1)
    assert transfer.state == 'TRANSFER_ANCHOR_WAIT'
    assert transfer.snapshot()['confirm_count'] == 0

    place = TaskPlatformPositioningCore(_place_params())
    place.set_place_platform_id('place1')
    place.set_route_phase('PLACE_PLATFORM_APPROACH')
    place.tick(0.0)
    place.update_sensor_freshness(white=True, line=False)
    place.tick(.1)
    assert place.state == 'PLACE_WHITE_BAR_APPROACH'
    assert place.snapshot()['confirm_count'] == 0


def test_stale_odom_fails_closed_before_or_during_pickup_turn():
    core = TaskPlatformPositioningCore(_pickup_params())
    core.set_route_phase('PICKUP_PLATFORM_APPROACH')
    core.tick(0.0)
    core.observe_pickup_board(_board())
    core.observe_pickup_board(_board())
    core.tick(.1)
    core.observe_final_command(PlatformCommand())
    core.observe_final_command(PlatformCommand())
    core.tick(.2)
    core.update_sensor_freshness(odom=False)
    core.tick(.3)
    assert core.state == 'PICKUP_ODOM_STALE'


def test_old_final_zero_sample_cannot_be_recounted_each_timer_tick():
    core = TaskPlatformPositioningCore(_pickup_params())
    core.set_route_phase('PICKUP_PLATFORM_APPROACH')
    core.tick(0.0)
    core.observe_pickup_board(_board())
    core.observe_pickup_board(_board())
    core.tick(.1)
    core.observe_final_command(PlatformCommand(), sequence=7)
    assert core.snapshot()['zero_confirm_count'] == 1
    core.tick(.2)
    core.tick(.3)
    assert core.snapshot()['zero_confirm_count'] == 1


def test_active_motion_requires_fresh_mission_source_and_axis_clean_vector():
    params = _pickup_params(
        allow_motion_execution=True, pickup_view_reverse_required=True,
        pickup_view_reverse_speed_mps=.1, pickup_view_reverse_duration_sec=1.0,
    )
    core = TaskPlatformPositioningCore(params)
    core.state = 'PICKUP_VIEW_REVERSE'
    core.update_sensor_freshness(board=True)
    command = PlatformCommand(vx=-.1)
    core.observe_final_command(command)
    _mux(core, command, source='line')
    core.tick(1.0)
    assert core.snapshot()['active_motion_elapsed_sec'] == 0.0
    _mux(core, PlatformCommand(vx=-.1, wz=.1))
    core.tick(2.0)
    assert core.snapshot()['active_motion_elapsed_sec'] == 0.0
    _mux(core, command, fresh=False)
    core.tick(3.0)
    assert core.snapshot()['active_motion_elapsed_sec'] == 0.0
    _mux(core, command)
    core.tick(3.5)
    assert core.snapshot()['active_motion_elapsed_sec'] == .5


def test_place_profiles_are_independent_and_unknown_profile_fails_closed():
    core = TaskPlatformPositioningCore(_place_params())
    core.set_place_platform_id('place2')
    core.set_route_phase('PLACE_PLATFORM_APPROACH')
    assert core.snapshot()['target_values']['white_bar_y_ratio'] == .7
    unknown = TaskPlatformPositioningCore(_place_params())
    unknown.set_place_platform_id('not-a-platform')
    unknown.set_route_phase('PLACE_PLATFORM_APPROACH')
    assert unknown.state == 'PLACE_PREFLIGHT_NOT_READY'


def test_finish_interlock_contract_needs_place_done_and_explicit_rearm():
    contract = PlatformFinishInterlockContract()
    assert not contract.can_forward_stage('FINISH')
    contract.observe_platform_event('REARM_FINISH')
    assert not contract.finish_allowed()
    contract.observe_platform_event('PLACE_DONE')
    assert not contract.finish_allowed()
    contract.observe_platform_event('REARM_FINISH')
    assert contract.finish_allowed()
    assert contract.production_status == 'PRODUCTION_FINISH_INTERLOCK_PENDING'


def test_capture_zero_samples_are_explicitly_invalid():
    valid, reason = pickup_capture_result({'area_ratio': []})
    assert not valid and 'DETECTOR_THRESHOLD_CALIBRATION_REQUIRED' in reason
    valid, reason = place_capture_result({
        'center_y_ratio': [0.8], 'line_lateral_error': [],
    })
    assert not valid and reason == 'insufficient_white_bar_or_line_samples'
