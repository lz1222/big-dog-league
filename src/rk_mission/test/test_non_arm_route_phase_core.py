"""非机械臂路线阶段核心的 ROS 无关回归测试。"""

from pathlib import Path
import sys

import pytest
import yaml

from rk_mission.non_arm_route_phase_core import NonArmRoutePhaseCore
from rk_mission.non_arm_route_phase_core import (
    validate_white_bar_timeout_chain,
)
from rk_mission.white_bar_stage_command_core import (
    WhiteBarStageCommandSequencer,
)


def _started_core():
    """建立已开始、尚未通过 START 白线的测试任务。"""
    core = NonArmRoutePhaseCore()
    event = core.mission_start('run-001')
    assert event.accepted
    return core


def _to_mid_route(core):
    """走完 START 跳跃和对齐，供中段门控测试复用。"""
    assert core.accept_stage_command('START').accepted
    assert core.white_bar_action_started('START').accepted
    assert core.white_bar_action_completed('START').route_phase == (
        'START_REACQUIRE'
    )
    assert core.alignment_completed('start').route_phase == 'MID_ROUTE'


def _to_post_inspection(core):
    """走完红圈和检查成功闭环，但尚未 arm FINISH。"""
    _to_mid_route(core)
    assert core.red_circle_confirmed().accepted
    assert core.red_circle_reached('inspection-001').accepted
    assert core.inspection_action_progress(
        'inspection-001',
        running=True,
    ).accepted
    assert core.inspection_action_succeeded('inspection-001').route_phase == (
        'POST_INSPECTION'
    )


def test_full_legal_route_requires_every_explicit_milestone():
    """START、检查、FINISH 和终点区只能按一次性顺序推进。"""
    core = _started_core()

    _to_post_inspection(core)
    armed = core.accept_stage_command('FINISH')
    assert armed.accepted
    assert armed.route_phase == 'FINISH_STAGE'
    assert core.alignment_completed('red').accepted
    assert core.white_bar_action_started('FINISH').accepted
    completed = core.white_bar_action_completed('FINISH')
    assert completed.route_phase == 'FINISH_REACQUIRE'
    final_armed = core.alignment_completed('finish')
    assert final_armed.route_phase == 'FINAL_ZONE_ARMED'
    assert final_armed.final_zone_armed
    assert core.stop_zone_confirmed().accepted
    final = core.stop_zone_inside_confirmed()

    assert final.route_phase == 'FINAL_STOP'
    assert final.start_jump_completed
    assert final.inspection_completed
    assert final.finish_jump_completed
    assert final.final_zone_armed


def test_start_and_stop_are_idempotent_without_reusing_old_run_state():
    """重复 start 不重置；stop 清空 run/request/动作状态。"""
    core = _started_core()
    assert core.accept_stage_command('START').accepted

    duplicate = core.mission_start('run-should-not-replace')
    assert duplicate.action == 'IGNORED'
    assert duplicate.run_id == 'run-001'
    assert duplicate.active_stage == 'START'

    stopped = core.mission_stop()
    repeated_stop = core.mission_stop()
    assert stopped.route_phase == 'WAIT_START'
    assert repeated_stop.reason == 'mission_stop_idempotent'
    assert not repeated_stop.mission_started
    assert repeated_stop.run_id == ''
    assert repeated_stop.active_request_id == ''
    assert repeated_stop.active_stage == ''


def test_stage_command_publisher_does_not_reset_on_duplicate_start():
    """全局重复 start 不得使 START 的 run/sequence 从头开始。"""
    sequencer = WhiteBarStageCommandSequencer()

    first = sequencer.mission_start(1.0)
    duplicate = sequencer.mission_start(2.0)

    assert first.state == 'WAIT_RUN'
    assert duplicate.state == 'WAIT_RUN'
    assert duplicate.reason == 'mission_start_duplicate_ignored'


def test_finish_stage_reacknowledgement_returns_status_instead_of_none():
    """重复 FINISH ACK 只能诊断，发布器回调不得返回 None。"""
    sequencer = WhiteBarStageCommandSequencer()
    assert sequencer.mission_start(0.0).state == 'WAIT_RUN'
    assert sequencer.on_stage_status({
        'run_id': 'run-001',
        'state': 'DISARMED',
        'last_sequence': 0,
        'active_stage': '',
        'motion_name': '',
        'request_sent': False,
        'action_done': False,
    }, 0.0).requested_stage == 'START'
    assert sequencer.on_stage_status({
        'run_id': 'run-001',
        'state': 'START_COMPLETED',
        'last_sequence': 1,
        'active_stage': 'START',
        'motion_name': 'start_jump',
        'request_sent': True,
        'action_done': True,
    }, 0.1).state == 'WAIT_FINISH_MILESTONE'
    assert sequencer.on_line_course_state({
        'state': 'TURN_AFTER_RED',
        'mission_started': True,
        'white_bar_stage_run_id': 'run-001',
    }, 0.2).requested_stage == 'FINISH'
    first = sequencer.on_stage_status({
        'run_id': 'run-001',
        'state': 'FINISH_ARMED',
        'last_sequence': 2,
        'active_stage': 'FINISH',
        'motion_name': 'finish_jump',
        'request_sent': False,
        'action_done': False,
    }, 0.3)
    repeated = sequencer.on_stage_status({
        'run_id': 'run-001',
        'state': 'FINISH_ARMED',
        'last_sequence': 2,
        'active_stage': 'FINISH',
        'motion_name': 'finish_jump',
        'request_sent': False,
        'action_done': False,
    }, 0.4)

    assert first.state == 'FINISH_ACKED'
    assert repeated is not None
    assert repeated.reason == 'finish_command_acknowledged_already'


def test_start_phase_red_and_stop_detections_are_ignored_without_count_leakage(
):
    """起点红圈/蓝区不触发动作或终点。

    核心保持 START_STAGE。
    """
    core = _started_core()

    red = core.red_circle_confirmed()
    stop = core.stop_zone_confirmed()
    inside = core.stop_zone_inside_confirmed()

    assert not red.accepted
    assert not stop.accepted
    assert not inside.accepted
    assert core.route_phase == 'START_STAGE'
    assert not core.red_circle_consumed
    assert not core.final_zone_armed


def test_finish_requires_start_and_matching_successful_inspection():
    """FINISH 命令不能因白线或早到 milestone 越过检查阶段。"""
    core = _started_core()

    rejected = core.accept_stage_command('FINISH')

    assert not rejected.accepted
    assert rejected.route_phase == 'FAULTED'
    assert rejected.fault_reason == 'illegal_stage_command_FINISH'


@pytest.mark.parametrize(
    ('operation', 'reason'),
    (
        (
            lambda core: core.white_bar_action_started('START'),
            'illegal_start_jump_start',
        ),
        (
            lambda core: core.white_bar_action_completed('START'),
            'white_bar_completion_stage_mismatch',
        ),
        (
            lambda core: core.red_circle_reached('inspection-001'),
            'red_circle_reached_out_of_order',
        ),
        (
            lambda core: core.alignment_completed('finish'),
            'alignment_finish_out_of_order',
        ),
    ),
)
def test_illegal_non_sensor_transitions_fail_closed(operation, reason):
    """动作完成、跳跃开始和对齐越序必须进入 FAULTED。"""
    core = _started_core()

    event = operation(core)

    assert not event.accepted
    assert event.route_phase == 'FAULTED'
    assert event.fault_reason == reason


def test_inspection_result_must_match_the_current_request_id():
    """旧 run/request 的成功不能把当前路线跨阶段推进。"""
    core = _started_core()
    _to_mid_route(core)
    assert core.red_circle_confirmed().accepted
    assert core.red_circle_reached('inspection-current').accepted

    old = core.inspection_action_succeeded('inspection-old')
    current = core.inspection_action_succeeded('inspection-current')

    assert not old.accepted
    assert old.route_phase == 'INSPECTION_WAIT_SIGN'
    assert current.accepted
    assert current.route_phase == 'POST_INSPECTION'
    assert current.active_request_id == ''


def test_corner_requires_followup_alignment_but_does_not_skip_route_phase():
    """角点完成后仍需 ALIGN_TO_LINE，对齐只恢复当前中段。"""
    core = _started_core()
    _to_mid_route(core)

    corner = core.corner_confirmed()
    aligned = core.alignment_completed('corner')

    assert corner.accepted
    assert aligned.accepted
    assert aligned.route_phase == 'MID_ROUTE'


def test_timeout_chain_uses_real_front_jump_profile_totals_and_strict_margin():
    """22/26 秒严格大于真实 profile 与执行器时长加 3 秒。"""
    source_root = Path(__file__).resolve().parents[2] / 'rk_locomotion'
    source_text = str(source_root)
    if source_text not in sys.path:
        sys.path.insert(0, source_text)
    from rk_locomotion.front_jump_supervisor import FrontJumpProfile

    start = FrontJumpProfile(
        name='start',
        pre_stop_duration=0.5,
        final_zero_epsilon=0.001,
        final_zero_confirm_samples=3,
        final_zero_timeout=2.0,
        sdk_timeout=12.0,
        post_settle_duration=2.5,
    )
    finish = FrontJumpProfile(
        name='finish',
        pre_stop_duration=0.5,
        final_zero_epsilon=0.001,
        final_zero_confirm_samples=3,
        final_zero_timeout=2.0,
        sdk_timeout=12.0,
        post_settle_duration=2.5,
    )

    assert start.worst_case_duration_sec == 17.0
    assert finish.worst_case_duration_sec == 17.0
    assert validate_white_bar_timeout_chain(start, finish, 22.0, 26.0)
    with pytest.raises(ValueError):
        validate_white_bar_timeout_chain(start, finish, 20.0, 26.0)
    with pytest.raises(ValueError):
        validate_white_bar_timeout_chain(start, finish, 22.0, 25.0)


def test_formal_yaml_timeout_chain_matches_gait_profiles_and_strict_margin():
    """正式 YAML 变化时，重新证明 profile 到两层超时余量。"""
    source_root = Path(__file__).resolve().parents[2]
    gait_path = source_root / 'rk_locomotion' / 'config' / 'gait_params.yaml'
    formal_path = (
        source_root
        / 'rk_bringup'
        / 'config'
        / 'non_arm_competition_params.yaml'
    )
    gait = yaml.safe_load(gait_path.read_text(encoding='utf-8'))
    formal = yaml.safe_load(formal_path.read_text(encoding='utf-8'))
    profiles = gait['gait_control_node']['ros__parameters']['front_jump']
    line_params = formal['line_course_mission_node']['ros__parameters']
    executor_params = formal['white_bar_action_executor']['ros__parameters']

    def profile_duration(profile):
        return sum(float(profile[name]) for name in (
            'pre_stop_duration',
            'final_zero_timeout',
            'sdk_timeout',
            'post_settle_duration',
        ))

    start_duration = profile_duration(profiles['start'])
    finish_duration = profile_duration(profiles['finish'])
    assert line_params['front_jump_start_worst_case_duration_sec'] == (
        start_duration
    )
    assert line_params['front_jump_finish_worst_case_duration_sec'] == (
        finish_duration
    )
    assert line_params['white_bar_executor_action_timeout_sec'] == (
        executor_params['action_timeout_sec']
    )
    assert validate_white_bar_timeout_chain(
        start_duration,
        finish_duration,
        executor_params['action_timeout_sec'],
        line_params['white_bar_action_timeout_sec'],
    )


def test_white_bar_executor_safe_timeout_defaults_match_formal_chain():
    """未加载 YAML 时，执行器不能回退到旧 5 秒超时。"""
    source = (
        Path(__file__).resolve().parents[1]
        / 'rk_mission'
        / 'white_bar_action_executor_node.py'
    ).read_text(encoding='utf-8')

    assert "'server_wait_timeout_sec', 5.0" in source
    assert "'action_timeout_sec', 22.0" in source


def test_line_course_source_exposes_required_route_state_and_no_static_red_sdk(
):
    """状态 JSON 必含验收字段。

    正式红圈不读 deprecated 动作。
    """
    source = (
        Path(__file__).resolve().parents[1]
        / 'rk_mission'
        / 'line_course_mission_node.py'
    ).read_text(encoding='utf-8')
    for field in (
        'route_phase',
        'start_jump_completed',
        'inspection_completed',
        'finish_jump_completed',
        'final_zone_armed',
        'active_request_id',
        'fault_reason',
    ):
        assert "'{}'".format(field) in source
    assert 'self.red_circle_sdk_action' not in source
    assert "payload.get('mission_started') is True" in source
    assert source.count('if not self._line_follower_is_ready(now):') >= 4
