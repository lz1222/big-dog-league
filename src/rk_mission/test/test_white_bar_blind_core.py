"""START/FINISH 共用白横线 seen→lost→blind 状态机回归测试。"""

from pathlib import Path

import pytest
import yaml

from rk_mission.white_bar_blind_core import FOLLOW
from rk_mission.white_bar_blind_core import REQUEST_ACTION
from rk_mission.white_bar_blind_core import ZERO
from rk_mission.white_bar_blind_core import WhiteBarBlindApproachCore


FRAME_NS = 10_000_000
REQUIRED_STATUS_FIELDS = {
    'white_bar_stage',
    'white_bar_seen_latched',
    'white_bar_seen_count',
    'white_bar_lost_count',
    'white_bar_lost_confirmed',
    'white_bar_lost_monotonic_ns',
    'white_bar_blind_duration_sec',
    'white_bar_blind_elapsed_sec',
    'white_bar_blind_active_forward_sec',
    'white_bar_blind_wall_elapsed_sec',
    'white_bar_blind_remaining_sec',
    'white_bar_blind_recovery_count',
    'white_bar_blind_recovery_state',
    'white_bar_blind_last_line_valid',
    'white_bar_center_y',
    'white_bar_visible',
    'white_bar_confidence',
}


def _core(stage='START', duration=0.5):
    """建立显式 ARM 的纯逻辑核心，时间统一用单调纳秒。"""
    core = WhiteBarBlindApproachCore(
        confirm_frames=3,
        min_confidence=0.55,
        blind_duration_sec=duration,
        detection_timeout_sec=0.35,
        line_recovery_timeout_sec=0.75,
        line_recovery_confirm_frames=3,
    )
    core.arm(stage, 0)
    return core


def _observe(core, index, visible, confidence=0.9, center_y=0.5):
    timestamp_ns = index * FRAME_NS
    core.observe_detection(
        timestamp_ns,
        visible=visible,
        confidence=confidence,
        center_y=center_y,
    )
    return timestamp_ns


def _safe_inputs(vx=0.27, wz=0.12, **overrides):
    values = {
        'line_valid': True,
        'line_fresh': True,
        'suggested_fresh': True,
        'suggested_vx': vx,
        'suggested_wz': wz,
        'follower_ready': True,
        'follower_fresh': True,
        'gait_locked': False,
        'gait_fresh': True,
        'mux_healthy': True,
        'mux_fresh': True,
        'mux_source_valid': True,
    }
    values.update(overrides)
    return values


def _latch_seen(core, start_index=1, center_y=0.5):
    for index in range(start_index, start_index + 3):
        _observe(core, index, True, center_y=center_y)
    assert core.seen_latched


def _confirm_lost(core, start_index=4):
    timestamp_ns = 0
    for index in range(start_index, start_index + 3):
        timestamp_ns = _observe(core, index, False)
    assert core.lost_confirmed
    return timestamp_ns


def test_a_never_seen_false_frames_cannot_become_lost():
    core = _core()

    for index in range(1, 7):
        _observe(core, index, False)

    assert not core.seen_latched
    assert core.lost_count == 0
    assert not core.lost_confirmed
    decision = core.evaluate(6 * FRAME_NS, **_safe_inputs())
    assert decision.action == FOLLOW


def test_b_three_stable_visible_frames_latch_seen():
    core = _core()

    _latch_seen(core)

    assert core.seen_count == 3
    assert core.seen_latched


def test_c_single_false_after_seen_does_not_confirm_lost():
    core = _core()
    _latch_seen(core)

    _observe(core, 4, False)

    assert core.lost_count == 1
    assert not core.lost_confirmed
    assert core.evaluate(4 * FRAME_NS, **_safe_inputs()).action == FOLLOW


def test_d_three_false_frames_record_exact_lost_timestamp():
    core = _core()
    _latch_seen(core)

    lost_ns = _confirm_lost(core)

    assert lost_ns == 6 * FRAME_NS
    assert core.lost_monotonic_ns == lost_ns
    assert core.lost_count == 3


def test_e_stale_detection_fails_closed_without_becoming_lost():
    core = _core()
    _observe(core, 1, True)

    decision = core.evaluate(400_000_001, **_safe_inputs())

    assert decision.action == ZERO
    assert decision.reason == 'WHITE_BAR_DETECTION_STALE'
    assert not core.lost_confirmed


@pytest.mark.parametrize(
    'stage,motion_name',
    (('START', 'start_jump'), ('FINISH', 'finish_jump')),
)
def test_f_g_shared_blind_completion_maps_only_motion_name(
    stage, motion_name,
):
    core = _core(stage=stage, duration=0.2)
    _latch_seen(core)
    lost_ns = _confirm_lost(core)

    start = core.evaluate(
        lost_ns, **_safe_inputs(vx=0.27, wz=-0.2)
    )
    during = core.evaluate(
        lost_ns + 199_999_999, **_safe_inputs(vx=0.27, wz=-0.2)
    )
    complete = core.evaluate(
        lost_ns + 200_000_000, **_safe_inputs(vx=0.27, wz=-0.2)
    )

    assert start.action == FOLLOW
    assert during.action == FOLLOW
    assert during.linear_x == pytest.approx(0.27)
    assert during.angular_z == pytest.approx(-0.2)
    assert complete.action == REQUEST_ACTION
    assert complete.motion_name == motion_name


def test_h_formal_start_and_finish_share_exactly_one_duration_parameter():
    source_root = Path(__file__).resolve().parents[2]
    config_path = (
        source_root / 'rk_bringup' / 'config'
        / 'non_arm_competition_params.yaml'
    )
    raw_config = config_path.read_text(encoding='utf-8')
    config = yaml.safe_load(raw_config)
    parameters = config['line_course_mission_node']['ros__parameters']
    follower_parameters = config['line_follower_node']['ros__parameters']

    assert parameters['white_bar_blind_forward_duration_sec'] == 1.3
    assert parameters['white_bar_line_recovery_timeout_sec'] == (
        follower_parameters['short_lost_timeout']
    )
    assert raw_config.count('white_bar_blind_forward_duration_sec:') == 1
    assert 'start_blind_duration' not in raw_config
    assert 'finish_blind_duration' not in raw_config
    assert 'start_lost_time' not in raw_config
    assert 'finish_lost_time' not in raw_config


def test_i_center_y_above_old_threshold_never_triggers_action():
    core = _core(duration=0.5)
    _latch_seen(core, center_y=0.99)

    decision = core.evaluate(3 * FRAME_NS, **_safe_inputs())

    assert decision.action == FOLLOW
    assert core.snapshot(3 * FRAME_NS)['white_bar_center_y'] == 0.99


def test_j_formal_control_does_not_read_deprecated_approach_parameters():
    mission_source = (
        Path(__file__).resolve().parents[1]
        / 'rk_mission' / 'line_course_mission_node.py'
    ).read_text(encoding='utf-8')
    control_source = mission_source.split(
        '    def _control_armed_white_bar', 1
    )[1].split('    def _control_white_bar_wait', 1)[0]

    assert 'self.white_bar_approach_speed' not in mission_source
    assert 'self.white_bar_stop_y_ratio' not in mission_source
    assert 'center_y >=' not in control_source
    assert 'DEPRECATED / rollback-only' in mission_source
    zero_index = control_source.index(
        'self._publish_mission_candidate(Twist())'
    )
    completion_index = control_source.index(
        'self.white_stage_controller.white_bar_event(True)'
    )
    request_index = control_source.index(
        'self.white_bar_action_request_publisher.publish(request)'
    )
    assert zero_index < completion_index < request_index


def test_k_blind_preserves_fresh_follower_speed_and_steering():
    core = _core(duration=1.0)
    _latch_seen(core)
    lost_ns = _confirm_lost(core)

    decision = core.evaluate(
        lost_ns + FRAME_NS, **_safe_inputs(vx=0.27, wz=0.31)
    )

    assert decision.action == FOLLOW
    assert decision.linear_x == pytest.approx(0.27)
    assert decision.angular_z == pytest.approx(0.31)


def test_l_single_invalid_line_zeroes_then_recovers_without_fault():
    core = _core(duration=1.0)
    _latch_seen(core)
    lost_ns = _confirm_lost(core)

    core.evaluate(lost_ns, **_safe_inputs())
    invalid_ns = lost_ns + FRAME_NS
    core.observe_line(invalid_ns, valid=False)
    decision = core.evaluate(
        invalid_ns, **_safe_inputs(line_valid=False)
    )

    assert decision.action == ZERO
    assert decision.reason == 'WHITE_BAR_BLIND_LINE_RECOVERY'
    assert core.fault_reason == ''
    for offset in (2, 3, 4):
        core.observe_line(lost_ns + offset * FRAME_NS, valid=True)
    recovered = core.evaluate(
        lost_ns + 4 * FRAME_NS, **_safe_inputs()
    )
    assert recovered.action == FOLLOW
    assert recovered.reason == 'WHITE_BAR_BLIND_FORWARD'
    assert core.blind_recovery_count == 1


def test_m_zero_duration_is_not_calibrated_and_never_requests_action():
    core = _core(duration=0.0)
    _latch_seen(core)
    lost_ns = _confirm_lost(core)

    decision = core.evaluate(lost_ns, **_safe_inputs())

    assert decision.action == ZERO
    assert decision.reason == 'WHITE_BAR_BLIND_DURATION_NOT_CALIBRATED'
    assert decision.motion_name == ''


@pytest.mark.parametrize('stage', ('START', 'FINISH'))
def test_zero_motion_synthetic_sequence_resets_loss_and_stops(stage):
    """复现要求的 true×3,false,true,false×3，且绝不产生动作请求。"""
    core = _core(stage=stage, duration=0.0)
    _latch_seen(core)
    assert core.evaluate(3 * FRAME_NS, **_safe_inputs()).action == FOLLOW

    _observe(core, 4, False)
    assert core.lost_count == 1
    _observe(core, 5, True)
    assert core.lost_count == 0
    lost_ns = _confirm_lost(core, start_index=6)
    decision = core.evaluate(lost_ns, **_safe_inputs())

    assert decision.action == ZERO
    assert decision.reason == 'WHITE_BAR_BLIND_DURATION_NOT_CALIBRATED'
    assert REQUIRED_STATUS_FIELDS == set(core.snapshot(lost_ns))


def test_start_finish_zero_motion_sequences_have_one_shared_contract():
    """除 stage 身份外，两阶段的计数、时钟和零速结果必须完全相同。"""
    results = {}
    for stage in ('START', 'FINISH'):
        core = _core(stage=stage, duration=0.0)
        _latch_seen(core)
        _observe(core, 4, False)
        _observe(core, 5, True)
        lost_ns = _confirm_lost(core, start_index=6)
        decision = core.evaluate(lost_ns, **_safe_inputs())
        snapshot = core.snapshot(lost_ns)
        snapshot.pop('white_bar_stage')
        results[stage] = (decision, snapshot)

    assert results['START'] == results['FINISH']


def test_active_a_continuous_forward_completes_at_1p3_seconds():
    """连续安全正速候选只按有效前进时间完成。"""
    core = _core(duration=1.3)
    _latch_seen(core)
    lost_ns = _confirm_lost(core)
    assert core.evaluate(lost_ns, **_safe_inputs()).action == FOLLOW
    for elapsed_ms in (300, 600, 900, 1200):
        now_ns = lost_ns + elapsed_ms * 1_000_000
        core.observe_detection(
            now_ns, visible=False, confidence=0.0, center_y=0.0
        )
        assert core.evaluate(now_ns, **_safe_inputs()).action == FOLLOW
    complete_ns = lost_ns + 1_300_000_000
    core.observe_detection(
        complete_ns, visible=False, confidence=0.0, center_y=0.0
    )
    decision = core.evaluate(complete_ns, **_safe_inputs())
    snapshot = core.snapshot(complete_ns)

    assert decision.action == REQUEST_ACTION
    assert snapshot['white_bar_blind_active_forward_sec'] == pytest.approx(1.3)
    assert snapshot['white_bar_blind_wall_elapsed_sec'] == pytest.approx(1.3)
    assert snapshot['white_bar_blind_remaining_sec'] == pytest.approx(0.0)


def test_active_b_recovery_pause_freezes_timer_then_resumes():
    """0.6 s 前进 + 0.2 s 零速恢复 + 0.7 s 前进才完成。"""
    core = _core(duration=1.3)
    _latch_seen(core)
    lost_ns = _confirm_lost(core)
    core.evaluate(lost_ns, **_safe_inputs())

    forward_600_ns = lost_ns + 600_000_000
    core.observe_detection(
        forward_600_ns, visible=False, confidence=0.0, center_y=0.0
    )
    assert core.evaluate(
        forward_600_ns, **_safe_inputs()
    ).action == FOLLOW
    core.observe_line(forward_600_ns, valid=False)
    paused = core.evaluate(
        forward_600_ns, **_safe_inputs(line_valid=False)
    )
    assert paused.action == ZERO

    for elapsed_ms in (700, 750, 800):
        now_ns = lost_ns + elapsed_ms * 1_000_000
        core.observe_detection(
            now_ns, visible=False, confidence=0.0, center_y=0.0
        )
        core.observe_line(now_ns, valid=True)
    resumed_ns = lost_ns + 800_000_000
    assert core.evaluate(resumed_ns, **_safe_inputs()).action == FOLLOW
    complete_ns = lost_ns + 1_500_000_000
    core.observe_detection(
        complete_ns, visible=False, confidence=0.0, center_y=0.0
    )
    decision = core.evaluate(complete_ns, **_safe_inputs())
    snapshot = core.snapshot(complete_ns)

    assert decision.action == REQUEST_ACTION
    assert snapshot['white_bar_blind_active_forward_sec'] == pytest.approx(1.3)
    assert snapshot['white_bar_blind_wall_elapsed_sec'] == pytest.approx(1.5)
    assert snapshot['white_bar_blind_recovery_count'] == 1


def test_active_d_persistent_invalid_times_out_without_action():
    """持续丢线超过 follower 短时窗口必须故障关闭且不请求跳跃。"""
    core = _core(duration=1.3)
    _latch_seen(core)
    lost_ns = _confirm_lost(core)
    core.evaluate(lost_ns, **_safe_inputs())
    invalid_ns = lost_ns + 100_000_000
    core.observe_line(invalid_ns, valid=False)
    assert core.evaluate(
        invalid_ns, **_safe_inputs(line_valid=False)
    ).reason == 'WHITE_BAR_BLIND_LINE_RECOVERY'

    timeout_ns = invalid_ns + 750_000_001
    core.observe_detection(
        timeout_ns, visible=False, confidence=0.0, center_y=0.0
    )
    decision = core.evaluate(
        timeout_ns, **_safe_inputs(line_valid=False)
    )

    assert decision.action == ZERO
    assert decision.reason == 'WHITE_BAR_LINE_RECOVERY_TIMEOUT'
    assert decision.motion_name == ''
    assert core.snapshot(timeout_ns)[
        'white_bar_blind_recovery_state'
    ] == 'FAULTED'


@pytest.mark.parametrize('stage', ('START', 'FINISH'))
def test_active_e_start_finish_use_identical_recovery_logic(stage):
    """START/FINISH 只映射不同动作名，恢复与累计语义完全共用。"""
    core = _core(stage=stage, duration=1.3)
    _latch_seen(core)
    lost_ns = _confirm_lost(core)
    core.evaluate(lost_ns, **_safe_inputs())
    invalid_ns = lost_ns + 100_000_000
    core.observe_line(invalid_ns, valid=False)
    decision = core.evaluate(
        invalid_ns, **_safe_inputs(line_valid=False)
    )

    assert decision.action == ZERO
    assert decision.reason == 'WHITE_BAR_BLIND_LINE_RECOVERY'
    assert core.snapshot(invalid_ns)['white_bar_blind_recovery_count'] == 1
