"""单次动态 adapter 的零运动时序与逐帧白横杆统计回归。"""

from rk_bringup.validation_dynamic_core import adapter_schedule
from rk_bringup.validation_dynamic_core import legacy_line_stale_event
from rk_bringup.validation_dynamic_core import sdk_event_monotonic_ns
from rk_bringup.validation_dynamic_core import sdk_status_event_key
from rk_bringup.validation_dynamic_core import white_bar_metrics
from rk_safety.command_mux_core import CommandMuxCore, VelocityCommand


def test_armed_schedule_is_silent_before_arm_and_runs_full_second():
    events = adapter_schedule(100, 1_000_000_000)
    assert events[0] == (100, 'MOVE')
    assert events[-1] == (1_000_000_100, 'ZERO')
    assert all(kind == 'MOVE' for _timestamp, kind in events[:-1])
    assert len(events) == 51


def test_white_metrics_use_persisted_frames_for_t0_tstop_and_loss():
    frames = [
        {
            'monotonic_ns': 0, 'visible': False,
            'confidence': 0.0, 'center_y': 0.0,
        },
        {
            'monotonic_ns': 10, 'visible': True,
            'confidence': 0.8, 'center_y': 0.2,
        },
        {
            'monotonic_ns': 20, 'visible': True,
            'confidence': 0.8, 'center_y': 0.3,
        },
        {
            'monotonic_ns': 30, 'visible': True,
            'confidence': 0.8, 'center_y': 0.4,
        },
        {
            'monotonic_ns': 40, 'visible': False,
            'confidence': 0.0, 'center_y': 0.0,
        },
        {
            'monotonic_ns': 60, 'visible': True,
            'confidence': 0.8, 'center_y': 0.5,
        },
    ]
    metrics = white_bar_metrics(frames, 19, 59)
    assert metrics['first_visible_ns'] == 10
    assert metrics['first_stable_visible_ns'] == 10
    assert metrics['center_y_t0'] == 0.3
    assert metrics['center_y_tstop'] == 0.5
    assert metrics['longest_loss_sec'] == 20 / 1e9


def test_wait_start_mission_zero_has_priority_over_line_adapter():
    """复现 RUN1：mission 的 fresh zero 会按正式优先级覆盖 line。"""
    mux = CommandMuxCore()
    mux.update_line_command(VelocityCommand(linear_x=0.25), 0.0)
    mux.update_mission_command(VelocityCommand(), 0.01)
    decision = mux.evaluate(0.02)
    assert decision.active_source == 'mission'
    assert decision.command.linear_x == 0.0


def test_isolated_validation_mux_holds_line_for_full_second():
    """无 mission zero 候选时，50 Hz adapter 全程保持 line source。"""
    mux = CommandMuxCore()
    for timestamp, kind in adapter_schedule(0, 1_000_000_000)[:-1]:
        now = timestamp / 1e9
        assert kind == 'MOVE'
        mux.update_line_command(VelocityCommand(linear_x=0.25), now)
        decision = mux.evaluate(now)
        assert decision.active_source == 'line'
        assert decision.command.linear_x == 0.25


def test_sdk_status_replay_key_and_server_clock_are_stable():
    """同 instance/sequence 的重放只计一次，运动时钟采用 server monotonic。"""
    payload = {
        'server_instance_id': 'instance-a',
        'sequence': 3,
        'server_monotonic_ns': 123456,
        'receive_monotonic_ns': 999999,
    }
    replay = dict(payload, receive_monotonic_ns=1_999_999)
    assert sdk_status_event_key(payload) == ('instance-a', 3)
    assert sdk_status_event_key(replay) == ('instance-a', 3)
    assert sdk_event_monotonic_ns(payload, 999999) == 123456
    assert sdk_event_monotonic_ns({}, 999999) == 999999


def test_legacy_replay_reproduces_355ms_line_failure():
    """本次真机 receiver timestamps 必须稳定复现旧模糊分类。"""
    rows = [
        {'channel': 'line_track', 'monotonic_ns': 12422294664304},
        {'channel': 'line_track', 'monotonic_ns': 12422650457960},
    ]
    result = legacy_line_stale_event(rows)
    assert result == {
        'previous_ns': 12422294664304,
        'next_ns': 12422650457960,
        'gap_ns': 355793656,
        'stop_reason': 'LINE_TRACK_FAILURE',
    }
