"""独立动态 adapter 的 fail-closed 状态机与 recorder 隔离测试。"""

import json
import time

from rk_bringup.validation_motion_core import ADAPTER_EXECUTOR_STALL
from rk_bringup.validation_motion_core import GAIT_LOCKED
from rk_bringup.validation_motion_core import LINE_SOURCE_STALE
from rk_bringup.validation_motion_core import MOTION_WINDOW_COMPLETE
from rk_bringup.validation_motion_core import MOVE
from rk_bringup.validation_motion_core import MUX_SOURCE_INVALID
from rk_bringup.validation_motion_core import SDK_ERROR
from rk_bringup.validation_motion_core import SILENT
from rk_bringup.validation_motion_core import SUGGESTED_CMD_STALE
from rk_bringup.validation_motion_core import ValidationMotionCore
from rk_bringup.validation_motion_core import ZERO
from rk_bringup.validation_recorder_core import BoundedJsonlWriter


def _ready_core(now_ns=0):
    core = ValidationMotionCore()
    core.observe_line(
        now_ns, visible=True, lateral=0.1, heading=0.0,
        source_sec=1, source_nanosec=2,
    )
    core.observe_candidate(now_ns, linear_x=0.27, angular_z=0.1)
    core.observe_gait_lock(now_ns, False)
    ok, reason = core.request_arm(now_ns)
    assert (ok, reason) == (True, 'ARMING')
    core.enable_output()
    return core


def test_prearm_is_silent():
    core = ValidationMotionCore()
    decision = core.tick(20_000_000, 20_000_000)
    assert decision.action == SILENT


def test_recorder_writer_stall_does_not_interrupt_adapter(tmp_path):
    """writer 阻塞400ms时，独立 core 的20ms控制时序仍持续运行。"""
    writer = BoundedJsonlWriter(
        tmp_path / 'stall.jsonl', queue_size=256, batch_size=1,
        flush_interval_sec=0.01, writer_delay_sec=0.4,
    )
    writer.start()
    core = _ready_core()
    core.observe_sdk('MOVE', 0, 0)
    decisions = []
    for tick_ns in range(0, 1_000_000_000, 20_000_000):
        if tick_ns % 80_000_000 == 0:
            core.observe_line(
                tick_ns, visible=True, lateral=0.1, heading=0.0,
                source_sec=1, source_nanosec=tick_ns,
            )
        if tick_ns % 100_000_000 == 0:
            core.observe_candidate(
                tick_ns, linear_x=0.27, angular_z=0.1,
            )
            core.observe_gait_lock(tick_ns, False)
        decisions.append(core.tick(tick_ns, tick_ns))
        if tick_ns == 0:
            writer.enqueue('line_track', tick_ns, {'visible': True})
    assert all(item.action == MOVE for item in decisions)
    assert all(item.reason != LINE_SOURCE_STALE for item in decisions)
    writer.close(timeout_sec=30.0)


def test_recorder_queue_full_is_nonblocking_and_records_backpressure(tmp_path):
    path = tmp_path / 'full.jsonl'
    writer = BoundedJsonlWriter(
        path, queue_size=2, batch_size=1,
        flush_interval_sec=0.01, writer_delay_sec=0.2,
    )
    writer.start()
    started = time.monotonic()
    results = [
        writer.enqueue('event', index, {'index': index})
        for index in range(50)
    ]
    elapsed = time.monotonic() - started
    assert elapsed < 0.1
    assert not all(results)
    stats = writer.close(timeout_sec=10.0)
    assert stats['dropped'] > 0
    assert stats['backpressure'] is True
    channels = [
        json.loads(line)['channel']
        for line in path.read_text().splitlines()
    ]
    assert 'RECORDER_BACKPRESSURE' in channels
    assert channels[-1] == 'RECORDER_FINAL'


def test_real_line_source_stale_fails_closed():
    core = _ready_core()
    decision = core.tick(350_000_001, 350_000_001)
    assert (decision.action, decision.reason) == (ZERO, LINE_SOURCE_STALE)


def test_suggested_source_stale_fails_closed():
    core = _ready_core()
    core.observe_line(
        250_000_001, visible=True, lateral=0.1, heading=0.0,
    )
    core.observe_gait_lock(250_000_001, False)
    decision = core.tick(250_000_001, 250_000_001)
    assert (decision.action, decision.reason) == (ZERO, SUGGESTED_CMD_STALE)


def test_gait_lock_fails_closed():
    core = _ready_core()
    core.observe_gait_lock(1, True)
    decision = core.tick(1, 1)
    assert (decision.action, decision.reason) == (ZERO, GAIT_LOCKED)


def test_adapter_executor_stall_has_distinct_reason():
    core = _ready_core()
    decision = core.tick(150_000_001, 20_000_000)
    assert (decision.action, decision.reason) == (
        ZERO, ADAPTER_EXECUTOR_STALL,
    )


def test_sdk_error_fails_closed():
    core = _ready_core()
    core.observe_sdk('SDK_ERROR', -1, 1)
    decision = core.tick(1, 1)
    assert (decision.action, decision.reason) == (ZERO, SDK_ERROR)


def test_mux_source_switch_after_move_ack_fails_closed():
    core = _ready_core()
    core.observe_sdk('MOVE', 0, 1)
    core.observe_mux(2, 'mission')
    decision = core.tick(2, 2)
    assert (decision.action, decision.reason) == (
        ZERO, MUX_SOURCE_INVALID,
    )


def test_one_second_schedule_has_50_moves_then_zero():
    core = _ready_core()
    core.observe_sdk('MOVE', 0, 0)
    moves = []
    for now_ns in range(0, 1_000_000_000, 20_000_000):
        core.observe_line(
            now_ns, visible=True, lateral=0.1, heading=0.0,
        )
        core.observe_candidate(
            now_ns, linear_x=0.27, angular_z=0.1,
        )
        core.observe_gait_lock(now_ns, False)
        moves.append(core.tick(now_ns, now_ns))
    stop = core.tick(1_000_000_000, 1_000_000_000)
    assert len(moves) == 50
    assert all(item.action == MOVE for item in moves)
    assert (stop.action, stop.reason) == (ZERO, MOTION_WINDOW_COMPLETE)


def test_recorder_drains_and_writes_final_fsync_record(tmp_path):
    path = tmp_path / 'drain.jsonl'
    writer = BoundedJsonlWriter(path, queue_size=8, batch_size=4)
    writer.start()
    for index in range(8):
        assert writer.enqueue('event', index, {'index': index})
    stats = writer.close()
    rows = [json.loads(line) for line in path.read_text().splitlines()]
    assert stats['enqueued'] == 8
    assert stats['written'] == 8
    assert [row['data']['index'] for row in rows[:-1]] == list(range(8))
    assert rows[-1]['channel'] == 'RECORDER_FINAL'
