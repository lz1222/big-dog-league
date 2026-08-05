# -*- coding: utf-8 -*-
"""全图巡线专项验收 ARM 门禁的纯软件回归测试。"""

from pathlib import Path
import sys


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from rk_bringup.line_acceptance_core import (  # noqa: E402
    DISARM,
    LINE_TEST_READY,
    LineAcceptanceGate,
)


def test_gate_is_disarmed_until_exact_two_step_confirmation():
    """默认、模糊文本和未确认的分段口令均不得启动真机速度链。"""
    gate = LineAcceptanceGate('S03')

    assert not gate.handle_command('LINE_TEST_READY ').armed
    assert not gate.handle_command('SEGMENT_READY S03').armed
    assert gate.handle_command(LINE_TEST_READY).ready_confirmed
    decision = gate.handle_command('SEGMENT_READY S03')

    assert decision.armed
    assert decision.start_requested
    assert not decision.ready_confirmed


def test_gate_rejects_wrong_segment_and_consumes_ready_confirmation():
    """错误分段不会借用上一条安全确认，下一次 ARM 必须重新确认。"""
    gate = LineAcceptanceGate('S04')

    gate.handle_command(LINE_TEST_READY)
    wrong = gate.handle_command('SEGMENT_READY S03')
    assert not wrong.armed
    assert wrong.reason == 'segment_rejected_not_allowed'

    armed = gate.handle_command('SEGMENT_READY S04')
    assert armed.armed
    disarmed = gate.handle_command(DISARM)
    assert not disarmed.armed
    assert disarmed.stop_requested
    assert not gate.handle_command('SEGMENT_READY S04').armed


def test_force_disarm_requests_line_follower_stop_only_when_needed():
    """急停后的 DISARM 不可恢复旧候选速度，且重复调用保持幂等。"""
    gate = LineAcceptanceGate('S01')
    gate.handle_command(LINE_TEST_READY)
    gate.handle_command('SEGMENT_READY S01')

    first = gate.force_disarm('estop_active')
    second = gate.force_disarm('estop_active')

    assert first.stop_requested
    assert not second.stop_requested
    assert not second.armed
