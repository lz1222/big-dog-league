"""全局步态 owner 状态机回归。"""

import sys
from pathlib import Path


SCRIPTS = Path(__file__).resolve().parents[1] / 'scripts'
sys.path.insert(0, str(SCRIPTS))

from global_gait_owner_core import (  # noqa: E402
    CLASSIC_ESTABLISHED_BY_VALIDATED_SEQUENCE,
    CLASSIC_VALIDATED_SEQUENCE_SOURCE,
    FAILED,
    FREE_READY,
    GlobalGaitOwnerCore,
    SdkStatusSequenceGuard,
)


def _reason(request_id, target, source='command_ack'):
    return (
        'request_id={};target={};phase=ready;'
        'verification_source={}'
    ).format(request_id, target, source)


def test_startup_classic_requires_validated_sequence():
    core = GlobalGaitOwnerCore()
    assert not core.observe_startup_classic(0, 'command_ack')
    assert core.observe_startup_classic(0, CLASSIC_VALIDATED_SEQUENCE_SOURCE)
    assert core.state == CLASSIC_ESTABLISHED_BY_VALIDATED_SEQUENCE
    assert core.verification_source == CLASSIC_VALIDATED_SEQUENCE_SOURCE


def test_classic_observation_clears_old_request_and_failure():
    """特殊动作最终 Classic ACK 必须覆盖旧 FREE 请求并清除旧失败原因。"""
    core = GlobalGaitOwnerCore()
    core.begin('old-free', 'FREE')
    core.fail('old failure')
    assert core.observe_startup_classic(0, CLASSIC_VALIDATED_SEQUENCE_SOURCE)
    assert core.state == CLASSIC_ESTABLISHED_BY_VALIDATED_SEQUENCE
    assert core.request_id == ''
    assert core.failure_reason == ''


def test_stale_ack_cannot_complete_new_transition():
    core = GlobalGaitOwnerCore()
    assert core.begin('new', 'FREE')
    assert not core.observe_ack('GAIT_READY', 0, _reason('old', 'FREE'))
    assert core.observe_ack('GAIT_READY', 0, _reason('new', 'FREE'))
    assert core.state == FREE_READY


def test_failure_is_latched():
    core = GlobalGaitOwnerCore()
    core.begin('request', 'CLASSIC')
    assert core.observe_ack(
        'GAIT_FAILED', 3203, _reason('request', 'CLASSIC'))
    assert core.state == FAILED
    assert '3203' in core.failure_reason


def test_classic_command_ack_without_physical_signature_fails_closed():
    core = GlobalGaitOwnerCore()
    assert core.begin('classic', 'CLASSIC')
    assert not core.observe_ack('GAIT_READY', 0, _reason('classic', 'CLASSIC'))
    assert core.state == FAILED


def test_classic_command_ack_is_intermediate_and_keeps_physical_lock():
    core = GlobalGaitOwnerCore()
    assert core.begin('classic', 'CLASSIC')
    assert core.observe_classic_command_ack(
        0, _reason('classic', 'CLASSIC')
    )
    assert core.state == 'CLASSIC_COMMAND_ACK'
    assert core.verification_source == 'command_ack'


def test_status_guard_accepts_only_current_instance_in_strict_order():
    """forwarder 可重放，但重复启动事件不得把已就绪 Classic 重新锁住。"""
    guard = SdkStatusSequenceGuard('current')
    assert not guard.accept('old', 99)
    assert not guard.accept('current', 0)
    assert guard.accept('current', 1)
    assert guard.accept('current', 2)
    assert not guard.accept('current', 2)
    assert not guard.accept('current', 1)
    assert not guard.accept('current', 'bad')
