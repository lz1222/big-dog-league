"""全局步态 owner 状态机回归。"""

import sys
from pathlib import Path


SCRIPTS = Path(__file__).resolve().parents[1] / 'scripts'
sys.path.insert(0, str(SCRIPTS))

from global_gait_owner_core import (  # noqa: E402
    CLASSIC_READY,
    FAILED,
    FREE_READY,
    GlobalGaitOwnerCore,
)


def _reason(request_id, target):
    return (
        'request_id={};target={};phase=ready;'
        'verification_source=command_ack'
    ).format(request_id, target)


def test_startup_classic_is_only_command_ack_ready():
    core = GlobalGaitOwnerCore()
    assert core.observe_startup_classic(0)
    assert core.state == CLASSIC_READY
    assert core.verification_source == 'command_ack'


def test_classic_observation_clears_old_request_and_failure():
    """特殊动作最终 Classic ACK 必须覆盖旧 FREE 请求并清除旧失败原因。"""
    core = GlobalGaitOwnerCore()
    core.begin('old-free', 'FREE')
    core.fail('old failure')
    assert core.observe_startup_classic(0)
    assert core.state == CLASSIC_READY
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
