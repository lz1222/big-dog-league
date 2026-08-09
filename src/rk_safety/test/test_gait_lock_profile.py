"""gait lock profile 的纯逻辑 fail-closed 回归测试。"""

import pytest

from rk_safety.gait_lock_profile import evaluate_lock
from rk_safety.gait_lock_profile import INSPECTION_LOCK_TOPIC


GAIT_TOPIC = '/gait/control_lock_req/gait'
TOPICS = [GAIT_TOPIC, INSPECTION_LOCK_TOPIC]
NOW = 10.0
TIMEOUT = 2.0


def _source(value=False, last_time=9.0):
    return {'seen': True, 'value': value, 'last_time': last_time}


def _evaluate(profile, start_mission_nodes, sources):
    return evaluate_lock(
        profile=profile,
        start_mission_nodes=start_mission_nodes,
        input_topics=TOPICS,
        sources=sources,
        now=NOW,
        source_timeout_sec=TIMEOUT,
    )


def _source_status(statuses, topic):
    return next(status for status in statuses if status['topic'] == topic)


def test_production_inspection_unseen_stays_locked():
    """生产图不得因 gait 健康而忽略未出现的 inspection provider。"""
    locked, reasons, _statuses, graph_ok, _detail = _evaluate(
        'production', True, {GAIT_TOPIC: _source()},
    )
    assert graph_ok is True
    assert locked is True
    assert 'source_unseen:{}'.format(INSPECTION_LOCK_TOPIC) in reasons


def test_production_inspection_stale_stays_locked():
    """生产 inspection 心跳过期必须 fail-closed。"""
    locked, reasons, _statuses, _graph_ok, _detail = _evaluate(
        'production', True,
        {
            GAIT_TOPIC: _source(),
            INSPECTION_LOCK_TOPIC: _source(last_time=7.0),
        },
    )
    assert locked is True
    assert any(reason.startswith('source_stale:') for reason in reasons)


def test_production_all_required_sources_healthy_false_unlocks():
    """生产两路必需来源均新鲜 false 时才允许解锁。"""
    locked, reasons, statuses, _graph_ok, _detail = _evaluate(
        'production', True,
        {GAIT_TOPIC: _source(), INSPECTION_LOCK_TOPIC: _source()},
    )
    assert locked is False
    assert reasons == ['all_required_sources_fresh_false']
    assert all(status['required'] for status in statuses)


def test_isolated_absent_inspection_does_not_lock_or_fake_value():
    """isolated 可跳过 inspection，但证据必须保留真实 unseen/None。"""
    locked, _reasons, statuses, graph_ok, _detail = _evaluate(
        'isolated_line_validation', False, {GAIT_TOPIC: _source()},
    )
    inspection = _source_status(statuses, INSPECTION_LOCK_TOPIC)
    assert graph_ok is True
    assert locked is False
    assert inspection == {
        'topic': INSPECTION_LOCK_TOPIC,
        'required': False,
        'seen': False,
        'fresh': False,
        'value': None,
        'age_sec': None,
        'reason': 'SKIPPED_BY_PROFILE:isolated_line_validation',
        'fault': None,
    }


def test_isolated_gait_unseen_stays_locked():
    """isolated 仍要求 gait provider 已真实出现。"""
    locked, reasons, _statuses, _graph_ok, _detail = _evaluate(
        'isolated_line_validation', False, {},
    )
    assert locked is True
    assert 'source_unseen:{}'.format(GAIT_TOPIC) in reasons


def test_isolated_gait_stale_stays_locked():
    """isolated gait 心跳过期不能借 profile 解锁。"""
    locked, reasons, _statuses, _graph_ok, _detail = _evaluate(
        'isolated_line_validation', False,
        {GAIT_TOPIC: _source(last_time=7.0)},
    )
    assert locked is True
    assert any(reason.startswith('source_stale:') for reason in reasons)


def test_isolated_gait_true_stays_locked():
    """有效 gait true 是真实锁请求，必须立即锁定。"""
    locked, reasons, statuses, _graph_ok, _detail = _evaluate(
        'isolated_line_validation', False,
        {GAIT_TOPIC: _source(value=True)},
    )
    assert locked is True
    assert 'source_lock_requested:{}'.format(GAIT_TOPIC) in reasons
    assert _source_status(statuses, GAIT_TOPIC)['fault'] == 'lock_requested'


def test_isolated_gait_healthy_false_unlocks():
    """isolated 只有 gait 新鲜且明确 false 才能解锁。"""
    locked, reasons, statuses, _graph_ok, _detail = _evaluate(
        'isolated_line_validation', False, {GAIT_TOPIC: _source()},
    )
    assert locked is False
    assert reasons == ['all_required_sources_fresh_false']
    gait = _source_status(statuses, GAIT_TOPIC)
    assert gait['required'] is True
    assert gait['seen'] is True
    assert gait['fresh'] is True
    assert gait['value'] is False


@pytest.mark.parametrize(
    ('profile', 'start_mission_nodes', 'expected_detail'),
    [
        ('production', False, 'PRODUCTION_PROFILE_REQUIRES_MISSION_NODES'),
        (
            'isolated_line_validation', True,
            'READINESS_PROFILE_GRAPH_MISMATCH',
        ),
    ],
)
def test_profile_graph_mismatch_stays_locked(
    profile, start_mission_nodes, expected_detail,
):
    """profile/图混搭即使所有心跳健康也不得解锁。"""
    locked, reasons, _statuses, graph_ok, graph_detail = _evaluate(
        profile, start_mission_nodes,
        {GAIT_TOPIC: _source(), INSPECTION_LOCK_TOPIC: _source()},
    )
    assert graph_ok is False
    assert graph_detail == expected_detail
    assert locked is True
    assert expected_detail in reasons


def test_unknown_profile_is_rejected():
    """未知 profile 不得静默回退 production 或 isolated。"""
    with pytest.raises(ValueError, match='readiness_profile is not supported'):
        _evaluate('unknown', False, {})
