"""巡线安全锁的启动 profile 与纯判定逻辑。

profile 只决定哪些已配置来源在当前启动图中是必需的；它不能伪造心跳，也
不能改变任何来源的实际值。生产图要求全部来源，isolated 图仅跳过由
``start_mission_nodes=false`` 明确移除的 inspection provider。
"""

from __future__ import annotations

from typing import Dict, Iterable, List, Mapping, Tuple


PRODUCTION_PROFILE = 'production'
ISOLATED_LINE_VALIDATION_PROFILE = 'isolated_line_validation'
SUPPORTED_PROFILES = frozenset({
    PRODUCTION_PROFILE,
    ISOLATED_LINE_VALIDATION_PROFILE,
})
INSPECTION_LOCK_TOPIC = '/gait/control_lock_req/inspection'


def validate_profile(profile: object) -> str:
    """返回受支持 profile；未知值必须中止，不能静默放宽安全门。"""
    normalized = str(profile).strip()
    if normalized not in SUPPORTED_PROFILES:
        raise ValueError('readiness_profile is not supported')
    return normalized


def graph_contract(
    profile: object, start_mission_nodes: bool,
) -> Tuple[bool, str]:
    """检查 profile 与启动图的一致性，错误组合始终保持锁定。"""
    normalized = validate_profile(profile)
    if normalized == ISOLATED_LINE_VALIDATION_PROFILE and start_mission_nodes:
        return False, 'READINESS_PROFILE_GRAPH_MISMATCH'
    if normalized == PRODUCTION_PROFILE and not start_mission_nodes:
        return False, 'PRODUCTION_PROFILE_REQUIRES_MISSION_NODES'
    return True, 'profile={} start_mission_nodes={}'.format(
        normalized, bool(start_mission_nodes),
    )


def required_source_map(
    profile: object, input_topics: Iterable[str],
) -> Dict[str, bool]:
    """生成显式 required-set；isolated 只豁免 inspection 锁来源。"""
    normalized = validate_profile(profile)
    return {
        topic: not (
            normalized == ISOLATED_LINE_VALIDATION_PROFILE
            and topic == INSPECTION_LOCK_TOPIC
        )
        for topic in input_topics
    }


def evaluate_lock(
    *,
    profile: object,
    start_mission_nodes: bool,
    input_topics: Iterable[str],
    sources: Mapping[str, Mapping[str, object]],
    now: float,
    source_timeout_sec: float,
    shutting_down: bool = False,
) -> Tuple[bool, List[str], List[dict], bool, str]:
    """按 required-set 判定全局锁，并返回可持久化的逐来源证据。"""
    normalized = validate_profile(profile)
    topics = list(input_topics)
    required = required_source_map(normalized, topics)
    graph_ok, graph_detail = graph_contract(
        normalized, bool(start_mission_nodes),
    )
    locked = not graph_ok
    reasons: List[str] = []
    statuses = []

    if not graph_ok:
        reasons.append(graph_detail)
    if shutting_down:
        locked = True
        reasons.append('arbiter_shutting_down')

    for topic in topics:
        info = sources.get(topic, {})
        seen = bool(info.get('seen', False))
        value = bool(info.get('value', False)) if seen else None
        last_time = float(info.get('last_time', 0.0))
        age = float(now) - last_time if seen else None
        fresh = bool(seen and age is not None and age <= source_timeout_sec)
        is_required = required[topic]
        status = {
            'topic': topic,
            'required': is_required,
            'seen': seen,
            'fresh': fresh,
            'value': value,
            'age_sec': round(age, 6) if age is not None else None,
            'reason': '',
            'fault': None,
        }

        if not is_required:
            status['reason'] = 'SKIPPED_BY_PROFILE:{}'.format(normalized)
        elif not seen:
            locked = True
            status['fault'] = 'unseen'
            reasons.append('source_unseen:{}'.format(topic))
        elif not fresh:
            locked = True
            status['fault'] = 'stale'
            reasons.append('source_stale:{} age={:.3f}s'.format(topic, age))
        elif value:
            locked = True
            status['fault'] = 'lock_requested'
            reasons.append('source_lock_requested:{}'.format(topic))
        statuses.append(status)

    if not locked:
        reasons.append('all_required_sources_fresh_false')
    return locked, reasons, statuses, graph_ok, graph_detail
