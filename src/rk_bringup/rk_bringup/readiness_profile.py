"""非机械臂 readiness profile 的纯判定边界。

本模块不依赖 ROS，集中限定 validation profile 的唯一豁免范围。这样启动
参数组合和每个检查名称都能在不接触机器人或 ROS 图的单元测试中 fail-closed。
"""

from __future__ import annotations

from typing import Tuple


DEFAULT_READINESS_PROFILE = 'production'
ISOLATED_LINE_VALIDATION_PROFILE = 'isolated_line_validation'
SUPPORTED_READINESS_PROFILES = frozenset({
    DEFAULT_READINESS_PROFILE,
    ISOLATED_LINE_VALIDATION_PROFILE,
})

# 这些项目只属于 start_mission_nodes=false 时明确不启动的 mission/action
# 组件。任何 SDK、相机、tracker、follower、mux、estop、最终出口检查都不能
# 加入此集合，否则 validation 会错误放宽真实硬件安全门。
ISOLATED_PROFILE_SKIPPED_CHECKS = frozenset({
    'execute_motion_action_server',
    'line_course_state_available',
    'white_stage_publisher_status_available',
    'white_bar_action_idle',
    'inspection_action_idle',
    'route_wait_start_without_active_action',
})


def validate_readiness_profile(profile: object) -> str:
    """返回受支持 profile；未知值必须拒绝，不能静默回退生产或 validation。"""
    normalized = str(profile).strip()
    if normalized not in SUPPORTED_READINESS_PROFILES:
        raise ValueError('readiness_profile is not supported')
    return normalized


def readiness_profile_graph_check(
    profile: object, start_mission_nodes: bool,
) -> Tuple[bool, str]:
    """验证 profile 和正式 launch 图是否一致，错误组合一律 fail-closed。"""
    normalized = validate_readiness_profile(profile)
    if normalized == ISOLATED_LINE_VALIDATION_PROFILE and start_mission_nodes:
        return False, 'READINESS_PROFILE_GRAPH_MISMATCH'
    if normalized == DEFAULT_READINESS_PROFILE and not start_mission_nodes:
        return False, 'PRODUCTION_PROFILE_REQUIRES_MISSION_NODES'
    return True, 'profile={} start_mission_nodes={}'.format(
        normalized, bool(start_mission_nodes),
    )


def profile_skips_check(profile: object, check_name: str) -> bool:
    """仅返回 validation profile 明确允许跳过的业务检查。"""
    return (
        validate_readiness_profile(profile)
        == ISOLATED_LINE_VALIDATION_PROFILE
        and check_name in ISOLATED_PROFILE_SKIPPED_CHECKS
    )


def skipped_by_profile_detail(profile: object) -> str:
    """形成证据中可区分的跳过标记，绝不伪装成已实际通过。"""
    normalized = validate_readiness_profile(profile)
    return 'SKIPPED_BY_PROFILE:{}'.format(normalized)
