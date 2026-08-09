"""isolated_line_validation readiness profile 的无 ROS 安全边界测试。"""

import pytest

from rk_bringup.non_arm_competition_contract import ReadinessCheck
from rk_bringup.readiness_profile import (
    DEFAULT_READINESS_PROFILE,
    ISOLATED_LINE_VALIDATION_PROFILE,
    ISOLATED_PROFILE_SKIPPED_CHECKS,
    profile_skips_check,
    readiness_profile_graph_check,
    skipped_by_profile_detail,
    validate_readiness_profile,
)


def test_default_profile_is_production_and_preserves_all_checks():
    """生产默认及 mission 图组合不得产生 validation 豁免。"""
    assert DEFAULT_READINESS_PROFILE == 'production'
    assert validate_readiness_profile(DEFAULT_READINESS_PROFILE) == 'production'
    assert readiness_profile_graph_check('production', True)[0] is True
    assert not any(
        profile_skips_check('production', label)
        for label in ISOLATED_PROFILE_SKIPPED_CHECKS
    )


@pytest.mark.parametrize(
    ('profile', 'start_mission_nodes', 'detail'),
    [
        ('production', False, 'PRODUCTION_PROFILE_REQUIRES_MISSION_NODES'),
        ('isolated_line_validation', True, 'READINESS_PROFILE_GRAPH_MISMATCH'),
    ],
)
def test_profile_graph_mismatches_fail_closed(
    profile, start_mission_nodes, detail,
):
    """错误 profile/图组合不能静默进入混合运行态。"""
    assert readiness_profile_graph_check(profile, start_mission_nodes) == (
        False, detail,
    )


def test_isolated_profile_skips_only_disabled_mission_action_checks():
    """validation 只标记明确未启动的业务 action，不把它们伪装成实测 PASS。"""
    assert readiness_profile_graph_check(
        ISOLATED_LINE_VALIDATION_PROFILE, False,
    )[0] is True
    assert all(
        profile_skips_check(ISOLATED_LINE_VALIDATION_PROFILE, label)
        for label in ISOLATED_PROFILE_SKIPPED_CHECKS
    )
    skipped = ReadinessCheck(
        'white_bar_action_idle', True,
        skipped_by_profile_detail(ISOLATED_LINE_VALIDATION_PROFILE),
        critical=False,
    ).as_dict()
    assert skipped == {
        'name': 'white_bar_action_idle',
        'ok': True,
        'detail': 'SKIPPED_BY_PROFILE:isolated_line_validation',
        'critical': False,
    }


@pytest.mark.parametrize(
    'hard_check',
    [
        # SDK/current-instance error、forwarder/mux/follower、相机与感知
        # freshness 都必须继续由真正 readiness evaluate() 失败处理。
        'hardware_sdk_server_ready',
        'hardware_udp_forwarder_started',
        'SDK_MOTION_BACKEND_READY',
        'line_follower_status_available',
        'final_cmd_single_command_mux_publisher',
        'LINE_CAMERA_READY',
        'line_track_fresh',
        'final_cmd_fresh_and_zero',
        'estop_state_fresh_and_false',
        'front_jump_cleanup_guard_clean',
    ],
)
def test_isolated_profile_never_skips_hardware_or_final_motion_gates(hard_check):
    """缺 SDK/follower/mux/相机/LineTrack 或 current SDK_ERROR 绝不能被 profile 掩盖。"""
    assert not profile_skips_check(
        ISOLATED_LINE_VALIDATION_PROFILE, hard_check,
    )


def test_unknown_profile_is_rejected_not_defaulted():
    """拼写错误必须中止启动，避免意外套用较宽的 validation 契约。"""
    with pytest.raises(ValueError, match='readiness_profile is not supported'):
        validate_readiness_profile('maybe_validation')
