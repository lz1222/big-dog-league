"""Static and pure-unit guards for the formal non-arm competition launch."""

import ast
from pathlib import Path
import sys

import pytest
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_ROOT = PACKAGE_ROOT.parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from rk_bringup.non_arm_competition_contract import (  # noqa: E402
    DEFAULT_IMAGE_TOPIC,
    FORBIDDEN_FORMAL_NODE_MARKERS,
    REQUIRED_FORMAL_NODES,
    TEST_ONLY_SMOKE_HELPER_MARKER,
    bool_from_launch_value,
    hardware_processes_allowed,
    is_zero_twist,
    route_is_wait_start,
    status_is_terminal_or_idle,
    smoke_test_helper_status,
    validate_timeout_relationships,
)


FORMAL_CONFIG = (
    PACKAGE_ROOT / 'config' / 'non_arm_competition_params.yaml'
)
FORMAL_LAUNCH = (
    PACKAGE_ROOT / 'launch' / 'competition_non_arm.launch.py'
)


def read_formal_config():
    with FORMAL_CONFIG.open('r', encoding='utf-8') as stream:
        return yaml.safe_load(stream)


def node_executables_from_launch():
    """Extract literal Node executables without importing ROS launch."""
    tree = ast.parse(FORMAL_LAUNCH.read_text(encoding='utf-8'))
    executables = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        if not isinstance(node.func, ast.Name) or node.func.id != 'Node':
            continue
        for keyword in node.keywords:
            if (
                keyword.arg == 'executable'
                and isinstance(keyword.value, ast.Constant)
            ):
                executables.append(keyword.value.value)
    return executables


def test_formal_timeout_chain_exceeds_each_front_jump_profile_by_margin():
    """配置漂移不能把 executor/mission 超时缩短到 FrontJump 内。"""
    config = read_formal_config()
    gait = config['gait_control_node']['ros__parameters']
    executor = config['white_bar_action_executor']['ros__parameters']
    mission = config['line_course_mission_node']['ros__parameters']

    start_total, finish_total, executor_timeout = (
        validate_timeout_relationships(gait, executor, mission)
    )

    assert start_total == pytest.approx(17.0)
    assert finish_total == pytest.approx(17.0)
    assert executor_timeout == pytest.approx(22.0)
    assert mission['white_bar_action_timeout_sec'] == pytest.approx(26.0)
    assert mission['front_jump_start_worst_case_duration_sec'] == (
        pytest.approx(start_total)
    )
    assert mission['front_jump_finish_worst_case_duration_sec'] == (
        pytest.approx(finish_total)
    )


def test_timeout_chain_rejects_exact_margin_or_shorter_values():
    config = read_formal_config()
    gait = config['gait_control_node']['ros__parameters']
    executor = dict(
        config['white_bar_action_executor']['ros__parameters']
    )
    mission = dict(config['line_course_mission_node']['ros__parameters'])
    executor['action_timeout_sec'] = 20.0

    with pytest.raises(ValueError, match='executor timeout'):
        validate_timeout_relationships(gait, executor, mission)

    executor['action_timeout_sec'] = 22.0
    mission['white_bar_action_timeout_sec'] = 25.0
    with pytest.raises(ValueError, match='line-course timeout'):
        validate_timeout_relationships(gait, executor, mission)


def test_formal_launch_declares_all_required_nodes_without_excluded_nodes():
    executables = node_executables_from_launch()
    source = FORMAL_LAUNCH.read_text(encoding='utf-8')

    for executable in REQUIRED_FORMAL_NODES:
        if executable == 'go2_sdk_udp_server':
            assert executable in source
        else:
            assert executable in executables

    forbidden_executable_fragments = (
        'mock_', 'national_mission', 'arm_task', 'obstacle', 'stairs',
        'direct_route', 'standalone_direct',
    )
    for executable in executables:
        assert not any(
            fragment in executable
            for fragment in forbidden_executable_fragments
        )
    # 正式链只能有一个 gait Action server 与一个最终速度所有者；重复实例
    # 会令 readiness 的“可用”检查掩盖控制权分裂。
    assert executables.count('gait_control_node') == 1
    assert executables.count('command_mux_node') == 1


def test_formal_launch_shares_image_and_suppresses_hardware_in_smoke():
    source = FORMAL_LAUNCH.read_text(encoding='utf-8')

    assert DEFAULT_IMAGE_TOPIC == '/camera/camera/color/image_raw'
    assert "LaunchConfiguration('image_topic')" in source
    assert "'image_topic': image_topic" in source
    assert "LaunchConfiguration('software_smoke_mode')" in source
    assert "use_hardware_realsense" in source
    assert "use_hardware_sdk_server" in source
    assert "use_hardware_udp_forwarder" in source
    assert "use_smoke_publisher" in source
    assert "SOFTWARE_SMOKE_MODE" in source
    assert "front_jump.sdk_action_executable': selected_sdk_helper" in source
    assert "sdk_action_executable': selected_sdk_helper" in source
    assert "front_jump.cleanup_guard_path': selected_cleanup_guard" in source
    assert "front_jump.software_smoke_mode" in source
    assert "'fake_sdk_action_executable', default_value=''" in source


@pytest.mark.parametrize(
    ('hardware_mode', 'smoke_mode', 'expected'),
    [
        ('true', 'false', True),
        ('true', 'true', False),
        ('false', 'false', False),
        ('false', 'true', False),
    ],
)
def test_smoke_mode_preempts_hardware_backend(
    hardware_mode, smoke_mode, expected
):
    assert hardware_processes_allowed(hardware_mode, smoke_mode) is expected


def test_readiness_pure_guards_fail_closed_on_invalid_status_or_twist():
    assert bool_from_launch_value('YES') is True
    assert bool_from_launch_value('not-a-bool') is False
    assert is_zero_twist((0.0,) * 6)
    assert not is_zero_twist((0.0,) * 5)
    assert not is_zero_twist((0.0, 0.0, 0.0, 0.0, 0.0, float('nan')))
    assert status_is_terminal_or_idle({'status': 'IDLE'})
    assert not status_is_terminal_or_idle({'state': 'RUNNING'})
    assert route_is_wait_start({
        'route_phase': 'WAIT_START',
        'mission_started': False,
        'active_action': '',
    })
    assert not route_is_wait_start({
        'route_phase': 'WAIT_START',
        'mission_started': True,
        'active_action': '',
    })


def test_readiness_marks_all_known_direct_velocity_tools_as_forbidden():
    """即使工具暂未发布 Twist，正式 readiness 也必须拒绝其同图运行。"""
    for marker in (
        '/keyboard_route_node',
        '/cmd_vel_speed_sweep_node',
        '/two_step_walk_test_node',
        '/gait_basic_test_node',
    ):
        assert marker in FORBIDDEN_FORMAL_NODE_MARKERS


def test_smoke_helper_requires_normalized_marked_elf(tmp_path):
    """readiness 必须拒绝 /usr/bin/true、脚本和没有仓库标识的 ELF。"""
    helper = tmp_path / 'fake_sdk_motion_helper'
    helper.write_bytes(b'\x7fELF' + TEST_ONLY_SMOKE_HELPER_MARKER)
    helper.chmod(0o700)

    assert smoke_test_helper_status(str(helper)) == (
        True,
        'marked_test_only_elf',
    )
    assert smoke_test_helper_status('/usr/bin/true')[0] is False
    assert smoke_test_helper_status('relative-helper') == (
        False,
        'not_absolute',
    )


def test_start_script_publishes_one_start_and_acceptance_uses_compiled_elf():
    start_script = (
        PACKAGE_ROOT / 'scripts' / 'mission_start.sh'
    ).read_text(encoding='utf-8')
    acceptance_script = (
        WORKSPACE_ROOT / 'scripts' / 'accept_non_arm_competition.sh'
    ).read_text(encoding='utf-8')

    # 失败回滚可以发布 /mission/stop；正式 start 先等关键订阅者发现，
    # 仍只执行一次 native publisher.publish，不能退化成多次 topic pub 重试。
    assert 'publish_formal_start_once' in start_script
    assert 'publisher.get_subscription_count() < required_subscribers' in start_script
    assert start_script.count('publisher.publish(message)') == 1
    assert 'MISSION_START_STATE run_id=' in start_script
    assert '/competition/check_readiness' in start_script
    assert 'fake_sdk_motion_helper.c' in acceptance_script
    assert 'fake_sdk_action_executable:="$FAKE_HELPER"' in acceptance_script
    assert 'software_smoke_mode:=true' in acceptance_script
    assert '--install-base "${CHECK_DIR}/install"' in acceptance_script
    assert 'RK_ROS_OVERLAY_SETUP' in acceptance_script
    assert 'go2_sdk_server_process_running' in acceptance_script
    assert 'FAKE_HELPER_SEEN_FILE' in acceptance_script
    assert 'go2_sdk_udp_server' not in acceptance_script.split(
        'ros2 launch rk_bringup competition_non_arm.launch.py', 1
    )[1]
    assert 'run_fault_injection_matrix' in acceptance_script
    for test_name in (
        'test_start_ready_core.py',
        'test_non_arm_route_phase_core.py',
        'test_white_bar_action_core.py',
        'test_white_bar_stage_command_core.py',
        'test_inspection_action_core.py',
        'test_front_jump_supervisor.py',
    ):
        assert test_name in acceptance_script
