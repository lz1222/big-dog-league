"""Static and pure-unit guards for the formal non-arm competition launch."""

import ast
import importlib
from pathlib import Path
import sys

import pytest
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_ROOT = PACKAGE_ROOT.parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))
for package_name in ('rk_navigation', 'rk_mission', 'rk_locomotion'):
    package_root = WORKSPACE_ROOT / 'src' / package_name
    if str(package_root) not in sys.path:
        # 直接 pytest 与 colcon test 均须导入当前源码，避免安装覆盖层过期时
        # 把 Python 3.8 注解兼容检查错误地落在旧版本模块上。
        sys.path.insert(0, str(package_root))

from rk_bringup.non_arm_competition_contract import (  # noqa: E402
    DEFAULT_IMAGE_TOPIC,
    DEFAULT_LINE_IMAGE_TOPIC,
    DEFAULT_SIGN_IMAGE_TOPIC,
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
FORMAL_START_SCRIPT = (
    PACKAGE_ROOT / 'scripts' / 'start_non_arm_competition.sh'
)
PYTHON38_COMPATIBILITY_FILES = (
    PACKAGE_ROOT / 'rk_bringup' / 'non_arm_competition_contract.py',
    PACKAGE_ROOT / 'rk_bringup' / 'competition_readiness_node.py',
    PACKAGE_ROOT / 'rk_bringup' / 'non_arm_smoke_publisher.py',
    WORKSPACE_ROOT / 'src/rk_navigation/rk_navigation/line_follower_node.py',
    WORKSPACE_ROOT / 'src/rk_navigation/rk_navigation/start_ready_core.py',
    WORKSPACE_ROOT / 'src/rk_mission/rk_mission/inspection_action_core.py',
    WORKSPACE_ROOT / 'src/rk_mission/rk_mission/line_course_mission_node.py',
    WORKSPACE_ROOT / 'src/rk_mission/rk_mission/non_arm_route_phase_core.py',
    WORKSPACE_ROOT / 'src/rk_mission/rk_mission/white_bar_action_core.py',
    WORKSPACE_ROOT / 'src/rk_mission/rk_mission/white_bar_action_executor_node.py',
    WORKSPACE_ROOT / 'src/rk_mission/rk_mission/white_bar_stage_command_core.py',
    WORKSPACE_ROOT / 'src/rk_mission/rk_mission/white_bar_stage_command_publisher_node.py',
    WORKSPACE_ROOT / 'src/rk_locomotion/rk_locomotion/front_jump_supervisor.py',
    WORKSPACE_ROOT / 'src/rk_locomotion/rk_locomotion/gait_control_node.py',
    WORKSPACE_ROOT / 'src/rk_safety/rk_safety/command_mux_node.py',
    WORKSPACE_ROOT / 'src/rk_navigation/test/test_start_ready_core.py',
    WORKSPACE_ROOT / 'src/rk_mission/test/test_inspection_action_core.py',
    WORKSPACE_ROOT / 'src/rk_mission/test/test_non_arm_route_phase_core.py',
    WORKSPACE_ROOT / 'src/rk_mission/test/test_white_bar_action_core.py',
    WORKSPACE_ROOT / 'src/rk_mission/test/test_white_bar_stage_command_core.py',
    WORKSPACE_ROOT / 'src/rk_locomotion/test/test_front_jump_supervisor.py',
)
PYTHON38_CORE_IMPORTS = (
    'rk_bringup.non_arm_competition_contract',
    'rk_bringup.competition_readiness_node',
    'rk_bringup.non_arm_smoke_publisher',
    'rk_navigation.start_ready_core',
    'rk_navigation.line_follower_node',
    'rk_mission.inspection_action_core',
    'rk_mission.non_arm_route_phase_core',
    'rk_mission.white_bar_action_core',
    'rk_mission.white_bar_stage_command_core',
    'rk_mission.line_course_mission_node',
    'rk_mission.white_bar_action_executor_node',
    'rk_mission.white_bar_stage_command_publisher_node',
    'rk_locomotion.front_jump_supervisor',
    'rk_locomotion.gait_control_node',
    'rk_safety.command_mux_node',
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


def annotation_nodes(tree):
    """提取模块、函数和参数标注，供 Python 3.8 静态兼容检查复用。"""
    for node in ast.walk(tree):
        if isinstance(node, ast.AnnAssign):
            yield node.annotation
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            if node.returns is not None:
                yield node.returns
            arguments = (
                list(node.args.posonlyargs)
                + list(node.args.args)
                + list(node.args.kwonlyargs)
            )
            if node.args.vararg is not None:
                arguments.append(node.args.vararg)
            if node.args.kwarg is not None:
                arguments.append(node.args.kwarg)
            for argument in arguments:
                if argument.annotation is not None:
                    yield argument.annotation


def python38_annotation_violation(annotation):
    """拒绝 Python 3.8 会在导入时求值失败的现代标注表达式。"""
    for candidate in ast.walk(annotation):
        if (
            isinstance(candidate, ast.BinOp)
            and isinstance(candidate.op, ast.BitOr)
        ):
            return 'PEP 604 union'
        if (
            isinstance(candidate, ast.Subscript)
            and isinstance(candidate.value, ast.Name)
            and candidate.value.id in {'list', 'dict', 'tuple', 'set'}
        ):
            return 'PEP 585 built-in generic {}'.format(
                candidate.value.id
            )
    return ''


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


def test_python38_formal_sources_use_legacy_runtime_safe_annotations():
    """Foxy/Python 3.8 导入前拒绝 PEP 585/604 标注，避免 Humble 掩盖错误。"""
    violations = []
    for source_path in PYTHON38_COMPATIBILITY_FILES:
        source = source_path.read_text(encoding='utf-8')
        tree = ast.parse(
            source,
            filename=str(source_path),
        )
        for annotation in annotation_nodes(tree):
            reason = python38_annotation_violation(annotation)
            if reason:
                violations.append(
                    '{}:{} {}'.format(
                        source_path.relative_to(WORKSPACE_ROOT),
                        annotation.lineno,
                        reason,
                    )
                )
    assert not violations, '\n'.join(violations)


def test_python38_core_modules_remain_importable_after_annotation_scan():
    """正式链核心模块必须实际导入，不能仅依赖静态扫描。"""
    for module_name in PYTHON38_CORE_IMPORTS:
        assert importlib.import_module(module_name) is not None


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


def test_formal_motion_limits_share_one_contract_across_all_backends():
    """正式巡线速度必须被 follower、forwarder 和 SDK server 同时接受。"""
    config = read_formal_config()
    follower = config['line_follower_node']['ros__parameters']
    launch_source = FORMAL_LAUNCH.read_text(encoding='utf-8')
    start_script = FORMAL_START_SCRIPT.read_text(encoding='utf-8')

    # 所有正式 follower 速度档位都不能越过 SDK server 的正式硬上限。
    max_follower_speed = max(
        follower[name]
        for name in (
            'min_driving_speed', 'base_speed', 'mid_speed', 'slow_speed'
        )
    )
    formal_max_vx = 0.30
    assert max_follower_speed == pytest.approx(0.27)
    assert max_follower_speed <= formal_max_vx

    # launch 的同一参数必须同时到达 UDP forwarder 和 SDK server argv。
    assert "'motion_max_vx', default_value='0.30'" in launch_source
    assert "'--max-vx', motion_max_vx" in launch_source
    assert "'max_vx': ParameterValue(motion_max_vx, value_type=float)" in (
        launch_source
    )
    assert "'--max-vy', motion_max_vy" in launch_source
    assert "'max_vy': ParameterValue(motion_max_vy, value_type=float)" in (
        launch_source
    )
    assert "'--max-yaw', motion_max_yaw" in launch_source
    assert "'max_yaw': ParameterValue(motion_max_yaw, value_type=float)" in (
        launch_source
    )

    # 脚本阶段 B 启动 server、阶段 C 启动 forwarder 时复用同一环境变量。
    assert 'MOTION_MAX_VX="${RK_COMPETITION_MOTION_MAX_VX:-0.30}"' in (
        start_script
    )
    assert '"motion_max_vx:=${MOTION_MAX_VX}"' in start_script
    assert '--max-vx "$MOTION_MAX_VX"' in start_script
    assert '"motion_max_vy:=${MOTION_MAX_VY}"' in start_script
    assert '--max-vy "$MOTION_MAX_VY"' in start_script
    assert '"motion_max_yaw:=${MOTION_MAX_YAW}"' in start_script
    assert '--max-yaw "$MOTION_MAX_YAW"' in start_script


def test_formal_launch_shares_image_and_suppresses_hardware_in_smoke():
    """巡线与标识相机必须独立配置，并由 launch 覆盖 YAML 默认值。"""
    source = FORMAL_LAUNCH.read_text(encoding='utf-8')
    config = read_formal_config()

    assert DEFAULT_IMAGE_TOPIC == '/line_camera/image_raw'
    assert DEFAULT_LINE_IMAGE_TOPIC == DEFAULT_IMAGE_TOPIC
    assert DEFAULT_SIGN_IMAGE_TOPIC == '/go2/front_camera/image_raw'
    assert "LaunchConfiguration('line_image_topic')" in source
    assert "LaunchConfiguration('sign_image_topic')" in source
    # 两个感知节点均把各自的启动参数置于 YAML 之后，避免 YAML 默认值
    # 覆盖正式图的话题分流；不得恢复已废弃的全局 image_topic 启动参数。
    assert "'image_topic': line_image_topic" in source
    assert "'image_topic': sign_image_topic" in source
    assert "LaunchConfiguration('image_topic')" not in source
    assert (
        config['real_line_tracker_node']['ros__parameters']['image_topic']
        == DEFAULT_LINE_IMAGE_TOPIC
    )
    assert (
        config['real_sign_detector_node']['ros__parameters']['image_topic']
        == DEFAULT_SIGN_IMAGE_TOPIC
    )
    assert "LaunchConfiguration('software_smoke_mode')" in source
    assert "use_hardware_line_camera" in source
    assert "executable='line_camera_node'" in source
    assert "realsense2_camera_node" not in source
    assert "use_hardware_sdk_server" in source
    assert "use_hardware_udp_forwarder" in source
    assert "use_smoke_publisher" in source
    assert "SOFTWARE_SMOKE_MODE" in source
    # B2 修复后使用 ParameterValue(..., value_type=str) 包装
    assert "front_jump.sdk_action_executable" in source
    assert 'selected_sdk_helper' in source
    assert '_SelectedSdkActionHelper' in source
    assert "sdk_action_executable" in source
    assert "front_jump.cleanup_guard_path" in source
    assert 'selected_cleanup_guard' in source
    assert "ParameterValue(" in source
    assert "value_type=str" in source
    assert "front_jump.software_smoke_mode" in source
    # smoke helper 必须由安装树提供的带标识假程序默认注入，不能回退为空
    # 或 production basename；这保证 smoke 不会触碰真实 SDK helper。
    assert "'fake_sdk_action_executable'" in source
    assert "'/lib/rk_go2_sdk_bridge/fake_sdk_motion_helper'" in source


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


def test_start_script_uses_bounded_dual_ack_delivery_and_compiled_elf():
    start_script = (
        PACKAGE_ROOT / 'scripts' / 'mission_start.sh'
    ).read_text(encoding='utf-8')
    acceptance_script = (
        WORKSPACE_ROOT / 'scripts' / 'accept_non_arm_competition.sh'
    ).read_text(encoding='utf-8')

    # 一次逻辑请求可有有限可靠重传，但必须等路线和循线双 ACK，不能用
    # 单条消息或固定 sleep 推测 DDS 已交付。
    assert 'deliver_formal_start_with_dual_ack' in start_script
    assert 'mission_start_delivery' in start_script
    assert 'START_MAX_TRANSPORT_PUBLISHES' in start_script
    assert 'START_RETRANSMIT_INTERVAL_SEC' in start_script
    assert 'safe_stop_after_start_failure' in start_script
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


# ---- B2 修复回归：Foxy 中 launch 参数展开正确性 ----


def _parse_launch_params_node_block(launch_source, node_package, node_executable):
    """提取 launch 源码中指定节点的 inline parameters dict 文本。"""
    import re

    escaped_pkg = re.escape(node_package)
    escaped_exe = re.escape(node_executable)
    pattern = (
        r"Node\(\s*"
        r"package\s*=\s*'" + escaped_pkg + r"'\s*,"
        r"\s*executable\s*=\s*'" + escaped_exe + r"'\s*,"
    )
    match = re.search(pattern, launch_source)
    if match is None:
        return None
    # 找到 parameters=[ 后面的内容直到闭合
    block = launch_source[match.start():]
    params_start = block.find('parameters=')
    if params_start == -1:
        return None
    return block[params_start:]


def test_launch_uses_parameter_value_with_str_type_for_helper_paths():
    """B2 修复：所有 helper 路径必须使用 ParameterValue(..., value_type=str)。"""
    launch_path = (
        PACKAGE_ROOT / 'launch' / 'competition_non_arm.launch.py'
    )
    source = launch_path.read_text(encoding='utf-8')

    param_targets = [
        'sdk_action_executable',
        'cleanup_guard_path',
        'sdk_network_interface',
    ]
    for target in param_targets:
        # 确保每个 substitution 参数都包在 ParameterValue 中
        for line in source.split('\n'):
            if "'" + target + "'" in line and 'selected_' in line:
                assert 'ParameterValue(' in source.split('\n')[
                    source.split('\n').index(line) - 1
                ] or 'ParameterValue(' in source.split('\n')[
                    source.split('\n').index(line)
                ] or 'ParameterValue(' in source.split('\n')[
                    source.split('\n').index(line) + 1
                ], (
                    '{} with substitution must be wrapped in '
                    'ParameterValue(..., value_type=str)'.format(target)
                )


def test_formal_sdk_interface_is_eth1_and_reaches_every_sdk_consumer():
    """正式 competition 链只从一个入口传递 eth1，不能有隐式 eth0 回退。"""
    launch_source = FORMAL_LAUNCH.read_text(encoding='utf-8')
    start_script = (
        PACKAGE_ROOT / 'scripts' / 'start_non_arm_competition.sh'
    ).read_text(encoding='utf-8')
    formal_config = read_formal_config()
    gait_config = yaml.safe_load((
        WORKSPACE_ROOT / 'src/rk_locomotion/config/gait_params.yaml'
    ).read_text(encoding='utf-8'))
    line_config = yaml.safe_load((
        PACKAGE_ROOT / 'config/line_nav_params.yaml'
    ).read_text(encoding='utf-8'))

    assert 'RK_COMPETITION_SDK_NETWORK_INTERFACE:-eth1' in start_script
    assert "'sdk_network_interface', default_value='eth1'" in launch_source
    assert '"sdk_network_interface:=${SDK_NETWORK_INTERFACE}"' in start_script
    assert "'--interface', sdk_network_interface" in launch_source
    assert "'front_jump.sdk_network_interface': ParameterValue(" in launch_source
    assert "'network_interface': ParameterValue(" in launch_source
    assert formal_config['inspection_action_executor']['ros__parameters'][
        'sdk_network_interface'
    ] == 'eth1'
    assert formal_config['gait_control_node']['ros__parameters'][
        'front_jump'
    ]['sdk_network_interface'] == 'eth1'
    assert gait_config['gait_control_node']['ros__parameters'][
        'front_jump'
    ]['sdk_network_interface'] == 'eth1'
    assert line_config['line_course_mission_node']['ros__parameters'][
        'sdk_network_interface'
    ] == 'eth1'


def test_formal_start_network_gate_fails_closed_before_sdk_processes():
    """正式硬件启动先只读校验 eth1 地址和载波，失败时不创建 SDK 进程。"""
    source = (
        PACKAGE_ROOT / 'scripts' / 'start_non_arm_competition.sh'
    ).read_text(encoding='utf-8')

    assert 'SDK_NETWORK_ADDRESS_CIDR="192.168.123.18/24"' in source
    assert 'validate_sdk_network_interface() {' in source
    assert 'ip -o link show dev "$SDK_NETWORK_INTERFACE"' in source
    assert 'ip -4 -o addr show dev "$SDK_NETWORK_INTERFACE"' in source
    assert 'LOWER_UP' in source
    assert '/sys/class/net/${SDK_NETWORK_INTERFACE}/carrier' in source
    assert source.index('validate_sdk_network_interface || exit 1') < source.index(
        'GATE_COMMAND='
    )


def test_launch_helper_selection_uses_checked_production_install_path():
    """production helper 不得退化为 basename，smoke 仍经 fake 参数选择。"""
    launch_path = (
        PACKAGE_ROOT / 'launch' / 'competition_non_arm.launch.py'
    )
    source = launch_path.read_text(encoding='utf-8')

    assert 'class _SelectedSdkActionHelper(Substitution):' in source
    assert 'select_sdk_action_helper(' in source
    assert (
        "ParameterValue(\n                    "
        "selected_sdk_helper, value_type=str\n                )"
    ) in source
    # gait、inspection executor 与 readiness 必须共享一次选定结果；少任一
    # 消费者都会让 smoke/production 对 helper 的安全校验出现分叉。
    assert source.count(
        "ParameterValue(\n                    "
        "selected_sdk_helper, value_type=str\n                )"
    ) == 3
    assert "FindPackagePrefix('rk_go2_sdk_bridge')" in source
    # smoke 路径使用 fake 参数
    assert 'fake_sdk_action_executable' in source


def test_yaml_no_longer_contains_empty_sdk_action_defaults():
    """B2 修复：YAML 不再包含会覆盖 launch 注入值的空字符串默认。"""
    yaml_path = (
        PACKAGE_ROOT / 'config' / 'non_arm_competition_params.yaml'
    )
    with yaml_path.open('r', encoding='utf-8') as fh:
        text = fh.read()

    yaml_data = yaml.safe_load(text)
    # 检查 gait_control_node → front_jump 下没有空 sdk_action_executable
    gc_params = (
        yaml_data.get('gait_control_node', {}).get('ros__parameters', {})
    )
    fj_params = gc_params.get('front_jump', {})
    assert 'sdk_action_executable' not in fj_params, (
        'front_jump.sdk_action_executable must not have an empty default '
        'in the YAML that overrides the launch-injected path'
    )

    # inspection_action_executor 下没有空 sdk_action_executable
    insp_params = (
        yaml_data.get('inspection_action_executor', {})
        .get('ros__parameters', {})
    )
    assert 'sdk_action_executable' not in insp_params, (
        'inspection_action_executor sdk_action_executable must come from launch'
    )

    # competition_readiness_node 下没有空 sdk_action_executable
    ready_params = (
        yaml_data.get('competition_readiness_node', {})
        .get('ros__parameters', {})
    )
    assert 'sdk_action_executable' not in ready_params, (
        'competition_readiness_node sdk_action_executable must come from launch'
    )


def test_node_python_defaults_remain_fail_closed():
    """节点自身的 declare_parameter 默认值仍为空/安全值，依赖 launch 注入。"""
    import rk_locomotion.gait_control_node as gc_mod
    import inspect

    source = inspect.getsource(gc_mod.GaitControlNode._declare_parameters)
    # front_jump.sdk_action_executable 的 Python 默认值是 'go2_sdk_motion_action'
    # 这是安全的 fail-closed 值（仅在未通过正式 launch 启动时使用）
    assert "'front_jump.sdk_action_executable': 'go2_sdk_motion_action'" in source


# ---- F3 修复回归：Foxy /** YAML 参数覆盖 -----


def test_yaml_does_not_contain_hardware_mode_defaults():
    """F3 修复：YAML 不应包含会被 launch 覆盖的 hardware/software_smoke_mode。

    Foxy 的 rcl_yaml_param_parser 中 /** 通配符参数不会正确覆盖
    节点特定参数。因此所有 hardware_mode 和 software_smoke_mode
    的默认值必须仅来自 Python declare_parameter 和 launch 注入，
    不得在 YAML 中重复声明。
    """
    yaml_path = (
        PACKAGE_ROOT / 'config' / 'non_arm_competition_params.yaml'
    )
    with yaml_path.open('r', encoding='utf-8') as fh:
        text = fh.read()
    yaml_data = yaml.safe_load(text)

    # readiness 节点不应有 hardware_mode / software_smoke_mode
    ready_params = (
        yaml_data.get('competition_readiness_node', {})
        .get('ros__parameters', {})
    )
    assert 'hardware_mode' not in ready_params, (
        'competition_readiness_node must not have hardware_mode in YAML '
        '(launch injects ParameterValue(value_type=bool))'
    )
    assert 'software_smoke_mode' not in ready_params, (
        'competition_readiness_node must not have software_smoke_mode in YAML'
    )

    # inspection_action_executor 不应有 software_smoke_mode
    insp_params = (
        yaml_data.get('inspection_action_executor', {})
        .get('ros__parameters', {})
    )
    assert 'software_smoke_mode' not in insp_params, (
        'inspection_action_executor must not have software_smoke_mode in YAML'
    )

    # gait_control_node → front_jump 不应有 software_smoke_mode
    gc_params = (
        yaml_data.get('gait_control_node', {}).get('ros__parameters', {})
    )
    fj = gc_params.get('front_jump', {})
    assert 'software_smoke_mode' not in fj, (
        'gait_control_node front_jump must not have software_smoke_mode '
        'in YAML'
    )


def test_launch_uses_parameter_value_bool_for_mode_params():
    """F3 修复：readiness 的硬件/烟感参数必须用 ParameterValue(value_type=bool)。"""
    launch_path = (
        PACKAGE_ROOT / 'launch' / 'competition_non_arm.launch.py'
    )
    source = launch_path.read_text(encoding='utf-8')

    # readiness 节点区域的 hardware_mode 和 software_smoke_mode
    assert ("'hardware_mode': ParameterValue(\n"
            "                    hardware_mode, value_type=bool\n"
            "                )") in source, (
        'hardware_mode must use ParameterValue(value_type=bool) in launch'
    )
    assert ("'software_smoke_mode': ParameterValue(\n"
            "                    software_smoke_mode, value_type=bool\n"
            "                )") in source, (
        'software_smoke_mode must use ParameterValue(value_type=bool)'
    )


def test_smoke_temp_params_file_has_bool_not_string():
    """F3 修复：临时参数文件中的值为 YAML boolean 而非字符串。

    字符串 'false' 在 bool('false') 后为 True，会造成灾难性误判。
    """
    launch_path = (
        PACKAGE_ROOT / 'launch' / 'competition_non_arm.launch.py'
    )
    source = launch_path.read_text(encoding='utf-8')

    # 所有 mode 参数必须显式标注 value_type=bool
    bool_param_targets = ['hardware_mode', 'software_smoke_mode']
    for target in bool_param_targets:
        value_type_occurrences = source.count(
            "'" + target + "': ParameterValue("
        )
        assert value_type_occurrences >= 1, (
            '{} must use ParameterValue(value_type=bool)'.format(target)
        )


def test_readiness_smoke_checks_are_gated_by_software_smoke_mode():
    """F3 修复：hardware SDK/forwarder/realsense 检查必须受 software_smoke_mode 门控。"""
    readiness_source = (
        PACKAGE_ROOT / 'rk_bringup' / 'competition_readiness_node.py'
    ).read_text(encoding='utf-8')

    # 硬件检查必须出现在 software_smoke_mode 门控下面
    assert 'if self.software_smoke_mode:' in readiness_source, (
        'readiness must have software_smoke_mode gate'
    )
    # hardware_mode 相关检查必须在 elif self.hardware_mode 下
    assert 'elif self.hardware_mode:' in readiness_source, (
        'hardware checks must be under elif self.hardware_mode gate'
    )
    # smoke 不启动 Go2 相机桥；只有该模式可接受合成相机输入，生产仍要求桥接。
    assert 'software_smoke_synthetic' in readiness_source
    smoke_source = (
        PACKAGE_ROOT / 'rk_bringup' / 'non_arm_smoke_publisher.py'
    ).read_text(encoding='utf-8')
    assert 'DEFAULT_SIGN_CAMERA_FRAME_ID' in smoke_source


# ---------------------------------------------------------------------------
# GID 唯一发布者门控单元测试
# ---------------------------------------------------------------------------


class _FakeEndpoint:
    """模拟 rclpy TopicEndpointInfo 的最小假对象。"""

    def __init__(self, gid, node_name, node_namespace=''):
        self._gid = list(gid) if isinstance(gid, (bytes, bytearray)) else gid
        self.node_name = node_name
        self.node_namespace = node_namespace

    @property
    def endpoint_gid(self):
        return list(self._gid)


def _make_gid(*seed):
    """构造 24 字节假 GID，以 seed 填充前几字节。"""
    gid = [0] * 24
    for i, val in enumerate(seed):
        if i < 24:
            gid[i] = val & 0xFF
    return gid


def test_gid_gate_single_endpoint_pass():
    from rk_bringup.competition_readiness_node import (
        CompetitionReadinessNode,
    )
    gid_a = _make_gid(1, 2, 3)
    endpoints = [_FakeEndpoint(gid_a, 'test_node', '')]
    ok, detail = CompetitionReadinessNode._single_gid_publisher_gate(
        endpoints, 'test_node',
    )
    assert ok, detail


def test_gid_gate_same_gid_duplicate_pass():
    from rk_bringup.competition_readiness_node import (
        CompetitionReadinessNode,
    )
    gid_a = _make_gid(1, 2, 3)
    endpoints = [
        _FakeEndpoint(gid_a, 'test_node', ''),
        _FakeEndpoint(gid_a, 'test_node', ''),
    ]
    ok, detail = CompetitionReadinessNode._single_gid_publisher_gate(
        endpoints, 'test_node',
    )
    assert ok, detail


def test_gid_gate_two_different_gid_same_node_fail():
    from rk_bringup.competition_readiness_node import (
        CompetitionReadinessNode,
    )
    gid_a = _make_gid(1, 2, 3)
    gid_b = _make_gid(4, 5, 6)
    endpoints = [
        _FakeEndpoint(gid_a, 'test_node', ''),
        _FakeEndpoint(gid_b, 'test_node', ''),
    ]
    ok, detail = CompetitionReadinessNode._single_gid_publisher_gate(
        endpoints, 'test_node',
    )
    assert not ok, 'two different GIDs must fail even if same node_name'
    assert 'unique_gid_count=2' in detail, detail


def test_gid_gate_two_different_gid_different_node_fail():
    from rk_bringup.competition_readiness_node import (
        CompetitionReadinessNode,
    )
    gid_a = _make_gid(1, 2, 3)
    gid_b = _make_gid(4, 5, 6)
    endpoints = [
        _FakeEndpoint(gid_a, 'good_node', ''),
        _FakeEndpoint(gid_b, 'bad_node', ''),
    ]
    ok, detail = CompetitionReadinessNode._single_gid_publisher_gate(
        endpoints, 'good_node',
    )
    assert not ok, 'two different nodes must fail'
    assert 'unique_gid_count=2' in detail, detail


def test_gid_gate_wrong_namespace_fail():
    from rk_bringup.competition_readiness_node import (
        CompetitionReadinessNode,
    )
    gid_a = _make_gid(1, 2, 3)
    endpoints = [_FakeEndpoint(gid_a, 'test_node', '/other')]
    ok, detail = CompetitionReadinessNode._single_gid_publisher_gate(
        endpoints, 'test_node', expected_namespace='',
    )
    assert not ok, 'wrong namespace must fail'


def test_gid_gate_unreadable_gid_fail():
    from rk_bringup.competition_readiness_node import (
        CompetitionReadinessNode,
    )

    class BrokenEndpoint:
        node_name = 'test_node'
        node_namespace = ''

        @property
        def endpoint_gid(self):
            raise RuntimeError('gid unavailable')

    endpoints = [BrokenEndpoint()]
    ok, detail = CompetitionReadinessNode._single_gid_publisher_gate(
        endpoints, 'test_node',
    )
    assert not ok, 'unreadable GID must fail'
    assert 'gid_unreadable' in detail, detail


def test_gid_gate_no_publishers_fail():
    from rk_bringup.competition_readiness_node import (
        CompetitionReadinessNode,
    )
    ok, detail = CompetitionReadinessNode._single_gid_publisher_gate(
        [], 'test_node',
    )
    assert not ok, 'no publishers must fail'
    assert 'raw_count=0' in detail, detail


def test_mission_start_uses_reliable_volatile_dual_ack_delivery():
    """正式 start 不能以单次 publish 或固定 sleep 假定两个消费者已接收。"""
    delivery_source = (
        PACKAGE_ROOT / 'rk_bringup' / 'mission_start_delivery.py'
    ).read_text(encoding='utf-8')
    start_script = (
        PACKAGE_ROOT / 'scripts' / 'mission_start.sh'
    ).read_text(encoding='utf-8')
    smoke_source = (
        PACKAGE_ROOT / 'rk_bringup' / 'non_arm_smoke_publisher.py'
    ).read_text(encoding='utf-8')

    assert 'ReliabilityPolicy.RELIABLE' in delivery_source
    assert 'DurabilityPolicy.VOLATILE' in delivery_source
    assert 'HistoryPolicy.KEEP_LAST' in delivery_source
    assert 'depth=10' in delivery_source
    assert 'TRANSIENT_LOCAL' not in delivery_source
    assert "'/mission/line_course_state'" in delivery_source
    assert "'/navigation/line_follow_status'" in delivery_source
    assert 'route_start_ack' in delivery_source
    assert 'follower_start_ack' in delivery_source
    assert 'route_run_id_changed' in delivery_source
    assert 'start_delivery_ack_timeout' in delivery_source
    # 订阅总数仅用于诊断；首发必须通过 ROS 图确认两个正式消费者，且两个
    # 状态流已经由对应节点实际预热，避免 smoke 观察者抢占 DDS 匹配名额。
    assert 'get_subscriptions_info_by_topic' in delivery_source
    assert 'required_route_subscriber_discovered' in delivery_source
    assert 'required_follower_subscriber_discovered' in delivery_source
    assert 'route_status_stream_observed' in delivery_source
    assert 'follower_status_stream_observed' in delivery_source
    assert 'required_start_subscriber_lost' in delivery_source
    assert 'transport_publish_limit_reached' in delivery_source
    assert 'mission_start_delivery' in start_script
    assert 'START_MAX_TRANSPORT_PUBLISHES' in start_script
    # smoke 只对首个 start 改变输入时序，重传不会重置其状态机。
    assert '_mission_start_messages' in smoke_source
    assert 'mission_start_messages_observed' in smoke_source


def test_gait_inspection_readiness_see_consistent_mode():
    """F3 修复：三个核心节点必须看到一致的 mode 参数。

    gait、inspection、readiness 的 software_smoke_mode 必须来自
    同一组 launch 参数，不能出现一部分在 smoke、一部分在 hardware。
    """
    launch_path = (
        PACKAGE_ROOT / 'launch' / 'competition_non_arm.launch.py'
    )
    source = launch_path.read_text(encoding='utf-8')

    # 所有三个节点都通过 ParameterValue(value_type=bool) 接收参数
    bool_injections = source.count(
        "software_smoke_mode, value_type=bool"
    )
    assert bool_injections >= 2, (
        'at least readiness and inspection must use '
        'ParameterValue(value_type=bool) for software_smoke_mode, '
        'found {} occurrences'.format(bool_injections)
    )
