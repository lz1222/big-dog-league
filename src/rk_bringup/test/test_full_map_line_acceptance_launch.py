# -*- coding: utf-8 -*-
"""静态验证巡线专项 launch 不会把比赛动作节点带入实体验收。"""

import ast
from pathlib import Path


LAUNCH_PATH = (
    Path(__file__).resolve().parents[1]
    / 'launch' / 'full_map_line_acceptance.launch.py'
)
ACCEPTANCE_CONFIG_PATH = (
    Path(__file__).resolve().parents[1]
    / 'config' / 'full_map_line_acceptance_params.yaml'
)


def _node_executables():
    tree = ast.parse(LAUNCH_PATH.read_text(encoding='utf-8'))
    values = []
    for call in ast.walk(tree):
        if not isinstance(call, ast.Call):
            continue
        if not isinstance(call.func, ast.Name) or call.func.id != 'Node':
            continue
        for keyword in call.keywords:
            if (
                keyword.arg == 'executable'
                and isinstance(keyword.value, ast.Constant)
            ):
                values.append(keyword.value.value)
    return values


def test_acceptance_launch_has_only_line_safety_and_velocity_nodes():
    """实体专项入口不得包含 inspection、机械臂、跳跃或任务状态机。"""
    executables = _node_executables()
    assert set(executables) == {
        'realsense2_camera_node',
        'real_line_tracker_node',
        'line_acceptance_guard_node',
        'line_follower_node',
        'line_acceptance_cmd_gate_node',
        'command_mux_node',
    }
    forbidden_markers = (
        'inspection', 'mission', 'arm', 'jump', 'blink', 'stretch', 'hello',
        'gait_control_node', 'white_bar', 'motion_action',
    )
    assert not any(
        marker in executable
        for executable in executables
        for marker in forbidden_markers
    )


def test_acceptance_launch_keeps_final_velocity_owned_by_command_mux():
    """建议速度必须经 ARM 心跳门和 command_mux，不能直写最终 cmd_vel。"""
    source = LAUNCH_PATH.read_text(encoding='utf-8')
    acceptance_config = ACCEPTANCE_CONFIG_PATH.read_text(encoding='utf-8')
    assert "'output_cmd_topic': '/navigation/cmd_vel'" in source
    assert "'output_cmd_topic': '/control/line_cmd'" in source
    assert (
        'suggested_cmd_topic: /line_acceptance/line_cmd_suggested'
        in acceptance_config
    )
    assert 'mission_start_topic: /line_acceptance/start' in acceptance_config
    assert 'mission_stop_topic: /line_acceptance/stop' in acceptance_config
    assert "'line_speed', default_value='0.05'" in source


def test_acceptance_config_uses_exact_node_overrides_for_foxy_precedence():
    """专项参数必须压过基础 YAML 的精确节点段，不能退回比赛入口。"""
    source = ACCEPTANCE_CONFIG_PATH.read_text(encoding='utf-8')
    assert 'real_line_tracker_node:' in source
    assert 'line_follower_node:' in source
    assert 'enable_debug_image: true' in source
    assert 'mission_start_topic: /line_acceptance/start' in source
    assert 'mission_stop_topic: /line_acceptance/stop' in source
    assert 'suggested_cmd_topic: /line_acceptance/line_cmd_suggested' in source
