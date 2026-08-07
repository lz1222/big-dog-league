"""非机械臂比赛启动链的纯 Python 契约与安全判定工具。

这个模块故意不依赖 ROS 运行时，供 launch、readiness 和静态单元测试
共享。它只描述正式比赛链的固定边界，不负责下发任何机器人命令。
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import os
from pathlib import Path
from typing import Any, Dict, Iterable, Mapping, Optional, Tuple


DEFAULT_LINE_IMAGE_TOPIC = '/line_camera/image_raw'
DEFAULT_SIGN_IMAGE_TOPIC = '/go2/front_camera/image_raw'
DEFAULT_SIGN_CAMERA_FRAME_ID = 'go2_front_camera_optical_frame'
DEFAULT_IMAGE_TOPIC = DEFAULT_LINE_IMAGE_TOPIC
FINAL_CMD_TOPIC = '/navigation/cmd_vel'
MOTION_ACTION_NAME = '/locomotion/execute_motion'
SIGN_CAMERA_BRIDGE_NODE = 'go2_front_camera_bridge_node'
# software smoke 只认可由本仓库 C 测试源码编译的 ELF；这不是生产 helper
# 的通用信任机制，而是防止测试参数误指向真实 Unitree SDK 二进制。
TEST_ONLY_SMOKE_HELPER_MARKER = b'RK_NON_ARM_TEST_ONLY_FAKE_SDK_HELPER_V1'
_TEST_ONLY_SMOKE_HELPER_READ_LIMIT = 1024 * 1024

# 正式 launch 只能包含下面这些非硬件业务节点；SDK server/forwarder 是
# hardware_mode 的执行后端，software_smoke_mode 下必须不启动。
REQUIRED_FORMAL_NODES = (
    'line_camera_node',
    'real_line_tracker_node',
    'real_sign_detector_node',
    'line_follower_node',
    'line_course_mission_node',
    'white_bar_stage_command_publisher',
    'white_bar_action_executor',
    'gait_control_node',
    'inspection_action_executor',
    'command_mux_node',
    'go2_sdk_udp_server',
    'cmd_vel_udp_forwarder.py',
)

FORBIDDEN_FORMAL_NODE_MARKERS = (
    '/mock_',
    '/mock-',
    '/national_mission',
    '/arm_',
    '/arm/',
    '/obstacle',
    '/stairs',
    '/standalone_direct',
    '/direct_route',
    '/keyboard_route_node',
    '/cmd_vel_speed_sweep_node',
    '/two_step_walk_test_node',
    '/gait_basic_test_node',
)

ACTIVE_ACTION_STATES = frozenset({
    'ARMED',
    'WAIT_SERVER',
    'WAIT_ZERO',
    'GOAL_SENT',
    'RUNNING',
    'COMMAND_READY',
    'WAIT_SIGN',
    'CANCELING',
    'CLEANUP_PENDING',
})


@dataclass(frozen=True)
class ReadinessCheck:
    """一项只读 readiness 检查的稳定序列化结果。"""

    name: str
    ok: bool
    detail: str
    critical: bool = True

    def as_dict(self) -> Dict[str, Any]:
        """返回可安全编码成 JSON 的检查结果。"""
        return {
            'name': self.name,
            'ok': bool(self.ok),
            'detail': str(self.detail),
            'critical': bool(self.critical),
        }


def bool_from_launch_value(value: Any) -> bool:
    """解析 launch 参数布尔值，未知文本一律按 false 处理。"""
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in {'1', 'true', 'yes', 'on'}


def hardware_processes_allowed(
    hardware_mode: Any,
    software_smoke_mode: Any,
) -> bool:
    """仅在真实硬件模式允许启动相机、SDK server 和 UDP forwarder。

    smoke 标志优先级最高，防止现场误传 hardware_mode=true 时仍触发真实
    SDK 或 UDP 后端。
    """
    return (
        bool_from_launch_value(hardware_mode)
        and not bool_from_launch_value(software_smoke_mode)
    )


def smoke_test_helper_status(path: Any) -> Tuple[bool, str]:
    """验证 smoke helper 是带固定标识、可执行且绝对路径的 ELF。

    读取限制防止 readiness 因异常大文件阻塞；任何软链接、脚本、无标识 ELF
    或不可读文件都按不可信处理。真实动作路径还会重复此约束，因此 launch
    参数即使被误改也不能由 readiness 放行。
    """
    if not isinstance(path, str) or not path.strip():
        return False, 'empty'
    candidate = Path(path.strip())
    if not candidate.is_absolute():
        return False, 'not_absolute'
    try:
        normalized = Path(os.path.realpath(str(candidate)))
    except (OSError, TypeError, ValueError):
        return False, 'path_unresolvable'
    if normalized != candidate:
        return False, 'not_normalized'
    try:
        if not normalized.is_file() or not os.access(str(normalized), os.X_OK):
            return False, 'not_executable_file'
        with normalized.open('rb') as stream:
            header = stream.read(4)
            remainder = stream.read(_TEST_ONLY_SMOKE_HELPER_READ_LIMIT)
    except OSError:
        return False, 'unreadable'
    if header != b'\x7fELF':
        return False, 'not_elf'
    if TEST_ONLY_SMOKE_HELPER_MARKER not in remainder:
        return False, 'test_marker_missing'
    return True, 'marked_test_only_elf'


def front_jump_worst_case_seconds(profile: Mapping[str, Any]) -> float:
    """计算一个 FrontJump profile 的保守软件时长上界。

    这与 supervisor 的阶段顺序一致：预停、最终零速确认、SDK helper、
    动作后稳定。任何缺失、非有限或负值均拒绝，避免错误配置缩短超时。
    """
    keys = (
        'pre_stop_duration',
        'final_zero_timeout',
        'sdk_timeout',
        'post_settle_duration',
    )
    values = []
    for key in keys:
        try:
            value = float(profile[key])
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError('missing or invalid FrontJump {}'.format(key)) \
                from error
        if not math.isfinite(value) or value < 0.0:
            raise ValueError(
                'FrontJump {} must be finite and nonnegative'.format(key)
            )
        values.append(value)
    return sum(values)


def validate_timeout_relationships(
    gait_parameters: Mapping[str, Any],
    executor_parameters: Mapping[str, Any],
    line_course_parameters: Mapping[str, Any],
    *,
    margin_seconds: float = 3.0,
) -> Tuple[float, float, float]:
    """验证白横线执行器与路线节点的超时层级，失败时 fail-closed。"""
    if not math.isfinite(float(margin_seconds)) or margin_seconds < 0.0:
        raise ValueError('margin_seconds must be finite and nonnegative')
    front_jump = gait_parameters.get('front_jump')
    if not isinstance(front_jump, Mapping):
        raise ValueError('gait front_jump parameters are required')
    start_total = front_jump_worst_case_seconds(front_jump.get('start', {}))
    finish_total = front_jump_worst_case_seconds(front_jump.get('finish', {}))
    try:
        executor_timeout = float(executor_parameters['action_timeout_sec'])
        line_course_timeout = float(
            line_course_parameters['white_bar_action_timeout_sec']
        )
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(
            'white-bar timeout parameters are required'
        ) from error
    for name, value in (
        ('executor timeout', executor_timeout),
        ('line-course timeout', line_course_timeout),
    ):
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError('{} must be finite and positive'.format(name))
    worst_total = max(start_total, finish_total)
    if executor_timeout <= worst_total + margin_seconds:
        raise ValueError(
            'white-bar executor timeout {:.3f}s is not greater than '
            'FrontJump worst case {:.3f}s plus {:.3f}s margin'.format(
                executor_timeout, worst_total, margin_seconds
            )
        )
    if line_course_timeout <= executor_timeout + margin_seconds:
        raise ValueError(
            'line-course timeout {:.3f}s is not greater than executor '
            'timeout {:.3f}s plus {:.3f}s margin'.format(
                line_course_timeout, executor_timeout, margin_seconds
            )
        )
    return start_total, finish_total, executor_timeout


def endpoint_is_command_mux(endpoint: Any) -> bool:
    """判断 ROS 图端点是否是唯一允许的最终速度发布者。"""
    node_name = getattr(endpoint, 'node_name', None)
    namespace = getattr(endpoint, 'node_namespace', '')
    return node_name == 'command_mux_node' and namespace in ('', '/')


def is_zero_twist(values: Iterable[Any], epsilon: float = 0.001) -> bool:
    """检查六维 Twist 值均有限且足够接近零。"""
    try:
        numeric_values = [float(value) for value in values]
        tolerance = float(epsilon)
    except (TypeError, ValueError):
        return False
    if len(numeric_values) != 6 or not math.isfinite(tolerance):
        return False
    return all(
        math.isfinite(value) and abs(value) <= abs(tolerance)
        for value in numeric_values
    )


def json_object(raw: Any) -> Optional[Dict[str, Any]]:
    """安全解析状态 topic 的 JSON object，错误输入不抛给控制循环。"""
    import json

    if not isinstance(raw, str):
        return None
    try:
        decoded = json.loads(raw)
    except (TypeError, ValueError, json.JSONDecodeError):
        return None
    return decoded if isinstance(decoded, dict) else None


def status_is_terminal_or_idle(
    status: Optional[Mapping[str, Any]],
) -> bool:
    """只有明确的 idle/terminal 状态才允许 readiness 放行。"""
    if not isinstance(status, Mapping):
        return False
    raw_state = status.get('state', status.get('status', ''))
    state = str(raw_state).strip().upper()
    return state in {
        'IDLE',
        'SUCCEEDED',
        'FAILED',
        'TIMEOUT',
        'CANCELED',
        'ABORTED',
        'REJECTED',
    }


def route_is_wait_start(state: Optional[Mapping[str, Any]]) -> bool:
    """确认任务尚未启动，避免 readiness 对进行中任务错误放行。"""
    if not isinstance(state, Mapping):
        return False
    return (
        str(state.get('route_phase', '')).strip() == 'WAIT_START'
        and not bool(state.get('mission_started', False))
        and not str(state.get('active_action', '')).strip()
    )
