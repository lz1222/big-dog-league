"""红色任务转向速度合同的回归测试。"""

import math
from types import SimpleNamespace

import pytest

from rk_mission.line_course_mission_node import LineCourseMissionNode


class _RightRedTurnHarness:
    """以最小安全前提执行右转控制分支，避免初始化 ROS 节点。"""

    def __init__(self):
        self.state_enter_time = 0.0
        self.red_yaw_timeout_sec = 8.0
        self.latest_odom_time = 0.0
        self.red_turn_start_yaw = 0.0
        self.latest_odom_yaw = 0.0
        self.red_turn_left_angle_deg = 83.0
        self.red_turn_right_angle_deg = 80.0
        self.red_turn_left_angular_z = 1.0
        # 生产合同：右转参数为非负幅值。
        self.red_turn_right_angular_z = 1.0
        self.published = []

    def _is_fresh(self, _stamp, _now):
        return True

    def _red_motion_safety_reason(self, _now):
        return ''

    def _normalize_angle(self, value):
        return value

    def _publish_mission_candidate(self, command):
        self.published.append(command)

    def _enter_emergency_stop(self, reason):
        raise AssertionError(f'unexpected emergency stop: {reason}')


class _ParameterHarness:
    """复用节点参数校验，不初始化 ROS graph。"""

    def __init__(self, value):
        self.value = value

    def get_parameter(self, _name):
        return SimpleNamespace(value=self.value)


@pytest.mark.parametrize('value', (0.0, 1.0))
def test_right_red_turn_magnitude_parameter_accepts_finite_nonnegative(value):
    """右转幅值允许零或正有限值，启动阶段拒绝反号和非有限值。"""
    harness = _ParameterHarness(value)

    assert LineCourseMissionNode._float_parameter(
        harness, 'red_turn_right_angular_z', positive=False
    ) == value


@pytest.mark.parametrize('value', (-0.1, math.inf, math.nan))
def test_right_red_turn_magnitude_parameter_rejects_invalid_value(value):
    """右转幅值必须是有限非负数，避免非安全的运行时符号修正。"""
    harness = _ParameterHarness(value)

    with pytest.raises(
        ValueError,
        match='red_turn_right_angular_z must be finite and nonnegative',
    ):
        LineCourseMissionNode._float_parameter(
            harness, 'red_turn_right_angular_z', positive=False
        )


def test_right_red_turn_uses_nonnegative_magnitude_and_outputs_negative_wz():
    """右转幅值必须生成 ROS 约定的负 angular.z。"""
    harness = _RightRedTurnHarness()

    LineCourseMissionNode._control_red_turn(harness, 1.0, left=False)

    assert harness.red_turn_right_angular_z >= 0.0
    assert harness.published[-1].angular.z < 0.0
