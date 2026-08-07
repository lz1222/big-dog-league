"""巡线角速度链的离线回归测试。

测试直接调用 follower 的角速度计算，不初始化 ROS 节点，也不会连接任何硬件。
正/负方向沿用既有控制约定：``wz = -(kp_lateral*lateral +
kp_heading*heading)``。因此负 lateral 请求正 yaw，正 lateral 请求负 yaw。
"""

from types import SimpleNamespace

import pytest

from rk_navigation.line_follower_node import LineFollowerNode


def make_follower(lateral_error, heading_error=0.0):
    """构造最小 follower 状态，固定为比赛正式角速度参数。"""
    follower = object.__new__(LineFollowerNode)
    follower.last_line_msg = SimpleNamespace(
        line_visible=True,
        confidence=1.0,
        lateral_error=lateral_error,
        heading_error=heading_error,
    )
    follower.kp_lateral = 0.85
    follower.kp_heading = 0.0
    follower.max_angular_z = 0.28
    follower.angular_deadband = 0.08
    follower.angular_smoothing_alpha = 0.22
    follower.last_angular_z = 0.0
    follower.last_turn_direction = 1
    follower.default_turn_direction = 1
    follower.error_slow_threshold = 0.12
    follower.error_slowest_threshold = 0.28
    follower.base_speed = 0.27
    follower.mid_speed = 0.27
    follower.slow_speed = 0.27
    # 此文件验证已进入 LINE_FOLLOW 后的角速度数学，不把消息有效性另测一遍。
    follower.is_valid_line_msg = lambda message: True
    follower.is_trackable_line = lambda message: True
    return follower


def command_for_frames(follower, frame_count=5):
    """重复相同可信 LineTrack，模拟控制定时器的连续周期。"""
    return [
        follower.command_line_follow(now=None).angular.z
        for _ in range(frame_count)
    ]


def test_sustained_positive_raw_request_eventually_publishes_positive_yaw():
    """从静止持续请求正 yaw 时，滤波不能被死区永久锁死。"""
    outputs = command_for_frames(make_follower(lateral_error=-1.0))

    assert outputs[-1] > 0.0


def test_sustained_negative_raw_request_eventually_publishes_negative_yaw():
    """从静止持续请求负 yaw 时，滤波不能被死区永久锁死。"""
    outputs = command_for_frames(make_follower(lateral_error=1.0))

    assert outputs[-1] < 0.0


def test_true_small_request_inside_deadband_stays_zero():
    """真正小于 deadband 的原始请求仍必须被抑制。"""
    outputs = command_for_frames(make_follower(lateral_error=-0.01))

    assert outputs == [0.0] * 5


def test_large_request_never_exceeds_configured_max_angular_z():
    """大误差即使被夹紧，也不得突破比赛角速度上限。"""
    outputs = command_for_frames(
        make_follower(lateral_error=-10.0), frame_count=30
    )

    assert all(abs(value) <= 0.28 for value in outputs)
    assert outputs[-1] > 0.0


def test_smoothing_starts_at_formal_alpha_times_clamped_target():
    """正式 alpha=0.22 时，限幅正请求首帧应为 0.0616。"""
    first_output = command_for_frames(make_follower(lateral_error=-1.0), 1)[0]

    assert first_output == pytest.approx(0.0616)
    assert 0.0 < first_output < 0.28


def test_sustained_clamped_request_monotonically_converges_not_deadlocks():
    """连续有效大误差应单调逼近上限，不能重复停在零角速度。"""
    outputs = command_for_frames(make_follower(lateral_error=-1.0), 5)

    assert outputs[0] == pytest.approx(0.0616)
    assert all(left < right for left, right in zip(outputs, outputs[1:]))
    assert 0.0 < outputs[-1] < 0.28


def test_wait_start_start_ready_stop_and_publish_zero_reset_previous_angular():
    """任务保护状态和显式零命令必须清除平滑器历史，禁止残余转向。"""
    follower = make_follower(lateral_error=-1.0)
    follower.state = 'LINE_FOLLOW'
    follower.stable_seen_count = 3
    follower.last_loss_reason = 'none'
    follower.last_line_msg_time = None
    follower.elapsed_in_state = lambda now: 0.0
    follower.get_logger = lambda: SimpleNamespace(
        info=lambda message: None,
        warn=lambda message: None,
    )
    follower.start_ready_confirm_frames = 5
    follower.start_ready_min_confidence = 0.55
    follower.start_ready_max_lateral_error = 0.80
    follower.start_ready_max_heading_error = 0.80
    follower.publisher = SimpleNamespace(publish=lambda message: None)
    follower.log_control_debug = lambda command, reason: None
    follower.last_published_cmd_is_zero = False

    follower.last_angular_z = 0.20
    follower.set_state('WAIT_START', 'test_wait_start', object())
    assert follower.last_angular_z == 0.0

    follower.state = 'LINE_FOLLOW'
    follower.last_angular_z = 0.20
    follower.set_state('START_READY', 'test_start_ready', object())
    assert follower.last_angular_z == 0.0

    follower.state = 'LINE_FOLLOW'
    follower.last_angular_z = -0.20
    follower.set_state('STOP', 'test_stop', object())
    assert follower.last_angular_z == 0.0

    follower.last_angular_z = 0.20
    follower.publish_zero('test_publish_zero', force=True)
    assert follower.last_angular_z == 0.0


def test_line_loss_uses_explicit_turn_lost_command_not_filter_residue():
    """丢线保持转向只继承方向，不能直接泄漏滤波器的旧幅值。"""
    follower = make_follower(lateral_error=1.0)
    follower.state = 'LINE_FOLLOW'
    follower.last_angular_z = -0.20
    follower.turn_lost_min_angular_z = 0.08
    follower.lost_turn_linear_speed = 0.0
    follower.lost_turn_angular_speed = 0.25
    follower.line_loss_reason = lambda message: 'line_visible_false'
    follower.set_state = lambda state, reason, now: setattr(
        follower, 'state', state
    )

    command = follower.enter_line_lost_state(
        SimpleNamespace(line_visible=False), now=None
    )

    assert follower.state == 'TURN_LOST_KEEP'
    assert command.angular.z == -0.25
