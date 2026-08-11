"""TRANSFER 最终路线的纯软件闭环测试，不初始化 ROS/硬件。"""

from dataclasses import replace
import math

import pytest

from rk_mission.line_course_mission_node import corner_cooldown_elapsed
from rk_mission.transfer_route_core import (
    ARC_APPROACH,
    ARC_TO_EXIT,
    ARC_TO_TASK,
    COMPLETE,
    LINE_REACQUIRE,
    PICK_TASK,
    PLACE_TASK,
    PRE_CORNER_FORWARD,
    PRE_CORNER_REACQUIRE,
    PRE_CORNER_STOP,
    PRE_CORNER_TURN,
    RIGHT_CORNER_SEARCH,
    SAFE_STOP,
    TASK_STOP,
    TransferInputs,
    TransferRouteConfig,
    TransferRouteCore,
)


def calibrated_config(**overrides):
    """单测专用已标定数值；正式 YAML 仍保持 false/zero。"""
    config = TransferRouteConfig(
        right_corner_confirm_frames=3,
        right_corner_motion_calibrated=True,
        right_corner_forward_speed=0.12,
        right_corner_forward_active_time_sec=0.15,
        right_corner_turn_wz=0.30,
        right_corner_target_yaw_deg=90.0,
        transfer_arc_entry_calibrated=True,
        transfer_arc_entry_confirm_frames=2,
        transfer_arc_motion_calibrated=True,
        transfer_arc_vx=0.10,
        transfer_arc_wz=0.25,
        transfer_arc_task_stop_yaw_deg=60.0,
        transfer_arc_exit_delta_yaw_deg=45.0,
        reacquire_frames=2,
        zero_confirm_frames=2,
    )
    return replace(config, **overrides)


def inputs(final=(0.0, 0.0, 0.0), yaw=0.0, *, safe=True,
           final_fresh=True, odom_fresh=True):
    """构造 mux/gait/odom 真相快照。"""
    return TransferInputs(
        final_cmd=final,
        final_cmd_fresh=final_fresh,
        mux_healthy=safe,
        gait_available=safe,
        odom_yaw=yaw,
        odom_fresh=odom_fresh,
    )


def confirm_right_corner(core, now=0.0):
    for index in range(3):
        core.observe_corner(True, 0.8, 'right', now + index * 0.01)
    assert core.state == PRE_CORNER_FORWARD


def reach_right_reacquire(core):
    """推进首个右角至重新找线。"""
    confirm_right_corner(core)
    first = core.tick(0.10, inputs(final=(0.12, 0.0, 0.0)))
    second = core.tick(0.20, inputs(final=(0.12, 0.0, 0.0)))
    third = core.tick(0.30, inputs(final=(0.12, 0.0, 0.0)))
    assert first.vx > 0.0 and first.wz == 0.0
    assert second.vx > 0.0 and second.wz == 0.0
    assert third.state == PRE_CORNER_STOP and third.vx == third.wz == 0.0
    core.tick(0.31, inputs(yaw=0.0))
    zero = core.tick(0.32, inputs(yaw=0.0))
    assert zero.state == PRE_CORNER_TURN
    turn = core.tick(0.33, inputs(yaw=0.0))
    assert turn.vx == 0.0 and turn.wz < 0.0
    stopped = core.tick(
        0.40, inputs(final=(0.0, 0.0, -0.30), yaw=-math.pi / 2.0)
    )
    assert stopped.state == PRE_CORNER_REACQUIRE
    assert stopped.vx == stopped.wz == 0.0


def test_corner_cooldown_blocks_same_corner_until_duration_elapsed():
    """ALIGN_TO_LINE 完成后的同一弯角在 cooldown 内不得重触发。"""
    assert not corner_cooldown_elapsed(11.9, 10.0, 3.0)
    assert corner_cooldown_elapsed(13.0, 10.0, 3.0)


def test_right_corner_three_frames_forward_active_time_and_yaw_closure():
    """验证 3 帧、实际前进时间、ZERO 和负 yaw 90° 闭环。"""
    core = TransferRouteCore(calibrated_config())
    confirm_right_corner(core)
    assert core.transfer_right_corner_detected

    # 安全门停车时即使 final 数值相同也不计时。
    core.tick(0.10, inputs(final=(0.12, 0.0, 0.0), safe=False))
    assert core.active_elapsed == 0.0
    core.observe_line(False, 0.0, 0.0, 0.11)
    assert core.state == PRE_CORNER_FORWARD
    core.tick(0.20, inputs(final=(0.12, 0.0, 0.0)))
    assert core.active_elapsed == pytest.approx(0.10)
    decision = core.tick(0.30, inputs(final=(0.12, 0.0, 0.0)))
    assert decision.state == PRE_CORNER_STOP
    core.tick(0.31, inputs(yaw=0.0))
    core.tick(0.32, inputs(yaw=0.0))
    assert core.state == PRE_CORNER_TURN
    turn = core.tick(0.33, inputs(yaw=0.0))
    assert turn.vx == 0.0 and turn.wz < 0.0
    core.observe_line(False, 0.0, 0.0, 0.34)
    assert core.state == PRE_CORNER_TURN
    decision = core.tick(
        0.40, inputs(final=(0.0, 0.0, -0.30), yaw=-math.pi / 2.0)
    )
    assert decision.state == PRE_CORNER_REACQUIRE
    assert core.transfer_right_corner_completed


def test_full_transfer_flow_keeps_both_blind_arcs_left_and_reacquires():
    """整链路确认短直线 follower、place/pick 顺序与两段 +wz。"""
    core = TransferRouteCore(calibrated_config())
    reach_right_reacquire(core)
    core.observe_line(True, 0.8, 0.0, 0.41)
    core.observe_line(True, 0.8, 0.0, 0.42)
    follow = core.tick(0.43, inputs(yaw=-math.pi / 2.0))
    assert core.state == ARC_APPROACH
    assert follow.control == 'LINE_FOLLOW'

    # 首个 RIGHT 二次出现不能当作 blind LEFT 入口。
    core.observe_corner(True, 0.9, 'right', 0.44)
    core.observe_corner(True, 0.9, 'right', 0.45)
    assert core.state == ARC_APPROACH and core.arc_entry_count == 0
    core.observe_corner(True, 0.9, 'left', 0.46)
    core.observe_corner(True, 0.9, 'left', 0.47)
    start = core.tick(0.48, inputs(yaw=-math.pi / 2.0))
    assert start.state == ARC_TO_TASK and start.vx == start.wz == 0.0
    arc = core.tick(0.49, inputs(yaw=-math.pi / 2.0))
    assert arc.vx > 0.0 and arc.wz > 0.0
    core.observe_line(False, 0.0, 0.0, 0.50)
    assert core.state == ARC_TO_TASK
    stop = core.tick(
        0.60,
        inputs(
            final=(0.10, 0.0, 0.25),
            yaw=-math.pi / 2.0 + math.radians(60),
        ),
    )
    assert stop.state == TASK_STOP and stop.vx == stop.wz == 0.0

    core.tick(0.61, inputs(yaw=-math.pi / 2.0 + math.radians(60)))
    core.tick(0.62, inputs(yaw=-math.pi / 2.0 + math.radians(60)))
    assert core.state == PLACE_TASK and core.transfer_task_stop_reached
    place = core.tick(0.63, inputs())
    assert place.action == 'transfer_place'
    assert core.mark_action_dispatched(place.action, 0.63)
    assert core.action_result('transfer_place', True, 0.64)
    assert core.state == PICK_TASK and core.transfer_place_completed
    pick = core.tick(0.65, inputs())
    assert pick.action == 'transfer_pick'
    assert core.mark_action_dispatched(pick.action, 0.65)
    assert core.action_result('transfer_pick', True, 0.66)
    assert core.transfer_pick_completed

    exit_yaw = -math.pi / 2.0 + math.radians(60)
    core.tick(0.67, inputs(yaw=exit_yaw))
    core.tick(0.68, inputs(yaw=exit_yaw))
    assert core.state == ARC_TO_EXIT
    arc_out = core.tick(0.69, inputs(yaw=exit_yaw))
    assert arc_out.vx > 0.0 and arc_out.wz > 0.0
    stop_out = core.tick(
        0.75,
        inputs(
            final=(0.10, 0.0, 0.25),
            yaw=exit_yaw + math.radians(45),
        ),
    )
    assert stop_out.state == LINE_REACQUIRE
    assert stop_out.vx == stop_out.wz == 0.0
    core.observe_line(True, 0.8, 0.0, 0.76)
    core.observe_line(True, 0.8, 0.0, 0.77)
    assert core.state == COMPLETE and core.transfer_arc_completed


def test_uncalibrated_right_corner_and_blind_arc_fail_closed():
    """两类非零固定动作都有独立实体标定门。"""
    right = TransferRouteCore()
    for index in range(3):
        right.observe_corner(True, 0.8, 'right', index * 0.01)
    assert right.state == SAFE_STOP
    assert right.fault_reason == 'RIGHT_CORNER_NOT_CALIBRATED'

    arc = TransferRouteCore(replace(
        calibrated_config(), transfer_arc_entry_calibrated=False
    ))
    reach_right_reacquire(arc)
    arc.observe_line(True, 0.8, 0.0, 0.41)
    arc.observe_line(True, 0.8, 0.0, 0.42)
    arc.observe_corner(True, 0.8, 'left', 0.43)
    arc.observe_corner(True, 0.8, 'left', 0.44)
    assert arc.state == SAFE_STOP
    assert arc.fault_reason == 'TRANSFER_ARC_ENTRY_NOT_CALIBRATED'


@pytest.mark.parametrize('phase', ('right_turn', 'arc_task', 'arc_exit'))
def test_stale_odom_during_fixed_yaw_motion_enters_safe_stop(phase):
    """右转和两段 blind arc 的 odom stale 均不得开环继续。"""
    core = TransferRouteCore(calibrated_config())
    if phase == 'right_turn':
        confirm_right_corner(core)
        core.tick(0.10, inputs(final=(0.12, 0.0, 0.0)))
        core.tick(0.30, inputs(final=(0.12, 0.0, 0.0)))
        core.tick(0.31, inputs(yaw=0.0))
        core.tick(0.32, inputs(yaw=0.0))
    else:
        reach_right_reacquire(core)
        core.observe_line(True, 0.8, 0.0, 0.41)
        core.observe_line(True, 0.8, 0.0, 0.42)
        core.observe_corner(True, 0.8, 'left', 0.43)
        core.observe_corner(True, 0.8, 'left', 0.44)
        core.tick(0.45, inputs(yaw=0.0))
        if phase == 'arc_exit':
            core.tick(
                0.50,
                inputs(final=(0.10, 0.0, 0.25), yaw=math.radians(60)),
            )
            core.tick(0.51, inputs(yaw=math.radians(60)))
            core.tick(0.52, inputs(yaw=math.radians(60)))
            core.tick(0.53, inputs())
            core.mark_action_dispatched('transfer_place', 0.53)
            core.action_result('transfer_place', True, 0.54)
            core.tick(0.55, inputs())
            core.mark_action_dispatched('transfer_pick', 0.55)
            core.action_result('transfer_pick', True, 0.56)
            core.tick(0.57, inputs(yaw=math.radians(60)))
            core.tick(0.58, inputs(yaw=math.radians(60)))
    core.tick(0.80, inputs(odom_fresh=False))
    assert core.state == SAFE_STOP


@pytest.mark.parametrize(
    ('expected_state', 'wrong_yaw'),
    ((PRE_CORNER_TURN, math.radians(5)),
     (ARC_TO_TASK, -math.radians(5)),
     (ARC_TO_EXIT, math.radians(55))),
)
def test_yaw_wrong_direction_enters_safe_stop(expected_state, wrong_yaw):
    """任一固定 yaw 朝反方向超过容差都必须 SAFE_STOP。"""
    core = TransferRouteCore(calibrated_config())
    if expected_state == PRE_CORNER_TURN:
        confirm_right_corner(core)
        core.tick(0.10, inputs(final=(0.12, 0.0, 0.0)))
        core.tick(0.30, inputs(final=(0.12, 0.0, 0.0)))
        core.tick(0.31, inputs(yaw=0.0))
        core.tick(0.32, inputs(yaw=0.0))
    else:
        reach_right_reacquire(core)
        core.observe_line(True, 0.8, 0.0, 0.41)
        core.observe_line(True, 0.8, 0.0, 0.42)
        core.observe_corner(True, 0.8, 'left', 0.43)
        core.observe_corner(True, 0.8, 'left', 0.44)
        core.tick(0.45, inputs(yaw=0.0))
        if expected_state == ARC_TO_EXIT:
            core.tick(
                0.50,
                inputs(final=(0.10, 0.0, 0.25), yaw=math.radians(60)),
            )
            core.tick(0.51, inputs(yaw=math.radians(60)))
            core.tick(0.52, inputs(yaw=math.radians(60)))
            core.tick(0.53, inputs())
            core.mark_action_dispatched('transfer_place', 0.53)
            core.action_result('transfer_place', True, 0.54)
            core.tick(0.55, inputs())
            core.mark_action_dispatched('transfer_pick', 0.55)
            core.action_result('transfer_pick', True, 0.56)
            core.tick(0.57, inputs(yaw=math.radians(60)))
            core.tick(0.58, inputs(yaw=math.radians(60)))
    assert core.state == expected_state
    core.tick(0.80, inputs(yaw=wrong_yaw))
    assert core.state == SAFE_STOP
    assert core.fault_reason == 'TRANSFER_YAW_WRONG_DIRECTION'


def test_nonfinite_final_command_fails_closed():
    """NaN/Inf 最终命令不得进入有效时间或运动判定。"""
    core = TransferRouteCore(calibrated_config())
    confirm_right_corner(core)

    decision = core.tick(
        0.1, inputs(final=(float('nan'), 0.0, 0.0))
    )

    assert decision.state == SAFE_STOP
    assert decision.vx == decision.wz == 0.0


def test_yaw_progress_without_verified_mission_final_command_fails_closed():
    """odom 在 mux 未证明 mission 命令时变化，不得偷渡转角闭环。"""
    core = TransferRouteCore(calibrated_config())
    confirm_right_corner(core)
    core.tick(0.10, inputs(final=(0.12, 0.0, 0.0)))
    core.tick(0.30, inputs(final=(0.12, 0.0, 0.0)))
    core.tick(0.31, inputs(yaw=0.0))
    core.tick(0.32, inputs(yaw=0.0))

    core.tick(0.40, inputs(final=(0.0, 0.0, 0.0), yaw=-0.10))

    assert core.state == SAFE_STOP
    assert core.fault_reason == 'TRANSFER_YAW_PROGRESS_UNVERIFIED'


def test_duplicate_milestones_and_out_of_order_action_results_are_ignored():
    """重复右角、入口和旧 Action 结果不得使状态倒退。"""
    core = TransferRouteCore(calibrated_config())
    confirm_right_corner(core)
    detected_state = core.state
    for index in range(3):
        core.observe_corner(True, 0.9, 'right', 0.1 + index * 0.01)
    assert core.state == detected_state
    assert not core.action_result('transfer_place', True, 0.2)
    assert core.state == detected_state
    assert core.transfer_right_corner_detected
    assert not core.transfer_arc_started
    assert core.state != RIGHT_CORNER_SEARCH
