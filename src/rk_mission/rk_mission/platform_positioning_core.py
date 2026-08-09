"""三类任务平台的纯软件定位状态机。

本模块不导入 ROS，也不直接调用底盘或机械臂。它只根据已被路线层明确
选择的 ``route_phase`` 生成意图和可观测快照；未标定、输入不完整或顺序
错误时始终返回零速度，避免影响基础巡线验收。
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Dict, Optional


ZERO = 'ZERO'
RELEASE = 'RELEASE_LINE_FOLLOWER'
ZERO_CONFIRM_STATES = frozenset((
    'TRANSFER_STOP_CONFIRM',
    'PICKUP_STOP_CONFIRM',
    'PICKUP_REVERSE_ZERO_CONFIRM',
    'PICKUP_FORWARD_ZERO_CONFIRM',
    'PICKUP_SIDE_POSITION_READY',
    'PLACE_STOP_CONFIRM',
))
GAIT_LOCK_STATES = frozenset((
    'TRANSFER_TASK',
    'PICKUP_OBJECT_RECOGNITION', 'PICKUP_ARM_HANDOFF_READY',
    'PLACE_POSITION_LOCKED', 'PLACE_DONE',
))


@dataclass(frozen=True)
class PlatformCommand:
    """任务定位的候选速度；适配层仍须经最终 mux 发布。"""

    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0


@dataclass(frozen=True)
class BoardObservation:
    """前置挡板检测结果，所有比例均相对原始图像尺寸。"""

    detected: bool = False
    confidence: float = 0.0
    center_x_ratio: float = 0.0
    center_y_ratio: float = 0.0
    bottom_y_ratio: float = 0.0
    width_ratio: float = 0.0
    height_ratio: float = 0.0
    area_ratio: float = 0.0


@dataclass(frozen=True)
class WhiteBarObservation:
    """复用现有白横线检测器输出，白线只参与纵向定位。"""

    detected: bool = False
    confidence: float = 0.0
    center_y_ratio: float = 0.0
    span_ratio: float = 0.0
    height_ratio: float = 0.0


def _finite(value: object) -> Optional[float]:
    """拒绝 NaN/Inf，防止异常视觉数值穿透到移动状态。"""
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _angle_delta(target: float, current: float) -> float:
    """返回 [-pi, pi] 的最短 yaw 误差，保证跨 pi 边界仍可闭环。"""
    return math.atan2(math.sin(target - current), math.cos(target - current))


class TaskPlatformPositioningCore:
    """任务平台定位编排器，所有实体动作默认被显式禁用。

    ``allow_motion_execution`` 仅是将来经独立安全评审后使用的接口；本轮
    默认 ``False``，故使状态可完整演练也只会输出零速度。
    """

    def __init__(self, parameters: Optional[dict] = None):
        self.p = dict(DEFAULT_PLATFORM_PARAMETERS)
        if parameters:
            self.p.update(parameters)
        self.route_phase = 'WAIT_START'
        self.state = 'IDLE'
        self.reason = 'platform_positioning_idle'
        self.anchor_detected = False
        self.stop_confirmed = False
        self._counts: Dict[str, int] = {}
        self._yaw_target: Optional[float] = None
        self._yaw_start: Optional[float] = None
        self._yaw_current: Optional[float] = None
        self._turn_started_at: Optional[float] = None
        self._active_elapsed = 0.0
        self._last_tick: Optional[float] = None
        self._final_command = PlatformCommand()
        self._recognition_result: Optional[bool] = None
        self._place_done = False
        self._finish_rearmed = False
        self.failure_reason = ''
        self._command = PlatformCommand()
        self._last_transfer = {}
        self._last_board = BoardObservation()
        self._last_white = WhiteBarObservation()
        self._last_place_line = {}
        self._validate()

    def _validate(self) -> None:
        """启动时校验关键边界，错误参数保持在软件层而不猜测数值。"""
        for name in (
            'transfer_anchor_confirm_frames',
            'pickup_board_confirm_frames',
            'pickup_position_confirm_frames',
            'place_white_bar_confirm_frames',
            'transfer_zero_confirm_samples',
            'pickup_zero_confirm_samples',
            'place_zero_confirm_samples',
        ):
            if int(self.p[name]) < 1:
                raise ValueError('{} must be >= 1'.format(name))
        for name in (
            'transfer_zero_epsilon',
            'pickup_zero_epsilon',
            'place_zero_epsilon',
            'active_motion_direction_epsilon',
        ):
            if float(self.p[name]) < 0.0:
                raise ValueError('{} must be nonnegative'.format(name))
        if float(self.p['pickup_yaw_timeout_sec']) <= 0.0:
            raise ValueError('pickup_yaw_timeout_sec must be positive')

    def set_route_phase(self, route_phase: str) -> None:
        """仅接收路线层显式阶段；普通角点/挡板/白线均不能改变路线。"""
        normalized = str(route_phase)
        if normalized == self.route_phase:
            return
        self.route_phase = normalized
        if self.route_phase == 'TRANSFER_PLATFORM_APPROACH':
            self._enter_transfer()
        elif self.route_phase == 'PICKUP_PLATFORM_APPROACH':
            self._enter_pickup()
        elif self.route_phase == 'PLACE_PLATFORM_APPROACH':
            self._enter_place()
        elif self.route_phase not in ('TRANSFER_PLATFORM_APPROACH',
                                      'PICKUP_PLATFORM_APPROACH',
                                      'PLACE_PLATFORM_APPROACH'):
            self._reset_idle('route_phase_not_platform')

    def observe_transfer_anchor(
            self,
            *,
            detected: bool,
            confidence: float,
            heading_error: float,
            lateral_error: float) -> None:
        """用既有 corner/line geometry 建立中转锚点，且受路线双重门控。"""
        if self.state != 'TRANSFER_ANCHOR_WAIT':
            return
        self._last_transfer = {
            'confidence': confidence,
            'heading_error': heading_error,
            'lateral_error': lateral_error,
        }
        valid = all(_finite(value) is not None for value in (
            confidence, heading_error, lateral_error,
        )) and bool(detected) and float(confidence) >= float(
            self.p['transfer_anchor_min_confidence']
        ) and abs(float(heading_error) - float(
            self.p['transfer_anchor_target_heading_error']
        )) <= float(self.p['transfer_anchor_heading_tolerance']) and abs(
            float(lateral_error) - float(
                self.p['transfer_anchor_target_lateral_error']
            )
        ) <= float(self.p['transfer_anchor_lateral_tolerance'])
        self._counts['transfer'] = self._counts.get(
            'transfer', 0) + 1 if valid else 0
        self.anchor_detected = self._counts['transfer'] >= int(
            self.p['transfer_anchor_confirm_frames']
        )
        if self.anchor_detected:
            self._active_elapsed = 0.0
            self.state = 'TRANSFER_FINE_OFFSET'
            self.reason = 'transfer_corner_anchor_confirmed'

    def observe_pickup_board(self, observation: BoardObservation) -> None:
        """挡板仅在抓取平台接近阶段参与停车画面确认。"""
        if self.state != 'PICKUP_VISUAL_APPROACH':
            return
        self._last_board = observation
        valid = observation.detected and self._pickup_target_matches(
            observation)
        self._counts['pickup_board'] = self._counts.get(
            'pickup_board', 0) + 1 if valid else 0
        self.anchor_detected = self._counts['pickup_board'] >= int(
            self.p['pickup_position_confirm_frames']
        )
        if self.anchor_detected:
            self.state = 'PICKUP_VISUAL_POSITION_LOCKED'
            self.reason = 'pickup_board_golden_signature_confirmed'
            self._reset_zero_confirmation()

    def observe_place_white_bar(self, observation: WhiteBarObservation,
                                *, line_lateral_error: float,
                                line_heading_error: float) -> None:
        """白线决定纵向，LineTrack 同时通过才允许放置位置锁定。"""
        if self.state != 'PLACE_WHITE_BAR_APPROACH':
            return
        self._last_white = observation
        self._last_place_line = {
            'lateral_error': line_lateral_error,
            'heading_error': line_heading_error,
        }
        values = (
            line_lateral_error,
            line_heading_error,
            observation.center_y_ratio,
            observation.span_ratio)
        valid = (
            observation.detected
            and all(_finite(value) is not None for value in values)
            and abs(float(observation.center_y_ratio) - float(
                self.p['place_white_bar_target_y_ratio'])) <= float(
                    self.p['place_white_bar_y_tolerance'])
            and abs(float(observation.span_ratio) - float(
                self.p['place_white_bar_target_span_ratio'])) <= float(
                    self.p['place_white_bar_span_tolerance'])
            and abs(float(line_lateral_error)) <= float(
                self.p['place_line_max_lateral_error'])
            and abs(float(line_heading_error)) <= float(
                self.p['place_line_max_heading_error'])
        )
        self._counts['place'] = self._counts.get(
            'place', 0) + 1 if valid else 0
        self.anchor_detected = self._counts['place'] >= int(
            self.p['place_white_bar_confirm_frames']
        )
        if self.anchor_detected:
            self.state = (
                'PLACE_FINAL_OFFSET'
                if bool(self.p['place_final_offset_enabled'])
                else 'PLACE_STOP_CONFIRM'
            )
            self.reason = 'place_white_bar_and_line_confirmed'
            self._active_elapsed = 0.0
            self._reset_zero_confirmation()

    def observe_final_command(self, command: PlatformCommand) -> None:
        """缓存 mux 最终命令，并仅在停车阶段累计连续零速样本。"""
        self._final_command = command
        values = (command.vx, command.vy, command.wz)
        is_zero = all(
            _finite(value) is not None and abs(
                float(value)) <= float(
                self._zero_epsilon()) for value in values)
        if self.state in ZERO_CONFIRM_STATES:
            self._counts['zero'] = self._counts.get(
                'zero', 0) + 1 if is_zero else 0
        else:
            self._counts['zero'] = 0
        required = self._zero_samples_required()
        self.stop_confirmed = self._counts['zero'] >= required

    def observe_odom_yaw(self, yaw: float) -> None:
        """保存 yaw 闭环输入；从不以时间完成 90° 转向。"""
        if _finite(yaw) is not None:
            self._yaw_current = float(yaw)

    def recognition_result(self, success: bool) -> None:
        """接收既有物资识别模块的最终结果，不重写其算法。"""
        if self.state == 'PICKUP_OBJECT_RECOGNITION':
            self._recognition_result = bool(success)

    def place_done(self) -> None:
        """记录放置完成；FINISH 仍需独立显式 re-arm。"""
        if self.state == 'PLACE_POSITION_LOCKED':
            self._place_done = True
            self.state = 'PLACE_DONE'
            self.reason = 'place_done_waiting_finish_rearm'

    def rearm_finish(self) -> None:
        """只有 PLACE_DONE 后的显式路线事件才能恢复 FINISH 白线语义。"""
        if self.state == 'PLACE_DONE' and self._place_done:
            self._finish_rearmed = True
            self.reason = 'finish_white_bar_explicitly_rearmed'

    def transfer_task_done(self) -> None:
        """由既有中转任务回执驱动退出；未锁定前不能伪造任务完成。"""
        if self.state == 'TRANSFER_TASK':
            self.state = 'TRANSFER_EXIT'
            self.reason = 'transfer_task_completed'

    def gait_lock_requested(self) -> bool:
        """任务/停车阶段请求既有 arbiter 锁，接近阶段仍由巡线控制。"""
        return self.state in GAIT_LOCK_STATES

    def tick(self, now: float) -> PlatformCommand:
        """推进纯状态机；默认零输出保证本轮没有任何真机平移运动。"""
        now = float(now)
        dt = 0.0 if self._last_tick is None else max(
            0.0, now - self._last_tick)
        self._last_tick = now
        self._command = PlatformCommand()
        if self.state == 'TRANSFER_APPROACH':
            self.state = 'TRANSFER_ANCHOR_WAIT'
            self.reason = 'transfer_line_follow_released_for_anchor'
        elif self.state == 'PICKUP_PLATFORM_APPROACH':
            self.state = 'PICKUP_VISUAL_APPROACH'
            self.reason = 'pickup_line_follow_released_for_board'
        elif self.state == 'PLACE_PLATFORM_APPROACH':
            self.state = 'PLACE_WHITE_BAR_APPROACH'
            self.reason = 'place_line_follow_released_for_white_bar'
        elif self.state == 'TRANSFER_FINE_OFFSET':
            self._run_timed_motion(
                'transfer_offset', dt, 'TRANSFER_STOP_CONFIRM')
        elif self.state == 'TRANSFER_STOP_CONFIRM' and self.stop_confirmed:
            self.state = 'TRANSFER_TASK'
            self.reason = 'transfer_stop_confirmed'
        elif self.state == 'PICKUP_VISUAL_POSITION_LOCKED':
            self.state = 'PICKUP_STOP_CONFIRM'
            self.reason = 'pickup_position_locked_waiting_zero'
        elif self.state == 'PICKUP_STOP_CONFIRM' and self.stop_confirmed:
            self.state = 'PICKUP_STOP_CONFIRMED'
            self.reason = 'pickup_final_cmd_zero_confirmed'
        elif self.state == 'PICKUP_STOP_CONFIRMED':
            self._start_turn(left=True, now=now)
        elif self.state in ('PICKUP_TURN_LEFT', 'PICKUP_TURN_RIGHT'):
            self._run_yaw_turn(now)
        elif self.state == 'PICKUP_VIEW_REVERSE':
            self._run_timed_motion(
                'pickup_view_reverse',
                dt,
                'PICKUP_REVERSE_ZERO_CONFIRM')
        elif (self.state == 'PICKUP_REVERSE_ZERO_CONFIRM'
              and self.stop_confirmed):
            self.state = 'PICKUP_OBJECT_RECOGNITION'
            self.reason = 'pickup_reverse_zero_confirmed'
        elif self.state == 'PICKUP_OBJECT_RECOGNITION':
            if self._recognition_result is True:
                self.state = 'PICKUP_SIDE_FORWARD'
                self.reason = 'pickup_recognition_handoff_success'
                self._active_elapsed = 0.0
            elif self._recognition_result is False:
                self._fail('PICKUP_RECOGNITION_FAILED')
        elif self.state == 'PICKUP_SIDE_FORWARD':
            self._run_timed_motion(
                'pickup_side_forward',
                dt,
                'PICKUP_FORWARD_ZERO_CONFIRM')
        elif (self.state == 'PICKUP_FORWARD_ZERO_CONFIRM'
              and self.stop_confirmed):
            self._start_turn(left=False, now=now)
        elif (self.state == 'PICKUP_SIDE_POSITION_READY'
              and self.stop_confirmed):
            self.state = 'PICKUP_ARM_HANDOFF_READY'
            self.reason = 'pickup_right_turn_stop_confirmed'
        elif self.state == 'PLACE_STOP_CONFIRM' and self.stop_confirmed:
            self.state = 'PLACE_POSITION_LOCKED'
            self.reason = 'place_stop_confirmed'
        elif self.state == 'PLACE_FINAL_OFFSET':
            self._run_timed_motion(
                'place_final_offset', dt, 'PLACE_STOP_CONFIRM')
        return self._command

    def snapshot(self) -> dict:
        """提供现场标定所需状态、计时、yaw 和隔离语义的完整快照。"""
        yaw_delta = None
        if self._yaw_target is not None and self._yaw_current is not None:
            yaw_delta = _angle_delta(self._yaw_target, self._yaw_current)
        return {
            'platform': self._platform(),
            'route_phase': self.route_phase,
            'state': self.state,
            'reason': self.reason,
            'calibrated': self._platform_calibrated(),
            'signature_valid': self._signature_valid(),
            'anchor_detected': self.anchor_detected,
            'confirm_count': self._confirm_count(),
            'stop_confirmed': self.stop_confirmed,
            'active_motion_elapsed_sec': self._active_elapsed,
            'active_motion_remaining_sec': self._remaining_duration(),
            'yaw_start_rad': self._yaw_start,
            'yaw_current_rad': self._yaw_current,
            'yaw_target_rad': self._yaw_target,
            'yaw_delta_rad': yaw_delta,
            'zero_confirm_count': self._counts.get('zero', 0),
            'failure_reason': self.failure_reason,
            'gait_lock_requested': self.gait_lock_requested(),
            'finish_white_bar_armed': (
                self._place_done and self._finish_rearmed),
            'target_values': self._target_values(),
            'current_values': self._current_values(),
            'errors': self._position_errors(),
            'command': self._command.__dict__,
        }

    def _enter_transfer(self) -> None:
        self._reset_runtime()
        if (not bool(self.p['transfer_platform_calibrated'])
                or not bool(self.p['transfer_anchor_signature_valid'])):
            self._fail('TRANSFER_PLATFORM_NOT_CALIBRATED')
        else:
            self.state = 'TRANSFER_APPROACH'
            self.reason = 'transfer_route_phase_armed'

    def _enter_pickup(self) -> None:
        self._reset_runtime()
        if not bool(self.p['pickup_platform_calibrated']):
            self._fail('PICKUP_PLATFORM_NOT_CALIBRATED')
        elif not self._pickup_target_configured():
            self._fail('PICKUP_BOARD_NOT_CALIBRATED')
        else:
            self.state = 'PICKUP_PLATFORM_APPROACH'
            self.reason = 'pickup_route_phase_armed'

    def _enter_place(self) -> None:
        self._reset_runtime()
        if not bool(self.p['place_platform_calibrated']):
            self._fail('PLACE_PLATFORM_NOT_CALIBRATED')
        elif not self._place_target_configured():
            self._fail('PLACE_WHITE_BAR_NOT_CALIBRATED')
        else:
            self.state = 'PLACE_PLATFORM_APPROACH'
            self.reason = 'place_route_phase_armed'

    def _reset_idle(self, reason: str) -> None:
        self._reset_runtime()
        self.state, self.reason = 'IDLE', reason

    def _reset_runtime(self) -> None:
        self.anchor_detected = self.stop_confirmed = False
        self._counts.clear()
        self._active_elapsed = 0.0
        self._last_tick = None
        self._final_command = PlatformCommand()
        self._yaw_target = None
        self._yaw_start = None
        self._turn_started_at = None
        self._recognition_result = None
        self._place_done = False
        self._finish_rearmed = False
        self.failure_reason = ''
        self._command = PlatformCommand()
        self._last_transfer = {}
        self._last_board = BoardObservation()
        self._last_white = WhiteBarObservation()
        self._last_place_line = {}

    def _pickup_target_matches(self, o: BoardObservation) -> bool:
        pairs = (
            ('area_ratio',
             'pickup_target_area_ratio',
             'pickup_area_tolerance'),
            ('bottom_y_ratio',
             'pickup_target_bottom_y_ratio',
             'pickup_bottom_y_tolerance'),
            ('center_x_ratio',
             'pickup_target_center_x_ratio',
             'pickup_center_x_tolerance'),
            ('width_ratio',
             'pickup_target_width_ratio',
             'pickup_width_tolerance'))
        return all(
            _finite(
                getattr(
                    o,
                    key)) is not None and abs(
                float(
                    getattr(
                        o,
                        key)) -
                float(
                    self.p[target])) <= float(
                self.p[tolerance]) for key,
            target,
            tolerance in pairs)

    def _pickup_target_configured(self) -> bool:
        """显式签名标记避免把数值 0 错当成已完成的现场标定。"""
        return bool(self.p['pickup_board_signature_valid'])

    def _place_target_configured(self) -> bool:
        """显式签名标记避免空 YAML 值被 ROS 参数系统静默转换。"""
        return bool(self.p['place_white_bar_signature_valid'])

    def _run_timed_motion(
            self,
            prefix: str,
            dt: float,
            destination: str) -> None:
        duration = float(self.p[prefix + '_duration_sec'])
        speed = abs(float(self.p[prefix + '_speed_mps']))
        required = self._motion_required(prefix)
        if not required:
            self.state = destination
            self.reason = prefix + '_explicitly_not_required'
            self._reset_zero_confirmation()
            return
        if (prefix.startswith('pickup_')
                and not bool(self.p['pickup_timing_calibrated'])):
            self._fail(self._not_calibrated_state(prefix))
            return
        if duration <= 0.0 or speed <= 0.0:
            if required:
                self._fail(self._not_calibrated_state(prefix))
            return
        # 候选只表达意图；累计时间只读取 mux 的最终实际命令方向。
        requested = self._timed_command(prefix)
        if bool(self.p['allow_motion_execution']):
            self._command = requested
        if (bool(self.p['allow_motion_execution'])
                and self._final_command_matches(prefix)):
            self._active_elapsed += dt
        if self._active_elapsed >= duration:
            self.state = destination
            self.reason = prefix + '_active_duration_met'
            self._reset_zero_confirmation()

    def _timed_command(self, prefix: str) -> PlatformCommand:
        speed = abs(float(self.p[prefix + '_speed_mps']))
        if prefix == 'pickup_view_reverse':
            return PlatformCommand(vx=-speed)
        if prefix == 'pickup_side_forward':
            return PlatformCommand(vx=speed)
        direction = str(self.p.get(prefix + '_direction', 'forward'))
        if direction == 'reverse':
            return PlatformCommand(vx=-speed)
        if direction == 'left':
            return PlatformCommand(vy=speed)
        if direction == 'right':
            return PlatformCommand(vy=-speed)
        return PlatformCommand(vx=speed)

    def _start_turn(self, *, left: bool, now: float) -> None:
        if self._yaw_current is None:
            self.reason = 'yaw_unavailable'
            return
        angle_key = ('pickup_turn_left_angle_deg' if left else
                     'pickup_turn_right_angle_deg')
        angle = math.radians(float(self.p[angle_key]))
        self._yaw_start = self._yaw_current
        self._yaw_target = self._yaw_start + (angle if left else -angle)
        self._turn_started_at = float(now)
        self.state = 'PICKUP_TURN_LEFT' if left else 'PICKUP_TURN_RIGHT'
        self.reason = 'pickup_yaw_turn_started'

    def _run_yaw_turn(self, now: float) -> None:
        if (self._turn_started_at is not None
                and now - self._turn_started_at >= float(
                    self.p['pickup_yaw_timeout_sec'])):
            self._fail('PICKUP_YAW_TIMEOUT')
            return
        if self._yaw_target is None or self._yaw_current is None:
            self.reason = 'yaw_unavailable'
            return
        delta = _angle_delta(self._yaw_target, self._yaw_current)
        if abs(delta) <= math.radians(
                float(self.p['pickup_yaw_tolerance_deg'])):
            previous = self.state
            self._yaw_target = None
            self._turn_started_at = None
            if previous == 'PICKUP_TURN_LEFT':
                self.state = 'PICKUP_VIEW_REVERSE'
                self.reason = 'pickup_left_yaw_closed'
                self._active_elapsed = 0.0
            else:
                self.state = 'PICKUP_SIDE_POSITION_READY'
                self.reason = 'pickup_right_yaw_closed'
                self._reset_zero_confirmation()
            return
        if bool(self.p['allow_motion_execution']):
            self._command = PlatformCommand(wz=math.copysign(
                float(self.p['pickup_turn_speed_radps']), delta))

    def _zero_samples_required(self) -> int:
        if self.state.startswith('TRANSFER'):
            return int(self.p['transfer_zero_confirm_samples'])
        if self.state.startswith('PLACE'):
            return int(self.p['place_zero_confirm_samples'])
        return int(self.p['pickup_zero_confirm_samples'])

    def _zero_epsilon(self) -> float:
        """按平台选择最终速度零门限，避免一个参数掩盖各任务标定。"""
        if self.state.startswith('TRANSFER'):
            return float(self.p['transfer_zero_epsilon'])
        if self.state.startswith('PLACE'):
            return float(self.p['place_zero_epsilon'])
        return float(self.p['pickup_zero_epsilon'])

    def _reset_zero_confirmation(self) -> None:
        """进入新停车门时清除旧阶段零样本，禁止跨状态借用确认。"""
        self._counts['zero'] = 0
        self.stop_confirmed = False

    def _motion_required(self, prefix: str) -> bool:
        if prefix == 'place_final_offset':
            return bool(self.p['place_final_offset_enabled'])
        return bool(self.p.get(prefix + '_required', True))

    def _final_command_matches(self, prefix: str) -> bool:
        """只有 mux 最终命令方向与当前短距离动作一致才累计时间。"""
        epsilon = float(self.p['active_motion_direction_epsilon'])
        command = self._final_command
        if prefix == 'pickup_view_reverse':
            return command.vx < -epsilon
        if prefix == 'pickup_side_forward':
            return command.vx > epsilon
        direction = str(self.p.get(prefix + '_direction', 'forward'))
        if direction == 'reverse':
            return command.vx < -epsilon
        if direction == 'left':
            return command.vy > epsilon
        if direction == 'right':
            return command.vy < -epsilon
        return command.vx > epsilon

    @staticmethod
    def _not_calibrated_state(prefix: str) -> str:
        names = {
            'transfer_offset': 'TRANSFER_OFFSET_NOT_CALIBRATED',
            'pickup_view_reverse': 'PICKUP_REVERSE_NOT_CALIBRATED',
            'pickup_side_forward': 'PICKUP_FORWARD_NOT_CALIBRATED',
            'place_final_offset': 'PLACE_FINAL_OFFSET_NOT_CALIBRATED',
        }
        return names[prefix]

    def _fail(self, reason: str) -> None:
        """所有配置/控制超时故障均锁存原因并保持零候选。"""
        self.state = str(reason)
        self.reason = str(reason)
        self.failure_reason = str(reason)
        self._command = PlatformCommand()

    def _remaining_duration(self) -> Optional[float]:
        prefixes = {
            'TRANSFER_FINE_OFFSET': 'transfer_offset',
            'PICKUP_VIEW_REVERSE': 'pickup_view_reverse',
            'PICKUP_SIDE_FORWARD': 'pickup_side_forward',
            'PLACE_FINAL_OFFSET': 'place_final_offset'}
        prefix = prefixes.get(self.state)
        return None if prefix is None else max(0.0, float(
            self.p[prefix + '_duration_sec']) - self._active_elapsed)

    def _platform(self) -> str:
        if self.state.startswith('TRANSFER'):
            return 'transfer'
        if self.state.startswith('PICKUP'):
            return 'pickup'
        if self.state.startswith('PLACE'):
            return 'place'
        return 'none'

    def _platform_calibrated(self) -> bool:
        return bool(
            self.p.get(
                self._platform() +
                '_platform_calibrated',
                False))

    def _signature_valid(self) -> bool:
        names = {
            'transfer': 'transfer_anchor_signature_valid',
            'pickup': 'pickup_board_signature_valid',
            'place': 'place_white_bar_signature_valid',
        }
        parameter = names.get(self._platform())
        return bool(parameter and self.p[parameter])

    def _confirm_count(self) -> int:
        names = {
            'transfer': 'transfer',
            'pickup': 'pickup_board',
            'place': 'place',
        }
        return self._counts.get(names.get(self._platform(), ''), 0)

    def _target_values(self) -> dict:
        """将当前平台的黄金目标显式放入状态，便于现场逐项核对。"""
        if self._platform() == 'transfer':
            return {
                'heading_error': self.p[
                    'transfer_anchor_target_heading_error'],
                'lateral_error': self.p[
                    'transfer_anchor_target_lateral_error'],
            }
        if self._platform() == 'pickup':
            return {
                'area_ratio': self.p['pickup_target_area_ratio'],
                'bottom_y_ratio': self.p['pickup_target_bottom_y_ratio'],
                'center_x_ratio': self.p['pickup_target_center_x_ratio'],
                'width_ratio': self.p['pickup_target_width_ratio'],
            }
        if self._platform() == 'place':
            return {
                'white_bar_y_ratio': self.p['place_white_bar_target_y_ratio'],
                'white_bar_span_ratio': self.p[
                    'place_white_bar_target_span_ratio'],
                'line_lateral_error': 0.0,
                'line_heading_error': 0.0,
            }
        return {}

    def _current_values(self) -> dict:
        if self._platform() == 'transfer':
            return dict(self._last_transfer)
        if self._platform() == 'pickup':
            return dict(self._last_board.__dict__)
        if self._platform() == 'place':
            values = dict(self._last_white.__dict__)
            values.update(self._last_place_line)
            return values
        return {}

    def _position_errors(self) -> dict:
        targets = self._target_values()
        current = self._current_values()
        aliases = {
            'white_bar_y_ratio': 'center_y_ratio',
            'white_bar_span_ratio': 'span_ratio',
        }
        return {
            target: None if target_value is None or current.get(
                aliases.get(target, target)) is None else current.get(
                aliases.get(target, target)) - target_value
            for target, target_value in targets.items()
        }


# 所有黄金值初始为空，避免把未经现场测量的猜测写成生产标定值。
DEFAULT_PLATFORM_PARAMETERS = {
    'allow_motion_execution': False,
    'active_motion_direction_epsilon': 0.001,
    'transfer_platform_calibrated': False,
    'transfer_anchor_signature_valid': False,
    'transfer_anchor_confirm_frames': 3,
    'transfer_anchor_min_confidence': 0.0,
    'transfer_anchor_target_heading_error': 0.0,
    'transfer_anchor_heading_tolerance': 0.0,
    'transfer_anchor_target_lateral_error': 0.0,
    'transfer_anchor_lateral_tolerance': 0.0,
    'transfer_offset_required': True,
    'transfer_offset_direction': 'forward',
    'transfer_offset_speed_mps': 0.0,
    'transfer_offset_duration_sec': 0.0,
    'transfer_zero_confirm_samples': 3,
    'transfer_zero_epsilon': 0.001,
    'pickup_platform_calibrated': False,
    'pickup_board_signature_valid': False,
    'pickup_timing_calibrated': False,
    'pickup_board_confirm_frames': 3,
    'pickup_position_confirm_frames': 3,
    'pickup_zero_confirm_samples': 3,
    'pickup_zero_epsilon': 0.001,
    'pickup_target_area_ratio': 0.0,
    'pickup_area_tolerance': 0.0,
    'pickup_target_bottom_y_ratio': 0.0,
    'pickup_bottom_y_tolerance': 0.0,
    'pickup_target_center_x_ratio': 0.0,
    'pickup_center_x_tolerance': 0.0,
    'pickup_target_width_ratio': 0.0,
    'pickup_width_tolerance': 0.0,
    'pickup_visual_approach_speed_mps': 0.0,
    'pickup_visual_slow_trigger_ratio': 0.0,
    'pickup_turn_left_angle_deg': 90.0,
    'pickup_turn_right_angle_deg': 90.0,
    'pickup_turn_speed_radps': 0.0,
    'pickup_yaw_tolerance_deg': 0.0,
    'pickup_yaw_timeout_sec': 8.0,
    'pickup_view_reverse_required': True,
    'pickup_view_reverse_speed_mps': 0.0,
    'pickup_view_reverse_duration_sec': 0.0,
    'pickup_side_forward_required': True,
    'pickup_side_forward_speed_mps': 0.0,
    'pickup_side_forward_duration_sec': 0.0,
    'place_platform_calibrated': False,
    'place_white_bar_signature_valid': False,
    'place_white_bar_confirm_frames': 3,
    'place_zero_confirm_samples': 3,
    'place_zero_epsilon': 0.001,
    'place_white_bar_target_y_ratio': 0.0,
    'place_white_bar_y_tolerance': 0.0,
    'place_white_bar_target_span_ratio': 0.0,
    'place_white_bar_span_tolerance': 0.0,
    'place_white_bar_slow_y_ratio': 0.0,
    'place_visual_approach_speed_mps': 0.0,
    'place_line_max_lateral_error': 0.0,
    'place_line_max_heading_error': 0.0,
    'place_final_offset_enabled': False,
    'place_final_offset_direction': 'forward',
    'place_final_offset_speed_mps': 0.0,
    'place_final_offset_duration_sec': 0.0,
}
