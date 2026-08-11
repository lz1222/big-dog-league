"""国赛 TRANSFER 固定路线的 ROS 无关安全状态机。

数据方向是“视觉/里程计/mux 最终命令 -> 状态决策 -> mission
candidate”。固定前进只累计经 mux 验证的有效运动时间；固定转弯只使用
新鲜真实 odom 闭环。所有未标定或非有限输入均故障关闭。
"""

from dataclasses import dataclass
import math
from typing import Optional, Tuple


RIGHT_CORNER_SEARCH = 'TRANSFER_RIGHT_CORNER_SEARCH'
PRE_CORNER_FORWARD = 'TRANSFER_PRE_CORNER_FORWARD'
PRE_CORNER_STOP = 'TRANSFER_PRE_CORNER_STOP'
PRE_CORNER_TURN = 'TRANSFER_PRE_CORNER_TURN'
PRE_CORNER_REACQUIRE = 'TRANSFER_PRE_CORNER_REACQUIRE'
ARC_APPROACH = 'TRANSFER_ARC_APPROACH'
ARC_TO_TASK = 'TRANSFER_ARC_TO_TASK'
TASK_STOP = 'TRANSFER_TASK_STOP'
PLACE_TASK = 'TRANSFER_PLACE_TASK'
PICK_TASK = 'TRANSFER_PICK_TASK'
ARC_EXIT_STOP = 'TRANSFER_ARC_EXIT_STOP'
ARC_TO_EXIT = 'TRANSFER_ARC_TO_EXIT'
LINE_REACQUIRE = 'TRANSFER_LINE_REACQUIRE'
COMPLETE = 'TRANSFER_COMPLETE'
SAFE_STOP = 'SAFE_STOP'

TRANSFER_STATES = frozenset((
    RIGHT_CORNER_SEARCH, PRE_CORNER_FORWARD, PRE_CORNER_STOP,
    PRE_CORNER_TURN, PRE_CORNER_REACQUIRE, ARC_APPROACH, ARC_TO_TASK,
    TASK_STOP, PLACE_TASK, PICK_TASK, ARC_EXIT_STOP, ARC_TO_EXIT,
    LINE_REACQUIRE, COMPLETE, SAFE_STOP,
))


@dataclass(frozen=True)
class TransferRouteConfig:
    """路线参数；所有实体距离/角度均保留 YAML 标定入口。"""

    right_corner_confirm_frames: int = 3
    right_corner_motion_calibrated: bool = False
    right_corner_forward_speed: float = 0.0
    right_corner_forward_active_time_sec: float = 0.0
    right_corner_forward_timeout_sec: float = 20.0
    right_corner_turn_wz: float = 0.0
    right_corner_target_yaw_deg: float = 90.0
    right_corner_turn_timeout_sec: float = 8.0
    transfer_arc_entry_calibrated: bool = False
    transfer_arc_entry_confirm_frames: int = 3
    transfer_arc_motion_calibrated: bool = False
    transfer_arc_vx: float = 0.0
    transfer_arc_wz: float = 0.0
    transfer_arc_task_stop_yaw_deg: float = 0.0
    transfer_arc_exit_delta_yaw_deg: float = 0.0
    transfer_arc_yaw_timeout_sec: float = 12.0
    reacquire_frames: int = 3
    reacquire_min_confidence: float = 0.65
    reacquire_max_lateral_error: float = 0.75
    zero_confirm_frames: int = 3
    zero_epsilon: float = 0.001
    final_cmd_tolerance: float = 0.005
    yaw_wrong_direction_tolerance_deg: float = 3.0
    arm_action_timeout_sec: float = 90.0


@dataclass(frozen=True)
class TransferInputs:
    """每个控制周期的安全真相快照。"""

    final_cmd: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    final_cmd_fresh: bool = False
    mux_healthy: bool = False
    gait_available: bool = False
    odom_yaw: Optional[float] = None
    odom_fresh: bool = False


@dataclass(frozen=True)
class TransferDecision:
    """单周期决策；``control`` 说明速度所有权。"""

    state: str
    control: str
    vx: float
    wz: float
    action: str
    reason: str


class TransferRouteCore:
    """严格执行 RIGHT corner -> LEFT arc task -> LEFT arc exit。"""

    def __init__(self, config=None):
        self.config = config or TransferRouteConfig()
        self._validate_common_config()
        self.reset()

    def reset(self):
        """新 run 清空全部 one-shot 里程碑和时钟。"""
        self.state = RIGHT_CORNER_SEARCH
        self.reason = 'transfer_route_ready'
        self.fault_reason = ''
        self.state_enter_time = None
        self.last_tick_time = None
        self.active_elapsed = 0.0
        self.right_corner_count = 0
        self.arc_entry_count = 0
        self.reacquire_count = 0
        self.zero_count = 0
        self.start_yaw = None
        self.action_dispatched = False
        self.action_started_time = None
        self.transfer_right_corner_detected = False
        self.transfer_right_corner_completed = False
        self.transfer_arc_started = False
        self.transfer_task_stop_reached = False
        self.transfer_place_completed = False
        self.transfer_pick_completed = False
        self.transfer_arc_completed = False

    def observe_corner(self, visible, confidence, direction_hint, now):
        """仅在当前专用 phase 消费角点，已完成里程碑永不重复。"""
        valid = (
            bool(visible) and self._finite(confidence)
            and float(confidence) >= 0.0
        )
        if self.state == RIGHT_CORNER_SEARCH:
            self.right_corner_count = (
                self.right_corner_count + 1 if valid else 0
            )
            if (
                self.right_corner_count
                >= self.config.right_corner_confirm_frames
            ):
                if self.transfer_right_corner_detected:
                    return
                self.transfer_right_corner_detected = True
                if not self.config.right_corner_motion_calibrated:
                    self._fail('RIGHT_CORNER_NOT_CALIBRATED', now)
                    return
                if not self._right_motion_values_valid():
                    self._fail('RIGHT_CORNER_PARAMETERS_INVALID', now)
                    return
                self.active_elapsed = 0.0
                self._transition(PRE_CORNER_FORWARD, now,
                                 'TRANSFER_RIGHT_CORNER_POINT')
        elif self.state == ARC_APPROACH:
            # U-turn 入口是 phase 限定的独立左向门，不调用 generic
            # corner 动作，也不允许右角二次消费。
            valid_left = valid and str(direction_hint).lower() == 'left'
            self.arc_entry_count = (
                self.arc_entry_count + 1 if valid_left else 0
            )
            if (
                self.arc_entry_count
                >= self.config.transfer_arc_entry_confirm_frames
            ):
                if self.transfer_arc_started:
                    return
                if not self.config.transfer_arc_entry_calibrated:
                    self._fail('TRANSFER_ARC_ENTRY_NOT_CALIBRATED', now)
                    return
                if not self.config.transfer_arc_motion_calibrated:
                    self._fail('TRANSFER_ARC_NOT_CALIBRATED', now)
                    return
                if not self._arc_motion_values_valid():
                    self._fail('TRANSFER_ARC_PARAMETERS_INVALID', now)
                    return
                self.reason = 'TRANSFER_ARC_ENTRY_CONFIRMED'
                self.transfer_arc_started = True

    def observe_line(self, visible, confidence, lateral_error, now):
        """固定动作中丢线完全允许；只有两个 reacquire 状态累计。"""
        if self.state not in (PRE_CORNER_REACQUIRE, LINE_REACQUIRE):
            return
        valid = (
            bool(visible)
            and self._finite(confidence)
            and self._finite(lateral_error)
            and float(confidence) >= self.config.reacquire_min_confidence
            and abs(float(lateral_error))
            <= self.config.reacquire_max_lateral_error
        )
        self.reacquire_count = self.reacquire_count + 1 if valid else 0
        if self.reacquire_count < self.config.reacquire_frames:
            return
        if self.state == PRE_CORNER_REACQUIRE:
            self._transition(ARC_APPROACH, now, 'RIGHT_CORNER_LINE_REACQUIRED')
        else:
            self.transfer_arc_completed = True
            self._transition(COMPLETE, now, 'TRANSFER_EXIT_LINE_REACQUIRED')

    def action_result(self, action, success, now, message=''):
        """只接受当前任务的一次结果，失败不自动重试机械臂。"""
        expected = (
            'transfer_place' if self.state == PLACE_TASK
            else 'transfer_pick' if self.state == PICK_TASK else ''
        )
        if action != expected or not self.action_dispatched:
            return False
        if not success:
            self._fail(
                '{}_FAILED{}'.format(
                    action.upper(), ':{}'.format(message) if message else ''
                ), now,
            )
            return True
        self.action_dispatched = False
        self.action_started_time = None
        if self.state == PLACE_TASK:
            self.transfer_place_completed = True
            self._transition(PICK_TASK, now, 'TRANSFER_PLACE_COMPLETED')
        else:
            self.transfer_pick_completed = True
            self.zero_count = 0
            self._transition(ARC_EXIT_STOP, now, 'TRANSFER_PICK_COMPLETED')
        return True

    def mark_action_dispatched(self, action, now):
        """记录 ActionClient 已发送，防止 10/20 Hz 定时器重复发 goal。"""
        expected = (
            'transfer_place' if self.state == PLACE_TASK
            else 'transfer_pick' if self.state == PICK_TASK else ''
        )
        if action != expected or self.action_dispatched:
            return False
        self.action_dispatched = True
        self.action_started_time = float(now)
        return True

    def tick(self, now, inputs):
        """根据最终命令与 odom 推进，从不使用 sleep/墙钟猜转角。"""
        now = float(now)
        if not self._finite(now):
            self._fail('TRANSFER_CLOCK_INVALID', 0.0)
            return self._decision('ZERO')
        if self.state_enter_time is None:
            self.state_enter_time = now
        dt = 0.0 if self.last_tick_time is None else max(
            0.0, now - self.last_tick_time
        )
        self.last_tick_time = now

        if self._fresh_final_is_nonfinite(inputs):
            self._fail('TRANSFER_FINAL_COMMAND_INVALID', now)
        if self.state == SAFE_STOP:
            return self._decision('ZERO')
        if self.state in (RIGHT_CORNER_SEARCH, ARC_APPROACH, COMPLETE):
            if self.state == ARC_APPROACH and self.transfer_arc_started:
                if not self._valid_odom(inputs):
                    self._fail('TRANSFER_ARC_ODOM_STALE_OR_INVALID', now)
                    return self._decision('ZERO')
                self.start_yaw = float(inputs.odom_yaw)
                self._transition(
                    ARC_TO_TASK, now, 'TRANSFER_ARC_TO_TASK_START'
                )
                return self._decision('ZERO')
            return self._decision('LINE_FOLLOW')
        if self.state == PRE_CORNER_FORWARD:
            if (
                self._elapsed(now)
                >= self.config.right_corner_forward_timeout_sec
            ):
                self._fail('RIGHT_CORNER_FORWARD_TIMEOUT', now)
                return self._decision('ZERO')
            command = (
                self.config.right_corner_forward_speed, 0.0, 0.0
            )
            if self._verified_motion(inputs, command):
                self.active_elapsed += dt
            if self.active_elapsed >= (
                self.config.right_corner_forward_active_time_sec
            ):
                self.zero_count = 0
                self._transition(PRE_CORNER_STOP, now,
                                 'RIGHT_CORNER_FORWARD_COMPLETE')
                return self._decision('ZERO')
            return self._fixed(command)
        if self.state == PRE_CORNER_STOP:
            if self._zero_confirmed(inputs):
                if not self._valid_odom(inputs):
                    self._fail('RIGHT_CORNER_ODOM_STALE_OR_INVALID', now)
                    return self._decision('ZERO')
                self.start_yaw = float(inputs.odom_yaw)
                self._transition(PRE_CORNER_TURN, now,
                                 'RIGHT_CORNER_ZERO_CONFIRMED')
            return self._decision('ZERO')
        if self.state == PRE_CORNER_TURN:
            return self._run_yaw_motion(
                now, inputs, expected_sign=-1.0,
                target_deg=self.config.right_corner_target_yaw_deg,
                timeout=self.config.right_corner_turn_timeout_sec,
                command=(0.0, 0.0, -abs(self.config.right_corner_turn_wz)),
                completion_state=PRE_CORNER_REACQUIRE,
                completion_reason='RIGHT_CORNER_TURN_COMPLETE',
            )
        if self.state in (PRE_CORNER_REACQUIRE, LINE_REACQUIRE):
            return self._decision('REACQUIRE')
        if self.state == ARC_TO_TASK:
            return self._run_yaw_motion(
                now, inputs, expected_sign=1.0,
                target_deg=self.config.transfer_arc_task_stop_yaw_deg,
                timeout=self.config.transfer_arc_yaw_timeout_sec,
                command=(self.config.transfer_arc_vx, 0.0,
                         abs(self.config.transfer_arc_wz)),
                completion_state=TASK_STOP,
                completion_reason='TRANSFER_TASK_STOP_REACHED',
            )
        if self.state == TASK_STOP:
            if self._zero_confirmed(inputs):
                self.transfer_task_stop_reached = True
                self._transition(
                    PLACE_TASK, now, 'TRANSFER_TASK_ZERO_CONFIRMED'
                )
            return self._decision('ZERO')
        if self.state in (PLACE_TASK, PICK_TASK):
            if (
                self.action_dispatched
                and self.action_started_time is not None
                and now - self.action_started_time
                >= self.config.arm_action_timeout_sec
            ):
                self._fail('TRANSFER_ARM_ACTION_TIMEOUT', now)
                return self._decision('ZERO')
            action = '' if self.action_dispatched else (
                'transfer_place'
                if self.state == PLACE_TASK else 'transfer_pick'
            )
            return self._decision('ZERO', action=action)
        if self.state == ARC_EXIT_STOP:
            if self._zero_confirmed(inputs):
                if not self._valid_odom(inputs):
                    self._fail('TRANSFER_ARC_EXIT_ODOM_STALE_OR_INVALID', now)
                    return self._decision('ZERO')
                self.start_yaw = float(inputs.odom_yaw)
                self._transition(
                    ARC_TO_EXIT, now, 'TRANSFER_ARC_TO_EXIT_START'
                )
            return self._decision('ZERO')
        if self.state == ARC_TO_EXIT:
            return self._run_yaw_motion(
                now, inputs, expected_sign=1.0,
                target_deg=self.config.transfer_arc_exit_delta_yaw_deg,
                timeout=self.config.transfer_arc_yaw_timeout_sec,
                command=(self.config.transfer_arc_vx, 0.0,
                         abs(self.config.transfer_arc_wz)),
                completion_state=LINE_REACQUIRE,
                completion_reason='TRANSFER_ARC_EXIT_COMPLETE',
            )
        self._fail('TRANSFER_STATE_INVALID', now)
        return self._decision('ZERO')

    def snapshot(self):
        """发布 one-shot 里程碑与调试时钟，便于赛前静态审计。"""
        return {
            'transfer_state': self.state,
            'transfer_reason': self.reason,
            'transfer_fault_reason': self.fault_reason,
            'transfer_right_corner_detected': (
                self.transfer_right_corner_detected
            ),
            'transfer_right_corner_completed': (
                self.transfer_right_corner_completed
            ),
            'transfer_arc_started': self.transfer_arc_started,
            'transfer_task_stop_reached': self.transfer_task_stop_reached,
            'transfer_place_completed': self.transfer_place_completed,
            'transfer_pick_completed': self.transfer_pick_completed,
            'transfer_arc_completed': self.transfer_arc_completed,
            'transfer_forward_active_elapsed': self.active_elapsed,
            'transfer_right_corner_confirm_count': self.right_corner_count,
            'transfer_arc_entry_confirm_count': self.arc_entry_count,
            'transfer_reacquire_count': self.reacquire_count,
        }

    def _run_yaw_motion(self, now, inputs, expected_sign, target_deg, timeout,
                        command, completion_state, completion_reason):
        if not self._valid_odom(inputs) or self.start_yaw is None:
            self._fail('TRANSFER_ODOM_STALE_OR_INVALID', now)
            return self._decision('ZERO')
        if self._elapsed(now) >= timeout:
            self._fail('TRANSFER_YAW_TIMEOUT', now)
            return self._decision('ZERO')
        delta = self._normalize_angle(float(inputs.odom_yaw) - self.start_yaw)
        motion_verified = self._verified_motion(inputs, command)
        wrong_tolerance = math.radians(
            self.config.yaw_wrong_direction_tolerance_deg
        )
        if expected_sign * delta < -wrong_tolerance:
            self._fail('TRANSFER_YAW_WRONG_DIRECTION', now)
            return self._decision('ZERO')
        if abs(delta) > wrong_tolerance and not motion_verified:
            self._fail('TRANSFER_YAW_PROGRESS_UNVERIFIED', now)
            return self._decision('ZERO')
        if (
            expected_sign * delta >= math.radians(target_deg)
            and motion_verified
        ):
            if self.state == PRE_CORNER_TURN:
                self.transfer_right_corner_completed = True
            self.zero_count = 0
            self.reacquire_count = 0
            self._transition(completion_state, now, completion_reason)
            return self._decision('ZERO')
        return self._fixed(command)

    def _zero_confirmed(self, inputs):
        zero = (
            inputs.final_cmd_fresh
            and self._command_matches(
                inputs.final_cmd, (0.0, 0.0, 0.0), self.config.zero_epsilon
            )
        )
        self.zero_count = self.zero_count + 1 if zero else 0
        return self.zero_count >= self.config.zero_confirm_frames

    def _verified_motion(self, inputs, expected):
        return (
            inputs.final_cmd_fresh and inputs.mux_healthy
            and inputs.gait_available
            and self._command_matches(
                inputs.final_cmd, expected, self.config.final_cmd_tolerance
            )
        )

    def _fixed(self, command):
        if not all(self._finite(value) for value in command):
            self._fail(
                'TRANSFER_COMMAND_NONFINITE', self.last_tick_time or 0.0
            )
            return self._decision('ZERO')
        return TransferDecision(
            self.state, 'FIXED', float(command[0]), float(command[2]), '',
            self.reason,
        )

    def _decision(self, control, action=''):
        return TransferDecision(
            self.state, control, 0.0, 0.0, action, self.reason
        )

    def _transition(self, state, now, reason):
        self.state = state
        self.state_enter_time = float(now)
        self.reason = str(reason)
        self.zero_count = 0
        if state in (PRE_CORNER_REACQUIRE, LINE_REACQUIRE):
            self.reacquire_count = 0

    def _fail(self, reason, now):
        if self.state != SAFE_STOP:
            self.fault_reason = str(reason)
        self._transition(SAFE_STOP, now, self.fault_reason)

    def _elapsed(self, now):
        entered = (
            now if self.state_enter_time is None else self.state_enter_time
        )
        return max(0.0, float(now) - float(entered))

    def _valid_odom(self, inputs):
        return inputs.odom_fresh and self._finite(inputs.odom_yaw)

    def _fresh_final_is_nonfinite(self, inputs):
        return inputs.final_cmd_fresh and not all(
            self._finite(value) for value in inputs.final_cmd
        )

    def _right_motion_values_valid(self):
        p = self.config
        return all((
            self._positive(p.right_corner_forward_speed),
            self._positive(p.right_corner_forward_active_time_sec),
            self._positive(p.right_corner_turn_wz),
            self._positive(p.right_corner_target_yaw_deg),
        ))

    def _arc_motion_values_valid(self):
        p = self.config
        return all((
            self._positive(p.transfer_arc_vx),
            self._positive(p.transfer_arc_wz),
            self._positive(p.transfer_arc_task_stop_yaw_deg),
            self._positive(p.transfer_arc_exit_delta_yaw_deg),
        ))

    def _validate_common_config(self):
        p = self.config
        integer_values = (
            p.right_corner_confirm_frames,
            p.transfer_arc_entry_confirm_frames,
            p.reacquire_frames, p.zero_confirm_frames,
        )
        if any(
            type(value) is not int or value < 1
            for value in integer_values
        ):
            raise ValueError('transfer frame counts must be integers >= 1')
        positive_values = (
            p.right_corner_forward_timeout_sec,
            p.right_corner_turn_timeout_sec, p.transfer_arc_yaw_timeout_sec,
            p.zero_epsilon, p.final_cmd_tolerance,
            p.arm_action_timeout_sec,
        )
        if not all(self._positive(value) for value in positive_values):
            raise ValueError('transfer safety time/tolerance must be positive')
        if not 0.0 <= p.reacquire_min_confidence <= 1.0:
            raise ValueError('reacquire_min_confidence must be in [0, 1]')

    @staticmethod
    def _command_matches(actual, expected, tolerance):
        return (
            len(actual) == 3
            and all(math.isfinite(float(value)) for value in actual)
            and all(
                abs(float(value) - float(target)) <= float(tolerance)
                for value, target in zip(actual, expected)
            )
        )

    @staticmethod
    def _normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def _finite(value):
        try:
            return math.isfinite(float(value))
        except (TypeError, ValueError):
            return False

    @classmethod
    def _positive(cls, value):
        return cls._finite(value) and float(value) > 0.0
