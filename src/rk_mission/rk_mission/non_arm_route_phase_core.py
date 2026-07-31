"""非机械臂赛道的 ROS 无关阶段门控核心。

本模块保存路线阶段、run_id 和一次性里程碑，
不依赖 ROS 或直接速度控制。
ROS 节点把传感器确认、动作结果和停止命令，
转换为显式事件，
避免依据白线数量等隐式信号猜测当前赛程。
"""

from dataclasses import dataclass
import math


ROUTE_PHASES = frozenset((
    'WAIT_START',
    'START_STAGE',
    'START_REACQUIRE',
    'MID_ROUTE',
    'INSPECTION_APPROACH',
    'INSPECTION_WAIT_SIGN',
    'INSPECTION_ACTION',
    'POST_INSPECTION',
    'FINISH_STAGE',
    'FINISH_REACQUIRE',
    'FINAL_ZONE_ARMED',
    'FINAL_STOP',
    'FAULTED',
))


def validate_white_bar_timeout_chain(
    start_profile_duration_sec,
    finish_profile_duration_sec,
    executor_timeout_sec,
    line_course_timeout_sec,
    safety_margin_sec=3.0,
):
    """校验跳跃、执行器和路线层的严格超时链。

    ``start_profile_duration_sec`` 与 ``finish_profile_duration_sec``
    可为数字，也可为带 ``worst_case_duration_sec`` 的 profile。
    测试可直接使用真实 profile；
    ROS 节点可使用可审计的配置快照。
    """
    durations = (
        _timeout_number(start_profile_duration_sec, 'start_profile_duration'),
        _timeout_number(
            finish_profile_duration_sec,
            'finish_profile_duration',
        ),
    )
    executor_timeout = _timeout_number(
        executor_timeout_sec,
        'executor_timeout_sec',
    )
    line_timeout = _timeout_number(
        line_course_timeout_sec,
        'line_course_timeout_sec',
    )
    margin = _timeout_number(safety_margin_sec, 'safety_margin_sec')
    profile_limit = max(durations) + margin
    if executor_timeout <= profile_limit:
        raise ValueError(
            'executor_timeout_sec must exceed every FrontJump profile plus '
            'safety_margin_sec'
        )
    if line_timeout <= executor_timeout + margin:
        raise ValueError(
            'line_course_timeout_sec must exceed executor_timeout_sec plus '
            'safety_margin_sec'
        )
    return True


def _timeout_number(value, name):
    """解包时间配置，拒绝无效值以使超时链故障关闭。"""
    candidate = getattr(value, 'worst_case_duration_sec', value)
    try:
        number = float(candidate)
    except (TypeError, ValueError) as error:
        raise ValueError(f'{name} must be a finite positive number') from error
    if not math.isfinite(number) or number <= 0.0:
        raise ValueError(f'{name} must be a finite positive number')
    return number


@dataclass(frozen=True)
class RoutePhaseEvent:
    """路线决策的不可变快照，供 ROS 适配层安全发布。"""

    accepted: bool
    action: str
    route_phase: str
    reason: str
    mission_started: bool
    run_id: str
    start_jump_completed: bool
    inspection_completed: bool
    finish_jump_completed: bool
    final_zone_armed: bool
    active_request_id: str
    active_stage: str
    fault_reason: str


class NonArmRoutePhaseCore:
    """显式事件约束 START、检查、FINISH 与终点停车顺序。

    错误阶段传感器消息是常见噪声；适配层清零计数，
    不跨阶段累积。非法动作或命令进入 ``FAULTED``，
    以零速故障关闭。
    """

    def __init__(self):
        """建立未开始任务的安全初始状态。"""
        self._reset_wait_start()

    def mission_start(self, run_id):
        """开始新任务；重复 start 保留 run_id 与已有阶段。"""
        if self.mission_started:
            return self._event(
                True,
                'IGNORED',
                'mission_start_duplicate_ignored',
            )
        if not self._is_nonempty_string(run_id):
            return self.fault('mission_start_invalid_run_id')

        self.mission_started = True
        self.run_id = run_id.strip()
        self.route_phase = 'START_STAGE'
        self.start_jump_completed = False
        self.inspection_completed = False
        self.finish_jump_completed = False
        self.final_zone_armed = False
        self.active_request_id = ''
        self.active_stage = ''
        self.red_circle_consumed = False
        self.fault_reason = ''
        return self._event(True, 'TRANSITION', 'mission_start')

    def mission_stop(self):
        """幂等清空关联状态，使旧 run 与动作结果失效。"""
        was_started = self.mission_started
        self._reset_wait_start()
        reason = (
            'mission_stop_reset'
            if was_started else 'mission_stop_idempotent'
        )
        return self._event(True, 'RESET', reason)

    def status_event(self, reason='route_phase_status'):
        """返回当前快照，不改变任何阶段或安全标志。"""
        return self._event(True, 'STATUS', str(reason))

    def can_accept_stage_command(self, stage):
        """检查白横线阶段命令是否符合正式路线次序。"""
        if stage == 'START':
            return (
                self.mission_started
                and self.route_phase == 'START_STAGE'
                and not self.start_jump_completed
                and self.active_stage in ('', 'START')
            )
        if stage == 'FINISH':
            return (
                self.mission_started
                and self.start_jump_completed
                and self.inspection_completed
                and self.route_phase in ('POST_INSPECTION', 'FINISH_STAGE')
                and not self.finish_jump_completed
                and self.active_stage in ('', 'FINISH')
            )
        return False

    def accept_stage_command(self, stage):
        """记录通过校验的 START 或 FINISH 命令，禁止倒退。"""
        if not self.can_accept_stage_command(stage):
            return self.fault(f'illegal_stage_command_{stage}')
        if self.active_stage == stage:
            return self._event(
                True,
                'IGNORED',
                'stage_command_duplicate_ignored',
            )
        self.active_stage = stage
        if stage == 'FINISH':
            self.route_phase = 'FINISH_STAGE'
        return self._event(True, 'STAGE_ARMED', f'{stage.lower()}_stage_armed')

    def white_bar_detection_allowed(self, stage):
        """仅在已显式 arm 的对应阶段允许白横线确认计数。"""
        if not self.mission_started or self.active_stage != stage:
            return False
        if stage == 'START':
            return self.route_phase == 'START_STAGE'
        if stage == 'FINISH':
            return self.route_phase == 'FINISH_STAGE'
        return False

    def white_bar_action_started(self, stage):
        """确认白横线只触发当前阶段映射的跳跃动作。"""
        if not self.white_bar_detection_allowed(stage):
            return self.fault(f'illegal_{stage.lower()}_jump_start')
        return self._event(
            True,
            'WHITE_BAR_ACTION_STARTED',
            f'{stage.lower()}_jump_started',
        )

    def white_bar_action_completed(self, stage):
        """匹配跳跃成功后重新找线，禁止直接恢复巡线。"""
        if stage != self.active_stage:
            return self.fault('white_bar_completion_stage_mismatch')
        if stage == 'START' and self.route_phase == 'START_STAGE':
            self.start_jump_completed = True
            self.active_stage = ''
            self.route_phase = 'START_REACQUIRE'
            return self._event(True, 'ALIGN_REQUIRED', 'start_jump_completed')
        if stage == 'FINISH' and self.route_phase == 'FINISH_STAGE':
            self.finish_jump_completed = True
            self.active_stage = ''
            self.route_phase = 'FINISH_REACQUIRE'
            return self._event(True, 'ALIGN_REQUIRED', 'finish_jump_completed')
        return self.fault('white_bar_completion_out_of_order')

    def white_bar_action_failed(self, reason):
        """跳跃超时、拒绝或取消后不能继续，安全停车。"""
        return self.fault(str(reason or 'white_bar_action_failed'))

    def red_detection_allowed(self):
        """红圈只在 START 对齐后的中段路线允许确认一次。"""
        return (
            self.mission_started
            and self.route_phase == 'MID_ROUTE'
            and not self.red_circle_consumed
            and not self.inspection_completed
        )

    def red_circle_confirmed(self):
        """消费一次稳定红圈，进入受限的红圈靠近阶段。"""
        if not self.red_detection_allowed():
            return self._event(
                False,
                'IGNORED',
                'red_circle_not_allowed_in_phase',
            )
        self.red_circle_consumed = True
        self.route_phase = 'INSPECTION_APPROACH'
        return self._event(True, 'TRANSITION', 'red_circle_confirmed')

    def red_circle_reached(self, request_id):
        """红圈到位后创建唯一检查请求，零速等待结果。"""
        if self.route_phase != 'INSPECTION_APPROACH':
            return self.fault('red_circle_reached_out_of_order')
        if not self._is_nonempty_string(request_id):
            return self.fault('inspection_request_id_invalid')
        self.active_request_id = request_id.strip()
        self.route_phase = 'INSPECTION_WAIT_SIGN'
        return self._event(
            True,
            'REQUEST_INSPECTION',
            'inspection_request_sent',
        )

    def inspection_action_progress(self, request_id, running=False):
        """接受当前请求进展，安全忽略旧请求和其他 run。"""
        if request_id != self.active_request_id or not request_id:
            return self._event(
                False,
                'IGNORED',
                'inspection_status_request_id_mismatch',
            )
        if self.route_phase not in (
            'INSPECTION_WAIT_SIGN',
            'INSPECTION_ACTION',
        ):
            return self._event(
                False,
                'IGNORED',
                'inspection_status_not_active',
            )
        if running:
            self.route_phase = 'INSPECTION_ACTION'
        return self._event(True, 'STATUS', 'inspection_action_progress')

    def inspection_action_succeeded(self, request_id):
        """只接受当前 request_id 成功结果，随后转向。"""
        if request_id != self.active_request_id or not request_id:
            return self._event(
                False,
                'IGNORED',
                'inspection_success_request_id_mismatch',
            )
        if self.route_phase not in (
            'INSPECTION_WAIT_SIGN',
            'INSPECTION_ACTION',
        ):
            return self._event(
                False,
                'IGNORED',
                'inspection_success_not_active',
            )
        self.inspection_completed = True
        self.active_request_id = ''
        self.route_phase = 'POST_INSPECTION'
        return self._event(
            True,
            'TURN_AFTER_RED',
            'inspection_action_succeeded',
        )

    def inspection_action_failed(self, request_id, reason):
        """匹配请求失败不可回退或重试，直接故障关闭。"""
        if request_id != self.active_request_id or not request_id:
            return self._event(
                False,
                'IGNORED',
                'inspection_failure_request_id_mismatch',
            )
        return self.fault(str(reason or 'inspection_action_failed'))

    def corner_detection_allowed(self):
        """巡线才允许角点候选，检查与终点前不累计。"""
        return self.mission_started and self.route_phase in (
            'MID_ROUTE',
            'POST_INSPECTION',
            'FINISH_STAGE',
        )

    def corner_confirmed(self):
        """角点是局部受控动作，对齐后路线阶段不变。"""
        if not self.corner_detection_allowed():
            return self._event(False, 'IGNORED', 'corner_not_allowed_in_phase')
        return self._event(True, 'CORNER_TURN', 'corner_confirmed')

    def alignment_completed(self, context):
        """只允许指定动作后的对齐成功解锁下一阶段。"""
        if context == 'start' and self.route_phase == 'START_REACQUIRE':
            self.route_phase = 'MID_ROUTE'
            return self._event(True, 'TRANSITION', 'start_alignment_completed')
        if context == 'finish' and self.route_phase == 'FINISH_REACQUIRE':
            self.route_phase = 'FINAL_ZONE_ARMED'
            self.final_zone_armed = True
            return self._event(
                True,
                'TRANSITION',
                'finish_alignment_completed',
            )
        if context == 'red' and self.route_phase in (
            'POST_INSPECTION',
            'FINISH_STAGE',
        ):
            return self._event(
                True,
                'TRANSITION',
                'red_turn_alignment_completed',
            )
        if context == 'corner' and self.corner_detection_allowed():
            return self._event(
                True,
                'TRANSITION',
                'corner_alignment_completed',
            )
        return self.fault(f'alignment_{context}_out_of_order')

    def stop_zone_detection_allowed(self):
        """终点蓝区只在 FINISH 重新找线成功后才开始累计。"""
        return (
            self.mission_started
            and self.route_phase == 'FINAL_ZONE_ARMED'
            and self.final_zone_armed
        )

    def stop_zone_confirmed(self):
        """允许适配层低速靠近终点，但不提前完成任务。"""
        if not self.stop_zone_detection_allowed():
            return self._event(
                False,
                'IGNORED',
                'stop_zone_not_allowed_in_phase',
            )
        return self._event(True, 'APPROACH_STOP_ZONE', 'stop_zone_confirmed')

    def stop_zone_inside_confirmed(self):
        """只在已 arm 的终点区内，连续 inside 才终止任务。"""
        if not self.stop_zone_detection_allowed():
            return self._event(
                False,
                'IGNORED',
                'stop_zone_inside_not_allowed',
            )
        self.route_phase = 'FINAL_STOP'
        self.active_stage = ''
        self.active_request_id = ''
        return self._event(True, 'FINAL_STOP', 'stop_zone_inside_confirmed')

    def fault(self, reason):
        """记录首次故障原因并锁定，等待显式 stop/reset。"""
        if self.route_phase != 'FAULTED':
            self.fault_reason = str(reason or 'route_phase_fault')
        self.route_phase = 'FAULTED'
        self.active_stage = ''
        self.active_request_id = ''
        return self._event(False, 'FAULT', self.fault_reason)

    def _reset_wait_start(self):
        """集中重置跨阶段关联，防止旧任务泄漏。"""
        self.mission_started = False
        self.run_id = ''
        self.route_phase = 'WAIT_START'
        self.start_jump_completed = False
        self.inspection_completed = False
        self.finish_jump_completed = False
        self.final_zone_armed = False
        self.active_request_id = ''
        self.active_stage = ''
        self.red_circle_consumed = False
        self.fault_reason = ''

    @staticmethod
    def _is_nonempty_string(value):
        """拒绝空白 run/request id，避免旧结果匹配。"""
        return type(value) is str and bool(value.strip())

    def _event(self, accepted, action, reason):
        """从唯一真相状态生成事件，保证发布字段一致。"""
        return RoutePhaseEvent(
            accepted=bool(accepted),
            action=str(action),
            route_phase=self.route_phase,
            reason=str(reason),
            mission_started=bool(self.mission_started),
            run_id=self.run_id,
            start_jump_completed=bool(self.start_jump_completed),
            inspection_completed=bool(self.inspection_completed),
            finish_jump_completed=bool(self.finish_jump_completed),
            final_zone_armed=bool(self.final_zone_armed),
            active_request_id=self.active_request_id,
            active_stage=self.active_stage,
            fault_reason=self.fault_reason,
        )
