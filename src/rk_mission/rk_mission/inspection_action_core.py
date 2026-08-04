"""警示牌动作的 ROS 无关状态机和受监督 helper 运行器。

路线节点只需发送带 run_id/request_id 的显式请求；本模块不会根据普通
检测消息自行启动动作。这样既能在没有 ROS 图的单元测试中覆盖安全分支，
也能避免旧检测或其他任务的结果误触发当前比赛路线。
"""

from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time


REQUEST_ACTION = 'detect_and_execute_warning'
IDLE = 'IDLE'
ARMED = 'ARMED'
WAIT_SIGN = 'WAIT_SIGN'
COMMAND_READY = 'COMMAND_READY'
WAIT_ZERO = 'WAIT_ZERO'
RUNNING = 'RUNNING'
SUCCEEDED = 'SUCCEEDED'
FAILED = 'FAILED'
TIMEOUT = 'TIMEOUT'
CANCELED = 'CANCELED'
FAULTED = 'FAULTED'
# helper 已收到 TERM/KILL 请求但尚未 reap 时仍属于活动控制阶段；不能把
# CANCELED/FAULTED 提前暴露给停止脚本，否则会在进程组仍存活时结束流程。
CLEANUP_PENDING = 'CLEANUP_PENDING'

TERMINAL_STATES = frozenset((
    SUCCEEDED,
    FAILED,
    TIMEOUT,
    CANCELED,
    FAULTED,
))

# 比赛规则只允许这三种警示牌动作。特意不用默认值或近似匹配，未知牌必须
# 保持等待/超时而不是误做 stretch。
WARNING_ACTIONS = {
    ('warning', 'electric_shock'): 'stretch',
    ('warning', 'strong_oxidizer'): 'hello',
    ('warning', 'radiation'): 'blink_front_light_3',
}


class InspectionActionConfigurationError(ValueError):
    """在节点启动前报告不可安全执行的 inspection 参数。"""


class InspectionActionProtocolError(ValueError):
    """报告不符合 request JSON 合约的输入。"""


def normalize_label(value):
    """归一化检测标签，但不把未知标签映射为允许动作。"""
    normalized = str(value or '').strip().lower()
    normalized = normalized.replace('-', '_').replace(' ', '_')
    while '__' in normalized:
        normalized = normalized.replace('__', '_')
    return normalized


def warning_action_for(sign_type, sign_value):
    """返回三种允许警示牌的精确动作；其它输入一律返回 ``None``。"""
    key = (normalize_label(sign_type), normalize_label(sign_value))
    return WARNING_ACTIONS.get(key)


def finite_float(value, name, *, positive=False, nonnegative=False):
    """校验浮点配置，防止 NaN/负超时破坏 fail-closed 时间门。"""
    if isinstance(value, bool):
        raise InspectionActionConfigurationError(
            '{} must be a finite number'.format(name)
        )
    try:
        number = float(value)
    except (TypeError, ValueError) as error:
        raise InspectionActionConfigurationError(
            '{} must be a finite number'.format(name)
        ) from error
    if not math.isfinite(number):
        raise InspectionActionConfigurationError(
            '{} must be a finite number'.format(name)
        )
    if positive and number <= 0.0:
        raise InspectionActionConfigurationError(
            '{} must be greater than 0'.format(name)
        )
    if nonnegative and number < 0.0:
        raise InspectionActionConfigurationError(
            '{} must be greater than or equal to 0'.format(name)
        )
    return number


def positive_int(value, name):
    """校验连续帧/样本数量，拒绝 bool 和零值。"""
    if isinstance(value, bool):
        raise InspectionActionConfigurationError(
            '{} must be a positive integer'.format(name)
        )
    try:
        number = int(value)
    except (TypeError, ValueError) as error:
        raise InspectionActionConfigurationError(
            '{} must be a positive integer'.format(name)
        ) from error
    if number <= 0 or number != value:
        raise InspectionActionConfigurationError(
            '{} must be a positive integer'.format(name)
        )
    return number


def nonempty_text(value, name, error_type=InspectionActionProtocolError):
    """只接受非空字符串标识，避免数字或 bool 形成模糊 request_id。"""
    if not isinstance(value, str) or not value.strip():
        raise error_type('{} must be a non-empty string'.format(name))
    return value.strip()


def all_velocity_values_zero(values, epsilon):
    """确认最终 Twist 的所有分量有限且接近零，不只检查 linear.x。"""
    epsilon = finite_float(
        epsilon, 'final_zero_epsilon', nonnegative=True
    )
    try:
        numbers = tuple(float(value) for value in values)
    except (TypeError, ValueError):
        return False
    return bool(
        numbers
        and all(math.isfinite(value) and abs(value) <= epsilon
                for value in numbers)
    )


@dataclass(frozen=True)
class InspectionActionRequest:
    """路线节点发出的、可与结果精确对应的一次检查请求。"""

    run_id: str
    request_id: str
    action: str


def decode_inspection_request(payload):
    """严格解码 request JSON，拒绝缺失/未知 action 而不是猜测意图。"""
    try:
        decoded = json.loads(str(payload))
    except (TypeError, ValueError) as error:
        raise InspectionActionProtocolError(
            'inspection_request_invalid_json'
        ) from error
    if not isinstance(decoded, dict):
        raise InspectionActionProtocolError(
            'inspection_request_must_be_json_object'
        )
    request = InspectionActionRequest(
        run_id=nonempty_text(decoded.get('run_id'), 'run_id'),
        request_id=nonempty_text(decoded.get('request_id'), 'request_id'),
        action=nonempty_text(decoded.get('action'), 'action'),
    )
    if request.action != REQUEST_ACTION:
        raise InspectionActionProtocolError(
            'inspection_request_unsupported_action'
        )
    return request


@dataclass(frozen=True)
class InspectionActionConfig:
    """纯状态机的时间、置信度和连续确认安全边界。"""

    sign_confirm_frames: int = 5
    sign_min_confidence: float = 0.70
    sign_wait_timeout_sec: float = 8.0
    final_zero_epsilon: float = 0.001
    final_zero_confirm_samples: int = 3
    final_zero_timeout_sec: float = 3.0
    final_cmd_stale_timeout_sec: float = 0.50
    estop_state_stale_timeout_sec: float = 0.50

    def __post_init__(self):
        object.__setattr__(
            self,
            'sign_confirm_frames',
            positive_int(self.sign_confirm_frames, 'sign_confirm_frames'),
        )
        object.__setattr__(
            self,
            'sign_min_confidence',
            finite_float(
                self.sign_min_confidence,
                'sign_min_confidence',
                nonnegative=True,
            ),
        )
        if self.sign_min_confidence > 1.0:
            raise InspectionActionConfigurationError(
                'sign_min_confidence must be less than or equal to 1'
            )
        object.__setattr__(
            self,
            'sign_wait_timeout_sec',
            finite_float(
                self.sign_wait_timeout_sec,
                'sign_wait_timeout_sec',
                positive=True,
            ),
        )
        object.__setattr__(
            self,
            'final_zero_epsilon',
            finite_float(
                self.final_zero_epsilon,
                'final_zero_epsilon',
                nonnegative=True,
            ),
        )
        object.__setattr__(
            self,
            'final_zero_confirm_samples',
            positive_int(
                self.final_zero_confirm_samples,
                'final_zero_confirm_samples',
            ),
        )
        object.__setattr__(
            self,
            'final_zero_timeout_sec',
            finite_float(
                self.final_zero_timeout_sec,
                'final_zero_timeout_sec',
                positive=True,
            ),
        )
        object.__setattr__(
            self,
            'final_cmd_stale_timeout_sec',
            finite_float(
                self.final_cmd_stale_timeout_sec,
                'final_cmd_stale_timeout_sec',
                positive=True,
            ),
        )
        object.__setattr__(
            self,
            'estop_state_stale_timeout_sec',
            finite_float(
                self.estop_state_stale_timeout_sec,
                'estop_state_stale_timeout_sec',
                positive=True,
            ),
        )


@dataclass(frozen=True)
class InspectionActionEvent:
    """状态机交给 ROS 适配层的一次无副作用决策。"""

    state: str
    run_id: str
    request_id: str
    sign_type: str
    sign_value: str
    sdk_action: str
    confidence: float
    reason: str
    success: bool
    acquire_gait_lock: bool = False
    release_gait_lock: bool = False
    start_helper: bool = False
    terminate_helper: bool = False


class InspectionActionCore:
    """串行化一次 inspection 请求，所有异常路径都保持 fail-closed。"""

    def __init__(self, config=None):
        self.config = config or InspectionActionConfig()
        if not isinstance(self.config, InspectionActionConfig):
            raise InspectionActionConfigurationError(
                'config must be an InspectionActionConfig'
            )
        self.state = IDLE
        self.run_id = ''
        self.request_id = ''
        self.action = ''
        self.sign_type = ''
        self.sign_value = ''
        self.sdk_action = ''
        self.confidence = 0.0
        self.active = False
        self._lock_held = False
        self._helper_cleanup_pending = False
        self._pending_terminal_state = ''
        self._pending_terminal_reason = ''
        self._sign_deadline = None
        self._zero_deadline = None
        self._candidate_key = None
        self._candidate_count = 0
        self._final_zero_streak = 0
        self._last_final_cmd_time = None
        self._last_final_cmd_zero = False
        self._estop_active = None
        self._estop_receive_time = None

    @property
    def candidate_count(self):
        """暴露连续稳定计数，供单元测试和只读诊断使用。"""
        return self._candidate_count

    @property
    def final_zero_streak(self):
        """暴露在锁定后收到的连续最终零速度样本数。"""
        return self._final_zero_streak

    @property
    def gait_lock_held(self):
        """说明是否仍必须保持 lock，尤其是 cleanup 未验证的故障。"""
        return self._lock_held

    def request(self, request, now):
        """接收一个显式请求；重复请求不重启已运行或已完成的动作。"""
        now = finite_float(now, 'now', nonnegative=True)
        if not isinstance(request, InspectionActionRequest):
            raise InspectionActionProtocolError(
                'request must be an InspectionActionRequest'
            )
        self._validate_request(request)
        same_request = (
            request.run_id == self.run_id
            and request.request_id == self.request_id
        )
        if self.active:
            reason = (
                'duplicate_request_ignored'
                if same_request else 'request_ignored_while_active'
            )
            return self._event(reason)
        if self.state in TERMINAL_STATES and same_request:
            return self._event('duplicate_terminal_request_ignored')

        self._clear_request_state()
        self.run_id = request.run_id
        self.request_id = request.request_id
        self.action = request.action
        self.state = ARMED
        self.active = True
        return self._event('inspection_request_accepted')

    def arm(self, run_id, request_id, now):
        """开始观察当前请求之后的检测帧，阻断请求前的旧检测。"""
        now = finite_float(now, 'now', nonnegative=True)
        if not self._matches(run_id, request_id, ARMED):
            return None
        self.state = WAIT_SIGN
        self._sign_deadline = now + self.config.sign_wait_timeout_sec
        self._reset_sign_candidate()
        return self._event('waiting_for_stable_warning_sign')

    def observe_detection(self, sign_type, sign_value, confidence, now):
        """累计同一合格警示牌的连续帧，低置信/未知牌会清空计数。"""
        finite_float(now, 'now', nonnegative=True)
        if self.state != WAIT_SIGN or not self.active:
            return None
        try:
            confidence = float(confidence)
        except (TypeError, ValueError):
            confidence = float('nan')
        sdk_action = warning_action_for(sign_type, sign_value)
        if (
            sdk_action is None
            or not math.isfinite(confidence)
            or confidence < self.config.sign_min_confidence
        ):
            self._reset_sign_candidate()
            return None

        normalized_type = normalize_label(sign_type)
        normalized_value = normalize_label(sign_value)
        candidate_key = (normalized_type, normalized_value)
        if candidate_key == self._candidate_key:
            self._candidate_count += 1
        else:
            self._candidate_key = candidate_key
            self._candidate_count = 1
        self.sign_type = normalized_type
        self.sign_value = normalized_value
        self.sdk_action = sdk_action
        self.confidence = confidence
        if self._candidate_count < self.config.sign_confirm_frames:
            return None

        self.state = COMMAND_READY
        return self._event(
            'stable_warning_sign_confirmed', acquire_gait_lock=True
        )

    def lock_acquired(self, run_id, request_id, now):
        """仅在 gait lock 发布成功后开始统计最终速度归零样本。"""
        now = finite_float(now, 'now', nonnegative=True)
        if not self._matches(run_id, request_id, COMMAND_READY):
            return None
        self._lock_held = True
        self.state = WAIT_ZERO
        self._zero_deadline = now + self.config.final_zero_timeout_sec
        self._final_zero_streak = 0
        self._last_final_cmd_time = None
        self._last_final_cmd_zero = False
        return self._event('gait_lock_acquired_waiting_final_cmd_zero')

    def lock_failed(self, run_id, request_id, reason):
        """锁发布失败时不执行 SDK，并发出一次 best-effort 解锁请求。"""
        if not self._matches(run_id, request_id, COMMAND_READY):
            return None
        self._lock_held = True
        return self._finish(
            FAULTED,
            'gait_lock_acquire_failed:{}'.format(str(reason)),
        )

    def observe_final_cmd(self, is_zero, now):
        """处理最终 mux 输出；运行中重新出现非零速度会立即中止 helper。"""
        now = finite_float(now, 'now', nonnegative=True)
        if not self.active:
            return None
        if self.state == RUNNING and not bool(is_zero):
            return self._finish(
                FAULTED,
                'final_cmd_nonzero_during_sdk_action',
                terminate_helper=True,
            )
        if self.state != WAIT_ZERO:
            return None

        self._last_final_cmd_time = now
        self._last_final_cmd_zero = bool(is_zero)
        if not bool(is_zero):
            self._final_zero_streak = 0
            return None
        self._final_zero_streak += 1
        return self._maybe_start_helper(now)

    def observe_estop(self, active, now):
        """记录 typed estop 心跳；active estop 对任何活动请求立即 fail-closed。"""
        now = finite_float(now, 'now', nonnegative=True)
        self._estop_active = bool(active)
        self._estop_receive_time = now
        if self.active and self._estop_active:
            return self._finish(
                FAULTED,
                'estop_active',
                terminate_helper=self.state == RUNNING,
            )
        if self.state == WAIT_ZERO and self.active:
            return self._maybe_start_helper(now)
        return None

    def tick(self, now):
        """执行 sign/zero 超时，超时原因保留新鲜度和 estop 诊断。"""
        now = finite_float(now, 'now', nonnegative=True)
        if not self.active:
            return None
        if self.state == WAIT_SIGN and now >= self._sign_deadline:
            return self._finish(TIMEOUT, 'sign_wait_timeout')
        if self.state != WAIT_ZERO:
            return None
        if self._estop_active:
            return self._finish(FAULTED, 'estop_active')
        if now < self._zero_deadline:
            return None
        if self._estop_receive_time is None:
            reason = 'final_zero_timeout_estop_state_missing'
        elif not self._estop_is_fresh_false(now):
            reason = 'final_zero_timeout_estop_state_stale'
        elif self._last_final_cmd_time is None:
            reason = 'final_zero_timeout_final_cmd_missing'
        elif (
            now - self._last_final_cmd_time
            > self.config.final_cmd_stale_timeout_sec
        ):
            reason = 'final_zero_timeout_final_cmd_stale'
        else:
            reason = 'final_zero_timeout'
        return self._finish(TIMEOUT, reason)

    def helper_launching(self, run_id, request_id):
        """标记 helper 已交给线程，之后 stop 必须等进程组清理再解锁。"""
        if not self._matches(run_id, request_id, RUNNING):
            return False
        self._helper_cleanup_pending = True
        return True

    def helper_finished(
        self,
        run_id,
        request_id,
        terminal_state,
        reason,
        *,
        cleanup_completed,
    ):
        """在 helper 已 reap 后完成状态；cleanup 未验证时刻意保持 gait lock。"""
        if not self._identity_matches(run_id, request_id):
            return None
        if not cleanup_completed:
            self._helper_cleanup_pending = False
            self._pending_terminal_state = ''
            self._pending_terminal_reason = ''
            self.state = FAULTED
            self.active = False
            return self._event(
                '{};helper_cleanup_unverified_lock_held'.format(reason)
            )
        self._helper_cleanup_pending = False
        pending_state = self._pending_terminal_state
        pending_reason = self._pending_terminal_reason
        self._pending_terminal_state = ''
        self._pending_terminal_reason = ''
        if pending_state:
            # stop/estop 已在 earlier event 请求回收；现在 runner 已完成
            # TERM/KILL/wait，才允许以原来的安全终态释放 gait lock。
            return self._finish(
                pending_state,
                '{};{}'.format(pending_reason, reason),
            )
        if terminal_state not in TERMINAL_STATES:
            terminal_state = FAILED
            reason = 'helper_invalid_terminal_state'
        return self._finish(terminal_state, reason)

    def mission_stop(self):
        """取消活动请求；helper 正在运行时先终止并等待进程组清理。"""
        if not self.active:
            return self._event('mission_stop_no_active_inspection')
        return self._finish(
            CANCELED,
            'mission_stop_cancel_requested',
            terminate_helper=self._helper_cleanup_pending,
        )

    def _maybe_start_helper(self, now):
        if self.state != WAIT_ZERO or not self.active:
            return None
        if self._final_zero_streak < self.config.final_zero_confirm_samples:
            return None
        if self._last_final_cmd_time is None or not self._last_final_cmd_zero:
            return None
        if (
            now - self._last_final_cmd_time
            > self.config.final_cmd_stale_timeout_sec
        ):
            return None
        if not self._estop_is_fresh_false(now):
            return None
        self.state = RUNNING
        return self._event('final_cmd_zero_and_estop_clear', start_helper=True)

    def _estop_is_fresh_false(self, now):
        return bool(
            self._estop_active is False
            and self._estop_receive_time is not None
            and now - self._estop_receive_time
            <= self.config.estop_state_stale_timeout_sec
        )

    def _finish(self, state, reason, terminate_helper=False):
        if self._helper_cleanup_pending:
            # 先保留请求的终态和 lock，等待 helper 线程完成进程组回收；此状态
            # 仍被 readiness/mission_stop 视为 active，避免假阳性的“已停止”。
            self._pending_terminal_state = state
            self._pending_terminal_reason = str(reason)
            self.state = CLEANUP_PENDING
            self.active = True
            return self._event(
                '{};helper_cleanup_pending'.format(reason),
                terminate_helper=bool(terminate_helper),
            )
        self.state = state
        self.active = False
        # helper 线程拥有进程组；它报告 reap 完成前不能解除控制权锁。
        wait_for_cleanup = self._helper_cleanup_pending
        release_gait_lock = self._lock_held and not wait_for_cleanup
        if release_gait_lock:
            self._lock_held = False
        return self._event(
            reason,
            release_gait_lock=release_gait_lock,
            terminate_helper=bool(terminate_helper and wait_for_cleanup),
        )

    def _event(
        self,
        reason,
        *,
        acquire_gait_lock=False,
        release_gait_lock=False,
        start_helper=False,
        terminate_helper=False,
    ):
        return InspectionActionEvent(
            state=self.state,
            run_id=self.run_id,
            request_id=self.request_id,
            sign_type=self.sign_type,
            sign_value=self.sign_value,
            sdk_action=self.sdk_action,
            confidence=self.confidence,
            reason=str(reason),
            success=self.state == SUCCEEDED,
            acquire_gait_lock=bool(acquire_gait_lock),
            release_gait_lock=bool(release_gait_lock),
            start_helper=bool(start_helper),
            terminate_helper=bool(terminate_helper),
        )

    def _matches(self, run_id, request_id, required_state):
        return bool(
            self.active
            and self.state == required_state
            and self._identity_matches(run_id, request_id)
        )

    def _identity_matches(self, run_id, request_id):
        return bool(
            str(run_id) == self.run_id
            and str(request_id) == self.request_id
        )

    def _validate_request(self, request):
        nonempty_text(request.run_id, 'run_id')
        nonempty_text(request.request_id, 'request_id')
        if request.action != REQUEST_ACTION:
            raise InspectionActionProtocolError(
                'inspection_request_unsupported_action'
            )

    def _clear_request_state(self):
        self.sign_type = ''
        self.sign_value = ''
        self.sdk_action = ''
        self.confidence = 0.0
        self._lock_held = False
        self._helper_cleanup_pending = False
        self._pending_terminal_state = ''
        self._pending_terminal_reason = ''
        self._sign_deadline = None
        self._zero_deadline = None
        self._reset_sign_candidate()
        self._final_zero_streak = 0
        self._last_final_cmd_time = None
        self._last_final_cmd_zero = False

    def _reset_sign_candidate(self):
        self._candidate_key = None
        self._candidate_count = 0


@dataclass(frozen=True)
class HelperRunResult:
    """helper 线程可直接转换为核心终态的受监督执行结果。"""

    terminal_state: str
    reason: str
    return_code: object
    cleanup_completed: bool


class ProcessGroupHelperRunner:
    """以独立 session 运行 helper，并在取消/超时时回收整个进程组。"""

    def __init__(
        self,
        *,
        poll_interval_sec=0.05,
        terminate_grace_sec=0.50,
        kill_grace_sec=0.50,
        clock=None,
        sleeper=None,
        popen_factory=None,
    ):
        self.poll_interval_sec = finite_float(
            poll_interval_sec, 'poll_interval_sec', positive=True
        )
        self.terminate_grace_sec = finite_float(
            terminate_grace_sec, 'terminate_grace_sec', positive=True
        )
        self.kill_grace_sec = finite_float(
            kill_grace_sec, 'kill_grace_sec', positive=True
        )
        self._clock = clock or time.monotonic
        self._sleeper = sleeper or time.sleep
        self._popen_factory = popen_factory or subprocess.Popen

    def run(
        self, argv, timeout_sec, cancel_event, *, environment=None,
        expected_executable=None,
    ):
        """运行固定 argv；timeout/cancel 均先 SIGTERM、后 SIGKILL、最后 wait。"""
        timeout_sec = finite_float(
            timeout_sec, 'sdk_action_timeout_sec', positive=True
        )
        if not isinstance(argv, (list, tuple)) or not argv:
            return HelperRunResult(
                FAILED, 'helper_argv_invalid', None, True
            )
        command = [str(value) for value in argv]
        try:
            process = self._popen_factory(
                command,
                shell=False,
                stdin=subprocess.DEVNULL,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                start_new_session=True,
                close_fds=True,
                env=environment,
            )
        except Exception as error:
            return HelperRunResult(
                FAILED,
                'helper_start_failed:{}'.format(type(error).__name__),
                None,
                True,
            )

        identity = self._capture_identity(
            process, expected_executable or command[0]
        )
        if identity is False:
            self._best_effort_leader_cleanup(process)
            return HelperRunResult(
                FAULTED,
                'helper_process_identity_unverified',
                self._safe_return_code(process),
                False,
            )

        pgid, session_id = identity
        deadline = self._clock() + timeout_sec
        while True:
            if cancel_event is not None and cancel_event.is_set():
                cleanup = self._cleanup_process_group(
                    process, pgid, session_id
                )
                return HelperRunResult(
                    CANCELED,
                    'helper_cancel_requested',
                    self._safe_return_code(process),
                    cleanup,
                )
            return_code = self._safe_return_code(process)
            if return_code is not None:
                cleanup = self._cleanup_process_group(
                    process, pgid, session_id
                )
                if not cleanup:
                    return HelperRunResult(
                        FAULTED,
                        'helper_process_group_cleanup_failed',
                        return_code,
                        False,
                    )
                if return_code == 0:
                    return HelperRunResult(
                        SUCCEEDED,
                        'sdk_helper_returned_zero',
                        return_code,
                        True,
                    )
                return HelperRunResult(
                    FAILED,
                    'sdk_helper_returned_nonzero:{}'.format(return_code),
                    return_code,
                    True,
                )
            if self._clock() >= deadline:
                cleanup = self._cleanup_process_group(
                    process, pgid, session_id
                )
                return HelperRunResult(
                    TIMEOUT,
                    'sdk_helper_timeout',
                    self._safe_return_code(process),
                    cleanup,
                )
            self._sleeper(self.poll_interval_sec)

    def _capture_identity(self, process, executable):
        """只向自己的 session/process group 发信号，拒绝不明 PGID。"""
        if self._safe_return_code(process) is not None:
            # start_new_session 保证刚创建的组以 child PID 为 session/PGID。
            # 即使 leader 极快退出，仍须扫描并清理可能留下的同组子孙进程。
            return int(process.pid), int(process.pid)
        try:
            pid = int(process.pid)
            pgid = os.getpgid(pid)
            session_id = os.getsid(pid)
            observed_executable = os.path.realpath('/proc/{}/exe'.format(pid))
            expected_executable = os.path.realpath(executable)
            if (
                pgid != pid
                or session_id != pid
                or observed_executable != expected_executable
            ):
                return False
            return pgid, session_id
        except (OSError, TypeError, ValueError):
            if self._safe_return_code(process) is not None:
                return int(process.pid), int(process.pid)
            return False

    def _cleanup_process_group(self, process, pgid, session_id):
        """按 TERM -> 等待 -> KILL -> 等待顺序清理 helper 及其子孙进程。"""
        if not self._identity_is_safe(process, pgid, session_id):
            return False
        if not self._group_members(pgid, session_id):
            self._reap_process(process)
            return True
        if not self._signal_group(pgid, signal.SIGTERM):
            return False
        if self._wait_group_empty(pgid, session_id, self.terminate_grace_sec):
            self._reap_process(process)
            return True
        if not self._signal_group(pgid, signal.SIGKILL):
            return False
        cleaned = self._wait_group_empty(
            pgid, session_id, self.kill_grace_sec
        )
        self._reap_process(process)
        return cleaned

    def _identity_is_safe(self, process, pgid, session_id):
        """在 helper 存活时复核 PID/session/exe，避免误杀已复用 PID。"""
        if self._safe_return_code(process) is None:
            try:
                pid = int(process.pid)
                return bool(
                    os.getpgid(pid) == pgid
                    and os.getsid(pid) == session_id
                    and pgid == pid
                    and session_id == pid
                )
            except (OSError, TypeError, ValueError):
                return False
        # leader 已退出时，只有同一 session 的原进程组成员才允许被清理。
        return True

    def _group_members(self, pgid, session_id):
        """从 /proc 精确找同一 session 的成员，用于确认进程组真的为空。"""
        members = []
        try:
            entries = list(Path('/proc').iterdir())
        except OSError:
            return None
        for entry in entries:
            if not entry.name.isdigit():
                continue
            try:
                raw = (entry / 'stat').read_text(encoding='ascii')
                close_index = raw.rfind(')')
                fields = raw[close_index + 2:].split()
                if close_index < 0 or len(fields) < 4:
                    continue
                state = fields[0]
                member_pgid = int(fields[2])
                member_session = int(fields[3])
            except (OSError, ValueError, UnicodeError):
                continue
            if (
                member_pgid == pgid
                and member_session == session_id
                and state != 'Z'
            ):
                members.append(int(entry.name))
        return members

    def _wait_group_empty(self, pgid, session_id, timeout_sec):
        deadline = self._clock() + timeout_sec
        while True:
            members = self._group_members(pgid, session_id)
            if members == []:
                return True
            if members is None or self._clock() >= deadline:
                return False
            self._sleeper(self.poll_interval_sec)

    @staticmethod
    def _signal_group(pgid, signal_number):
        try:
            os.killpg(pgid, signal_number)
            return True
        except ProcessLookupError:
            return True
        except (OSError, TypeError, ValueError):
            return False

    @staticmethod
    def _safe_return_code(process):
        try:
            return process.poll()
        except Exception:
            return None

    @staticmethod
    def _reap_process(process):
        try:
            process.wait(timeout=0.20)
        except Exception:
            return False
        return True

    @staticmethod
    def _best_effort_leader_cleanup(process):
        """身份校验失败时绝不 killpg，只有限度回收直接子进程。"""
        try:
            if process.poll() is None:
                process.terminate()
            process.wait(timeout=0.20)
        except Exception:
            try:
                if process.poll() is None:
                    process.kill()
                process.wait(timeout=0.20)
            except Exception:
                pass
