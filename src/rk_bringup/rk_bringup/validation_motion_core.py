"""动态验收轻量运动 adapter 的纯状态机与安全判定。

本模块不依赖 ROS，也不执行文件 I/O。ROS callback 只把最新状态写入本状态机，
20 Hz timer 再统一做 freshness、executor lateness 和运动窗口判定。
"""

from dataclasses import dataclass
import math
from typing import Optional


SILENT = 'SILENT'
MOVE = 'MOVE'
ZERO = 'ZERO'

LINE_SOURCE_STALE = 'LINE_SOURCE_STALE'
LINE_SOURCE_INVALID = 'LINE_SOURCE_INVALID'
SUGGESTED_CMD_STALE = 'SUGGESTED_CMD_STALE'
SUGGESTED_CMD_INVALID = 'SUGGESTED_CMD_INVALID'
GAIT_LOCKED = 'GAIT_LOCKED'
SDK_ERROR = 'SDK_ERROR'
MUX_SOURCE_INVALID = 'MUX_SOURCE_INVALID'
ADAPTER_EXECUTOR_STALL = 'ADAPTER_EXECUTOR_STALL'
MOTION_WINDOW_COMPLETE = 'MOTION_WINDOW_COMPLETE'
OPERATOR_STOP = 'OPERATOR_STOP'
HARD_WATCHDOG_LIMIT = 'HARD_WATCHDOG_LIMIT'


@dataclass(frozen=True)
class AdapterDecision:
    """一次 timer 判定；SILENT 表示 arm 前不向 line topic 写入任何数据。"""

    action: str
    reason: str
    linear_x: float = 0.0
    linear_y: float = 0.0
    angular_z: float = 0.0


class ValidationMotionCore:
    """仅保存 latest-state 的一次性动态验收 fail-closed 状态机。"""

    def __init__(
        self,
        *,
        line_timeout_ns=350_000_000,
        candidate_timeout_ns=250_000_000,
        gait_timeout_ns=350_000_000,
        motion_ns=1_000_000_000,
        watchdog_ns=1_100_000_000,
        executor_lateness_ns=100_000_000,
        max_vx=0.25,
        max_yaw=0.8,
        max_lateral_error=0.8,
    ):
        self.line_timeout_ns = int(line_timeout_ns)
        self.candidate_timeout_ns = int(candidate_timeout_ns)
        self.gait_timeout_ns = int(gait_timeout_ns)
        self.motion_ns = int(motion_ns)
        self.watchdog_ns = int(watchdog_ns)
        self.executor_lateness_ns = int(executor_lateness_ns)
        self.max_vx = float(max_vx)
        self.max_yaw = float(max_yaw)
        self.max_lateral_error = float(max_lateral_error)

        self.line_receive_ns: Optional[int] = None
        self.line_source_sec: Optional[int] = None
        self.line_source_nanosec: Optional[int] = None
        self.line_visible = False
        self.line_lateral = 0.0
        self.line_heading = 0.0
        self.candidate_receive_ns: Optional[int] = None
        self.candidate_vx = 0.0
        self.candidate_yaw = 0.0
        self.gait_receive_ns: Optional[int] = None
        self.gait_locked = True
        self.mux_receive_ns: Optional[int] = None
        self.mux_source: Optional[str] = None
        self.sdk_t0_ns: Optional[int] = None
        self.first_nonzero_ns: Optional[int] = None
        self.armed = False
        self.output_enabled = False
        self.stop_reason: Optional[str] = None
        self.max_timer_lateness_ns = 0

    @staticmethod
    def _finite(value):
        return isinstance(value, (float, int)) and math.isfinite(float(value))

    def observe_line(
        self, receive_ns, *, visible, lateral, heading,
        source_sec=None, source_nanosec=None,
    ):
        """保存最新 LineTrack；source stamp 只用于状态证据，不参与实时相减。"""
        self.line_receive_ns = int(receive_ns)
        self.line_visible = bool(visible)
        self.line_lateral = float(lateral)
        self.line_heading = float(heading)
        self.line_source_sec = source_sec
        self.line_source_nanosec = source_nanosec

    def observe_candidate(self, receive_ns, *, linear_x, angular_z):
        """保存 follower 最新建议，不复制历史队列。"""
        self.candidate_receive_ns = int(receive_ns)
        self.candidate_vx = float(linear_x)
        self.candidate_yaw = float(angular_z)

    def observe_gait_lock(self, receive_ns, locked):
        """gait lock 为 true 或其独立心跳过期都保持 fail-closed。"""
        self.gait_receive_ns = int(receive_ns)
        self.gait_locked = bool(locked)

    def observe_mux(self, receive_ns, source):
        self.mux_receive_ns = int(receive_ns)
        self.mux_source = str(source)

    def observe_sdk(self, event, ret, event_ns):
        """只消费运动安全所需的 ACK/error，不做统计或历史保存。"""
        event = str(event)
        ret = int(ret)
        if event == 'SDK_ERROR' or (event == 'MOVE' and ret != 0):
            self._stop(SDK_ERROR)
        elif (
            self.armed
            and event == 'MOVE'
            and ret == 0
            and self.sdk_t0_ns is None
        ):
            self.sdk_t0_ns = int(event_ns)

    def request_arm(self, now_ns):
        """通过当前 latest-state 做一次 arm 门；失败时保持 topic 沉默。"""
        reason = self._input_failure(int(now_ns))
        if self.stop_reason is not None:
            return False, self.stop_reason
        if reason is not None:
            return False, reason
        self.armed = True
        self.output_enabled = False
        return True, 'ARMING'

    def enable_output(self):
        """仅在 estop service 明确解除后允许 timer 发布首条非零命令。"""
        if self.armed and self.stop_reason is None:
            self.output_enabled = True

    def operator_stop(self):
        self._stop(OPERATOR_STOP)

    def _stop(self, reason):
        if self.stop_reason is None:
            self.stop_reason = str(reason)
        self.output_enabled = False

    def _input_failure(self, now_ns):
        if (
            self.line_receive_ns is None
            or now_ns - self.line_receive_ns > self.line_timeout_ns
        ):
            return LINE_SOURCE_STALE
        if (
            not self.line_visible
            or not self._finite(self.line_lateral)
            or abs(self.line_lateral) > self.max_lateral_error
            or not self._finite(self.line_heading)
        ):
            return LINE_SOURCE_INVALID
        if (
            self.candidate_receive_ns is None
            or now_ns - self.candidate_receive_ns > self.candidate_timeout_ns
        ):
            return SUGGESTED_CMD_STALE
        if (
            not self._finite(self.candidate_vx)
            or self.candidate_vx <= 0.0
            or not self._finite(self.candidate_yaw)
            or abs(self.candidate_yaw) > self.max_yaw
        ):
            return SUGGESTED_CMD_INVALID
        if (
            self.gait_receive_ns is None
            or now_ns - self.gait_receive_ns > self.gait_timeout_ns
            or self.gait_locked
        ):
            return GAIT_LOCKED
        return None

    def tick(self, actual_ns, expected_ns):
        """执行 O(1) 安全判定；timer 自身迟到与感知断流必须独立分类。"""
        actual_ns = int(actual_ns)
        lateness_ns = max(0, actual_ns - int(expected_ns))
        self.max_timer_lateness_ns = max(
            self.max_timer_lateness_ns, lateness_ns,
        )
        if lateness_ns > self.executor_lateness_ns:
            self._stop(ADAPTER_EXECUTOR_STALL)

        if self.stop_reason is not None:
            # pre-arm 本来就没有运动，保持 topic 沉默；故障只阻止后续 arm。
            if not self.armed:
                return AdapterDecision(SILENT, self.stop_reason)
            return AdapterDecision(ZERO, self.stop_reason)
        if not self.armed or not self.output_enabled:
            reason = 'PREARM' if not self.armed else 'ARMING'
            return AdapterDecision(SILENT, reason)

        failure = self._input_failure(actual_ns)
        if failure is not None:
            self._stop(failure)
            return AdapterDecision(ZERO, self.stop_reason)
        if (
            self.sdk_t0_ns is not None
            and self.mux_source not in (None, 'line')
        ):
            self._stop(MUX_SOURCE_INVALID)
            return AdapterDecision(ZERO, self.stop_reason)

        reference_ns = self.sdk_t0_ns or self.first_nonzero_ns
        if (
            self.sdk_t0_ns is not None
            and actual_ns - self.sdk_t0_ns >= self.motion_ns
        ):
            self._stop(MOTION_WINDOW_COMPLETE)
            return AdapterDecision(ZERO, self.stop_reason)
        if (
            reference_ns is not None
            and actual_ns - reference_ns >= self.watchdog_ns
        ):
            self._stop(HARD_WATCHDOG_LIMIT)
            return AdapterDecision(ZERO, self.stop_reason)

        if self.first_nonzero_ns is None:
            self.first_nonzero_ns = actual_ns
        return AdapterDecision(
            MOVE,
            'ARMED_FORWARD',
            min(self.candidate_vx, self.max_vx),
            0.0,
            self.candidate_yaw,
        )

    def status(self, now_ns, timer_lateness_ns):
        """生成小型状态快照；不包含历史数组或统计序列化。"""
        return {
            'monotonic_ns': int(now_ns),
            'armed': bool(self.armed),
            'output_enabled': bool(self.output_enabled),
            'stop_reason': self.stop_reason,
            'timer_lateness_ms': max(0, int(timer_lateness_ns)) / 1e6,
            'max_timer_lateness_ms': self.max_timer_lateness_ns / 1e6,
            'line_receiver_monotonic_ns': self.line_receive_ns,
            'line_source_header_sec': self.line_source_sec,
            'line_source_header_nanosec': self.line_source_nanosec,
            'line_source_sequence': None,
        }
