"""START/FINISH 共用的白横线丢失与盲行控制核心。"""

from dataclasses import dataclass
import math


WHITE_BAR_STAGES = frozenset(('START', 'FINISH'))
MOTION_BY_STAGE = {
    'START': 'start_jump',
    'FINISH': 'finish_jump',
}

FOLLOW = 'FOLLOW'
ZERO = 'ZERO'
REQUEST_ACTION = 'REQUEST_ACTION'

RECOVERY_IDLE = 'IDLE'
RECOVERY_FORWARD = 'FORWARD'
RECOVERY_WAITING = 'RECOVERING'
RECOVERY_COMPLETE = 'COMPLETE'
RECOVERY_FAULTED = 'FAULTED'


@dataclass(frozen=True)
class WhiteBarBlindDecision:
    """一次控制周期的纯数据决策，不直接发布 ROS 命令或动作。"""

    action: str
    reason: str
    motion_name: str = ''
    linear_x: float = 0.0
    angular_z: float = 0.0


class WhiteBarBlindApproachCore:
    """以 stable seen→stable lost→blind forward 驱动共用阶段。

    每次显式 ARM 都清空上一阶段的检测历史。检测和重捕 streak 只由新鲜的
    新消息推进；控制 timer 只能读取结果，不能把同一帧重复计数。blind
    期间短时丢线立即零速并冻结有效前进计时；只有超过有界恢复窗口才锁存
    FAULTED，其他 gait/mux 安全门仍保持立即故障关闭。
    """

    def __init__(
        self, *, confirm_frames, min_confidence,
        blind_duration_sec, detection_timeout_sec,
        line_recovery_timeout_sec, line_recovery_confirm_frames,
    ):
        if type(confirm_frames) is not int or confirm_frames < 1:
            raise ValueError('confirm_frames must be an integer >= 1')
        values = (
            float(min_confidence), float(blind_duration_sec),
            float(detection_timeout_sec),
            float(line_recovery_timeout_sec),
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError('white-bar timing/confidence must be finite')
        if (
            values[0] < 0.0 or values[1] < 0.0
            or values[2] <= 0.0 or values[3] <= 0.0
        ):
            raise ValueError('invalid white-bar timing/confidence range')
        if (
            type(line_recovery_confirm_frames) is not int
            or line_recovery_confirm_frames < 1
        ):
            raise ValueError(
                'line_recovery_confirm_frames must be an integer >= 1'
            )
        self.confirm_frames = confirm_frames
        self.min_confidence = values[0]
        self.blind_duration_sec = values[1]
        self.detection_timeout_ns = int(values[2] * 1e9)
        self.line_recovery_timeout_ns = int(values[3] * 1e9)
        self.line_recovery_confirm_frames = line_recovery_confirm_frames
        self.reset()

    def reset(self):
        """清除 stage 及全部检测证据；新 ARM 不复用历史帧。"""
        self.stage = ''
        self.arm_monotonic_ns = None
        self.last_detection_monotonic_ns = None
        self.seen_latched = False
        self.seen_count = 0
        self.lost_count = 0
        self.lost_confirmed = False
        self.lost_monotonic_ns = None
        self.center_y = None
        self.visible = False
        self.confidence = 0.0
        self.fault_reason = ''
        self.blind_active_forward_ns = 0
        self.blind_last_control_ns = None
        self.blind_last_candidate_forward = False
        self.blind_recovery_count = 0
        self.blind_recovery_state = RECOVERY_IDLE
        self.blind_recovery_started_ns = None
        self.blind_recovery_confirm_count = 0
        self.blind_last_line_valid = False

    def arm(self, stage, now_ns):
        """显式 ARM 一个 START/FINISH stage，并重新开始白线观察。"""
        normalized = str(stage).strip().upper()
        if normalized not in WHITE_BAR_STAGES:
            raise ValueError('white-bar stage must be START or FINISH')
        self.reset()
        self.stage = normalized
        self.arm_monotonic_ns = int(now_ns)

    def observe_detection(
        self, now_ns, *, visible, confidence, center_y,
    ):
        """只用一次新 detection 推进 seen/lost streak。

        ``visible=false`` 才属于 lost 候选；低置信但仍 visible 的帧既不能
        建立 seen，也不能伪装成 lost。center_y 只保存作证据，永不参与决策。
        """
        if not self.stage or self.fault_reason:
            return
        now_ns = int(now_ns)
        confidence = self._finite_or_none(confidence)
        center_y = self._finite_or_none(center_y)
        self.last_detection_monotonic_ns = now_ns
        self.visible = bool(visible)
        self.confidence = confidence if confidence is not None else 0.0
        self.center_y = center_y
        if self.lost_confirmed:
            return

        stable_visible = (
            self.visible
            and confidence is not None
            and confidence >= self.min_confidence
        )
        if stable_visible:
            self.seen_count = min(
                self.confirm_frames, self.seen_count + 1,
            )
            self.lost_count = 0
            if self.seen_count >= self.confirm_frames:
                self.seen_latched = True
            return

        self.seen_count = 0
        if not self.visible and self.seen_latched:
            self.lost_count += 1
            if self.lost_count >= self.confirm_frames:
                self.lost_count = self.confirm_frames
                self.lost_confirmed = True
                self.lost_monotonic_ns = now_ns
                self.blind_last_control_ns = now_ns
                self.blind_recovery_state = RECOVERY_FORWARD
            return
        self.lost_count = 0

    def observe_line(self, now_ns, *, valid):
        """用真正的新 LineTrack 帧推进重捕确认。

        invalid 帧会在 blind 期间立即进入零速恢复；连续有效帧只负责
        累计证据，最终放行还要在控制周期同时验证 suggested/follower。
        """
        now_ns = int(now_ns)
        self.blind_last_line_valid = bool(valid)
        if not self.lost_confirmed or self.fault_reason:
            return
        if not valid:
            # invalid 帧到达前的上一条正速候选确实已被持续执行，
            # 因此先结算该区间，再冻结计时并进入零速恢复。
            self._accrue_previous_forward_interval(now_ns)
            self._enter_line_recovery(now_ns)
            self.blind_recovery_confirm_count = 0
            return
        if self.blind_recovery_state == RECOVERY_WAITING:
            self.blind_recovery_confirm_count = min(
                self.line_recovery_confirm_frames,
                self.blind_recovery_confirm_count + 1,
            )

    def evaluate(
        self, now_ns, *, line_valid, line_fresh, suggested_fresh,
        suggested_vx, suggested_wz, follower_ready, follower_fresh,
        gait_locked, gait_fresh,
        mux_healthy, mux_fresh, mux_source_valid,
    ):
        """应用统一安全门，并返回跟随、零速或动作请求决策。"""
        now_ns = int(now_ns)
        if not self.stage:
            return WhiteBarBlindDecision(ZERO, 'WHITE_BAR_STAGE_NOT_ARMED')
        if self.fault_reason:
            return WhiteBarBlindDecision(ZERO, self.fault_reason)

        detection_fresh = self._detection_fresh(now_ns)
        if detection_fresh is False:
            return self._fault('WHITE_BAR_DETECTION_STALE')
        suggested_vx = self._finite_or_none(suggested_vx)
        suggested_wz = self._finite_or_none(suggested_wz)
        if not gait_fresh or gait_locked:
            return self._fault('WHITE_BAR_GAIT_LOCKED_OR_STALE')
        if not mux_fresh or not mux_healthy:
            return self._fault('WHITE_BAR_MUX_UNHEALTHY_OR_STALE')
        if self.lost_confirmed and not mux_source_valid:
            return self._fault('WHITE_BAR_MUX_SOURCE_INVALID')

        if not self.lost_confirmed:
            if not line_fresh:
                return self._fault('WHITE_BAR_LINE_TRACK_STALE')
            if not line_valid:
                return self._fault('WHITE_BAR_LINE_TRACK_INVALID')
            if not suggested_fresh:
                return self._fault('WHITE_BAR_SUGGESTED_CMD_STALE')
            if suggested_vx is None or suggested_wz is None:
                return self._fault('WHITE_BAR_SUGGESTED_CMD_INVALID')
            if not follower_fresh or not follower_ready:
                return self._fault('WHITE_BAR_LINE_FOLLOWER_NOT_READY')

            reason = (
                'WHITE_BAR_SEEN_LATCHED'
                if self.seen_latched else 'WHITE_BAR_WAITING_STABLE_SEEN'
            )
            return WhiteBarBlindDecision(
                FOLLOW, reason,
                linear_x=suggested_vx,
                angular_z=suggested_wz,
            )

        if self.blind_duration_sec <= 0.0:
            return self._fault(
                'WHITE_BAR_BLIND_DURATION_NOT_CALIBRATED'
            )
        self._accrue_previous_forward_interval(now_ns)

        line_safe = bool(line_fresh and line_valid)
        suggested_safe = (
            bool(suggested_fresh)
            and suggested_vx is not None
            and suggested_wz is not None
            and suggested_vx > 0.0
        )
        follower_safe = bool(follower_fresh and follower_ready)
        if not line_safe or not suggested_safe or not follower_safe:
            self._enter_line_recovery(now_ns)

        if self.blind_recovery_state == RECOVERY_WAITING:
            self.blind_last_candidate_forward = False
            if self._line_recovery_timed_out(now_ns):
                return self._fault('WHITE_BAR_LINE_RECOVERY_TIMEOUT')
            recovered = (
                line_safe
                and suggested_safe
                and follower_safe
                and self.blind_recovery_confirm_count
                >= self.line_recovery_confirm_frames
            )
            if not recovered:
                return WhiteBarBlindDecision(
                    ZERO, 'WHITE_BAR_BLIND_LINE_RECOVERY'
                )
            self.blind_recovery_state = RECOVERY_FORWARD
            self.blind_recovery_started_ns = None
            self.blind_recovery_confirm_count = 0

        required_ns = int(self.blind_duration_sec * 1e9)
        if self.blind_active_forward_ns >= required_ns:
            self.blind_last_candidate_forward = False
            self.blind_recovery_state = RECOVERY_COMPLETE
            return WhiteBarBlindDecision(
                REQUEST_ACTION,
                'WHITE_BAR_BLIND_FORWARD_COMPLETE',
                motion_name=MOTION_BY_STAGE[self.stage],
            )
        self.blind_last_candidate_forward = True
        return WhiteBarBlindDecision(
            FOLLOW, 'WHITE_BAR_BLIND_FORWARD',
            linear_x=suggested_vx,
            angular_z=suggested_wz,
        )

    def snapshot(self, now_ns):
        """生成 START/FINISH 完全同构的状态字段。"""
        wall_elapsed = 0.0
        if self.lost_monotonic_ns is not None:
            wall_elapsed = max(
                0.0, (int(now_ns) - self.lost_monotonic_ns) / 1e9,
            )
        active_forward = self.blind_active_forward_ns / 1e9
        remaining = max(0.0, self.blind_duration_sec - active_forward)
        return {
            'white_bar_stage': self.stage,
            'white_bar_seen_latched': bool(self.seen_latched),
            'white_bar_seen_count': int(self.seen_count),
            'white_bar_lost_count': int(self.lost_count),
            'white_bar_lost_confirmed': bool(self.lost_confirmed),
            'white_bar_lost_monotonic_ns': self.lost_monotonic_ns,
            'white_bar_blind_duration_sec': self.blind_duration_sec,
            # 兼容字段跟随新参数语义：表示累计有效前进时间。
            'white_bar_blind_elapsed_sec': active_forward,
            'white_bar_blind_active_forward_sec': active_forward,
            'white_bar_blind_wall_elapsed_sec': wall_elapsed,
            'white_bar_blind_remaining_sec': remaining,
            'white_bar_blind_recovery_count': self.blind_recovery_count,
            'white_bar_blind_recovery_state': self.blind_recovery_state,
            'white_bar_blind_last_line_valid': bool(
                self.blind_last_line_valid
            ),
            'white_bar_center_y': self.center_y,
            'white_bar_visible': bool(self.visible),
            'white_bar_confidence': float(self.confidence),
        }

    def _detection_fresh(self, now_ns):
        reference = self.last_detection_monotonic_ns
        if reference is None:
            # ARM 后给第一条新帧一个 detection timeout 的有界发现窗口；
            # 超过窗口仍无帧才故障，且从不把“未见”解释成“已丢失”。
            reference = self.arm_monotonic_ns
        if reference is None:
            return False
        age_ns = int(now_ns) - int(reference)
        return 0 <= age_ns <= self.detection_timeout_ns

    def _fault(self, reason):
        self.fault_reason = str(reason)
        self.blind_last_candidate_forward = False
        self.blind_recovery_state = RECOVERY_FAULTED
        return WhiteBarBlindDecision(ZERO, self.fault_reason)

    def _accrue_previous_forward_interval(self, now_ns):
        """按 mission candidate 的实际零阶保持区间累计前进时间。"""
        if self.blind_last_control_ns is not None:
            interval_ns = max(0, now_ns - self.blind_last_control_ns)
            if self.blind_last_candidate_forward:
                self.blind_active_forward_ns += interval_ns
        self.blind_last_control_ns = now_ns

    def _enter_line_recovery(self, now_ns):
        """首次异常帧进入有界恢复，重复异常不重置超时时钟。"""
        if self.blind_recovery_state != RECOVERY_WAITING:
            self.blind_recovery_state = RECOVERY_WAITING
            self.blind_recovery_started_ns = int(now_ns)
            self.blind_recovery_count += 1
            self.blind_recovery_confirm_count = 0
        self.blind_last_candidate_forward = False

    def _line_recovery_timed_out(self, now_ns):
        """恢复窗口从第一次异常起算，不被后续帧延长。"""
        if self.blind_recovery_started_ns is None:
            return False
        return (
            int(now_ns) - self.blind_recovery_started_ns
            > self.line_recovery_timeout_ns
        )

    @staticmethod
    def _finite_or_none(value):
        try:
            result = float(value)
        except (TypeError, ValueError):
            return None
        return result if math.isfinite(result) else None
