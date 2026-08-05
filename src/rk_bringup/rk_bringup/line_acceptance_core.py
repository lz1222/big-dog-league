"""全图巡线专项验收的纯逻辑安全门禁."""

from dataclasses import dataclass


LINE_TEST_READY = 'LINE_TEST_READY'
DISARM = 'DISARM'
SEGMENT_READY_PREFIX = 'SEGMENT_READY '


@dataclass(frozen=True)
class AcceptanceGateDecision:
    """描述一次操作口令后是否允许进入当前分段的 ARM 状态."""

    armed: bool
    ready_confirmed: bool
    start_requested: bool
    stop_requested: bool
    reason: str


class LineAcceptanceGate:
    """以精确操作口令保护单段巡线，默认和异常输入均保持 DISARM."""

    def __init__(self, allowed_segment_id):
        """固定本次启动允许的 segment，避免错误口令启动另一段."""
        segment_id = str(allowed_segment_id).strip()
        if not segment_id:
            raise ValueError('allowed_segment_id must not be empty')
        self.allowed_segment_id = segment_id
        self.ready_confirmed = False
        self.armed = False

    def handle_command(self, command):
        """处理一条精确口令；任意未识别内容不会改变安全状态."""
        if not isinstance(command, str):
            return self._decision('invalid_command_type')

        if command == DISARM:
            was_armed = self.armed
            self.armed = False
            self.ready_confirmed = False
            return AcceptanceGateDecision(
                armed=False,
                ready_confirmed=False,
                start_requested=False,
                stop_requested=was_armed,
                reason='operator_disarm',
            )

        if command == LINE_TEST_READY:
            if self.armed:
                return self._decision('ready_ignored_while_armed')
            self.ready_confirmed = True
            return self._decision('operator_ready_confirmed')

        expected_segment_command = (
            SEGMENT_READY_PREFIX + self.allowed_segment_id
        )
        if command == expected_segment_command:
            if self.armed:
                return self._decision('segment_ignored_already_armed')
            if not self.ready_confirmed:
                return self._decision('segment_rejected_ready_not_confirmed')
            self.armed = True
            # 每次 ARM 都消耗一次实体安全确认，下一分段必须重新确认。
            self.ready_confirmed = False
            return AcceptanceGateDecision(
                armed=True,
                ready_confirmed=False,
                start_requested=True,
                stop_requested=False,
                reason='segment_armed',
            )

        if command.startswith(SEGMENT_READY_PREFIX):
            return self._decision('segment_rejected_not_allowed')
        return self._decision('unrecognized_command')

    def force_disarm(self, reason):
        """急停或守护异常时无条件撤销 ARM，且请求巡线状态机停机."""
        was_armed = self.armed
        self.armed = False
        self.ready_confirmed = False
        return AcceptanceGateDecision(
            armed=False,
            ready_confirmed=False,
            start_requested=False,
            stop_requested=was_armed,
            reason=str(reason),
        )

    def _decision(self, reason):
        return AcceptanceGateDecision(
            armed=self.armed,
            ready_confirmed=self.ready_confirmed,
            start_requested=False,
            stop_requested=False,
            reason=reason,
        )
