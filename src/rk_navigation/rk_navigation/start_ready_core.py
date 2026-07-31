"""START_READY 阶段的纯 Python 稳定门，便于离线验证不盲走约束。"""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class StartReadyDecision:
    """一次巡线观测对启动稳定门的判定结果。"""

    ready: bool
    confirm_count: int
    reason: str


class StartReadyGate:
    """只在连续可信且居中的线迹出现后允许巡线启动。

    该对象不依赖 ROS，节点负责消息新鲜度和超时；这里仅处理一帧
    LineTrack 的数值边界与连续确认计数，防止旧帧或异常帧被误当作启动条件。
    """

    def __init__(
        self,
        confirm_frames=5,
        min_confidence=0.55,
        max_lateral_error=0.80,
        max_heading_error=0.80,
    ):
        self.configure(
            confirm_frames,
            min_confidence,
            max_lateral_error,
            max_heading_error,
        )
        self.reset()

    def configure(
        self,
        confirm_frames,
        min_confidence,
        max_lateral_error,
        max_heading_error,
    ):
        """更新可热调参数，同时保持当前连续帧计数。"""
        if type(confirm_frames) is not int or confirm_frames < 1:
            raise ValueError('confirm_frames must be a positive integer')
        values = (
            float(min_confidence),
            float(max_lateral_error),
            float(max_heading_error),
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError('START_READY thresholds must be finite')
        if not 0.0 <= values[0] <= 1.0:
            raise ValueError('min_confidence must be within [0, 1]')
        if values[1] < 0.0 or values[2] < 0.0:
            raise ValueError('START_READY error limits must be nonnegative')
        self.confirm_frames = confirm_frames
        self.min_confidence = values[0]
        self.max_lateral_error = values[1]
        self.max_heading_error = values[2]

    def reset(self):
        """清除跨任务计数，避免旧任务的可见帧累积到新任务。"""
        self.confirm_count = 0

    def observe(self, line_visible, confidence, lateral_error, heading_error):
        """记录一帧；任何不可用或越界数据都会中断连续确认。"""
        try:
            confidence = float(confidence)
            lateral_error = float(lateral_error)
            heading_error = float(heading_error)
        except (TypeError, ValueError):
            self.reset()
            return self._decision('start_ready_non_numeric_line_track')
        if not all(math.isfinite(value) for value in (
            confidence,
            lateral_error,
            heading_error,
        )):
            self.reset()
            return self._decision('start_ready_non_finite_line_track')
        if not bool(line_visible):
            self.reset()
            return self._decision('start_ready_line_not_visible')
        if confidence < self.min_confidence:
            self.reset()
            return self._decision('start_ready_confidence_low')
        if abs(lateral_error) > self.max_lateral_error:
            self.reset()
            return self._decision('start_ready_lateral_error_large')
        if abs(heading_error) > self.max_heading_error:
            self.reset()
            return self._decision('start_ready_heading_error_large')
        self.confirm_count += 1
        if self.confirm_count >= self.confirm_frames:
            return self._decision('start_ready_confirmed', ready=True)
        return self._decision('start_ready_confirming')

    def _decision(self, reason, ready=False):
        return StartReadyDecision(
            ready=bool(ready),
            confirm_count=int(self.confirm_count),
            reason=reason,
        )
