"""Small independently testable gates used by the national mission FSM."""

from dataclasses import dataclass
import math
from typing import Generic, Optional, TypeVar

from .segment_progress import SegmentProgress


@dataclass(frozen=True)
class GateResult:
    reached: bool
    exceeded: bool = False
    reason: str = ''


@dataclass
class TimedDistanceGate:
    min_distance: float
    target_distance: float
    max_distance: float
    min_effective_time: float
    target_effective_time: float
    hard_timeout: float

    def check(self, progress: SegmentProgress, elapsed: float) -> GateResult:
        if elapsed >= self.hard_timeout:
            return GateResult(False, True, 'hard_timeout')
        if progress.commanded_forward_distance > self.max_distance:
            return GateResult(False, True, 'max_distance_exceeded')
        minimum_met = (
            progress.commanded_forward_distance >= self.min_distance
            and progress.effective_forward_time >= self.min_effective_time
        )
        target_met = (
            progress.commanded_forward_distance >= self.target_distance
            or progress.effective_forward_time >= self.target_effective_time
        )
        return GateResult(bool(minimum_met and target_met), False, 'target')


@dataclass
class CurvedApexGate:
    min_effective_distance: float
    target_effective_distance: float
    max_effective_distance: float
    min_curve_duration: float
    min_abs_angular_z: float
    target_turn_progress_rad: float
    expected_turn_sign: str = 'auto'
    hard_timeout: float = 10.0
    locked_turn_sign: Optional[int] = None
    curve_duration: float = 0.0
    reverse_duration: float = 0.0

    def update(self, dt: float, angular_z: float) -> None:
        if not math.isfinite(float(dt)) or not math.isfinite(float(angular_z)):
            return
        if abs(float(angular_z)) < self.min_abs_angular_z:
            return
        sign = 1 if angular_z > 0.0 else -1
        configured = str(self.expected_turn_sign).lower()
        if self.locked_turn_sign is None:
            if configured == 'left':
                self.locked_turn_sign = 1
            elif configured == 'right':
                self.locked_turn_sign = -1
            else:
                self.locked_turn_sign = sign
        if sign == self.locked_turn_sign:
            self.curve_duration += max(0.0, float(dt))
        else:
            self.reverse_duration += max(0.0, float(dt))

    def check(self, progress: SegmentProgress, elapsed: float) -> GateResult:
        if elapsed >= self.hard_timeout:
            return GateResult(False, True, 'hard_timeout')
        if progress.commanded_forward_distance > self.max_effective_distance:
            return GateResult(False, True, 'max_distance_exceeded')
        direction_ok = self.locked_turn_sign is not None
        signed_turn = progress.signed_turn_progress
        if self.locked_turn_sign is not None:
            signed_turn *= self.locked_turn_sign
        reached = (
            progress.commanded_forward_distance >= self.min_effective_distance
            and progress.commanded_forward_distance >= self.target_effective_distance
            and self.curve_duration >= self.min_curve_duration
            and direction_ok
            and signed_turn >= self.target_turn_progress_rad
        )
        return GateResult(reached, False, 'curved_apex')


@dataclass
class RedMarkerBodyOffsetGate:
    post_marker_distance: float
    post_marker_time_fallback: float
    hard_timeout: float
    anchored: bool = False

    def anchor(self) -> None:
        self.anchored = True

    def check(self, progress: SegmentProgress, elapsed: float) -> GateResult:
        if not self.anchored:
            return GateResult(False, False, 'not_anchored')
        if elapsed >= self.hard_timeout:
            return GateResult(False, True, 'hard_timeout')
        reached = (
            progress.commanded_forward_distance >= self.post_marker_distance
            or progress.effective_forward_time >= self.post_marker_time_fallback
        )
        return GateResult(reached, False, 'post_marker_offset')


@dataclass
class StableLineGate:
    required_frames: int
    min_confidence: float
    count: int = 0

    def update(self, visible: bool, confidence: float) -> bool:
        if bool(visible) and math.isfinite(float(confidence)) and (
            float(confidence) >= self.min_confidence
        ):
            self.count += 1
        else:
            self.count = 0
        return self.count >= self.required_frames

    def reset(self) -> None:
        self.count = 0


T = TypeVar('T')


@dataclass
class ConsecutiveDetectionGate(Generic[T]):
    required_frames: int
    min_confidence: float
    value: Optional[T] = None
    count: int = 0

    def update(self, value: T, confidence: float) -> bool:
        if not math.isfinite(float(confidence)) or float(confidence) < self.min_confidence:
            self.reset()
            return False
        if self.value != value:
            self.value = value
            self.count = 1
        else:
            self.count += 1
        return self.count >= self.required_frames

    def reset(self) -> None:
        self.value = None
        self.count = 0
