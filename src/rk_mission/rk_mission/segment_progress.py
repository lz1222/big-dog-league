"""Command-based segment progress, never represented as physical odometry."""

from dataclasses import dataclass
import math
from typing import Optional

from .mission_types import MotionCommand


@dataclass
class SegmentProgress:
    """Accumulate only safe, state-authorized final command motion.

    ``commanded_forward_distance`` is intentionally only a command estimate.
    It is not a measured robot distance and must not be used as one.
    """

    effective_forward_time: float = 0.0
    commanded_forward_distance: float = 0.0
    signed_turn_progress: float = 0.0
    absolute_turn_progress: float = 0.0
    line_visible_duration: float = 0.0
    current_segment_anchor: Optional[str] = None
    anchor_time: Optional[float] = None

    def reset_at_anchor(self, anchor_name: str, now: float) -> None:
        self.effective_forward_time = 0.0
        self.commanded_forward_distance = 0.0
        self.signed_turn_progress = 0.0
        self.absolute_turn_progress = 0.0
        self.line_visible_duration = 0.0
        self.current_segment_anchor = str(anchor_name)
        self.anchor_time = float(now)

    def update(
        self,
        dt: float,
        final_command: MotionCommand,
        *,
        line_visible: bool,
        line_confidence: float,
        min_line_confidence: float,
        min_effective_speed: float,
        estop: bool,
        gait_lock: bool,
        arm_lock: bool,
        state_allows_forward: bool,
        searching_in_place: bool,
    ) -> bool:
        """Record a sample iff every required validity condition holds."""
        try:
            dt = float(dt)
            vx = float(final_command.vx)
            wz = float(final_command.wz)
            confidence = float(line_confidence)
        except (TypeError, ValueError):
            return False
        if (
            not math.isfinite(dt)
            or not math.isfinite(vx)
            or not math.isfinite(wz)
            or not math.isfinite(confidence)
            or dt <= 0.0
        ):
            return False

        line_ok = bool(line_visible) and confidence >= float(min_line_confidence)
        if line_ok:
            self.line_visible_duration += dt
        valid_forward = (
            line_ok
            and vx > 0.0
            and vx >= float(min_effective_speed)
            and not bool(estop)
            and not bool(gait_lock)
            and not bool(arm_lock)
            and bool(state_allows_forward)
            and not bool(searching_in_place)
        )
        if valid_forward:
            self.effective_forward_time += dt
            self.commanded_forward_distance += max(vx, 0.0) * dt
            self.signed_turn_progress += wz * dt
            self.absolute_turn_progress += abs(wz) * dt
        return valid_forward

    def update_turn(
        self,
        dt: float,
        final_command: MotionCommand,
        *,
        estop: bool,
        gait_lock: bool,
        arm_lock: bool,
    ) -> bool:
        """Integrate a stationary mission turn from final command feedback."""
        try:
            dt = float(dt)
            vx = float(final_command.vx)
            wz = float(final_command.wz)
        except (TypeError, ValueError):
            return False
        if (
            not math.isfinite(dt)
            or not math.isfinite(vx)
            or not math.isfinite(wz)
            or dt <= 0.0
            or bool(estop)
            or bool(gait_lock)
            or bool(arm_lock)
        ):
            return False
        self.signed_turn_progress += wz * dt
        self.absolute_turn_progress += abs(wz) * dt
        return True
