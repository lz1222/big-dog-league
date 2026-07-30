"""Per-run mission context with explicit reset semantics."""

from dataclasses import dataclass, field
from typing import List, Optional
from uuid import uuid4

from .mission_types import (
    InspectionType,
    MissionFailureCode,
    MissionState,
    TransitionRecord,
)


@dataclass
class MissionContext:
    """Mutable context owned by exactly one national mission run."""

    mission_id: str = field(default_factory=lambda: uuid4().hex)
    attempt_index: int = 0
    current_state: MissionState = MissionState.WAIT_START
    previous_state: Optional[MissionState] = None
    target_place_platform: Optional[int] = None
    pick_marker_id: Optional[int] = None
    inspection_type: Optional[InspectionType] = None
    inspection_completed: bool = False
    transfer_place_completed: bool = False
    transfer_pick_completed: bool = False
    place_completed: bool = False
    start_jump_completed: bool = False
    finish_jump_completed: bool = False
    failure_code: MissionFailureCode = MissionFailureCode.NONE
    failure_reason: str = ''
    state_enter_time: float = 0.0
    transition_history: List[TransitionRecord] = field(default_factory=list)
    physical_crossing_unverified: bool = False

    def reset(self, now: float, attempt_index: Optional[int] = None) -> None:
        """Clear every route-dependent value without changing system estop."""
        self.mission_id = uuid4().hex
        self.attempt_index = (
            int(attempt_index)
            if attempt_index is not None
            else self.attempt_index + 1
        )
        self.current_state = MissionState.WAIT_START
        self.previous_state = None
        self.target_place_platform = None
        self.pick_marker_id = None
        self.inspection_type = None
        self.inspection_completed = False
        self.transfer_place_completed = False
        self.transfer_pick_completed = False
        self.place_completed = False
        self.start_jump_completed = False
        self.finish_jump_completed = False
        self.failure_code = MissionFailureCode.NONE
        self.failure_reason = ''
        self.state_enter_time = float(now)
        self.transition_history.clear()
        self.physical_crossing_unverified = False

    def transition(self, state: MissionState, now: float, reason: str) -> None:
        """Set state and retain a complete, monotonic transition history."""
        previous = self.current_state
        self.previous_state = previous
        self.current_state = state
        self.state_enter_time = float(now)
        self.transition_history.append(
            TransitionRecord(float(now), previous, state, str(reason))
        )

    def fail(self, code: MissionFailureCode, reason: str) -> None:
        self.failure_code = code
        self.failure_reason = str(reason)
