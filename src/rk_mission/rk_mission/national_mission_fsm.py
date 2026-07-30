"""
Fail-closed national competition mission policy.

No ROS type is imported here.  The node owns topic and Action wiring, while
this module owns state entry, event acceptance, timing, and safe transitions.
"""

from __future__ import annotations

import math
from typing import Dict, Optional

from .mission_adapters import ActionRequest, MissionAdapter
from .mission_context import MissionContext
from .mission_types import (
    InspectionType,
    MissionFailureCode,
    MissionState,
    MotionCommand,
    StationSide,
    TaskResult,
)
from .position_gates import (
    ConsecutiveDetectionGate,
    CurvedApexGate,
    RedMarkerBodyOffsetGate,
    StableLineGate,
    TimedDistanceGate,
)
from .segment_progress import SegmentProgress


DEFAULT_PARAMETERS = {
    'control_rate_hz': 20.0,
    'min_line_confidence': 0.55,
    'min_effective_speed': 0.05,
    'final_cmd_stale_sec': 0.75,
    'zero_epsilon': 0.01,
    'zero_confirm_frames': 2,
    'line_reacquire_frames': 3,
    'white_confirm_frames': 3,
    'marker_confirm_frames': 3,
    'sign_confirm_frames': 3,
    'red_confirm_frames': 3,
    'marker_min_confidence': 0.70,
    'sign_min_confidence': 0.70,
    'red_min_confidence': 0.60,
    'red_near_y': 0.72,
    'state_timeout_sec': 12.0,
    'action_timeout_sec': 8.0,
    'reacquire_timeout_sec': 7.0,
    'pattern_search_timeout_sec': 7.0,
    'sign_search_timeout_sec': 6.0,
    'turn_timeout_sec': 5.0,
    'mission_turn_speed': 0.70,
    'search_turn_speed': 0.40,
    'pick_turn_target_rad': math.pi / 2.0,
    'inspection_turn_target_rad': math.pi / 2.0,
    'search_small_angle_rad': 0.30,
    'search_medium_angle_rad': 0.55,
    'search_large_angle_rad': 0.85,
    'search_settle_sec': 0.20,
    'red_track_speed': 0.10,
    'red_track_kp': 1.2,
    'red_track_max_wz': 0.70,
    'pick_arm_task': 'pick_item',
    'transfer_place_task': 'transfer_place',
    'transfer_pick_task': 'transfer_pick',
    'place_platform_1_task': 'place_platform_1',
    'place_platform_2_task': 'place_platform_2',
    'inspection_electric_shock_task': 'stretch',
    'inspection_oxidizer_task': 'hello',
    'inspection_radiation_task': 'headlight_blink_3',
    'start_jump_motion': 'start_jump',
    'finish_jump_motion': 'finish_jump',
    'stairs_motion': 'stairs_traverse',
    'maze_motion': 'maze_traverse_fake',
    'start_white_min_segment_sec': 0.10,
    'finish_white_min_segment_sec': 0.10,
    'simulation_mode': False,
    'transfer_platform_turn_to_station': False,
}


def _segment(prefix: str, **overrides: float) -> Dict[str, float]:
    result = {
        prefix + '_min_distance': 0.10,
        prefix + '_target_distance': 0.18,
        prefix + '_max_distance': 0.45,
        prefix + '_min_effective_time': 0.15,
        prefix + '_target_effective_time': 0.35,
        prefix + '_hard_timeout': 8.0,
    }
    result.update(overrides)
    return result


DEFAULT_PARAMETERS.update(_segment('maze_entry'))
DEFAULT_PARAMETERS.update(_segment('stairs_approach'))
DEFAULT_PARAMETERS.update(_segment('transfer_platform'))
DEFAULT_PARAMETERS.update(_segment('place_platform'))
DEFAULT_PARAMETERS.update(_segment('final_zone'))
DEFAULT_PARAMETERS.update({
    'pick_arc_min_distance': 0.12,
    'pick_arc_target_distance': 0.22,
    'pick_arc_max_distance': 0.50,
    'pick_arc_min_curve_duration': 0.20,
    'pick_arc_min_abs_angular_z': 0.10,
    'pick_arc_target_turn_progress_rad': 0.24,
    'pick_arc_expected_turn_sign': 'auto',
    'pick_arc_hard_timeout': 8.0,
    'post_red_marker_distance': 0.10,
    'post_red_marker_time_fallback': 0.45,
    'post_red_marker_hard_timeout': 5.0,
})


REACQUIRE_DESTINATIONS = {
    MissionState.START_LINE_REACQUIRE: MissionState.MAZE_ENTRY_APPROACH,
    MissionState.MAZE_EXIT_REACQUIRE: MissionState.STAIRS_APPROACH,
    MissionState.STAIRS_EXIT_REACQUIRE: MissionState.PICK_ARC_APPROACH,
    MissionState.PICK_LINE_REACQUIRE: MissionState.TRANSFER_PLATFORM_APPROACH,
    MissionState.TRANSFER_LINE_REACQUIRE: MissionState.INSPECTION_APPROACH,
    MissionState.INSPECTION_LINE_REACQUIRE: MissionState.PLACE_PLATFORM_APPROACH,
    MissionState.FINISH_LINE_REACQUIRE: MissionState.FINAL_ZONE_APPROACH,
}
REACQUIRE_STATES = frozenset(REACQUIRE_DESTINATIONS)
LINE_PROGRESS_STATES = frozenset({
    MissionState.MAZE_ENTRY_APPROACH,
    MissionState.STAIRS_APPROACH,
    MissionState.PICK_ARC_APPROACH,
    MissionState.TRANSFER_PLATFORM_APPROACH,
    MissionState.INSPECTION_APPROACH,
    MissionState.RED_CIRCLE_TRACK,
    MissionState.RED_CIRCLE_POST_OFFSET,
    MissionState.PLACE_PLATFORM_APPROACH,
    MissionState.RETURN_LINE_FOLLOW,
    MissionState.FINAL_ZONE_APPROACH,
})
TURN_STATES = frozenset({
    MissionState.PICK_TURN_LEFT,
    MissionState.PICK_TURN_RIGHT,
    MissionState.INSPECTION_TURN_LEFT,
    MissionState.INSPECTION_TURN_RIGHT,
})
TERMINAL_STATES = frozenset({
    MissionState.MISSION_COMPLETE,
    MissionState.SAFE_STOP,
    MissionState.ESTOP,
    MissionState.MISSION_FAILED,
})


class NationalMissionFSM:
    """Full route FSM with only stable adapter calls at its boundary."""

    def __init__(self, adapter: MissionAdapter, parameters: Optional[dict] = None):
        self.adapter = adapter
        self.params = dict(DEFAULT_PARAMETERS)
        if parameters:
            self.params.update(parameters)
        self.context = MissionContext()
        self.progress = SegmentProgress()
        self.final_command = MotionCommand()
        self.last_final_command_time: Optional[float] = None
        self.last_tick_time: Optional[float] = None
        self.line_visible = False
        self.line_confidence = 0.0
        self.line_lateral_error = 0.0
        self.line_heading_error = 0.0
        self.gait_lock = False
        self.arm_lock = False
        self.estop = False
        self.line_enabled = False
        self.zero_confirm_count = 0
        self.pending_action: Optional[ActionRequest] = None
        self.pending_action_time: Optional[float] = None
        self.completed_tokens = set()
        self.paused_from: Optional[MissionState] = None
        self.state_data: Dict[str, object] = {}
        self._reset_gates()

    @property
    def state(self) -> MissionState:
        return self.context.current_state

    def _reset_gates(self) -> None:
        self.white_gate = ConsecutiveDetectionGate[bool](
            int(self.params['white_confirm_frames']), 0.5
        )
        self.line_gate = StableLineGate(
            int(self.params['line_reacquire_frames']),
            float(self.params['min_line_confidence']),
        )
        self.marker_gate = ConsecutiveDetectionGate[int](
            int(self.params['marker_confirm_frames']),
            float(self.params['marker_min_confidence']),
        )
        self.sign_gate = ConsecutiveDetectionGate[InspectionType](
            int(self.params['sign_confirm_frames']),
            float(self.params['sign_min_confidence']),
        )
        self.red_gate = ConsecutiveDetectionGate[bool](
            int(self.params['red_confirm_frames']),
            float(self.params['red_min_confidence']),
        )
        self.pick_arc_gate = CurvedApexGate(
            min_effective_distance=float(self.params['pick_arc_min_distance']),
            target_effective_distance=float(self.params['pick_arc_target_distance']),
            max_effective_distance=float(self.params['pick_arc_max_distance']),
            min_curve_duration=float(self.params['pick_arc_min_curve_duration']),
            min_abs_angular_z=float(self.params['pick_arc_min_abs_angular_z']),
            target_turn_progress_rad=float(
                self.params['pick_arc_target_turn_progress_rad']
            ),
            expected_turn_sign=str(self.params['pick_arc_expected_turn_sign']),
            hard_timeout=float(self.params['pick_arc_hard_timeout']),
        )
        self.red_offset_gate = RedMarkerBodyOffsetGate(
            post_marker_distance=float(self.params['post_red_marker_distance']),
            post_marker_time_fallback=float(
                self.params['post_red_marker_time_fallback']
            ),
            hard_timeout=float(self.params['post_red_marker_hard_timeout']),
        )

    def reset(self, now: float) -> bool:
        """Reset a completed/failed run; never clears an active system estop."""
        if self.estop:
            self._enter_safe(MissionState.ESTOP, MissionFailureCode.ESTOP_ACTIVE,
                             'reset refused while estop is active', now)
            return False
        self.context.reset(now)
        self.progress.reset_at_anchor('RESET', now)
        self.pending_action = None
        self.pending_action_time = None
        self.completed_tokens.clear()
        self.zero_confirm_count = 0
        self.state_data.clear()
        self._reset_gates()
        self._set_line_enabled(False)
        self.adapter.publish_mission_command(MotionCommand())
        self.adapter.emit_event('RESET', 'new mission context created')
        return True

    def start(self, now: float, start_from_state: str = '') -> bool:
        if self.estop:
            self._enter_safe(MissionState.ESTOP, MissionFailureCode.ESTOP_ACTIVE,
                             'start refused while estop is active', now)
            return False
        if self.state not in TERMINAL_STATES and self.state != MissionState.WAIT_START:
            self.adapter.emit_event('START_IGNORED', 'mission already active')
            return False
        self.reset(now)
        start_state = MissionState.START_LINE_FOLLOW
        if start_from_state:
            try:
                start_state = MissionState(str(start_from_state))
            except ValueError:
                self._enter_safe(
                    MissionState.SAFE_STOP,
                    MissionFailureCode.INVALID_TRANSITION,
                    'unknown start_from_state: {}'.format(start_from_state),
                    now,
                )
                return False
        self._transition(start_state, now, 'mission_start')
        return True

    def pause(self, now: float) -> None:
        if self.state in TERMINAL_STATES or self.state == MissionState.PAUSED:
            return
        self.paused_from = self.state
        self._transition(MissionState.PAUSED, now, 'pause_requested')

    def resume(self, now: float) -> None:
        if self.state != MissionState.PAUSED or self.paused_from is None:
            return
        previous = self.paused_from
        self.paused_from = None
        self._transition(previous, now, 'resume_requested')

    def stop(self, now: float, reason: str = 'external mission stop') -> None:
        self._enter_safe(MissionState.SAFE_STOP, MissionFailureCode.EXTERNAL_STOP,
                         reason, now)

    def on_estop(self, active: bool, now: float) -> None:
        self.estop = bool(active)
        if self.estop:
            self._enter_safe(MissionState.ESTOP, MissionFailureCode.ESTOP_ACTIVE,
                             'estop active', now)

    def on_locks(self, gait_lock: bool, arm_lock: bool) -> None:
        self.gait_lock = bool(gait_lock)
        self.arm_lock = bool(arm_lock)

    def on_line(self, visible: bool, confidence: float, lateral: float,
                heading: float) -> None:
        values = (confidence, lateral, heading)
        if not all(math.isfinite(float(value)) for value in values):
            self.line_visible = False
            self.line_confidence = 0.0
            return
        self.line_visible = bool(visible)
        self.line_confidence = float(confidence)
        self.line_lateral_error = float(lateral)
        self.line_heading_error = float(heading)
        if self.state in REACQUIRE_STATES:
            self.line_gate.update(self.line_visible, self.line_confidence)

    def on_white_line(self, visible: bool, confidence: float, now: float) -> None:
        if self.state not in (
            MissionState.START_WHITE_LINE_CONFIRM,
            MissionState.FINISH_WHITE_LINE_CONFIRM,
        ):
            return
        if self._elapsed(now) < self._white_min_elapsed():
            return
        if self.white_gate.update(bool(visible), float(confidence)):
            if self.state == MissionState.START_WHITE_LINE_CONFIRM:
                self.adapter.emit_event('WHITE_LINE', 'start white line confirmed')
                self._transition(MissionState.START_JUMP, now,
                                 'start_white_line_confirmed')
            else:
                self.adapter.emit_event('WHITE_LINE', 'finish white line confirmed')
                self._transition(MissionState.FINISH_JUMP, now,
                                 'finish_white_line_confirmed')

    def on_red_circle(self, visible: bool, x: float, y: float,
                      confidence: float, now: float) -> None:
        del x
        if self.context.inspection_completed or self.state not in (
            MissionState.INSPECTION_APPROACH,
            MissionState.RED_CIRCLE_TRACK,
        ):
            return
        if not all(math.isfinite(float(value)) for value in (y, confidence)):
            return
        confirmed = self.red_gate.update(bool(visible), float(confidence))
        if self.state == MissionState.INSPECTION_APPROACH and confirmed:
            self.adapter.emit_event('RED_CIRCLE', 'red circle first stable')
            self._transition(MissionState.RED_CIRCLE_TRACK, now,
                             'red_circle_first_stable')
            return
        if self.state == MissionState.RED_CIRCLE_TRACK and confirmed and (
            float(y) >= float(self.params['red_near_y'])
        ):
            self.progress.reset_at_anchor('RED_CIRCLE_NEAR', now)
            self.red_offset_gate.anchor()
            self.adapter.emit_event('RED_CIRCLE', 'red circle near ROI')
            self._transition(MissionState.RED_CIRCLE_POST_OFFSET, now,
                             'red_circle_near')

    def on_pick_marker(self, marker_id: int, confidence: float,
                       now: float) -> None:
        if self.context.pick_marker_id is not None or self.state not in (
            MissionState.PICK_PATTERN_SEARCH,
            MissionState.PICK_PATTERN_CONFIRM,
        ):
            return
        try:
            marker_id = int(marker_id)
        except (TypeError, ValueError):
            return
        if not self.marker_gate.update(marker_id, float(confidence)):
            return
        if marker_id not in (1, 2):
            self._enter_safe(
                MissionState.SAFE_STOP,
                MissionFailureCode.INVALID_TARGET_PLATFORM,
                'unsupported pick marker {}'.format(marker_id),
                now,
            )
            return
        self.context.pick_marker_id = marker_id
        self.context.target_place_platform = marker_id
        self._transition(MissionState.PICK_PATTERN_CONFIRM, now,
                         'pick_marker_{}_confirmed'.format(marker_id))

    def on_inspection_sign(self, inspection_type: str, confidence: float,
                           now: float) -> None:
        if self.context.inspection_completed or self.state != (
            MissionState.INSPECTION_SIGN_CONFIRM
        ):
            return
        try:
            normalized = InspectionType(str(inspection_type))
        except ValueError:
            return
        if self.sign_gate.update(normalized, float(confidence)):
            self.context.inspection_type = normalized
            self._transition(MissionState.INSPECTION_ACTION, now,
                             'inspection_sign_{}_confirmed'.format(
                                 normalized.value
                             ))

    def on_final_command(self, command: MotionCommand, now: float) -> None:
        if not all(math.isfinite(float(value)) for value in (command.vx, command.wz)):
            self._enter_safe(
                MissionState.SAFE_STOP,
                MissionFailureCode.INVALID_FINAL_COMMAND,
                'non-finite final command',
                now,
            )
            return
        self.final_command = MotionCommand(float(command.vx), float(command.wz))
        self.last_final_command_time = float(now)
        if (
            abs(self.final_command.vx) <= float(self.params['zero_epsilon'])
            and abs(self.final_command.wz) <= float(self.params['zero_epsilon'])
        ):
            self.zero_confirm_count += 1
        else:
            self.zero_confirm_count = 0

    def on_action_result(self, result: TaskResult, now: float) -> None:
        if result.token in self.completed_tokens:
            self.adapter.emit_event('ACTION_DUPLICATE_RESULT', result.token)
            return
        if self.pending_action is None or result.token != self.pending_action.token:
            self.adapter.emit_event('ACTION_IGNORED_RESULT', result.token)
            return
        request = self.pending_action
        self.pending_action = None
        self.pending_action_time = None
        self.completed_tokens.add(result.token)
        self.adapter.emit_event(
            'ACTION_RESULT',
            '{} success={} {}'.format(request.task_name, result.success,
                                      result.message),
        )
        if not result.success:
            self._enter_safe(
                MissionState.SAFE_STOP,
                MissionFailureCode.ACTION_FAILED,
                '{} failed: {}'.format(request.task_name, result.message),
                now,
            )
            return
        if self.state == MissionState.START_JUMP:
            self.context.start_jump_completed = True
            self.context.physical_crossing_unverified = True
            self._transition(MissionState.START_LINE_REACQUIRE, now,
                             'start_jump_success')
        elif self.state == MissionState.MAZE_TRAVERSE_FAKE:
            self._transition(MissionState.MAZE_EXIT_REACQUIRE, now,
                             'maze_placeholder_success')
        elif self.state == MissionState.STAIRS_TRAVERSE_FAKE:
            self._transition(MissionState.STAIRS_EXIT_REACQUIRE, now,
                             'stairs_placeholder_success')
        elif self.state == MissionState.ARM_PICK_FAKE:
            self._transition(MissionState.PICK_TURN_RIGHT, now,
                             'arm_pick_success')
        elif self.state == MissionState.ARM_TRANSFER_PLACE_FAKE:
            self.context.transfer_place_completed = True
            self._transition(MissionState.ARM_TRANSFER_PICK_FAKE, now,
                             'transfer_place_success')
        elif self.state == MissionState.ARM_TRANSFER_PICK_FAKE:
            self.context.transfer_pick_completed = True
            self._transition(MissionState.TRANSFER_LINE_REACQUIRE, now,
                             'transfer_pick_success')
        elif self.state == MissionState.INSPECTION_ACTION:
            self.context.inspection_completed = True
            self._transition(MissionState.INSPECTION_TURN_RIGHT, now,
                             'inspection_action_success')
        elif self.state == MissionState.ARM_PLACE_SELECTED_FAKE:
            self.context.place_completed = True
            self._transition(MissionState.RETURN_LINE_FOLLOW, now,
                             'place_action_success')
        elif self.state == MissionState.FINISH_JUMP:
            self.context.finish_jump_completed = True
            self.context.physical_crossing_unverified = True
            self._transition(MissionState.FINISH_LINE_REACQUIRE, now,
                             'finish_jump_success')

    def on_mux_invalid_command(self, invalid_count: int, now: float) -> None:
        """Fail closed when the real mux rejects a non-finite candidate."""
        if int(invalid_count) > 0 and self.state not in TERMINAL_STATES:
            self._enter_safe(
                MissionState.SAFE_STOP,
                MissionFailureCode.INVALID_FINAL_COMMAND,
                'command mux rejected invalid candidate',
                now,
            )

    def tick(self, now: float) -> None:
        now = float(now)
        if self.last_tick_time is None:
            self.last_tick_time = now
            return
        dt = max(0.0, now - self.last_tick_time)
        self.last_tick_time = now
        if self.state in TERMINAL_STATES:
            self._hold_zero()
            return
        if self.state == MissionState.WAIT_START:
            # Waiting for an explicit start is intentionally unbounded.
            self._hold_zero()
            return
        if self.estop:
            self._enter_safe(MissionState.ESTOP, MissionFailureCode.ESTOP_ACTIVE,
                             'estop active', now)
            return
        if self.state == MissionState.PAUSED:
            self._hold_zero()
            return
        if self._final_command_is_stale(now):
            self._enter_safe(
                MissionState.SAFE_STOP,
                MissionFailureCode.FINAL_COMMAND_STALE,
                'final command feedback stale',
                now,
            )
            return
        self._update_progress(dt)
        if self._timed_out(now):
            return
        self._run_state(now)

    def _run_state(self, now: float) -> None:
        state = self.state
        if state == MissionState.WAIT_START:
            self._hold_zero()
        elif state == MissionState.START_LINE_FOLLOW:
            self._transition(MissionState.START_WHITE_LINE_CONFIRM, now,
                             'line_follower_started')
        elif state == MissionState.START_JUMP:
            self._run_action(
                'locomotion', self.params['start_jump_motion'], '', now
            )
        elif state in REACQUIRE_STATES:
            self._run_reacquire(now)
        elif state == MissionState.MAZE_ENTRY_APPROACH:
            self._run_distance_state('maze_entry', MissionState.MAZE_TRAVERSE_FAKE,
                                     now)
        elif state == MissionState.MAZE_TRAVERSE_FAKE:
            self._run_action('maze', self.params['maze_motion'], '', now)
        elif state == MissionState.STAIRS_APPROACH:
            self._run_distance_state('stairs_approach',
                                     MissionState.STAIRS_TRAVERSE_FAKE, now)
        elif state == MissionState.STAIRS_TRAVERSE_FAKE:
            self._run_action('locomotion', self.params['stairs_motion'], '', now)
        elif state == MissionState.PICK_ARC_APPROACH:
            result = self.pick_arc_gate.check(self.progress, self._elapsed(now))
            if result.exceeded:
                self._enter_safe(MissionState.SAFE_STOP,
                                 MissionFailureCode.POSITION_GATE_EXCEEDED,
                                 'pick arc {}'.format(result.reason), now)
            elif result.reached:
                self.adapter.emit_event('POSITION', 'pick apex gate reached')
                self._transition(MissionState.PICK_ARC_APEX_STOP, now,
                                 'pick_arc_apex_reached')
        elif state == MissionState.PICK_ARC_APEX_STOP:
            if self._zero_confirmed():
                self._transition(MissionState.PICK_TURN_LEFT, now,
                                 'pick_apex_final_zero')
            else:
                self._hold_zero()
        elif state == MissionState.PICK_TURN_LEFT:
            self._run_turn(+1.0, float(self.params['pick_turn_target_rad']), now,
                           MissionState.PICK_PATTERN_SEARCH, 'pick_left_turn')
        elif state == MissionState.PICK_PATTERN_SEARCH:
            self._run_pattern_search(now)
        elif state == MissionState.PICK_PATTERN_CONFIRM:
            if self.context.target_place_platform in (1, 2):
                self._transition(MissionState.ARM_PICK_FAKE, now,
                                 'pick_target_latched')
            else:
                self._enter_safe(MissionState.SAFE_STOP,
                                 MissionFailureCode.INVALID_TARGET_PLATFORM,
                                 'pick target missing after confirmation', now)
        elif state == MissionState.ARM_PICK_FAKE:
            self._run_action('arm', self.params['pick_arm_task'], 'start_item', now)
        elif state == MissionState.PICK_TURN_RIGHT:
            target = float(self.state_data.get('pick_left_turn_progress', 0.0))
            self._run_turn(-1.0, target, now, MissionState.PICK_LINE_REACQUIRE,
                           'pick_right_turn')
        elif state == MissionState.TRANSFER_PLATFORM_APPROACH:
            self._run_distance_state('transfer_platform', MissionState.TRANSFER_STOP,
                                     now)
        elif state == MissionState.TRANSFER_STOP:
            if self._zero_confirmed():
                self._transition(MissionState.ARM_TRANSFER_PLACE_FAKE, now,
                                 'transfer_final_zero')
            else:
                self._hold_zero()
        elif state == MissionState.ARM_TRANSFER_PLACE_FAKE:
            self._run_action('arm', self.params['transfer_place_task'],
                             'transfer_platform', now)
        elif state == MissionState.ARM_TRANSFER_PICK_FAKE:
            self._run_action('arm', self.params['transfer_pick_task'],
                             'transfer_platform', now)
        elif state == MissionState.INSPECTION_APPROACH:
            self._release_for_line()
        elif state in (MissionState.RED_CIRCLE_TRACK,
                       MissionState.RED_CIRCLE_POST_OFFSET):
            self._run_red_circle_track(now)
        elif state == MissionState.INSPECTION_STOP:
            if self._zero_confirmed():
                self._transition(MissionState.INSPECTION_TURN_LEFT, now,
                                 'inspection_final_zero')
            else:
                self._hold_zero()
        elif state == MissionState.INSPECTION_TURN_LEFT:
            self._run_turn(+1.0, float(self.params['inspection_turn_target_rad']),
                           now, MissionState.INSPECTION_SIGN_CONFIRM,
                           'inspection_left_turn')
        elif state == MissionState.INSPECTION_SIGN_CONFIRM:
            self._run_sign_search(now)
        elif state == MissionState.INSPECTION_ACTION:
            action = self._inspection_action_name()
            if action is None:
                self._enter_safe(MissionState.SAFE_STOP,
                                 MissionFailureCode.DETECTION_TIMEOUT,
                                 'inspection sign is missing', now)
            else:
                self._run_action('locomotion', action, '', now)
        elif state == MissionState.INSPECTION_TURN_RIGHT:
            target = float(self.state_data.get('inspection_left_turn_progress', 0.0))
            self._run_turn(-1.0, target, now,
                           MissionState.INSPECTION_LINE_REACQUIRE,
                           'inspection_right_turn')
        elif state == MissionState.PLACE_PLATFORM_APPROACH:
            self._run_distance_state('place_platform', MissionState.PLACE_PLATFORM_STOP,
                                     now)
        elif state == MissionState.PLACE_PLATFORM_STOP:
            if self._zero_confirmed():
                self._transition(MissionState.ARM_PLACE_SELECTED_FAKE, now,
                                 'place_final_zero')
            else:
                self._hold_zero()
        elif state == MissionState.ARM_PLACE_SELECTED_FAKE:
            request = self._selected_place_action()
            if request is None:
                self._enter_safe(MissionState.SAFE_STOP,
                                 MissionFailureCode.INVALID_TARGET_PLATFORM,
                                 'target place platform is missing or invalid', now)
            else:
                self._run_action('arm', request[0], request[1], now)
        elif state == MissionState.RETURN_LINE_FOLLOW:
            self._transition(MissionState.FINISH_WHITE_LINE_CONFIRM, now,
                             'return_line_follower_started')
        elif state == MissionState.FINAL_ZONE_APPROACH:
            self._run_distance_state('final_zone', MissionState.FINAL_STOP, now)
        elif state == MissionState.FINISH_JUMP:
            self._run_action(
                'locomotion', self.params['finish_jump_motion'], '', now
            )
        elif state == MissionState.FINAL_STOP:
            if self._zero_confirmed():
                self._transition(MissionState.MISSION_COMPLETE, now,
                                 'final_zero_confirmed')
                self.adapter.emit_event('MISSION_COMPLETE', 'route completed')
            else:
                self._hold_zero()
        elif state in (
            MissionState.START_WHITE_LINE_CONFIRM,
            MissionState.FINISH_WHITE_LINE_CONFIRM,
        ):
            self._release_for_line()
        else:
            self._enter_safe(MissionState.SAFE_STOP,
                             MissionFailureCode.INVALID_TRANSITION,
                             'unhandled state {}'.format(state.value), now)

    def _run_reacquire(self, now: float) -> None:
        if self.line_gate.count >= int(self.params['line_reacquire_frames']):
            destination = REACQUIRE_DESTINATIONS[self.state]
            self._set_line_enabled(True)
            self._release_for_line()
            self._transition(destination, now, 'stable_line_reacquired')
            return
        elapsed = self._elapsed(now)
        if elapsed >= float(self.params['reacquire_timeout_sec']):
            self._enter_safe(MissionState.SAFE_STOP,
                             MissionFailureCode.LINE_REACQUIRE_TIMEOUT,
                             'line reacquire timeout', now)
            return
        phase = int(elapsed / max(0.05, float(self.params['search_settle_sec'])))
        sequence = (-1.0, 0.0, 1.0, 0.0, -1.0, 1.0)
        sign = sequence[phase % len(sequence)]
        self.adapter.publish_mission_command(
            MotionCommand(0.0, sign * float(self.params['search_turn_speed']))
        )

    def _run_distance_state(self, prefix: str, destination: MissionState,
                            now: float) -> None:
        self._release_for_line()
        gate = self._distance_gate(prefix)
        result = gate.check(self.progress, self._elapsed(now))
        if result.exceeded:
            code = (
                MissionFailureCode.POSITION_GATE_TIMEOUT
                if result.reason == 'hard_timeout'
                else MissionFailureCode.POSITION_GATE_EXCEEDED
            )
            self._enter_safe(MissionState.SAFE_STOP, code,
                             '{} {}'.format(prefix, result.reason), now)
        elif result.reached:
            self._transition(destination, now, '{} gate reached'.format(prefix))

    def _run_action(self, adapter_name: str, task_name: str, target: str,
                    now: float) -> None:
        self._hold_zero()
        if self.pending_action is not None:
            if self.pending_action_time is not None and (
                now - self.pending_action_time >= float(self.params['action_timeout_sec'])
            ):
                self._enter_safe(MissionState.SAFE_STOP,
                                 MissionFailureCode.ACTION_TIMEOUT,
                                 '{} timed out'.format(self.pending_action.task_name),
                                 now)
            return
        if not self._zero_confirmed():
            return
        token = '{}:{}:{}'.format(
            self.context.attempt_index, self.state.value, len(self.completed_tokens)
        )
        request = ActionRequest(token, adapter_name, str(task_name), str(target))
        self.pending_action = request
        self.pending_action_time = now
        try:
            self.adapter.dispatch(request)
        except Exception as exc:  # Adapter errors must remain fail-closed.
            self.pending_action = None
            self.pending_action_time = None
            self._enter_safe(MissionState.SAFE_STOP,
                             MissionFailureCode.ACTION_UNAVAILABLE,
                             '{} dispatch failed: {}'.format(task_name, exc), now)

    def _run_turn(self, sign: float, target: float, now: float,
                  destination: MissionState, label: str) -> None:
        target = max(0.0, float(target))
        if target <= 0.0:
            self._transition(destination, now, label + '_not_required')
            return
        if self.progress.absolute_turn_progress >= target:
            if label == 'pick_left_turn':
                self.state_data['pick_left_turn_progress'] = (
                    self.progress.absolute_turn_progress
                )
            elif label == 'inspection_left_turn':
                self.state_data['inspection_left_turn_progress'] = (
                    self.progress.absolute_turn_progress
                )
            self.adapter.publish_mission_command(MotionCommand())
            self._transition(destination, now, label + '_complete')
            return
        if self._elapsed(now) >= float(self.params['turn_timeout_sec']):
            self._enter_safe(MissionState.SAFE_STOP,
                             MissionFailureCode.POSITION_GATE_TIMEOUT,
                             label + ' timeout', now)
            return
        self.adapter.publish_mission_command(
            MotionCommand(0.0, float(sign) * float(self.params['mission_turn_speed']))
        )

    def _run_pattern_search(self, now: float) -> None:
        elapsed = self._elapsed(now)
        if elapsed >= float(self.params['pattern_search_timeout_sec']):
            self._enter_safe(MissionState.SAFE_STOP,
                             MissionFailureCode.DETECTION_TIMEOUT,
                             'pick pattern search timeout', now)
            return
        self.adapter.emit_event('PICK_SEARCH', 'turn left search')
        steps = (
            (0.0, float(self.params['search_settle_sec'])),
            (+1.0, float(self.params['search_small_angle_rad'])),
            (-1.0, float(self.params['search_medium_angle_rad'])),
            (+1.0, float(self.params['search_large_angle_rad'])),
            (-1.0, float(self.params['search_large_angle_rad'])),
        )
        remaining = elapsed
        speed = max(0.05, float(self.params['search_turn_speed']))
        for sign, size in steps:
            duration = size if sign == 0.0 else size / speed
            if remaining <= duration:
                self.adapter.publish_mission_command(
                    MotionCommand(0.0, sign * speed)
                )
                return
            remaining -= duration
        self.adapter.publish_mission_command(MotionCommand())

    def _run_sign_search(self, now: float) -> None:
        if self._elapsed(now) >= float(self.params['sign_search_timeout_sec']):
            self._enter_safe(MissionState.SAFE_STOP,
                             MissionFailureCode.DETECTION_TIMEOUT,
                             'inspection sign confirm timeout', now)
            return
        period = max(0.10, float(self.params['search_settle_sec']))
        phase = int(self._elapsed(now) / period)
        sign = (0.0, +1.0, 0.0, -1.0)[phase % 4]
        self.adapter.publish_mission_command(
            MotionCommand(0.0, sign * float(self.params['search_turn_speed']))
        )

    def _run_red_circle_track(self, now: float) -> None:
        if not self.line_visible or self.line_confidence < float(
            self.params['min_line_confidence']
        ):
            self.adapter.publish_mission_command(MotionCommand())
        else:
            error = self.line_lateral_error + self.line_heading_error
            wz = max(-float(self.params['red_track_max_wz']), min(
                float(self.params['red_track_max_wz']),
                -float(self.params['red_track_kp']) * error,
            ))
            self.adapter.publish_mission_command(
                MotionCommand(float(self.params['red_track_speed']), wz)
            )
        if self.state == MissionState.RED_CIRCLE_POST_OFFSET:
            result = self.red_offset_gate.check(self.progress, self._elapsed(now))
            if result.exceeded:
                self._enter_safe(MissionState.SAFE_STOP,
                                 MissionFailureCode.POSITION_GATE_TIMEOUT,
                                 'post red marker offset timeout', now)
            elif result.reached:
                self.adapter.emit_event('POSITION', 'red circle body offset reached')
                self._transition(MissionState.INSPECTION_STOP, now,
                                 'post_red_offset_reached')

    def _transition(self, target: MissionState, now: float, reason: str) -> None:
        if target == self.state:
            return
        old = self.state
        self.context.transition(target, now, reason)
        self.zero_confirm_count = 0
        self.pending_action = None
        self.pending_action_time = None
        self.line_gate.reset()
        self.white_gate.reset()
        self.marker_gate.reset()
        self.sign_gate.reset()
        self.red_gate.reset()
        self.progress.reset_at_anchor(target.value, now)
        if target in REACQUIRE_STATES:
            self._set_line_enabled(False)
        elif target in LINE_PROGRESS_STATES or target in (
            MissionState.START_LINE_FOLLOW,
            MissionState.START_WHITE_LINE_CONFIRM,
            MissionState.FINISH_WHITE_LINE_CONFIRM,
            MissionState.RETURN_LINE_FOLLOW,
        ):
            if target not in (
                MissionState.RED_CIRCLE_TRACK,
                MissionState.RED_CIRCLE_POST_OFFSET,
            ):
                self._set_line_enabled(True)
        else:
            self._set_line_enabled(False)
        if target == MissionState.PICK_ARC_APPROACH:
            self.pick_arc_gate = CurvedApexGate(
                min_effective_distance=float(self.params['pick_arc_min_distance']),
                target_effective_distance=float(self.params['pick_arc_target_distance']),
                max_effective_distance=float(self.params['pick_arc_max_distance']),
                min_curve_duration=float(self.params['pick_arc_min_curve_duration']),
                min_abs_angular_z=float(self.params['pick_arc_min_abs_angular_z']),
                target_turn_progress_rad=float(
                    self.params['pick_arc_target_turn_progress_rad']
                ),
                expected_turn_sign=str(self.params['pick_arc_expected_turn_sign']),
                hard_timeout=float(self.params['pick_arc_hard_timeout']),
            )
        if target == MissionState.RED_CIRCLE_POST_OFFSET:
            self.red_offset_gate = RedMarkerBodyOffsetGate(
                post_marker_distance=float(self.params['post_red_marker_distance']),
                post_marker_time_fallback=float(
                    self.params['post_red_marker_time_fallback']
                ),
                hard_timeout=float(self.params['post_red_marker_hard_timeout']),
            )
            self.red_offset_gate.anchor()
        self.adapter.emit_event(
            'TRANSITION', '{} -> {} ({})'.format(old.value, target.value, reason)
        )

    def _enter_safe(self, state: MissionState, code: MissionFailureCode,
                    reason: str, now: float) -> None:
        if (
            self.state in TERMINAL_STATES
            and self.state != MissionState.ESTOP
            and state != MissionState.ESTOP
        ):
            return
        self.context.fail(code, reason)
        self._set_line_enabled(False)
        self.adapter.publish_mission_command(MotionCommand())
        if self.state != state:
            self.context.transition(state, now, reason)
        self.adapter.emit_event('SAFE_STOP', '{}: {}'.format(code.value, reason))

    def _set_line_enabled(self, enabled: bool) -> None:
        enabled = bool(enabled)
        if self.line_enabled != enabled:
            self.line_enabled = enabled
            self.adapter.set_line_enabled(enabled)

    def _release_for_line(self) -> None:
        self._set_line_enabled(True)
        self.adapter.release_mission_command()

    def _hold_zero(self) -> None:
        self._set_line_enabled(False)
        self.adapter.publish_mission_command(MotionCommand())

    def _update_progress(self, dt: float) -> None:
        if self.state in LINE_PROGRESS_STATES:
            valid_forward = self.progress.update(
                dt,
                self.final_command,
                line_visible=self.line_visible,
                line_confidence=self.line_confidence,
                min_line_confidence=float(self.params['min_line_confidence']),
                min_effective_speed=float(self.params['min_effective_speed']),
                estop=self.estop,
                gait_lock=self.gait_lock,
                arm_lock=self.arm_lock,
                state_allows_forward=True,
                searching_in_place=False,
            )
            if self.state == MissionState.PICK_ARC_APPROACH and valid_forward:
                self.pick_arc_gate.update(dt, self.final_command.wz)
        elif self.state in TURN_STATES:
            self.progress.update_turn(
                dt, self.final_command, estop=self.estop,
                gait_lock=self.gait_lock, arm_lock=self.arm_lock,
            )

    def _distance_gate(self, prefix: str) -> TimedDistanceGate:
        return TimedDistanceGate(
            min_distance=float(self.params[prefix + '_min_distance']),
            target_distance=float(self.params[prefix + '_target_distance']),
            max_distance=float(self.params[prefix + '_max_distance']),
            min_effective_time=float(self.params[prefix + '_min_effective_time']),
            target_effective_time=float(self.params[prefix + '_target_effective_time']),
            hard_timeout=float(self.params[prefix + '_hard_timeout']),
        )

    def _inspection_action_name(self) -> Optional[str]:
        inspection = self.context.inspection_type
        if inspection == InspectionType.ELECTRIC_SHOCK:
            return str(self.params['inspection_electric_shock_task'])
        if inspection == InspectionType.OXIDIZER:
            return str(self.params['inspection_oxidizer_task'])
        if inspection == InspectionType.RADIATION:
            return str(self.params['inspection_radiation_task'])
        return None

    def _selected_place_action(self) -> Optional[tuple]:
        target = self.context.target_place_platform
        if target == 1:
            return (str(self.params['place_platform_1_task']), StationSide.LEFT.value)
        if target == 2:
            return (str(self.params['place_platform_2_task']), StationSide.RIGHT.value)
        return None

    def _final_command_is_stale(self, now: float) -> bool:
        feedback_required = (
            self.state in LINE_PROGRESS_STATES
            or self.state in TURN_STATES
            or self.state in REACQUIRE_STATES
            or self.state in (
                MissionState.START_WHITE_LINE_CONFIRM,
                MissionState.FINISH_WHITE_LINE_CONFIRM,
            )
        )
        if not feedback_required:
            return False
        if self.last_final_command_time is None:
            return self._elapsed(now) >= float(self.params['final_cmd_stale_sec'])
        return now - self.last_final_command_time >= float(
            self.params['final_cmd_stale_sec']
        )

    def _timed_out(self, now: float) -> bool:
        state = self.state
        special = {
            MissionState.PICK_PATTERN_SEARCH: self.params['pattern_search_timeout_sec'],
            MissionState.INSPECTION_SIGN_CONFIRM: self.params['sign_search_timeout_sec'],
        }
        timeout = float(special.get(state, self.params['state_timeout_sec']))
        if state in REACQUIRE_STATES:
            timeout = max(timeout, float(self.params['reacquire_timeout_sec']))
        if self._elapsed(now) >= timeout:
            self._enter_safe(MissionState.SAFE_STOP, MissionFailureCode.STATE_TIMEOUT,
                             '{} state timeout'.format(state.value), now)
            return True
        return False

    def _elapsed(self, now: float) -> float:
        return max(0.0, float(now) - float(self.context.state_enter_time))

    def _white_min_elapsed(self) -> float:
        if self.state == MissionState.START_WHITE_LINE_CONFIRM:
            return float(self.params['start_white_min_segment_sec'])
        return float(self.params['finish_white_min_segment_sec'])

    def _zero_confirmed(self) -> bool:
        return self.zero_confirm_count >= int(self.params['zero_confirm_frames'])
