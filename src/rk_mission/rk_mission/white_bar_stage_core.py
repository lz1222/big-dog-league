"""ROS-independent stage routing for white-bar actions."""

import json
from dataclasses import dataclass


WHITE_BAR_STAGE_STATES = frozenset((
    'DISARMED',
    'START_ARMED',
    'START_RUNNING',
    'START_COMPLETED',
    'FINISH_ARMED',
    'FINISH_RUNNING',
    'FINISH_COMPLETED',
    'FAULTED',
))
STAGE_COMMANDS = frozenset(('START', 'FINISH', 'CLEAR', 'RESET'))
MOTION_BY_STAGE = {
    'START': 'start_jump',
    'FINISH': 'finish_jump',
}


@dataclass(frozen=True)
class WhiteBarStageEvent:
    """One state-machine decision that a ROS adapter can publish."""

    accepted: bool
    action: str
    run_id: str
    state: str
    active_stage: str
    motion_name: str
    last_sequence: int
    reason: str
    request_sent: bool
    action_done: bool


class WhiteBarStageController:
    """Route explicitly armed white-bar stages without ROS dependencies."""

    def __init__(self, allow_finish_only_test=False):
        self.allow_finish_only_test = bool(allow_finish_only_test)
        self.run_id = ''
        self.state = 'DISARMED'
        self.active_stage = ''
        self.last_sequence = 0
        self.request_sent = False
        self.action_done = False

    def start_run(self, run_id):
        """Create one fresh mission run with no armed white-bar stage."""
        if not self._is_nonempty_string(run_id):
            return self._event(False, 'REJECTED', 'invalid_run_id')
        self.run_id = run_id.strip()
        self.state = 'DISARMED'
        self.active_stage = ''
        self.last_sequence = 0
        self.request_sent = False
        self.action_done = False
        return self._event(True, 'RESET', 'mission_start_new_run')

    def mission_stop(self):
        """Discard the active run and make all previous commands stale."""
        self.run_id = ''
        self.state = 'DISARMED'
        self.active_stage = ''
        self.last_sequence = 0
        self.request_sent = False
        self.action_done = False
        return self._event(True, 'RESET', 'mission_stop_reset')

    def status_event(self, reason):
        """Expose the current state without changing controller state."""
        return self._event(True, 'STATUS', str(reason))

    def apply_json_command(self, raw_command):
        """Parse one strict JSON command before applying it."""
        if not isinstance(raw_command, str):
            return self._event(False, 'REJECTED', 'stage_command_not_string')
        try:
            command = json.loads(raw_command)
        except (TypeError, ValueError, json.JSONDecodeError):
            return self._event(False, 'REJECTED', 'stage_command_invalid_json')
        return self.apply_command(command)

    def apply_command(self, command):
        """Apply a run-correlated START, FINISH, CLEAR, or RESET command."""
        if not isinstance(command, dict):
            return self._event(False, 'REJECTED', 'stage_command_not_object')

        run_id = command.get('run_id')
        sequence = command.get('sequence')
        stage = command.get('stage')
        if not self._is_nonempty_string(run_id):
            return self._event(False, 'REJECTED', 'stage_command_invalid_run_id')
        if type(sequence) is not int or sequence <= 0:
            return self._event(False, 'REJECTED', 'stage_command_invalid_sequence')
        if type(stage) is not str or stage not in STAGE_COMMANDS:
            return self._event(False, 'REJECTED', 'stage_command_invalid_stage')
        if not self.run_id or run_id != self.run_id:
            return self._event(False, 'REJECTED', 'stage_command_run_id_mismatch')
        if sequence <= self.last_sequence:
            return self._event(False, 'REJECTED', 'stage_command_sequence_stale')
        if self._is_running():
            return self._event(False, 'REJECTED', 'stage_change_while_running')
        if self.state == 'FAULTED':
            return self._event(False, 'REJECTED', 'stage_faulted_reset_required')

        if stage == 'START':
            return self._arm_start(sequence)
        if stage == 'FINISH':
            return self._arm_finish(sequence)
        if stage == 'CLEAR':
            return self._clear(sequence)
        return self._reset(sequence)

    def white_bar_event(self, stop_threshold_reached):
        """Advance an armed stage only after the white-bar stop threshold."""
        if not stop_threshold_reached:
            return self._event(True, 'APPROACH', 'white_bar_stop_threshold_not_reached')
        if self.state == 'START_ARMED':
            return self._start_action('START')
        if self.state == 'FINISH_ARMED':
            return self._start_action('FINISH')
        if self._is_running():
            return self._event(True, 'WAIT_RESULT', 'white_bar_action_pending')
        if self.state == 'FAULTED':
            return self._event(False, 'FAULTED', 'white_bar_stage_faulted')
        return self._event(False, 'NOT_ARMED', 'white_bar_stage_not_armed')

    def complete_action(self, done=True):
        """Complete only the current running stage after a matching success."""
        if not done or not self._is_running() or not self.request_sent:
            return self._event(False, 'IGNORED', 'white_bar_done_ignored')
        self.action_done = True
        if self.state == 'START_RUNNING':
            self.state = 'START_COMPLETED'
        else:
            self.state = 'FINISH_COMPLETED'
        return self._event(True, 'COMPLETED', 'white_bar_action_done')

    def action_fault(self, reason):
        """Fail closed after a running action fails, times out, or cancels."""
        if not self._is_running():
            return self._event(False, 'IGNORED', 'white_bar_fault_ignored')
        self.state = 'FAULTED'
        self.action_done = False
        return self._event(False, 'FAULTED', str(reason or 'white_bar_action_failed'))

    def may_monitor_white_bar(self):
        """Return whether a detected bar may start a new handling cycle."""
        return self.state in ('DISARMED', 'START_ARMED', 'FINISH_ARMED')

    def _arm_start(self, sequence):
        if self.state != 'DISARMED':
            return self._event(False, 'REJECTED', 'start_stage_not_available')
        self._set_armed('START', sequence)
        return self._event(True, 'ARMED', 'start_stage_armed')

    def _arm_finish(self, sequence):
        allowed = self.state == 'START_COMPLETED' or (
            self.allow_finish_only_test and self.state == 'DISARMED'
        )
        if not allowed:
            return self._event(False, 'REJECTED', 'finish_requires_start_completed')
        self._set_armed('FINISH', sequence)
        return self._event(True, 'ARMED', 'finish_stage_armed')

    def _clear(self, sequence):
        if self.state not in ('START_ARMED', 'FINISH_ARMED'):
            return self._event(False, 'REJECTED', 'clear_requires_armed_stage')
        self.state = 'DISARMED'
        self.active_stage = ''
        self.last_sequence = sequence
        self.request_sent = False
        self.action_done = False
        return self._event(True, 'CLEARED', 'stage_cleared')

    def _reset(self, sequence):
        self.state = 'DISARMED'
        self.active_stage = ''
        self.last_sequence = 0
        self.request_sent = False
        self.action_done = False
        return self._event(True, 'RESET', 'stage_reset')

    def _set_armed(self, stage, sequence):
        self.active_stage = stage
        self.state = f'{stage}_ARMED'
        self.last_sequence = sequence
        self.request_sent = False
        self.action_done = False

    def _start_action(self, stage):
        self.active_stage = stage
        self.state = f'{stage}_RUNNING'
        self.request_sent = True
        self.action_done = False
        return self._event(True, 'SEND_REQUEST', 'white_bar_action_requested')

    def _is_running(self):
        return self.state in ('START_RUNNING', 'FINISH_RUNNING')

    @staticmethod
    def _is_nonempty_string(value):
        return type(value) is str and bool(value.strip())

    def _event(self, accepted, action, reason):
        motion_name = MOTION_BY_STAGE.get(self.active_stage, '')
        return WhiteBarStageEvent(
            accepted=accepted,
            action=action,
            run_id=self.run_id,
            state=self.state,
            active_stage=self.active_stage,
            motion_name=motion_name,
            last_sequence=self.last_sequence,
            reason=reason,
            request_sent=self.request_sent,
            action_done=self.action_done,
        )
