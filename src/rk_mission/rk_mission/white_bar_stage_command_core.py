"""ROS-independent sequencing of explicit white-bar stage commands."""

from dataclasses import dataclass
import math


SEQUENCER_STATES = frozenset((
    'IDLE',
    'WAIT_RUN',
    'START_PENDING',
    'START_ACKED',
    'WAIT_FINISH_MILESTONE',
    'FINISH_PENDING',
    'FINISH_ACKED',
    'COMPLETED',
    'FAULTED',
))
FINISH_MILESTONE_STATE = 'TURN_AFTER_RED'


@dataclass(frozen=True)
class WhiteBarStageCommandEvent:
    """One sequencer outcome for a ROS topic adapter."""

    action: str
    command_payload: dict | None
    run_id: str
    sequence: int
    requested_stage: str
    state: str
    reason: str
    retry_count: int
    start_completed: bool
    finish_milestone_seen: bool
    finish_completed: bool


class WhiteBarStageCommandSequencer:
    """Issue run-correlated START then FINISH commands exactly as allowed."""

    def __init__(
        self,
        command_retry_sec=0.5,
        command_ack_timeout_sec=5.0,
        max_command_retries=5,
        finish_milestone_state=FINISH_MILESTONE_STATE,
    ):
        if finish_milestone_state != FINISH_MILESTONE_STATE:
            raise ValueError(
                'finish_milestone_state must be TURN_AFTER_RED'
            )
        self.command_retry_sec = self._positive_float(
            command_retry_sec,
            'command_retry_sec'
        )
        self.command_ack_timeout_sec = self._positive_float(
            command_ack_timeout_sec,
            'command_ack_timeout_sec'
        )
        if type(max_command_retries) is not int or max_command_retries < 0:
            raise ValueError('max_command_retries must be a nonnegative integer')
        self.max_command_retries = max_command_retries
        self.finish_milestone_state = finish_milestone_state
        self._reset_to_idle()

    def mission_start(self, now=0.0):
        """Start a new sequencing cycle without inventing a run identifier."""
        self._set_now(now)
        self._reset_for_new_mission()
        return self._event('STATUS', 'mission_start_waiting_for_run')

    def mission_stop(self, now=0.0):
        """Forget the active run without sending a conflicting reset command."""
        self._set_now(now)
        self._reset_to_idle()
        return self._event('STATUS', 'mission_stop_reset_to_idle')

    def status_event(self, reason):
        """Expose the current sequencing state without changing it."""
        return self._event('STATUS', str(reason))

    def on_stage_status(self, payload, now=0.0):
        """Process one already-decoded white-bar stage status object."""
        self._set_now(now)
        status = self._validated_stage_status(payload)
        if status is None:
            return self._event('STATUS', 'stage_status_invalid')
        if self.state == 'IDLE':
            return self._event('STATUS', 'stage_status_ignored_not_started')

        status_run_id = status['run_id']
        if not status_run_id:
            return self._event('STATUS', 'stage_status_ignored_empty_run_id')
        if self.run_id and status_run_id != self.run_id:
            return self._fault('stage_status_run_id_mismatch')
        if self.state == 'WAIT_RUN':
            return self._wait_for_run_status(status)

        if status['state'] == 'FAULTED':
            return self._fault('white_bar_stage_faulted')
        if status['last_sequence'] < self._last_stage_sequence:
            return self._fault('stage_status_sequence_rollback')
        self._last_stage_sequence = status['last_sequence']

        if self._is_start_status(status):
            return self._accept_start_status(status)
        if self._is_finish_status(status):
            return self._accept_finish_status(status)
        return self._event('STATUS', 'stage_status_not_current_ack')

    def on_line_course_state(self, payload, now=0.0):
        """Latch only the configured red-action milestone for the active run."""
        self._set_now(now)
        line_state = self._validated_line_course_state(payload)
        if line_state is None:
            return self._event('STATUS', 'line_course_state_invalid')
        if self.state == 'IDLE':
            return self._event('STATUS', 'line_course_state_ignored_not_started')
        if not line_state['mission_started']:
            return self._fault('line_course_mission_not_started')
        if line_state['state'] == 'EMERGENCY_STOP':
            return self._fault('line_course_emergency_stop')
        if (
            line_state['state'] == 'FINAL_STOP'
            and not self.finish_completed
        ):
            return self._fault('line_course_final_stop_before_finish_completed')
        if not self.run_id:
            return self._event('STATUS', 'line_course_state_waiting_for_run')
        if not line_state['white_bar_stage_run_id']:
            return self._event('STATUS', 'line_course_state_ignored_empty_run_id')
        if line_state['white_bar_stage_run_id'] != self.run_id:
            return self._event('STATUS', 'line_course_state_run_id_mismatch')
        if line_state['state'] != self.finish_milestone_state:
            return self._event('STATUS', 'line_course_state_not_finish_milestone')

        self.finish_milestone_seen = True
        if self.start_completed and self.state == 'WAIT_FINISH_MILESTONE':
            return self._begin_command('FINISH', 2, 'finish_milestone_seen')
        return self._event(
            'STATUS',
            'finish_milestone_latched_waiting_for_start'
        )

    def on_timer(self, now):
        """Retry one unacknowledged command or fault on its deadline."""
        self._set_now(now)
        if self.state not in ('START_PENDING', 'FINISH_PENDING'):
            return self._event('NONE', 'no_pending_command')
        if self._pending_started_at is None or self._pending_last_sent_at is None:
            return self._fault('pending_command_clock_missing')
        if now - self._pending_started_at >= self.command_ack_timeout_sec:
            return self._fault('command_ack_timeout')
        if now - self._pending_last_sent_at < self.command_retry_sec:
            return self._event('NONE', 'command_retry_not_due')
        if self.retry_count >= self.max_command_retries:
            return self._fault('command_max_retries_exceeded')

        self.retry_count += 1
        self._pending_last_sent_at = now
        return self._event(
            'SEND_COMMAND',
            'command_retry',
            self._command_payload()
        )

    def _wait_for_run_status(self, status):
        if (
            status['state'] != 'DISARMED'
            or status['last_sequence'] != 0
        ):
            return self._event('STATUS', 'waiting_for_disarmed_new_run')
        self.run_id = status['run_id']
        self.sequence = 0
        self._last_stage_sequence = 0
        return self._begin_command('START', 1, 'new_run_disarmed')

    def _accept_start_status(self, status):
        if status['motion_name'] == 'finish_jump':
            return self._fault('start_stage_mapped_to_finish_jump')
        if status['motion_name'] not in ('', 'start_jump'):
            return self._event('STATUS', 'start_stage_motion_not_acknowledged')
        if status['state'] == 'START_COMPLETED':
            if self.start_completed:
                return self._event('STATUS', 'start_completed_already_seen')
            self._clear_pending()
            self.start_completed = True
            self.state = 'WAIT_FINISH_MILESTONE'
            self.requested_stage = ''
            if self.finish_milestone_seen:
                return self._begin_command(
                    'FINISH',
                    2,
                    'start_completed_after_finish_milestone'
                )
            return self._event('STATUS', 'start_completed_waiting_for_milestone')
        if self.state == 'START_PENDING':
            self._clear_pending()
            self.state = 'START_ACKED'
            return self._event('STATUS', 'start_command_acknowledged')
        return self._event('STATUS', 'start_command_acknowledged_already')

    def _accept_finish_status(self, status):
        if status['motion_name'] == 'start_jump':
            return self._fault('finish_stage_mapped_to_start_jump')
        if status['motion_name'] not in ('', 'finish_jump'):
            return self._event('STATUS', 'finish_stage_motion_not_acknowledged')
        if status['state'] == 'FINISH_COMPLETED':
            self._clear_pending()
            self.finish_completed = True
            self.state = 'COMPLETED'
            self.requested_stage = 'FINISH'
            return self._event('STATUS', 'finish_completed')
        if self.state == 'FINISH_PENDING':
            self._clear_pending()
            self.state = 'FINISH_ACKED'
            return self._event('STATUS', 'finish_command_acknowledged')
        return self._event('STATUS', 'finish_command_acknowledged_already')

    def _is_start_status(self, status):
        state = status['state']
        if state not in ('START_ARMED', 'START_RUNNING', 'START_COMPLETED'):
            return False
        if status['last_sequence'] < 1:
            return False
        return state == 'START_COMPLETED' or status['active_stage'] == 'START'

    def _is_finish_status(self, status):
        state = status['state']
        if state not in ('FINISH_ARMED', 'FINISH_RUNNING', 'FINISH_COMPLETED'):
            return False
        if status['last_sequence'] < 2:
            return False
        return state == 'FINISH_COMPLETED' or status['active_stage'] == 'FINISH'

    def _begin_command(self, stage, sequence, reason):
        self.sequence = sequence
        self.requested_stage = stage
        self.state = f'{stage}_PENDING'
        self.retry_count = 0
        self._pending_started_at = self._now
        self._pending_last_sent_at = self._now
        return self._event('SEND_COMMAND', reason, self._command_payload())

    def _command_payload(self):
        return {
            'run_id': self.run_id,
            'sequence': self.sequence,
            'stage': self.requested_stage,
        }

    def _clear_pending(self):
        self._pending_started_at = None
        self._pending_last_sent_at = None
        self.retry_count = 0

    def _fault(self, reason):
        self.state = 'FAULTED'
        self._clear_pending()
        return self._event('FAULT', reason)

    def _reset_for_new_mission(self):
        self.state = 'WAIT_RUN'
        self.run_id = ''
        self.sequence = 0
        self.requested_stage = ''
        self.retry_count = 0
        self.start_completed = False
        self.finish_milestone_seen = False
        self.finish_completed = False
        self._last_stage_sequence = 0
        self._pending_started_at = None
        self._pending_last_sent_at = None

    def _reset_to_idle(self):
        self.state = 'IDLE'
        self.run_id = ''
        self.sequence = 0
        self.requested_stage = ''
        self.retry_count = 0
        self.start_completed = False
        self.finish_milestone_seen = False
        self.finish_completed = False
        self._last_stage_sequence = 0
        self._pending_started_at = None
        self._pending_last_sent_at = None
        self._now = 0.0

    def _event(self, action, reason, command_payload=None):
        return WhiteBarStageCommandEvent(
            action=action,
            command_payload=command_payload,
            run_id=self.run_id,
            sequence=self.sequence,
            requested_stage=self.requested_stage,
            state=self.state,
            reason=reason,
            retry_count=self.retry_count,
            start_completed=self.start_completed,
            finish_milestone_seen=self.finish_milestone_seen,
            finish_completed=self.finish_completed,
        )

    def _set_now(self, now):
        value = float(now)
        if not math.isfinite(value):
            raise ValueError('now must be finite')
        self._now = value

    @staticmethod
    def _positive_float(value, name):
        result = float(value)
        if not math.isfinite(result) or result <= 0.0:
            raise ValueError(f'{name} must be finite and positive')
        return result

    @staticmethod
    def _validated_stage_status(payload):
        if type(payload) is not dict:
            return None
        required = (
            'run_id',
            'state',
            'last_sequence',
            'active_stage',
            'motion_name',
            'request_sent',
            'action_done',
        )
        if any(name not in payload for name in required):
            return None
        if (
            type(payload['run_id']) is not str
            or type(payload['state']) is not str
            or type(payload['last_sequence']) is not int
            or payload['last_sequence'] < 0
            or type(payload['active_stage']) is not str
            or type(payload['motion_name']) is not str
            or type(payload['request_sent']) is not bool
            or type(payload['action_done']) is not bool
        ):
            return None
        return payload

    @staticmethod
    def _validated_line_course_state(payload):
        if type(payload) is not dict:
            return None
        required = ('state', 'mission_started', 'white_bar_stage_run_id')
        if any(name not in payload for name in required):
            return None
        if (
            type(payload['state']) is not str
            or type(payload['mission_started']) is not bool
            or type(payload['white_bar_stage_run_id']) is not str
        ):
            return None
        return payload
