"""ROS-independent safety state machines for white-bar actions."""

from dataclasses import dataclass


ALLOWED_WHITE_BAR_MOTIONS = frozenset(('start_jump', 'finish_jump'))


def normalize_motion_name(value):
    """Return the configured motion name without accepting aliases."""
    return str(value or '').strip()


def is_allowed_white_bar_motion(value):
    """Return whether a motion is explicitly approved for a white bar."""
    return normalize_motion_name(value) in ALLOWED_WHITE_BAR_MOTIONS


@dataclass(frozen=True)
class WhiteBarActionEvent:
    """One state-machine decision for the ROS action adapter."""

    status: str
    motion_name: str
    reason: str
    request_id: int = 0
    send_goal: bool = False
    cancel_goal: bool = False
    publish_done: bool = False


class WhiteBarActionExecutorCore:
    """Track one ExecuteMotion request without depending on rclpy."""

    def __init__(self):
        self.status = 'IDLE'
        self.motion_name = ''
        self.request_id = 0
        self.active = False

    def request(self, requested_motion_name):
        """Accept one approved request, or reject/ignore it safely."""
        motion_name = normalize_motion_name(requested_motion_name)
        if self.active:
            if motion_name == self.motion_name:
                reason = 'duplicate_request_ignored'
            else:
                reason = 'request_ignored_while_action_active'
            return self._event(reason=reason)

        if motion_name not in ALLOWED_WHITE_BAR_MOTIONS:
            self.status = 'FAILED'
            self.motion_name = motion_name
            return self._event(reason='unsupported_motion_name')

        self.request_id += 1
        self.motion_name = motion_name
        self.status = 'WAIT_SERVER'
        self.active = True
        return self._event(reason='waiting_for_action_server')

    def server_ready(self, request_id):
        """Advance a pending request exactly once when the server is ready."""
        if not self._matches(request_id, 'WAIT_SERVER'):
            return None
        self.status = 'GOAL_SENT'
        return self._event(reason='execute_motion_goal_sent', send_goal=True)

    def goal_accepted(self, request_id):
        """Record that the action server accepted the current goal."""
        if not self._matches(request_id, 'GOAL_SENT'):
            return None
        self.status = 'RUNNING'
        return self._event(reason='execute_motion_goal_accepted')

    def goal_rejected(self, request_id, reason='execute_motion_goal_rejected'):
        """Fail the active request when its goal cannot run."""
        if not self._matches(request_id, 'GOAL_SENT'):
            return None
        return self._finish('FAILED', reason)

    def goal_send_failed(self, request_id, reason='execute_motion_goal_send_failed'):
        """Fail the active request when sending the goal raises an error."""
        if not self._matches(request_id, 'GOAL_SENT'):
            return None
        return self._finish('FAILED', reason)

    def action_result(
        self,
        request_id,
        action_completed_normally,
        result_success,
        reason=''
    ):
        """Publish completion permission only for a successful action result."""
        if not self._matches(request_id, 'RUNNING'):
            return None
        if action_completed_normally and result_success:
            return self._finish(
                'SUCCEEDED',
                reason or 'execute_motion_result_success',
                publish_done=True
            )
        if not action_completed_normally:
            reason = reason or 'execute_motion_not_succeeded'
        else:
            reason = reason or 'execute_motion_result_success_false'
        return self._finish('FAILED', reason)

    def timeout(self, request_id, reason='execute_motion_timeout'):
        """Time out and request cancellation of the active goal."""
        if not self._matches(request_id):
            return None
        return self._finish('TIMEOUT', reason, cancel_goal=True)

    def mission_stop(self):
        """Cancel any active action as soon as the mission stops."""
        if not self.active:
            self.status = 'IDLE'
            self.motion_name = ''
            return self._event(reason='mission_stop_no_active_action')
        return self._finish(
            'CANCELED',
            'mission_stop_cancel_requested',
            cancel_goal=True
        )

    def _matches(self, request_id, required_status=None):
        if not self.active or request_id != self.request_id:
            return False
        return required_status is None or self.status == required_status

    def _finish(self, status, reason, cancel_goal=False, publish_done=False):
        self.status = status
        self.active = False
        return self._event(
            reason=reason,
            cancel_goal=cancel_goal,
            publish_done=publish_done
        )

    def _event(self, reason, send_goal=False, cancel_goal=False, publish_done=False):
        return WhiteBarActionEvent(
            status=self.status,
            motion_name=self.motion_name,
            reason=reason,
            request_id=self.request_id,
            send_goal=send_goal,
            cancel_goal=cancel_goal,
            publish_done=publish_done,
        )


@dataclass(frozen=True)
class WhiteBarMissionEvent:
    """One request-gate decision for LineCourseMissionNode."""

    action: str
    motion_name: str
    reason: str


class WhiteBarActionRequestGate:
    """Require threshold arrival and explicit motion config before requesting."""

    def __init__(self, motion_name=''):
        self.motion_name = normalize_motion_name(motion_name)
        self.request_sent = False
        self.action_done = False

    def reset(self, motion_name=None):
        """Clear a prior request and any stale completion indication."""
        if motion_name is not None:
            self.motion_name = normalize_motion_name(motion_name)
        self.request_sent = False
        self.action_done = False

    def evaluate(self, stop_threshold_reached):
        """Return the next safe transition for the current white-bar state."""
        if self.action_done:
            return WhiteBarMissionEvent(
                'COMPLETE', self.motion_name, 'white_bar_action_done'
            )
        if not stop_threshold_reached:
            return WhiteBarMissionEvent(
                'APPROACH', self.motion_name, 'white_bar_stop_threshold_not_reached'
            )
        if not is_allowed_white_bar_motion(self.motion_name):
            return WhiteBarMissionEvent(
                'CONFIG_ERROR', self.motion_name, 'white_bar_motion_not_configured'
            )
        if self.request_sent:
            return WhiteBarMissionEvent(
                'WAIT_RESULT', self.motion_name, 'white_bar_action_request_pending'
            )
        self.request_sent = True
        return WhiteBarMissionEvent(
            'SEND_REQUEST', self.motion_name, 'white_bar_action_requested'
        )

    def accept_done(self, done):
        """Accept success only after this gate has sent a request."""
        if not done or not self.request_sent:
            return False
        self.action_done = True
        return True
