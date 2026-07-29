from rk_mission.mission_adapters import MissionAdapter
from rk_mission.mission_types import MissionFailureCode, MissionState, MotionCommand
from rk_mission.national_mission_fsm import NationalMissionFSM


class Adapter(MissionAdapter):
    def __init__(self):
        self.lines = []
        self.commands = []
        self.events = []
        self.requests = []

    def set_line_enabled(self, enabled):
        self.lines.append(enabled)

    def publish_mission_command(self, command):
        self.commands.append(command)

    def release_mission_command(self):
        self.events.append(('release', ''))

    def emit_event(self, label, detail=''):
        self.events.append((label, detail))

    def execute_motion(self, request):
        self.requests.append(request)

    def execute_arm_task(self, request):
        self.requests.append(request)

    def execute_maze_placeholder(self, request):
        self.requests.append(request)


def _fast_fsm(adapter):
    return NationalMissionFSM(adapter, {
        'white_confirm_frames': 2,
        'marker_confirm_frames': 2,
        'zero_confirm_frames': 1,
        'state_timeout_sec': 3.0,
        'final_cmd_stale_sec': 3.0,
        'start_white_min_segment_sec': 0.0,
        'finish_white_min_segment_sec': 0.0,
    })


def test_start_white_requires_confirmation_then_dispatches_fake_jump():
    adapter = Adapter()
    fsm = _fast_fsm(adapter)
    assert fsm.start(0.0)
    fsm.on_final_command(MotionCommand(), 0.01)
    fsm.tick(0.01)
    fsm.tick(0.02)
    assert fsm.state == MissionState.START_WHITE_LINE_CONFIRM
    fsm.on_white_line(True, 0.9, 0.02)
    fsm.on_white_line(True, 0.9, 0.03)
    assert fsm.state == MissionState.START_JUMP
    fsm.on_final_command(MotionCommand(), 0.04)
    fsm.tick(0.04)
    assert adapter.requests[0].task_name == 'start_jump'


def test_invalid_marker_fails_closed_and_estop_has_priority():
    adapter = Adapter()
    fsm = _fast_fsm(adapter)
    fsm.start(0.0)
    fsm._transition(MissionState.PICK_PATTERN_SEARCH, 0.01, 'test')
    fsm.on_pick_marker(3, 0.9, 0.02)
    fsm.on_pick_marker(3, 0.9, 0.03)
    assert fsm.state == MissionState.SAFE_STOP
    assert fsm.context.failure_code == MissionFailureCode.INVALID_TARGET_PLATFORM

    fsm.on_estop(True, 0.04)
    assert fsm.state == MissionState.ESTOP


def test_duplicate_action_result_is_ignored_by_token():
    adapter = Adapter()
    fsm = _fast_fsm(adapter)
    fsm.start(0.0)
    fsm._transition(MissionState.START_JUMP, 0.01, 'test')
    fsm.on_final_command(MotionCommand(), 0.02)
    fsm.tick(0.02)
    fsm.tick(0.03)
    request = adapter.requests[0]
    from rk_mission.mission_types import TaskResult
    fsm.on_action_result(TaskResult(request.token, True), 0.03)
    state_after_first = fsm.state
    fsm.on_action_result(TaskResult(request.token, True), 0.04)
    assert fsm.state == state_after_first
    assert any(event[0] == 'ACTION_DUPLICATE_RESULT' for event in adapter.events)
