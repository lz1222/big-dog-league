from rk_mission.mission_context import MissionContext
from rk_mission.mission_types import InspectionType, MissionFailureCode, MissionState


def test_reset_clears_all_per_run_selection_and_completion_fields():
    context = MissionContext()
    context.target_place_platform = 2
    context.pick_marker_id = 2
    context.inspection_type = InspectionType.RADIATION
    context.inspection_completed = True
    context.transfer_place_completed = True
    context.transfer_pick_completed = True
    context.place_completed = True
    context.start_jump_completed = True
    context.finish_jump_completed = True
    context.fail(MissionFailureCode.ACTION_FAILED, 'injected')
    context.transition(MissionState.ARM_PICK_FAKE, 3.0, 'test')

    context.reset(5.0)

    assert context.current_state == MissionState.WAIT_START
    assert context.target_place_platform is None
    assert context.pick_marker_id is None
    assert context.inspection_type is None
    assert not context.inspection_completed
    assert not context.transfer_place_completed
    assert not context.transfer_pick_completed
    assert not context.place_completed
    assert not context.start_jump_completed
    assert not context.finish_jump_completed
    assert context.failure_code == MissionFailureCode.NONE
    assert context.failure_reason == ''
    assert context.transition_history == []
