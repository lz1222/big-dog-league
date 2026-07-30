from pathlib import Path

import pytest

from rk_mission.white_bar_stage_core import WhiteBarStageController


RUN_ID = 'run-001'


def _controller(allow_finish_only_test=False):
    controller = WhiteBarStageController(allow_finish_only_test)
    event = controller.start_run(RUN_ID)
    assert event.accepted
    return controller


def _command(run_id=RUN_ID, sequence=1, stage='START'):
    return {
        'run_id': run_id,
        'sequence': sequence,
        'stage': stage,
    }


def _start_running(controller):
    assert controller.apply_command(_command()).accepted
    event = controller.white_bar_event(True)
    assert event.action == 'SEND_REQUEST'
    return event


def test_mission_start_is_disarmed_with_a_new_run_id():
    controller = _controller()

    assert controller.state == 'DISARMED'
    assert controller.run_id == RUN_ID
    assert controller.last_sequence == 0
    assert not controller.request_sent
    assert not controller.action_done


def test_unarmed_white_bar_at_threshold_fails_safely():
    controller = _controller()

    event = controller.white_bar_event(True)

    assert event.action == 'NOT_ARMED'
    assert event.reason == 'white_bar_stage_not_armed'
    assert not controller.request_sent


def test_start_command_with_current_run_and_new_sequence_is_accepted():
    controller = _controller()

    event = controller.apply_command(_command())

    assert event.accepted
    assert event.state == 'START_ARMED'
    assert event.last_sequence == 1


def test_start_maps_only_to_start_jump():
    controller = _controller()

    event = controller.apply_command(_command())

    assert event.active_stage == 'START'
    assert event.motion_name == 'start_jump'


def test_start_armed_sends_one_request_only_after_threshold():
    controller = _controller()
    assert controller.apply_command(_command()).accepted

    approach = controller.white_bar_event(False)
    sent = controller.white_bar_event(True)
    duplicate = controller.white_bar_event(True)

    assert approach.action == 'APPROACH'
    assert sent.action == 'SEND_REQUEST'
    assert sent.motion_name == 'start_jump'
    assert duplicate.action == 'WAIT_RESULT'
    assert controller.request_sent


def test_duplicate_start_command_with_same_sequence_is_rejected():
    controller = _controller()
    assert controller.apply_command(_command()).accepted

    event = controller.apply_command(_command())

    assert not event.accepted
    assert event.reason == 'stage_command_sequence_stale'


def test_smaller_sequence_is_rejected():
    controller = _controller()
    assert controller.apply_command(_command(sequence=2)).accepted

    event = controller.apply_command(_command(sequence=1))

    assert not event.accepted
    assert event.reason == 'stage_command_sequence_stale'


def test_wrong_run_id_is_rejected():
    controller = _controller()

    event = controller.apply_command(_command(run_id='another-run'))

    assert not event.accepted
    assert event.reason == 'stage_command_run_id_mismatch'


def test_running_start_cannot_switch_to_finish():
    controller = _controller()
    _start_running(controller)

    event = controller.apply_command(_command(sequence=2, stage='FINISH'))

    assert not event.accepted
    assert event.reason == 'stage_change_while_running'
    assert controller.state == 'START_RUNNING'


def test_start_success_enters_start_completed():
    controller = _controller()
    _start_running(controller)

    event = controller.complete_action(True)

    assert event.accepted
    assert event.state == 'START_COMPLETED'
    assert event.action_done


def test_start_completed_can_explicitly_arm_finish():
    controller = _controller()
    _start_running(controller)
    assert controller.complete_action(True).accepted

    event = controller.apply_command(_command(sequence=2, stage='FINISH'))

    assert event.accepted
    assert event.state == 'FINISH_ARMED'
    assert not event.request_sent
    assert not event.action_done


def test_finish_maps_only_to_finish_jump():
    controller = _controller()
    _start_running(controller)
    assert controller.complete_action(True).accepted

    event = controller.apply_command(_command(sequence=2, stage='FINISH'))

    assert event.active_stage == 'FINISH'
    assert event.motion_name == 'finish_jump'


def test_finish_success_enters_finish_completed():
    controller = _controller()
    _start_running(controller)
    assert controller.complete_action(True).accepted
    assert controller.apply_command(_command(sequence=2, stage='FINISH')).accepted
    assert controller.white_bar_event(True).action == 'SEND_REQUEST'

    event = controller.complete_action(True)

    assert event.accepted
    assert event.state == 'FINISH_COMPLETED'


def test_finish_requires_completed_start_by_default():
    controller = _controller()

    event = controller.apply_command(_command(stage='FINISH'))

    assert not event.accepted
    assert event.reason == 'finish_requires_start_completed'


def test_finish_only_test_mode_allows_explicit_finish_arm():
    controller = _controller(allow_finish_only_test=True)

    event = controller.apply_command(_command(stage='FINISH'))

    assert event.accepted
    assert event.state == 'FINISH_ARMED'


def test_clear_cannot_interrupt_a_running_action():
    controller = _controller()
    _start_running(controller)

    event = controller.apply_command(_command(sequence=2, stage='CLEAR'))

    assert not event.accepted
    assert event.reason == 'stage_change_while_running'
    assert controller.state == 'START_RUNNING'


def test_clear_disarms_an_armed_stage_without_completing_it():
    controller = _controller()
    assert controller.apply_command(_command()).accepted

    event = controller.apply_command(_command(sequence=2, stage='CLEAR'))

    assert event.accepted
    assert event.action == 'CLEARED'
    assert event.state == 'DISARMED'
    assert event.last_sequence == 2


def test_timeout_after_action_request_faults_the_stage():
    controller = _controller()
    _start_running(controller)

    event = controller.action_fault('white_bar_action_timeout')

    assert event.action == 'FAULTED'
    assert event.state == 'FAULTED'


@pytest.mark.parametrize('reason', ('white_bar_action_failed', 'white_bar_action_canceled'))
def test_failed_or_canceled_action_faults_the_stage(reason):
    controller = _controller()
    _start_running(controller)

    event = controller.action_fault(reason)

    assert event.state == 'FAULTED'
    assert event.reason == reason


def test_faulted_stage_never_automatically_retries():
    controller = _controller()
    _start_running(controller)
    assert controller.action_fault('white_bar_action_timeout').state == 'FAULTED'

    event = controller.white_bar_event(True)
    command = controller.apply_command(_command(sequence=2))

    assert event.action == 'FAULTED'
    assert not command.accepted
    assert command.reason == 'stage_faulted_reset_required'


def test_old_done_is_ignored_when_no_current_running_stage_exists():
    controller = _controller()
    assert controller.apply_command(_command()).accepted

    event = controller.complete_action(True)

    assert not event.accepted
    assert event.reason == 'white_bar_done_ignored'
    assert controller.state == 'START_ARMED'


def test_one_run_can_explicitly_complete_start_then_finish():
    controller = _controller()
    _start_running(controller)
    assert controller.complete_action(True).state == 'START_COMPLETED'
    assert controller.apply_command(_command(sequence=2, stage='FINISH')).accepted
    assert controller.white_bar_event(True).motion_name == 'finish_jump'

    event = controller.complete_action(True)

    assert event.state == 'FINISH_COMPLETED'
    assert event.run_id == RUN_ID


def test_invalid_json_and_field_types_are_rejected_without_transition():
    controller = _controller()

    invalid_json = controller.apply_json_command('{')
    invalid_sequence = controller.apply_command(
        _command(sequence=True)
    )
    invalid_stage = controller.apply_command(_command(stage='ARM_START'))

    assert invalid_json.reason == 'stage_command_invalid_json'
    assert invalid_sequence.reason == 'stage_command_invalid_sequence'
    assert invalid_stage.reason == 'stage_command_invalid_stage'
    assert controller.state == 'DISARMED'


def test_reset_clears_completed_stage_and_sequence():
    controller = _controller()
    _start_running(controller)
    assert controller.complete_action(True).accepted

    event = controller.apply_command(_command(sequence=2, stage='RESET'))

    assert event.accepted
    assert event.state == 'DISARMED'
    assert event.last_sequence == 0


def test_line_course_has_no_count_based_white_bar_action_routing():
    mission_source = (
        Path(__file__).resolve().parents[1]
        / 'rk_mission'
        / 'mission_state_machine_node.py'
    ).read_text(encoding='utf-8')

    assert 'white_bar_handled' not in mission_source
    assert 'white_bar_count' not in mission_source
    assert 'jump_count' not in mission_source
    assert 'first_white_bar' not in mission_source
    assert 'second_white_bar' not in mission_source
    assert 'white_bar_motion_name' in mission_source
