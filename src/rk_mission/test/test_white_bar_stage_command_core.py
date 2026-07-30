from pathlib import Path

from rk_mission.white_bar_stage_command_core import (
    WhiteBarStageCommandSequencer,
)
from rk_mission.white_bar_stage_command_publisher_node import (
    decode_json_object,
)


RUN_ID = 'run-001'


def _sequencer(**kwargs):
    return WhiteBarStageCommandSequencer(**kwargs)


def _stage_status(
    run_id=RUN_ID,
    state='DISARMED',
    last_sequence=0,
    active_stage='',
    motion_name='',
    request_sent=False,
    action_done=False,
):
    return {
        'run_id': run_id,
        'state': state,
        'last_sequence': last_sequence,
        'active_stage': active_stage,
        'motion_name': motion_name,
        'request_sent': request_sent,
        'action_done': action_done,
    }


def _line_state(
    state='LINE_FOLLOW',
    mission_started=True,
    run_id=RUN_ID,
):
    return {
        'state': state,
        'mission_started': mission_started,
        'white_bar_stage_run_id': run_id,
    }


def _start_pending(sequencer, now=0.0):
    assert sequencer.mission_start(now).state == 'WAIT_RUN'
    event = sequencer.on_stage_status(_stage_status(), now)
    assert event.action == 'SEND_COMMAND'
    assert event.command_payload == {
        'run_id': RUN_ID,
        'sequence': 1,
        'stage': 'START',
    }
    return event


def _start_completed(sequencer, now=0.0):
    _start_pending(sequencer, now)
    event = sequencer.on_stage_status(
        _stage_status(
            state='START_COMPLETED',
            last_sequence=1,
            active_stage='START',
            motion_name='start_jump',
            request_sent=True,
            action_done=True,
        ),
        now + 0.1,
    )
    assert event.start_completed
    return event


def _finish_pending(sequencer, now=0.0):
    _start_completed(sequencer, now)
    event = sequencer.on_line_course_state(
        _line_state(state='TURN_AFTER_RED'),
        now + 0.2,
    )
    assert event.action == 'SEND_COMMAND'
    assert event.command_payload['stage'] == 'FINISH'
    return event


def test_initial_state_is_idle():
    sequencer = _sequencer()

    assert sequencer.state == 'IDLE'
    assert sequencer.run_id == ''


def test_mission_start_enters_wait_run_without_a_command():
    sequencer = _sequencer()

    event = sequencer.mission_start(1.0)

    assert event.action == 'STATUS'
    assert event.state == 'WAIT_RUN'
    assert event.command_payload is None


def test_stage_status_before_mission_start_is_ignored():
    sequencer = _sequencer()

    event = sequencer.on_stage_status(_stage_status())

    assert event.action == 'STATUS'
    assert event.reason == 'stage_status_ignored_not_started'
    assert event.state == 'IDLE'


def test_disarmed_new_run_sends_start_with_sequence_one():
    sequencer = _sequencer()

    event = _start_pending(sequencer)

    assert event.run_id == RUN_ID
    assert event.sequence == 1
    assert event.requested_stage == 'START'
    assert event.state == 'START_PENDING'


def test_start_is_sent_only_once_before_a_retry_is_due():
    sequencer = _sequencer()
    _start_pending(sequencer)

    duplicate = sequencer.on_stage_status(_stage_status(), 0.1)
    timer = sequencer.on_timer(0.49)

    assert duplicate.action == 'STATUS'
    assert timer.action == 'NONE'
    assert sequencer.sequence == 1


def test_unacknowledged_start_retries_same_command_and_sequence():
    sequencer = _sequencer(command_retry_sec=0.5)
    _start_pending(sequencer)

    event = sequencer.on_timer(0.5)

    assert event.action == 'SEND_COMMAND'
    assert event.command_payload == {
        'run_id': RUN_ID,
        'sequence': 1,
        'stage': 'START',
    }
    assert event.retry_count == 1


def test_retry_never_increments_sequence():
    sequencer = _sequencer(command_retry_sec=0.5)
    _start_pending(sequencer)
    sequencer.on_timer(0.5)

    event = sequencer.on_timer(1.0)

    assert event.command_payload['sequence'] == 1
    assert event.retry_count == 2


def test_start_armed_acknowledges_and_stops_retries():
    sequencer = _sequencer()
    _start_pending(sequencer)

    event = sequencer.on_stage_status(
        _stage_status(
            state='START_ARMED',
            last_sequence=1,
            active_stage='START',
            motion_name='start_jump',
        ),
        0.1,
    )
    timer = sequencer.on_timer(1.0)

    assert event.state == 'START_ACKED'
    assert timer.action == 'NONE'
    assert timer.state == 'START_ACKED'


def test_start_running_acknowledges_and_stops_retries():
    sequencer = _sequencer()
    _start_pending(sequencer)

    event = sequencer.on_stage_status(
        _stage_status(
            state='START_RUNNING',
            last_sequence=1,
            active_stage='START',
            motion_name='start_jump',
            request_sent=True,
        ),
        0.1,
    )

    assert event.state == 'START_ACKED'
    assert sequencer.on_timer(1.0).action == 'NONE'


def test_start_completed_sets_the_completion_flag():
    sequencer = _sequencer()

    event = _start_completed(sequencer)

    assert event.start_completed
    assert event.state == 'WAIT_FINISH_MILESTONE'


def test_start_completion_without_red_milestone_does_not_send_finish():
    sequencer = _sequencer()

    event = _start_completed(sequencer)

    assert event.action == 'STATUS'
    assert event.state == 'WAIT_FINISH_MILESTONE'
    assert event.requested_stage == ''


def test_early_red_milestone_waits_for_start_completion():
    sequencer = _sequencer()
    _start_pending(sequencer)

    milestone = sequencer.on_line_course_state(
        _line_state(state='TURN_AFTER_RED'),
        0.1,
    )
    completed = sequencer.on_stage_status(
        _stage_status(
            state='START_COMPLETED',
            last_sequence=1,
            active_stage='START',
            motion_name='start_jump',
            request_sent=True,
            action_done=True,
        ),
        0.2,
    )

    assert milestone.action == 'STATUS'
    assert milestone.finish_milestone_seen
    assert completed.action == 'SEND_COMMAND'
    assert completed.command_payload['stage'] == 'FINISH'


def test_start_completion_then_red_milestone_sends_finish_sequence_two():
    sequencer = _sequencer()
    _start_completed(sequencer)

    event = sequencer.on_line_course_state(
        _line_state(state='TURN_AFTER_RED'),
        0.2,
    )

    assert event.action == 'SEND_COMMAND'
    assert event.command_payload == {
        'run_id': RUN_ID,
        'sequence': 2,
        'stage': 'FINISH',
    }


def test_reacquire_line_never_triggers_finish():
    sequencer = _sequencer()
    _start_completed(sequencer)

    event = sequencer.on_line_course_state(
        _line_state(state='REACQUIRE_LINE'),
        0.2,
    )

    assert event.action == 'STATUS'
    assert event.state == 'WAIT_FINISH_MILESTONE'


def test_finish_armed_acknowledges_and_stops_retries():
    sequencer = _sequencer()
    _finish_pending(sequencer)

    event = sequencer.on_stage_status(
        _stage_status(
            state='FINISH_ARMED',
            last_sequence=2,
            active_stage='FINISH',
            motion_name='finish_jump',
        ),
        0.3,
    )

    assert event.state == 'FINISH_ACKED'
    assert sequencer.on_timer(1.0).action == 'NONE'


def test_finish_running_acknowledges_and_stops_retries():
    sequencer = _sequencer()
    _finish_pending(sequencer)

    event = sequencer.on_stage_status(
        _stage_status(
            state='FINISH_RUNNING',
            last_sequence=2,
            active_stage='FINISH',
            motion_name='finish_jump',
            request_sent=True,
        ),
        0.3,
    )

    assert event.state == 'FINISH_ACKED'
    assert sequencer.on_timer(1.0).action == 'NONE'


def test_finish_completed_enters_completed():
    sequencer = _sequencer()
    _finish_pending(sequencer)

    event = sequencer.on_stage_status(
        _stage_status(
            state='FINISH_COMPLETED',
            last_sequence=2,
            active_stage='FINISH',
            motion_name='finish_jump',
            request_sent=True,
            action_done=True,
        ),
        0.3,
    )

    assert event.state == 'COMPLETED'
    assert event.finish_completed


def test_other_stage_run_id_faults_after_locking_a_run():
    sequencer = _sequencer()
    _start_pending(sequencer)

    event = sequencer.on_stage_status(
        _stage_status(run_id='run-other'),
        0.1,
    )

    assert event.action == 'FAULT'
    assert event.reason == 'stage_status_run_id_mismatch'


def test_start_status_with_finish_jump_faults():
    sequencer = _sequencer()
    _start_pending(sequencer)

    event = sequencer.on_stage_status(
        _stage_status(
            state='START_ARMED',
            last_sequence=1,
            active_stage='START',
            motion_name='finish_jump',
        ),
        0.1,
    )

    assert event.action == 'FAULT'
    assert event.reason == 'start_stage_mapped_to_finish_jump'


def test_finish_status_with_start_jump_faults():
    sequencer = _sequencer()
    _finish_pending(sequencer)

    event = sequencer.on_stage_status(
        _stage_status(
            state='FINISH_ARMED',
            last_sequence=2,
            active_stage='FINISH',
            motion_name='start_jump',
        ),
        0.3,
    )

    assert event.action == 'FAULT'
    assert event.reason == 'finish_stage_mapped_to_start_jump'


def test_stage_status_sequence_rollback_faults():
    sequencer = _sequencer()
    _start_pending(sequencer)
    sequencer.on_stage_status(
        _stage_status(
            state='START_ARMED',
            last_sequence=1,
            active_stage='START',
            motion_name='start_jump',
        ),
        0.1,
    )

    event = sequencer.on_stage_status(
        _stage_status(
            state='START_COMPLETED',
            last_sequence=0,
            active_stage='START',
            motion_name='start_jump',
        ),
        0.2,
    )

    assert event.action == 'FAULT'
    assert event.reason == 'stage_status_sequence_rollback'


def test_stage_faulted_faults_the_sequencer():
    sequencer = _sequencer()
    _start_pending(sequencer)

    event = sequencer.on_stage_status(
        _stage_status(state='FAULTED', last_sequence=1),
        0.1,
    )

    assert event.state == 'FAULTED'
    assert event.reason == 'white_bar_stage_faulted'


def test_line_course_emergency_stop_faults_the_sequencer():
    sequencer = _sequencer()
    _start_pending(sequencer)

    event = sequencer.on_line_course_state(
        _line_state(state='EMERGENCY_STOP'),
        0.1,
    )

    assert event.action == 'FAULT'
    assert event.reason == 'line_course_emergency_stop'


def test_final_stop_before_finish_completion_faults_the_sequencer():
    sequencer = _sequencer()
    _start_pending(sequencer)

    event = sequencer.on_line_course_state(
        _line_state(state='FINAL_STOP'),
        0.1,
    )

    assert event.action == 'FAULT'
    assert event.reason == 'line_course_final_stop_before_finish_completed'


def test_start_ack_timeout_faults():
    sequencer = _sequencer(
        command_retry_sec=10.0,
        command_ack_timeout_sec=5.0,
    )
    _start_pending(sequencer)

    event = sequencer.on_timer(5.0)

    assert event.action == 'FAULT'
    assert event.reason == 'command_ack_timeout'


def test_finish_ack_timeout_faults():
    sequencer = _sequencer(
        command_retry_sec=10.0,
        command_ack_timeout_sec=5.0,
    )
    _finish_pending(sequencer)

    event = sequencer.on_timer(5.2)

    assert event.action == 'FAULT'
    assert event.reason == 'command_ack_timeout'


def test_exceeding_maximum_retries_faults():
    sequencer = _sequencer(
        command_retry_sec=0.5,
        command_ack_timeout_sec=5.0,
        max_command_retries=1,
    )
    _start_pending(sequencer)
    assert sequencer.on_timer(0.5).action == 'SEND_COMMAND'

    event = sequencer.on_timer(1.0)

    assert event.action == 'FAULT'
    assert event.reason == 'command_max_retries_exceeded'


def test_mission_stop_clears_a_pending_command_and_old_statuses():
    sequencer = _sequencer()
    _start_pending(sequencer)

    stopped = sequencer.mission_stop(0.1)
    old = sequencer.on_stage_status(_stage_status(), 0.2)

    assert stopped.state == 'IDLE'
    assert stopped.run_id == ''
    assert old.action == 'STATUS'
    assert old.reason == 'stage_status_ignored_not_started'


def test_new_mission_uses_a_new_run_and_sequence_one():
    sequencer = _sequencer()
    _start_pending(sequencer)
    sequencer.mission_stop(0.1)

    sequencer.mission_start(1.0)
    event = sequencer.on_stage_status(
        _stage_status(run_id='run-002'),
        1.1,
    )

    assert event.command_payload == {
        'run_id': 'run-002',
        'sequence': 1,
        'stage': 'START',
    }


def test_invalid_payloads_publish_diagnostics_without_faulting():
    sequencer = _sequencer()
    sequencer.mission_start()

    stage_event = sequencer.on_stage_status({'run_id': RUN_ID})
    line_event = sequencer.on_line_course_state({'state': 'LINE_FOLLOW'})

    assert stage_event.action == 'STATUS'
    assert stage_event.state == 'WAIT_RUN'
    assert line_event.action == 'STATUS'
    assert line_event.state == 'WAIT_RUN'


def test_ros_json_helper_rejects_malformed_or_non_object_payloads():
    assert decode_json_object('{') is None
    assert decode_json_object('[]') is None
    assert decode_json_object('"text"') is None
    assert decode_json_object('{"state":"DISARMED"}') == {
        'state': 'DISARMED'
    }


def test_finish_milestone_parameter_rejects_any_other_state():
    try:
        _sequencer(finish_milestone_state='REACQUIRE_LINE')
    except ValueError as exc:
        assert str(exc) == 'finish_milestone_state must be TURN_AFTER_RED'
    else:
        raise AssertionError('invalid milestone state must fail')


def test_no_white_line_count_selects_a_stage():
    source = Path(__file__).resolve().parents[1] / 'rk_mission'
    command_source = (source / 'white_bar_stage_command_core.py').read_text(
        encoding='utf-8'
    )
    node_source = (
        source / 'white_bar_stage_command_publisher_node.py'
    )

    assert 'white_bar_confirm_count' not in command_source
    assert 'first_white_bar' not in command_source
    assert 'second_white_bar' not in command_source
    assert not node_source.exists() or 'white_bar_confirm_count' not in (
        node_source.read_text(encoding='utf-8')
    )
