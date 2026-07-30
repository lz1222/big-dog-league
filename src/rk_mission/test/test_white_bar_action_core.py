from rk_mission.white_bar_action_core import WhiteBarActionExecutorCore
from rk_mission.white_bar_action_core import WhiteBarActionRequestGate


def _running_core(motion_name='start_jump'):
    core = WhiteBarActionExecutorCore()
    request = core.request(motion_name)
    goal = core.server_ready(request.request_id)
    accepted = core.goal_accepted(request.request_id)
    return core, request, goal, accepted


def test_invalid_motion_name_is_rejected_without_goal():
    core = WhiteBarActionExecutorCore()

    event = core.request('jump')

    assert event.status == 'FAILED'
    assert event.reason == 'unsupported_motion_name'
    assert not event.send_goal
    assert not event.publish_done
    assert not core.active


def test_duplicate_request_sends_only_one_goal():
    core = WhiteBarActionExecutorCore()

    request = core.request('start_jump')
    goal = core.server_ready(request.request_id)
    duplicate = core.request('start_jump')

    assert request.status == 'WAIT_SERVER'
    assert goal.send_goal
    assert duplicate.status == 'GOAL_SENT'
    assert duplicate.reason == 'duplicate_request_ignored'
    assert not duplicate.send_goal


def test_unavailable_server_times_out_without_done():
    core = WhiteBarActionExecutorCore()
    request = core.request('finish_jump')

    event = core.timeout(
        request.request_id,
        'execute_motion_server_unavailable_timeout'
    )

    assert event.status == 'TIMEOUT'
    assert event.cancel_goal
    assert not event.publish_done


def test_rejected_goal_does_not_publish_done():
    core = WhiteBarActionExecutorCore()
    request = core.request('start_jump')
    core.server_ready(request.request_id)

    event = core.goal_rejected(request.request_id)

    assert event.status == 'FAILED'
    assert not event.publish_done


def test_result_success_false_does_not_publish_done():
    core, request, _, _ = _running_core()

    event = core.action_result(
        request.request_id,
        action_completed_normally=True,
        result_success=False
    )

    assert event.status == 'FAILED'
    assert not event.publish_done


def test_result_success_true_publishes_done():
    core, request, _, _ = _running_core('finish_jump')

    event = core.action_result(
        request.request_id,
        action_completed_normally=True,
        result_success=True
    )

    assert event.status == 'SUCCEEDED'
    assert event.publish_done
    assert not core.active


def test_running_action_timeout_enters_timeout():
    core, request, _, _ = _running_core()

    event = core.timeout(request.request_id)

    assert event.status == 'TIMEOUT'
    assert event.cancel_goal
    assert not event.publish_done


def test_mission_stop_cancels_running_action():
    core, _, _, _ = _running_core()

    event = core.mission_stop()

    assert event.status == 'CANCELED'
    assert event.cancel_goal
    assert not event.publish_done


def test_old_done_is_ignored_before_current_request():
    gate = WhiteBarActionRequestGate('start_jump')

    accepted = gate.accept_done(True)

    assert not accepted
    assert not gate.action_done


def test_white_bar_below_stop_threshold_does_not_send_request():
    gate = WhiteBarActionRequestGate('start_jump')

    event = gate.evaluate(stop_threshold_reached=False)

    assert event.action == 'APPROACH'
    assert not gate.request_sent


def test_stop_threshold_sends_one_request_then_waits():
    gate = WhiteBarActionRequestGate('finish_jump')

    first = gate.evaluate(stop_threshold_reached=True)
    second = gate.evaluate(stop_threshold_reached=True)

    assert first.action == 'SEND_REQUEST'
    assert first.motion_name == 'finish_jump'
    assert second.action == 'WAIT_RESULT'
    assert gate.request_sent


def test_empty_white_bar_motion_name_fails_safely_at_threshold():
    gate = WhiteBarActionRequestGate('')

    event = gate.evaluate(stop_threshold_reached=True)

    assert event.action == 'CONFIG_ERROR'
    assert event.reason == 'white_bar_motion_not_configured'
    assert not gate.request_sent
