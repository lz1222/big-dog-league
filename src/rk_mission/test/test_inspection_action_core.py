"""Unit tests for the request-driven warning-sign action safety core."""

import os
import threading

import pytest

from rk_mission.inspection_action_core import ARMED
from rk_mission.inspection_action_core import CANCELED
from rk_mission.inspection_action_core import CLEANUP_PENDING
from rk_mission.inspection_action_core import COMMAND_READY
from rk_mission.inspection_action_core import FAILED
from rk_mission.inspection_action_core import FAULTED
from rk_mission.inspection_action_core import InspectionActionConfig
from rk_mission.inspection_action_core import InspectionActionCore
from rk_mission.inspection_action_core import InspectionActionRequest
from rk_mission.inspection_action_core import ProcessGroupHelperRunner
from rk_mission.inspection_action_core import RUNNING
from rk_mission.inspection_action_core import SUCCEEDED
from rk_mission.inspection_action_core import TIMEOUT
from rk_mission.inspection_action_core import WAIT_SIGN
from rk_mission.inspection_action_core import WAIT_ZERO
from rk_mission.inspection_action_core import all_velocity_values_zero
from rk_mission.inspection_action_core import decode_inspection_request
from rk_mission.inspection_action_core import warning_action_for


RUN_ID = 'run-001'
REQUEST_ID = 'inspection-001'


def _request(run_id=RUN_ID, request_id=REQUEST_ID):
    return InspectionActionRequest(
        run_id=run_id,
        request_id=request_id,
        action='detect_and_execute_warning',
    )


def _core(**config_overrides):
    values = {
        'sign_confirm_frames': 3,
        'sign_min_confidence': 0.70,
        'sign_wait_timeout_sec': 2.0,
        'final_zero_epsilon': 0.001,
        'final_zero_confirm_samples': 3,
        'final_zero_timeout_sec': 2.0,
        'final_cmd_stale_timeout_sec': 0.50,
        'estop_state_stale_timeout_sec': 0.50,
    }
    values.update(config_overrides)
    return InspectionActionCore(InspectionActionConfig(**values))


def _armed_core(**config_overrides):
    core = _core(**config_overrides)
    request = core.request(_request(), 0.0)
    arm = core.arm(request.run_id, request.request_id, 0.0)
    assert request.state == ARMED
    assert arm.state == WAIT_SIGN
    return core


def _stable_sign(core, now=0.1):
    event = None
    for frame in range(core.config.sign_confirm_frames):
        event = core.observe_detection(
            'warning', 'electric_shock', 0.90, now + frame * 0.01
        )
    assert event is not None
    assert event.state == COMMAND_READY
    assert event.acquire_gait_lock
    return event


def _running_core(**config_overrides):
    core = _armed_core(**config_overrides)
    command = _stable_sign(core)
    waiting = core.lock_acquired(command.run_id, command.request_id, 0.2)
    assert waiting.state == WAIT_ZERO
    assert core.observe_estop(False, 0.21) is None
    event = None
    for index in range(core.config.final_zero_confirm_samples):
        event = core.observe_final_cmd(True, 0.22 + index * 0.01)
    assert event is not None
    assert event.state == RUNNING
    assert event.start_helper
    assert core.helper_launching(event.run_id, event.request_id)
    return core, event


@pytest.mark.parametrize(
    'sign_value,sdk_action',
    [
        ('electric_shock', 'stretch'),
        ('strong_oxidizer', 'hello'),
        ('radiation', 'blink_front_light_3'),
    ],
)
def test_exact_warning_mappings_have_no_fallback(sign_value, sdk_action):
    assert warning_action_for('warning', sign_value) == sdk_action


@pytest.mark.parametrize(
    'sign_type,sign_value',
    [
        ('warning', 'unknown_warning'),
        ('place_marker', 'place_1'),
        ('warning', ''),
        ('', 'electric_shock'),
    ],
)
def test_unknown_warning_mappings_are_rejected_without_fallback(
    sign_type, sign_value
):
    assert warning_action_for(sign_type, sign_value) is None


def test_request_payload_requires_both_ids_and_explicit_action():
    decoded = decode_inspection_request(
        '{"run_id":"run-a","request_id":"request-a",'
        '"action":"detect_and_execute_warning"}'
    )

    assert decoded.run_id == 'run-a'
    assert decoded.request_id == 'request-a'
    with pytest.raises(ValueError):
        decode_inspection_request('{"run_id":"run-a"}')
    with pytest.raises(ValueError):
        decode_inspection_request(
            '{"run_id":"run-a","request_id":"request-a",'
            '"action":"stretch"}'
        )


def test_detection_before_request_cannot_arm_an_action():
    core = _core()

    event = core.observe_detection('warning', 'electric_shock', 0.95, 0.0)

    assert event is None
    assert core.state == 'IDLE'
    assert core.candidate_count == 0


def test_stable_detection_requires_same_valid_value_on_consecutive_frames():
    core = _armed_core()

    assert core.observe_detection(
        'warning', 'electric_shock', 0.90, 0.1
    ) is None
    assert core.observe_detection('warning', 'radiation', 0.90, 0.2) is None
    assert core.candidate_count == 1
    assert core.observe_detection(
        'warning', 'electric_shock', 0.60, 0.3
    ) is None
    assert core.candidate_count == 0
    event = _stable_sign(core, now=0.4)

    assert event.sign_value == 'electric_shock'
    assert event.sdk_action == 'stretch'
    assert core.state == COMMAND_READY


def test_gait_lock_precedes_new_final_zero_samples_and_fresh_estop_gate():
    core = _armed_core()
    command = _stable_sign(core)
    # 锁前的零样本绝不能拿来满足锁后的确认门。
    assert core.observe_final_cmd(True, 0.15) is None
    waiting = core.lock_acquired(command.run_id, command.request_id, 0.2)

    assert waiting.state == WAIT_ZERO
    assert core.final_zero_streak == 0
    assert core.observe_estop(False, 0.21) is None
    assert core.observe_final_cmd(True, 0.22) is None
    assert core.observe_final_cmd(True, 0.23) is None
    event = core.observe_final_cmd(True, 0.24)

    assert event.state == RUNNING
    assert event.start_helper
    assert core.gait_lock_held


def test_stale_final_cmd_or_estop_never_starts_helper():
    core = _armed_core(final_cmd_stale_timeout_sec=0.05)
    command = _stable_sign(core)
    core.lock_acquired(command.run_id, command.request_id, 0.2)
    core.observe_final_cmd(True, 0.22)
    core.observe_final_cmd(True, 0.23)
    assert core.observe_final_cmd(True, 0.24) is None
    # estop 变为 false 时，旧最终零速度已经过期，不能补发 SDK helper。
    assert core.observe_estop(False, 0.40) is None
    assert core.state == WAIT_ZERO

    core = _armed_core(estop_state_stale_timeout_sec=0.05)
    command = _stable_sign(core)
    core.lock_acquired(command.run_id, command.request_id, 0.2)
    core.observe_estop(False, 0.21)
    assert core.observe_final_cmd(True, 0.30) is None
    assert core.observe_final_cmd(True, 0.31) is None
    assert core.observe_final_cmd(True, 0.32) is None
    assert core.state == WAIT_ZERO


def test_final_zero_timeout_reports_missing_estop_and_releases_lock():
    core = _armed_core(final_zero_timeout_sec=0.20)
    command = _stable_sign(core)
    core.lock_acquired(command.run_id, command.request_id, 0.2)

    event = core.tick(0.41)

    assert event.state == TIMEOUT
    assert event.reason == 'final_zero_timeout_estop_state_missing'
    assert event.release_gait_lock
    assert not core.gait_lock_held


def test_estop_active_is_faulted_before_sdk_start():
    core = _armed_core()
    command = _stable_sign(core)
    core.lock_acquired(command.run_id, command.request_id, 0.2)

    event = core.observe_estop(True, 0.21)

    assert event.state == FAULTED
    assert event.reason == 'estop_active'
    assert event.release_gait_lock
    assert not event.start_helper


def test_wrong_run_or_request_id_cannot_complete_current_action():
    core, event = _running_core()

    assert core.helper_finished(
        'other-run', event.request_id, SUCCEEDED, 'old',
        cleanup_completed=True,
    ) is None
    assert core.helper_finished(
        event.run_id, 'other-request', SUCCEEDED, 'old',
        cleanup_completed=True,
    ) is None
    assert core.state == RUNNING
    result = core.helper_finished(
        event.run_id, event.request_id, SUCCEEDED, 'sdk_ok',
        cleanup_completed=True,
    )

    assert result.state == SUCCEEDED
    assert result.success
    assert result.release_gait_lock


def test_mission_stop_waits_for_running_helper_cleanup_before_unlock():
    core, event = _running_core()

    canceled = core.mission_stop()

    assert canceled.state == CLEANUP_PENDING
    assert canceled.terminate_helper
    assert not canceled.release_gait_lock
    assert core.active
    assert core.gait_lock_held
    cleaned = core.helper_finished(
        event.run_id,
        event.request_id,
        CANCELED,
        'helper_cancel_requested',
        cleanup_completed=True,
    )

    assert cleaned.state == CANCELED
    assert cleaned.release_gait_lock
    assert not core.gait_lock_held


def test_unverified_helper_cleanup_faults_and_keeps_gait_lock():
    core, event = _running_core()

    result = core.helper_finished(
        event.run_id,
        event.request_id,
        FAILED,
        'helper_process_group_cleanup_failed',
        cleanup_completed=False,
    )

    assert result.state == FAULTED
    assert not result.release_gait_lock
    assert 'cleanup_unverified_lock_held' in result.reason
    assert core.gait_lock_held


def test_nonzero_final_cmd_during_sdk_action_aborts_helper():
    core, _ = _running_core()

    event = core.observe_final_cmd(False, 0.5)

    assert event.state == CLEANUP_PENDING
    assert event.terminate_helper
    assert not event.release_gait_lock


def test_duplicate_request_and_terminal_replay_are_ignored():
    core = _armed_core()

    duplicate = core.request(_request(), 0.1)

    assert duplicate.reason == 'duplicate_request_ignored'
    assert duplicate.state == WAIT_SIGN
    command = _stable_sign(core)
    core.lock_acquired(command.run_id, command.request_id, 0.2)
    terminal = core.mission_stop()
    duplicate_terminal = core.request(_request(), 0.3)

    assert terminal.state == CANCELED
    assert duplicate_terminal.reason == 'duplicate_terminal_request_ignored'


def test_all_final_twist_components_must_be_finite_zero():
    assert all_velocity_values_zero((0.0,) * 6, 0.001)
    assert not all_velocity_values_zero(
        (0.0, 0.0, 0.0, 0.0, 0.01, 0.0), 0.001
    )
    assert not all_velocity_values_zero(
        (0.0, 0.0, 0.0, float('nan'), 0.0, 0.0), 0.001
    )


def test_helper_timeout_cleans_entire_process_group(tmp_path):
    # /bin/sh 是 ELF，runner 可验证 argv[0] 身份；脚本的后台 sleep 用来
    # 验证 timeout 不只 wait 直接子进程，而会回收同一进程组的子孙。
    script = tmp_path / 'spawn_child.sh'
    script.write_text('sleep 30 &\nwait\n', encoding='utf-8')
    os.chmod(script, 0o700)
    runner = ProcessGroupHelperRunner(
        poll_interval_sec=0.02,
        terminate_grace_sec=0.20,
        kill_grace_sec=0.20,
    )

    result = runner.run(
        ['/bin/sh', str(script)],
        0.10,
        threading.Event(),
    )

    assert result.terminal_state == TIMEOUT
    assert result.cleanup_completed


def test_helper_cancel_cleans_process_group(tmp_path):
    script = tmp_path / 'wait_forever.sh'
    script.write_text('sleep 30 &\nwait\n', encoding='utf-8')
    os.chmod(script, 0o700)
    cancel = threading.Event()
    cancel.set()
    runner = ProcessGroupHelperRunner(
        poll_interval_sec=0.02,
        terminate_grace_sec=0.20,
        kill_grace_sec=0.20,
    )

    result = runner.run(
        ['/bin/sh', str(script)],
        5.0,
        cancel,
    )

    assert result.terminal_state == CANCELED
    assert result.cleanup_completed


def test_helper_nonzero_exit_is_not_success(tmp_path):
    script = tmp_path / 'fail.sh'
    # 给 runner 留出读取 /proc 身份的窗口；本例要验证的是已识别 helper 的
    # 非零返回码，而不是“进程过早退出”这一独立的 FAULTED 保护分支。
    script.write_text('sleep 0.1\nexit 7\n', encoding='utf-8')
    os.chmod(script, 0o700)
    runner = ProcessGroupHelperRunner()

    result = runner.run(['/bin/sh', str(script)], 1.0, threading.Event())

    assert result.terminal_state == FAILED
    assert result.return_code == 7
