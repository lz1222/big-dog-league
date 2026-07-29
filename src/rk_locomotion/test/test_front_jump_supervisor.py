"""Deterministic tests for supervised start and finish FrontJump flows."""

import ast
from dataclasses import dataclass
import inspect
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import threading
import time
import uuid

import pytest

import rk_locomotion.front_jump_supervisor as front_jump_module
from rk_locomotion.front_jump_supervisor import FrontJumpConfig
from rk_locomotion.front_jump_supervisor import FrontJumpConfigurationError
from rk_locomotion.front_jump_supervisor import FrontJumpOutcome
from rk_locomotion.front_jump_supervisor import FrontJumpProfile
from rk_locomotion.front_jump_supervisor import FrontJumpSupervisor
from rk_locomotion.front_jump_supervisor import CleanupGuardError
from rk_locomotion.front_jump_supervisor import PersistentCleanupGuard
from rk_locomotion.front_jump_supervisor import ProcessIdentityError
from rk_locomotion.front_jump_supervisor import ProcessStartError
from rk_locomotion.front_jump_supervisor import ProcessResult
from rk_locomotion.front_jump_supervisor import SubprocessRunner
from rk_locomotion.front_jump_supervisor import resolve_sdk_executable


EXPECTED_STAGES = [
    'acquire_gait_lock',
    'publish_locomotion_zero',
    'wait_final_cmd_zero',
    'sdk_front_jump',
    'post_settle',
    'stability_unavailable',
    'supervised_flow_done',
]


def load_lock_publisher_types():
    source_path = (
        Path(__file__).parents[1]
        / 'rk_locomotion'
        / 'gait_control_node.py'
    )
    tree = ast.parse(source_path.read_text(encoding='utf-8'))
    selected = [
        node
        for node in tree.body
        if isinstance(node, (ast.ClassDef,))
        and node.name in {
            'LockPublishResult',
            '_SerializedControlLockPublisher',
        }
    ]

    class FakeBool:
        def __init__(self):
            self.data = False

    namespace = {
        'Bool': FakeBool,
        'dataclass': dataclass,
        'threading': threading,
    }
    code = compile(
        ast.Module(selected, type_ignores=[]),
        str(source_path),
        'exec',
    )
    exec(code, namespace)
    return (
        namespace['LockPublishResult'],
        namespace['_SerializedControlLockPublisher'],
    )


class FakeClock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now

    def advance(self, duration):
        self.now += max(0.001, float(duration))


class FakeProcess:
    _next_pid = 100

    def __init__(
        self,
        clock,
        *,
        return_code=0,
        complete_after=0.0,
        terminate_works=True,
        stdout='sdk stdout',
        stderr='sdk stderr',
    ):
        self.clock = clock
        self.return_code = return_code
        self.complete_after = complete_after
        self.terminate_works = terminate_works
        self.stdout = stdout
        self.stderr = stderr
        self.started_at = clock()
        self.alive = True
        self.terminated = False
        self.killed = False
        self.reaped = False
        self.poll_count = 0
        self.pid = FakeProcess._next_pid
        FakeProcess._next_pid += 1

    def poll(self):
        self.poll_count += 1
        if (
            self.alive
            and self.complete_after is not None
            and self.clock() - self.started_at >= self.complete_after
        ):
            self.alive = False
        return None if self.alive else self.return_code

    def terminate(self):
        self.terminated = True
        if self.terminate_works:
            self.alive = False
            self.return_code = -15

    def kill(self):
        self.killed = True
        self.alive = False
        self.return_code = -9

    def reap(self, timeout):
        del timeout
        if self.alive:
            raise subprocess.TimeoutExpired('fake-helper', 0.0)
        self.reaped = True
        return ProcessResult(
            self.return_code,
            self.stdout,
            self.stderr,
        )


class FakeRunner:
    def __init__(
        self,
        clock,
        *,
        process_factory=None,
        start_error=None,
        on_start=None,
    ):
        self.clock = clock
        self.process_factory = process_factory
        self.start_error = start_error
        self.on_start = on_start
        self.calls = []
        self.start_times = []
        self.processes = []

    def start(self, argv):
        self.calls.append(list(argv))
        self.start_times.append(self.clock())
        if self.start_error is not None:
            raise self.start_error
        if self.process_factory is None:
            process = FakeProcess(self.clock)
        else:
            process = self.process_factory()
        self.processes.append(process)
        if self.on_start is not None:
            self.on_start(process)
        return process


class Harness:
    def __init__(
        self,
        *,
        on_wait=None,
        start_overrides=None,
        finish_overrides=None,
        config_overrides=None,
        runner=None,
        resolver=None,
        interface_resolver=None,
        cleanup_guard=None,
    ):
        self.clock = FakeClock()
        self.events = []
        self.feedback = []
        self.logs = []
        self.wait_count = 0
        self.on_wait = on_wait
        self.runner = runner or FakeRunner(self.clock)

        start_values = self._profile_values('start')
        finish_values = self._profile_values('finish')
        start_values.update(start_overrides or {})
        finish_values.update(finish_overrides or {})
        common_values = {
            'sdk_action_executable': 'fake_sdk_helper',
            'sdk_network_interface': 'fake0',
            'zero_publish_rate_hz': 20.0,
            'final_cmd_stale_timeout': 0.15,
            'estop_state_stale_timeout': 0.15,
        }
        common_values.update(config_overrides or {})

        self.supervisor = FrontJumpSupervisor(
            profiles={
                'start': FrontJumpProfile(**start_values),
                'finish': FrontJumpProfile(**finish_values),
            },
            config=FrontJumpConfig(**common_values),
            publish_lock=self._publish_lock,
            publish_zero=self._publish_zero,
            process_runner=self.runner,
            clock=self.clock,
            waiter=self._wait,
            event_logger=self.logs.append,
            executable_resolver=resolver or (lambda unused: '/fake/helper'),
            interface_index_resolver=interface_resolver
            or (lambda unused: 1),
            cleanup_guard=cleanup_guard,
        )

    @staticmethod
    def _profile_values(name):
        return {
            'name': name,
            'pre_stop_duration': 0.10,
            'final_zero_epsilon': 0.001,
            'final_zero_confirm_samples': 3,
            'final_zero_timeout': 0.75,
            'sdk_timeout': 0.50,
            'post_settle_duration': 0.10,
        }

    def _publish_lock(self, locked):
        self.events.append(('lock', bool(locked), self.clock()))

    def _publish_zero(self):
        self.events.append(('zero', (0.0, 0.0, 0.0), self.clock()))

    def _wait(self, duration):
        self.wait_count += 1
        self.clock.advance(duration)
        if self.on_wait is not None:
            self.on_wait(self, duration)

    def stage(self):
        context = self.supervisor.active_context
        return None if context is None else context.stage

    def fresh_false_estop(self):
        self.supervisor.update_estop(False)

    def zero_final_cmd(self):
        self.supervisor.update_final_command(0.0, 0.0, 0.0)

    def run_success_inputs(self, unused_duration):
        self.fresh_false_estop()
        if self.stage() == 'wait_final_cmd_zero':
            self.zero_final_cmd()

    def run(self, motion_name='start_jump', **kwargs):
        return self.supervisor.run(
            motion_name,
            feedback_callback=lambda step, progress: self.feedback.append(
                (step, progress)
            ),
            **kwargs,
        )


def successful_harness(**kwargs):
    harness = Harness(**kwargs)
    harness.on_wait = (
        lambda current_harness, duration:
        current_harness.run_success_inputs(duration)
    )
    harness.fresh_false_estop()
    return harness


def assert_cleaned(harness, outcome):
    assert outcome.cleanup_completed is True
    assert harness.supervisor.active_context is None
    assert harness.events[-1][0:2] == ('lock', False)
    final_zero_index = max(
        index
        for index, event in enumerate(harness.events)
        if event[0] == 'zero'
    )
    unlock_index = max(
        index
        for index, event in enumerate(harness.events)
        if event[0:2] == ('lock', False)
    )
    assert final_zero_index < unlock_index
    assert all(
        not process.alive and process.reaped
        for process in harness.runner.processes
    )


def test_start_and_finish_select_independent_frozen_profiles():
    harness = successful_harness(
        start_overrides={'sdk_timeout': 0.41},
        finish_overrides={'sdk_timeout': 0.73},
    )

    start_profile = harness.supervisor.select_profile('start_jump')
    finish_profile = harness.supervisor.select_profile('finish_jump')

    assert start_profile.name == 'start'
    assert finish_profile.name == 'finish'
    assert start_profile is not finish_profile
    assert start_profile.sdk_timeout == pytest.approx(0.41)
    assert finish_profile.sdk_timeout == pytest.approx(0.73)
    with pytest.raises(Exception):
        start_profile.sdk_timeout = 99.0


@pytest.mark.parametrize(
    'profile_values',
    [
        {'pre_stop_duration': -0.1},
        {'final_zero_epsilon': 0.0},
        {'final_zero_confirm_samples': 0},
        {'final_zero_confirm_samples': 1.0},
        {'final_zero_timeout': float('nan')},
        {'sdk_timeout': float('inf')},
        {'post_settle_duration': -0.1},
    ],
)
def test_profile_parameters_reject_invalid_types_and_ranges(profile_values):
    values = Harness._profile_values('start')
    values.update(profile_values)
    with pytest.raises(FrontJumpConfigurationError):
        FrontJumpProfile(**values)


@pytest.mark.parametrize(
    'common_values',
    [
        {'sdk_action_executable': ''},
        {'sdk_network_interface': ''},
        {'sdk_network_interface': 'bad interface'},
        {'zero_publish_rate_hz': 0.0},
        {'final_cmd_stale_timeout': float('nan')},
        {'estop_state_stale_timeout': -1.0},
    ],
)
def test_common_parameters_reject_invalid_types_and_ranges(common_values):
    values = {
        'sdk_action_executable': 'helper',
        'sdk_network_interface': 'eth0',
        'zero_publish_rate_hz': 10.0,
        'final_cmd_stale_timeout': 0.2,
        'estop_state_stale_timeout': 0.2,
    }
    values.update(common_values)
    with pytest.raises(FrontJumpConfigurationError):
        FrontJumpConfig(**values)


@pytest.mark.parametrize('motion_name', ['start_jump', 'finish_jump'])
def test_feedback_order_and_progress_are_supervision_only(motion_name):
    harness = successful_harness()

    outcome = harness.run(motion_name)

    assert outcome.success
    assert [
        step for step, unused_progress in harness.feedback
    ] == [
        '{}: {}'.format(motion_name, stage)
        for stage in EXPECTED_STAGES
    ]
    progress = [value for unused_step, value in harness.feedback]
    assert progress == sorted(progress)
    assert all(0.0 <= value <= 1.0 for value in progress)


def test_lock_then_immediate_zero_and_only_zero_for_whole_flow():
    harness = successful_harness()

    outcome = harness.run()

    assert outcome.success
    assert harness.events[0][0:2] == ('lock', True)
    assert harness.events[1][0] == 'zero'
    zero_events = [
        event for event in harness.events if event[0] == 'zero'
    ]
    assert zero_events
    assert all(event[1] == (0.0, 0.0, 0.0) for event in zero_events)
    assert_cleaned(harness, outcome)


def test_final_cmd_requires_three_axis_consecutive_new_zero_samples():
    samples = [
        (0.0, 0.0, 0.0),
        (0.0, 0.0, 0.002),
        (0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0),
    ]

    def on_wait(harness, unused_duration):
        harness.fresh_false_estop()
        if harness.stage() == 'wait_final_cmd_zero' and samples:
            harness.supervisor.update_final_command(*samples.pop(0))

    harness = Harness(on_wait=on_wait)
    harness.fresh_false_estop()
    outcome = harness.run()

    assert outcome.success
    assert not samples
    assert len(harness.runner.calls) == 1
    assert (
        harness.supervisor.completed_contexts[-1].final_zero_streak
        == 3
    )


def test_same_final_cmd_sample_is_not_counted_more_than_once():
    sent = [False]

    def on_wait(harness, unused_duration):
        harness.fresh_false_estop()
        if (
            harness.stage() == 'wait_final_cmd_zero'
            and not sent[0]
        ):
            harness.zero_final_cmd()
            sent[0] = True

    harness = Harness(
        on_wait=on_wait,
        start_overrides={'final_zero_timeout': 0.30},
    )
    outcome = harness.run()

    assert not outcome.success
    assert 'final_zero_timeout' in outcome.reason
    assert harness.runner.calls == []


@pytest.mark.parametrize(
    'bad_sample',
    [
        (0.002, 0.0, 0.0),
        (0.0, 0.002, 0.0),
        (0.0, 0.0, 0.002),
        (math.nan, 0.0, 0.0),
        (0.0, math.inf, 0.0),
    ],
)
def test_nonzero_nan_and_inf_reset_final_zero_streak(bad_sample):
    samples = [
        (0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0),
        bad_sample,
        (0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0),
    ]

    def on_wait(harness, unused_duration):
        harness.fresh_false_estop()
        if harness.stage() == 'wait_final_cmd_zero' and samples:
            harness.supervisor.update_final_command(*samples.pop(0))

    harness = Harness(on_wait=on_wait)
    outcome = harness.run()

    assert outcome.success
    assert not samples
    assert len(harness.runner.calls) == 1


def test_stale_final_cmd_resets_streak_and_cannot_start_helper():
    sent_count = [0]

    def on_wait(harness, unused_duration):
        harness.fresh_false_estop()
        if (
            harness.stage() == 'wait_final_cmd_zero'
            and sent_count[0] < 3
        ):
            harness.zero_final_cmd()
            sent_count[0] += 1

    harness = Harness(
        on_wait=on_wait,
        start_overrides={
            'pre_stop_duration': 0.40,
            'final_zero_timeout': 0.60,
        },
        config_overrides={'final_cmd_stale_timeout': 0.10},
    )
    outcome = harness.run()

    assert not outcome.success
    assert harness.runner.calls == []
    assert (
        harness.supervisor.completed_contexts[-1].final_zero_streak
        == 0
    )


def test_no_new_final_cmd_cannot_pass_and_final_zero_times_out():
    def on_wait(harness, unused_duration):
        harness.fresh_false_estop()

    harness = Harness(
        on_wait=on_wait,
        start_overrides={'final_zero_timeout': 0.25},
    )
    outcome = harness.run()

    assert not outcome.success
    assert outcome.reason == 'final_zero_timeout'
    assert harness.runner.calls == []
    assert_cleaned(harness, outcome)


def test_pre_stop_duration_must_elapse_before_helper_start():
    harness = successful_harness(
        start_overrides={'pre_stop_duration': 0.31}
    )

    outcome = harness.run()

    assert outcome.success
    assert harness.runner.start_times[0] >= 0.31


@pytest.mark.parametrize(
    'estop_mode, expected_reason',
    [
        ('missing', 'estop_state_missing'),
        ('stale', 'estop_state_stale'),
        ('active', 'estop_active'),
    ],
)
def test_missing_stale_or_active_typed_estop_never_starts_helper(
    estop_mode,
    expected_reason,
):
    def on_wait(harness, unused_duration):
        if harness.stage() == 'wait_final_cmd_zero':
            harness.zero_final_cmd()
        if estop_mode == 'active':
            harness.supervisor.update_estop(True)

    harness = Harness(
        on_wait=on_wait,
        start_overrides={'final_zero_timeout': 0.30},
        config_overrides={'estop_state_stale_timeout': 0.08},
    )
    if estop_mode in ('stale', 'active'):
        harness.supervisor.update_estop(estop_mode == 'active')

    outcome = harness.run()

    assert not outcome.success
    assert expected_reason in outcome.reason
    assert harness.runner.calls == []


def test_fresh_false_typed_estop_allows_helper():
    harness = successful_harness()

    outcome = harness.run()

    assert outcome.success
    assert len(harness.runner.calls) == 1


@pytest.mark.parametrize('failure_mode', ['active', 'stale'])
def test_helper_running_estop_active_or_stale_aborts_and_reaps(failure_mode):
    runner_holder = {}

    def on_wait(harness, unused_duration):
        if harness.stage() == 'wait_final_cmd_zero':
            harness.fresh_false_estop()
            harness.zero_final_cmd()
        elif harness.stage() == 'sdk_front_jump':
            if failure_mode == 'active':
                harness.supervisor.update_estop(True)

    harness = Harness(
        on_wait=on_wait,
        runner=FakeRunner(
            FakeClock(),
            process_factory=lambda: None,
        ),
    )
    harness.runner = FakeRunner(
        harness.clock,
        process_factory=lambda: FakeProcess(
            harness.clock, complete_after=None
        ),
    )
    harness.supervisor._process_runner = harness.runner
    runner_holder['runner'] = harness.runner
    harness.fresh_false_estop()

    outcome = harness.run()

    assert not outcome.success
    assert (
        'estop_active' in outcome.reason
        if failure_mode == 'active'
        else 'estop_state_stale' in outcome.reason
    )
    assert_cleaned(harness, outcome)


def test_post_settle_estop_true_aborts_after_sdk_acceptance():
    def on_wait(harness, unused_duration):
        if harness.stage() == 'wait_final_cmd_zero':
            harness.fresh_false_estop()
            harness.zero_final_cmd()
        elif harness.stage() == 'post_settle':
            harness.supervisor.update_estop(True)

    harness = Harness(on_wait=on_wait)
    harness.fresh_false_estop()

    outcome = harness.run()

    assert not outcome.success
    assert outcome.reason == 'estop_active'
    assert outcome.sdk_command_accepted is True
    assert outcome.post_settle_completed is False
    assert_cleaned(harness, outcome)


@pytest.mark.parametrize('stop_kind', ['cancel', 'gait_stop'])
def test_cancel_or_stop_before_helper_uses_fresh_per_goal_event(stop_kind):
    cancel_event = threading.Event()
    stop_event = threading.Event()
    (cancel_event if stop_kind == 'cancel' else stop_event).set()
    harness = successful_harness()

    outcome = harness.run(
        cancel_requested=cancel_event,
        gait_stop_requested=stop_event,
    )

    assert not outcome.success
    assert outcome.terminal_state == (
        'canceled' if stop_kind == 'cancel' else 'abort'
    )
    assert harness.runner.calls == []
    assert_cleaned(harness, outcome)


@pytest.mark.parametrize('stop_kind', ['cancel', 'gait_stop'])
def test_cancel_or_stop_after_helper_start_terminates_local_process(
    stop_kind,
):
    cancel_event = threading.Event()
    stop_event = threading.Event()

    def on_start(unused_process):
        (cancel_event if stop_kind == 'cancel' else stop_event).set()

    harness = successful_harness()
    harness.runner = FakeRunner(
        harness.clock,
        process_factory=lambda: FakeProcess(
            harness.clock, complete_after=None
        ),
        on_start=on_start,
    )
    harness.supervisor._process_runner = harness.runner

    outcome = harness.run(
        cancel_requested=cancel_event,
        gait_stop_requested=stop_event,
    )

    assert not outcome.success
    assert outcome.helper_started is True
    assert outcome.sdk_request_may_have_been_sent is True
    assert harness.runner.processes[0].terminated
    assert_cleaned(harness, outcome)


def test_cancel_state_from_old_goal_does_not_pollute_next_goal():
    first_cancel = threading.Event()
    first_cancel.set()
    harness = successful_harness()

    first = harness.run(cancel_requested=first_cancel)
    second = harness.run()

    assert first.terminal_state == 'canceled'
    assert second.success
    assert len(harness.supervisor.completed_contexts) == 2
    assert (
        harness.supervisor.completed_contexts[0].cancel_requested
        is not harness.supervisor.completed_contexts[1].cancel_requested
    )


def test_mux_diagnostic_cannot_replace_missing_typed_estop():
    def on_wait(harness, unused_duration):
        harness.supervisor.update_mux_status(
            '{"gait_lock": true, "active_source": "locomotion"}'
        )
        if harness.stage() == 'wait_final_cmd_zero':
            harness.zero_final_cmd()

    harness = Harness(
        on_wait=on_wait,
        start_overrides={'final_zero_timeout': 0.25},
    )

    outcome = harness.run()

    assert not outcome.success
    assert 'estop_state_missing' in outcome.reason
    assert harness.runner.calls == []


def test_helper_fixed_argv_has_no_shell_or_string_concatenation():
    harness = successful_harness(
        config_overrides={'sdk_network_interface': 'enp3s0'}
    )

    outcome = harness.run()

    assert outcome.success
    assert harness.runner.calls == [
        ['/fake/helper', 'enp3s0', 'front_jump', '0']
    ]


def test_new_nonzero_final_cmd_before_process_start_closes_gate():
    harness = successful_harness()

    def resolver(unused_name):
        harness.supervisor.update_final_command(0.01, 0.0, 0.0)
        return '/fake/helper'

    harness.supervisor._resolve_executable = resolver

    outcome = harness.run()

    assert not outcome.success
    assert outcome.reason == 'final_cmd_zero_gate_lost_before_helper'
    assert harness.runner.calls == []
    assert_cleaned(harness, outcome)


def test_absolute_missing_helper_aborts_without_fallback(tmp_path):
    missing = tmp_path / 'missing-helper'
    harness = successful_harness(
        config_overrides={'sdk_action_executable': str(missing)},
        resolver=resolve_sdk_executable,
    )

    outcome = harness.run()

    assert not outcome.success
    assert 'helper_resolution_failed' in outcome.reason
    assert harness.runner.calls == []
    assert_cleaned(harness, outcome)


def test_relative_helper_path_with_separator_is_rejected():
    with pytest.raises(
        FrontJumpConfigurationError,
        match='separators',
    ):
        resolve_sdk_executable('./go2_sdk_motion_action')


def test_path_helper_resolution(tmp_path):
    helper = tmp_path / 'go2_sdk_motion_action'
    helper.write_text('#!/bin/sh\nexit 0\n', encoding='utf-8')
    helper.chmod(0o755)

    resolved = resolve_sdk_executable(
        'go2_sdk_motion_action',
        environment={
            'PATH': str(tmp_path),
            'AMENT_PREFIX_PATH': '',
        },
    )

    assert resolved == str(helper.resolve())


def test_ament_prefix_helper_resolution(tmp_path):
    helper = (
        tmp_path
        / 'lib'
        / 'rk_go2_sdk_bridge'
        / 'go2_sdk_motion_action'
    )
    helper.parent.mkdir(parents=True)
    helper.write_text('#!/bin/sh\nexit 0\n', encoding='utf-8')
    helper.chmod(0o755)

    resolved = resolve_sdk_executable(
        'go2_sdk_motion_action',
        environment={
            'PATH': '',
            'AMENT_PREFIX_PATH': str(tmp_path),
        },
        which=lambda unused_name, path=None: None,
    )

    assert resolved == str(helper.resolve())


def test_absolute_helper_must_be_regular_and_executable(tmp_path):
    directory = tmp_path / 'helper-dir'
    directory.mkdir()
    with pytest.raises(FrontJumpConfigurationError, match='regular file'):
        resolve_sdk_executable(str(directory))

    helper = tmp_path / 'helper'
    helper.write_text('not executable', encoding='utf-8')
    helper.chmod(0o644)
    with pytest.raises(FrontJumpConfigurationError, match='not executable'):
        resolve_sdk_executable(str(helper))


@pytest.mark.parametrize(
    'interface_resolver',
    [
        lambda unused_name: 0,
        lambda unused_name: False,
        lambda unused_name: (_ for _ in ()).throw(OSError('missing')),
    ],
)
def test_missing_network_interface_aborts_before_process_start(
    interface_resolver,
):
    harness = successful_harness(
        interface_resolver=interface_resolver
    )

    outcome = harness.run()

    assert not outcome.success
    assert outcome.reason == 'sdk_network_interface_not_found'
    assert harness.runner.calls == []


def test_supervisor_rejects_nonabsolute_resolved_helper_path():
    harness = successful_harness(
        resolver=lambda unused_name: 'relative-helper',
    )

    outcome = harness.run()

    assert not outcome.success
    assert outcome.reason.startswith('helper_resolution_failed:')
    assert harness.runner.calls == []


def test_helper_start_exception_is_captured_and_aborted():
    harness = successful_harness()
    harness.runner = FakeRunner(
        harness.clock,
        start_error=OSError('cannot start'),
    )
    harness.supervisor._process_runner = harness.runner

    outcome = harness.run()

    assert not outcome.success
    assert outcome.reason == 'helper_start_failed'
    assert_cleaned(harness, outcome)


@pytest.mark.parametrize('cleanup_completed', [True, False])
def test_helper_start_error_after_popen_is_conservative(
    cleanup_completed,
):
    harness = successful_harness()
    harness.runner = FakeRunner(
        harness.clock,
        start_error=ProcessStartError(
            'identity initialization failed',
            process_started=True,
            cleanup_completed=cleanup_completed,
        ),
    )
    harness.supervisor._process_runner = harness.runner

    outcome = harness.run()

    assert not outcome.success
    assert outcome.helper_started
    assert outcome.sdk_request_may_have_been_sent
    assert outcome.cleanup_completed is cleanup_completed
    assert outcome.helper_group_empty is cleanup_completed
    if cleanup_completed:
        assert outcome.reason == 'helper_start_failed_after_process_start'
        assert_cleaned(harness, outcome)
    else:
        assert outcome.reason == (
            'helper_start_failed_process_identity_cleanup_unverified'
        )
        lock_states = [
            event[1]
            for event in harness.events
            if event[0] == 'lock'
        ]
        assert lock_states == [True]


@pytest.mark.parametrize('return_code', [0, 7])
def test_helper_return_code_and_output_are_captured(return_code):
    harness = successful_harness()
    harness.runner = FakeRunner(
        harness.clock,
        process_factory=lambda: FakeProcess(
            harness.clock,
            return_code=return_code,
            stdout='captured stdout',
            stderr='captured stderr',
        ),
    )
    harness.supervisor._process_runner = harness.runner

    outcome = harness.run()
    context = harness.supervisor.completed_contexts[-1]

    assert context.helper_return_code == return_code
    assert context.helper_stdout == 'captured stdout'
    assert context.helper_stderr == 'captured stderr'
    assert outcome.success is (return_code == 0)
    assert outcome.sdk_command_accepted is (return_code == 0)
    assert_cleaned(harness, outcome)


def test_helper_timeout_terminates_and_reaps_without_leftover():
    harness = successful_harness(
        start_overrides={'sdk_timeout': 0.16}
    )
    harness.runner = FakeRunner(
        harness.clock,
        process_factory=lambda: FakeProcess(
            harness.clock, complete_after=None
        ),
    )
    harness.supervisor._process_runner = harness.runner

    outcome = harness.run()
    process = harness.runner.processes[0]

    assert not outcome.success
    assert outcome.reason == 'sdk_timeout'
    assert process.terminated
    assert process.reaped
    assert not process.alive
    assert_cleaned(harness, outcome)


def test_helper_that_ignores_terminate_is_killed_and_reaped():
    cancel_event = threading.Event()

    def on_start(unused_process):
        cancel_event.set()

    harness = successful_harness()
    harness.runner = FakeRunner(
        harness.clock,
        process_factory=lambda: FakeProcess(
            harness.clock,
            complete_after=None,
            terminate_works=False,
        ),
        on_start=on_start,
    )
    harness.supervisor._process_runner = harness.runner

    outcome = harness.run(cancel_requested=cancel_event)
    process = harness.runner.processes[0]

    assert outcome.terminal_state == 'canceled'
    assert process.terminated
    assert process.killed
    assert process.reaped
    assert_cleaned(harness, outcome)


def test_sdk_zero_requires_post_settle_and_remains_physical_unverified():
    harness = successful_harness(
        start_overrides={'post_settle_duration': 0.21}
    )

    outcome = harness.run()

    assert outcome.success
    assert outcome.sdk_command_accepted is True
    assert outcome.post_settle_completed is True
    assert outcome.physical_crossing_unverified is True
    assert harness.clock() >= 0.21
    assert 'physical_crossing_unverified=true' in outcome.message()
    assert 'physical jump succeeded' not in outcome.message()
    assert 'obstacle crossed' not in outcome.message()
    assert 'landing succeeded' not in outcome.message()


def test_placeholder_stability_function_is_not_used_by_supervisor():
    source = inspect.getsource(FrontJumpSupervisor)

    assert 'is_robot_stable' not in source
    harness = successful_harness()
    outcome = harness.run()
    assert outcome.success
    assert any(
        record['event'] == 'stability_unavailable'
        and record['stability_check'] == 'unavailable'
        for record in harness.logs
    )


@pytest.mark.parametrize(
    'scenario',
    [
        'success',
        'final_timeout',
        'helper_nonzero',
        'helper_exception',
        'helper_timeout',
        'cancel',
        'gait_stop',
        'estop',
    ],
)
def test_every_exit_path_uses_same_cleanup_and_zero_before_unlock(scenario):
    cancel_event = threading.Event()
    stop_event = threading.Event()

    if scenario == 'final_timeout':
        harness = Harness(
            on_wait=lambda h, unused: h.fresh_false_estop(),
            start_overrides={'final_zero_timeout': 0.20},
        )
    else:
        harness = successful_harness()

    if scenario == 'helper_nonzero':
        harness.runner = FakeRunner(
            harness.clock,
            process_factory=lambda: FakeProcess(
                harness.clock, return_code=3
            ),
        )
        harness.supervisor._process_runner = harness.runner
    elif scenario == 'helper_exception':
        harness.runner = FakeRunner(
            harness.clock, start_error=OSError('start failed')
        )
        harness.supervisor._process_runner = harness.runner
    elif scenario == 'helper_timeout':
        harness.supervisor.profiles['start'] = FrontJumpProfile(
            **{
                **Harness._profile_values('start'),
                'sdk_timeout': 0.12,
            }
        )
        harness.runner = FakeRunner(
            harness.clock,
            process_factory=lambda: FakeProcess(
                harness.clock, complete_after=None
            ),
        )
        harness.supervisor._process_runner = harness.runner
    elif scenario == 'cancel':
        cancel_event.set()
    elif scenario == 'gait_stop':
        stop_event.set()
    elif scenario == 'estop':
        harness.on_wait = (
            lambda h, unused: h.supervisor.update_estop(True)
        )
        harness.supervisor.update_estop(True)

    outcome = harness.run(
        cancel_requested=cancel_event,
        gait_stop_requested=stop_event,
    )

    assert outcome.success is (scenario == 'success')
    assert_cleaned(harness, outcome)


def test_unlock_is_last_goal_lifecycle_event_and_zero_never_restarts():
    harness = successful_harness()

    outcome = harness.run()
    zero_count = sum(event[0] == 'zero' for event in harness.events)
    event_count = len(harness.events)
    harness.clock.advance(10.0)
    harness.supervisor.wake()

    assert outcome.success
    assert len(harness.events) == event_count
    assert sum(event[0] == 'zero' for event in harness.events) == zero_count
    assert harness.events[-1][0:2] == ('lock', False)


def test_no_background_thread_or_timer_is_created():
    before = {thread.ident for thread in threading.enumerate()}
    harness = successful_harness()

    outcome = harness.run()

    after = {thread.ident for thread in threading.enumerate()}
    assert outcome.success
    assert after == before
    assert not hasattr(harness.supervisor, '_timer')
    assert not hasattr(harness.supervisor, '_thread')


@pytest.mark.parametrize(
    'outcome',
    [
        FrontJumpOutcome(
            True,
            'succeed',
            'supervised_flow_done',
            'supervised_flow_completed',
            True,
            True,
            True,
            True,
            True,
        ),
        FrontJumpOutcome(
            False,
            'abort',
            'sdk_front_jump',
            'sdk_timeout',
            True,
            True,
            True,
            False,
            False,
        ),
        FrontJumpOutcome(
            False,
            'canceled',
            'post_settle',
            'cancel_requested',
            True,
            True,
            True,
            True,
            False,
        ),
    ],
)
def test_terminal_state_result_and_required_message_fields_are_consistent(
    outcome,
):
    assert outcome.success is (outcome.terminal_state == 'succeed')
    message = outcome.message()
    for field_name in (
        'stage=',
        'reason=',
        'helper_started=',
        'sdk_request_may_have_been_sent=',
        'cleanup_completed=',
    ):
        assert field_name in message


def test_terminal_state_is_after_cleanup_and_no_feedback_follows():
    harness = successful_harness()

    outcome = harness.run()
    context = harness.supervisor.completed_contexts[-1]
    feedback_indexes = [
        index
        for index, event in enumerate(context.event_history)
        if event[0] == 'feedback'
    ]
    cleanup_index = max(
        index
        for index, event in enumerate(context.event_history)
        if event[0] == 'log'
        and event[1]['event'] == 'cleanup_completed'
    )

    assert outcome.cleanup_completed
    assert max(feedback_indexes) < cleanup_index
    assert context.feedback_enabled is False


def test_lock_true_publish_exception_fails_closed_without_helper_or_unlock():
    harness = successful_harness()
    lock_calls = []

    def publish_lock(locked):
        lock_calls.append(locked)
        if locked:
            raise RuntimeError('lock publish failed')

    harness.supervisor._publish_lock_callback = publish_lock

    outcome = harness.run()

    assert not outcome.success
    assert not outcome.cleanup_completed
    assert not outcome.lock_acquire_command_published
    assert not outcome.lock_release_command_published
    assert harness.runner.calls == []
    assert lock_calls == [True]
    assert any(event[0] == 'zero' for event in harness.events)


def test_zero_publish_exception_keeps_lock_and_aborts_cleanup():
    harness = successful_harness()
    lock_calls = []

    def publish_lock(locked):
        lock_calls.append(locked)

    def publish_zero():
        raise RuntimeError('zero publish failed')

    harness.supervisor._publish_lock_callback = publish_lock
    harness.supervisor._publish_zero_callback = publish_zero

    outcome = harness.run()

    assert not outcome.success
    assert not outcome.cleanup_completed
    assert not outcome.lock_release_command_published
    assert lock_calls == [True]


def test_serialized_lock_true_publish_failure_stays_desired_true():
    result_type, publisher_type = load_lock_publisher_types()
    calls = []
    mirrored = {}

    class Publisher:
        def publish(self, msg):
            calls.append(msg.data)
            raise RuntimeError('publish true failed')

    state_lock = threading.RLock()
    lock_publisher = publisher_type(
        Publisher(),
        state_lock,
        lambda desired, faulted: mirrored.update(
            desired=desired,
            faulted=faulted,
        ),
    )

    result = lock_publisher.set_locked(True)

    assert isinstance(result, result_type)
    assert not result.publish_succeeded
    assert result.requested_state is True
    assert lock_publisher.desired_state is True
    assert lock_publisher.lock_publish_fault is True
    assert mirrored == {'desired': True, 'faulted': True}
    assert calls == [True]


def test_serialized_lock_false_failure_rolls_back_without_opposite_publish():
    _, publisher_type = load_lock_publisher_types()
    calls = []

    class Publisher:
        def publish(self, msg):
            calls.append(msg.data)
            if msg.data is False:
                raise RuntimeError('publish false failed')

    lock_publisher = publisher_type(
        Publisher(),
        threading.RLock(),
        lambda desired, faulted: None,
    )

    assert lock_publisher.set_locked(True).publish_succeeded
    failed_release = lock_publisher.set_locked(False)

    assert not failed_release.publish_succeeded
    assert lock_publisher.desired_state is True
    assert lock_publisher.lock_publish_fault is True
    assert calls == [True, False]

    periodic = lock_publisher.republish()
    assert periodic.publish_succeeded
    assert periodic.requested_state is True
    assert calls == [True, False, True]
    assert not failed_release.publish_succeeded


@pytest.mark.parametrize(
    'initial_state, transition_state',
    [(False, True), (True, False)],
)
def test_serialized_periodic_snapshot_cannot_overwrite_new_transition(
    initial_state,
    transition_state,
):
    _, publisher_type = load_lock_publisher_types()
    calls = []
    periodic_entered = threading.Event()
    release_periodic = threading.Event()

    class Publisher:
        def publish(self, msg):
            calls.append(msg.data)
            if len(calls) == 1:
                periodic_entered.set()
                assert release_periodic.wait(timeout=1.0)

    lock_publisher = publisher_type(
        Publisher(),
        threading.RLock(),
        lambda desired, faulted: None,
        initial_state=initial_state,
    )

    periodic_thread = threading.Thread(target=lock_publisher.republish)
    transition_thread = threading.Thread(
        target=lambda: lock_publisher.set_locked(transition_state)
    )
    periodic_thread.start()
    assert periodic_entered.wait(timeout=1.0)
    transition_thread.start()
    release_periodic.set()
    periodic_thread.join(timeout=1.0)
    transition_thread.join(timeout=1.0)

    assert not periodic_thread.is_alive()
    assert not transition_thread.is_alive()
    assert calls == [initial_state, transition_state]
    assert lock_publisher.desired_state is transition_state


def guard_operation(identity='json-1'):
    return {
        'reservation_token': 'reservation-token',
        'entry_type': 'json',
        'motion_name': 'start_jump',
        'goal_uuid': '',
        'command_identity': identity,
    }


def test_cleanup_guard_atomic_round_trip_permissions_and_unique_fault_id(
    tmp_path,
):
    guard_path = tmp_path / 'runtime' / 'front_jump_guard.json'
    guard = PersistentCleanupGuard(
        guard_path,
        boot_id_reader=lambda: 'boot-a',
        wall_clock_ns=lambda: 123,
    )

    first = guard.begin_dirty(guard_operation('json-1'))
    first_id = first['cleanup_fault_id']
    assert first['schema_version'] == 1
    assert stat_mode(guard_path.parent) == 0o700
    assert stat_mode(guard_path) == 0o600
    assert guard.load() == first

    guard.clear(first_id)
    second = guard.begin_dirty(guard_operation('json-2'))
    assert second['cleanup_fault_id'] != first_id


def stat_mode(path):
    return os.stat(str(path), follow_symlinks=False).st_mode & 0o777


def test_cleanup_guard_rejects_old_fault_id(tmp_path):
    guard = PersistentCleanupGuard(
        tmp_path / 'guard.json',
        boot_id_reader=lambda: 'boot-a',
    )
    current = guard.begin_dirty(guard_operation())

    with pytest.raises(CleanupGuardError, match='mismatch'):
        guard.clear('old-cleanup-fault-id')

    assert guard.load()['cleanup_fault_id'] == current['cleanup_fault_id']


def test_cleanup_guard_never_overwrites_an_armed_fault_id(tmp_path):
    guard = PersistentCleanupGuard(
        tmp_path / 'guard.json',
        boot_id_reader=lambda: 'boot-a',
    )
    first = guard.begin_dirty(guard_operation('json-first'))

    with pytest.raises(CleanupGuardError, match='already armed'):
        guard.begin_dirty(guard_operation('json-second'))

    updated = guard.record_fault('cleanup_fault', 'keep first identity')
    assert updated['cleanup_fault_id'] == first['cleanup_fault_id']
    assert updated['operation']['command_identity'] == 'json-first'


@pytest.mark.parametrize(
    'payload',
    [
        b'{broken json',
        json.dumps(
            {
                'schema_version': 999,
                'state': 'DIRTY',
            }
        ).encode('utf-8'),
    ],
)
def test_cleanup_guard_corrupt_or_unknown_schema_fails_closed(
    tmp_path,
    payload,
):
    guard_path = tmp_path / 'guard.json'
    guard_path.write_bytes(payload)
    os.chmod(str(guard_path), 0o600)
    guard = PersistentCleanupGuard(
        guard_path,
        boot_id_reader=lambda: 'boot-a',
    )

    with pytest.raises(CleanupGuardError):
        guard.load()

    assert guard_path.read_bytes() == payload


@pytest.mark.parametrize(
    'mutate',
    [
        lambda record: record['helper'].__setitem__('started', 1),
        lambda record: record['helper'].__setitem__('pid', True),
        lambda record: record['lock'].__setitem__(
            'lock_acquire_command_published',
            'false',
        ),
        lambda record: record['cleanup'].__setitem__(
            'group_empty',
            None,
        ),
        lambda record: record.__setitem__('created_at_unix_ns', 1.5),
        lambda record: record['operation'].__setitem__(
            'entry_type',
            'topic',
        ),
    ],
)
def test_cleanup_guard_nested_type_errors_fail_closed(
    tmp_path,
    mutate,
):
    guard_path = tmp_path / 'guard.json'
    guard = PersistentCleanupGuard(
        guard_path,
        boot_id_reader=lambda: 'boot-a',
    )
    record = guard.begin_dirty(guard_operation())
    mutate(record)
    original_bytes = (
        json.dumps(record, separators=(',', ':')) + '\n'
    ).encode('utf-8')
    guard_path.write_bytes(original_bytes)
    os.chmod(str(guard_path), 0o600)
    reloaded_guard = PersistentCleanupGuard(
        guard_path,
        boot_id_reader=lambda: 'boot-a',
    )

    with pytest.raises(CleanupGuardError):
        reloaded_guard.load()

    assert guard_path.read_bytes() == original_bytes


def test_cleanup_guard_rejects_symbolic_link(tmp_path):
    real_path = tmp_path / 'real.json'
    real_path.write_text('{}', encoding='utf-8')
    os.chmod(str(real_path), 0o600)
    link_path = tmp_path / 'guard.json'
    link_path.symlink_to(real_path)
    guard = PersistentCleanupGuard(
        link_path,
        boot_id_reader=lambda: 'boot-a',
    )

    with pytest.raises(CleanupGuardError):
        guard.load()


def test_cleanup_guard_rejects_symbolic_link_directory(tmp_path):
    real_directory = tmp_path / 'real-runtime'
    real_directory.mkdir(mode=0o700)
    linked_directory = tmp_path / 'linked-runtime'
    linked_directory.symlink_to(real_directory, target_is_directory=True)
    guard = PersistentCleanupGuard(
        linked_directory / 'guard.json',
        boot_id_reader=lambda: 'boot-a',
    )

    with pytest.raises(CleanupGuardError, match='symbolic link'):
        guard.begin_dirty(guard_operation())


def test_cleanup_guard_rejects_nonregular_file(tmp_path):
    guard_path = tmp_path / 'guard.json'
    guard_path.mkdir(mode=0o700)
    guard = PersistentCleanupGuard(
        guard_path,
        boot_id_reader=lambda: 'boot-a',
    )

    with pytest.raises(CleanupGuardError, match='regular file'):
        guard.load()


def test_cleanup_guard_atomic_replace_failure_preserves_previous_record(
    tmp_path,
    monkeypatch,
):
    import rk_locomotion.front_jump_supervisor as supervisor_module

    guard = PersistentCleanupGuard(
        tmp_path / 'guard.json',
        boot_id_reader=lambda: 'boot-a',
    )
    original = guard.begin_dirty(guard_operation())
    original_bytes = guard.path.read_bytes()

    def fail_replace(unused_source, unused_destination, **unused_kwargs):
        raise OSError('replace failed')

    monkeypatch.setattr(supervisor_module.os, 'replace', fail_replace)
    with pytest.raises(OSError, match='replace failed'):
        guard.record_fault('cleanup_fault', 'forced failure')

    assert guard.path.read_bytes() == original_bytes
    assert guard.current_record == original


def test_recovery_evidence_requires_new_zero_samples_and_fresh_false_estop():
    harness = Harness()
    baseline = harness.supervisor.begin_recovery_window()
    ready, _ = harness.supervisor.recovery_evidence_ready(
        baseline,
        confirm_samples=3,
        epsilon=0.001,
    )
    assert not ready

    harness.supervisor.update_estop(False)
    for _ in range(2):
        harness.supervisor.update_final_command(0.0, 0.0, 0.0)
    ready, _ = harness.supervisor.recovery_evidence_ready(
        baseline,
        confirm_samples=3,
        epsilon=0.001,
    )
    assert not ready

    harness.supervisor.update_final_command(0.0, 0.0, 0.0)
    ready, _ = harness.supervisor.recovery_evidence_ready(
        baseline,
        confirm_samples=3,
        epsilon=0.001,
    )
    assert ready

    harness.supervisor.update_estop(True)
    ready, _ = harness.supervisor.recovery_evidence_ready(
        baseline,
        confirm_samples=3,
        epsilon=0.001,
    )
    assert not ready


@pytest.mark.parametrize(
    'failure_mode',
    [
        'stale_estop',
        'stale_final_cmd',
        'nonzero',
        'nan',
        'inf',
    ],
)
def test_recovery_evidence_fails_closed_for_stale_or_bad_samples(
    failure_mode,
):
    harness = Harness()
    baseline = harness.supervisor.begin_recovery_window()
    harness.supervisor.update_estop(False)
    for _ in range(3):
        harness.supervisor.update_final_command(0.0, 0.0, 0.0)

    if failure_mode == 'stale_estop':
        harness.clock.advance(0.16)
        for _ in range(3):
            harness.supervisor.update_final_command(0.0, 0.0, 0.0)
    elif failure_mode == 'stale_final_cmd':
        harness.clock.advance(0.16)
        harness.supervisor.update_estop(False)
    else:
        bad_value = {
            'nonzero': 0.01,
            'nan': math.nan,
            'inf': math.inf,
        }[failure_mode]
        harness.supervisor.update_final_command(bad_value, 0.0, 0.0)

    ready, _ = harness.supervisor.recovery_evidence_ready(
        baseline,
        confirm_samples=3,
        epsilon=0.001,
    )

    assert not ready


def test_guard_process_update_failure_reaps_without_unlock(tmp_path):
    class FailProcessIdentityGuard(PersistentCleanupGuard):
        def update(self, mutator):
            record = self.current_record
            candidate = json.loads(json.dumps(record))
            mutator(candidate)
            if candidate['helper']['started']:
                raise OSError('process identity update failed')
            return super().update(mutator)

    class IdentityFakeProcess(FakeProcess):
        pid = 101
        pgid = 101
        session_id = 101
        start_ticks = 500
        executable = '/fake/helper'

        def group_empty(self):
            return self.poll() is not None

    guard = FailProcessIdentityGuard(
        tmp_path / 'guard.json',
        boot_id_reader=lambda: 'boot-a',
    )
    harness = successful_harness(
        cleanup_guard=guard,
        runner=FakeRunner(
            FakeClock(),
            process_factory=lambda: IdentityFakeProcess(harness.clock),
        ),
    )

    outcome = harness.run()

    assert not outcome.success
    assert not outcome.cleanup_completed
    assert harness.runner.processes[0].reaped
    assert [event[1] for event in harness.events if event[0] == 'lock'] == [
        True
    ]
    assert guard.load()['state'] == 'DIRTY'


def test_guard_clear_failure_relocks_and_preserves_dirty_evidence(tmp_path):
    class UnclearableGuard(PersistentCleanupGuard):
        def clear(self, expected_fault_id):
            del expected_fault_id
            raise OSError('guard unlink failed')

    guard = UnclearableGuard(
        tmp_path / 'guard.json',
        boot_id_reader=lambda: 'boot-a',
    )
    harness = successful_harness(cleanup_guard=guard)

    class IdentityFakeProcess(FakeProcess):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            self.pgid = self.pid
            self.session_id = self.pid
            self.start_ticks = 500
            self.executable = '/fake/helper'

        def group_empty(self):
            return not self.alive

    harness.runner = FakeRunner(
        harness.clock,
        process_factory=lambda: IdentityFakeProcess(harness.clock),
    )
    harness.supervisor._process_runner = harness.runner

    outcome = harness.run()
    record = guard.load()

    assert not outcome.success
    assert not outcome.cleanup_completed
    assert [event[1] for event in harness.events if event[0] == 'lock'] == [
        True,
        False,
        True,
    ]
    assert record['state'] == 'DIRTY'
    assert record['cleanup']['cleanup_completed'] is True


def test_cleanup_permission_error_keeps_guard_and_lock(tmp_path):
    cancel_event = threading.Event()

    class PermissionDeniedProcess(FakeProcess):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            self.pgid = self.pid
            self.session_id = self.pid
            self.start_ticks = 500
            self.executable = '/fake/helper'

        def group_empty(self):
            return not self.alive

        def terminate(self):
            raise PermissionError('group signal denied')

    harness = successful_harness(
        cleanup_guard=PersistentCleanupGuard(
            tmp_path / 'guard.json',
            boot_id_reader=lambda: 'boot-a',
        ),
    )
    harness.runner = FakeRunner(
        harness.clock,
        process_factory=lambda: PermissionDeniedProcess(
            harness.clock,
            complete_after=None,
        ),
        on_start=lambda unused_process: cancel_event.set(),
    )
    harness.supervisor._process_runner = harness.runner

    outcome = harness.run(cancel_requested=cancel_event)

    assert outcome.terminal_state == 'canceled'
    assert not outcome.cleanup_completed
    assert harness.supervisor.cleanup_pending
    assert [event[1] for event in harness.events if event[0] == 'lock'] == [
        True
    ]
    assert harness.supervisor.cleanup_guard.load()['state'] == 'DIRTY'


@pytest.mark.parametrize('ignore_sigterm', [False, True])
def test_real_subprocess_runner_reaps_entire_process_group(
    tmp_path,
    ignore_sigterm,
):
    helper_script = tmp_path / 'harmless_helper.py'
    leader_pid_path = tmp_path / 'leader.pid'
    child_pid_path = tmp_path / 'child.pid'
    helper_script.write_text(
        '\n'.join(
            [
                'import os',
                'from pathlib import Path',
                'import signal',
                'import subprocess',
                'import sys',
                'import time',
                'leader_path = Path(sys.argv[1])',
                'child_path = Path(sys.argv[2])',
                "ignore = sys.argv[3] == 'ignore'",
                'if ignore:',
                '    signal.signal(signal.SIGTERM, signal.SIG_IGN)',
                'child_code = (',
                "    'import os,signal,sys,time;'",
                "    \"signal.signal(signal.SIGTERM, signal.SIG_IGN) \"",
                "    \"if sys.argv[2] == \\'ignore\\' else None;\"",
                "    \"open(sys.argv[1], \\'w\\').write(str(os.getpid()));\"",
                "    \"time.sleep(60)\"",
                ')',
                'child = subprocess.Popen([',
                '    sys.executable,',
                "    '-c',",
                '    child_code,',
                '    str(child_path),',
                "    'ignore' if ignore else 'normal',",
                '])',
                "leader_path.write_text(str(os.getpid()), encoding='ascii')",
                'while True:',
                '    time.sleep(1)',
            ]
        )
        + '\n',
        encoding='utf-8',
    )
    runner = SubprocessRunner()
    process = runner.start(
        [
            sys.executable,
            str(helper_script),
            str(leader_pid_path),
            str(child_pid_path),
            'ignore' if ignore_sigterm else 'normal',
        ]
    )

    leader_pid = process.pid
    child_pid = None
    try:
        deadline = time.monotonic() + 2.0
        while (
            time.monotonic() < deadline
            and (
                not leader_pid_path.exists()
                or not child_pid_path.exists()
                or leader_pid_path.stat().st_size == 0
                or child_pid_path.stat().st_size == 0
            )
        ):
            time.sleep(0.01)
        assert leader_pid_path.exists()
        assert child_pid_path.exists()
        assert leader_pid_path.stat().st_size > 0
        assert child_pid_path.stat().st_size > 0
        child_pid = int(child_pid_path.read_text(encoding='ascii'))
        assert process.pgid == leader_pid
        assert os.getpgid(child_pid) == process.pgid

        process.terminate()
        if ignore_sigterm:
            time.sleep(0.10)
            assert process.poll() is None
            process.kill()

        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline and process.poll() is None:
            time.sleep(0.01)
        result = process.reap(timeout=0.5)
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline and not process.group_empty():
            time.sleep(0.01)

        assert result.return_code is not None
        assert process.poll() is not None
        assert process.group_empty()
        assert not Path('/proc/{}'.format(leader_pid)).exists()
        assert not Path('/proc/{}'.format(child_pid)).exists()
        with pytest.raises(ChildProcessError):
            os.waitpid(leader_pid, os.WNOHANG)
    finally:
        try:
            process.kill()
        except (OSError, ProcessLookupError, ProcessIdentityError):
            pass
        try:
            process.reap(timeout=0.5)
        except Exception:
            pass


def test_real_subprocess_large_output_cannot_block_or_bloat_result():
    runner = SubprocessRunner()
    process = runner.start(
        [
            sys.executable,
            '-c',
            (
                "import sys;"
                "sys.stdout.write('o' * 1000000);"
                "sys.stderr.write('e' * 1000000)"
            ),
        ]
    )

    try:
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline and process.poll() is None:
            time.sleep(0.01)
        result = process.reap(timeout=0.5)

        assert result.return_code == 0
        assert result.stdout.startswith('[truncated ')
        assert result.stderr.startswith('[truncated ')
        assert len(result.stdout) < 66000
        assert len(result.stderr) < 66000
        assert process.group_empty()
    finally:
        try:
            process.kill()
        except (OSError, ProcessLookupError, ProcessIdentityError):
            pass
        try:
            process.reap(timeout=0.5)
        except Exception:
            pass


class RosInjectedProcess:
    """Harmless process double for real GaitControlNode wiring tests."""

    _next_pid = 700000

    def __init__(self, *, blocking=False):
        self.pid = RosInjectedProcess._next_pid
        RosInjectedProcess._next_pid += 1
        self.pgid = self.pid
        self.session_id = self.pid
        self.start_ticks = 100
        self.executable = '/fake/front_jump_helper'
        self.return_code = None if blocking else 0
        self.terminated = False
        self.killed = False
        self.reaped = False

    def poll(self):
        return self.return_code

    def terminate(self):
        self.terminated = True
        self.return_code = -15

    def kill(self):
        self.killed = True
        self.return_code = -9

    def group_empty(self):
        return self.return_code is not None

    def reap(self, timeout):
        del timeout
        if self.return_code is None:
            raise subprocess.TimeoutExpired('injected-helper', 0.0)
        self.reaped = True
        return ProcessResult(self.return_code, '', '')


class RosInjectedRunner:
    def __init__(self, *, blocking=False):
        self.blocking = bool(blocking)
        self.calls = []
        self.processes = []

    def start(self, argv):
        self.calls.append(list(argv))
        process = RosInjectedProcess(blocking=self.blocking)
        self.processes.append(process)
        return process


def load_ros_integration_types():
    try:
        import rclpy
        from action_msgs.msg import GoalStatus
        from geometry_msgs.msg import Twist
        from rclpy.action import ActionClient
        from rclpy.action.server import CancelResponse, GoalResponse
        from rclpy.context import Context
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.node import Node
        from rclpy.parameter import Parameter
        from rclpy.qos import QoSDurabilityPolicy
        from rclpy.qos import QoSHistoryPolicy
        from rclpy.qos import QoSProfile
        from rclpy.qos import QoSReliabilityPolicy
        from std_msgs.msg import Bool, String

        from rk_interfaces.action import ExecuteMotion
        from rk_locomotion.gait_control_node import GaitControlNode
    except (ImportError, ModuleNotFoundError) as error:
        pytest.skip(
            'ROS integration dependencies are not built: {}'.format(error)
        )
    return {
        'rclpy': rclpy,
        'GoalStatus': GoalStatus,
        'Twist': Twist,
        'ActionClient': ActionClient,
        'CancelResponse': CancelResponse,
        'GoalResponse': GoalResponse,
        'Context': Context,
        'MultiThreadedExecutor': MultiThreadedExecutor,
        'Node': Node,
        'Parameter': Parameter,
        'QoSDurabilityPolicy': QoSDurabilityPolicy,
        'QoSHistoryPolicy': QoSHistoryPolicy,
        'QoSProfile': QoSProfile,
        'QoSReliabilityPolicy': QoSReliabilityPolicy,
        'Bool': Bool,
        'String': String,
        'ExecuteMotion': ExecuteMotion,
        'GaitControlNode': GaitControlNode,
    }


def make_ros_gait_node(tmp_path, context, runner):
    ros = load_ros_integration_types()
    suffix = uuid.uuid4().hex
    prefix = '/front_jump_test_{}'.format(suffix)
    topics = {
        'cmd_vel': prefix + '/locomotion_cmd',
        'status': prefix + '/status',
        'lock': prefix + '/control_lock',
        'debug': prefix + '/debug',
        'mode': prefix + '/mode',
        'command_json': prefix + '/command_json',
        'action': prefix + '/execute_motion',
        'final_cmd': prefix + '/final_cmd',
        'mux_status': prefix + '/mux_status',
        'estop': prefix + '/estop',
        'depth': prefix + '/depth',
        'scan': prefix + '/scan',
    }
    values = {
        'cmd_vel_topic': topics['cmd_vel'],
        'status_topic': topics['status'],
        'control_lock_topic': topics['lock'],
        'debug_topic': topics['debug'],
        'current_mode_topic': topics['mode'],
        'command_json_topic': topics['command_json'],
        'motion_action_name': topics['action'],
        'obstacle_safety.enable_depth': False,
        'obstacle_safety.enable_scan': False,
        'obstacle_safety.depth_image_topic': topics['depth'],
        'obstacle_safety.scan_topic': topics['scan'],
        'stop_publish_count': 1,
        'stop_publish_period_sec': 0.0,
        'publish_rate_hz': 50.0,
        'status_publish_rate_hz': 20.0,
        'default_hold_duration_sec': 0.2,
        'front_jump.start.pre_stop_duration': 0.02,
        'front_jump.start.final_zero_confirm_samples': 2,
        'front_jump.start.final_zero_timeout': 1.0,
        'front_jump.start.sdk_timeout': 1.0,
        'front_jump.start.post_settle_duration': 0.02,
        'front_jump.finish.pre_stop_duration': 0.02,
        'front_jump.finish.final_zero_confirm_samples': 2,
        'front_jump.finish.final_zero_timeout': 1.0,
        'front_jump.finish.sdk_timeout': 1.0,
        'front_jump.finish.post_settle_duration': 0.02,
        'front_jump.zero_publish_rate_hz': 50.0,
        'front_jump.final_cmd_topic': topics['final_cmd'],
        'front_jump.final_cmd_stale_timeout': 0.5,
        'front_jump.cmd_mux_status_topic': topics['mux_status'],
        'front_jump.estop_state_topic': topics['estop'],
        'front_jump.estop_state_stale_timeout': 0.5,
        'front_jump.sdk_action_executable': (
            '/fake/front_jump_helper'
        ),
        'front_jump.sdk_network_interface': 'fake0',
        'front_jump.cleanup_guard_path': str(
            tmp_path / 'runtime' / 'guard.json'
        ),
        'front_jump.shutdown_drain_timeout_sec': 1.0,
    }
    parameters = [
        ros['Parameter'](name, value=value)
        for name, value in values.items()
    ]
    node = ros['GaitControlNode'](
        node_name='gait_control_{}'.format(suffix),
        context=context,
        parameter_overrides=parameters,
        process_runner=runner,
        network_interface_validator=lambda unused_name: 1,
        executable_resolver=(
            lambda unused_name: '/fake/front_jump_helper'
        ),
    )
    return ros, node, topics


def spin_until(executor, predicate, timeout=3.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        executor.spin_once(timeout_sec=0.02)
    return bool(predicate())


def destroy_ros_test_graph(context, executor, *nodes):
    try:
        executor.shutdown(timeout_sec=1.0)
    except Exception:
        pass
    for node in nodes:
        try:
            executor.remove_node(node)
        except Exception:
            pass
    for node in nodes:
        try:
            node.destroy_node()
        except Exception:
            pass
    context.try_shutdown()


def create_ros_test_client(ros, context, topics, action_type):
    client_node = ros['Node'](
        'front_jump_client_{}'.format(uuid.uuid4().hex),
        context=context,
    )
    action_client = ros['ActionClient'](
        client_node,
        action_type,
        topics['action'],
    )
    final_pub = client_node.create_publisher(
        ros['Twist'],
        topics['final_cmd'],
        ros['QoSProfile'](
            history=ros['QoSHistoryPolicy'].KEEP_LAST,
            depth=10,
            reliability=ros['QoSReliabilityPolicy'].RELIABLE,
            durability=ros['QoSDurabilityPolicy'].VOLATILE,
        ),
    )
    estop_pub = client_node.create_publisher(
        ros['Bool'],
        topics['estop'],
        ros['QoSProfile'](
            history=ros['QoSHistoryPolicy'].KEEP_LAST,
            depth=1,
            reliability=ros['QoSReliabilityPolicy'].RELIABLE,
            durability=ros['QoSDurabilityPolicy'].TRANSIENT_LOCAL,
        ),
    )
    json_pub = client_node.create_publisher(
        ros['String'],
        topics['command_json'],
        10,
    )

    def publish_heartbeat():
        estop = ros['Bool']()
        estop.data = False
        estop_pub.publish(estop)
        final_pub.publish(ros['Twist']())

    heartbeat_timer = client_node.create_timer(0.02, publish_heartbeat)
    return (
        client_node,
        action_client,
        json_pub,
        heartbeat_timer,
    )


def test_real_ros_front_jump_action_success_and_zero_only(tmp_path):
    ros = load_ros_integration_types()
    context = ros['Context']()
    ros['rclpy'].init(context=context)
    runner = RosInjectedRunner()
    ros, node, topics = make_ros_gait_node(tmp_path, context, runner)
    client_node = None
    executor = ros['MultiThreadedExecutor'](
        num_threads=4,
        context=context,
    )
    executor.add_node(node)
    events = []
    lifecycle_calls = []
    original_lock_callback = (
        node.front_jump_supervisor._publish_lock_callback
    )
    original_motion_move = node.motion.move

    def tracked_lock_callback(locked):
        lifecycle_calls.append(('lock', bool(locked)))
        return original_lock_callback(locked)

    def tracked_motion_move(vx, vy, wz):
        lifecycle_calls.append(
            ('cmd', (float(vx), float(vy), float(wz)))
        )
        return original_motion_move(vx, vy, wz)

    node.front_jump_supervisor._publish_lock_callback = (
        tracked_lock_callback
    )
    node.motion.move = tracked_motion_move
    try:
        (
            client_node,
            action_client,
            unused_json_pub,
            unused_timer,
        ) = create_ros_test_client(
            ros,
            context,
            topics,
            ros['ExecuteMotion'],
        )
        client_node.create_subscription(
            ros['Bool'],
            topics['lock'],
            lambda msg: events.append(
                ('lock', bool(msg.data), time.monotonic())
            ),
            10,
        )
        client_node.create_subscription(
            ros['Twist'],
            topics['cmd_vel'],
            lambda msg: events.append(
                (
                    'cmd',
                    (
                        msg.linear.x,
                        msg.linear.y,
                        msg.angular.z,
                    ),
                    time.monotonic(),
                )
            ),
            10,
        )
        executor.add_node(client_node)
        assert spin_until(
            executor,
            action_client.server_is_ready,
            timeout=2.0,
        )

        goal = ros['ExecuteMotion'].Goal()
        goal.motion_name = 'start_jump'
        send_future = action_client.send_goal_async(goal)
        assert spin_until(executor, send_future.done)
        goal_handle = send_future.result()
        assert goal_handle.accepted
        result_future = goal_handle.get_result_async()
        assert spin_until(executor, result_future.done, timeout=3.0)
        response = result_future.result()
        assert response.status == ros['GoalStatus'].STATUS_SUCCEEDED
        assert response.result.success
        assert 'physical_crossing_unverified=true' in (
            response.result.message
        )
        assert len(runner.calls) == 1
        assert runner.processes[0].reaped
        assert node._motion_slot is None

        assert spin_until(
            executor,
            lambda: any(
                event[0:2] == ('lock', False)
                for event in events
            ),
        )
        true_indexes = [
            index
            for index, event in enumerate(events)
            if event[0:2] == ('lock', True)
        ]
        false_indexes = [
            index
            for index, event in enumerate(events)
            if event[0:2] == ('lock', False)
        ]
        command_events = [
            event for event in events if event[0] == 'cmd'
        ]
        assert true_indexes
        assert false_indexes
        assert command_events
        assert all(
            all(math.isfinite(value) and value == 0.0 for value in event[1])
            for event in command_events
        )
        lifecycle_unlock = max(
            index
            for index, event in enumerate(lifecycle_calls)
            if event == ('lock', False)
        )
        assert not any(
            event[0] == 'cmd'
            for event in lifecycle_calls[lifecycle_unlock + 1:]
        )
    finally:
        node.request_shutdown()
        spin_until(executor, node.shutdown_drained, timeout=1.0)
        node.prepare_finalize_shutdown(context_valid=context.ok())
        node.commit_finalize_shutdown(
            executor_shutdown_succeeded=True,
            context_valid=context.ok(),
        )
        destroy_ros_test_graph(
            context,
            executor,
            *(
                [node, client_node]
                if client_node is not None
                else [node]
            ),
        )


def test_real_ros_action_and_json_cross_gate_and_cancel(tmp_path):
    ros = load_ros_integration_types()
    context = ros['Context']()
    ros['rclpy'].init(context=context)
    runner = RosInjectedRunner(blocking=True)
    ros, node, topics = make_ros_gait_node(tmp_path, context, runner)
    executor = ros['MultiThreadedExecutor'](
        num_threads=4,
        context=context,
    )
    executor.add_node(node)
    client_node = None
    try:
        (
            client_node,
            action_client,
            json_pub,
            unused_timer,
        ) = create_ros_test_client(
            ros,
            context,
            topics,
            ros['ExecuteMotion'],
        )
        executor.add_node(client_node)
        assert spin_until(
            executor,
            action_client.server_is_ready,
            timeout=2.0,
        )

        first_goal = ros['ExecuteMotion'].Goal()
        first_goal.motion_name = 'start_jump'
        first_send = action_client.send_goal_async(first_goal)
        assert spin_until(executor, first_send.done)
        first_handle = first_send.result()
        assert first_handle.accepted
        assert spin_until(
            executor,
            lambda: bool(runner.processes),
            timeout=2.0,
        )
        status_before_rejection = node._status

        rejected_json = ros['String']()
        rejected_json.data = json.dumps(
            {'command': 'HOLD_STABLE', 'duration_sec': 0.01}
        )
        json_pub.publish(rejected_json)
        second_goal = ros['ExecuteMotion'].Goal()
        second_goal.motion_name = 'finish_jump'
        second_send = action_client.send_goal_async(second_goal)
        assert spin_until(executor, second_send.done)
        assert not second_send.result().accepted
        for _ in range(5):
            executor.spin_once(timeout_sec=0.02)
        assert node._status == status_before_rejection
        assert len(runner.calls) == 1

        cancel_future = first_handle.cancel_goal_async()
        assert spin_until(executor, cancel_future.done)
        assert cancel_future.result().goals_canceling
        first_result = first_handle.get_result_async()
        assert spin_until(executor, first_result.done, timeout=2.0)
        response = first_result.result()
        assert response.status == ros['GoalStatus'].STATUS_CANCELED
        assert not response.result.success
        assert runner.processes[0].terminated
        assert runner.processes[0].reaped
        assert node._motion_slot is None

        hold_json = ros['String']()
        hold_json.data = json.dumps(
            {'command': 'HOLD_STABLE', 'duration_sec': 0.3}
        )
        json_pub.publish(hold_json)
        assert spin_until(
            executor,
            lambda: (
                node._motion_slot is not None
                and node._motion_slot.entry_type == 'json'
            ),
        )
        running_status = node._status
        rejected_action = ros['ExecuteMotion'].Goal()
        rejected_action.motion_name = 'hold_stable'
        rejected_send = action_client.send_goal_async(rejected_action)
        assert spin_until(executor, rejected_send.done)
        assert not rejected_send.result().accepted
        json_pub.publish(rejected_json)
        for _ in range(5):
            executor.spin_once(timeout_sec=0.02)
        assert node._status == running_status
        assert spin_until(
            executor,
            lambda: node._motion_slot is None,
            timeout=2.0,
        )
    finally:
        node.request_shutdown()
        spin_until(executor, node.shutdown_drained, timeout=1.0)
        node.prepare_finalize_shutdown(context_valid=context.ok())
        node.commit_finalize_shutdown(
            executor_shutdown_succeeded=True,
            context_valid=context.ok(),
        )
        destroy_ros_test_graph(
            context,
            executor,
            *(
                [node, client_node]
                if client_node is not None
                else [node]
            ),
        )


class FakeAcceptedGoalHandle:
    def __init__(
        self,
        action_type,
        motion_name,
        *,
        execute_error=None,
        missing_uuid=False,
        terminal_error=None,
    ):
        self.request = action_type.Goal()
        self.request.motion_name = motion_name
        self.goal_id = type(
            'GoalId',
            (),
            {'uuid': [] if missing_uuid else [1] * 16},
        )()
        self.execute_error = execute_error
        self.terminal_error = terminal_error
        self.execute_calls = 0
        self.terminal_calls = []
        self.feedback = []

    def execute(self):
        self.execute_calls += 1
        if self.execute_error is not None:
            raise self.execute_error

    def publish_feedback(self, feedback):
        self.feedback.append(feedback)

    def _terminal(self, name):
        self.terminal_calls.append(name)
        if self.terminal_error is not None:
            raise self.terminal_error

    def succeed(self):
        self._terminal('succeed')

    def abort(self):
        self._terminal('abort')

    def canceled(self):
        self._terminal('canceled')


def test_request_shutdown_is_nonblocking_and_drains_accepted_goal(
    tmp_path,
):
    ros = load_ros_integration_types()
    context = ros['Context']()
    ros['rclpy'].init(context=context)
    runner = RosInjectedRunner()
    ros, node, unused_topics = make_ros_gait_node(
        tmp_path,
        context,
        runner,
    )
    try:
        request = ros['ExecuteMotion'].Goal()
        request.motion_name = 'start_jump'
        assert (
            node._motion_goal_callback(request)
            == ros['GoalResponse'].ACCEPT
        )
        goal_handle = FakeAcceptedGoalHandle(
            ros['ExecuteMotion'],
            'start_jump',
        )
        node._motion_handle_accepted_callback(goal_handle)
        assert node._motion_slot.state == 'ACCEPTED'
        assert not node._motion_slot.worker_started_event.is_set()

        started = time.monotonic()
        node.request_shutdown()
        elapsed = time.monotonic() - started
        assert elapsed < 0.05
        assert node._shutdown_requested.is_set()
        assert node._motion_slot.stop_event.is_set()
        assert node._motion_slot.first_abort_reason == 'node_shutdown'

        result = node._execute_motion_action(goal_handle)
        assert not result.success
        assert goal_handle.terminal_calls == ['abort']
        assert runner.calls == []
        assert node.shutdown_drained()
    finally:
        node.destroy_node()
        context.try_shutdown()


def test_shutdown_before_handle_binding_drains_reserved_goal(tmp_path):
    ros = load_ros_integration_types()
    context = ros['Context']()
    ros['rclpy'].init(context=context)
    runner = RosInjectedRunner()
    ros, node, unused_topics = make_ros_gait_node(
        tmp_path,
        context,
        runner,
    )
    try:
        request = ros['ExecuteMotion'].Goal()
        request.motion_name = 'start_jump'
        assert (
            node._motion_goal_callback(request)
            == ros['GoalResponse'].ACCEPT
        )
        assert node._motion_slot.state == 'RESERVED'

        node.request_shutdown()
        assert node._motion_slot.state == 'STOPPING'
        goal_handle = FakeAcceptedGoalHandle(
            ros['ExecuteMotion'],
            'start_jump',
        )
        node._motion_handle_accepted_callback(goal_handle)
        assert goal_handle.execute_calls == 1
        assert node._motion_slot.state == 'STOPPING'
        assert 'action_reservation_fault' not in node._safety_faults

        result = node._execute_motion_action(goal_handle)
        assert not result.success
        assert goal_handle.terminal_calls == ['abort']
        assert runner.calls == []
        assert node.shutdown_drained()
    finally:
        node.destroy_node()
        context.try_shutdown()


def test_cancel_before_execute_finishes_canceled_without_helper(tmp_path):
    ros = load_ros_integration_types()
    context = ros['Context']()
    ros['rclpy'].init(context=context)
    runner = RosInjectedRunner()
    ros, node, unused_topics = make_ros_gait_node(
        tmp_path,
        context,
        runner,
    )
    try:
        request = ros['ExecuteMotion'].Goal()
        request.motion_name = 'start_jump'
        assert (
            node._motion_goal_callback(request)
            == ros['GoalResponse'].ACCEPT
        )
        goal_handle = FakeAcceptedGoalHandle(
            ros['ExecuteMotion'],
            'start_jump',
        )
        node._motion_handle_accepted_callback(goal_handle)
        assert (
            node._motion_cancel_callback(goal_handle)
            == ros['CancelResponse'].ACCEPT
        )

        result = node._execute_motion_action(goal_handle)

        assert not result.success
        assert goal_handle.terminal_calls == ['canceled']
        assert 'reason=cancel_requested' in result.message
        assert runner.calls == []
        assert node._motion_slot is None
    finally:
        node.destroy_node()
        context.try_shutdown()


def test_context_shutdown_callback_only_requests_local_shutdown(tmp_path):
    ros = load_ros_integration_types()
    context = ros['Context']()
    ros['rclpy'].init(context=context)
    ros, node, unused_topics = make_ros_gait_node(
        tmp_path,
        context,
        RosInjectedRunner(),
    )
    try:
        context.on_shutdown(node.request_shutdown_from_context)
        started = time.monotonic()
        context.try_shutdown()
        elapsed = time.monotonic() - started
        assert elapsed < 0.05
        assert node._shutdown_requested.is_set()
        assert not node._ros_cleanup_allowed.is_set()
        assert node.shutdown_drained()
    finally:
        node.destroy_node()
        context.try_shutdown()


def test_context_invalid_cleanup_mode_reaps_helper_without_unlock(
    tmp_path,
):
    ros = load_ros_integration_types()
    context = ros['Context']()
    ros['rclpy'].init(context=context)
    runner = RosInjectedRunner(blocking=True)
    ros, node, topics = make_ros_gait_node(tmp_path, context, runner)
    lifecycle_locks = []
    original_lock_callback = (
        node.front_jump_supervisor._publish_lock_callback
    )

    def tracked_lock_callback(locked):
        lifecycle_locks.append(bool(locked))
        return original_lock_callback(locked)

    node.front_jump_supervisor._publish_lock_callback = (
        tracked_lock_callback
    )
    executor = ros['MultiThreadedExecutor'](
        num_threads=4,
        context=context,
    )
    executor.add_node(node)
    client_node = None
    try:
        (
            client_node,
            action_client,
            unused_json_pub,
            unused_timer,
        ) = create_ros_test_client(
            ros,
            context,
            topics,
            ros['ExecuteMotion'],
        )
        executor.add_node(client_node)
        assert spin_until(
            executor,
            action_client.server_is_ready,
            timeout=2.0,
        )
        goal = ros['ExecuteMotion'].Goal()
        goal.motion_name = 'start_jump'
        send_future = action_client.send_goal_async(goal)
        assert spin_until(executor, send_future.done)
        assert send_future.result().accepted
        assert spin_until(
            executor,
            lambda: bool(runner.processes),
            timeout=2.0,
        )

        node._ros_cleanup_allowed.clear()
        node.request_shutdown()
        assert node._action_workers_done_event.wait(timeout=2.0)
        process = runner.processes[0]
        assert process.terminated
        assert process.reaped
        assert lifecycle_locks == [True]
        assert not node._ros_cleanup_allowed.is_set()
        assert node.front_jump_supervisor.cleanup_pending
        assert node._control_lock_publisher.desired_state is True
        assert node._cleanup_guard.load()['state'] == 'DIRTY'
    finally:
        destroy_ros_test_graph(
            context,
            executor,
            *(
                [node, client_node]
                if client_node is not None
                else [node]
            ),
        )


def test_shutdown_is_not_drained_while_cleanup_retry_is_pending(tmp_path):
    ros = load_ros_integration_types()
    context = ros['Context']()
    ros['rclpy'].init(context=context)
    ros, node, unused_topics = make_ros_gait_node(
        tmp_path,
        context,
        RosInjectedRunner(),
    )

    class PendingCleanupSupervisor:
        active_context = None
        active_process = None
        cleanup_pending = True

    try:
        node.front_jump_supervisor = PendingCleanupSupervisor()
        assert not node.shutdown_drained()
        node.front_jump_supervisor.cleanup_pending = False
        assert node.shutdown_drained()
    finally:
        node.destroy_node()
        context.try_shutdown()


@pytest.mark.parametrize(
    'terminal_state',
    ['succeed', 'abort', 'canceled'],
)
def test_terminal_api_exception_is_single_call_and_blocks_all_motion(
    tmp_path,
    terminal_state,
):
    ros = load_ros_integration_types()
    context = ros['Context']()
    ros['rclpy'].init(context=context)
    ros, node, unused_topics = make_ros_gait_node(
        tmp_path,
        context,
        RosInjectedRunner(),
    )
    try:
        slot, reason = node._try_reserve_motion_slot(
            entry_type='action',
            motion_name='start_jump',
            command='JUMP_START_OBSTACLE',
            identity='pending_uuid',
        )
        assert not reason
        goal_handle = FakeAcceptedGoalHandle(
            ros['ExecuteMotion'],
            'start_jump',
            terminal_error=RuntimeError('terminal failed'),
        )
        slot.goal_handle = goal_handle
        if terminal_state == 'canceled':
            slot.cancel_accepted = True

        assert not node._attempt_action_terminal(
            goal_handle,
            slot,
            terminal_state,
        )
        assert not node._attempt_action_terminal(
            goal_handle,
            slot,
            terminal_state,
        )
        assert goal_handle.terminal_calls == [terminal_state]
        assert slot.expected_terminal == terminal_state
        assert slot.state == 'FINALIZING'
        assert not slot.completion_event.is_set()
        assert 'action_terminal_delivery_fault' in node._safety_faults

        action_request = ros['ExecuteMotion'].Goal()
        action_request.motion_name = 'finish_jump'
        assert (
            node._motion_goal_callback(action_request)
            == ros['GoalResponse'].REJECT
        )
        status_before = node._status
        json_result = node.handle_command(
            {'command': 'HOLD_STABLE', 'duration_sec': 0.01},
            entry_type='json',
        )
        assert not json_result.success
        assert node._status == status_before
    finally:
        node.destroy_node()
        context.try_shutdown()


@pytest.mark.parametrize(
    'missing_uuid, execute_error',
    [
        (True, None),
        (False, RuntimeError('execute scheduling failed')),
    ],
)
def test_action_reservation_binding_or_execute_fault_is_fail_closed(
    tmp_path,
    missing_uuid,
    execute_error,
):
    ros = load_ros_integration_types()
    context = ros['Context']()
    ros['rclpy'].init(context=context)
    runner = RosInjectedRunner()
    ros, node, unused_topics = make_ros_gait_node(
        tmp_path,
        context,
        runner,
    )
    try:
        request = ros['ExecuteMotion'].Goal()
        request.motion_name = 'start_jump'
        assert (
            node._motion_goal_callback(request)
            == ros['GoalResponse'].ACCEPT
        )
        goal_handle = FakeAcceptedGoalHandle(
            ros['ExecuteMotion'],
            'start_jump',
            execute_error=execute_error,
            missing_uuid=missing_uuid,
        )
        node._motion_handle_accepted_callback(goal_handle)

        assert node._motion_slot.state == (
            'FAULTED' if missing_uuid else 'FINALIZING'
        )
        assert node._motion_slot.reservation_token
        assert node._motion_slot.fault_type == 'action_reservation_fault'
        assert 'action_reservation_fault' in node._safety_faults
        assert goal_handle.terminal_calls == ['abort']
        assert runner.calls == []
        if execute_error is not None:
            late_result = node._execute_motion_action(goal_handle)
            assert not late_result.success
            assert goal_handle.terminal_calls == ['abort']
            assert 'stage=action_reservation' in late_result.message
            assert 'helper_started=false' in late_result.message
            assert (
                'sdk_request_may_have_been_sent=false'
                in late_result.message
            )
            assert 'cleanup_completed=false' in late_result.message
    finally:
        node.destroy_node()
        context.try_shutdown()


def test_full_boot_guard_needs_live_estop_and_final_zero_checks(
    tmp_path,
):
    ros = load_ros_integration_types()
    guard_path = tmp_path / 'runtime' / 'guard.json'
    old_guard = PersistentCleanupGuard(
        guard_path,
        boot_id_reader=lambda: 'prior-compute-boot',
    )
    record = old_guard.begin_dirty(guard_operation())
    fault_id = record['cleanup_fault_id']
    context = ros['Context']()
    ros['rclpy'].init(context=context)
    runner = RosInjectedRunner()
    ros, node, unused_topics = make_ros_gait_node(
        tmp_path,
        context,
        runner,
    )
    try:
        assert node._full_boot_recovery_guard_id == fault_id
        assert not node._accept_new_motion
        assert guard_path.exists()

        estop = ros['Bool']()
        estop.data = False
        node._on_front_jump_estop_state(estop)
        assert guard_path.exists()
        node._on_front_jump_final_cmd(ros['Twist']())
        assert guard_path.exists()
        node._on_front_jump_final_cmd(ros['Twist']())

        assert not guard_path.exists()
        assert node._full_boot_recovery_guard_id == ''
        assert node._accept_new_motion
        assert not node._safety_faults
        assert node._control_lock_publisher.desired_state is False
    finally:
        node.destroy_node()
        context.try_shutdown()


def test_same_boot_node_restart_never_auto_clears_dirty_guard(tmp_path):
    ros = load_ros_integration_types()
    guard_path = tmp_path / 'runtime' / 'guard.json'
    boot_id = Path('/proc/sys/kernel/random/boot_id').read_text(
        encoding='ascii'
    ).strip()
    old_guard = PersistentCleanupGuard(
        guard_path,
        boot_id_reader=lambda: boot_id,
    )
    old_guard.begin_dirty(guard_operation())
    context = ros['Context']()
    ros['rclpy'].init(context=context)
    ros, node, unused_topics = make_ros_gait_node(
        tmp_path,
        context,
        RosInjectedRunner(),
    )
    try:
        estop = ros['Bool']()
        estop.data = False
        node._on_front_jump_estop_state(estop)
        node._on_front_jump_final_cmd(ros['Twist']())
        node._on_front_jump_final_cmd(ros['Twist']())

        assert guard_path.exists()
        assert not node._accept_new_motion
        assert node._full_boot_recovery_guard_id == ''
        assert node._control_lock_publisher.desired_state is True
    finally:
        node.destroy_node()
        context.try_shutdown()


def test_same_boot_restart_restores_every_persisted_fault_type(tmp_path):
    ros = load_ros_integration_types()
    guard_path = tmp_path / 'runtime' / 'guard.json'
    boot_id = Path('/proc/sys/kernel/random/boot_id').read_text(
        encoding='ascii'
    ).strip()
    guard = PersistentCleanupGuard(
        guard_path,
        boot_id_reader=lambda: boot_id,
    )
    record = guard.begin_dirty(guard_operation())
    guard.record_fault('lock_publish_fault', 'lock publish failed')
    guard.record_fault('cleanup_fault', 'cleanup incomplete')
    context = ros['Context']()
    ros['rclpy'].init(context=context)
    ros, node, unused_topics = make_ros_gait_node(
        tmp_path,
        context,
        RosInjectedRunner(),
    )
    try:
        assert set(node._safety_faults) >= {
            'lock_publish_fault',
            'cleanup_fault',
        }
        result = node._execute_front_jump_recovery(
            {
                'cleanup_fault_id': record['cleanup_fault_id'],
                'confirm_no_front_jump_helper': True,
            }
        )
        assert not result.success
        assert 'lock_publish_fault' in result.message
        assert guard_path.exists()
        assert node._control_lock_publisher.desired_state is True
    finally:
        node.destroy_node()
        context.try_shutdown()


def test_executor_shutdown_fault_never_claims_clean_state(tmp_path):
    ros = load_ros_integration_types()
    context = ros['Context']()
    ros['rclpy'].init(context=context)
    ros, node, unused_topics = make_ros_gait_node(
        tmp_path,
        context,
        RosInjectedRunner(),
    )
    try:
        node.record_executor_shutdown_fault(
            'executor shutdown returned false'
        )
        assert node._fatal_shutdown_fault
        assert not node._accept_new_motion
        assert 'fatal_shutdown_fault' in node._safety_faults
        assert node._cleanup_guard.load()['state'] == 'DIRTY'
        assert node._control_lock_publisher.desired_state is True
        assert not node.prepare_finalize_shutdown(context_valid=True)
        assert not node.commit_finalize_shutdown(
            executor_shutdown_succeeded=True,
            context_valid=True,
        )
    finally:
        node.destroy_node()
        context.try_shutdown()


def test_jump_route_precedes_generic_lock_and_nonjump_path_remains():
    source_path = (
        Path(__file__).parents[1]
        / 'rk_locomotion'
        / 'gait_control_node.py'
    )
    source = source_path.read_text(encoding='utf-8')
    handle_start = source.index('    def handle_command(')
    handle_end = source.index('    def execute_stop(self):', handle_start)
    handle_source = source[handle_start:handle_end]

    jump_route = handle_source.index(
        "if command in ('JUMP_START_OBSTACLE', 'JUMP_END_OBSTACLE')"
    )
    generic_lock = handle_source.index('lock_result = self.publish_lock(True)')
    assert jump_route < generic_lock
    assert "if command == 'RECOVERY_STAND':" in handle_source
    assert "elif command == 'LOW_SPEED_MOVE':" in handle_source
    assert "elif command == 'TURN_IN_PLACE':" in handle_source
    assert 'execute_jump_obstacle' not in handle_source


def test_node_declares_required_qos_and_four_thread_executor():
    source_path = (
        Path(__file__).parents[1]
        / 'rk_locomotion'
        / 'gait_control_node.py'
    )
    source = source_path.read_text(encoding='utf-8')

    assert 'FINAL_CMD_QOS = QoSProfile(' in source
    assert 'depth=10' in source
    assert 'durability=QoSDurabilityPolicy.VOLATILE' in source
    assert 'ESTOP_STATE_QOS = QoSProfile(' in source
    assert 'depth=1' in source
    assert (
        'durability=QoSDurabilityPolicy.TRANSIENT_LOCAL'
        in source
    )
    assert (
        'MultiThreadedExecutor(num_threads=4, context=context)'
        in source
    )
    assert 'self.motion_action_callback_group = ReentrantCallbackGroup()' in (
        source
    )
    assert 'self.front_jump_callback_group = ReentrantCallbackGroup()' in (
        source
    )


def test_old_timed_nonzero_jump_path_is_absent():
    source_path = (
        Path(__file__).parents[1]
        / 'rk_locomotion'
        / 'gait_control_node.py'
    )
    source = source_path.read_text(encoding='utf-8')

    for removed_name in (
        'jump_phase1_vx',
        'jump_phase1_duration',
        'jump_pause_duration',
        'jump_phase2_vx',
        'jump_phase2_duration',
        'prepare_obstacle_pose',
        '_run_obstacle_velocity_phase',
        'execute_jump_obstacle',
    ):
        assert removed_name not in source


def test_guard_update_rechecks_permissions_owner_and_target_identity(tmp_path):
    guard_path = tmp_path / 'runtime' / 'guard.json'
    guard = PersistentCleanupGuard(
        guard_path,
        boot_id_reader=lambda: 'boot-a',
    )
    guard.begin_dirty(guard_operation())
    original = guard_path.read_bytes()

    os.chmod(str(guard_path), 0o644)
    with pytest.raises(CleanupGuardError, match='permissions must be 0600'):
        guard.update(lambda record: record['faults'].append({
            'fault_id': uuid.uuid4().hex,
            'fault_type': 'test_fault',
            'reason': 'must not overwrite bad permissions',
        }))
    assert guard_path.read_bytes() == original

    os.chmod(str(guard_path), 0o600)
    os.chmod(str(guard_path), 0o666)
    with pytest.raises(CleanupGuardError, match='permissions must be 0600'):
        guard.update(lambda record: None)
    assert guard_path.read_bytes() == original

    os.chmod(str(guard_path), 0o600)
    os.chmod(str(guard_path.parent), 0o755)
    with pytest.raises(CleanupGuardError, match='permissions must be 0700'):
        guard.update(lambda record: None)
    os.chmod(str(guard_path.parent), 0o700)

    replacement = tmp_path / 'replacement.json'
    replacement.write_bytes(original)
    os.chmod(str(replacement), 0o600)
    guard_path.unlink()
    guard_path.symlink_to(replacement)
    with pytest.raises(CleanupGuardError, match='symbolic link'):
        guard.update(lambda record: None)
    assert replacement.read_bytes() == original

    clear_path = tmp_path / 'runtime-clear' / 'guard.json'
    clear_guard = PersistentCleanupGuard(
        clear_path,
        boot_id_reader=lambda: 'boot-a',
    )
    clear_record = clear_guard.begin_dirty(guard_operation())
    clear_guard.update(
        lambda record: record['cleanup'].update(
            {'cleanup_completed': True}
        )
    )
    clear_original = clear_path.read_bytes()
    os.chmod(str(clear_path), 0o644)
    with pytest.raises(CleanupGuardError, match='permissions must be 0600'):
        clear_guard.clear(clear_record['cleanup_fault_id'])
    assert clear_path.read_bytes() == clear_original


@pytest.mark.parametrize(
    'identity_failure',
    [
        'start_ticks_read_failure',
        'executable_read_failure',
        'pgid_mismatch',
        'session_mismatch',
    ],
)
def test_subprocess_identity_failure_never_signals_an_unverified_group(
    monkeypatch,
    identity_failure,
):
    unrelated = subprocess.Popen(
        [sys.executable, '-c', 'import time; time.sleep(60)'],
        start_new_session=True,
    )
    killpg_calls = []
    runner = SubprocessRunner()

    if identity_failure == 'start_ticks_read_failure':
        monkeypatch.setattr(
            front_jump_module,
            '_read_process_stat',
            lambda unused_pid: (_ for _ in ()).throw(
                OSError('simulated start-ticks read failure')
            ),
        )
    elif identity_failure == 'executable_read_failure':
        monkeypatch.setattr(
            front_jump_module,
            '_read_process_executable',
            lambda unused_pid: (_ for _ in ()).throw(
                OSError('simulated executable identity read failure')
            ),
        )
    elif identity_failure == 'pgid_mismatch':
        original_getpgid = os.getpgid
        monkeypatch.setattr(
            front_jump_module.os,
            'getpgid',
            lambda pid: original_getpgid(pid) + 1,
        )
    else:
        original_getsid = os.getsid
        monkeypatch.setattr(
            front_jump_module.os,
            'getsid',
            lambda pid: original_getsid(pid) + 1,
        )
    monkeypatch.setattr(
        front_jump_module.os,
        'killpg',
        lambda pgid, signal_number: killpg_calls.append(
            (pgid, signal_number)
        ),
    )
    try:
        with pytest.raises(ProcessStartError) as raised:
            runner.start(
                [sys.executable, '-c', 'import time; time.sleep(60)']
            )
        error = raised.value
        assert error.process_started
        assert error.identity_unverified
        assert not error.cleanup_completed
        assert error.diagnostics['failure_stage'] == (
            'post_popen_identity_collection'
        )
        assert error.diagnostics['leader_terminate_sent']
        assert error.diagnostics['leader_reaped']
        assert killpg_calls == []
        assert unrelated.poll() is None
    finally:
        try:
            unrelated.terminate()
            unrelated.wait(timeout=1.0)
        except Exception:
            try:
                unrelated.kill()
                unrelated.wait(timeout=1.0)
            except Exception:
                pass


def test_subprocess_handle_rejects_changed_pid_identity_before_killpg(
    monkeypatch,
):
    runner = SubprocessRunner()
    handle = runner.start(
        [sys.executable, '-c', 'import time; time.sleep(60)']
    )
    killpg_calls = []
    original_read_stat = front_jump_module._read_process_stat

    def changed_start_ticks(pid):
        record = original_read_stat(pid)
        record['start_ticks'] += 1
        return record

    monkeypatch.setattr(
        front_jump_module,
        '_read_process_stat',
        changed_start_ticks,
    )
    monkeypatch.setattr(
        front_jump_module.os,
        'killpg',
        lambda pgid, signal_number: killpg_calls.append(
            (pgid, signal_number)
        ),
    )
    try:
        with pytest.raises(ProcessIdentityError, match='start ticks changed'):
            handle.terminate()
        assert killpg_calls == []
    finally:
        # The identity test deliberately forbids group signalling. It uses a
        # no-child leader, so direct Popen cleanup cannot create an orphan.
        process = handle._process
        try:
            process.terminate()
            process.wait(timeout=1.0)
        except Exception:
            try:
                process.kill()
                process.wait(timeout=1.0)
            except Exception:
                pass
        try:
            handle.reap(timeout=0.1)
        except Exception:
            pass


def test_real_context_shutdown_reaps_ignoring_group_no_ros_output(tmp_path):
    ros = load_ros_integration_types()
    context = ros['Context']()
    ros['rclpy'].init(context=context)
    started = threading.Event()
    finished = threading.Event()
    zero_calls = []
    lock_calls = []
    ready_path = tmp_path / 'sigterm_ignore_ready'
    process_code = (
        "import signal,subprocess,sys,time;"
        "signal.signal(signal.SIGTERM, signal.SIG_IGN);"
        "subprocess.Popen([sys.executable, '-c', "
        "'import signal,time;signal.signal(signal.SIGTERM,"
        "signal.SIG_IGN);time.sleep(60)']);"
        "open(sys.argv[1], 'w').write('ready');"
        "time.sleep(60)"
    )

    class NotifyingRunner:
        def __init__(self):
            self.handle = None

        def start(self, unused_argv):
            self.handle = SubprocessRunner().start(
                [sys.executable, '-c', process_code, str(ready_path)]
            )
            started.set()
            return self.handle

    runner = NotifyingRunner()
    supervisor = FrontJumpSupervisor(
        profiles={
            'start': FrontJumpProfile(
                name='start',
                pre_stop_duration=0.0,
                final_zero_epsilon=0.001,
                final_zero_confirm_samples=2,
                final_zero_timeout=2.0,
                sdk_timeout=5.0,
                post_settle_duration=0.0,
            ),
            'finish': FrontJumpProfile(
                name='finish',
                pre_stop_duration=0.0,
                final_zero_epsilon=0.001,
                final_zero_confirm_samples=2,
                final_zero_timeout=2.0,
                sdk_timeout=5.0,
                post_settle_duration=0.0,
            ),
        },
        config=FrontJumpConfig(
            sdk_action_executable=sys.executable,
            sdk_network_interface='fake0',
            zero_publish_rate_hz=50.0,
            final_cmd_stale_timeout=0.50,
            estop_state_stale_timeout=0.50,
        ),
        publish_lock=lambda locked: lock_calls.append(bool(locked)),
        publish_zero=lambda: zero_calls.append(time.monotonic()),
        process_runner=runner,
        executable_resolver=lambda unused: os.path.realpath(sys.executable),
        interface_index_resolver=lambda unused: 1,
        ros_cleanup_allowed=context.ok,
    )

    def request_context_shutdown():
        # rclpy Humble's callback registry invokes regular functions once
        # while resolving them.  Return None so it never mistakes the bool
        # from request_stop() for a second callback.
        supervisor.request_stop('context_shutdown')

    context.on_shutdown(request_context_shutdown)

    def feed_safety_evidence():
        while not finished.is_set() and context.ok():
            supervisor.update_estop(False)
            supervisor.update_final_command(0.0, 0.0, 0.0)
            time.sleep(0.01)

    outcome_box = []
    feeder = threading.Thread(target=feed_safety_evidence)
    worker = threading.Thread(
        target=lambda: outcome_box.append(supervisor.run('start_jump'))
    )
    feeder.start()
    worker.start()
    try:
        assert started.wait(timeout=2.0)
        ready_deadline = time.monotonic() + 2.0
        while not ready_path.exists() and time.monotonic() < ready_deadline:
            time.sleep(0.01)
        assert ready_path.read_text() == 'ready'
        calls_before_shutdown = len(zero_calls)
        context.try_shutdown()
        worker.join(timeout=3.0)
        assert not worker.is_alive()
        assert outcome_box
        outcome = outcome_box[0]
        completed = supervisor.completed_contexts[-1]
        assert len(zero_calls) == calls_before_shutdown
        assert completed.helper_terminate_sent
        assert completed.helper_kill_sent
        assert completed.helper_reaped
        assert completed.helper_group_empty
        assert runner.handle.poll() is not None
        assert runner.handle.group_empty()
        assert not outcome.cleanup_completed
        assert lock_calls == [True]
    finally:
        finished.set()
        feeder.join(timeout=1.0)
        worker.join(timeout=1.0)
        if runner.handle is not None:
            try:
                runner.handle.kill()
            except (OSError, ProcessLookupError, ProcessIdentityError):
                pass
            try:
                runner.handle.reap(timeout=0.5)
            except Exception:
                pass
        context.try_shutdown()


@pytest.mark.parametrize(
    'motion_name,command_success,cancel_accepted',
    [
        ('hold_stable', True, False),
        ('hold_stable', False, False),
        ('start_jump', True, False),
        ('start_jump', False, True),
    ],
)
def test_terminal_delivery_failure_never_returns_success(
    tmp_path,
    motion_name,
    command_success,
    cancel_accepted,
):
    ros = load_ros_integration_types()
    context = ros['Context']()
    ros['rclpy'].init(context=context)
    ros, node, unused_topics = make_ros_gait_node(
        tmp_path,
        context,
        RosInjectedRunner(),
    )
    try:
        request = ros['ExecuteMotion'].Goal()
        request.motion_name = motion_name
        assert (
            node._motion_goal_callback(request)
            == ros['GoalResponse'].ACCEPT
        )
        goal_handle = FakeAcceptedGoalHandle(
            ros['ExecuteMotion'],
            motion_name,
            terminal_error=RuntimeError('terminal delivery failed'),
        )
        node._motion_handle_accepted_callback(goal_handle)
        slot = node._motion_slot
        if cancel_accepted:
            slot.cancel_accepted = True
            slot.cancel_event.set()
        if motion_name == 'start_jump':
            node._run_front_jump = lambda *args, **kwargs: FrontJumpOutcome(
                success=command_success,
                terminal_state=(
                    'succeed' if command_success else 'abort'
                ),
                stage='supervised_flow_done',
                reason='test_outcome',
                helper_started=False,
                sdk_request_may_have_been_sent=False,
                cleanup_completed=True,
                sdk_command_accepted=command_success,
                post_settle_completed=command_success,
            )
        else:
            original_handle_command = node.handle_command
            node.handle_command = lambda *args, **kwargs: type(
                'Result',
                (),
                {
                    'success': command_success,
                    'status': 'DONE' if command_success else 'FAILED',
                    'message': 'test command result',
                },
            )()

        result = node._execute_motion_action(goal_handle)

        assert not result.success
        assert goal_handle.terminal_calls == [slot.expected_terminal]
        assert 'terminal_delivery_succeeded=false' in result.message
        assert 'terminal_delivery_fault=true' in result.message
        assert 'action_terminal_delivery_fault' in node._safety_faults
        next_request = ros['ExecuteMotion'].Goal()
        next_request.motion_name = 'finish_jump'
        assert (
            node._motion_goal_callback(next_request)
            == ros['GoalResponse'].REJECT
        )
        if motion_name == 'hold_stable':
            node.handle_command = original_handle_command
        assert not node.handle_command(
            {'command': 'HOLD_STABLE', 'duration_sec': 0.01},
            entry_type='json',
        ).success
    finally:
        node.destroy_node()
        context.try_shutdown()


def test_slot_finalization_transition_and_completion_semantics(tmp_path):
    ros = load_ros_integration_types()
    context = ros['Context']()
    ros['rclpy'].init(context=context)
    ros, node, unused_topics = make_ros_gait_node(
        tmp_path,
        context,
        RosInjectedRunner(),
    )
    try:
        slot, reason = node._try_reserve_motion_slot(
            entry_type='json',
            motion_name='hold_stable',
            command='HOLD_STABLE',
            identity='pending',
        )
        assert not reason
        node.handle_command(
            {'command': 'HOLD_STABLE', 'duration_sec': 0.0},
            slot=slot,
            entry_type='json',
        )
        node._complete_motion_slot(slot)
        assert slot.transitions == [
            'RESERVED',
            'ACCEPTED',
            'EXECUTING',
            'FINALIZING',
            'DONE',
        ]
        assert slot.worker_done_event.is_set()
        assert slot.completion_event.is_set()

        fault_slot, reason = node._try_reserve_motion_slot(
            entry_type='json',
            motion_name='hold_stable',
            command='HOLD_STABLE',
            identity='pending',
        )
        assert not reason
        assert node._transition_motion_slot(fault_slot, 'ACCEPTED')
        assert node._transition_motion_slot(fault_slot, 'EXECUTING')
        fault_slot.worker_started_event.set()
        node._fault_current_motion_slot('test_fault', 'worker still active')
        assert not fault_slot.completion_event.is_set()
        node._complete_motion_slot(fault_slot)
        assert fault_slot.transitions == [
            'RESERVED', 'ACCEPTED', 'EXECUTING', 'STOPPING', 'FINALIZING',
            'FAULTED',
        ]
        assert fault_slot.worker_done_event.is_set()
        assert fault_slot.completion_event.is_set()

        illegal_slot = type(fault_slot)(
            reservation_token='illegal',
            entry_type='json',
            motion_name='hold_stable',
            command='HOLD_STABLE',
            identity='illegal',
        )
        assert not node._transition_motion_slot(illegal_slot, 'DONE')
        assert illegal_slot.state == 'FAULTED'
        assert 'motion_slot_transition_fault' in node._safety_faults
    finally:
        node.destroy_node()
        context.try_shutdown()


@pytest.mark.parametrize(
    'executor_ok,context_valid,inject_fault,expected_false',
    [
        (True, True, False, True),
        (False, True, False, False),
        (True, False, False, False),
        (True, True, True, False),
    ],
)
def test_shutdown_prepare_commit_never_unlocks_before_executor_result(
    tmp_path,
    executor_ok,
    context_valid,
    inject_fault,
    expected_false,
):
    ros = load_ros_integration_types()
    context = ros['Context']()
    ros['rclpy'].init(context=context)
    ros, node, unused_topics = make_ros_gait_node(
        tmp_path,
        context,
        RosInjectedRunner(),
    )

    class RecordingPublisher:
        def __init__(self):
            self.states = []

        def publish(self, message):
            self.states.append(bool(message.data))

    publisher = RecordingPublisher()
    node._control_lock_publisher._publisher = publisher
    try:
        if inject_fault:
            node._fault_current_motion_slot('cleanup_fault', 'test fault')
        node.request_shutdown()
        prepared = node.prepare_finalize_shutdown(
            context_valid=context_valid,
        )
        assert (prepared is True) is (context_valid and not inject_fault)
        before_commit = list(publisher.states)
        assert False not in before_commit
        committed = node.commit_finalize_shutdown(
            executor_shutdown_succeeded=executor_ok and prepared,
            context_valid=context_valid,
            failure_reason='test executor outcome',
        )
        assert committed is expected_false
        assert (False in publisher.states) is expected_false
        if not expected_false and context_valid:
            assert publisher.states and all(publisher.states)
        assert not node.commit_finalize_shutdown(
            executor_shutdown_succeeded=True,
            context_valid=True,
        )
    finally:
        node.destroy_node()
        context.try_shutdown()


@pytest.mark.parametrize(
    'shutdown_result,expected_executor_ok',
    [
        (True, True),
        (False, False),
        (RuntimeError('executor shutdown exploded'), False),
    ],
)
def test_shutdown_coordinator_waits_for_executor_before_commit(
    shutdown_result,
    expected_executor_ok,
):
    ros = load_ros_integration_types()
    from rk_locomotion.gait_control_node import (
        _shutdown_executor_then_commit,
    )

    del ros
    events = []

    class FakeExecutor:
        def shutdown(self, *, timeout_sec):
            events.append(('executor_shutdown', timeout_sec))
            if isinstance(shutdown_result, BaseException):
                raise shutdown_result
            return shutdown_result

    class FakeNode:
        def commit_finalize_shutdown(
            self,
            *,
            executor_shutdown_succeeded,
            context_valid,
            failure_reason,
        ):
            events.append(
                (
                    'commit',
                    executor_shutdown_succeeded,
                    context_valid,
                    failure_reason,
                )
            )
            return bool(executor_shutdown_succeeded and context_valid)

    executor_ok, finalized = _shutdown_executor_then_commit(
        FakeNode(),
        FakeExecutor(),
        prepared=True,
        context_valid=True,
    )

    assert executor_ok is expected_executor_ok
    assert finalized is expected_executor_ok
    assert events[0] == ('executor_shutdown', 2.0)
    assert events[1][0] == 'commit'
    assert events[1][1] is expected_executor_ok
    if isinstance(shutdown_result, BaseException):
        assert 'RuntimeError: executor shutdown exploded' in events[1][3]
    elif not shutdown_result:
        assert 'timeout with outstanding callbacks' in events[1][3]
