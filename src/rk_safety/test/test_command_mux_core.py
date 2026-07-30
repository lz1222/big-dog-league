from importlib import import_module
import json
import math
from pathlib import Path
import sys

import pytest


PACKAGE_ROOT = str(Path(__file__).resolve().parents[1])
if PACKAGE_ROOT not in sys.path:
    sys.path.insert(0, PACKAGE_ROOT)

command_mux_core = import_module('rk_safety.command_mux_core')
CommandMuxCore = command_mux_core.CommandMuxCore
VelocityCommand = command_mux_core.VelocityCommand


def make_core(**overrides):
    parameters = {
        'line_cmd_timeout_sec': 0.5,
        'mission_cmd_timeout_sec': 0.5,
        'locomotion_cmd_timeout_sec': 0.3,
        'max_linear_x': 0.60,
        'max_linear_y': 0.15,
        'max_angular_z': 1.30,
    }
    parameters.update(overrides)
    return CommandMuxCore(**parameters)


def command(vx=0.1, vy=0.0, wz=0.0):
    return VelocityCommand(vx, vy, wz)


def assert_zero(decision):
    assert decision.command == VelocityCommand()
    assert decision.status['final_vx'] == 0.0
    assert decision.status['final_vy'] == 0.0
    assert decision.status['final_wz'] == 0.0


def test_no_command_outputs_zero():
    decision = make_core().evaluate(0.0)
    assert_zero(decision)
    assert decision.active_source == 'none'


def test_fresh_line_command_passes():
    core = make_core()
    core.update_line_command(command(0.2, 0.1, -0.3), 1.0)
    decision = core.evaluate(1.1)
    assert decision.command == command(0.2, 0.1, -0.3)
    assert decision.active_source == 'line'


def test_mission_overrides_line():
    core = make_core()
    core.update_line_command(command(0.2), 1.0)
    core.update_mission_command(command(0.4), 1.0)
    decision = core.evaluate(1.1)
    assert decision.command == command(0.4)
    assert decision.active_source == 'mission'


def test_stale_mission_falls_back_to_fresh_line():
    core = make_core()
    core.update_mission_command(command(0.4), 1.0)
    core.update_line_command(command(0.2), 1.4)
    decision = core.evaluate(1.6)
    assert decision.command == command(0.2)
    assert decision.active_source == 'line'
    assert decision.status['mission_fresh'] is False


def test_gait_lock_selects_fresh_locomotion():
    core = make_core()
    core.update_line_command(command(0.2), 1.0)
    core.update_mission_command(command(0.3), 1.0)
    core.update_locomotion_command(command(0.5), 1.0)
    core.set_gait_lock(True, 1.0)
    decision = core.evaluate(1.1)
    assert decision.command == command(0.5)
    assert decision.active_source == 'locomotion'


def test_gait_lock_stale_locomotion_does_not_fall_back():
    core = make_core()
    core.update_line_command(command(0.2), 1.0)
    core.update_mission_command(command(0.3), 1.0)
    core.update_locomotion_command(command(0.5), 1.0)
    core.set_gait_lock(True, 1.0)
    decision = core.evaluate(1.31)
    assert_zero(decision)
    assert decision.active_source == 'gait_lock_stale'


def test_arm_lock_always_outputs_zero():
    core = make_core()
    core.update_mission_command(command(0.4), 1.0)
    core.update_locomotion_command(command(0.5), 1.0)
    core.set_gait_lock(True, 1.0)
    core.set_arm_lock(True, 1.0)
    decision = core.evaluate(1.1)
    assert_zero(decision)
    assert decision.active_source == 'arm_lock'


def test_estop_overrides_every_source_and_lock():
    core = make_core()
    core.update_locomotion_command(command(0.5), 1.0)
    core.set_gait_lock(True, 1.0)
    core.set_arm_lock(True, 1.0)
    core.set_estop(True, 1.0)
    decision = core.evaluate(1.1)
    assert_zero(decision)
    assert decision.active_source == 'estop'


def test_estop_release_invalidates_old_commands():
    core = make_core()
    core.update_mission_command(command(0.4), 1.0)
    core.set_estop(True, 1.1)
    core.set_estop(False, 1.2)
    decision = core.evaluate(1.2)
    assert_zero(decision)
    assert decision.status['mission_fresh'] is False
    assert decision.status['mission_age_sec'] is None


def test_estop_enable_transition_invalidates_old_commands():
    core = make_core()
    core.update_mission_command(command(0.4), 1.0)
    changed = core.set_estop(True, 1.1)

    decision = core.evaluate(1.1)
    assert changed is True
    assert_zero(decision)
    assert decision.status['mission_fresh'] is False
    assert decision.status['mission_age_sec'] is None


def test_repeated_estop_values_are_idempotent():
    core = make_core()
    core.update_line_command(command(0.2), 1.0)
    assert core.set_estop(False, 1.1) is False
    assert core.evaluate(1.1).active_source == 'line'

    assert core.set_estop(True, 1.2) is True
    assert core.set_estop(True, 1.3) is False
    assert core.evaluate(1.3).active_source == 'estop'

    assert core.set_estop(False, 1.4) is True
    assert core.set_estop(False, 1.5) is False
    decision = core.evaluate(1.5)
    assert_zero(decision)
    assert decision.status['line_age_sec'] is None


def test_arm_lock_release_invalidates_commands_received_while_locked():
    core = make_core()
    core.set_arm_lock(True, 1.0)
    core.update_line_command(command(0.2), 1.1)
    core.set_arm_lock(False, 1.2)
    decision = core.evaluate(1.2)
    assert_zero(decision)
    assert decision.status['line_fresh'] is False


def test_gait_lock_release_invalidates_old_commands():
    core = make_core()
    core.update_line_command(command(0.2), 1.0)
    core.update_mission_command(command(0.3), 1.0)
    core.set_gait_lock(True, 1.1)
    core.set_gait_lock(False, 1.2)
    decision = core.evaluate(1.2)
    assert_zero(decision)
    assert decision.status['line_fresh'] is False
    assert decision.status['mission_fresh'] is False


def test_new_command_after_release_restores_motion():
    core = make_core()
    core.update_line_command(command(0.2), 1.0)
    core.set_estop(True, 1.1)
    core.set_estop(False, 1.2)
    core.update_line_command(command(0.3), 1.3)
    decision = core.evaluate(1.3)
    assert decision.command == command(0.3)
    assert decision.active_source == 'line'


def test_nan_command_is_rejected():
    core = make_core()
    accepted = core.update_line_command(command(float('nan')), 1.0)
    decision = core.evaluate(1.0)
    assert accepted is False
    assert_zero(decision)
    assert decision.status['invalid_command_count'] == 1


def test_infinite_command_is_rejected():
    core = make_core()
    accepted = core.update_mission_command(
        command(0.1, float('inf'), 0.0), 1.0
    )
    decision = core.evaluate(1.0)
    assert accepted is False
    assert_zero(decision)
    assert decision.status['invalid_command_count'] == 1


def test_out_of_range_command_is_clamped():
    core = make_core()
    core.update_line_command(command(1.2, -0.3, 2.6), 1.0)
    decision = core.evaluate(1.0)
    assert decision.command == command(0.60, -0.15, 1.30)
    assert decision.status['clamped'] is True


def test_time_rollback_invalidates_existing_commands():
    core = make_core()
    core.update_line_command(command(0.2), 10.0)
    assert core.evaluate(10.1).active_source == 'line'
    decision = core.evaluate(5.0)
    assert_zero(decision)
    assert decision.reason == 'time_moved_backwards'
    assert decision.status['line_age_sec'] is None


def test_time_rollback_during_update_forces_next_evaluate_to_zero():
    core = make_core()
    core.update_line_command(command(0.2), 10.0)
    assert core.evaluate(10.0).active_source == 'line'

    accepted = core.update_mission_command(command(0.4), 5.0)
    decision = core.evaluate(5.0)
    assert accepted is False
    assert_zero(decision)
    assert decision.reason == 'time_moved_backwards'
    assert decision.status['mission_fresh'] is False

    assert_zero(core.evaluate(5.1))
    core.update_mission_command(command(0.3), 5.2)
    assert core.evaluate(5.2).active_source == 'mission'


def test_time_rollback_during_lock_callback_forces_zero_cycle():
    core = make_core()
    core.update_line_command(command(0.2), 10.0)
    core.set_arm_lock(True, 10.0)
    core.set_arm_lock(False, 5.0)
    core.update_line_command(command(0.3), 5.1)

    decision = core.evaluate(5.1)
    assert_zero(decision)
    assert decision.reason == 'time_moved_backwards'
    assert core.evaluate(5.2).active_source == 'line'


def test_multiple_invalid_commands_increment_count():
    core = make_core()
    core.update_line_command(command(float('nan')), 1.0)
    core.update_mission_command(command(0.0, float('-inf')), 1.1)
    core.update_locomotion_command(command(0.0, 0.0, float('inf')), 1.2)
    decision = core.evaluate(1.2)
    assert decision.status['invalid_command_count'] == 3


def test_invalid_mission_clears_fresh_line_and_forces_zero():
    core = make_core()
    core.update_line_command(command(0.2), 1.0)
    accepted = core.update_mission_command(command(float('nan')), 1.1)

    decision = core.evaluate(1.1)
    assert accepted is False
    assert_zero(decision)
    assert decision.reason == 'invalid_command_received'
    assert decision.status['line_fresh'] is False
    assert decision.status['mission_fresh'] is False


def test_command_after_invalid_input_waits_for_forced_zero_cycle():
    core = make_core()
    core.update_mission_command(command(float('inf')), 1.0)
    core.update_line_command(command(0.2), 1.1)

    first_decision = core.evaluate(1.1)
    assert_zero(first_decision)
    assert first_decision.reason == 'invalid_command_received'
    assert core.evaluate(1.2).active_source == 'line'


def test_clamping_does_not_modify_input_command():
    core = make_core()
    original = command(1.2, -0.3, 2.6)
    before = (
        original.linear_x,
        original.linear_y,
        original.angular_z,
    )
    core.update_line_command(original, 1.0)
    core.evaluate(1.0)
    after = (
        original.linear_x,
        original.linear_y,
        original.angular_z,
    )
    assert after == before


def test_status_can_be_encoded_as_strict_json():
    core = make_core()
    core.update_line_command(command(float('nan')), 1.0)
    decision = core.evaluate(1.1)
    encoded = json.dumps(decision.status, allow_nan=False)
    decoded = json.loads(encoded)
    assert decoded['invalid_command_count'] == 1


def test_status_numbers_are_finite_or_none():
    core = make_core()
    core.update_line_command(command(0.2), 1.0)
    status = core.evaluate(10.0).status
    numeric_keys = (
        'line_age_sec',
        'mission_age_sec',
        'locomotion_age_sec',
        'final_vx',
        'final_vy',
        'final_wz',
    )
    for key in numeric_keys:
        value = status[key]
        assert value is None or math.isfinite(value)


def test_normal_mode_ignores_locomotion_command():
    core = make_core()
    core.update_locomotion_command(command(0.5), 1.0)
    decision = core.evaluate(1.1)
    assert_zero(decision)
    assert decision.active_source == 'none'


def test_gait_lock_without_locomotion_reports_stale():
    core = make_core()
    core.set_gait_lock(True, 1.0)
    decision = core.evaluate(1.0)
    assert_zero(decision)
    assert decision.active_source == 'gait_lock_stale'


@pytest.mark.parametrize(
    'parameter',
    (
        {'line_cmd_timeout_sec': 0.0},
        {'mission_cmd_timeout_sec': float('nan')},
        {'locomotion_cmd_timeout_sec': float('inf')},
        {'max_linear_x': -1.0},
        {'max_linear_y': 0.0},
        {'max_angular_z': float('nan')},
    ),
)
def test_invalid_configuration_is_rejected(parameter):
    with pytest.raises(ValueError):
        make_core(**parameter)


def test_timeout_boundary_is_still_fresh():
    core = make_core()
    core.update_line_command(command(0.2), 1.0)
    decision = core.evaluate(1.5)
    assert decision.active_source == 'line'


def test_timed_out_source_age_remains_diagnostic():
    core = make_core()
    core.update_line_command(command(0.2), 1.0)
    decision = core.evaluate(1.6)
    assert_zero(decision)
    assert decision.status['line_fresh'] is False
    assert decision.status['line_age_sec'] == pytest.approx(0.6)
