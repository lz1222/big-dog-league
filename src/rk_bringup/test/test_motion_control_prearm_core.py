"""motion-control prearm 一次性恢复合同的无硬件回归。"""

from pathlib import Path

from rk_bringup.motion_control_prearm_core import (
    mode_recovery_plan,
    sport_recovery_plan,
    startup_stop_plan,
)


def test_empty_mode_is_observed_without_any_release():
    assert mode_recovery_plan(0, '0', '', False) == 'MODE_OBSERVED_NO_MUTATION'
    assert mode_recovery_plan(0, '0', '', True) == 'MODE_OBSERVED_NO_MUTATION'


def test_mcf_is_classic_observation_not_a_release_trigger():
    assert mode_recovery_plan(0, '0', 'mcf', False) == 'MCF_OBSERVED_NO_MUTATION'
    assert mode_recovery_plan(0, '0', 'mcf', True) == 'MCF_OBSERVED_NO_MUTATION'


def test_other_mode_is_observed_without_mutation_but_invalid_reply_fails():
    assert mode_recovery_plan(0, '0', 'advanced', True) == 'MODE_OBSERVED_NO_MUTATION'
    assert mode_recovery_plan(-1, '0', '', True) == 'MOTION_SWITCHER_CHECK_FAILED'


def test_server_version_never_requests_service_switch():
    assert sport_recovery_plan('1.0.0.1', True) == 'SPORT_RPC_READY_NO_MUTATION'
    assert sport_recovery_plan('', True) == 'SPORT_RPC_UNAVAILABLE_NO_MUTATION'
    assert sport_recovery_plan('', False) == 'SPORT_RPC_UNAVAILABLE_NO_MUTATION'


def test_2055_is_not_a_prearm_input_and_startup_stop_is_one_shot():
    assert startup_stop_plan(0) == 'READY'
    assert startup_stop_plan(-1) == 'STARTUP_STOP_FAILED_NO_RETRY'


def test_formal_startup_does_not_pass_any_mutating_recovery_option():
    """正式入口只能运行只读 prearm，不得注入自动恢复参数。"""
    script = (
        Path(__file__).resolve().parents[1] / 'scripts' /
        'start_non_arm_competition.sh'
    ).read_text(encoding='utf-8')
    assert 'RK_COMPETITION_ENABLE_MOTION_CONTROL_RECOVERY' not in script
    assert '--enable-recovery' not in script
