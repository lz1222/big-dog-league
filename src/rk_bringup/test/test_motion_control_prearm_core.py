"""motion-control prearm 一次性恢复合同的无硬件回归。"""

from rk_bringup.motion_control_prearm_core import (
    mode_recovery_plan,
    sport_recovery_plan,
    startup_stop_plan,
)


def test_empty_mode_never_requests_release():
    assert mode_recovery_plan(0, '0', '', False) == 'NO_RELEASE'
    assert mode_recovery_plan(0, '0', '', True) == 'NO_RELEASE'


def test_mcf_requires_explicit_recovery_and_allows_one_release():
    assert mode_recovery_plan(0, '0', 'mcf', False) == 'RECOVERY_DISABLED'
    assert mode_recovery_plan(0, '0', 'mcf', True) == 'RELEASE_ONCE'


def test_unknown_mode_is_fail_closed_without_mutation():
    assert mode_recovery_plan(0, '0', 'advanced', True) == 'UNKNOWN_MOTION_MODE'
    assert mode_recovery_plan(-1, '0', '', True) == 'MOTION_SWITCHER_CHECK_FAILED'


def test_valid_server_version_never_requests_service_switch():
    assert sport_recovery_plan('1.0.0.1', True) == 'NO_SERVICE_SWITCH'
    assert sport_recovery_plan('', True) == 'SWITCH_OFF_ON_ONCE'
    assert sport_recovery_plan('', False) == 'SPORT_RPC_RECOVERY_DISABLED'


def test_2055_is_not_a_prearm_input_and_startup_stop_is_one_shot():
    assert startup_stop_plan(0) == 'READY'
    assert startup_stop_plan(-1) == 'STARTUP_STOP_FAILED_NO_RETRY'
