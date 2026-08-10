"""省赛经典步态合同的无硬件状态机回归。"""

from rk_bringup.competition_gait_manager_core import (
    CLASSIC,
    FAULT,
    RESTORING_CLASSIC,
    CompetitionGaitManagerCore,
)


def _enter_classic(core):
    assert core.begin_startup(0, '0', 'mcf')['action'] == 'SELECT_NORMAL_ONCE'
    assert core.select_normal_result(0)['action'] == 'STAND_UP_IF_NEEDED'
    assert core.stand_up_result(False)['action'] == 'SPEED_LEVEL_1_ONCE'
    assert core.speed_level_result(0)['action'] == 'CLASSIC_WALK_ONCE'
    assert core.classic_walk_result(0)['action'] == 'VERIFY_CLASSIC'
    assert core.classic_verified(True)['action'] == 'CLASSIC_READY'


def test_mcf_is_accepted_observation_and_startup_uses_normal_once():
    core = CompetitionGaitManagerCore()
    event = core.begin_startup(0, '0', 'mcf')
    assert event['action'] == 'SELECT_NORMAL_ONCE'
    assert event['observed_name'] == 'mcf'
    assert not event['ordinary_move_allowed']


def test_classic_only_unlocks_after_real_verification():
    core = CompetitionGaitManagerCore()
    _enter_classic(core)
    assert core.state == CLASSIC
    assert core.ordinary_move_allowed


def test_any_startup_ret_failure_is_fault_without_alternate_recovery():
    core = CompetitionGaitManagerCore()
    core.begin_startup(0, '0', 'mcf')
    event = core.select_normal_result(7004)
    assert core.state == FAULT
    assert event['action'] == 'FAULT_ZERO_MOVE'
    assert not core.ordinary_move_allowed


def test_special_action_and_free_gait_both_require_classic_handback():
    core = CompetitionGaitManagerCore()
    _enter_classic(core)
    assert core.begin_special_action()['action'] == 'ZERO_THEN_SPECIAL_ACTION'
    assert core.special_complete()['action'] == 'ZERO_STOP_MOVE_THEN_CLASSIC_WALK_ONCE'
    assert core.state == RESTORING_CLASSIC
    assert core.classic_walk_result(0)['action'] == 'VERIFY_CLASSIC'
    assert core.classic_verified(True)['state'] == CLASSIC
    assert core.begin_special_gait()['action'] == 'ZERO_THEN_SPECIAL_GAIT'


def test_nonclassic_special_state_never_allows_ordinary_move():
    core = CompetitionGaitManagerCore()
    _enter_classic(core)
    core.begin_special_gait()
    assert not core.ordinary_move_allowed
