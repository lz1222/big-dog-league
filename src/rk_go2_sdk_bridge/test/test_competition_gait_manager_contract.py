"""国赛经典步态 backend 的静态安全合同。"""

from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_manager_uses_provincial_startup_calls_without_motion_move():
    source = (PACKAGE_ROOT / 'src' / 'go2_sdk_competition_gait_manager.cpp').read_text(
        encoding='utf-8')
    for required in ('SelectMode("normal")', 'StandUp()', 'SpeedLevel(1)',
                     'ClassicWalk(true)', 'StopMove()', '--execute-zero-translation'):
        assert required in source
    for forbidden in ('ReleaseMode(', 'ServiceSwitch(', '.Move('):
        assert forbidden not in source


def test_manager_is_dry_run_unless_explicitly_authorized():
    source = (PACKAGE_ROOT / 'src' / 'go2_sdk_competition_gait_manager.cpp').read_text(
        encoding='utf-8')
    assert 'bool execute{false};' in source
    assert 'if (!config.execute)' in source
    assert 'event=DRY_RUN' in source


def test_prearm_contains_no_automatic_mutation_path():
    source = (PACKAGE_ROOT / 'src' / 'go2_motion_control_prearm.cpp').read_text(
        encoding='utf-8')
    for forbidden in ('ReleaseMode(', 'ServiceSwitch(', '.Move(', '.StopMove('):
        assert forbidden not in source
    assert 'mutation_count=0' in source
