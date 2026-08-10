"""架空步态矩阵的静态安全合同。"""

from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_matrix_is_dry_run_and_has_off_ground_guard():
    source = (PACKAGE_ROOT / 'src' / 'go2_sdk_gait_matrix.cpp').read_text(
        encoding='utf-8')
    assert 'bool execute{false};' in source
    assert '--execute-off-ground' in source
    assert 'kMaxOffGroundFootForce' in source
    assert 'off_ground_guard_failed' in source


def test_matrix_has_bounded_motion_and_final_stop():
    source = (PACKAGE_ROOT / 'src' / 'go2_sdk_gait_matrix.cpp').read_text(
        encoding='utf-8')
    for required in ('kVxMps = 0.10', 'kYawRadps = 0.15',
                     'duration_sec > 1.0', 'FinalStopGuard',
                     'client_.Move(0.0F, 0.0F, 0.0F)', 'client_.StopMove()',
                     'ClassicWalk(true)', 'FreeWalk()'):
        assert required in source
    for forbidden in ('SelectMode(', 'ReleaseMode(', 'StandUp(', 'FrontJump('):
        assert forbidden not in source
