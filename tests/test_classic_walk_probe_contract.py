"""经典步态直行、左半圆弧验证工具的静态安全契约。"""

from pathlib import Path


SOURCE = (Path(__file__).resolve().parents[1] / 'src' / 'rk_go2_sdk_bridge' /
          'src' / 'go2_sdk_classic_walk_probe.cpp')


def test_classic_walk_probe_preserves_an_existing_classic_walk_state():
    """默认不得触碰 SDK；已开启经典步态时只停车，不得擅自关闭操作者状态。"""
    text = SOURCE.read_text(encoding='utf-8')

    # 直行速度允许操作者在源码中细调，不把数值写死在测试中覆盖人工设置。
    assert 'constexpr double kClassicWalkVxMps' in text
    assert 'constexpr double kLeftArcVxMps' in text
    assert 'constexpr double kLeftArcWzRadps' in text
    assert 'constexpr double kLeftArcDurationSec' in text
    assert 'constexpr double kPostLeftArcStraightDurationSec = 2.1;' in text
    # 右弧必须镜像左弧，避免人工调整左转后两侧曲率不一致。
    assert 'constexpr double kRightArcVxMps = kLeftArcVxMps;' in text
    assert 'constexpr double kRightArcWzRadps = -kLeftArcWzRadps;' in text
    assert 'constexpr double kRightArcDurationSec = kLeftArcDurationSec;' in text
    assert 'constexpr double kPostRightArcStraightDurationSec = 2.0;' in text
    assert 'constexpr double kFinalLeftArcVxMps = kLeftArcVxMps;' in text
    assert 'constexpr double kFinalLeftArcWzRadps = kLeftArcWzRadps;' in text
    assert 'constexpr double kFinalLeftArcDurationSec = 1.89;' in text
    assert 'kObservedClassicWalkStateCode = 2010' in text
    assert 'SportStateSnapshot state_monitor' in text
    assert 'classic_walk_already_active' in text
    assert 'if (!config.execute)' in text
    assert 'ClassicWalk(true)' in text
    assert 'client.Move(' in text
    assert 'client.StopMove()' in text
    assert 'client.ClassicWalk(false)' in text
    assert 'classic_walk_off=skipped_preserve_operator_state' in text
    assert 'option == "--execute"' in text
    assert 'option == "--left-arc-start-sec"' in text
    assert 'straight_before_left_arc' in text
    assert 'left_half_arc' in text
    assert 'straight_between_arcs' in text
    assert 'right_half_arc' in text
    assert 'straight_after_right_arc' in text
    assert 'final_left_arc' in text
