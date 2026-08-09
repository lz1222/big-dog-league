"""原生只读 Topic observer 的纯函数安全回归。"""

import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace


OBSERVER_PATH = (
    Path(__file__).resolve().parents[1]
    / 'scripts' / 'non_arm_smoke_observer.py'
)
SPEC = importlib.util.spec_from_file_location(
    'non_arm_smoke_observer_under_test', OBSERVER_PATH
)
OBSERVER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(OBSERVER)


def _twist(values):
    """按 geometry_msgs/Twist 字段形状构造无 ROS 的测试对象。"""
    return SimpleNamespace(
        linear=SimpleNamespace(x=values[0], y=values[1], z=values[2]),
        angular=SimpleNamespace(x=values[3], y=values[4], z=values[5]),
    )


def test_twist_is_zero_requires_all_six_exact_finite_zeroes():
    assert OBSERVER.twist_is_zero(_twist((0.0,) * 6))
    assert not OBSERVER.twist_is_zero(
        _twist((0.0, 0.0, 0.0, 0.0, 0.0, 1e-12))
    )
    assert not OBSERVER.twist_is_zero(
        _twist((0.0, 0.0, math.nan, 0.0, 0.0, 0.0))
    )


def test_rate_result_requires_minimum_rate_and_continuous_span():
    timestamps = [index * 0.05 for index in range(61)]
    result = OBSERVER.compute_rate_result(timestamps, 10.0, 2.0)
    assert result['success'] is True
    assert result['samples'] == 61
    assert result['rate_hz'] == 20.0
    assert result['max_gap_sec'] <= 0.051

    assert not OBSERVER.compute_rate_result(
        timestamps[:10], 10.0, 2.0
    )['success']
    assert not OBSERVER.compute_rate_result(
        [0.0, 0.2, 0.4, 2.5], 10.0, 2.0
    )['success']
