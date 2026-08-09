"""动态 preflight 统计与 fail-closed 分类的纯单元测试。"""

import pytest

from rk_bringup.dynamic_preflight_core import (
    arrival_statistics,
    classify_stream,
    stream_passes,
)


def test_arrival_statistics_requires_real_span_for_rate():
    statistics = arrival_statistics([1_000_000_000, 1_100_000_000, 1_200_000_000])

    assert statistics['sample_count'] == 3
    assert statistics['rate_hz'] == pytest.approx(10.0)
    assert statistics['max_gap_sec'] == pytest.approx(0.1)
    assert stream_passes(statistics, 10.0, 0.5) is True


def test_discovery_failure_is_not_reported_as_zero_hz():
    statistics = arrival_statistics([])

    assert classify_stream(
        'camera', 0, 0, statistics, True, True, True, None,
    ) == 'DISCOVERY_NOT_READY'


def test_existing_endpoint_without_match_is_qos_mismatch():
    assert classify_stream(
        'line_track', 1, 0, arrival_statistics([]), True, True, True, None,
    ) == 'OBSERVER_QOS_MISMATCH'


def test_received_frame_overrides_delayed_foxy_match_counter():
    """真实数据比 Foxy 延迟更新的 publisher-count 更有证明力。"""
    assert classify_stream(
        'line_track', 1, 0, arrival_statistics([1, 2]), True, True, True, None,
    ) == 'PASS'


def test_matched_zero_camera_frames_require_producer_evidence():
    assert classify_stream(
        'camera', 1, 1, arrival_statistics([]), True, True, False, False,
    ) == 'CAMERA_IO_FAILURE'
    assert classify_stream(
        'white_bar', 1, 1, arrival_statistics([]), True, False, True, True,
    ) == 'TRACKER_FAILURE'
