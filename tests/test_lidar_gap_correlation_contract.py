#!/usr/bin/env python3
"""关联探针只能观察传感器和网卡，不能成为新的控制路径。"""

from pathlib import Path


SOURCE = Path(__file__).resolve().parents[1] / 'tools' / 'lidar_gap_correlation_probe.py'


def test_probe_is_read_only_and_records_gap_and_interface_counters():
    """杜绝 publisher、Twist 与 SDK，保证十分钟测试不改变机器人状态。"""
    text = SOURCE.read_text(encoding='utf-8')
    assert "CLOUD_TOPIC = '/utlidar/cloud_base'" in text
    assert "LIDAR_STATE_TOPIC = '/utlidar/lidar_state'" in text
    assert 'rx_dropped' in text and "'cloud_gap'" in text
    assert 'self._flush_output()' in text
    assert 'create_publisher' not in text
    assert 'SportClient' not in text
    assert 'Twist' not in text
