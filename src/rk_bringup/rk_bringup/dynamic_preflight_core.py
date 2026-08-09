"""动态验收 preflight 的纯统计与分类规则。

本模块不导入 ROS，也不创建控制端点；把到达时间统计和故障分类放在这里，
让真机 observer 与离线回归使用同一 fail-closed 语义。
"""

import math


def arrival_statistics(timestamps_ns):
    """汇总单个 topic 的到达时间；少于两帧时频率明确为零。"""
    values = [int(value) for value in timestamps_ns]
    if not values:
        return {
            'sample_count': 0,
            'first_frame_monotonic_ns': None,
            'last_frame_monotonic_ns': None,
            'span_sec': 0.0,
            'rate_hz': 0.0,
            'max_gap_sec': None,
        }
    gaps_ns = [
        current - previous
        for previous, current in zip(values, values[1:])
    ]
    span_ns = values[-1] - values[0]
    span_sec = max(0.0, float(span_ns) / 1e9)
    rate_hz = (
        float(len(values) - 1) / span_sec
        if len(values) >= 2 and span_sec > 0.0 else 0.0
    )
    max_gap_sec = (
        max(gaps_ns) / 1e9 if gaps_ns else None
    )
    return {
        'sample_count': len(values),
        'first_frame_monotonic_ns': values[0],
        'last_frame_monotonic_ns': values[-1],
        'span_sec': span_sec,
        'rate_hz': rate_hz,
        'max_gap_sec': max_gap_sec,
    }


def stream_passes(statistics, minimum_rate_hz, maximum_gap_sec):
    """要求完整窗口有帧、平均频率达标且没有可见中断。"""
    max_gap = statistics['max_gap_sec']
    return (
        statistics['sample_count'] >= 2
        and math.isfinite(statistics['rate_hz'])
        and statistics['rate_hz'] >= float(minimum_rate_hz)
        and max_gap is not None
        and math.isfinite(max_gap)
        and max_gap <= float(maximum_gap_sec)
    )


def classify_stream(
    stream_name,
    publisher_count,
    matched_publisher_count,
    statistics,
    line_camera_alive,
    tracker_alive,
    camera_device_present,
    readiness_stream_ok,
):
    """把零帧现象归入可操作类别，未知时禁止假装为 QoS 问题。"""
    # 收到真实帧是 DDS 匹配的最强证据。Foxy 某些 RMW 中 publisher-count
    # 可能晚于首帧更新，不能让这个诊断计数否定已观测到的数据流。
    if statistics['sample_count'] > 0:
        return 'PASS'
    if matched_publisher_count < 1:
        if publisher_count > 0:
            return 'OBSERVER_QOS_MISMATCH'
        if stream_name == 'camera' and not line_camera_alive:
            return 'PRODUCER_DIED'
        if stream_name in ('line_track', 'white_bar') and not tracker_alive:
            return 'TRACKER_FAILURE'
        return 'DISCOVERY_NOT_READY'
    if stream_name == 'camera':
        if not line_camera_alive:
            return 'PRODUCER_DIED'
        if not camera_device_present or readiness_stream_ok is False:
            return 'CAMERA_IO_FAILURE'
        return 'UNKNOWN'
    if not tracker_alive:
        return 'TRACKER_FAILURE'
    if readiness_stream_ok is False:
        return 'TRACKER_FAILURE'
    return 'UNKNOWN'
