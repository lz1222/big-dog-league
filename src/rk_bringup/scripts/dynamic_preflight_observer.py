#!/usr/bin/env python3
"""动态验收的原生感知 observer；只订阅、记录，绝不发布控制。"""

import argparse
import json
import os
from pathlib import Path
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from rclpy.qos import ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

from rk_interfaces.msg import LineTrack, SpecialTargetDetection
from rk_bringup.dynamic_preflight_core import (
    arrival_statistics,
    classify_stream,
    stream_passes,
)


def _node_process_alive(token):
    """只读 /proc，确认 producer 进程存在而不向 ROS 图写入任何数据。"""
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():
            continue
        try:
            command = (entry / 'cmdline').read_bytes().decode(
                'utf-8', errors='ignore'
            )
        except OSError:
            continue
        if token in command:
            return True
    return False


def _endpoint_qos(endpoint):
    """序列化实际 endpoint QoS，供 0-frame 分类复核而非假设默认值。"""
    profile = getattr(endpoint, 'qos_profile', None)
    if profile is None:
        return {'available': False}
    return {
        'available': True,
        # Foxy 的 QoS enum 在不同补丁版本中不总能直接转 int；保留原始
        # 可读值，避免 observer 自己因诊断字段抛异常而掩盖感知故障。
        'reliability': _qos_value(profile.reliability),
        'durability': _qos_value(profile.durability),
        'history': _qos_value(profile.history),
        'depth': int(profile.depth),
    }


def _qos_value(value):
    """以跨 Foxy 补丁版本稳定的形式记录 QoS 枚举。"""
    try:
        return int(value)
    except (TypeError, ValueError):
        return str(value)


def _endpoint_description(endpoint):
    return {
        'node_name': str(getattr(endpoint, 'node_name', '')),
        'node_namespace': str(getattr(endpoint, 'node_namespace', '')),
        'qos': _endpoint_qos(endpoint),
    }


class DynamicPreflightObserver(Node):
    """等待发现完成后观察三路正式感知流，不参与任何运动决策。"""

    def __init__(self, arguments):
        super().__init__('dynamic_preflight_observer')
        self.arguments = arguments
        # camera 严格复用 ROS sensor-data QoS；该 topic 的 producer 是
        # BEST_EFFORT/VOLATILE。tracker 两路为 RELIABLE/VOLATILE，单独使用
        # 对等订阅 QoS，避免诊断端把 QoS 不兼容误判为相机或 tracker 故障。
        self.camera_qos = qos_profile_sensor_data
        self.tracker_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.streams = {
            'camera': {
                'topic': '/line_camera/image_raw',
                'subscription': self.create_subscription(
                    Image, '/line_camera/image_raw', self._camera_callback,
                    self.camera_qos,
                ),
                'timestamps_ns': [],
            },
            'line_track': {
                'topic': '/perception/line_track',
                'subscription': self.create_subscription(
                    LineTrack, '/perception/line_track',
                    self._line_callback, self.tracker_qos,
                ),
                'timestamps_ns': [],
            },
            'white_bar': {
                'topic': '/perception/white_bar_detection',
                'subscription': self.create_subscription(
                    SpecialTargetDetection, '/perception/white_bar_detection',
                    self._white_callback, self.tracker_qos,
                ),
                'timestamps_ns': [],
            },
        }
        self._readiness_payload = None
        self.create_subscription(
            String, '/competition/readiness_status', self._on_readiness, 10
        )
        self._discovery_deadline = time.monotonic() + arguments.discovery_timeout_sec
        self._window_start_ns = None

    def _camera_callback(self, _message):
        self._record('camera')

    def _line_callback(self, _message):
        self._record('line_track')

    def _white_callback(self, _message):
        self._record('white_bar')

    def _record(self, stream_name):
        if self._window_start_ns is not None:
            self.streams[stream_name]['timestamps_ns'].append(time.monotonic_ns())

    def _on_readiness(self, message):
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        if isinstance(payload, dict):
            self._readiness_payload = payload

    def _matched_count(self, stream):
        try:
            return int(stream['subscription'].get_publisher_count())
        except Exception:
            return 0

    def discovery_ready(self):
        # Foxy 在部分 RMW 组合中 subscription.get_publisher_count() 会在
        # 回调开始前短暂保持 0。endpoint discovery 是此阶段唯一可靠的
        # publisher 存在证据；真正的数据匹配仍由后续完整窗口的帧统计裁决。
        return all(
            len(self.get_publishers_info_by_topic(stream['topic'])) >= 1
            for stream in self.streams.values()
        )

    def start_window_if_ready(self):
        if self._window_start_ns is not None or not self.discovery_ready():
            return False
        self._window_start_ns = time.monotonic_ns()
        for stream in self.streams.values():
            stream['timestamps_ns'] = []
        return True

    def readiness_check_ok(self, name):
        payload = self._readiness_payload
        if not isinstance(payload, dict):
            return None
        for check in payload.get('checks', []):
            if isinstance(check, dict) and check.get('name') == name:
                return bool(check.get('ok'))
        return None

    def result(self, discovery_timed_out):
        line_camera_alive = _node_process_alive('line_camera_node')
        tracker_alive = _node_process_alive('real_line_tracker_node')
        device_present = os.path.exists(self.arguments.camera_device)
        readiness_ok = {
            'camera': self.readiness_check_ok('LINE_CAMERA_READY'),
            'line_track': self.readiness_check_ok('line_track_fresh'),
            'white_bar': self.readiness_check_ok('line_track_fresh'),
        }
        result_streams = {}
        all_pass = not discovery_timed_out
        for name, stream in self.streams.items():
            endpoints = self.get_publishers_info_by_topic(stream['topic'])
            statistics = arrival_statistics(stream['timestamps_ns'])
            matched = self._matched_count(stream)
            stream_ok = stream_passes(
                statistics,
                self.arguments.minimum_rate_hz,
                self.arguments.maximum_gap_sec,
            )
            # discovery deadline 内未得到全部匹配时，必须明确是发现阶段失败，
            # 不能把尚未开始的窗口误报为 producer 的 0 Hz。
            classification = (
                'DISCOVERY_NOT_READY'
                if discovery_timed_out and not endpoints else classify_stream(
                    name, len(endpoints), matched, statistics,
                    line_camera_alive, tracker_alive, device_present,
                    readiness_ok[name],
                )
            )
            result_streams[name] = {
                'topic': stream['topic'],
                'publisher_count': len(endpoints),
                'publishers': [_endpoint_description(item) for item in endpoints],
                'subscription_match_count': matched,
                'statistics': statistics,
                'stream_pass': stream_ok,
                'classification': classification,
            }
            all_pass = all_pass and stream_ok and classification == 'PASS'
        return {
            'success': all_pass,
            'discovery_timed_out': bool(discovery_timed_out),
            'window_start_monotonic_ns': self._window_start_ns,
            'window_duration_sec': self.arguments.duration_sec,
            'minimum_rate_hz': self.arguments.minimum_rate_hz,
            'maximum_gap_sec': self.arguments.maximum_gap_sec,
            'producer_context': {
                'line_camera_node_alive': line_camera_alive,
                'real_line_tracker_node_alive': tracker_alive,
                'camera_device': self.arguments.camera_device,
                'camera_device_present': device_present,
                'readiness_payload_available': self._readiness_payload is not None,
                'readiness_stream_checks': readiness_ok,
            },
            'streams': result_streams,
        }


def parse_arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration-sec', type=float, default=5.0)
    parser.add_argument('--discovery-timeout-sec', type=float, default=12.0)
    parser.add_argument('--minimum-rate-hz', type=float, default=10.0)
    parser.add_argument('--maximum-gap-sec', type=float, default=0.5)
    parser.add_argument(
        '--camera-device',
        default=(
            '/dev/v4l/by-id/'
            'usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera_SN0001-video-index0'
        ),
    )
    arguments = parser.parse_args(argv)
    for name in (
        'duration_sec', 'discovery_timeout_sec', 'minimum_rate_hz',
        'maximum_gap_sec',
    ):
        if getattr(arguments, name) <= 0.0:
            parser.error('--{} must be positive'.format(name.replace('_', '-')))
    return arguments


def main(argv=None):
    arguments = parse_arguments(argv)
    rclpy.init(args=[])
    node = DynamicPreflightObserver(arguments)
    discovery_timed_out = False
    try:
        while rclpy.ok() and node._window_start_ns is None:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.start_window_if_ready():
                break
            if time.monotonic() >= node._discovery_deadline:
                discovery_timed_out = True
                break
        if node._window_start_ns is not None:
            deadline_ns = node._window_start_ns + int(arguments.duration_sec * 1e9)
            while rclpy.ok() and time.monotonic_ns() < deadline_ns:
                rclpy.spin_once(node, timeout_sec=0.1)
        result = node.result(discovery_timed_out)
        print('DYNAMIC_PREFLIGHT_OBSERVER ' + json.dumps(
            result, separators=(',', ':'), allow_nan=False,
        ))
        return 0 if result['success'] else 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
