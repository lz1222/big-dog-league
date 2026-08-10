#!/usr/bin/env python3
"""只读关联 Go2 点云断流与网卡计数器，绝不创建任何运动输出。

`/utlidar/lidar_state` 的 DDS 类型在本机可发现，但缺少 Python message package 时
不能安全反序列化。本工具仍记录其发现状态，并以 cloud 接收 gap 与 eth0/eth1
的 RX/error/drop 计数作为可复现关联证据；不会为了读取状态消息临时编译或改 DDS。
"""

import argparse
import csv
import math
import time
from pathlib import Path
from threading import Lock
from typing import Dict, Iterable

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2


CLOUD_TOPIC = '/utlidar/cloud_base'
LIDAR_STATE_TOPIC = '/utlidar/lidar_state'
COUNTER_NAMES = ('rx_bytes', 'rx_packets', 'rx_errors', 'rx_dropped',
                 'tx_bytes', 'tx_packets', 'tx_errors', 'tx_dropped')


def _read_interface_counters(interface: str) -> Dict[str, int]:
    """读取内核单调递增计数；接口不存在或不可读时用 -1 保留证据。"""
    base = Path('/sys/class/net') / interface / 'statistics'
    values = {}
    for name in COUNTER_NAMES:
        try:
            values[name] = int((base / name).read_text(encoding='ascii').strip())
        except (OSError, ValueError):
            values[name] = -1
    return values


class LidarGapCorrelationProbe(Node):
    """只订阅 cloud；回调只记时，不解码点云，避免探针自行制造断流。"""

    def __init__(self, writer: csv.DictWriter, flush_output, interfaces: Iterable[str], gap_threshold: float):
        super().__init__('lidar_gap_correlation_probe')
        self._writer = writer
        self._flush_output = flush_output
        self._interfaces = tuple(interfaces)
        self._gap_threshold = gap_threshold
        self._lock = Lock()
        self._last_cloud_monotonic = 0.0
        self._cloud_count = 0
        self._max_gap = 0.0
        self._gap_events = 0
        self._last_frame_id = ''
        self._started = time.monotonic()
        self._last_sample = 0.0
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )
        self.create_subscription(PointCloud2, CLOUD_TOPIC, self._on_cloud, sensor_qos)
        # 定时器只采样本机内核计数器，不创建 publisher，也不影响机器人控制面。
        self.create_timer(1.0, self._sample_counters)

    def _row(self, event: str, now: float, **extra):
        row = {
            'event': event,
            'elapsed_sec': '{:.6f}'.format(now - self._started),
            'unix_sec': '{:.6f}'.format(time.time()),
            'cloud_count': self._cloud_count,
            'cloud_gap_sec': '',
            'cloud_frame_id': self._last_frame_id,
        }
        for interface in self._interfaces:
            for name, value in _read_interface_counters(interface).items():
                row['{}_{}'.format(interface, name)] = value
        row.update(extra)
        self._writer.writerow(row)
        # 现场断流可能伴随进程异常；逐行落盘才能保留故障前后关联证据。
        self._flush_output()

    def _on_cloud(self, message: PointCloud2):
        now = time.monotonic()
        with self._lock:
            gap = now - self._last_cloud_monotonic if self._last_cloud_monotonic else 0.0
            self._last_cloud_monotonic = now
            self._cloud_count += 1
            self._last_frame_id = message.header.frame_id
            self._max_gap = max(self._max_gap, gap)
            if gap >= self._gap_threshold:
                self._gap_events += 1
                # gap 行与相邻的 1 Hz 网卡样本共享时间轴，可离线比较计数突变。
                self._row('cloud_gap', now, cloud_gap_sec='{:.6f}'.format(gap))
                print('GAP elapsed={:.1f}s gap={:.3f}s cloud_count={}'.format(
                    now - self._started, gap, self._cloud_count), flush=True)

    def _sample_counters(self):
        now = time.monotonic()
        with self._lock:
            self._last_sample = now
            self._row('counter_sample', now)

    def summary(self) -> Dict[str, object]:
        """返回最终统计，供主线程在销毁节点前打印为单行验收结论。"""
        with self._lock:
            return {
                'cloud_count': self._cloud_count,
                'max_gap_sec': self._max_gap,
                'gap_events': self._gap_events,
                'frame_id': self._last_frame_id or '<none>',
            }


def main() -> int:
    parser = argparse.ArgumentParser(description='Go2 LiDAR cloud-gap read-only correlation probe')
    parser.add_argument('--duration', type=float, default=600.0)
    parser.add_argument('--gap-threshold', type=float, default=1.0)
    parser.add_argument('--interfaces', nargs='+', default=['eth0', 'eth1'])
    parser.add_argument('--output-dir', default='logs')
    args = parser.parse_args()
    if args.duration <= 0.0 or not math.isfinite(args.duration):
        parser.error('--duration 必须为正有限数')
    if args.gap_threshold <= 0.0 or not math.isfinite(args.gap_threshold):
        parser.error('--gap-threshold 必须为正有限数')

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    output = output_dir / 'lidar_gap_correlation_{}.csv'.format(
        time.strftime('%Y%m%d_%H%M%S'))
    fields = ['event', 'elapsed_sec', 'unix_sec', 'cloud_count', 'cloud_gap_sec', 'cloud_frame_id']
    for interface in args.interfaces:
        fields.extend('{}_{}'.format(interface, name) for name in COUNTER_NAMES)

    print('READ_ONLY_LIDAR_GAP_PROBE duration={}s threshold={}s interfaces={} output={}'.format(
        args.duration, args.gap_threshold, ','.join(args.interfaces), output), flush=True)
    print('LIDAR_STATE_DISCOVERY_ONLY topic={} reason=unitree_go_python_message_unavailable'.format(
        LIDAR_STATE_TOPIC), flush=True)
    rclpy.init()
    with output.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        stream.flush()
        node = LidarGapCorrelationProbe(writer, stream.flush, args.interfaces, args.gap_threshold)
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        try:
            deadline = time.monotonic() + args.duration
            while time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.5)
        finally:
            summary = node.summary()
            print('RESULT cloud_count={} max_gap_sec={:.3f} gap_events={} frame_id={}'.format(
                summary['cloud_count'], summary['max_gap_sec'], summary['gap_events'],
                summary['frame_id']), flush=True)
            # 先从执行器解绑再销毁节点，避免 Foxy 在解释器退出时访问失效句柄。
            executor.shutdown()
            executor.remove_node(node)
            node.destroy_node()
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
