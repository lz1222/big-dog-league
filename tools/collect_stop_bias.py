#!/usr/bin/env python3
"""T6: 停车偏差数据采集工具。

记录每次停止的 odom yaw 变化，用于 StopBiasEstimator 统计。

用法 (手动模式):
  python3 tools/collect_stop_bias.py --mode manual --label "straight_slow"

  运行后:
  1. 遥控器让狗行走
  2. 按 Enter → 记录当前 yaw 作为 yaw_before_stop
  3. 遥控器让狗停
  4. 等待稳定后按 Enter → 记录 yaw_after_settle
  5. 重复 10+ 次
  6. Ctrl-C 退出并保存

输出: evidence/stop_bias/<label>_<timestamp>.json
"""

import argparse
import json
import math
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class StopBiasCollector(Node):
    """Monitor odom yaw, record before/after stop on manual trigger."""

    def __init__(self, label: str, mode: str = 'manual'):
        super().__init__('stop_bias_collector')

        self._label = label
        self._mode = mode
        self._lock = threading.Lock()
        self._yaw = 0.0
        self._yaw_ts = 0.0
        self._running = True
        self._records = []
        self._pending_before = None
        self._pending_before_ts = None

        self.create_subscription(
            Odometry, '/utlidar/robot_odom', self._on_odom, 10,
        )

        self.get_logger().info(
            f'Stop bias collector ready: mode={mode}, label={label}'
        )
        self.get_logger().info(
            '  ENTER = mark stop (before/after), Ctrl-C = save & exit'
        )

    def _on_odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self._lock:
            self._yaw = yaw
            self._yaw_ts = ts

    def mark_before(self):
        """Record yaw before stop."""
        with self._lock:
            yaw = self._yaw
            ts = self._yaw_ts
        self._pending_before = yaw
        self._pending_before_ts = ts

    def mark_after(self):
        """Record yaw after settle, complete one record."""
        if self._pending_before is None:
            self.get_logger().warn('No before-stop recorded. Press ENTER twice: once before stop, once after.')
            return

        with self._lock:
            yaw = self._yaw
            ts = self._yaw_ts

        before = self._pending_before
        shift = self._normalize(yaw - before)
        settle_time = ts - self._pending_before_ts if self._pending_before_ts else 0

        record = {
            'index': len(self._records) + 1,
            'timestamp': datetime.now().isoformat(),
            'yaw_before_rad': round(before, 6),
            'yaw_before_deg': round(math.degrees(before), 2),
            'yaw_after_rad': round(yaw, 6),
            'yaw_after_deg': round(math.degrees(yaw), 2),
            'stop_yaw_shift_rad': round(shift, 6),
            'stop_yaw_shift_deg': round(math.degrees(shift), 2),
            'settling_time_sec': round(settle_time, 3),
            'label': self._label,
        }

        self._records.append(record)
        self._pending_before = None

        direction = 'LEFT' if shift > 0 else 'RIGHT' if shift < 0 else 'NONE'
        self.get_logger().info(
            f'Record #{record["index"]:2d}: '
            f'yaw {record["yaw_before_deg"]:+.1f}deg → '
            f'{record["yaw_after_deg"]:+.1f}deg  '
            f'shift={record["stop_yaw_shift_deg"]:+.2f}deg [{direction}]  '
            f'settle={settle_time:.2f}s'
        )

    def save(self):
        """Save all records to JSON."""
        if not self._records:
            self.get_logger().info('No records to save.')
            return

        out_dir = Path.home() / 'rk_inspection_ws' / 'evidence' / 'stop_bias'
        out_dir.mkdir(parents=True, exist_ok=True)
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        path = out_dir / f'{self._label}_{ts}.json'

        # Compute summary statistics
        shifts = [r['stop_yaw_shift_rad'] for r in self._records]
        shifts_deg = [r['stop_yaw_shift_deg'] for r in self._records]
        n = len(shifts)
        median = sorted(shifts)[n // 2] if n else 0
        mean = sum(shifts) / n if n else 0
        variance = sum((s - mean)**2 for s in shifts) / n if n else 0
        std_dev = math.sqrt(variance)
        dominant = 1 if median >= 0 else -1
        consistent = sum(1 for s in shifts if (s >= 0) == (dominant >= 0))

        data = {
            'label': self._label,
            'timestamp': ts,
            'sample_count': n,
            'records': self._records,
            'summary': {
                'median_shift_deg': round(math.degrees(median), 2),
                'mean_shift_deg': round(math.degrees(mean), 2),
                'std_dev_deg': round(math.degrees(std_dev), 2),
                'direction_consistency': round(consistent / max(1, n), 2),
                'min_shift_deg': round(min(shifts_deg), 2) if shifts_deg else 0,
                'max_shift_deg': round(max(shifts_deg), 2) if shifts_deg else 0,
                'enabled': (
                    n >= 10
                    and math.degrees(std_dev) < 3.0
                    and consistent / n >= 0.7
                ),
            },
        }

        with open(path, 'w') as f:
            json.dump(data, f, indent=2)

        print(f'\nSaved: {path}')
        print(f'Samples: {n}')
        print(f'Median shift: {math.degrees(median):+.2f}deg')
        print(f'Std dev: {math.degrees(std_dev):.2f}deg')
        print(f'Direction consistent: {consistent}/{n} ({consistent/n:.0%})')
        print(f'Enabled for feedforward: {data["summary"]["enabled"]}')

        return str(path)

    @staticmethod
    def _normalize(a):
        while a > math.pi: a -= 2*math.pi
        while a < -math.pi: a += 2*math.pi
        return a


def input_thread(collector):
    """Read keyboard in background thread."""
    while collector._running:
        try:
            input()
            if not collector._running:
                break
            if collector._pending_before is None:
                collector.mark_before()
                print('\n>>> BEFORE recorded. Now STOP the robot.')
                print('>>> Wait for settle, then press ENTER again.\n')
            else:
                collector.mark_after()
                print(f'\n>>> Record complete. '
                      f'{len(collector._records)} samples collected.')
                print('>>> Walk again, then press ENTER before stop...\n')
        except (EOFError, KeyboardInterrupt):
            collector._running = False
            break


def main():
    parser = argparse.ArgumentParser(description='T6 stop bias data collector')
    parser.add_argument('--mode', default='manual', choices=['manual'],
                        help='Collection mode')
    parser.add_argument('--label', '-l', required=True,
                        help='Label for this session (e.g. straight_slow)')
    args = parser.parse_args()

    rclpy.init()
    collector = StopBiasCollector(label=args.label, mode=args.mode)

    thread = threading.Thread(target=input_thread, args=(collector,), daemon=True)
    thread.start()

    try:
        rclpy.spin(collector)
    except KeyboardInterrupt:
        pass
    finally:
        collector._running = False
        collector.save()
        collector.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
