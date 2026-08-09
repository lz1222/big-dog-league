#!/usr/bin/env python3

"""只读 ROS Topic observer，避免 Foxy ``ros2 topic echo`` 的发现竞态。

String 主题逐行输出 ``msg.data``；Twist 主题输出稳定的 YAML 片段，供
Software Smoke 检查最终速度。两种模式都只建立订阅，不创建会影响机器人的
Publisher、Service 或 Action。

本脚本不创建 Publisher、Service 或 Action，不会向机器人发送任何命令。

模式：
  stream          — 持续订阅，msg.data 写入 stdout（每行一条）
  --once --match  — 等待匹配的 key=value，成功后退出 0
  --once --value  — 输出指定键的值后退出 0
"""

import argparse
import json
import math
import sys
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import String
from rk_interfaces.msg import LineTrack, SpecialTargetDetection

_OBSERVER_PREAMBLE = '__NON_ARM_SMOKE_OBSERVER_READY__'


def _unique_suffix():
    """返回仅含字母数字和下划线的唯一后缀，Foxy 节点名不允许 '.'。"""
    return str(int(time.monotonic() * 1e6))


class OnceMatchObserver(Node):
    """等待 key=expected 的 JSON payload，单次匹配即退出。"""

    def __init__(self, topic_name, key, expected, timeout_sec):
        super().__init__('smoke_match_' + _unique_suffix())
        self._found = False
        self._key = key
        self._expected = expected
        self._deadline = time.monotonic() + timeout_sec
        self.create_subscription(String, topic_name, self._on_string, 10)

    def _on_string(self, msg):
        if self._found:
            return
        data = str(msg.data).strip()
        if not data:
            return
        try:
            payload = json.loads(data)
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        if not isinstance(payload, dict):
            return
        value = payload.get(self._key)
        if self._expected == '__true__' and value is True:
            self._found = True
        elif self._expected == '__false__' and value is False:
            self._found = True
        elif self._expected not in ('__true__', '__false__') \
                and str(value) == self._expected:
            self._found = True

    def timed_out(self):
        return time.monotonic() >= self._deadline


class OnceValueObserver(Node):
    """等待一条消息，输出指定键的值后退出。"""

    def __init__(self, topic_name, key, timeout_sec):
        super().__init__('smoke_value_' + _unique_suffix())
        self._value = None
        self._key = key
        self._deadline = time.monotonic() + timeout_sec
        self.create_subscription(String, topic_name, self._on_string, 10)

    def _on_string(self, msg):
        if self._value is not None:
            return
        data = str(msg.data).strip()
        if not data:
            return
        try:
            payload = json.loads(data)
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        if not isinstance(payload, dict) or self._key not in payload:
            return
        self._value = payload[self._key]

    def timed_out(self):
        return time.monotonic() >= self._deadline


class DumpOnceObserver(Node):
    """等待一条 String 消息，输出完整 msg.data 后退出。"""

    def __init__(self, topic_name, timeout_sec):
        super().__init__('smoke_dump_' + _unique_suffix())
        self._payload = None
        self._deadline = time.monotonic() + timeout_sec
        self.create_subscription(String, topic_name, self._on_string, 10)

    def _on_string(self, msg):
        if self._payload is not None:
            return
        data = str(msg.data).strip()
        if not data:
            return
        self._payload = data

    def timed_out(self):
        return time.monotonic() >= self._deadline


class StreamStringObserver(Node):
    """持续订阅 String Topic，msg.data 每行写入 stdout。"""

    def __init__(self, topic_name):
        super().__init__('smoke_str_' + _unique_suffix())
        self._first = True
        self.create_subscription(String, topic_name, self._on_string, 10)

    def _on_string(self, msg):
        line = str(msg.data).strip()
        if not line:
            return
        if self._first:
            self._first = False
            self._emit(_OBSERVER_PREAMBLE)
        self._emit(line)

    @staticmethod
    def _emit(line):
        sys.stdout.write(line + '\n')
        sys.stdout.flush()


class StreamTwistObserver(Node):
    """持续订阅 Twist，并以现有 shell 检查兼容的 YAML 写入 stdout。"""

    def __init__(self, topic_name):
        super().__init__('smoke_twist_' + _unique_suffix())
        self.create_subscription(Twist, topic_name, self._on_twist, 10)

    @staticmethod
    def _on_twist(msg):
        # 字段顺序与 ros2 topic echo 保持一致，避免改变既有零速判定语义。
        sys.stdout.write(
            'linear:\n'
            '  x: {}\n  y: {}\n  z: {}\n'
            'angular:\n'
            '  x: {}\n  y: {}\n  z: {}\n---\n'.format(
                msg.linear.x, msg.linear.y, msg.linear.z,
                msg.angular.x, msg.angular.y, msg.angular.z,
            )
        )
        sys.stdout.flush()


def twist_is_zero(msg):
    """只接受六个有限且严格为零的 Twist 字段。"""
    values = (
        msg.linear.x, msg.linear.y, msg.linear.z,
        msg.angular.x, msg.angular.y, msg.angular.z,
    )
    return all(math.isfinite(value) and value == 0.0 for value in values)


class ConsecutiveZeroTwistObserver(StreamTwistObserver):
    """用单一长期订阅确认连续全零，避免反复 CLI 发现竞态。"""

    def __init__(self, topic_name, required_count, timeout_sec):
        self._zero_count = 0
        self._required_count = required_count
        self._deadline = time.monotonic() + timeout_sec
        self._done = False
        super().__init__(topic_name)

    def _on_twist(self, msg):
        super()._on_twist(msg)
        if twist_is_zero(msg):
            self._zero_count += 1
        else:
            self._zero_count = 0
        self._done = self._zero_count >= self._required_count

    def timed_out(self):
        return time.monotonic() >= self._deadline


def compute_rate_result(timestamps, minimum_rate_hz, required_span_sec):
    """用首末样本计算平均频率，并要求足够时间跨度排除短突发。"""
    gaps = [
        current - previous
        for previous, current in zip(timestamps, timestamps[1:])
    ]
    span_sec = timestamps[-1] - timestamps[0] if gaps else 0.0
    rate_hz = (len(timestamps) - 1) / span_sec if span_sec > 0.0 else 0.0
    max_gap_sec = max(gaps) if gaps else None
    return {
        'success': (
            len(timestamps) >= 2
            and span_sec >= required_span_sec
            and rate_hz >= minimum_rate_hz
        ),
        'samples': len(timestamps),
        'span_sec': span_sec,
        'rate_hz': rate_hz,
        'max_gap_sec': max_gap_sec,
        'minimum_rate_hz': minimum_rate_hz,
        'required_span_sec': required_span_sec,
    }


class RateObserver(Node):
    """在完整有界窗口内统计指定正式感知 Topic 的实际到达频率。"""

    MESSAGE_TYPES = {
        'image': Image,
        'line_track': LineTrack,
        'special_target': SpecialTargetDetection,
    }

    def __init__(self, topic_name, rate_type):
        super().__init__('smoke_rate_' + _unique_suffix())
        self._timestamps = []
        qos = qos_profile_sensor_data if rate_type == 'image' else 10
        self.subscription = self.create_subscription(
            self.MESSAGE_TYPES[rate_type], topic_name, self._on_message, qos
        )

    def _on_message(self, _msg):
        self._timestamps.append(time.monotonic())


def main():
    parser = argparse.ArgumentParser(
        description='Foxy-compatible read-only String topic observer'
    )
    parser.add_argument('topic_name', help='String ROS topic to observe')
    parser.add_argument('--timeout-sec', type=float, default=180.0)
    parser.add_argument('--once', action='store_true')
    parser.add_argument('--dump', action='store_true',
                        help='Wait for one message, output full msg.data, exit 0')
    parser.add_argument('--match-key')
    parser.add_argument('--match-value', default='')
    parser.add_argument('--value-key')
    parser.add_argument(
        '--twist', action='store_true',
        help='Observe geometry_msgs/Twist and emit compatible YAML samples',
    )
    parser.add_argument(
        '--consecutive-zero-count', type=int, default=0,
        help='With --twist, exit 0 only after this many consecutive zero samples',
    )
    parser.add_argument(
        '--rate-type', choices=tuple(RateObserver.MESSAGE_TYPES),
        help='Measure arrival rate for a supported formal perception type',
    )
    parser.add_argument('--minimum-rate-hz', type=float, default=0.0)
    parser.add_argument('--required-span-sec', type=float, default=2.0)
    args = parser.parse_args()

    if args.once and not args.dump and not args.match_key and not args.value_key:
        sys.stderr.write(
            '--once requires --dump, --match-key or --value-key\n'
        )
        sys.exit(2)
    if args.twist and (args.once or args.dump or args.match_key or args.value_key):
        sys.stderr.write('--twist only supports stream mode\n')
        sys.exit(2)
    if args.consecutive_zero_count < 0:
        sys.stderr.write('--consecutive-zero-count must be nonnegative\n')
        sys.exit(2)
    if args.consecutive_zero_count and not args.twist:
        sys.stderr.write('--consecutive-zero-count requires --twist\n')
        sys.exit(2)
    if args.rate_type and (args.twist or args.once):
        sys.stderr.write('--rate-type cannot be combined with --twist/--once\n')
        sys.exit(2)
    if args.rate_type and (
        not math.isfinite(args.minimum_rate_hz)
        or args.minimum_rate_hz <= 0.0
        or not math.isfinite(args.required_span_sec)
        or args.required_span_sec <= 0.0
        or args.required_span_sec >= args.timeout_sec
    ):
        sys.stderr.write('rate thresholds must be positive and span < timeout\n')
        sys.exit(2)

    rclpy.init(args=[])
    node = None
    try:
        if args.rate_type:
            node = RateObserver(args.topic_name, args.rate_type)
        elif args.once and args.dump:
            node = DumpOnceObserver(
                args.topic_name, args.timeout_sec,
            )
        elif args.once and args.match_key:
            node = OnceMatchObserver(
                args.topic_name, args.match_key, args.match_value,
                args.timeout_sec,
            )
        elif args.once and args.value_key:
            node = OnceValueObserver(
                args.topic_name, args.value_key, args.timeout_sec,
            )
        elif args.twist and args.consecutive_zero_count:
            node = ConsecutiveZeroTwistObserver(
                args.topic_name, args.consecutive_zero_count, args.timeout_sec,
            )
        elif args.twist:
            node = StreamTwistObserver(args.topic_name)
        else:
            node = StreamStringObserver(args.topic_name)

        deadline = time.monotonic() + args.timeout_sec

        if isinstance(node, DumpOnceObserver):
            while rclpy.ok() and node._payload is None \
                    and not node.timed_out():
                rclpy.spin_once(node, timeout_sec=0.1)
            if node._payload is not None:
                sys.stdout.write(node._payload + '\n')
                sys.exit(0)
            sys.exit(1)

        if isinstance(node, OnceMatchObserver):
            while rclpy.ok() and not node._found and not node.timed_out():
                rclpy.spin_once(node, timeout_sec=0.1)
            sys.exit(0 if node._found else 1)

        if isinstance(node, OnceValueObserver):
            while rclpy.ok() and node._value is None \
                    and not node.timed_out():
                rclpy.spin_once(node, timeout_sec=0.1)
            if node._value is not None:
                sys.stdout.write(str(node._value) + '\n')
                sys.exit(0)
            sys.exit(1)

        if isinstance(node, ConsecutiveZeroTwistObserver):
            while rclpy.ok() and not node._done and not node.timed_out():
                rclpy.spin_once(node, timeout_sec=0.1)
            sys.exit(0 if node._done else 1)

        if isinstance(node, RateObserver):
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
            result = compute_rate_result(
                node._timestamps,
                args.minimum_rate_hz,
                args.required_span_sec,
            )
            sys.stdout.write(json.dumps(
                result, separators=(',', ':'), allow_nan=False
            ) + '\n')
            sys.exit(0 if result['success'] else 1)

        # Stream mode
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)

    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
