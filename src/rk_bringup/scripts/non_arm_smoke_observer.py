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
import sys
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

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
    args = parser.parse_args()

    if args.once and not args.dump and not args.match_key and not args.value_key:
        sys.stderr.write(
            '--once requires --dump, --match-key or --value-key\n'
        )
        sys.exit(2)
    if args.twist and (args.once or args.dump or args.match_key or args.value_key):
        sys.stderr.write('--twist only supports stream mode\n')
        sys.exit(2)

    rclpy.init(args=[])
    node = None
    try:
        if args.once and args.dump:
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
