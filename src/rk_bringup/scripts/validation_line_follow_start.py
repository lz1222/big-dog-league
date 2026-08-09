#!/usr/bin/env python3
"""仅为动态验收激活 follower 的一次性私有 start publisher。

该工具从不订阅或发布 ``/mission/start``，不发布 ``/control/line_cmd`` 或
``/navigation/cmd_vel``。它在确认唯一订阅者是 line_follower_node 后只发送
一次 Bool(true)，并以原生订阅证明候选存在而最终速度仍为零。
"""

import argparse
import json
import math
import sys
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from rclpy.qos import ReliabilityPolicy
from std_msgs.msg import Bool, String


def _finite_nonzero_forward(message):
    return (
        math.isfinite(message.linear.x)
        and math.isfinite(message.linear.y)
        and math.isfinite(message.angular.z)
        and message.linear.x > 0.0
        and message.linear.y == 0.0
    )


def _zero_twist(message):
    return all(
        math.isfinite(value) and value == 0.0
        for value in (
            message.linear.x, message.linear.y, message.linear.z,
            message.angular.x, message.angular.y, message.angular.z,
        )
    )


class ValidationFollowerStart(Node):
    """验证私有 start 只影响 follower，并持续记录 final cmd 的零约束。"""

    def __init__(self, arguments):
        super().__init__('validation_line_follow_start')
        self.arguments = arguments
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.publisher = self.create_publisher(Bool, arguments.topic, qos)
        self.states = []
        self.suggested = []
        self.final_samples = 0
        self.final_nonzero_samples = 0
        self.sdk_move_count = 0
        self.sdk_error_count = 0
        self.create_subscription(
            String, '/navigation/line_follow_status', self._on_status, qos
        )
        self.create_subscription(
            Twist, '/navigation/line_follow_cmd_suggested',
            self._on_suggested, qos,
        )
        self.create_subscription(
            Twist, '/navigation/cmd_vel', self._on_final, qos,
        )
        status_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            String, '/go2/sdk_motion_status', self._on_sdk_status, status_qos
        )

    def _on_status(self, message):
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        state = str(payload.get('nav_state', '')).strip()
        if state:
            self.states.append(state)

    def _on_suggested(self, message):
        if _finite_nonzero_forward(message):
            self.suggested.append({
                'vx': float(message.linear.x),
                'vy': float(message.linear.y),
                'yaw': float(message.angular.z),
                'monotonic_ns': time.monotonic_ns(),
            })

    def _on_final(self, message):
        self.final_samples += 1
        if not _zero_twist(message):
            self.final_nonzero_samples += 1

    def _on_sdk_status(self, message):
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        if payload.get('event') == 'MOVE':
            self.sdk_move_count += 1
        elif payload.get('event') == 'SDK_ERROR':
            self.sdk_error_count += 1

    def wait_for_follower_only(self, deadline):
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                subscriber_count = int(self.publisher.get_subscription_count())
            except Exception:
                subscriber_count = 0
            if subscriber_count == 1 and 'WAIT_START' in self.states:
                return True, subscriber_count
        try:
            subscriber_count = int(self.publisher.get_subscription_count())
        except Exception:
            subscriber_count = 0
        return False, subscriber_count

    def observe_activation(self, deadline):
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        return {
            'wait_start_seen': 'WAIT_START' in self.states,
            'start_ready_seen': 'START_READY' in self.states,
            'line_follow_seen': 'LINE_FOLLOW' in self.states,
            'suggested_candidates': self.suggested,
            'final_samples': self.final_samples,
            'final_nonzero_samples': self.final_nonzero_samples,
            'sdk_move_count': self.sdk_move_count,
            'sdk_error_count': self.sdk_error_count,
        }


def parse_arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--topic', default='/validation/line_follow/start')
    parser.add_argument('--discovery-timeout-sec', type=float, default=10.0)
    parser.add_argument('--observe-sec', type=float, default=5.0)
    arguments = parser.parse_args(argv)
    if not arguments.topic.startswith('/validation/'):
        parser.error('--topic must remain under /validation/')
    if arguments.discovery_timeout_sec <= 0.0 or arguments.observe_sec <= 0.0:
        parser.error('timeouts must be positive')
    return arguments


def main(argv=None):
    arguments = parse_arguments(argv)
    rclpy.init(args=[])
    node = ValidationFollowerStart(arguments)
    try:
        ready, subscribers = node.wait_for_follower_only(
            time.monotonic() + arguments.discovery_timeout_sec
        )
        if ready:
            # 只发送一次，不能以重试 publish 掩盖 topic 映射或 DDS 发现错误。
            message = Bool()
            message.data = True
            node.publisher.publish(message)
        evidence = node.observe_activation(
            time.monotonic() + arguments.observe_sec
        )
        evidence['validation_topic'] = arguments.topic
        evidence['subscriber_count_before_publish'] = subscribers
        evidence['publish_count'] = 1 if ready else 0
        success = (
            ready
            and evidence['start_ready_seen']
            and evidence['line_follow_seen']
            and bool(evidence['suggested_candidates'])
            and evidence['final_samples'] > 0
            and evidence['final_nonzero_samples'] == 0
            and evidence['sdk_move_count'] == 0
            and evidence['sdk_error_count'] == 0
        )
        evidence['success'] = success
        print('VALIDATION_FOLLOWER_START ' + json.dumps(
            evidence, separators=(',', ':'), allow_nan=False,
        ))
        return 0 if success else 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
