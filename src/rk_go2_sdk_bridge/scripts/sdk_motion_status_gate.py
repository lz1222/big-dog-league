#!/usr/bin/env python3
"""正式启动使用的只读 SDK status/receiver ROS 门禁。"""

import argparse
import json
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from rclpy.qos import ReliabilityPolicy
from std_msgs.msg import String

from sdk_motion_status import decode_status_datagram


STATUS_TOPIC = '/go2/sdk_motion_status'
READY_TOPIC = '/go2/sdk_motion_status_receiver_ready'


def decode_receiver_ready(payload, expected_instance_id, status_ip, status_port):
    """验证 B0 listener 身份和 bind 目标，拒绝其他轮次的 latched 消息。"""
    try:
        value = json.loads(payload)
    except (TypeError, ValueError):
        return None
    if not isinstance(value, dict):
        return None
    if (
        value.get('ready') is not True
        or value.get('expected_server_instance_id') != expected_instance_id
        or value.get('status_ip') != status_ip
        or value.get('status_port') != status_port
        or not isinstance(value.get('receiver_instance_id'), str)
        or not value['receiver_instance_id']
        or not isinstance(value.get('ready_monotonic_ns'), int)
        or isinstance(value.get('ready_monotonic_ns'), bool)
        or value['ready_monotonic_ns'] <= 0
    ):
        return None
    return value


class StatusGateNode(Node):
    """订阅 transient-local 状态；只观察，不创建任何控制 publisher。"""

    def __init__(self, arguments):
        super().__init__('sdk_motion_status_gate')
        self.arguments = arguments
        self.result = None
        self.failure = None
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(String, READY_TOPIC, self._on_ready, qos)
        self.create_subscription(String, STATUS_TOPIC, self._on_status, qos)

    def _on_ready(self, message):
        if self.arguments.mode != 'receiver' or self.result is not None:
            return
        value = decode_receiver_ready(
            message.data,
            self.arguments.expected_server_instance_id,
            self.arguments.status_ip,
            self.arguments.status_port,
        )
        if value is not None:
            self.result = value

    def _on_status(self, message):
        if self.arguments.mode == 'receiver' or self.result is not None:
            return
        status = decode_status_datagram(message.data)
        if status is None:
            return
        if status['server_instance_id'] != self.arguments.expected_server_instance_id:
            return
        if self.arguments.reject_move and status['event'] == 'MOVE':
            self.failure = {
                'classification': 'MOVE_EVENT_OBSERVED',
                'status': status,
            }
            return
        if (
            status['event'] == self.arguments.event
            and status['ret'] == self.arguments.required_ret
            and status['sequence'] > self.arguments.min_sequence
            and status['receive_monotonic_ns']
            >= self.arguments.min_receive_monotonic_ns
        ):
            self.result = status


def parse_arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=('receiver', 'status'), required=True)
    parser.add_argument('--expected-server-instance-id', required=True)
    parser.add_argument('--status-ip', default='127.0.0.1')
    parser.add_argument('--status-port', type=int, default=15002)
    parser.add_argument('--event', default='STARTUP_STOP')
    parser.add_argument('--required-ret', type=int, default=0)
    parser.add_argument('--min-sequence', type=int, default=0)
    parser.add_argument('--min-receive-monotonic-ns', type=int, default=0)
    parser.add_argument('--timeout-sec', type=float, default=10.0)
    parser.add_argument('--reject-move', action='store_true')
    arguments = parser.parse_args(argv)
    if not arguments.expected_server_instance_id.strip():
        parser.error('--expected-server-instance-id must not be empty')
    if not 0 < arguments.status_port <= 65535:
        parser.error('--status-port must be in range 1..65535')
    if arguments.timeout_sec <= 0.0:
        parser.error('--timeout-sec must be positive')
    return arguments


def main(argv=None):
    """在有界时间内等待精确实例 ACK，超时或 MOVE 均 fail-closed。"""
    arguments = parse_arguments(argv)
    rclpy.init(args=None)
    node = StatusGateNode(arguments)
    deadline = time.monotonic() + arguments.timeout_sec
    try:
        while (
            node.result is None
            and node.failure is None
            and time.monotonic() < deadline
        ):
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.failure is not None:
            print('SDK_STATUS_GATE ' + json.dumps(
                node.failure, separators=(',', ':'), allow_nan=False
            ))
            return 2
        if node.result is None:
            print('SDK_STATUS_GATE ' + json.dumps({
                'classification': 'TIMEOUT',
                'mode': arguments.mode,
                'expected_server_instance_id': (
                    arguments.expected_server_instance_id
                ),
                'event': arguments.event,
            }, separators=(',', ':'), allow_nan=False))
            return 1
        print('SDK_STATUS_GATE ' + json.dumps({
            'classification': 'PASS',
            'mode': arguments.mode,
            'value': node.result,
        }, separators=(',', ':'), allow_nan=False))
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
