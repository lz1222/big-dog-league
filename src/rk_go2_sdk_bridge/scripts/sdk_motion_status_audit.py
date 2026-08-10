#!/usr/bin/env python3
"""在有界窗口内只读汇总当前 SDK server 实例的 ROS 状态历史。"""

import argparse
import json
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from rclpy.qos import ReliabilityPolicy
from std_msgs.msg import String

from sdk_motion_status import SdkMotionStatusTracker


class StatusAuditNode(Node):
    """收集 transient-local 状态；不创建 publisher，不触碰命令链。"""

    def __init__(self, expected_server_instance_id):
        super().__init__('sdk_motion_status_audit')
        self.tracker = SdkMotionStatusTracker(expected_server_instance_id)
        self.statuses = []
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            String, '/go2/sdk_motion_status', self._on_status, qos
        )

    def _on_status(self, message):
        status = self.tracker.observe(message.data)
        if status is not None:
            self.statuses.append(status)


def summarize(statuses, expected_server_instance_id):
    """验证启动停车、随后经典 ACK、零命令停车及无 MOVE 的闭环合同。"""
    startup = [
        status for status in statuses
        if status['event'] == 'STARTUP_STOP' and status['ret'] == 0
    ]
    stop = [
        status for status in statuses
        if status['event'] == 'STOP_MOVE' and status['ret'] == 0
    ]
    classic_verified = [
        status for status in statuses
        if status['event'] == 'CLASSIC_VERIFIED' and status['ret'] == 0
    ]
    move = [status for status in statuses if status['event'] == 'MOVE']
    errors = [status for status in statuses if status['event'] == 'SDK_ERROR']
    startup_sequence = startup[-1]['sequence'] if startup else 0
    classic_verified_sequence = (
        classic_verified[-1]['sequence'] if classic_verified else 0
    )
    zero_sequence = stop[-1]['sequence'] if stop else 0
    sequences = [status['sequence'] for status in statuses]
    monotonic = all(
        current > previous
        for previous, current in zip(sequences, sequences[1:])
    )
    success = (
        bool(startup)
        and bool(classic_verified)
        and bool(stop)
        and startup_sequence < classic_verified_sequence < zero_sequence
        and not move
        and not errors
        and monotonic
    )
    return {
        'success': success,
        'server_instance_id': expected_server_instance_id,
        'status_count': len(statuses),
        'startup_sequence': startup_sequence,
        'classic_verified_sequence': classic_verified_sequence,
        'zero_sequence': zero_sequence,
        'move_count': len(move),
        'sdk_error_count': len(errors),
        'sequence_monotonic': monotonic,
        'events': [status['event'] for status in statuses],
        'statuses': statuses,
    }


def parse_arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--expected-server-instance-id', required=True)
    parser.add_argument('--duration-sec', type=float, default=2.0)
    arguments = parser.parse_args(argv)
    if not arguments.expected_server_instance_id.strip():
        parser.error('--expected-server-instance-id must not be empty')
    if arguments.duration_sec <= 0.0:
        parser.error('--duration-sec must be positive')
    return arguments


def main(argv=None):
    arguments = parse_arguments(argv)
    rclpy.init(args=None)
    node = StatusAuditNode(arguments.expected_server_instance_id)
    deadline = time.monotonic() + arguments.duration_sec
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        result = summarize(
            node.statuses, arguments.expected_server_instance_id
        )
        print('SDK_STATUS_AUDIT ' + json.dumps(
            result, separators=(',', ':'), allow_nan=False
        ))
        return 0 if result['success'] else 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
