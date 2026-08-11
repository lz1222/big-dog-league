#!/usr/bin/env python3
"""以 status receiver 的当前实例握手门控正式 SDK server 启动。

本进程不创建任何速度、动作或步态控制接口。它只等待已经绑定 UDP status
端口的 forwarder 发布 current-instance ready；握手超时则不启动 SDK server。
"""

import argparse
import os
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from rclpy.qos import ReliabilityPolicy
from std_msgs.msg import String

from sdk_motion_status_gate import READY_TOPIC, decode_receiver_ready


class ReceiverStartupContract:
    """保存唯一可放行 server 的 receiver-ready，不接受错实例或旧格式。"""

    def __init__(self, expected_instance_id, status_ip, status_port):
        self.expected_instance_id = expected_instance_id
        self.status_ip = status_ip
        self.status_port = status_port
        self.ready = None

    def observe(self, payload):
        """返回本条 ready 是否精确匹配当前启动身份。"""
        if self.ready is not None:
            return True
        value = decode_receiver_ready(
            payload,
            self.expected_instance_id,
            self.status_ip,
            self.status_port,
        )
        if value is None:
            return False
        self.ready = value
        return True


class ReceiverReadyLatch(Node):
    """只接受当前启动 nonce 的 receiver-ready，避免旧 transient 样本放行。"""

    def __init__(self, arguments):
        super().__init__('sdk_server_start_gate')
        self.arguments = arguments
        self.contract = ReceiverStartupContract(
            arguments.server_instance_id,
            arguments.status_ip,
            arguments.status_port,
        )
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(String, READY_TOPIC, self._on_ready, qos)

    def _on_ready(self, message):
        self.contract.observe(message.data)


def _executable(value, option):
    """拒绝不可执行路径，避免握手通过后把启动失败误记为 SDK 故障。"""
    path = os.path.abspath(value)
    if not os.path.isfile(path) or not os.access(path, os.X_OK):
        raise argparse.ArgumentTypeError(
            '{} must name an executable file: {}'.format(option, value)
        )
    return path


def parse_arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--runtime-wrapper', required=True)
    parser.add_argument('--sdk-server', required=True)
    parser.add_argument('--interface', required=True)
    parser.add_argument('--listen-ip', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=15001)
    parser.add_argument('--status-ip', default='127.0.0.1')
    parser.add_argument('--status-port', type=int, default=15002)
    parser.add_argument('--server-instance-id', required=True)
    parser.add_argument('--max-vx', type=float, required=True)
    parser.add_argument('--max-vy', type=float, required=True)
    parser.add_argument('--max-yaw', type=float, required=True)
    parser.add_argument(
        '--manual-classic-confirmed', choices=('true', 'false'),
        default='false'
    )
    parser.add_argument(
        '--receiver-ready-timeout-sec', type=float, default=12.0
    )
    arguments = parser.parse_args(argv)
    arguments.runtime_wrapper = _executable(
        arguments.runtime_wrapper, '--runtime-wrapper'
    )
    arguments.sdk_server = _executable(
        arguments.sdk_server, '--sdk-server'
    )
    if not arguments.server_instance_id.strip():
        parser.error('--server-instance-id must not be empty')
    if not arguments.interface.strip() or not arguments.listen_ip:
        parser.error('--interface and --listen-ip must not be empty')
    for option, value in (
            ('--port', arguments.port),
            ('--status-port', arguments.status_port)):
        if not 0 < value <= 65535:
            parser.error('{} must be in range 1..65535'.format(option))
    for option, value in (
            ('--max-vx', arguments.max_vx), ('--max-vy', arguments.max_vy),
            ('--max-yaw', arguments.max_yaw),
            ('--receiver-ready-timeout-sec',
             arguments.receiver_ready_timeout_sec)):
        if value <= 0.0:
            parser.error('{} must be positive'.format(option))
    return arguments


def server_command(arguments):
    """生成唯一既有 server 的参数，不加入第二个 SDK writer。"""
    return [
        arguments.runtime_wrapper, arguments.sdk_server,
        '--interface', arguments.interface,
        '--listen-ip', arguments.listen_ip,
        '--port', str(arguments.port),
        '--status-ip', arguments.status_ip,
        '--status-port', str(arguments.status_port),
        '--server-instance-id', arguments.server_instance_id,
        '--max-vx', str(arguments.max_vx),
        '--max-vy', str(arguments.max_vy),
        '--max-yaw', str(arguments.max_yaw),
        '--manual-classic-confirmed', arguments.manual_classic_confirmed,
    ]


def wait_for_receiver_ready(node, timeout_sec):
    """以单调 deadline 等待 TLS replay/current ready，绝不以 sleep 猜测。"""
    deadline = time.monotonic() + timeout_sec
    while (rclpy.ok() and node.contract.ready is None
           and time.monotonic() < deadline):
        rclpy.spin_once(node, timeout_sec=0.1)
    return node.contract.ready


def _forward_signal(child, signum, _frame):
    """父进程退出时让既有 server 走其 signal-exit StopMove 路径。"""
    if child.poll() is None:
        child.send_signal(signum)


def main(argv=None):
    arguments = parse_arguments(argv)
    rclpy.init(args=None)
    node = ReceiverReadyLatch(arguments)
    child = None
    try:
        print(
            'SDK_SERVER_START_GATE event=WAIT_RECEIVER_READY '
            'instance={} status={}:{}'.format(
                arguments.server_instance_id, arguments.status_ip,
                arguments.status_port,
            ), flush=True,
        )
        ready = wait_for_receiver_ready(
            node, arguments.receiver_ready_timeout_sec
        )
        if ready is None:
            print(
                'SDK_SERVER_START_GATE event=REFUSED '
                'reason=receiver_ready_timeout instance={}'.format(
                    arguments.server_instance_id
                ), flush=True,
            )
            return 1
        print(
            'SDK_SERVER_START_GATE event=RECEIVER_READY '
            'receiver_instance_id={} ready_monotonic_ns={}'.format(
                ready['receiver_instance_id'], ready['ready_monotonic_ns'],
            ), flush=True,
        )
        command = server_command(arguments)
        child = subprocess.Popen(command)
        signal.signal(signal.SIGINT, lambda signum, frame: _forward_signal(
            child, signum, frame
        ))
        signal.signal(signal.SIGTERM, lambda signum, frame: _forward_signal(
            child, signum, frame
        ))
        print(
            'SDK_SERVER_START_GATE event=SERVER_STARTED pid={}'.format(
                child.pid
            ), flush=True,
        )
        return child.wait()
    finally:
        if child is not None and child.poll() is None:
            child.terminate()
            child.wait(timeout=3.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
