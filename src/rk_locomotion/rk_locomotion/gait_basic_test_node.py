#!/usr/bin/env python3

import json
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


class GaitBasicTestNode(Node):
    """Publish basic gait JSON commands in a safe, low-speed sequence."""

    def __init__(self):
        super().__init__('gait_basic_test_node')
        self.command_topic = self.declare_parameter(
            'command_json_topic',
            '/gait/command_json'
        ).value
        self.publisher = self.create_publisher(String, self.command_topic, 10)

    def run(self):
        self.publish_command('STOP')
        self.sleep_with_spin(0.5)

        self.publish_command('RECOVERY_STAND')
        self.sleep_with_spin(0.5)

        self.publish_command('HOLD_STABLE', duration_sec=3.0)
        self.sleep_with_spin(3.5)

        self.publish_command('LOW_SPEED_MOVE', vx=0.1, duration_sec=2.0)
        self.sleep_with_spin(2.5)

        self.publish_command('STOP')
        self.sleep_with_spin(0.5)

    def publish_command(self, command, **kwargs):
        payload = {'command': command}
        payload.update(kwargs)

        msg = String()
        msg.data = json.dumps(payload, separators=(',', ':'))
        self.publisher.publish(msg)
        self.get_logger().info(f'Published {self.command_topic}: {msg.data}')

    def sleep_with_spin(self, duration_sec):
        end_time = time.monotonic() + float(duration_sec)
        while rclpy.ok() and time.monotonic() < end_time:
            rclpy.spin_once(self, timeout_sec=0.05)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = GaitBasicTestNode()
        node.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        if node is not None and rclpy.ok():
            node.get_logger().warn('Interrupted by user')
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
