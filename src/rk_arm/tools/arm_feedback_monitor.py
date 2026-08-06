#!/usr/bin/env python3
"""只订阅 arm/raw_state，供人工确认反馈；不含任何 DDS 或运动控制代码。"""
import rclpy
from rclpy.node import Node
from rk_interfaces.msg import ArmRawState


class Monitor(Node):
    def __init__(self):
        super().__init__('arm_feedback_monitor')
        self.create_subscription(ArmRawState, 'arm/raw_state', self._callback, 10)

    def _callback(self, message):
        self.get_logger().info(f'valid={message.feedback_valid} stale={message.feedback_stale} reason={message.reason} values={list(message.app_values)}')


def main():
    rclpy.init(); node = Monitor()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__': main()
