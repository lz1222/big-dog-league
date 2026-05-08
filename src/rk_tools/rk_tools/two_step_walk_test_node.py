#!/usr/bin/env python3

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class TwoStepWalkTestNode(Node):
    """Publish a short, low-speed forward command followed by stop commands."""

    def __init__(self):
        super().__init__('two_step_walk_test_node')

        self.cmd_vel_topic = self.declare_parameter(
            'cmd_vel_topic',
            '/navigation/cmd_vel'
        ).value
        self.forward_speed = self.declare_parameter(
            'forward_speed',
            0.10
        ).value
        self.walk_duration = self.declare_parameter(
            'walk_duration',
            2.0
        ).value
        self.publish_rate = self.declare_parameter(
            'publish_rate',
            10.0
        ).value
        self.stop_duration = self.declare_parameter(
            'stop_duration',
            1.0
        ).value

        self.validate_parameters()
        self.publisher = self.create_publisher(Twist, self.cmd_vel_topic, 10)

    def validate_parameters(self):
        if not self.cmd_vel_topic:
            raise ValueError('cmd_vel_topic must not be empty')

        positive_parameters = {
            'forward_speed': self.forward_speed,
            'walk_duration': self.walk_duration,
            'publish_rate': self.publish_rate,
            'stop_duration': self.stop_duration,
        }
        for name, value in positive_parameters.items():
            if value <= 0.0:
                raise ValueError(f'{name} must be positive, got {value}')

    def run(self):
        self.get_logger().info(
            f'Publishing forward cmd_vel to {self.cmd_vel_topic}: '
            f'linear.x={self.forward_speed:.2f} m/s for '
            f'{self.walk_duration:.2f} s'
        )

        forward_cmd = Twist()
        forward_cmd.linear.x = float(self.forward_speed)

        try:
            self.publish_for_duration(forward_cmd, self.walk_duration)
        finally:
            self.get_logger().info(
                f'Publishing stop cmd_vel for {self.stop_duration:.2f} s'
            )
            self.stop()

    def stop(self):
        stop_cmd = Twist()
        self.publish_for_duration(stop_cmd, self.stop_duration)

    def publish_for_duration(self, cmd, duration):
        period = 1.0 / float(self.publish_rate)
        end_time = time.monotonic() + float(duration)

        while rclpy.ok() and time.monotonic() < end_time:
            self.publisher.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TwoStepWalkTestNode()
        node.run()
    except KeyboardInterrupt:
        if node is not None:
            node.get_logger().warn('Interrupted by user')
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
