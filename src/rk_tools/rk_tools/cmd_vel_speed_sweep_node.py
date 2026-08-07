#!/usr/bin/env python3

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class CmdVelSpeedSweepNode(Node):
    """Publish a cmd_vel speed sweep to find the robot's motion deadband."""

    VALID_MODES = {'linear', 'angular'}

    def __init__(self):
        super().__init__('cmd_vel_speed_sweep_node')

        self.cmd_vel_topic = self.declare_parameter(
            'cmd_vel_topic',
            '/navigation/cmd_vel'
        ).value
        self.mode = str(self.declare_parameter('mode', 'linear').value)
        self.speeds_csv = str(self.declare_parameter(
            'speeds_csv',
            '0.05,0.10,0.15,0.20,0.25,0.30,0.35,0.40'
        ).value)
        self.duration_sec = float(self.declare_parameter(
            'duration_sec',
            2.0
        ).value)
        self.stop_sec = float(self.declare_parameter('stop_sec', 1.0).value)
        self.publish_rate_hz = float(self.declare_parameter(
            'publish_rate_hz',
            10.0
        ).value)
        self.direction = float(self.declare_parameter('direction', 1.0).value)

        self.speeds = self._parse_speeds(self.speeds_csv)
        self._validate_parameters()

        self.publisher = self.create_publisher(Twist, self.cmd_vel_topic, 10)

    def _parse_speeds(self, speeds_csv):
        speeds = []
        for raw_speed in speeds_csv.split(','):
            value = raw_speed.strip()
            if not value:
                continue
            speed = float(value)
            if not math.isfinite(speed) or speed < 0.0:
                raise ValueError(
                    'speeds_csv must contain finite nonnegative numbers'
                )
            speeds.append(speed)

        if not speeds:
            raise ValueError('speeds_csv must contain at least one speed')

        return speeds

    def _validate_parameters(self):
        if not self.cmd_vel_topic:
            raise ValueError('cmd_vel_topic must not be empty')

        if self.mode not in self.VALID_MODES:
            valid_modes = ', '.join(sorted(self.VALID_MODES))
            raise ValueError(f'mode must be one of: {valid_modes}')

        positive_parameters = {
            'duration_sec': self.duration_sec,
            'publish_rate_hz': self.publish_rate_hz,
        }
        for name, value in positive_parameters.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f'{name} must be a finite positive number')

        if not math.isfinite(self.stop_sec) or self.stop_sec < 0.0:
            raise ValueError('stop_sec must be a finite nonnegative number')

        if not math.isfinite(self.direction) or self.direction == 0.0:
            raise ValueError('direction must be a finite nonzero number')

        self.direction = 1.0 if self.direction > 0.0 else -1.0

    def run(self):
        self.get_logger().info(
            f'Starting {self.mode} cmd_vel sweep on {self.cmd_vel_topic}: '
            f'speeds={self.speeds}, duration={self.duration_sec:.2f}s, '
            f'stop={self.stop_sec:.2f}s, direction={self.direction:+.0f}'
        )

        self._publish_stop('initial stop')
        try:
            for speed in self.speeds:
                cmd = self._make_cmd(speed)
                signed_speed = self.direction * speed
                if self.mode == 'linear':
                    self.get_logger().info(
                        f'TEST linear.x={signed_speed:.3f} m/s '
                        f'for {self.duration_sec:.2f}s'
                    )
                else:
                    self.get_logger().info(
                        f'TEST angular.z={signed_speed:.3f} rad/s '
                        f'for {self.duration_sec:.2f}s'
                    )

                self._publish_for_duration(cmd, self.duration_sec)
                self._publish_stop('step stop')
        finally:
            self._publish_stop('final stop')

    def _make_cmd(self, speed):
        cmd = Twist()
        signed_speed = self.direction * speed

        if self.mode == 'linear':
            cmd.linear.x = signed_speed
        else:
            cmd.angular.z = signed_speed

        return cmd

    def _publish_stop(self, label):
        if self.stop_sec <= 0.0:
            return

        self.get_logger().info(f'{label}: zero cmd_vel for {self.stop_sec:.2f}s')
        self._publish_for_duration(Twist(), self.stop_sec)

    def _publish_for_duration(self, cmd, duration_sec):
        period_sec = 1.0 / self.publish_rate_hz
        end_time = time.monotonic() + duration_sec

        while rclpy.ok() and time.monotonic() < end_time:
            self.publisher.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period_sec)


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = CmdVelSpeedSweepNode()
        node.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        if node is not None:
            node.get_logger().warn('Interrupted by user')
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
