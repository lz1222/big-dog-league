#!/usr/bin/env python3

import math
import time
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


@dataclass(frozen=True)
class RouteStep:
    name: str
    kind: str
    value: float
    speed: float


# Hardcoded obstacle-zone route.
#
# Edit only this table for first-round field tuning:
# - forward value: distance in meters
# - turn_left / turn_right value: angle in degrees
# - forward speed: linear.x in m/s
# - turn speed: angular.z in rad/s
ROUTE_STEPS = [
    RouteStep('entry_up', 'forward', 0.48, 0.35),
    RouteStep('turn_left_to_top', 'turn_left', 90.0, 0.80),
    RouteStep('top_left', 'forward', 0.40, 0.35),
    RouteStep('turn_left_to_middle_down', 'turn_left', 90.0, 0.80),
    RouteStep('middle_down', 'forward', 0.40, 0.35),
    RouteStep('turn_right_to_bottom_left', 'turn_right', 90.0, 0.80),
    RouteStep('bottom_left', 'forward', 0.32, 0.35),
    RouteStep('turn_right_to_left_up', 'turn_right', 90.0, 0.80),
    RouteStep('left_up', 'forward', 0.40, 0.35),
    RouteStep('turn_left_to_exit', 'turn_left', 90.0, 0.80),
    RouteStep('exit_left', 'forward', 0.32, 0.35),
]


class ObstacleDirectRouteNode(Node):
    """Publish a hardcoded obstacle-zone route directly to cmd_vel."""

    VALID_STEP_TYPES = {'forward', 'turn_left', 'turn_right'}

    def __init__(self):
        super().__init__('obstacle_direct_route_node')

        self.cmd_vel_topic = self.declare_parameter(
            'cmd_vel_topic',
            '/navigation/cmd_vel'
        ).value
        self.publish_rate_hz = float(self.declare_parameter(
            'publish_rate_hz',
            20.0
        ).value)
        self.countdown_sec = float(self.declare_parameter(
            'countdown_sec',
            3.0
        ).value)
        self.pre_stop_sec = float(self.declare_parameter(
            'pre_stop_sec',
            0.8
        ).value)
        self.step_stop_sec = float(self.declare_parameter(
            'step_stop_sec',
            0.25
        ).value)
        self.final_stop_sec = float(self.declare_parameter(
            'final_stop_sec',
            1.0
        ).value)
        self.distance_scale = float(self.declare_parameter(
            'distance_scale',
            1.0
        ).value)
        self.turn_scale = float(self.declare_parameter(
            'turn_scale',
            1.0
        ).value)
        self.speed_scale = float(self.declare_parameter(
            'speed_scale',
            1.0
        ).value)

        self._validate_parameters()
        self.publisher = self.create_publisher(Twist, self.cmd_vel_topic, 10)

    def _validate_parameters(self):
        if not self.cmd_vel_topic:
            raise ValueError('cmd_vel_topic must not be empty')

        positive = {
            'publish_rate_hz': self.publish_rate_hz,
            'distance_scale': self.distance_scale,
            'turn_scale': self.turn_scale,
            'speed_scale': self.speed_scale,
        }
        for name, value in positive.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f'{name} must be a finite positive number')

        nonnegative = {
            'countdown_sec': self.countdown_sec,
            'pre_stop_sec': self.pre_stop_sec,
            'step_stop_sec': self.step_stop_sec,
            'final_stop_sec': self.final_stop_sec,
        }
        for name, value in nonnegative.items():
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(
                    f'{name} must be a finite nonnegative number'
                )

        for step in ROUTE_STEPS:
            if step.kind not in self.VALID_STEP_TYPES:
                raise ValueError(f'invalid step type: {step.kind}')
            if step.value <= 0.0 or step.speed <= 0.0:
                raise ValueError(
                    f'{step.name} must have positive value and speed'
                )

    def run(self):
        self.get_logger().warn(
            'Direct hardcoded obstacle route will start. '
            f'cmd_vel={self.cmd_vel_topic}, steps={len(ROUTE_STEPS)}, '
            f'countdown={self.countdown_sec:.1f}s'
        )

        self._publish_stop('countdown stop', self.countdown_sec)
        self._publish_stop('pre-route stop', self.pre_stop_sec)

        try:
            for index, step in enumerate(ROUTE_STEPS, start=1):
                self._run_step(index, len(ROUTE_STEPS), step)
                self._publish_stop(
                    f'after {step.name}',
                    self.step_stop_sec
                )
        finally:
            self._publish_stop('final stop', self.final_stop_sec)

        self.get_logger().info('Direct obstacle route completed.')

    def _run_step(self, index, total, step):
        cmd = Twist()
        duration_sec = self._step_duration_sec(step)

        if step.kind == 'forward':
            cmd.linear.x = step.speed * self.speed_scale
            detail = (
                f'distance={step.value * self.distance_scale:.3f}m, '
                f'vx={cmd.linear.x:.3f}m/s'
            )
        elif step.kind == 'turn_left':
            cmd.angular.z = step.speed * self.speed_scale
            detail = (
                f'angle={step.value * self.turn_scale:.1f}deg, '
                f'wz={cmd.angular.z:.3f}rad/s'
            )
        else:
            cmd.angular.z = -step.speed * self.speed_scale
            detail = (
                f'angle={step.value * self.turn_scale:.1f}deg, '
                f'wz={cmd.angular.z:.3f}rad/s'
            )

        self.get_logger().info(
            f'route step {index}/{total}: {step.name} '
            f'({step.kind}), {detail}, duration={duration_sec:.2f}s'
        )
        self._publish_for_duration(cmd, duration_sec)

    def _step_duration_sec(self, step):
        if step.kind == 'forward':
            distance_m = step.value * self.distance_scale
            speed_mps = step.speed * self.speed_scale
            return distance_m / speed_mps

        angle_rad = math.radians(step.value * self.turn_scale)
        speed_radps = step.speed * self.speed_scale
        return angle_rad / speed_radps

    def _publish_stop(self, label, duration_sec):
        if duration_sec <= 0.0:
            return

        self.get_logger().info(
            f'{label}: zero cmd_vel for {duration_sec:.2f}s'
        )
        self._publish_for_duration(Twist(), duration_sec)

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
        node = ObstacleDirectRouteNode()
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
