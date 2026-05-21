#!/usr/bin/env python3

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from geometry_msgs.msg import Twist
from rk_interfaces.msg import LineTrack


class LineFollowerNode(Node):
    """Convert line tracking data into a conservative velocity command."""

    def __init__(self):
        super().__init__('line_follower_node')

        self.declare_parameter('forward_speed', 0.10)
        self.declare_parameter('min_confidence', 0.15)
        self.declare_parameter('lateral_gain', 0.80)
        self.declare_parameter('heading_gain', 0.40)
        self.declare_parameter('max_angular_z', 0.30)
        self.declare_parameter('cmd_vel_topic', '/navigation/cmd_vel')
        self.declare_parameter('line_track_topic', '/perception/line_track')
        self.declare_parameter('lost_line_stop', True)
        self.declare_parameter('debug_log', True)

        self.cmd_vel_topic = self.get_parameter(
            'cmd_vel_topic'
        ).get_parameter_value().string_value
        self.line_track_topic = self.get_parameter(
            'line_track_topic'
        ).get_parameter_value().string_value

        self.last_debug_log_ns = 0
        self.debug_log_period_ns = 1_000_000_000
        self.refresh_parameters()

        self.publisher = self.create_publisher(
            Twist,
            self.cmd_vel_topic,
            10
        )
        self.subscription = self.create_subscription(
            LineTrack,
            self.line_track_topic,
            self.on_line_track,
            10
        )
        self.get_logger().info(
            'Line follower node started: '
            f'line_track_topic={self.line_track_topic}, '
            f'cmd_vel_topic={self.cmd_vel_topic}'
        )

    def refresh_parameters(self):
        """Refresh runtime-tunable control parameters."""
        self.forward_speed = self.get_parameter(
            'forward_speed'
        ).get_parameter_value().double_value
        self.min_confidence = self.get_parameter(
            'min_confidence'
        ).get_parameter_value().double_value
        self.lateral_gain = self.get_parameter(
            'lateral_gain'
        ).get_parameter_value().double_value
        self.heading_gain = self.get_parameter(
            'heading_gain'
        ).get_parameter_value().double_value
        self.max_angular_z = self.get_parameter(
            'max_angular_z'
        ).get_parameter_value().double_value
        self.lost_line_stop = self.get_parameter(
            'lost_line_stop'
        ).get_parameter_value().bool_value
        self.debug_log = self.get_parameter(
            'debug_log'
        ).get_parameter_value().bool_value

        self.forward_speed = max(0.0, self.forward_speed)
        self.min_confidence = self.clamp(self.min_confidence, 0.0, 1.0)
        self.max_angular_z = max(0.0, self.max_angular_z)

    def on_line_track(self, msg):
        self.refresh_parameters()

        if not self.is_valid_input(msg):
            self.publish_stop('invalid data', msg)
            return

        if not msg.line_visible:
            self.publish_stop('line lost', msg)
            return

        if msg.confidence < self.min_confidence:
            self.publish_stop('confidence low', msg)
            return

        cmd = Twist()
        angular = -(
            self.lateral_gain * msg.lateral_error +
            self.heading_gain * msg.heading_error
        )
        angular = self.clamp(
            angular,
            -self.max_angular_z,
            self.max_angular_z
        )

        cmd.linear.x = self.forward_speed
        cmd.angular.z = angular
        self.publisher.publish(cmd)

        self.log_debug(
            'cmd_vel: '
            f'linear.x={cmd.linear.x:.3f}, '
            f'angular.z={cmd.angular.z:.3f}, '
            f'lateral_error={msg.lateral_error:.3f}, '
            f'heading_error={msg.heading_error:.3f}, '
            f'confidence={msg.confidence:.3f}'
        )

    def is_valid_input(self, msg):
        values = [
            msg.lateral_error,
            msg.heading_error,
            msg.confidence,
            self.forward_speed,
            self.min_confidence,
            self.lateral_gain,
            self.heading_gain,
            self.max_angular_z,
        ]
        return all(math.isfinite(float(value)) for value in values)

    def publish_stop(self, reason, msg):
        cmd = Twist()
        self.publisher.publish(cmd)
        self.log_debug(
            'stopping: '
            f'reason={reason}, '
            f'lateral_error={msg.lateral_error:.3f}, '
            f'heading_error={msg.heading_error:.3f}, '
            f'confidence={msg.confidence:.3f}, '
            f'line_visible={msg.line_visible}'
        )

    def log_debug(self, message):
        if not self.debug_log:
            return

        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_debug_log_ns < self.debug_log_period_ns:
            return

        self.last_debug_log_ns = now_ns
        self.get_logger().info(message)

    @staticmethod
    def clamp(value, minimum, maximum):
        return max(minimum, min(maximum, value))


def main(args=None):
    rclpy.init(args=args)
    node = LineFollowerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
