#!/usr/bin/env python3

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, String

from rk_interfaces.msg import LineTrack


class LineNavTestClientNode(Node):
    """Standalone helper for starting and observing line navigation."""

    def __init__(self):
        super().__init__('line_nav_test_client_node')

        self.declare_parameter('mission_start_topic', '/mission/start')
        self.declare_parameter('mission_stop_topic', '/mission/stop')
        self.declare_parameter('line_track_topic', '/perception/line_track')
        self.declare_parameter('cmd_vel_topic', '/navigation/cmd_vel')
        self.declare_parameter(
            'state_topic',
            '/navigation/line_follower/state'
        )
        self.declare_parameter('duration_sec', 10.0)
        self.declare_parameter('start_delay_sec', 0.5)
        self.declare_parameter('repeat_count', 3)
        self.declare_parameter('repeat_period_sec', 0.1)
        self.declare_parameter('stop_before_start', True)
        self.declare_parameter('stop_on_exit', True)
        self.declare_parameter('summary_period_sec', 1.0)

        self.mission_start_topic = self.string_parameter(
            'mission_start_topic'
        )
        self.mission_stop_topic = self.string_parameter('mission_stop_topic')
        self.line_track_topic = self.string_parameter('line_track_topic')
        self.cmd_vel_topic = self.string_parameter('cmd_vel_topic')
        self.state_topic = self.string_parameter('state_topic')
        self.duration_sec = self.nonnegative_float_parameter('duration_sec')
        self.start_delay_sec = self.nonnegative_float_parameter(
            'start_delay_sec'
        )
        self.repeat_count = self.positive_int_parameter('repeat_count')
        self.repeat_period_sec = self.nonnegative_float_parameter(
            'repeat_period_sec'
        )
        self.stop_before_start = self.bool_parameter('stop_before_start')
        self.stop_on_exit = self.bool_parameter('stop_on_exit')
        self.summary_period_sec = self.positive_float_parameter(
            'summary_period_sec'
        )

        self.start_publisher = self.create_publisher(
            Bool,
            self.mission_start_topic,
            10
        )
        self.stop_publisher = self.create_publisher(
            Bool,
            self.mission_stop_topic,
            10
        )
        self.create_subscription(
            LineTrack,
            self.line_track_topic,
            self.on_line_track,
            10
        )
        self.create_subscription(
            Twist,
            self.cmd_vel_topic,
            self.on_cmd_vel,
            10
        )
        self.create_subscription(
            String,
            self.state_topic,
            self.on_state,
            10
        )

        self.last_line = None
        self.last_cmd = None
        self.last_state = 'UNKNOWN'
        self.line_count = 0
        self.cmd_count = 0
        self.state_count = 0
        self.started_at = None
        self.last_summary_time = 0.0

    def run(self):
        self.get_logger().info(
            'Line nav test client ready: '
            f'duration={self.duration_sec:.2f}s, '
            f'line_track={self.line_track_topic}, '
            f'cmd_vel={self.cmd_vel_topic}, '
            f'state={self.state_topic}'
        )

        if self.stop_before_start:
            self.publish_bool(self.stop_publisher, True, 'stop before start')

        self.spin_for(self.start_delay_sec)
        self.publish_bool(self.start_publisher, True, 'start line navigation')
        self.started_at = time.monotonic()

        try:
            while rclpy.ok() and not self.duration_elapsed():
                rclpy.spin_once(self, timeout_sec=0.05)
                self.maybe_log_summary()
        finally:
            if self.stop_on_exit:
                self.publish_bool(self.stop_publisher, True, 'stop on exit')
                self.spin_for(0.2)

        self.log_summary('final')

    def duration_elapsed(self):
        if self.duration_sec <= 0.0:
            return False
        if self.started_at is None:
            return False
        return time.monotonic() - self.started_at >= self.duration_sec

    def spin_for(self, duration_sec):
        deadline = time.monotonic() + float(duration_sec)
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def publish_bool(self, publisher, value, reason):
        msg = Bool()
        msg.data = bool(value)
        for index in range(self.repeat_count):
            publisher.publish(msg)
            self.get_logger().info(
                f'Published {reason} ({index + 1}/{self.repeat_count})'
            )
            if (
                index + 1 < self.repeat_count
                and self.repeat_period_sec > 0.0
            ):
                self.spin_for(self.repeat_period_sec)

    def on_line_track(self, msg):
        self.last_line = msg
        self.line_count += 1

    def on_cmd_vel(self, msg):
        self.last_cmd = msg
        self.cmd_count += 1

    def on_state(self, msg):
        self.last_state = msg.data
        self.state_count += 1

    def maybe_log_summary(self):
        now = time.monotonic()
        if now - self.last_summary_time < self.summary_period_sec:
            return
        self.last_summary_time = now
        self.log_summary('summary')

    def log_summary(self, label):
        line = self.line_summary()
        cmd = self.cmd_summary()
        elapsed = 0.0
        if self.started_at is not None:
            elapsed = time.monotonic() - self.started_at
        self.get_logger().info(
            f'{label}: elapsed={elapsed:.2f}s, '
            f'state={self.last_state}, '
            f'line_count={self.line_count}, cmd_count={self.cmd_count}, '
            f'{line}, {cmd}'
        )

    def line_summary(self):
        if self.last_line is None:
            return 'line=None'
        return (
            'line='
            f'visible={bool(self.last_line.line_visible)}, '
            f'conf={float(self.last_line.confidence):.2f}, '
            f'lat={float(self.last_line.lateral_error):.3f}, '
            f'head={float(self.last_line.heading_error):.3f}'
        )

    def cmd_summary(self):
        if self.last_cmd is None:
            return 'cmd=None'
        return (
            'cmd='
            f'vx={float(self.last_cmd.linear.x):.3f}, '
            f'wz={float(self.last_cmd.angular.z):.3f}'
        )

    def string_parameter(self, name):
        value = str(self.get_parameter(name).value).strip()
        if not value:
            raise ValueError(f'{name} must not be empty')
        return value

    def bool_parameter(self, name):
        return bool(self.get_parameter(name).value)

    def nonnegative_float_parameter(self, name):
        value = float(self.get_parameter(name).value)
        if value < 0.0:
            raise ValueError(f'{name} must be nonnegative')
        return value

    def positive_float_parameter(self, name):
        value = self.nonnegative_float_parameter(name)
        if value <= 0.0:
            raise ValueError(f'{name} must be positive')
        return value

    def positive_int_parameter(self, name):
        value = int(self.get_parameter(name).value)
        if value <= 0:
            raise ValueError(f'{name} must be positive')
        return value


def main(args=None):
    rclpy.init(args=args)
    node = LineNavTestClientNode()
    try:
        node.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        node.get_logger().warn('Interrupted by user')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
