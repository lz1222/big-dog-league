#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from rk_unitree_driver.go2_motion_client import Go2MotionClient
from rk_unitree_driver.safety_monitor import SafetyMonitor


class CmdVelBridgeNode(Node):
    """Bridge /navigation/cmd_vel to Unitree Go2 Sport requests."""

    DYNAMIC_LIMIT_PARAMS = {'max_linear_x', 'max_angular_z'}

    def __init__(self):
        super().__init__('cmd_vel_bridge_node')

        self.declare_parameter('backend', Go2MotionClient.MOCK_BACKEND)
        self.declare_parameter('cmd_vel_topic', '/navigation/cmd_vel')
        self.declare_parameter('sport_request_topic', '/api/sport/request')
        self.declare_parameter('max_linear_x', 0.20)
        self.declare_parameter('max_angular_z', 0.60)
        self.declare_parameter('cmd_timeout_sec', 0.50)
        self.declare_parameter('stop_publish_count', 3)
        self.declare_parameter('stop_publish_period_sec', 0.05)
        self.declare_parameter('zero_cmd_debounce_time', 0.60)
        self.declare_parameter('stop_publish_min_interval', 0.50)

        self.backend = self._backend_parameter()
        self.cmd_vel_topic = self._string_parameter('cmd_vel_topic')
        self.sport_request_topic = self._string_parameter(
            'sport_request_topic'
        )
        self.cmd_timeout_sec = self._positive_float_parameter(
            'cmd_timeout_sec'
        )
        self.stop_publish_count = self._positive_int_parameter(
            'stop_publish_count'
        )
        self.stop_publish_period_sec = self._nonnegative_float_parameter(
            'stop_publish_period_sec'
        )
        self.zero_cmd_debounce_time = self._nonnegative_float_parameter(
            'zero_cmd_debounce_time'
        )
        self.stop_publish_min_interval = self._nonnegative_float_parameter(
            'stop_publish_min_interval'
        )

        max_linear_x = self._positive_float_parameter('max_linear_x')
        max_angular_z = self._positive_float_parameter('max_angular_z')

        self._safety_monitor = SafetyMonitor(max_linear_x, max_angular_z)
        self._motion_client = Go2MotionClient(
            self,
            self.sport_request_topic,
            self.backend
        )

        self._last_cmd_time = None
        self._motion_active = False
        self._last_vx = 0.0
        self._last_vyaw = 0.0
        self._shutdown_stop_sent = False
        self._zero_cmd_start_time = None
        self._last_stop_publish_time = None
        self._last_suppressed_stop_log_time = None

        self._subscription = self.create_subscription(
            Twist,
            self.cmd_vel_topic,
            self._on_cmd_vel,
            10
        )

        watchdog_period = max(0.02, min(0.10, self.cmd_timeout_sec / 2.0))
        self._watchdog_timer = self.create_timer(
            watchdog_period,
            self._on_watchdog_timer
        )

        self.add_on_set_parameters_callback(self._on_parameter_update)

        self.get_logger().info(
            'cmd_vel bridge started: '
            f'{self.cmd_vel_topic} -> {self.sport_request_topic}, '
            f'backend={self.backend}, '
            f'max_linear_x={max_linear_x:.3f}, '
            f'max_angular_z={max_angular_z:.3f}, '
            f'zero_cmd_debounce_time={self.zero_cmd_debounce_time:.3f}, '
            f'stop_publish_min_interval={self.stop_publish_min_interval:.3f}'
        )

    def _on_cmd_vel(self, msg):
        now = self.get_clock().now()
        self._last_cmd_time = now

        decision = self._safety_monitor.evaluate(
            msg.linear.x,
            msg.angular.z
        )

        if decision.should_stop:
            self._handle_stop_decision(decision, now)
            return

        self._zero_cmd_start_time = None
        self._last_stop_publish_time = None
        self._motion_client.send_move(decision.vx, decision.vyaw)
        self._motion_active = True
        self._last_vx = decision.vx
        self._last_vyaw = decision.vyaw

    def _handle_stop_decision(self, decision, now):
        if decision.reason == 'zero velocity command':
            self._handle_zero_cmd(decision.reason, now)
            return

        self._zero_cmd_start_time = None
        self._send_stop(decision.reason, now)

    def _handle_zero_cmd(self, reason, now):
        if not self._motion_active:
            self._zero_cmd_start_time = now
            self._log_suppressed_stop(
                'ignore repeated zero velocity command',
                now
            )
            return

        if self._zero_cmd_start_time is None:
            self._zero_cmd_start_time = now

        elapsed_sec = self._elapsed_since(now, self._zero_cmd_start_time)
        if elapsed_sec < self.zero_cmd_debounce_time:
            self._log_suppressed_stop(
                'debounced zero velocity command: '
                f'elapsed={elapsed_sec:.3f}s, '
                f'debounce={self.zero_cmd_debounce_time:.3f}s',
                now
            )
            return

        self._send_stop(reason, now)

    def _send_stop(self, reason, now):
        if not self._motion_active:
            self._log_suppressed_stop(
                f'ignore repeated StopMove: reason="{reason}"',
                now
            )
            return False

        if self._last_stop_publish_time is not None:
            elapsed_sec = self._elapsed_since(
                now,
                self._last_stop_publish_time
            )
            if elapsed_sec < self.stop_publish_min_interval:
                self._log_suppressed_stop(
                    'ignore repeated StopMove: '
                    f'elapsed={elapsed_sec:.3f}s, '
                    f'min_interval={self.stop_publish_min_interval:.3f}s, '
                    f'reason="{reason}"',
                    now
                )
                return False

        self._motion_active = False
        self._last_vx = 0.0
        self._last_vyaw = 0.0
        self._last_stop_publish_time = now
        self._zero_cmd_start_time = None
        self._motion_client.send_stop(reason)
        return True

    def _log_suppressed_stop(self, message, now):
        if self._last_suppressed_stop_log_time is not None:
            elapsed_sec = self._elapsed_since(
                now,
                self._last_suppressed_stop_log_time
            )
            if elapsed_sec < self.stop_publish_min_interval:
                return

        self._last_suppressed_stop_log_time = now
        self.get_logger().info(message)

    def _on_watchdog_timer(self):
        if not self._motion_active or self._last_cmd_time is None:
            return

        now = self.get_clock().now()
        elapsed_sec = self._elapsed_since(now, self._last_cmd_time)

        if elapsed_sec <= self.cmd_timeout_sec:
            return

        self._send_stop(
            f'cmd_vel timeout after {elapsed_sec:.3f}s',
            now
        )

    def _on_parameter_update(self, parameters):
        updates = {}

        for parameter in parameters:
            if parameter.name not in self.DYNAMIC_LIMIT_PARAMS:
                continue

            if not SafetyMonitor.is_positive_finite(parameter.value):
                return SetParametersResult(
                    successful=False,
                    reason=(
                        f'{parameter.name} must be a finite positive number'
                    ),
                )

            updates[parameter.name] = float(parameter.value)

        if not updates:
            return SetParametersResult(successful=True)

        max_linear_x = updates.get(
            'max_linear_x',
            self._safety_monitor.max_linear_x
        )
        max_angular_z = updates.get(
            'max_angular_z',
            self._safety_monitor.max_angular_z
        )
        self._safety_monitor.update_limits(max_linear_x, max_angular_z)

        self.get_logger().info(
            'Updated speed limits: '
            f'max_linear_x={max_linear_x:.3f}, '
            f'max_angular_z={max_angular_z:.3f}'
        )

        if self._motion_active and self._current_motion_exceeds_limits():
            self._send_stop(
                'current command exceeds updated speed limits',
                self.get_clock().now()
            )

        return SetParametersResult(successful=True)

    def _current_motion_exceeds_limits(self):
        return (
            abs(self._last_vx) > self._safety_monitor.max_linear_x
            or abs(self._last_vyaw) > self._safety_monitor.max_angular_z
        )

    @staticmethod
    def _elapsed_since(now, start_time):
        return (now.nanoseconds - start_time.nanoseconds) / 1_000_000_000.0

    def shutdown(self):
        if self._shutdown_stop_sent:
            return

        self._shutdown_stop_sent = True
        self._motion_active = False
        self._motion_client.send_repeated_stop(
            'node shutdown',
            self.stop_publish_count,
            self.stop_publish_period_sec
        )

    def destroy_node(self):
        self.shutdown()
        return super().destroy_node()

    def _string_parameter(self, name):
        value = str(self.get_parameter(name).value)
        if not value:
            raise ValueError(f'{name} must not be empty')
        return value

    def _backend_parameter(self):
        value = self._string_parameter('backend')
        if value not in Go2MotionClient.SUPPORTED_BACKENDS:
            supported = ', '.join(sorted(Go2MotionClient.SUPPORTED_BACKENDS))
            raise ValueError(f'backend must be one of: {supported}')
        return value

    def _positive_float_parameter(self, name):
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f'{name} must be a finite positive number')
        return value

    def _nonnegative_float_parameter(self, name):
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value < 0.0:
            raise ValueError(f'{name} must be a finite nonnegative number')
        return value

    def _positive_int_parameter(self, name):
        value = int(self.get_parameter(name).value)
        if value <= 0:
            raise ValueError(f'{name} must be a positive integer')
        return value


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = CmdVelBridgeNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        if node is not None:
            node.get_logger().warn('Interrupted by user')
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
