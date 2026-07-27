#!/usr/bin/env python3

import math
import threading
import time
from collections import deque

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)


class OdomYawMonitor(Node):
    """Monitor wrapped and accumulated yaw without commanding the robot."""

    def __init__(self):
        super().__init__('odom_yaw_monitor')

        self.odom_topic = self._string_parameter(
            'odom_topic', '/utlidar/robot_odom'
        )
        self.stale_timeout = self._positive_float_parameter(
            'stale_timeout', 0.50
        )
        self.print_rate = self._positive_float_parameter('print_rate', 2.0)
        self.stationary_linear_speed_threshold = (
            self._nonnegative_float_parameter(
                'stationary_linear_speed_threshold',
                0.02,
            )
        )
        self.stationary_angular_speed_threshold = (
            self._nonnegative_float_parameter(
                'stationary_angular_speed_threshold',
                0.02,
            )
        )
        self.stationary_min_duration = self._nonnegative_float_parameter(
            'stationary_min_duration', 2.0
        )

        self._lock = threading.Lock()
        self._receive_times = deque(maxlen=200)
        self._last_message_time = None
        self._last_frame_id = ''
        self._last_child_frame_id = ''
        self._last_roll = None
        self._last_pitch = None
        self._last_yaw = None
        self._initial_yaw = None
        self._previous_yaw = None
        self._accumulated_yaw = 0.0
        self._linear_speed = 0.0
        self._angular_speed = 0.0
        self._last_error = ''

        self._stationary_start_time = None
        self._stationary_start_yaw = 0.0
        self._stationary_drift_min = 0.0
        self._stationary_drift_max = 0.0
        self._stationary_sample_count = 0

        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.subscription = self.create_subscription(
            Odometry,
            self.odom_topic,
            self._on_odom,
            qos,
        )
        self.timer = self.create_timer(
            1.0 / self.print_rate,
            self._on_print_timer,
        )

        self.get_logger().info(
            'Odometry yaw monitor ready: '
            f'topic={self.odom_topic}, '
            'waiting for initial orientation'
        )

    def _on_odom(self, msg):
        receive_time = time.monotonic()
        orientation = msg.pose.pose.orientation

        try:
            roll, pitch, yaw = self._quaternion_to_rpy(
                float(orientation.x),
                float(orientation.y),
                float(orientation.z),
                float(orientation.w),
            )
        except ValueError as error:
            with self._lock:
                self._last_message_time = receive_time
                self._receive_times.append(receive_time)
                self._last_error = str(error)
            return

        linear = msg.twist.twist.linear
        angular = msg.twist.twist.angular
        linear_speed = math.sqrt(
            float(linear.x) ** 2
            + float(linear.y) ** 2
            + float(linear.z) ** 2
        )
        angular_speed = math.sqrt(
            float(angular.x) ** 2
            + float(angular.y) ** 2
            + float(angular.z) ** 2
        )

        with self._lock:
            if self._previous_yaw is None:
                self._initial_yaw = yaw
                self._previous_yaw = yaw
                self._accumulated_yaw = 0.0
                self.get_logger().info(
                    'Initial yaw captured: '
                    f'{yaw:.6f}rad ({math.degrees(yaw):.3f}deg)'
                )
            else:
                delta_yaw = self._normalize_angle(
                    yaw - self._previous_yaw
                )
                self._accumulated_yaw += delta_yaw
                self._previous_yaw = yaw

            self._last_message_time = receive_time
            self._last_frame_id = str(msg.header.frame_id)
            self._last_child_frame_id = str(msg.child_frame_id)
            self._last_roll = roll
            self._last_pitch = pitch
            self._last_yaw = yaw
            self._linear_speed = linear_speed
            self._angular_speed = angular_speed
            self._last_error = ''
            self._receive_times.append(receive_time)
            self._update_stationary_state(
                receive_time,
                linear_speed,
                angular_speed,
            )

    def _update_stationary_state(
        self,
        receive_time,
        linear_speed,
        angular_speed,
    ):
        stationary = (
            linear_speed <= self.stationary_linear_speed_threshold
            and angular_speed <= self.stationary_angular_speed_threshold
        )

        if not stationary:
            self._reset_stationary_state()
            return

        if self._stationary_start_time is None:
            self._stationary_start_time = receive_time
            self._stationary_start_yaw = self._accumulated_yaw
            self._stationary_drift_min = 0.0
            self._stationary_drift_max = 0.0
            self._stationary_sample_count = 1
            return

        drift = self._accumulated_yaw - self._stationary_start_yaw
        self._stationary_drift_min = min(
            self._stationary_drift_min,
            drift,
        )
        self._stationary_drift_max = max(
            self._stationary_drift_max,
            drift,
        )
        self._stationary_sample_count += 1

    def _reset_stationary_state(self):
        self._stationary_start_time = None
        self._stationary_start_yaw = self._accumulated_yaw
        self._stationary_drift_min = 0.0
        self._stationary_drift_max = 0.0
        self._stationary_sample_count = 0

    def _on_print_timer(self):
        now = time.monotonic()
        with self._lock:
            last_message_time = self._last_message_time
            frame_id = self._last_frame_id
            child_frame_id = self._last_child_frame_id
            roll = self._last_roll
            pitch = self._last_pitch
            yaw = self._last_yaw
            accumulated_yaw = self._accumulated_yaw
            linear_speed = self._linear_speed
            angular_speed = self._angular_speed
            receive_times = list(self._receive_times)
            last_error = self._last_error
            stationary_start_time = self._stationary_start_time
            stationary_start_yaw = self._stationary_start_yaw
            stationary_drift_min = self._stationary_drift_min
            stationary_drift_max = self._stationary_drift_max
            stationary_sample_count = self._stationary_sample_count

        if last_message_time is None:
            self.get_logger().warn(
                f'STALE odom: no message received on {self.odom_topic}'
            )
            return

        age_sec = now - last_message_time
        frequency_hz = self._frequency_hz(receive_times)
        state = 'FRESH' if age_sec <= self.stale_timeout else 'STALE'

        if roll is None or pitch is None or yaw is None:
            text = (
                f'{state} odom age={age_sec:.3f}s '
                f'hz={self._format_frequency(frequency_hz)} '
                f'error="{last_error or "no valid orientation"}"'
            )
            self.get_logger().warn(text)
            return

        text = (
            f'{state} odom age={age_sec:.3f}s '
            f'hz={self._format_frequency(frequency_hz)} '
            f'frame={frame_id or "<empty>"} '
            f'child={child_frame_id or "<empty>"} '
            f'roll={roll:.6f}rad/{math.degrees(roll):.3f}deg '
            f'pitch={pitch:.6f}rad/{math.degrees(pitch):.3f}deg '
            f'yaw={yaw:.6f}rad/{math.degrees(yaw):.3f}deg '
            f'turn={accumulated_yaw:.6f}rad/'
            f'{math.degrees(accumulated_yaw):.3f}deg '
            f'linear_speed={linear_speed:.4f}m/s '
            f'angular_speed={angular_speed:.4f}rad/s'
        )

        if stationary_start_time is None:
            text += ' stationary=no'
        else:
            stationary_duration = now - stationary_start_time
            drift = accumulated_yaw - stationary_start_yaw
            drift_span = stationary_drift_max - stationary_drift_min
            drift_rate = (
                drift / stationary_duration
                if stationary_duration > 0.0
                else 0.0
            )
            stationary_ready = (
                stationary_duration >= self.stationary_min_duration
            )
            text += (
                f' stationary={"yes" if stationary_ready else "settling"}'
                f' duration={stationary_duration:.1f}s'
                f' samples={stationary_sample_count}'
                f' drift={drift:.6f}rad/{math.degrees(drift):.3f}deg'
                f' drift_span={drift_span:.6f}rad/'
                f'{math.degrees(drift_span):.3f}deg'
                f' drift_rate={math.degrees(drift_rate) * 60.0:.3f}deg/min'
            )

        if state == 'STALE' or last_error:
            if last_error:
                text += f' error="{last_error}"'
            self.get_logger().warn(text)
        else:
            self.get_logger().info(text)

    @staticmethod
    def _quaternion_to_rpy(x, y, z, w):
        if not all(math.isfinite(value) for value in (x, y, z, w)):
            raise ValueError('odometry quaternion contains NaN or Inf')

        norm = math.sqrt(x * x + y * y + z * z + w * w)
        if norm <= 1.0e-12:
            raise ValueError('odometry quaternion has zero norm')

        x /= norm
        y /= norm
        z /= norm
        w /= norm

        sin_roll_cos_pitch = 2.0 * (w * x + y * z)
        cos_roll_cos_pitch = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sin_roll_cos_pitch, cos_roll_cos_pitch)

        sin_pitch = 2.0 * (w * y - z * x)
        sin_pitch = max(-1.0, min(1.0, sin_pitch))
        pitch = math.asin(sin_pitch)

        sin_yaw_cos_pitch = 2.0 * (w * z + x * y)
        cos_yaw_cos_pitch = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(sin_yaw_cos_pitch, cos_yaw_cos_pitch)
        return roll, pitch, yaw

    @staticmethod
    def _normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def _frequency_hz(receive_times):
        if len(receive_times) < 2:
            return None
        elapsed = receive_times[-1] - receive_times[0]
        if elapsed <= 0.0:
            return None
        return (len(receive_times) - 1) / elapsed

    @staticmethod
    def _format_frequency(frequency_hz):
        if frequency_hz is None:
            return 'n/a'
        return f'{frequency_hz:.2f}'

    def _string_parameter(self, name, default):
        value = str(self.declare_parameter(name, default).value)
        if not value:
            raise ValueError(f'{name} must not be empty')
        return value

    def _finite_float_parameter(self, name, default):
        value = float(self.declare_parameter(name, default).value)
        if not math.isfinite(value):
            raise ValueError(f'{name} must be finite')
        return value

    def _positive_float_parameter(self, name, default):
        value = self._finite_float_parameter(name, default)
        if value <= 0.0:
            raise ValueError(f'{name} must be positive')
        return value

    def _nonnegative_float_parameter(self, name, default):
        value = self._finite_float_parameter(name, default)
        if value < 0.0:
            raise ValueError(f'{name} must be nonnegative')
        return value


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = OdomYawMonitor()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
