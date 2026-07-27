#!/usr/bin/env python3

import math
import threading
import time
from collections import deque

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


SECTOR_NAMES = (
    'front',
    'left_front',
    'right_front',
    'left',
    'right',
)


class PointCloudSectorMonitor(Node):
    """Report robust obstacle distances without publishing motion commands."""

    def __init__(self):
        super().__init__('pointcloud_sector_monitor')

        self.cloud_topic = self._string_parameter(
            'cloud_topic', '/utlidar/cloud_base'
        )
        self.z_min = self._finite_float_parameter('z_min', -0.15)
        self.z_max = self._finite_float_parameter('z_max', 0.50)
        self.body_x_min = self._finite_float_parameter(
            'body_x_min', -0.45
        )
        self.body_x_max = self._finite_float_parameter(
            'body_x_max', 0.45
        )
        self.body_y_min = self._finite_float_parameter(
            'body_y_min', -0.25
        )
        self.body_y_max = self._finite_float_parameter(
            'body_y_max', 0.25
        )
        self.front_angle_deg = self._finite_float_parameter(
            'front_angle', 45.0
        )
        self.front_max_range = self._positive_float_parameter(
            'front_max_range', 3.0
        )
        self.side_max_range = self._positive_float_parameter(
            'side_max_range', 2.0
        )
        self.min_range = self._nonnegative_float_parameter(
            'min_range', 0.05
        )
        self.distance_percentile = self._finite_float_parameter(
            'distance_percentile', 10.0
        )
        self.stale_timeout = self._positive_float_parameter(
            'stale_timeout', 0.50
        )
        self.print_rate = self._positive_float_parameter('print_rate', 2.0)

        self._validate_parameters()
        self._front_angle_rad = math.radians(self.front_angle_deg)

        self._lock = threading.Lock()
        self._receive_times = deque(maxlen=100)
        self._last_message_time = None
        self._last_frame_id = ''
        self._last_total_points = 0
        self._last_valid_points = 0
        self._last_distances = {
            name: None
            for name in SECTOR_NAMES
        }
        self._last_sector_counts = {
            name: 0
            for name in SECTOR_NAMES
        }
        self._last_error = ''
        self._missing_fields_reported = False

        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.subscription = self.create_subscription(
            PointCloud2,
            self.cloud_topic,
            self._on_cloud,
            qos,
        )
        self.timer = self.create_timer(
            1.0 / self.print_rate,
            self._on_print_timer,
        )

        self.get_logger().info(
            'PointCloud sector monitor ready: '
            f'topic={self.cloud_topic}, '
            f'z=[{self.z_min:.3f}, {self.z_max:.3f}]m, '
            f'front_angle={self.front_angle_deg:.1f}deg, '
            f'percentile={self.distance_percentile:.1f}'
        )

    def _on_cloud(self, msg):
        receive_time = time.monotonic()
        field_names = {field.name for field in msg.fields}
        missing_fields = {'x', 'y', 'z'} - field_names

        with self._lock:
            self._last_message_time = receive_time
            self._last_frame_id = str(msg.header.frame_id)
            self._last_total_points = int(msg.width) * int(msg.height)
            self._receive_times.append(receive_time)

        if missing_fields:
            error = (
                'PointCloud2 is missing required fields: '
                + ', '.join(sorted(missing_fields))
            )
            with self._lock:
                self._last_error = error
            if not self._missing_fields_reported:
                self.get_logger().error(error)
                self._missing_fields_reported = True
            return

        sector_values = {
            name: []
            for name in SECTOR_NAMES
        }

        try:
            points = point_cloud2.read_points(
                msg,
                field_names=('x', 'y', 'z'),
                skip_nans=False,
            )
            for point in points:
                x = float(point[0])
                y = float(point[1])
                z = float(point[2])

                if not (
                    math.isfinite(x)
                    and math.isfinite(y)
                    and math.isfinite(z)
                ):
                    continue
                if z < self.z_min or z > self.z_max:
                    continue
                if self._inside_body_filter(x, y):
                    continue

                distance = math.hypot(x, y)
                if distance < self.min_range:
                    continue

                sector = self._classify_sector(math.atan2(y, x))
                if sector is None:
                    continue

                max_range = (
                    self.side_max_range
                    if sector in ('left', 'right')
                    else self.front_max_range
                )
                if distance > max_range:
                    continue
                sector_values[sector].append(distance)
        except (IndexError, TypeError, ValueError) as error:
            text = f'failed to read PointCloud2: {error}'
            with self._lock:
                self._last_error = text
            self.get_logger().error(text)
            return

        distances = {
            name: self._percentile(
                sector_values[name],
                self.distance_percentile,
            )
            for name in SECTOR_NAMES
        }
        counts = {
            name: len(sector_values[name])
            for name in SECTOR_NAMES
        }

        with self._lock:
            self._last_valid_points = sum(counts.values())
            self._last_distances = distances
            self._last_sector_counts = counts
            self._last_error = ''
            self._missing_fields_reported = False

    def _on_print_timer(self):
        now = time.monotonic()
        with self._lock:
            last_message_time = self._last_message_time
            frame_id = self._last_frame_id
            total_points = self._last_total_points
            valid_points = self._last_valid_points
            distances = dict(self._last_distances)
            counts = dict(self._last_sector_counts)
            receive_times = list(self._receive_times)
            last_error = self._last_error

        if last_message_time is None:
            self.get_logger().warn(
                f'STALE cloud: no message received on {self.cloud_topic}'
            )
            return

        age_sec = now - last_message_time
        frequency_hz = self._frequency_hz(receive_times)
        state = 'FRESH' if age_sec <= self.stale_timeout else 'STALE'
        text = (
            f'{state} cloud age={age_sec:.3f}s '
            f'hz={self._format_frequency(frequency_hz)} '
            f'frame={frame_id or "<empty>"} '
            f'valid_points={valid_points}/{total_points} '
            + ' '.join(
                f'{name}={self._format_distance(distances[name])}'
                f'(n={counts[name]})'
                for name in SECTOR_NAMES
            )
        )
        if last_error:
            text += f' error="{last_error}"'

        if state == 'STALE' or last_error:
            self.get_logger().warn(text)
        else:
            self.get_logger().info(text)

    def _inside_body_filter(self, x, y):
        return (
            self.body_x_min <= x <= self.body_x_max
            and self.body_y_min <= y <= self.body_y_max
        )

    def _classify_sector(self, angle):
        half_width = 0.5 * self._front_angle_rad
        front_side_boundary = 1.5 * self._front_angle_rad
        side_rear_boundary = 2.5 * self._front_angle_rad

        if -half_width <= angle <= half_width:
            return 'front'
        if half_width < angle <= front_side_boundary:
            return 'left_front'
        if -front_side_boundary <= angle < -half_width:
            return 'right_front'
        if front_side_boundary < angle <= side_rear_boundary:
            return 'left'
        if -side_rear_boundary <= angle < -front_side_boundary:
            return 'right'
        return None

    @staticmethod
    def _percentile(values, percentile):
        if not values:
            return None
        ordered = sorted(values)
        rank = (len(ordered) - 1) * float(percentile) / 100.0
        lower_index = int(math.floor(rank))
        upper_index = int(math.ceil(rank))
        if lower_index == upper_index:
            return ordered[lower_index]
        fraction = rank - lower_index
        return (
            ordered[lower_index] * (1.0 - fraction)
            + ordered[upper_index] * fraction
        )

    @staticmethod
    def _frequency_hz(receive_times):
        if len(receive_times) < 2:
            return None
        elapsed = receive_times[-1] - receive_times[0]
        if elapsed <= 0.0:
            return None
        return (len(receive_times) - 1) / elapsed

    @staticmethod
    def _format_distance(distance):
        if distance is None:
            return 'n/a'
        return f'{distance:.3f}m'

    @staticmethod
    def _format_frequency(frequency_hz):
        if frequency_hz is None:
            return 'n/a'
        return f'{frequency_hz:.2f}'

    def _validate_parameters(self):
        if self.z_max <= self.z_min:
            raise ValueError('z_max must be greater than z_min')
        if self.body_x_max <= self.body_x_min:
            raise ValueError('body_x_max must be greater than body_x_min')
        if self.body_y_max <= self.body_y_min:
            raise ValueError('body_y_max must be greater than body_y_min')
        if not 0.0 < self.front_angle_deg <= 72.0:
            raise ValueError('front_angle must be in (0, 72] degrees')
        if self.front_max_range <= self.min_range:
            raise ValueError('front_max_range must be greater than min_range')
        if self.side_max_range <= self.min_range:
            raise ValueError('side_max_range must be greater than min_range')
        if not 0.0 < self.distance_percentile <= 100.0:
            raise ValueError('distance_percentile must be in (0, 100]')

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
        node = PointCloudSectorMonitor()
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
