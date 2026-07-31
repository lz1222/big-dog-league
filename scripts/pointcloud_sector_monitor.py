#!/usr/bin/env python3

"""B0 点云五区域监视器，只读取传感器并输出诊断日志。"""

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

from maze_perception_core import (
    SECTOR_NAMES,
    SectorExtractor,
)


class PointCloudSectorMonitor(Node):
    """输出稳健障碍距离，不创建任何运动命令发布器。"""

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
        self.diagonal_angle_max = self._finite_float_parameter(
            'diagonal_angle_max', 67.5
        )
        self.side_angle_max = self._finite_float_parameter(
            'side_angle_max', 112.5
        )
        self.side_projection_angle_min = self._finite_float_parameter(
            'side_projection_angle_min', 15.0
        )
        self.side_projection_angle_max = self._finite_float_parameter(
            'side_projection_angle_max', 60.0
        )
        self.side_projection_x_min = self._finite_float_parameter(
            'side_projection_x_min', 0.45
        )
        self.side_projection_x_max = self._finite_float_parameter(
            'side_projection_x_max', 1.50
        )
        self.side_projection_min_x_span = (
            self._positive_float_parameter(
                'side_projection_min_x_span', 0.12
            )
        )
        self.side_projection_lateral_tolerance = (
            self._positive_float_parameter(
                'side_projection_lateral_tolerance', 0.04
            )
        )
        self.side_min_points = self._positive_int_parameter(
            'side_min_points', 3
        )
        self.stale_timeout = self._positive_float_parameter(
            'stale_timeout', 0.50
        )
        self.print_rate = self._positive_float_parameter('print_rate', 2.0)

        # B0 与 B1 共用同一提取器，避免标定工具和决策输入定义漂移。
        self.extractor = SectorExtractor(
            z_min=self.z_min,
            z_max=self.z_max,
            body_x_min=self.body_x_min,
            body_x_max=self.body_x_max,
            body_y_min=self.body_y_min,
            body_y_max=self.body_y_max,
            front_angle_deg=self.front_angle_deg,
            min_range=self.min_range,
            front_max_range=self.front_max_range,
            side_max_range=self.side_max_range,
            distance_percentile=self.distance_percentile,
            diagonal_angle_max_deg=self.diagonal_angle_max,
            side_angle_max_deg=self.side_angle_max,
            side_projection_angle_min_deg=(
                self.side_projection_angle_min
            ),
            side_projection_angle_max_deg=(
                self.side_projection_angle_max
            ),
            side_projection_x_min=self.side_projection_x_min,
            side_projection_x_max=self.side_projection_x_max,
            side_projection_min_x_span=(
                self.side_projection_min_x_span
            ),
            side_projection_lateral_tolerance=(
                self.side_projection_lateral_tolerance
            ),
            side_min_points=self.side_min_points,
        )

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
        self._last_sector_sources = {
            name: 'none'
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

        try:
            points = point_cloud2.read_points(
                msg,
                field_names=('x', 'y', 'z'),
                skip_nans=False,
            )
            result = self.extractor.extract(points)
        except (IndexError, TypeError, ValueError) as error:
            text = f'failed to read PointCloud2: {error}'
            with self._lock:
                self._last_error = text
            self.get_logger().error(text)
            return

        with self._lock:
            self._last_valid_points = result['valid_points']
            self._last_distances = result['distances']
            self._last_sector_counts = result['counts']
            self._last_sector_sources = result['sources']
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
            sources = dict(self._last_sector_sources)
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
                f'(n={counts[name]},src={sources[name]})'
                for name in SECTOR_NAMES
            )
        )
        if last_error:
            text += f' error="{last_error}"'

        if state == 'STALE' or last_error:
            self.get_logger().warn(text)
        else:
            self.get_logger().info(text)

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

    def _positive_int_parameter(self, name, default):
        """声明并读取正整数参数。"""
        value = int(self.declare_parameter(name, default).value)
        if value <= 0:
            raise ValueError(f'{name} must be positive')
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
