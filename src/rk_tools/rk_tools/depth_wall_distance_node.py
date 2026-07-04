#!/usr/bin/env python3

import math
import struct
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image


class DepthWallDistanceNode(Node):
    """Print D435i depth distances for front-wall alignment tests."""

    SUPPORTED_ENCODINGS = {'16UC1', 'MONO16', '32FC1'}

    def __init__(self):
        super().__init__('depth_wall_distance_node')

        self.depth_image_topic = self.declare_parameter(
            'depth_image_topic',
            '/camera/camera/depth/image_rect_raw'
        ).value
        self.print_rate_hz = self.float_parameter('print_rate_hz', 2.0)
        self.sample_step_px = self.int_parameter('sample_step_px', 8)
        self.min_valid_m = self.float_parameter('min_valid_m', 0.08)
        self.max_valid_m = self.float_parameter('max_valid_m', 3.00)
        self.percentile = self.ratio_parameter('percentile', 0.20)
        self.roi_top_ratio = self.ratio_parameter('roi_top_ratio', 0.30)
        self.roi_bottom_ratio = self.ratio_parameter(
            'roi_bottom_ratio',
            0.72
        )
        self.center_left_ratio = self.ratio_parameter(
            'center_left_ratio',
            0.35
        )
        self.center_right_ratio = self.ratio_parameter(
            'center_right_ratio',
            0.65
        )
        self.stale_warn_sec = self.float_parameter('stale_warn_sec', 1.0)

        self.validate_parameters()

        self.last_distances = None
        self.last_image_time = None
        self.last_encoding_warning = None

        self.subscription = self.create_subscription(
            Image,
            self.depth_image_topic,
            self.on_depth_image,
            10
        )
        self.timer = self.create_timer(
            1.0 / self.print_rate_hz,
            self.on_timer
        )

        self.get_logger().info(
            'Depth wall distance test started: '
            f'topic={self.depth_image_topic}, '
            f'roi_y={self.roi_top_ratio:.2f}-{self.roi_bottom_ratio:.2f}, '
            f'center_x={self.center_left_ratio:.2f}-'
            f'{self.center_right_ratio:.2f}, '
            f'percentile={self.percentile:.2f}'
        )

    def validate_parameters(self):
        if not self.depth_image_topic:
            raise ValueError('depth_image_topic must not be empty')
        if self.print_rate_hz <= 0.0:
            raise ValueError('print_rate_hz must be positive')
        if self.sample_step_px <= 0:
            raise ValueError('sample_step_px must be positive')
        if self.max_valid_m <= self.min_valid_m:
            raise ValueError('max_valid_m must be greater than min_valid_m')
        if self.roi_bottom_ratio <= self.roi_top_ratio:
            raise ValueError(
                'roi_bottom_ratio must be greater than roi_top_ratio'
            )
        if self.center_right_ratio <= self.center_left_ratio:
            raise ValueError(
                'center_right_ratio must be greater than center_left_ratio'
            )

    def on_depth_image(self, msg):
        distances = self.extract_distances(msg)
        if distances is None:
            return

        self.last_distances = distances
        self.last_image_time = time.monotonic()

    def on_timer(self):
        if self.last_distances is None or self.last_image_time is None:
            self.get_logger().warn(
                f'No depth image received yet on {self.depth_image_topic}'
            )
            return

        age_sec = time.monotonic() - self.last_image_time
        if age_sec > self.stale_warn_sec:
            self.get_logger().warn(
                f'Depth image is stale: age={age_sec:.2f}s'
            )

        text = (
            'depth distance m: '
            f'center={self.format_distance(self.last_distances["center"])}, '
            f'left={self.format_distance(self.last_distances["left"])}, '
            f'right={self.format_distance(self.last_distances["right"])}, '
            f'full={self.format_distance(self.last_distances["full"])}, '
            f'valid_center={self.last_distances["center_count"]}'
        )
        self.get_logger().info(text)

    def extract_distances(self, msg):
        encoding = str(msg.encoding or '').upper()
        if encoding not in self.SUPPORTED_ENCODINGS:
            if encoding != self.last_encoding_warning:
                self.get_logger().warn(
                    f'Unsupported depth encoding: {msg.encoding}'
                )
                self.last_encoding_warning = encoding
            return None

        if msg.width <= 0 or msg.height <= 0 or msg.step <= 0:
            return None

        row_start = int(msg.height * self.roi_top_ratio)
        row_stop = int(msg.height * self.roi_bottom_ratio)
        row_start = max(0, min(msg.height - 1, row_start))
        row_stop = max(row_start + 1, min(msg.height, row_stop))

        ranges = {
            'left': (0.05, self.center_left_ratio),
            'center': (self.center_left_ratio, self.center_right_ratio),
            'right': (self.center_right_ratio, 0.95),
            'full': (0.05, 0.95),
        }

        result = {}
        for name, (left_ratio, right_ratio) in ranges.items():
            values = self.collect_roi_values(
                msg,
                encoding,
                row_start,
                row_stop,
                left_ratio,
                right_ratio
            )
            result[name] = self.percentile_value(values, self.percentile)
            result[f'{name}_count'] = len(values)
        return result

    def collect_roi_values(
        self,
        msg,
        encoding,
        row_start,
        row_stop,
        left_ratio,
        right_ratio
    ):
        col_start = int(msg.width * left_ratio)
        col_stop = int(msg.width * right_ratio)
        col_start = max(0, min(msg.width - 1, col_start))
        col_stop = max(col_start + 1, min(msg.width, col_stop))

        values = []
        for row in range(row_start, row_stop, self.sample_step_px):
            for col in range(col_start, col_stop, self.sample_step_px):
                distance = self.read_depth_m(msg, encoding, row, col)
                if self.is_valid_distance(distance):
                    values.append(distance)
        return values

    def read_depth_m(self, msg, encoding, row, col):
        if encoding in ('16UC1', 'MONO16'):
            offset = row * msg.step + col * 2
            if offset + 2 > len(msg.data):
                return None
            fmt = '>H' if msg.is_bigendian else '<H'
            value = struct.unpack_from(fmt, msg.data, offset)[0]
            if value == 0:
                return None
            return value / 1000.0

        offset = row * msg.step + col * 4
        if offset + 4 > len(msg.data):
            return None
        fmt = '>f' if msg.is_bigendian else '<f'
        return struct.unpack_from(fmt, msg.data, offset)[0]

    def is_valid_distance(self, distance):
        return (
            distance is not None
            and math.isfinite(distance)
            and self.min_valid_m <= distance <= self.max_valid_m
        )

    @staticmethod
    def percentile_value(values, percentile):
        if not values:
            return None
        ordered = sorted(values)
        index = int((len(ordered) - 1) * float(percentile))
        index = max(0, min(len(ordered) - 1, index))
        return ordered[index]

    @staticmethod
    def format_distance(distance):
        if distance is None:
            return 'none'
        return f'{distance:.3f}'

    def float_parameter(self, name, default):
        value = float(self.declare_parameter(name, default).value)
        if not math.isfinite(value):
            raise ValueError(f'{name} must be finite')
        return value

    def int_parameter(self, name, default):
        return int(self.declare_parameter(name, default).value)

    def ratio_parameter(self, name, default):
        value = self.float_parameter(name, default)
        if value < 0.0 or value > 1.0:
            raise ValueError(f'{name} must be in [0.0, 1.0]')
        return value


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = DepthWallDistanceNode()
        rclpy.spin(node)
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
