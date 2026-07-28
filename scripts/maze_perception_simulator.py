#!/usr/bin/env python3

import math
import time

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
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


SCENARIOS = (
    (
        'clear',
        {
            'front': 2.20,
            'left_front': 1.60,
            'right_front': 1.60,
            'left': 1.20,
            'right': 1.20,
        },
        True,
        True,
    ),
    (
        'blocked_choose_left',
        {
            'front': 0.55,
            'left_front': 1.40,
            'right_front': 0.40,
            'left': 1.40,
            'right': 0.50,
        },
        True,
        True,
    ),
    (
        'hysteresis_band',
        {
            'front': 0.72,
            'left_front': 1.40,
            'right_front': 1.20,
            'left': 1.40,
            'right': 1.20,
        },
        True,
        True,
    ),
    (
        'clear_after_block',
        {
            'front': 2.20,
            'left_front': 1.60,
            'right_front': 1.60,
            'left': 1.20,
            'right': 1.20,
        },
        True,
        True,
    ),
    (
        'blocked_choose_right',
        {
            'front': 0.55,
            'left_front': 0.40,
            'right_front': 1.40,
            'left': 0.50,
            'right': 1.40,
        },
        True,
        True,
    ),
    (
        'boxed_stop',
        {
            'front': 0.55,
            'left_front': 0.40,
            'right_front': 0.40,
            'left': 0.40,
            'right': 0.40,
        },
        True,
        True,
    ),
    (
        'cloud_stale',
        {},
        False,
        True,
    ),
    (
        'odom_stale',
        {
            'front': 2.20,
            'left_front': 1.60,
            'right_front': 1.60,
            'left': 1.20,
            'right': 1.20,
        },
        True,
        False,
    ),
)

SECTOR_ANGLES_DEG = {
    'front': 0.0,
    'left_front': 45.0,
    'right_front': -45.0,
    'left': 90.0,
    'right': -90.0,
}


class MazePerceptionSimulator(Node):
    def __init__(self):
        super().__init__('maze_perception_simulator')

        self.cloud_topic = self._string_parameter(
            'cloud_topic', '/maze_sim/cloud'
        )
        self.odom_topic = self._string_parameter(
            'odom_topic', '/maze_sim/odom'
        )
        self.publish_rate = self._positive_float_parameter(
            'publish_rate', 15.0
        )
        self.scenario_duration = self._positive_float_parameter(
            'scenario_duration', 4.0
        )
        self.cloud_frame = self._string_parameter(
            'cloud_frame', 'base_link'
        )
        self.odom_frame = self._string_parameter(
            'odom_frame', 'odom'
        )
        self.child_frame = self._string_parameter(
            'child_frame', 'base_link'
        )

        sensor_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.cloud_publisher = self.create_publisher(
            PointCloud2,
            self.cloud_topic,
            sensor_qos,
        )
        self.odom_publisher = self.create_publisher(
            Odometry,
            self.odom_topic,
            sensor_qos,
        )
        self.timer = self.create_timer(
            1.0 / self.publish_rate,
            self._on_timer,
        )
        self._start_time = time.monotonic()
        self._last_scenario_name = ''

        self.get_logger().info(
            'Maze perception simulator ready: '
            f'cloud={self.cloud_topic}, odom={self.odom_topic}; '
            'sensor messages only'
        )

    def _on_timer(self):
        elapsed = time.monotonic() - self._start_time
        scenario_index = int(
            elapsed / self.scenario_duration
        ) % len(SCENARIOS)
        name, distances, publish_cloud, publish_odom = (
            SCENARIOS[scenario_index]
        )
        if name != self._last_scenario_name:
            self._last_scenario_name = name
            self.get_logger().info(f'scenario={name}')

        stamp = self.get_clock().now().to_msg()
        if publish_cloud:
            header = Header()
            header.stamp = stamp
            header.frame_id = self.cloud_frame
            points = self._scenario_points(distances)
            cloud = point_cloud2.create_cloud_xyz32(header, points)
            self.cloud_publisher.publish(cloud)

        if publish_odom:
            odom = Odometry()
            odom.header.stamp = stamp
            odom.header.frame_id = self.odom_frame
            odom.child_frame_id = self.child_frame
            odom.pose.pose.orientation.w = 1.0
            self.odom_publisher.publish(odom)

    @staticmethod
    def _scenario_points(distances):
        points = []
        angle_offsets_deg = (-2.0, -1.0, 0.0, 1.0, 2.0)
        distance_scales = (1.01, 1.005, 1.0, 0.995, 0.99)
        for sector, distance in distances.items():
            center_angle = SECTOR_ANGLES_DEG[sector]
            for angle_offset, distance_scale in zip(
                angle_offsets_deg,
                distance_scales,
            ):
                angle = math.radians(center_angle + angle_offset)
                point_distance = float(distance) * distance_scale
                points.append(
                    (
                        point_distance * math.cos(angle),
                        point_distance * math.sin(angle),
                        0.20,
                    )
                )
        return points

    def _string_parameter(self, name, default):
        value = str(self.declare_parameter(name, default).value)
        if not value:
            raise ValueError(f'{name} must not be empty')
        return value

    def _positive_float_parameter(self, name, default):
        value = float(self.declare_parameter(name, default).value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f'{name} must be positive and finite')
        return value


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MazePerceptionSimulator()
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
