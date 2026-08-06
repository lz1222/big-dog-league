#!/usr/bin/env python3
"""T4: 软障碍减速停车诊断 — dry_run, 只读, 不发布运动命令。

狗走向挡板，观测:
  前向距离 → LiDAR速度估计 → 动态停止距离 → 速度分级

速度分级: CLEAR → CAUTION → BRAKE → EMERGENCY → UNKNOWN
"""

import math, time, threading, json
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import String

import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src' / 'rk_maze'))

from rk_maze.lidar_distance_core import (
    Point3D, LidarDistanceConfig, LiDARSpeedEstimator,
    filter_point_cloud, voxel_downsample, find_effective_clusters,
    compute_hard_distance, compute_dynamic_stop_distance, classify_speed,
    SECTOR_FRONT,
    SPEED_CLEAR, SPEED_CAUTION, SPEED_BRAKE, SPEED_EMERGENCY, SPEED_UNKNOWN,
)


SPEED_LABELS = {
    SPEED_CLEAR:    '🟢 CLEAR  ',
    SPEED_CAUTION:  '🟡 CAUTION',
    SPEED_BRAKE:    '🟠 BRAKE  ',
    SPEED_EMERGENCY:'🔴 EMERG  ',
    SPEED_UNKNOWN:  '⚪ UNKNOWN',
}


class T4BrakingDiagnostics(Node):
    """Speed classification diagnostics as robot approaches obstacle."""

    def __init__(self):
        super().__init__('t4_braking_diagnostics')

        self._config = LidarDistanceConfig()
        self._speed_est = LiDARSpeedEstimator(min_samples=4, max_speed_m_s=0.80)
        self._lock = threading.Lock()
        self._cloud = []
        self._frame = 0

        self.create_subscription(
            PointCloud2, '/utlidar/cloud_base', self._on_cloud,
            qos_profile_sensor_data,
        )
        self.create_timer(0.10, self._tick)

        self.get_logger().info(
            'T4 braking diagnostics ready — approach the board!'
        )

    def _on_cloud(self, msg):
        pts = list(pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True))
        with self._lock:
            self._cloud = [
                Point3D(x=float(p[0]), y=float(p[1]), z=float(p[2]))
                for p in pts if all(math.isfinite(float(v)) for v in p)
            ]

    def _tick(self):
        with self._lock:
            cloud = self._cloud
            self._cloud = []

        if not cloud:
            return

        now = time.time()
        self._frame += 1

        # Filter + cluster
        filt = filter_point_cloud(cloud, self._config)
        filt = voxel_downsample(filt, self._config.voxel_size_m)

        front_pts = [p for p in filt
                     if abs(math.degrees(math.atan2(p.y, p.x))) <= 15
                     and p.z > 0.01]
        if not front_pts:
            return

        # Hard distance
        sd = compute_hard_distance(front_pts, SECTOR_FRONT, self._config, now, now)
        if not sd.valid:
            return

        hard_front = sd.hard_distance
        front_clearance = sd.front_clearance

        # Speed estimation
        self._speed_est.update(now, hard_front)
        speed = self._speed_est.speed(now)

        # Dynamic stop distance
        dyn_stop = compute_dynamic_stop_distance(speed, self._config)

        # Speed classification
        klass = classify_speed(hard_front, hard_front, dyn_stop, self._config)
        remaining = front_clearance - dyn_stop if front_clearance > 0 else -dyn_stop

        # Print every frame
        bar = self._make_bar(front_clearance, dyn_stop)
        label = SPEED_LABELS.get(klass, klass)
        self.get_logger().info(
            f'{label} | front={front_clearance:.2f}m '
            f'stop={dyn_stop:.2f}m '
            f'spd={speed:+.3f}m/s '
            f'rem={remaining:+.2f}m {bar}'
        )

    def _make_bar(self, clearance, stop_dist):
        """Visual bar: ===|===||| → approaching stop zone."""
        total_width = max(clearance, stop_dist, 0.10)
        stop_pos = int(stop_dist / total_width * 20) if total_width > 0 else 20
        clr_pos = int(clearance / total_width * 20) if total_width > 0 else 0
        bar = ''
        for i in range(20):
            if i < clr_pos: bar += '='
            elif i < stop_pos: bar += '-'
            else: bar += '|'
        return f'[{bar}]'


def main():
    rclpy.init()
    node = T4BrakingDiagnostics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
