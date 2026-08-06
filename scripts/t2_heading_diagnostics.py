#!/usr/bin/env python3
"""T2: 连续直行航向闭环诊断 — dry_run, 只读, 不发布运动命令。

观测 wall_based 航向闭环全链路:
  点云 → 过滤 → 网格 → 墙线 → 走廊航向 → heading_error → wz_reference
"""

import math, time, threading, json
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, Imu
from sensor_msgs_py import point_cloud2 as pc2
from nav_msgs.msg import Odometry
from std_msgs.msg import String

import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src' / 'rk_maze'))

from rk_maze.lidar_distance_core import (
    Point3D, LidarDistanceConfig, filter_point_cloud,
    voxel_downsample, SECTOR_FRONT, SECTOR_LEFT, SECTOR_RIGHT,
)
from rk_maze.local_occupancy_grid import LocalGridConfig, LocalOccupancyGrid
from rk_maze.lidar_wall_extractor import LidarWallExtractor, CorridorModel
from rk_maze.heading_controller import HeadingController, HeadingControllerConfig
from rk_maze.joint_health_guard import JointHealthGuard, JointHealthConfig


class T2HeadingDiagnostics(Node):
    """Read-only heading closed-loop diagnostics for straight corridor walking."""

    def __init__(self):
        super().__init__('t2_heading_diagnostics')

        # Modules
        self._ld_cfg = LidarDistanceConfig()
        self._grid_cfg = LocalGridConfig()
        self._wall_ext = LidarWallExtractor(self._grid_cfg)
        self._heading = HeadingController(HeadingControllerConfig())

        # State
        self._lock = threading.Lock()
        self._cloud = []
        self._odo_yaw = 0.0
        self._odo_yaw0 = None
        self._imu_wz = 0.0
        self._imu_stamp = 0.0
        self._grid = LocalOccupancyGrid(self._grid_cfg)
        self._grid_frames = 0
        self._sample_count = 0

        # Publishers
        self._diag_pub = self.create_publisher(String, '/maze/t2/diagnostics', 10)

        # Subscriptions
        self.create_subscription(
            PointCloud2, '/utlidar/cloud_base', self._on_cloud,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry, '/utlidar/robot_odom', self._on_odom,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Imu, '/utlidar/imu', self._on_imu,
            qos_profile_sensor_data,
        )

        # Timer: process and publish at 5Hz
        self.create_timer(0.20, self._tick)

        self.get_logger().info(
            'T2 heading diagnostics ready — dry_run, NO motion commands'
        )

    def _on_cloud(self, msg):
        pts = list(pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True))
        cloud = [Point3D(x=float(p[0]), y=float(p[1]), z=float(p[2]))
                 for p in pts if all(math.isfinite(float(v)) for v in p)]
        with self._lock:
            self._cloud = cloud

    def _on_odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        with self._lock:
            self._odo_yaw = yaw
            if self._odo_yaw0 is None:
                self._odo_yaw0 = yaw

    def _on_imu(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self._lock:
            self._imu_wz = msg.angular_velocity.z
            self._imu_stamp = stamp

    def _tick(self):
        with self._lock:
            cloud = self._cloud
            odo_yaw = self._odo_yaw
            odo_yaw0 = self._odo_yaw0
            imu_wz = self._imu_wz
            imu_stamp = self._imu_stamp
            self._cloud = []

        if not cloud:
            return

        now = time.time()

        # F1: Filter + build grid
        filt = filter_point_cloud(cloud, self._ld_cfg)
        filt = voxel_downsample(filt, self._ld_cfg.voxel_size_m)
        for pt in filt:
            self._grid.mark_occupied(pt.x, pt.y)
        self._grid_frames += 1

        # Every ~15 frames (1 second), extract walls and compute heading
        if self._grid_frames % 15 != 0:
            return

        # Wall extraction
        output = self._wall_ext.extract(self._grid, now)
        model = self._wall_ext.build_corridor_model(output.wall_segments, now)
        cells = len(self._grid.occupied_cells())

        # Heading control
        rel_yaw = 0.0
        if odo_yaw0 is not None and self._sample_count > 0:
            rel_yaw = self._heading._normalize_angle(odo_yaw - odo_yaw0)

        imu_age = now - imu_stamp if imu_stamp > 0 else 999.0

        state = self._heading.compute(
            corridor_heading=model.corridor_heading if model.valid else None,
            wall_confidence=model.confidence,
            wall_age_sec=0.05,
            odom_yaw=rel_yaw,
            odom_age_sec=0.05,
            imu_wz=imu_wz,
            imu_age_sec=imu_age,
            left_clearance=model.left_wall_distance
                if math.isfinite(model.left_wall_distance) else 1.0,
            right_clearance=model.right_wall_distance
                if math.isfinite(model.right_wall_distance) else 1.0,
            now_sec=now,
        )
        self._sample_count += 1

        # Diagnostics
        diag = {
            't': now,
            'frames': self._grid_frames,
            'grid_cells': cells,
            'wall_segs': len(output.wall_segments),
            'corridor_valid': model.valid,
            'corridor_heading_deg': round(math.degrees(model.corridor_heading), 2)
                if model.corridor_heading else None,
            'corridor_conf': round(model.confidence, 3),
            'right_wall_m': round(model.right_wall_distance, 3)
                if math.isfinite(model.right_wall_distance) else None,
            'left_wall_m': round(model.left_wall_distance, 3)
                if math.isfinite(model.left_wall_distance) else None,
            'front_wall_m': round(model.front_wall_distance, 3)
                if math.isfinite(model.front_wall_distance) else None,
            'rel_yaw_deg': round(math.degrees(rel_yaw), 2),
            'imu_wz': round(imu_wz, 4),
            'heading_err_deg': round(state.heading_error_deg, 2),
            'lateral_err_m': round(state.lateral_error_m, 3),
            'wz_reference': round(state.wz_reference, 4),
            'wz_heading': round(state.heading_component, 4),
            'wz_center': round(state.center_component, 4),
            'wz_gyro': round(state.gyro_component, 4),
            'mode': state.reason,
            'valid': state.valid,
            'dry_run': True,
            'motion': False,
        }

        msg = String()
        msg.data = json.dumps(diag)
        self._diag_pub.publish(msg)

        # Console summary
        walls_ok = 'L' if model.left_wall else '-'
        walls_ok += 'R' if model.right_wall else '-'
        walls_ok += 'F' if model.front_wall else '-'
        self.get_logger().info(
            f'T2 | cells={cells:3d} walls=[{walls_ok}] '
            f'hdg={diag["corridor_heading_deg"]}deg '
            f'wz={state.wz_reference:+.4f} '
            f'err={state.heading_error_deg:+.1f}deg '
            f'[{state.reason}]'
        )


def main():
    rclpy.init()
    node = T2HeadingDiagnostics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
