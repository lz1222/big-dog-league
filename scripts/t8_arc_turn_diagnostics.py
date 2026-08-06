#!/usr/bin/env python3
"""T8: 移动圆弧转弯诊断 — dry_run, 只读。

狗在拐角处，遥控器让它走弧线转弯。
观测: 走廊航向变化 → arc candidates 生成 → 足迹检查 → ROBUST_SAFE 选择
"""

import math, time, threading, json
from pathlib import Path
import rclpy; from rclpy.node import Node; from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, Imu; from sensor_msgs_py import point_cloud2 as pc2
from nav_msgs.msg import Odometry; from std_msgs.msg import String
import sys; sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src' / 'rk_maze'))

from rk_maze.lidar_distance_core import *
from rk_maze.local_occupancy_grid import LocalGridConfig, LocalOccupancyGrid
from rk_maze.lidar_wall_extractor import LidarWallExtractor
from rk_maze.heading_controller import HeadingController, HeadingControllerConfig
from rk_maze.swept_footprint_checker import *

class T8ArcTurnDiagnostics(Node):
    def __init__(self):
        super().__init__('t8_arc_turn')
        self._ld = LidarDistanceConfig()
        self._gc = LocalGridConfig(); self._we = LidarWallExtractor(self._gc)
        self._hc = HeadingController(HeadingControllerConfig())
        self._fp = DynamicFootprint(footprint_front_m=0.28, footprint_rear_m=0.42)
        self._checker = SweptFootprintChecker(self._fp, SweptFootprintConfig())
        self._lock = threading.Lock(); self._cloud = []
        self._odo_yaw = 0.0; self._odo_yaw0 = None; self._imu_wz = 0.0
        self._grid = LocalOccupancyGrid(self._gc); self._n = 0; self._total_yaw = 0.0

        self.create_subscription(PointCloud2, '/utlidar/cloud_base', self._on_c, 10)
        self.create_subscription(Odometry, '/utlidar/robot_odom', self._on_o, 10)
        self.create_subscription(Imu, '/utlidar/imu', self._on_i, 10)
        self.create_timer(0.15, self._tick)
        self.get_logger().info('T8 arc turn ready — turn the robot in an arc!')

    def _on_c(self, msg):
        pts = list(pc2.read_points(msg, field_names=('x','y','z'), skip_nans=True))
        with self._lock: self._cloud = [Point3D(x=float(p[0]),y=float(p[1]),z=float(p[2])) for p in pts if all(math.isfinite(float(v)) for v in p)]

    def _on_o(self, msg):
        q = msg.pose.pose.orientation; y = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        with self._lock:
            if self._odo_yaw0 is None: self._odo_yaw0 = y
            self._total_yaw = math.degrees(self._hc._normalize_angle(y - self._odo_yaw0))
            self._odo_yaw = y

    def _on_i(self, msg):
        with self._lock: self._imu_wz = msg.angular_velocity.z

    def _tick(self):
        with self._lock: cloud = self._cloud; self._cloud = []; odo_yaw = self._odo_yaw; odo_yaw0 = self._odo_yaw0; imu_wz = self._imu_wz
        if not cloud: return
        self._n += 1
        filt = voxel_downsample(filter_point_cloud(cloud, self._ld), 0.02)
        for pt in filt: self._grid.mark_occupied(pt.x, pt.y)
        if self._n % 5 != 0: return

        now = time.time(); cells = len(self._grid.occupied_cells())
        out = self._we.extract(self._grid, now)
        model = self._we.build_corridor_model(out.wall_segments, now)
        rel_yaw = self._hc._normalize_angle(odo_yaw - odo_yaw0) if odo_yaw0 else 0.0
        heading = self._hc.compute(
            corridor_heading=model.corridor_heading if model.valid else None,
            wall_confidence=model.confidence, wall_age_sec=0.05,
            odom_yaw=rel_yaw, odom_age_sec=0.05, imu_wz=imu_wz, imu_age_sec=0.05,
            left_clearance=model.left_wall_distance if math.isfinite(model.left_wall_distance) else 1.0,
            right_clearance=model.right_wall_distance if math.isfinite(model.right_wall_distance) else 1.0,
            now_sec=now,
        )

        obs = [Point3D(x=cx, y=cy, z=0.3) for cx, cy in self._grid.occupied_cell_points()]
        wz_ref = heading.wz_reference if heading.valid else 0.0
        direction = 'LEFT' if wz_ref > 0 else 'RIGHT'
        cands = []
        for n, vx, wz in [
            ('STOP', 0.0, 0.0),
            ('L_ARC_SM', 0.10, +0.30), ('L_ARC_MD', 0.18, +0.50), ('L_ARC_OUT', 0.10, +0.30),
            ('R_ARC_SM', 0.10, -0.30), ('R_ARC_MD', 0.18, -0.50), ('R_ARC_OUT', 0.10, -0.30),
            ('FINE_ALGN', 0.08, wz_ref),
        ]:
            c = VelocityCandidate(name=n, vx=vx, vy=0.0, wz=wz, duration_sec=1.0)
            cands.append(self._checker.check(c, obs))

        robust = [c for c in cands if c.robust_safe]
        sel = robust[0].name if robust else 'NONE'
        total_yaw = self._total_yaw

        # Compact output
        parts = []
        for c in cands:
            mark = '*' if c.name == sel else ' '
            parts.append(f'{mark}{c.name[:6]}({c.minimum_clearance:.2f})')
        cand_str = ' '.join(parts)

        self.get_logger().info(
            f'T8 | yaw={total_yaw:+5.1f}deg wz_ref={wz_ref:+5.3f} '
            f'hdg={math.degrees(model.corridor_heading) if model.corridor_heading else 0:+5.1f}deg '
            f'SEL={sel} [{cand_str}]'
        )


def main():
    rclpy.init(); n = T8ArcTurnDiagnostics()
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass
    finally: n.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()
