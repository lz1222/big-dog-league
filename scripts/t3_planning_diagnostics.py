#!/usr/bin/env python3
"""T3: 完整规划链诊断 — dry_run, 只读, 不发布运动命令。

观测完整链路:
  点云 → 网格 → 墙线 → 走廊航向
  → wz_reference (heading controller)
  → 速度候选生成 (planner)
  → 动态足迹检查 (每个候选)
  → 安全仲裁 → SELECTED candidate

输出: /maze/t3/diagnostics (JSON)
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
    voxel_downsample, SECTOR_FRONT,
)
from rk_maze.local_occupancy_grid import LocalGridConfig, LocalOccupancyGrid
from rk_maze.lidar_wall_extractor import LidarWallExtractor
from rk_maze.heading_controller import HeadingController, HeadingControllerConfig
from rk_maze.maze_local_planner import MazeLocalPlanner, PlannerConfig
from rk_maze.swept_footprint_checker import (
    DynamicFootprint, SweptFootprintConfig,
    VelocityCandidate, Pose2D,
    VERDICT_ROBUST_SAFE, VERDICT_UNSAFE, VERDICT_UNKNOWN,
)
from rk_maze.safety_arbiter import SafetyArbiter, SafetyVerdict
from rk_maze.joint_health_guard import JointHealthGuard, JointHealthConfig


class T3PlanningDiagnostics(Node):
    """Full planning chain diagnostics."""

    def __init__(self):
        super().__init__('t3_planning_diagnostics')

        # Modules
        self._ld_cfg = LidarDistanceConfig()
        self._grid_cfg = LocalGridConfig()
        self._wall_ext = LidarWallExtractor(self._grid_cfg)
        self._heading = HeadingController(HeadingControllerConfig())
        self._footprint = DynamicFootprint(
            footprint_front_m=0.28, footprint_rear_m=0.42,
            footprint_left_m=0.18, footprint_right_m=0.18,
        )
        self._planner = MazeLocalPlanner(
            PlannerConfig(corridor_cruise_vx=0.25, corridor_slow_vx=0.15,
                          delta_wz=0.05),
            self._footprint,
            SweptFootprintConfig(),
        )
        self._safety = SafetyArbiter()

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
        self._diag_pub = self.create_publisher(String, '/maze/t3/diagnostics', 10)

        # Subscriptions
        self.create_subscription(PointCloud2, '/utlidar/cloud_base',
                                 self._on_cloud, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/utlidar/robot_odom',
                                 self._on_odom, qos_profile_sensor_data)
        self.create_subscription(Imu, '/utlidar/imu',
                                 self._on_imu, qos_profile_sensor_data)

        self.create_timer(0.20, self._tick)

        self.get_logger().info(
            'T3 planning diagnostics ready — dry_run, NO motion commands'
        )

    def _on_cloud(self, msg):
        pts = list(pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True))
        cloud = [Point3D(x=float(p[0]), y=float(p[1]), z=float(p[2]))
                 for p in pts if all(math.isfinite(float(v)) for v in p)]
        with self._lock:
            self._cloud = cloud

    def _on_odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z))
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

        if self._grid_frames % 10 != 0:
            return

        # F4: Wall extraction
        output = self._wall_ext.extract(self._grid, now)
        model = self._wall_ext.build_corridor_model(output.wall_segments, now)
        cells = len(self._grid.occupied_cells())

        # Obstacle points for footprint checking
        obstacle_points = [
            Point3D(x=cx, y=cy, z=0.3)
            for cx, cy in self._grid.occupied_cell_points()
        ]

        # F7: Heading control
        rel_yaw = 0.0
        if odo_yaw0 is not None and self._sample_count > 0:
            rel_yaw = self._heading._normalize_angle(odo_yaw - odo_yaw0)
        imu_age = now - imu_stamp if imu_stamp > 0 else 999.0

        heading_state = self._heading.compute(
            corridor_heading=model.corridor_heading if model.valid else None,
            wall_confidence=model.confidence, wall_age_sec=0.05,
            odom_yaw=rel_yaw, odom_age_sec=0.05,
            imu_wz=imu_wz, imu_age_sec=imu_age,
            left_clearance=model.left_wall_distance
                if math.isfinite(model.left_wall_distance) else 1.0,
            right_clearance=model.right_wall_distance
                if math.isfinite(model.right_wall_distance) else 1.0,
            now_sec=now,
        )

        # F6: Planning — generate and check candidates around wz_reference
        candidates = []
        wz_ref = heading_state.wz_reference if heading_state.valid else 0.0
        dw = 0.05  # delta_wz

        for label, vx, wz in [
            ('STOP', 0.0, 0.0),
            ('FORWARD_SLOW', 0.15, wz_ref),
            ('FORWARD_CRUISE', 0.25, wz_ref),
            ('CORRECT_LEFT', 0.25, wz_ref + dw),
            ('CORRECT_RIGHT', 0.25, wz_ref - dw),
        ]:
            cand = VelocityCandidate(name=label, vx=vx, vy=0.0, wz=wz,
                                     duration_sec=1.0)
            checked = self._planner._checker.check_with_perturbation(
                cand, obstacle_points, Pose2D(),
            )
            candidates.append(checked)

        # F10: Safety arbitration
        selected = None
        robust = [c for c in candidates if c.robust_safe]
        if robust:
            robust.sort(key=lambda c: (-c.minimum_clearance, -c.vx))
            selected = robust[0]

        verdict = self._safety.evaluate(
            joint_health_state='NORMAL',
            cloud_stale=False, odom_stale=False, imu_stale=False,
            hard_front_distance=3.0, speed_class='CLEAR',
            selected_candidate=selected,
        )
        self._sample_count += 1

        # Console output
        cand_summary = ' | '.join(
            '{}({:.2f})'.format(c.name[:6], c.minimum_clearance)
            for c in candidates[:5]
        )
        sel_name = selected.name if selected else 'NONE'
        sel_cl = selected.minimum_clearance if selected else 0
        sel_robust = selected.robust_safe if selected else False
        sel_wz = selected.wz if selected else 0

        self.get_logger().info(
            f'T3 | cells={cells:3d} hdg={math.degrees(model.corridor_heading) if model.corridor_heading else 0:+.1f}deg '
            f'wz_ref={wz_ref:+.3f} '
            f'SELECTED={sel_name} wz={sel_wz:+.3f} cl={sel_cl:.2f}m '
            f'robust={sel_robust} '
            f'[{",".join(c.name[:4] for c in robust[:4])}] '
            f'can_move={verdict.can_move}'
        )

        # JSON diagnostics
        diag = {
            't': now,
            'frames': self._grid_frames,
            'cells': cells,
            'corridor_hdg_deg': round(math.degrees(model.corridor_heading), 2)
                if model.corridor_heading else None,
            'corridor_valid': model.valid,
            'wz_reference': round(wz_ref, 4),
            'heading_err_deg': round(heading_state.heading_error_deg, 2),
            'mode': heading_state.reason,
            'candidates': [
                {
                    'name': c.name, 'vx': c.vx, 'wz': c.wz,
                    'verdict': c.verdict, 'robust_safe': c.robust_safe,
                    'min_clearance': round(c.minimum_clearance, 3),
                    'collision': c.collision,
                }
                for c in candidates
            ],
            'selected': {
                'name': selected.name if selected else 'NONE',
                'vx': selected.vx if selected else 0,
                'wz': selected.wz if selected else 0,
                'robust_safe': sel_robust,
                'min_clearance': round(sel_cl, 3),
            } if selected else None,
            'safety_can_move': verdict.can_move,
            'safety_reason': verdict.reason,
            'dry_run': True,
            'motion': False,
        }

        msg = String()
        msg.data = json.dumps(diag)
        self._diag_pub.publish(msg)


def main():
    rclpy.init()
    node = T3PlanningDiagnostics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
