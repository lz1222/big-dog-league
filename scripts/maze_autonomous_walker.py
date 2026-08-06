#!/usr/bin/env python3
"""迷宫自主行走节点——融合控制器驱动SDK。

最小可用版本: 直走廊前进，前墙<0.5m停车。
逐步迭代到完整五弯迷宫。
"""

import math, time, threading, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src' / 'rk_maze'))

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud2, Imu
from sensor_msgs_py import point_cloud2 as pc2
from nav_msgs.msg import Odometry

from rk_maze.lidar_distance_core import *
from rk_maze.local_occupancy_grid import LocalGridConfig, LocalOccupancyGrid
from rk_maze.lidar_wall_extractor import LidarWallExtractor
from rk_maze.heading_controller import HeadingController, HeadingControllerConfig
from rk_maze.swept_footprint_checker import *
from rk_maze.safety_arbiter import SafetyArbiter


# --- Safety limits ---
MAX_SPEED = 0.25      # m/s — hard cap
MAX_WZ = 0.50         # rad/s
STOP_DISTANCE = 0.50  # m — stop if front clearance < this


class MazeAutonomousWalker(Node):
    """Minimal autonomous maze walker."""

    def __init__(self):
        super().__init__('maze_autonomous_walker')

        # Modules
        self._ld = LidarDistanceConfig(min_cluster_points=3)
        self._gc = LocalGridConfig()
        self._we = LidarWallExtractor(self._gc)
        self._hc = HeadingController(HeadingControllerConfig())
        self._fp = DynamicFootprint(footprint_front_m=0.28, footprint_rear_m=0.42)
        self._checker = SweptFootprintChecker(self._fp, SweptFootprintConfig())
        self._safety = SafetyArbiter()
        self._speed_est = LiDARSpeedEstimator(min_samples=3)

        # State
        self._lock = threading.Lock()
        self._cloud = []
        self._odo_yaw = 0.0; self._odo_yaw0 = None
        self._imu_wz = 0.0
        self._grid = LocalOccupancyGrid(self._gc)
        self._frame = 0
        self._state = 'INIT'
        self._cmd_count = 0

        # Publisher — directly to /navigation/cmd_vel for testing
        self._cmd_pub = self.create_publisher(Twist, '/navigation/cmd_vel', 10)

        # Subscriptions
        self.create_subscription(PointCloud2, '/utlidar/cloud_base',
                                 self._on_cloud, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/utlidar/robot_odom',
                                 self._on_odom, qos_profile_sensor_data)
        self.create_subscription(Imu, '/utlidar/imu',
                                 self._on_imu, qos_profile_sensor_data)

        # 10Hz control loop
        self.create_timer(0.10, self._control_loop)

        self.get_logger().info('Maze walker ready. Starting in 3 seconds...')
        self.get_logger().info(f'Max speed: {MAX_SPEED}m/s  Stop dist: {STOP_DISTANCE}m')

    def _on_cloud(self, msg):
        pts = list(pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True))
        with self._lock:
            self._cloud = [Point3D(x=float(p[0]), y=float(p[1]), z=float(p[2]))
                           for p in pts if all(math.isfinite(float(v)) for v in p)]

    def _on_odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        with self._lock:
            self._odo_yaw = yaw
            if self._odo_yaw0 is None:
                self._odo_yaw0 = yaw

    def _on_imu(self, msg):
        with self._lock:
            self._imu_wz = msg.angular_velocity.z

    def _control_loop(self):
        with self._lock:
            cloud = self._cloud; self._cloud = []
            odo_yaw = self._odo_yaw
            odo_yaw0 = self._odo_yaw0
            imu_wz = self._imu_wz

        if not cloud:
            return

        self._frame += 1

        # F1: Filter + grid
        filt = filter_point_cloud(cloud, self._ld)
        filt = voxel_downsample(filt, self._ld.voxel_size_m)
        for pt in filt:
            self._grid.mark_occupied(pt.x, pt.y)

        # Every 5th frame: compute decision
        if self._frame % 5 != 0:
            return

        now = time.time()

        # Hard distance
        front_pts = [p for p in filt if abs(math.degrees(math.atan2(p.y, p.x))) <= 30
                     and p.z > 0.005]
        if not front_pts:
            self._stop('no front points')
            return

        sd = compute_hard_distance(front_pts, SECTOR_FRONT, self._ld, now, now)
        if not sd.valid:
            return

        front_clearance = sd.front_clearance
        self._speed_est.update(now, sd.hard_distance)
        speed = self._speed_est.speed(now)
        dyn_stop = compute_dynamic_stop_distance(speed, self._ld)

        # ---- Decision ----
        twist = Twist()

        if front_clearance < STOP_DISTANCE:
            self._stop(f'front too close: {front_clearance:.2f}m < {STOP_DISTANCE}m')
            return

        # Simple forward with heading correction
        rel_yaw = self._hc._normalize_angle(odo_yaw - odo_yaw0) if odo_yaw0 else 0.0

        # Wall extraction for heading
        out = self._we.extract(self._grid, now)
        model = self._we.build_corridor_model(out.wall_segments, now)

        heading_state = self._hc.compute(
            corridor_heading=model.corridor_heading if model.valid else None,
            wall_confidence=model.confidence, wall_age_sec=0.05,
            odom_yaw=rel_yaw, odom_age_sec=0.05,
            imu_wz=imu_wz, imu_age_sec=0.05,
            left_clearance=model.left_wall_distance
                if math.isfinite(model.left_wall_distance) else 1.0,
            right_clearance=model.right_wall_distance
                if math.isfinite(model.right_wall_distance) else 1.0,
            now_sec=now,
        )

        # Generate candidates
        wz_ref = heading_state.wz_reference if heading_state.valid else 0.0
        obs = [Point3D(x=cx, y=cy, z=0.3) for cx, cy in self._grid.occupied_cell_points()]

        candidates = []
        for name, vx, wz in [
            ('FWD', min(MAX_SPEED, 0.20), wz_ref),
            ('STOP', 0.0, 0.0),
        ]:
            c = VelocityCandidate(name=name, vx=vx, vy=0.0, wz=wz, duration_sec=1.0)
            checked = self._checker.check(c, obs)
            candidates.append(checked)

        robust = [c for c in candidates if c.robust_safe]
        if not robust:
            self._stop('no robust safe candidate')
            return

        selected = robust[0]
        if selected.name == 'STOP':
            self._stop('only STOP is safe')
            return

        # Safety arbitration
        speed_class = classify_speed(front_clearance, front_clearance, dyn_stop, self._ld)
        verdict = self._safety.evaluate(
            joint_health_state='NORMAL',
            cloud_stale=False, odom_stale=False, imu_stale=False,
            hard_front_distance=front_clearance, speed_class=speed_class,
            selected_candidate=selected,
        )

        if not verdict.can_move:
            self._stop(f'safety: {verdict.reason}')
            return

        # PUBLISH COMMAND
        twist.linear.x = max(0.0, min(MAX_SPEED, selected.vx))
        twist.angular.z = max(-MAX_WZ, min(MAX_WZ, selected.wz))
        self._cmd_pub.publish(twist)
        self._cmd_count += 1

        if self._cmd_count % 10 == 0:
            self.get_logger().info(
                f'WALK | front={front_clearance:.2f}m '
                f'stop={dyn_stop:.2f}m '
                f'vx={twist.linear.x:.2f} wz={twist.angular.z:+.2f} '
                f'hdg={math.degrees(model.corridor_heading) if model.corridor_heading else 0:+.1f}deg '
                f'cells={len(self._grid.occupied_cells())} '
                f'[{heading_state.reason}]'
            )

    def _stop(self, reason):
        twist = Twist()
        self._cmd_pub.publish(twist)
        if reason != self._state:
            self.get_logger().warn(f'STOP: {reason}')
            self._state = reason


def main():
    rclpy.init()
    node = MazeAutonomousWalker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
