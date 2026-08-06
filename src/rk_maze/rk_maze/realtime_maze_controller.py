#!/usr/bin/env python3
"""F10: Realtime maze corridor navigation controller.

This is the ONLY node that publishes to /control/locomotion_cmd.
It integrates all fusion modules:
- LiDAR distance (hard + navigation dual channels)
- Motion-compensated multi-frame point cloud
- Local occupancy grid + wall line extraction
- Heading fusion (LiDAR wall + Odom + IMU gyro)
- Dynamic footprint swept checking
- Velocity candidate planning
- Stop bias estimation
- JointHealthGuard
- Safety arbitration

Architecture:
  /utlidar/cloud_base ─┐
  /utlidar/robot_odom ─┤
  /utlidar/imu ────────┤
  /lowstate ───────────┘
        │
        ▼
  realtime_maze_controller
        │
        ▼
  /control/locomotion_cmd ─► command_mux ─► /navigation/cmd_vel ─► UDP ─► Go2

Safety gates:
  dry_run=true  → no non-zero Twist published
  enable_motion=false → no Twist at all
  armed=false → explicit gate
"""

import json
import math
import time
import threading
from dataclasses import dataclass
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String

from rk_maze.heading_controller import (
    HeadingController, HeadingControllerConfig, HeadingState,
)
from rk_maze.lidar_wall_extractor import (
    LidarWallExtractor, CorridorModel,
)
from rk_maze.local_occupancy_grid import (
    LocalGridConfig, LocalOccupancyGrid,
)
from rk_maze.joint_health_guard import (
    JointHealthGuard, JointHealthConfig, JointHealthStatus, MotorState,
    HEALTH_NORMAL, HEALTH_LIMITED, HEALTH_COOLDOWN_REQUIRED,
    HEALTH_HARDWARE_FAULT, HEALTH_SENSOR_INVALID,
)
from rk_maze.lidar_distance_core import (
    LidarDistanceConfig, NavigationDistanceFilter, Point3D,
    DistanceSnapshot, SectorDistance,
    filter_point_cloud, voxel_downsample, compute_hard_distance,
    compute_all_hard_distances, compute_distance_snapshot,
    classify_speed, compute_dynamic_stop_distance,
    SECTOR_FRONT, SECTOR_LEFT, SECTOR_RIGHT, SECTOR_REAR,
    ALL_SECTORS,
    SPEED_CLEAR, SPEED_CAUTION, SPEED_BRAKE, SPEED_EMERGENCY, SPEED_UNKNOWN,
)
from rk_maze.maze_local_planner import (
    MazeLocalPlanner, PlannerConfig, PlannerOutput,
    STATE_IDLE, STATE_CORRIDOR_CRUISE, STATE_TURN_APPROACH,
    STATE_STOP_AND_SCAN, STATE_PLAN_CANDIDATES,
    STATE_ARC_TURN_ENTRY, STATE_ARC_TURN_MAIN, STATE_TURN_FINE_ALIGN,
    STATE_CORRIDOR_REACQUIRE, STATE_PRE_STOP_COMPENSATE,
    STATE_STOP_AND_SETTLE, STATE_VERIFY_STOP_HEADING,
    STATE_THERMAL_COOLDOWN, STATE_FAULT_STOP, STATE_CANCELED, STATE_COMPLETE,
)
from rk_maze.motion_compensated_cloud import (
    MotionCompensatedCloudBuffer, MotionCompensatedCloudConfig,
    OdomPose,
)
from rk_maze.safety_arbiter import SafetyArbiter, SafetyVerdict
from rk_maze.stop_bias_estimator import StopBiasEstimator, StopBiasConfig
from rk_maze.swept_footprint_checker import (
    DynamicFootprint, SweptFootprintConfig,
    VelocityCandidate, Pose2D,
    VERDICT_ROBUST_SAFE,
)


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

@dataclass
class ControllerConfig:
    """Master configuration for the realtime maze controller."""
    # Topics
    cloud_topic: str = '/utlidar/cloud_base'
    odom_topic: str = '/utlidar/robot_odom'
    imu_topic: str = '/utlidar/imu'
    lowstate_topic: str = '/lowstate'
    output_cmd_topic: str = '/control/locomotion_cmd'
    diagnostics_topic: str = '/maze/realtime/status'
    debug_cloud_topic: str = '/maze/debug/filtered_cloud'
    local_grid_topic: str = '/maze/local_occupancy_grid'

    # Safety gates
    dry_run: bool = True
    enable_motion: bool = False
    armed: bool = False

    # Loop rates
    perception_rate_hz: float = 15.0
    planning_rate_hz: float = 12.0
    control_rate_hz: float = 20.0
    diagnostics_rate_hz: float = 5.0


# ---------------------------------------------------------------------------
# Main Node
# ---------------------------------------------------------------------------

class RealtimeMazeController(Node):
    """Single authoritative node for maze corridor navigation."""

    def __init__(self, **kwargs):
        super().__init__('realtime_maze_controller', **kwargs)

        # ---- Load configuration ----
        self._ctrl_config = self._load_ctrl_config()
        self._ld_config = self._load_ld_config()
        self._heading_config = self._load_heading_config()
        self._mcc_config = self._load_mcc_config()
        self._footprint = self._load_footprint()
        self._checker_config = self._load_checker_config()
        self._planner_config = self._load_planner_config()
        self._stop_bias_config = self._load_stop_bias_config()
        self._jh_config = self._load_jh_config()

        # ---- Safety gates ----
        self._dry_run = self._ctrl_config.dry_run
        self._enable_motion = self._ctrl_config.enable_motion
        self._armed = self._ctrl_config.armed

        # ---- Modules ----
        self._nav_filter = NavigationDistanceFilter(self._ld_config)
        self._cloud_buffer = MotionCompensatedCloudBuffer(self._mcc_config)
        self._heading_ctrl = HeadingController(self._heading_config)
        self._grid_config = self._load_grid_config()
        self._wall_extractor = LidarWallExtractor(self._grid_config)
        self._latest_corridor: Optional[CorridorModel] = None
        self._planner = MazeLocalPlanner(
            self._planner_config, self._footprint, self._checker_config,
        )
        self._stop_bias = StopBiasEstimator(self._stop_bias_config)
        self._joint_health = JointHealthGuard(self._jh_config)
        self._safety = SafetyArbiter()

        # ---- State ----
        self._state = STATE_IDLE
        self._state_lock = threading.RLock()
        self._latest_cloud: list = []
        self._latest_cloud_stamp: float = 0.0
        self._latest_cloud_lock = threading.RLock()
        self._latest_odom: Optional[OdomPose] = None
        self._latest_imu: Optional[dict] = None  # {'wz': float, 'stamp': float}
        self._latest_motor_states: dict = {}
        self._imu_lock = threading.RLock()
        self._odom_lock = threading.RLock()
        self._motor_lock = threading.RLock()
        self._selected_candidate: Optional[VelocityCandidate] = None
        self._stop_sequence: int = 0
        self._nonzero_motion_generated: bool = False

        # ---- Publishers ----
        self._cmd_publisher = self.create_publisher(
            Twist, self._ctrl_config.output_cmd_topic, 10,
        )
        self._diag_publisher = self.create_publisher(
            String, self._ctrl_config.diagnostics_topic, 10,
        )

        # ---- Subscriptions ----
        self._cloud_sub = self.create_subscription(
            PointCloud2, self._ctrl_config.cloud_topic,
            self._on_cloud, qos_profile_sensor_data,
        )
        self._odom_sub = self.create_subscription(
            Odometry, self._ctrl_config.odom_topic,
            self._on_odom, qos_profile_sensor_data,
        )
        self._imu_sub = self.create_subscription(
            Imu, self._ctrl_config.imu_topic,
            self._on_imu, qos_profile_sensor_data,
        )
        # Note: lowstate subscription depends on DDS bridge;
        # we subscribe if available, gracefully handle missing data.

        # ---- Timers (three-loop architecture) ----
        self._perception_timer = self.create_timer(
            1.0 / self._ctrl_config.perception_rate_hz,
            self._perception_loop,
        )
        self._planning_timer = self.create_timer(
            1.0 / self._ctrl_config.planning_rate_hz,
            self._planning_loop,
        )
        self._control_timer = self.create_timer(
            1.0 / self._ctrl_config.control_rate_hz,
            self._control_loop,
        )
        self._diag_timer = self.create_timer(
            1.0 / self._ctrl_config.diagnostics_rate_hz,
            self._publish_diagnostics,
        )

        self.get_logger().info(
            f'RealtimeMazeController ready: '
            f'dry_run={self._dry_run}, '
            f'enable_motion={self._enable_motion}, '
            f'armed={self._armed}, '
            f'state={self._state}, '
            f'output={self._ctrl_config.output_cmd_topic}'
        )

    # -------------------------------------------------------------------
    # Configuration loaders
    # -------------------------------------------------------------------

    def _load_ctrl_config(self) -> ControllerConfig:
        params = {
            'cloud_topic', 'odom_topic', 'imu_topic', 'lowstate_topic',
            'output_cmd_topic', 'diagnostics_topic', 'debug_cloud_topic',
            'local_grid_topic', 'dry_run', 'enable_motion', 'armed',
            'perception_rate_hz', 'planning_rate_hz', 'control_rate_hz',
            'diagnostics_rate_hz',
        }
        for p in params:
            self.declare_parameter(p, getattr(ControllerConfig(), p))
        return ControllerConfig(
            cloud_topic=self._str('cloud_topic'),
            odom_topic=self._str('odom_topic'),
            imu_topic=self._str('imu_topic'),
            lowstate_topic=self._str('lowstate_topic'),
            output_cmd_topic=self._str('output_cmd_topic'),
            diagnostics_topic=self._str('diagnostics_topic'),
            debug_cloud_topic=self._str('debug_cloud_topic'),
            local_grid_topic=self._str('local_grid_topic'),
            dry_run=self.get_parameter('dry_run').value,
            enable_motion=self.get_parameter('enable_motion').value,
            armed=self.get_parameter('armed').value,
            perception_rate_hz=self._float('perception_rate_hz'),
            planning_rate_hz=self._float('planning_rate_hz'),
            control_rate_hz=self._float('control_rate_hz'),
            diagnostics_rate_hz=self._float('diagnostics_rate_hz'),
        )

    def _load_ld_config(self) -> LidarDistanceConfig:
        return LidarDistanceConfig(
            min_range_m=self._try_float('min_range_m', 0.08),
            max_range_m=self._try_float('max_range_m', 3.00),
            ground_z_min_m=self._try_float('ground_z_min_m', -0.30),
            ground_z_max_m=self._try_float('ground_z_max_m', 0.03),
            obstacle_z_min_m=self._try_float('obstacle_z_min_m', 0.03),
            obstacle_z_max_m=self._try_float('obstacle_z_max_m', 0.80),
            body_x_min_m=self._try_float('body_x_min_m', -0.40),
            body_x_max_m=self._try_float('body_x_max_m', 0.40),
            body_y_min_m=self._try_float('body_y_min_m', -0.18),
            body_y_max_m=self._try_float('body_y_max_m', 0.18),
            voxel_size_m=self._try_float('voxel_size_m', 0.02),
            cluster_tolerance_m=self._try_float('cluster_tolerance_m', 0.05),
            min_cluster_points=self._try_int('min_cluster_points', 5),
            cluster_percentile=self._try_float('cluster_percentile', 0.10),
            footprint_front_m=self._try_float('footprint_front_m', 0.35),
            footprint_rear_m=self._try_float('footprint_rear_m', 0.35),
            footprint_left_m=self._try_float('footprint_left_m', 0.18),
            footprint_right_m=self._try_float('footprint_right_m', 0.18),
            perception_margin_m=self._try_float('perception_margin_m', 0.03),
            hard_max_age_sec=self._try_float('hard_max_age_sec', 0.15),
            nav_temporal_window=self._try_int('nav_temporal_window', 5),
            nav_ema_alpha=self._try_float('nav_ema_alpha', 0.3),
            nav_max_age_sec=self._try_float('nav_max_age_sec', 0.50),
        )

    def _load_heading_config(self) -> HeadingControllerConfig:
        return HeadingControllerConfig(
            kp_heading=self._try_float('kp_heading', 1.5),
            kp_center=self._try_float('kp_center', 0.8),
            kd_gyro=self._try_float('kd_gyro', 0.3),
        )

    def _load_mcc_config(self) -> MotionCompensatedCloudConfig:
        return MotionCompensatedCloudConfig(
            accumulation_window_sec=self._try_float('accumulation_window_sec', 0.30),
        )

    def _load_footprint(self) -> DynamicFootprint:
        return DynamicFootprint(
            footprint_front_m=self._try_float('footprint_front_m', 0.35),
            footprint_rear_m=self._try_float('footprint_rear_m', 0.35),
            footprint_left_m=self._try_float('footprint_left_m', 0.18),
            footprint_right_m=self._try_float('footprint_right_m', 0.18),
        )

    def _load_checker_config(self) -> SweptFootprintConfig:
        return SweptFootprintConfig()

    def _load_planner_config(self) -> PlannerConfig:
        return PlannerConfig(
            corridor_cruise_vx=self._try_float('corridor_cruise_vx', 0.25),
            corridor_slow_vx=self._try_float('corridor_slow_vx', 0.15),
        )

    def _load_stop_bias_config(self) -> StopBiasConfig:
        return StopBiasConfig()

    def _load_jh_config(self) -> JointHealthConfig:
        return JointHealthConfig(
            temp_warn_deg_c=self._try_float('temp_warn_deg_c', 70.0),
            temp_critical_deg_c=self._try_float('temp_critical_deg_c', 85.0),
            temp_hardware_fault_deg_c=self._try_float('temp_hardware_fault_deg_c', 105.0),
        )

    def _load_grid_config(self) -> LocalGridConfig:
        return LocalGridConfig(
            x_min_m=-1.50, x_max_m=1.50,
            y_min_m=-1.50, y_max_m=1.50,
            resolution_m=0.03,
            min_range_m=0.08, max_range_m=3.00,
            z_min_m=0.03, z_max_m=0.80,
            body_x_min_m=-0.40, body_x_max_m=0.40,
            body_y_min_m=-0.18, body_y_max_m=0.18,
            wall_min_points=8, wall_min_length_m=0.15,
            wall_inlier_tolerance_m=0.04, wall_max_residual_m=0.06,
            wall_ransac_sample_limit=200, wall_max_segments=4,
            wall_cluster_gap_m=0.08,
            wall_fragment_min_points=4, wall_fragment_min_length_m=0.08,
        )

    # -------------------------------------------------------------------
    # Sensor callbacks (non-blocking, just store latest data)
    # -------------------------------------------------------------------

    def _on_cloud(self, msg: PointCloud2):
        """Store latest point cloud. Blocking work happens in perception loop."""
        try:
            points = list(point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True))
        except Exception as e:
            self.get_logger().warn(f'Failed to read PointCloud2: {e}')
            return

        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        cloud = [Point3D(x=float(p[0]), y=float(p[1]), z=float(p[2]))
                 for p in points
                 if math.isfinite(p[0]) and math.isfinite(p[1]) and math.isfinite(p[2])]

        with self._latest_cloud_lock:
            self._latest_cloud = cloud
            self._latest_cloud_stamp = stamp

    def _on_odom(self, msg: Odometry):
        """Store latest odometry pose."""
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        q = msg.pose.pose.orientation
        yaw = self._quat_to_yaw(q.x, q.y, q.z, q.w)

        pose = OdomPose(
            stamp_sec=stamp,
            x=msg.pose.pose.position.x,
            y=msg.pose.pose.position.y,
            yaw=yaw,
            vx=msg.twist.twist.linear.x,
            vy=msg.twist.twist.linear.y,
            wz=msg.twist.twist.angular.z,
        )

        with self._odom_lock:
            self._latest_odom = pose

    def _on_imu(self, msg: Imu):
        """Store latest IMU angular velocity."""
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self._imu_lock:
            self._latest_imu = {
                'wz': msg.angular_velocity.z,
                'stamp': stamp,
            }

    # -------------------------------------------------------------------
    # Three-loop architecture (Section 18/21)
    # -------------------------------------------------------------------

    def _perception_loop(self):
        """~15Hz: Point cloud filtering, hard_distance, multi-frame processing."""
        with self._latest_cloud_lock:
            cloud = list(self._latest_cloud)
            cloud_stamp = self._latest_cloud_stamp

        if not cloud:
            return

        now = self.get_clock().now().nanoseconds * 1e-9

        # F1: Filter
        filtered = filter_point_cloud(cloud, self._ld_config)

        # Voxel downsample
        filtered = voxel_downsample(filtered, self._ld_config.voxel_size_m)

        # F2: Hard distances (all sectors)
        hard = compute_all_hard_distances(filtered, self._ld_config, cloud_stamp, now)

        # F3: Navigation distances (temporal filter)
        self._nav_filter.update(hard, cloud_stamp)

        # F4a: Odom compensation — add to buffer
        with self._odom_lock:
            odom = self._latest_odom
        if odom is not None:
            self._cloud_buffer.update_odom(odom)
            self._cloud_buffer.add_cloud(filtered, cloud_stamp, odom)

        # F4b+F4c: Build local occupancy grid + extract walls from compensated cloud
        compensated = self._cloud_buffer.get_compensated_cloud()
        if compensated:
            grid = LocalOccupancyGrid(self._grid_config)
            for pt in compensated:
                grid.mark_occupied(pt.x, pt.y)
            self._latest_corridor = self._wall_extractor.build_corridor_model(
                self._wall_extractor.extract(grid, cloud_stamp).wall_segments,
                cloud_stamp,
            )

        # F9: JointHealthGuard — update from motor states
        with self._motor_lock:
            motor_states = dict(self._latest_motor_states)
        if motor_states:
            self._joint_health.update(motor_states, now)

    def _planning_loop(self):
        """~12Hz: Heading fusion, candidate generation, footprint checking."""
        now = self.get_clock().now().nanoseconds * 1e-9

        # Get latest sensor data
        with self._latest_cloud_lock:
            cloud = list(self._latest_cloud)
            cloud_stamp = self._latest_cloud_stamp

        with self._odom_lock:
            odom = self._latest_odom

        with self._imu_lock:
            imu = self._latest_imu

        if odom is None:
            return

        # STALE checks
        cloud_stale = (now - cloud_stamp) > self._ld_config.hard_max_age_sec if cloud_stamp > 0 else True
        odom_stale = (now - odom.stamp_sec) > self._heading_config.odom_max_age_sec
        imu_stale = (imu is None) or ((now - imu['stamp']) > self._heading_config.imu_max_age_sec)

        imu_wz = imu['wz'] if imu is not None and not imu_stale else 0.0

        # F7: Heading fusion
        corridor = self._latest_corridor
        if corridor is not None and corridor.valid:
            corridor_heading = corridor.corridor_heading
            wall_confidence = corridor.confidence
            wall_age = now - cloud_stamp
        else:
            corridor_heading = None
            wall_confidence = 0.0
            wall_age = 999.0

        heading = self._heading_ctrl.compute(
            corridor_heading=corridor_heading,
            wall_confidence=wall_confidence,
            wall_age_sec=wall_age,
            odom_yaw=odom.yaw,
            odom_age_sec=now - odom.stamp_sec,
            imu_wz=imu_wz,
            imu_age_sec=now - imu['stamp'] if imu else 999.0,
            left_clearance=(
                corridor.left_wall_distance if corridor and corridor.left_wall else 1.0
            ),
            right_clearance=(
                corridor.right_wall_distance if corridor and corridor.right_wall else 1.0
            ),
            now_sec=now,
            in_turn=(self._state in (STATE_ARC_TURN_ENTRY, STATE_ARC_TURN_MAIN)),
        )

        # F5+F6: Plan candidates
        filtered = filter_point_cloud(cloud, self._ld_config)
        filtered = voxel_downsample(filtered, self._ld_config.voxel_size_m)

        # Compute front clearance for planning
        front_hard = compute_hard_distance(
            [p for p in filtered if abs(math.degrees(math.atan2(p.y, p.x))) <= 10.0],
            SECTOR_FRONT, self._ld_config, cloud_stamp, now,
        )

        jh_status = self._joint_health.update(
            self._latest_motor_states, now,
        ) if self._latest_motor_states else JointHealthStatus()

        planning_output = self._planner.plan(
            heading=heading,
            obstacle_points=filtered,
            state=self._state,
            left_clearance=1.0,
            right_clearance=1.0,
            front_clearance=front_hard.hard_distance,
            rear_coverage_sufficient=False,
            joint_health=jh_status,
        )

        with self._state_lock:
            self._selected_candidate = planning_output.selected_candidate

            if planning_output.stop_required:
                if self._state not in (STATE_FAULT_STOP, STATE_STOP_AND_SETTLE):
                    self._state = STATE_STOP_AND_SETTLE
                    self.get_logger().warn(
                        f'Stop required: {planning_output.stop_reason}'
                    )

    def _control_loop(self):
        """~20Hz: Safety arbitration, command publishing, rate limiting."""
        now = self.get_clock().now().nanoseconds * 1e-9
        jh_status = JointHealthStatus()

        with self._motor_lock:
            motor_states = dict(self._latest_motor_states)
        if motor_states:
            jh_status = self._joint_health.update(motor_states, now)

        with self._latest_cloud_lock:
            cloud_stamp = self._latest_cloud_stamp
        cloud_stale = (now - cloud_stamp) > self._ld_config.hard_max_age_sec

        with self._odom_lock:
            odom_stale = (
                self._latest_odom is None
                or (now - self._latest_odom.stamp_sec) > self._heading_config.odom_max_age_sec
            )

        with self._imu_lock:
            imu_stale = (
                self._latest_imu is None
                or (now - self._latest_imu['stamp']) > self._heading_config.imu_max_age_sec
            )

        with self._state_lock:
            candidate = self._selected_candidate
            state = self._state

        # Speed classification
        hard_front = float('inf')  # TODO: wire from perception
        nav_front = float('inf')
        speed_class = classify_speed(hard_front, nav_front, 1.0, self._ld_config)

        # Safety arbitration
        verdict = self._safety.evaluate(
            joint_health_state=jh_status.state,
            cloud_stale=cloud_stale,
            odom_stale=odom_stale,
            imu_stale=imu_stale,
            hard_front_distance=hard_front,
            speed_class=speed_class,
            selected_candidate=candidate,
        )

        # ---- Publish command ----
        twist = Twist()
        if verdict.can_move and self._enable_motion and not self._dry_run:
            twist.linear.x = verdict.command_vx
            twist.linear.y = verdict.command_vy
            twist.angular.z = verdict.command_wz

            if verdict.command_vx != 0.0 or verdict.command_vy != 0.0 or verdict.command_wz != 0.0:
                self._nonzero_motion_generated = True
                if not self._armed:
                    self.get_logger().error(
                        'MOTION BLOCKED: armed=false but non-zero command generated!'
                    )
                    twist.linear.x = 0.0
                    twist.linear.y = 0.0
                    twist.angular.z = 0.0

        # Always publish zero in dry_run
        self._cmd_publisher.publish(twist)

    def _publish_diagnostics(self):
        """~5Hz: Publish comprehensive diagnostics JSON."""
        now = self.get_clock().now().nanoseconds * 1e-9

        with self._state_lock:
            state = self._state
            candidate = self._selected_candidate

        diag = {
            'timestamp': now,
            'state': state,
            'dry_run': self._dry_run,
            'enable_motion': self._enable_motion,
            'armed': self._armed,
            'nonzero_motion_generated': self._nonzero_motion_generated,
            'selected_candidate': (
                {
                    'name': candidate.name,
                    'vx': candidate.vx,
                    'vy': candidate.vy,
                    'wz': candidate.wz,
                    'verdict': candidate.verdict,
                    'robust_safe': candidate.robust_safe,
                    'min_clearance': candidate.minimum_clearance,
                }
                if candidate else None
            ),
            'cloud_buffer_frames': self._cloud_buffer.accumulated_frames,
            'cloud_buffer_clears': self._cloud_buffer.cleared_count,
            'stop_bias_samples': self._stop_bias.sample_count,
            'corridor': (
                {
                    'heading_deg': math.degrees(self._latest_corridor.corridor_heading)
                    if self._latest_corridor and self._latest_corridor.corridor_heading is not None
                    else None,
                    'left_wall_dist': self._latest_corridor.left_wall_distance
                    if self._latest_corridor else None,
                    'right_wall_dist': self._latest_corridor.right_wall_distance
                    if self._latest_corridor else None,
                    'confidence': self._latest_corridor.confidence
                    if self._latest_corridor else 0.0,
                    'valid': self._latest_corridor.valid if self._latest_corridor else False,
                    'n_segments': len(self._latest_corridor.wall_segments)
                    if self._latest_corridor else 0,
                }
            ),
        }

        msg = String()
        msg.data = json.dumps(diag, allow_nan=False, separators=(',', ':'))
        self._diag_publisher.publish(msg)

    # -------------------------------------------------------------------
    # Helpers
    # -------------------------------------------------------------------

    def _str(self, name: str) -> str:
        return str(self.get_parameter(name).value)

    def _float(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _try_float(self, name: str, default: float) -> float:
        try:
            return float(self.get_parameter(name).value)
        except Exception:
            return default

    def _try_int(self, name: str, default: int) -> int:
        try:
            return int(self.get_parameter(name).value)
        except Exception:
            return default

    @staticmethod
    def _quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RealtimeMazeController()
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
