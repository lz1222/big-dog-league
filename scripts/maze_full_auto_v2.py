#!/usr/bin/env python3
"""MAZE_RUNTIME_STABILITY_V1 — 迷宫全自主控制器（重构版）。

架构变更：
- Node + MultiThreadedExecutor 替代 spin_once 循环
- SensorSnapshot 线程安全数据共享
- Timer 控制循环（20Hz planning, 50Hz publish）
- STALE 传感器保护
- 地面过滤直方图估计
- OccupancyGrid 每帧清空
- 侧向 UNKNOWN 语义
- 拐角证据累积
- 状态机完整实现
- Dry Run 模式（默认开启，禁止真机运动）
"""

import math
import os
import signal
import sys
import time
import threading
from copy import deepcopy
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import List, Optional, Tuple

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from nav_msgs.msg import Odometry

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src', 'rk_maze'))
from rk_maze.lidar_distance_core import (
    LidarDistanceConfig, Point3D,
    filter_point_cloud, voxel_downsample, compute_hard_distance,
    SECTOR_FRONT,
)
from rk_maze.lidar_wall_extractor import LidarWallExtractor
from rk_maze.local_occupancy_grid import LocalGridConfig, LocalOccupancyGrid
from rk_maze.heading_controller import HeadingController, HeadingControllerConfig


# ============================================================================
# 配置
# ============================================================================

ROUTE = ['LEFT', 'LEFT', 'RIGHT', 'RIGHT', 'LEFT']
TURN_YAWS = {'LEFT': +90.0, 'RIGHT': -90.0}


@dataclass(frozen=True)
class MazeConfig:
    """迷宫控制器全部可调参数。"""
    # 路线
    route: tuple = ('LEFT', 'LEFT', 'RIGHT', 'RIGHT', 'LEFT')

    # 速度（dry_run 时强制零）
    vx_cruise: float = 0.25
    vx_slow: float = 0.15
    vx_approach: float = 0.08
    vx_turn: float = 0.10
    wz_turn: float = 0.50

    # 距离阈值（米，从机身算）
    stop_dist: float = 0.35
    emerg_dist: float = 0.20
    corner_front_max: float = 0.40
    side_blocked_max: float = 0.25

    # 拐角检测
    opening_point_density_drop: float = 0.40
    opening_free_space_min: float = 0.30
    corner_confirm_frames: int = 5
    corridor_reacquire_frames: int = 10
    reacquire_heading_max_deg: float = 8.0

    # 方向修正
    heading_p_gain: float = 0.02
    heading_max_wz: float = 0.15

    # 传感器 STALE（秒）
    cloud_warn_age: float = 1.0
    # 超过 stop_age 立即停止规划和输出零命令；不允许沿用旧点云运动。
    cloud_stop_age: float = 1.0
    odom_warn_age: float = 0.5
    odom_stop_age: float = 1.0
    imu_warn_age: float = 0.3
    imu_stop_age: float = 0.6
    # IMU 只用于转弯方向/阻尼/停稳和姿态异常保护；不用于持续侧偏补偿。
    turn_imu_min_wz: float = 0.04
    imu_settle_wz: float = 0.05
    turn_overshoot_deg: float = 8.0
    imu_yaw_rate_fault: float = 1.50
    imu_roll_pitch_fault_deg: float = 30.0
    # 真机观测到点云可短暂中断约 5.3 秒。期间始终零速；仅超过此时长才
    # 把已有运行上下文锁为故障，避免恢复后在未知位置继续执行。
    sensor_fault_latch_age: float = 6.0
    sensor_recovery_confirm_frames: int = 10

    # 命令看门狗（秒）
    cmd_publish_rate: float = 50.0
    cmd_max_age: float = 0.20
    plan_rate: float = 20.0

    # 安全
    dry_run: bool = True
    enable_motion: bool = False
    armed: bool = False

    # LiDAR
    obstacle_z_min: float = -0.40
    obstacle_z_max: float = 0.80
    ground_z_min: float = -0.50
    ground_z_max: float = -0.42
    min_cluster_points: int = 3
    voxel_size: float = 0.02
    # hard_distance 的贪心聚类代价随点数急增；只对正前方聚类输入做确定性
    # 限流，防止感知负载拖慢 STALE 与命令看门狗。
    front_cluster_max_points: int = 60

    # 侧向扇区角度
    left_opening_deg: tuple = (30.0, 70.0)
    right_opening_deg: tuple = (-70.0, -30.0)
    left_wall_deg: tuple = (70.0, 110.0)
    right_wall_deg: tuple = (-110.0, -70.0)

    # OccupancyGrid
    grid_mode: str = 'clear_per_frame'  # 'clear_per_frame' | 'odom_compensated'
    # 当前 Python RANSAC 在实测点云上会超出控制周期；本阶段先隔离它，
    # 后续仅在完成性能预算后显式开启，不能让低优先级几何阻塞安全线程。
    enable_wall_extraction: bool = False
    # 墙线 RANSAC 的组合复杂度高；该上限让单帧感知可在点云输入周期内完成。
    wall_ransac_sample_limit: int = 40


# ============================================================================
# 传感器快照（不可变）
# ============================================================================

@dataclass(frozen=True)
class SensorSnapshot:
    """一个控制周期内使用的不可变传感器快照。"""
    monotonic: float

    # rclpy 消息对象只在本控制周期解码；订阅回调不得在高频输入下做点云遍历。
    cloud_message: Optional[PointCloud2] = None
    cloud_pts: tuple = ()
    cloud_seq: int = 0
    cloud_header_stamp: float = 0.0
    cloud_frame_id: str = ""
    cloud_point_count: int = 0
    cloud_age: float = float('inf')
    cloud_valid: bool = False
    cloud_fresh: bool = False
    cloud_x_range: tuple = (0.0, 0.0)
    cloud_y_range: tuple = (0.0, 0.0)
    cloud_z_range: tuple = (0.0, 0.0)

    odom_yaw: float = 0.0
    odom_header_stamp: float = 0.0
    odom_age: float = float('inf')
    odom_valid: bool = False
    odom_fresh: bool = False

    imu_wz: float = 0.0
    imu_roll: float = 0.0
    imu_pitch: float = 0.0
    imu_roll_delta: float = 0.0
    imu_pitch_delta: float = 0.0
    imu_attitude_valid: bool = False
    imu_header_stamp: float = 0.0
    imu_age: float = float('inf')
    imu_valid: bool = False
    imu_fresh: bool = False

    @property
    def all_sensors_ok(self) -> bool:
        """只允许使用本控制周期内新鲜的三路数据规划运动。"""
        return self.cloud_fresh and self.odom_fresh and self.imu_fresh


# ============================================================================
# 侧向观测（UNKNOWN 语义）
# ============================================================================

@dataclass(frozen=True)
class SideObservation:
    valid: bool
    wall_range: float = 0.0        # 墙壁点中值 y 距离
    body_clearance: float = 0.0     # 机身净间隙
    point_count: int = 0
    density: float = 0.0
    reason: str = "no_points"


# ============================================================================
# 开口证据
# ============================================================================

@dataclass
class OpeningEvidence:
    direction: str = ""        # 'LEFT' | 'RIGHT'
    point_density_drop: bool = False
    free_space_detected: bool = False
    wall_endpoint_found: bool = False
    consecutive_frames: int = 0
    confirmed: bool = False
    confidence: float = 0.0


# ============================================================================
# 状态机
# ============================================================================

class MazeState(Enum):
    SYSTEM_PRECHECK = auto()
    WAIT_FOR_SENSORS = auto()
    CRUISE = auto()
    CORNER_CANDIDATE = auto()
    TURN_APPROACH = auto()
    ARC_TURN_MAIN = auto()
    TURN_FINE_ALIGN = auto()
    CORRIDOR_REACQUIRE = auto()
    EMERGENCY_STOP = auto()
    RECOVERY_DECISION = auto()
    BACKUP = auto()
    STOP_AND_REASSESS = auto()
    SENSOR_STALE = auto()
    FAULT_STOP = auto()
    DONE = auto()


# ============================================================================
# 主节点
# ============================================================================

class MazeAutoNode(Node):
    """迷宫全自主控制器节点。"""

    def __init__(self, config: MazeConfig):
        super().__init__('maze_controller')

        self.cfg = config
        self._lock = threading.Lock()

        # ---- 传感器缓存 ----
        self._cloud_message: Optional[PointCloud2] = None
        self._cloud_rx_monotonic: float = 0.0
        self._cloud_header_stamp: float = 0.0
        self._cloud_frame_id: str = ""
        self._cloud_seq: int = 0

        self._odom_yaw: float = 0.0
        self._odom_rx_monotonic: float = 0.0
        self._odom_header_stamp: float = 0.0
        self._odom_seq: int = 0

        self._imu_wz: float = 0.0
        self._imu_roll: float = 0.0
        self._imu_pitch: float = 0.0
        # L1 安装坐标未必与机器人水平坐标一致；基准只在首次有效姿态建立。
        self._imu_roll_baseline: Optional[float] = None
        self._imu_pitch_baseline: Optional[float] = None
        self._imu_attitude_valid: bool = False
        self._imu_rx_monotonic: float = 0.0
        self._imu_header_stamp: float = 0.0
        self._imu_seq: int = 0

        # ---- QoS ----
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )

        # ---- 订阅 ----
        self.cloud_sub = self.create_subscription(
            PointCloud2, '/utlidar/cloud_base', self._cb_cloud, sensor_qos)
        self.odom_sub = self.create_subscription(
            Odometry, '/utlidar/robot_odom', self._cb_odom, sensor_qos)
        self.imu_sub = self.create_subscription(
            Imu, '/utlidar/imu', self._cb_imu, sensor_qos)

        # ---- 发布 ----
        self.cmd_pub = self.create_publisher(Twist, '/navigation/cmd_vel', 10)
        self._cmd_timer = self.create_timer(1.0 / config.cmd_publish_rate, self._on_cmd_timer)
        self._desired_twist = Twist()
        self._desired_twist_ts: float = 0.0

        # ---- 帧计数 (必须在 plan 线程之前) ----
        self._frame: int = 0

        # ---- 状态 (必须在 plan 线程之前) ----
        self.state = MazeState.SYSTEM_PRECHECK
        self.turn_idx: int = 0
        self.turn_dir: str = ""
        self.rel_yaw_initial: Optional[float] = None
        self.turn_start_yaw: float = 0.0
        self.turn_expected_sign: float = 0.0
        self.total_turn_yaw: float = 0.0
        self.last_imu_stamp: float = 0.0
        self._corner_candidate_count: int = 0
        self._reacquire_count: int = 0
        self._reacquire_heading_ok_count: int = 0
        self._backup_distance: float = 0.0
        self._backup_start_odom: Optional[float] = None
        self._backup_count: int = 0
        self._state_enter_monotonic: float = 0.0
        self._state_enter_frame: int = 0
        self._last_report: float = 0.0
        self._last_state: Optional[MazeState] = None
        self._last_processed_cloud_seq: int = 0
        # 断流期间保存上下文而不保存旧速度；恢复后必须重新用新点云评估。
        self._stale_resume_state: Optional[MazeState] = None
        self._sensor_stale_started: float = 0.0
        self._sensor_recovery_count: int = 0

        # ---- 感知模块 ----
        self._ld_config = LidarDistanceConfig(
            min_cluster_points=config.min_cluster_points,
            obstacle_z_min_m=config.obstacle_z_min,
            obstacle_z_max_m=config.obstacle_z_max,
            ground_z_min_m=config.ground_z_min,
            ground_z_max_m=config.ground_z_max,
        )
        self._grid_config = LocalGridConfig(
            wall_ransac_sample_limit=config.wall_ransac_sample_limit,
        )
        self._we = LidarWallExtractor(self._grid_config)
        self._hc = HeadingController(HeadingControllerConfig())
        self._grid = LocalOccupancyGrid(self._grid_config)

        self._max_backup_per_corner: int = 2

        # 感知对象必须在规划线程前完成构造。线程若抢先收到首帧点云，会访问
        # 尚不存在的滤波/栅格成员并持续报错，掩盖真正的 DDS 回调状态。
        self._plan_running = True
        self._plan_thread = threading.Thread(target=self._plan_loop, daemon=True)
        self._plan_thread.start()

        # ---- 启动信息 ----
        motion_status = "DRY_RUN (ZERO 运动)" if config.dry_run else (
            "ARMED" if config.armed else "DISABLED")
        self.get_logger().info(
            f'迷宫控制器启动 | 路线: {"-".join(config.route)} | '
            f'运动: {motion_status} | '
            f'cloud_stop_age={config.cloud_stop_age}s | '
            f'wall_extraction={config.enable_wall_extraction}'
        )
        print(f'迷宫全自主 V2 (STABILITY) — {"-".join(config.route)}')
        print(f'运动模式: {motion_status}')
        print(f'状态: {self.state.name}  弯: {self.turn_idx+1}/{len(config.route)}')

    # ========================================================================
    # 传感器回调（仅轻量复制）
    # ========================================================================

    def _cb_cloud(self, msg: PointCloud2):
        """复制原始消息与接收时间，点云解码留给低频规划线程。"""
        now = time.monotonic()
        with self._lock:
            self._cloud_seq += 1
            # 深拷贝 raw buffer 后再离开 ROS 回调，避免底层消息生命周期或下一帧
            # 覆盖影响规划线程；这比逐点解码轻得多，且快照可跨线程安全使用。
            self._cloud_message = deepcopy(msg)
            self._cloud_rx_monotonic = now
            self._cloud_header_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self._cloud_frame_id = msg.header.frame_id

    def _cb_odom(self, msg: Odometry):
        now = time.monotonic()
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        with self._lock:
            self._odom_seq += 1
            self._odom_yaw = yaw
            self._odom_rx_monotonic = now
            self._odom_header_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def _cb_imu(self, msg: Imu):
        """只缓存角速度与姿态；转弯/安全判断在规划线程使用同一快照。"""
        now = time.monotonic()
        q = msg.orientation
        norm_sq = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w
        # 有些设备在姿态未知时发送零四元数；此时保留 yaw-rate，但不伪造 roll/pitch。
        attitude_valid = math.isfinite(norm_sq) and norm_sq > 0.25
        roll = pitch = 0.0
        if attitude_valid:
            roll = math.atan2(2.0 * (q.w * q.x + q.y * q.z),
                              1.0 - 2.0 * (q.x * q.x + q.y * q.y))
            pitch_sin = max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x)))
            pitch = math.asin(pitch_sin)
        with self._lock:
            if attitude_valid and self._imu_roll_baseline is None:
                # 不假设水平时 roll/pitch=0，而是以静止首帧的安装姿态为安全参考。
                self._imu_roll_baseline = roll
                self._imu_pitch_baseline = pitch
            self._imu_seq += 1
            self._imu_wz = msg.angular_velocity.z
            self._imu_roll = roll
            self._imu_pitch = pitch
            self._imu_attitude_valid = attitude_valid
            self._imu_rx_monotonic = now
            self._imu_header_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    # ========================================================================
    # 快照
    # ========================================================================

    def _take_snapshot(self) -> SensorSnapshot:
        now = time.monotonic()
        with self._lock:
            cloud_message = self._cloud_message
            cloud_rx = self._cloud_rx_monotonic
            cloud_stamp = self._cloud_header_stamp
            cloud_fid = self._cloud_frame_id
            cloud_seq = self._cloud_seq
            # width*height 是原始点槽数量；有效点数在规划线程跳过 NaN 后再统计。
            cloud_cnt = (cloud_message.width * cloud_message.height) if cloud_message else 0

            odom_yaw = self._odom_yaw
            odom_rx = self._odom_rx_monotonic
            odom_stamp = self._odom_header_stamp

            imu_wz = self._imu_wz
            imu_roll = self._imu_roll
            imu_pitch = self._imu_pitch
            imu_attitude_valid = self._imu_attitude_valid
            imu_roll_baseline = self._imu_roll_baseline
            imu_pitch_baseline = self._imu_pitch_baseline
            imu_rx = self._imu_rx_monotonic
            imu_stamp = self._imu_header_stamp

        return SensorSnapshot(
            monotonic=now,
            cloud_message=cloud_message,
            cloud_seq=cloud_seq,
            cloud_header_stamp=cloud_stamp,
            cloud_frame_id=cloud_fid,
            cloud_point_count=cloud_cnt,
            cloud_age=now - cloud_rx if cloud_rx > 0 else float('inf'),
            cloud_valid=(cloud_cnt > 0 and now - cloud_rx < self.cfg.cloud_stop_age),
            cloud_fresh=(cloud_cnt > 0 and now - cloud_rx < self.cfg.cloud_stop_age),
            odom_yaw=odom_yaw,
            odom_header_stamp=odom_stamp,
            odom_age=now - odom_rx if odom_rx > 0 else float('inf'),
            odom_valid=(odom_rx > 0 and now - odom_rx < self.cfg.odom_stop_age),
            odom_fresh=(odom_rx > 0 and now - odom_rx < self.cfg.odom_stop_age),
            imu_wz=imu_wz,
            imu_roll=imu_roll,
            imu_pitch=imu_pitch,
            imu_roll_delta=(self._hc._normalize_angle(imu_roll - imu_roll_baseline)
                            if imu_roll_baseline is not None else 0.0),
            imu_pitch_delta=(imu_pitch - imu_pitch_baseline
                             if imu_pitch_baseline is not None else 0.0),
            imu_attitude_valid=imu_attitude_valid,
            imu_header_stamp=imu_stamp,
            imu_age=now - imu_rx if imu_rx > 0 else float('inf'),
            imu_valid=(imu_rx > 0 and now - imu_rx < self.cfg.imu_stop_age),
            imu_fresh=(imu_rx > 0 and now - imu_rx < self.cfg.imu_stop_age),
        )

    # ========================================================================
    # 命令发布 Timer（50Hz）
    # ========================================================================

    def _on_cmd_timer(self):
        now = time.monotonic()
        cmd_age = now - self._desired_twist_ts if self._desired_twist_ts > 0 else float('inf')

        if cmd_age > self.cfg.cmd_max_age:
            # 命令过期 → 零
            self._desired_twist = Twist()
            self._desired_twist_ts = now
            self.get_logger().warn('命令过期，发布 ZERO', throttle_duration_sec=2.0)

        # Dry run 强制零
        if self.cfg.dry_run:
            self.cmd_pub.publish(Twist())
        elif not self.cfg.armed:
            self.cmd_pub.publish(Twist())
        else:
            self.cmd_pub.publish(self._desired_twist)

    def _set_command(self, tw: Twist, reason: str = ""):
        """线程安全设置期望命令。"""
        self._desired_twist = tw
        self._desired_twist_ts = time.monotonic()

    # ========================================================================
    # 规划 Timer（20Hz）
    # ========================================================================

    def _plan_loop(self):
        """后台 planning 线程（替代 ROS2 Timer）。"""
        period = 1.0 / self.cfg.plan_rate
        while self._plan_running:
            try:
                self._on_plan_timer_impl()
            except Exception as e:
                import traceback
                print(f'PLAN_CRASH: {e}', flush=True)
                traceback.print_exc()
            time.sleep(period)

    def _on_plan_timer_impl(self):
        self._frame += 1
        snap = self._take_snapshot()
        now = snap.monotonic

        # 终态不再解析最后一帧缓存。这样日志和后续维护都不会误把陈旧点云
        # 当成故障后的有效环境信息；命令看门狗仍会持续得到显式零速度。
        if self.state in (MazeState.FAULT_STOP, MazeState.DONE):
            self._set_command(Twist(), reason='terminal_stop')
            return

        # ---- STALE 检查（P0） ----
        # 新鲜度门限与“中断多久后锁故障”分离：短断流也必须停止，不能拿
        # 上一帧点云继续规划；但恢复时保留路线和上下文，不重置为首弯。
        if not snap.all_sensors_ok:
            self._enter_sensor_stale(snap)
            self._set_command(Twist(), reason='sensor_stale')
            self._maybe_report(snap, "SENSOR_STALE")
            return

        imu_fault = self._imu_safety_fault(snap)
        if imu_fault:
            # 姿态大幅异常或非转弯级 yaw-rate 不能靠航向补偿“拉回来”，必须锁停。
            self.get_logger().error(imu_fault)
            self._transition(MazeState.FAULT_STOP)
            self._set_command(Twist(), reason='imu_safety_fault')
            return

        if self.state == MazeState.SENSOR_STALE:
            self._set_command(Twist(), reason='sensor_recovery_confirming')
            self._recover_from_sensor_stale(snap)
            return

        # 高成本的栅格与墙线提取只处理新 PointCloud。控制线程仍以 20Hz
        # 检查 freshness；在下一帧到来前只刷新已有安全命令，超龄后由上方
        # STALE 分支无条件清零，避免同一帧被重复 RANSAC 而饿死回调。
        if snap.cloud_seq == self._last_processed_cloud_seq:
            self._set_command(self._desired_twist, reason='awaiting_next_cloud')
            return
        self._last_processed_cloud_seq = snap.cloud_seq

        # ---- 处理点云 ----
        cloud = self._decode_cloud_for_plan(snap.cloud_message)
        filt = voxel_downsample(filter_point_cloud(cloud, self._ld_config), self.cfg.voxel_size)

        # 前向扇区
        fp = [p for p in filt if abs(math.degrees(math.atan2(p.y, p.x))) <= 30 and p.z > self.cfg.obstacle_z_min]
        if not fp:
            self._set_command(Twist(), reason='no_front_points')
            self._maybe_report(snap, "NO_FRONT_POINTS")
            return

        if len(fp) > self.cfg.front_cluster_max_points:
            stride = math.ceil(len(fp) / self.cfg.front_cluster_max_points)
            fp = fp[::stride]

        sd = compute_hard_distance(fp, SECTOR_FRONT, self._ld_config, now, now)
        if not sd.valid:
            self._set_command(Twist(), reason='front_invalid')
            self._maybe_report(snap, "FRONT_INVALID")
            return

        front = sd.front_clearance
        rel_yaw = self._compute_rel_yaw(snap)

        heading_deg = 0.0
        if self.cfg.enable_wall_extraction:
            # OccupancyGrid MODE A：每帧重建，禁止累计不同 base_link 时刻的点。
            self._grid = LocalOccupancyGrid(self._grid_config)
            for pt in filt:
                self._grid.mark_occupied(pt.x, pt.y)

            model = self._we.build_corridor_model(
                self._we.extract(self._grid, now).wall_segments, now)
            heading_deg = math.degrees(model.corridor_heading) if model.corridor_heading else 0.0

        # 侧向观测
        left_obs = self._compute_side(filt, self.cfg.left_opening_deg, 'left')
        right_obs = self._compute_side(filt, self.cfg.right_opening_deg, 'right')

        # 日志
        self._maybe_report(snap, f"front={front:.2f} L={left_obs.body_clearance:.2f} R={right_obs.body_clearance:.2f} hdg={heading_deg:+.1f}")

        # ---- 状态机 ----
        tw = Twist()
        log_event = None

        if self.state == MazeState.SYSTEM_PRECHECK:
            self._transition(MazeState.WAIT_FOR_SENSORS)
            tw = Twist()

        elif self.state == MazeState.WAIT_FOR_SENSORS:
            if snap.cloud_valid and snap.odom_valid and snap.imu_valid and front > 0.01:
                self._transition(MazeState.CRUISE)
            tw = Twist()

        elif self.state == MazeState.CRUISE:
            tw = self._handle_cruise(front, left_obs, right_obs, heading_deg, rel_yaw)

        elif self.state == MazeState.CORNER_CANDIDATE:
            tw = self._handle_corner_candidate(front, left_obs, right_obs, heading_deg)

        elif self.state == MazeState.TURN_APPROACH:
            tw = self._handle_turn_approach(front, heading_deg)

        elif self.state == MazeState.ARC_TURN_MAIN:
            tw = self._handle_arc_turn(snap)

        elif self.state == MazeState.TURN_FINE_ALIGN:
            tw = self._handle_turn_fine_align(snap, heading_deg)

        elif self.state == MazeState.CORRIDOR_REACQUIRE:
            tw = self._handle_corridor_reacquire(snap, front, left_obs, right_obs, heading_deg)

        elif self.state == MazeState.EMERGENCY_STOP:
            tw = Twist()
            self._handle_emergency(snap, front)

        elif self.state == MazeState.RECOVERY_DECISION:
            tw = self._handle_recovery(snap, front, left_obs, right_obs)

        elif self.state == MazeState.BACKUP:
            tw = self._handle_backup(snap, front)

        elif self.state == MazeState.STOP_AND_REASSESS:
            tw = Twist()
            self._handle_stop_and_reassess(snap, front, left_obs, right_obs)

        elif self.state == MazeState.FAULT_STOP:
            tw = Twist()

        elif self.state == MazeState.DONE:
            tw = Twist()

        if self._frame % 20 == 0:
            self._maybe_report(snap, f"front={front:.2f} L={left_obs.body_clearance:.2f} R={right_obs.body_clearance:.2f} hdg={heading_deg:+.1f}")
        self._set_command(tw)

    # ========================================================================
    # CRUISE
    # ========================================================================

    def _handle_cruise(self, front: float, left: SideObservation, right: SideObservation,
                       heading_deg: float, rel_yaw: float) -> Twist:
        tw = Twist()

        if self.turn_idx >= len(self.cfg.route):
            self._transition(MazeState.DONE)
            print('全部弯完成!')
            return tw

        # 前方紧急
        if 0.01 < front < self.cfg.emerg_dist:
            self._transition(MazeState.EMERGENCY_STOP)
            print(f'EMERGENCY front={front:.2f}m')
            return tw

        # 前方近 + 两侧堵 → BACKUP
        side_blocked = (not left.valid or left.body_clearance < self.cfg.side_blocked_max) and \
                       (not right.valid or right.body_clearance < self.cfg.side_blocked_max)

        if 0.01 < front < self.cfg.stop_dist and side_blocked:
            self._transition(MazeState.BACKUP)
            print(f'BACKUP front={front:.2f}m')
            return tw

        # 拐角候选
        if 0.01 < front < self.cfg.corner_front_max and not side_blocked:
            self._transition(MazeState.CORNER_CANDIDATE)
            self._corner_candidate_count = 0
            self.turn_dir = self.cfg.route[self.turn_idx]
            return tw

        # 速度分级
        if front > 0.80:
            vx = self.cfg.vx_cruise
        elif front > 0.50:
            vx = self.cfg.vx_slow
        elif front > self.cfg.stop_dist:
            vx = self.cfg.vx_approach
        else:
            vx = 0.0

        tw.linear.x = vx
        tw.angular.z = max(-self.cfg.heading_max_wz,
                           min(self.cfg.heading_max_wz, heading_deg * self.cfg.heading_p_gain))
        return tw

    # ========================================================================
    # CORNER
    # ========================================================================

    def _handle_corner_candidate(self, front: float, left: SideObservation,
                                  right: SideObservation, heading_deg: float) -> Twist:
        tw = Twist()
        tw.linear.x = self.cfg.vx_approach

        expected = self.turn_dir
        opening = left if expected == 'LEFT' else right

        if opening.valid and opening.body_clearance > self.cfg.opening_free_space_min:
            self._corner_candidate_count += 1
            if self._corner_candidate_count >= self.cfg.corner_confirm_frames:
                self._transition(MazeState.TURN_APPROACH)
                print(f'拐角确认! {expected} opening={opening.body_clearance:.2f}m')
                self.turn_start_yaw = self._last_odom_yaw()
                self.turn_expected_sign = 1.0 if expected == 'LEFT' else -1.0
                self.total_turn_yaw = 0.0
                self._backup_count = 0
        else:
            self._corner_candidate_count = max(0, self._corner_candidate_count - 1)
            if self._corner_candidate_count == 0 and front > self.cfg.stop_dist:
                self._transition(MazeState.CRUISE)

        return tw

    # ========================================================================
    # TURN
    # ========================================================================

    def _handle_turn_approach(self, front: float, heading_deg: float) -> Twist:
        tw = Twist()
        tw.linear.x = self.cfg.vx_approach
        tw.angular.z = self.turn_expected_sign * 0.15  # 缓慢预转

        if abs(self._compute_turn_progress()) > 5.0:
            self._transition(MazeState.ARC_TURN_MAIN)
        return tw

    def _handle_arc_turn(self, snap: SensorSnapshot) -> Twist:
        tw = Twist()
        progress = self._compute_turn_progress()
        target = abs(TURN_YAWS[self.turn_dir])

        # 方向检查
        if (self.turn_expected_sign > 0 and progress < -3.0) or \
           (self.turn_expected_sign < 0 and progress > 3.0):
            self.get_logger().error(f'WRONG_TURN_DIRECTION! progress={progress:.1f}')
            self._transition(MazeState.FAULT_STOP)
            return tw

        # Odom 给出几何进度，IMU 给出瞬时方向；两者冲突时优先锁停而非猜测续转。
        if abs(snap.imu_wz) >= self.cfg.turn_imu_min_wz and \
                snap.imu_wz * self.turn_expected_sign < 0.0:
            self.get_logger().error(
                f'WRONG_TURN_DIRECTION_IMU! wz={snap.imu_wz:.2f}')
            self._transition(MazeState.FAULT_STOP)
            return tw

        remaining = target - abs(progress)
        if remaining < -self.cfg.turn_overshoot_deg:
            self.get_logger().error(
                f'TURN_OVERSHOOT! progress={progress:.1f} target={target:.1f}')
            self._transition(MazeState.FAULT_STOP)
            return tw
        if remaining < 15.0:
            self._transition(MazeState.TURN_FINE_ALIGN)
            return tw

        tw.linear.x = self.cfg.vx_turn
        tw.angular.z = self.turn_expected_sign * self.cfg.wz_turn
        return tw

    def _handle_turn_fine_align(self, snap: SensorSnapshot, heading_deg: float) -> Twist:
        tw = Twist()
        progress = self._compute_turn_progress()
        target = abs(TURN_YAWS[self.turn_dir])
        remaining = target - abs(progress)

        if remaining < 3.0 and abs(snap.imu_wz) <= self.cfg.imu_settle_wz:
            # 进入走廊重捕获，不立即推进 route_progress
            self._transition(MazeState.CORRIDOR_REACQUIRE)
            self._reacquire_count = 0
            self._reacquire_heading_ok_count = 0
            print(f'转弯完成，进入走廊重捕获')
            return tw

        # 微调
        tw.linear.x = 0.05
        # 接近目标时以 IMU yaw-rate 阻尼，降低过冲；仍受固定转弯方向约束。
        tw.angular.z = self.turn_expected_sign * 0.20 - 0.25 * snap.imu_wz
        tw.angular.z = max(-0.20, min(0.20, tw.angular.z))
        return tw

    # ========================================================================
    # CORRIDOR REACQUIRE
    # ========================================================================

    def _handle_corridor_reacquire(self, snap: SensorSnapshot, front: float,
                                    left: SideObservation, right: SideObservation,
                                    heading_deg: float) -> Twist:
        tw = Twist()
        tw.linear.x = self.cfg.vx_slow

        self._reacquire_count += 1
        if abs(heading_deg) < self.cfg.reacquire_heading_max_deg:
            self._reacquire_heading_ok_count += 1

        walls_seen = left.valid and right.valid and left.point_count > 5 and right.point_count > 5

        if self._reacquire_heading_ok_count >= self.cfg.corridor_reacquire_frames and walls_seen:
            self.turn_idx += 1
            self._transition(MazeState.CRUISE)
            print(f'走廊重捕获完成! 弯{self.turn_idx} → CRUISE')
            return tw

        if self._reacquire_count > 200:
            self.get_logger().error('走廊重捕获超时')
            self._transition(MazeState.FAULT_STOP)
            return tw

        return tw

    # ========================================================================
    # EMERGENCY / BACKUP / RECOVERY
    # ========================================================================

    def _handle_emergency(self, snap: SensorSnapshot, front: float):
        """紧急停止后决策。"""
        elapsed = snap.monotonic - self._state_enter_monotonic
        if elapsed > 0.5 and abs(snap.imu_wz) <= self.cfg.imu_settle_wz:
            self._transition(MazeState.RECOVERY_DECISION)

    def _handle_recovery(self, snap: SensorSnapshot, front: float,
                         left: SideObservation, right: SideObservation) -> Twist:
        tw = Twist()
        # 紧急恢复只能接受路线期望侧；任意一侧有开口不能覆盖固定路线约束。
        expected = self._expected_turn_direction()
        opening = left if expected == 'LEFT' else right if expected == 'RIGHT' else None
        if opening and opening.valid and \
                opening.body_clearance > self.cfg.opening_free_space_min:
            self._prepare_turn(expected)
            self._transition(MazeState.TURN_APPROACH)
        else:
            # 后方覆盖未验收，恢复期不允许以 BACKUP 猜测退出空间。
            self._transition(MazeState.STOP_AND_REASSESS)
        return tw

    def _handle_backup(self, snap: SensorSnapshot, front: float) -> Twist:
        tw = Twist()

        # 安全检查：无后方感知
        self._transition(MazeState.STOP_AND_REASSESS)
        return tw

    def _handle_stop_and_reassess(self, snap: SensorSnapshot, front: float,
                                   left: SideObservation, right: SideObservation) -> Twist:
        tw = Twist()
        # 如果 front 已清 → 重新巡航
        if front > self.cfg.stop_dist:
            self._transition(MazeState.CRUISE)
        elif front > self.cfg.emerg_dist:
            # 停稳后仍只允许进入路线期望的开口，防止反方向转弯。
            expected = self._expected_turn_direction()
            opening = left if expected == 'LEFT' else right if expected == 'RIGHT' else None
            if opening and opening.valid and \
                    opening.body_clearance > self.cfg.opening_free_space_min:
                self._prepare_turn(expected)
                self._transition(MazeState.TURN_APPROACH)
            else:
                self._transition(MazeState.FAULT_STOP)
                self.get_logger().error('STOP_AND_REASSESS: 无安全出口')
        else:
            self._transition(MazeState.EMERGENCY_STOP)
        return tw

    # ========================================================================
    # 辅助方法
    # ========================================================================

    def _expected_turn_direction(self) -> Optional[str]:
        """返回尚未完成弯道的固定路线方向；越界时拒绝生成新转弯。"""
        if 0 <= self.turn_idx < len(self.cfg.route):
            return self.cfg.route[self.turn_idx]
        return None

    def _prepare_turn(self, direction: str):
        """在任何安全入口统一冻结本次转弯的 Odom 基准和方向符号。"""
        self.turn_dir = direction
        self.turn_expected_sign = 1.0 if direction == 'LEFT' else -1.0
        self.turn_start_yaw = self._last_odom_yaw()
        self.total_turn_yaw = 0.0

    @staticmethod
    def _decode_cloud_for_plan(message: Optional[PointCloud2]) -> List[Point3D]:
        """在规划线程一次性解码当前快照，过滤 NaN 以保持控制周期输入一致。"""
        if message is None:
            return []
        return [
            Point3D(x=float(point[0]), y=float(point[1]), z=float(point[2]))
            for point in pc2.read_points(
                message, field_names=('x', 'y', 'z'), skip_nans=True)
            if all(math.isfinite(float(value)) for value in point)
        ]

    @staticmethod
    def _resume_target_after_sensor_stale(previous: Optional[MazeState]) -> Optional[MazeState]:
        """将可安全恢复的状态映射到重新感知后的入口。

        转弯主段中断后缺少机器人实际停止姿态和墙面连续性的保证，绝不能根据旧
        yaw 直接续转；因此返回 None 使调用方锁停并等待人工复核。仅
        TURN_APPROACH 可退回 RECOVERY_DECISION：该入口会用恢复后的新点云
        重新验证固定路线一侧的开口，并重新冻结 Odom 基准，而非续用旧转弯进度。
        """
        if previous in (MazeState.SYSTEM_PRECHECK, MazeState.WAIT_FOR_SENSORS):
            return MazeState.WAIT_FOR_SENSORS
        if previous in (MazeState.CRUISE, MazeState.CORNER_CANDIDATE):
            return MazeState.CRUISE
        if previous == MazeState.TURN_APPROACH:
            return MazeState.RECOVERY_DECISION
        return None

    def _enter_sensor_stale(self, snap: SensorSnapshot):
        """首次断流只保存状态并输出零速度；运行中长断流则 fail-closed。"""
        if self.state != MazeState.SENSOR_STALE:
            self._stale_resume_state = self.state
            self._sensor_stale_started = snap.monotonic
            self._sensor_recovery_count = 0
            self._transition(MazeState.SENSOR_STALE)

        elapsed = snap.monotonic - self._sensor_stale_started
        # 启动时从未收到数据可一直等待；只有已有运行上下文的长中断才锁故障。
        if self._stale_resume_state not in (None, MazeState.SYSTEM_PRECHECK,
                                            MazeState.WAIT_FOR_SENSORS) and \
                elapsed >= self.cfg.sensor_fault_latch_age:
            self.get_logger().error(
                f'传感器断流 {elapsed:.1f}s，保持零速度并锁定 FAULT_STOP')
            self._transition(MazeState.FAULT_STOP)

    def _recover_from_sensor_stale(self, snap: SensorSnapshot):
        """连续新鲜帧后恢复到安全入口，所有恢复过程始终维持零速度。"""
        self._sensor_recovery_count += 1
        if self._sensor_recovery_count < self.cfg.sensor_recovery_confirm_frames:
            self._maybe_report(
                snap,
                f'SENSOR_RECOVERY {self._sensor_recovery_count}/'
                f'{self.cfg.sensor_recovery_confirm_frames}',
            )
            return

        previous = self._stale_resume_state
        target = self._resume_target_after_sensor_stale(previous)
        self._stale_resume_state = None
        self._sensor_recovery_count = 0
        if target is None:
            self.get_logger().error(
                f'传感器在 {previous.name if previous else "UNKNOWN"} 阶段中断；'
                '恢复后拒绝自动续转，进入 FAULT_STOP')
            self._transition(MazeState.FAULT_STOP)
            return

        # 候选证据必须由恢复后的连续新点云重新建立，避免把断流前的开口误用。
        self._corner_candidate_count = 0
        self._transition(target)
        self.get_logger().info(
            f'传感器恢复确认完成：{previous.name if previous else "UNKNOWN"} → {target.name}')

    def _compute_rel_yaw(self, snap: SensorSnapshot) -> float:
        if self.rel_yaw_initial is None and snap.odom_valid:
            self.rel_yaw_initial = snap.odom_yaw
        if self.rel_yaw_initial is not None:
            return self._hc._normalize_angle(snap.odom_yaw - self.rel_yaw_initial)
        return 0.0

    def _imu_safety_fault(self, snap: SensorSnapshot) -> str:
        """将 IMU 限定为异常保护，不把姿态用于持续航向偏置补偿。

        零四元数代表姿态未知，已由 imu fresh 单独看门，不在这里误报倾倒；
        有效姿态与静止安装基准的偏差越阈值才锁停。yaw-rate 阈值高于正常转弯
        指令，防止撞击或坐标符号错误时继续输出运动。
        """
        if abs(snap.imu_wz) > self.cfg.imu_yaw_rate_fault:
            return f'IMU_YAW_RATE_FAULT wz={snap.imu_wz:.2f}'
        if snap.imu_attitude_valid:
            roll_deg = abs(math.degrees(snap.imu_roll_delta))
            pitch_deg = abs(math.degrees(snap.imu_pitch_delta))
            if max(roll_deg, pitch_deg) > self.cfg.imu_roll_pitch_fault_deg:
                return f'IMU_ATTITUDE_FAULT roll={roll_deg:.1f} pitch={pitch_deg:.1f}'
        return ''

    def _last_odom_yaw(self) -> float:
        with self._lock:
            return self._odom_yaw

    def _compute_turn_progress(self) -> float:
        current = self._last_odom_yaw()
        return self._hc._normalize_angle(current - self.turn_start_yaw)

    def _compute_side(self, filt: List[Point3D], angle_range: Tuple[float, float],
                      side: str) -> SideObservation:
        """计算侧向观测（UNKNOWN 语义）。"""
        lo, hi = angle_range
        if side == 'right':
            pts = [-p.y for p in filt if lo < math.degrees(math.atan2(p.y, p.x)) < hi]
        else:
            pts = [p.y for p in filt if lo < math.degrees(math.atan2(p.y, p.x)) < hi]

        if not pts:
            return SideObservation(valid=False, reason="no_points")

        median_y = sorted(pts)[len(pts) // 2]
        # 机身半宽 + 安全边距
        body_half = 0.18
        margin = 0.03
        clearance = median_y - body_half - margin

        return SideObservation(
            valid=True,
            wall_range=median_y,
            body_clearance=clearance,
            point_count=len(pts),
            density=len(pts) / ((hi - lo) * 0.01745),  # 粗略
            reason="ok",
        )

    def _transition(self, new_state: MazeState):
        if self.state != new_state:
            self.get_logger().info(f'{self.state.name} → {new_state.name}')
            print(f'[{self._frame}] {self.state.name} → {new_state.name}')
            self.state = new_state
            self._state_enter_monotonic = time.monotonic()
            self._state_enter_frame = self._frame

    def _maybe_report(self, snap: SensorSnapshot, detail: str):
        now = snap.monotonic
        if now - self._last_report < 1.5:
            return
        self._last_report = now
        age_str = ""
        if snap.cloud_age > self.cfg.cloud_warn_age:
            age_str += f" CLOUD_AGE={snap.cloud_age:.1f}s"
        if snap.odom_age > self.cfg.odom_warn_age:
            age_str += f" ODOM_AGE={snap.odom_age:.1f}s"
        if snap.imu_age > self.cfg.imu_warn_age:
            age_str += f" IMU_AGE={snap.imu_age:.1f}s"
        print(f'{self.state.name} {detail}{age_str}', flush=True)


# ============================================================================
# 入口
# ============================================================================

def main():
    config = MazeConfig(
        dry_run=True,
        enable_motion=False,
        armed=False,
    )

    rclpy.init()
    node = MazeAutoNode(config)
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    print("\nDry Run 模式 — 所有运动输出强制为零。Ctrl-C 退出。\n")

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n中断。")
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        print("结束。")


if __name__ == '__main__':
    main()
