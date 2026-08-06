#!/usr/bin/env python3

"""B2.1 第一左弯只读规划节点：输出 JSON，绝不发送运动命令。"""

from dataclasses import fields
import json
import math
import threading
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
from std_msgs.msg import Bool, String

from maze_first_turn_core import (
    DynamicFootprint,
    FirstTurnDryRunStateMachine,
    FirstTurnTrajectoryPlanner,
    LocalMapBuilder,
    LocalMapConfig,
    MotionPrimitive,
    PlannerConfig,
    PRIMITIVE_FINE_LEFT_ARC,
    PRIMITIVE_FORWARD,
    PRIMITIVE_LEFT_ARC,
    PRIMITIVE_LEFT_ARC_OUTSIDE,
    PRIMITIVE_OUTSIDE_DIAGONAL,
    PRIMITIVE_REVERSE,
    SafetyContext,
    SECTOR_NAMES,
    SectorFreshnessTracker,
    normalize_angle,
)
from maze_perception_core import quaternion_to_rpy


PRIMITIVE_PREFIXES = {
    PRIMITIVE_FORWARD: 'forward_short',
    PRIMITIVE_OUTSIDE_DIAGONAL: 'outside_diagonal_short',
    PRIMITIVE_LEFT_ARC: 'left_arc',
    PRIMITIVE_LEFT_ARC_OUTSIDE: 'left_arc_outside_vy',
    PRIMITIVE_REVERSE: 'reverse_short',
    PRIMITIVE_FINE_LEFT_ARC: 'fine_left_arc',
}

PRIMITIVE_DEFAULTS = {
    PRIMITIVE_FORWARD: (0.25, 0.0, 0.0, 0.50),
    PRIMITIVE_OUTSIDE_DIAGONAL: (0.18, -0.05, 0.0, 0.50),
    PRIMITIVE_LEFT_ARC: (0.18, 0.0, 0.35, 0.50),
    PRIMITIVE_LEFT_ARC_OUTSIDE: (0.18, -0.04, 0.35, 0.50),
    PRIMITIVE_REVERSE: (-0.18, 0.0, 0.0, 0.40),
    PRIMITIVE_FINE_LEFT_ARC: (0.12, 0.0, 0.20, 0.35),
}


class MazeFirstTurnDryRun(Node):
    """直接消费点云和 Odom，为第一弯生成局部轨迹安全诊断。"""

    def __init__(self):
        super().__init__('maze_first_turn_dry_run')

        self.cloud_topic = self._string_parameter(
            'cloud_topic',
            '/utlidar/cloud_base',
        )
        self.odom_topic = self._string_parameter(
            'odom_topic',
            '/utlidar/robot_odom',
        )
        self.status_topic = self._string_parameter(
            'status_topic',
            '/maze/first_turn/dry_run_status',
        )
        self.watchdog_status_topic = self._string_parameter(
            'watchdog_status_topic',
            '/maze/safety/watchdog_ok',
        )
        self.estop_status_topic = self._string_parameter(
            'estop_status_topic',
            '/maze/safety/estop',
        )
        self.expected_cloud_frame = self._string_parameter(
            'expected_cloud_frame',
            'base_link',
        )
        self.evaluation_rate = self._positive_float_parameter(
            'evaluation_rate',
            2.0,
        )
        self.print_rate = self._positive_float_parameter(
            'print_rate',
            1.0,
        )

        self.map_config = self._load_dataclass(LocalMapConfig)
        self.footprint = self._load_dataclass(DynamicFootprint)
        self.planner_config = self._load_dataclass(PlannerConfig)
        self.map_builder = LocalMapBuilder(self.map_config)
        self.sector_tracker = SectorFreshnessTracker(
            self.planner_config.sector_stale_timeout_sec
        )
        primitives = self._load_primitives()
        self.trajectory_planner = FirstTurnTrajectoryPlanner(
            self.planner_config,
            self.footprint,
            primitives,
        )
        self.state_machine = FirstTurnDryRunStateMachine(
            self.planner_config,
            self.trajectory_planner,
        )

        self._lock = threading.Lock()
        self._grid = None
        self._cloud_sequence = 0
        self._last_cloud_time = None
        self._cloud_timestamps = []
        self._last_map_build_time_ms = None
        self._last_odom_time = None
        self._cloud_frame = ''
        self._odom_frame = ''
        self._odom_child_frame = ''
        self._cloud_error = ''
        self._odom_error = ''
        self._roll = 0.0
        self._pitch = 0.0
        self._yaw = None
        self._previous_yaw = None
        self._turn_progress = 0.0
        self._yaw_jump = False
        self._linear_speed = 0.0
        self._angular_speed = 0.0
        self._watchdog_ok = None
        self._watchdog_time = None
        self._estop_triggered = None
        self._estop_time = None
        self._last_output = None
        self._last_signature = None

        sensor_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.cloud_subscription = self.create_subscription(
            PointCloud2,
            self.cloud_topic,
            self._on_cloud,
            sensor_qos,
        )
        self.odom_subscription = self.create_subscription(
            Odometry,
            self.odom_topic,
            self._on_odom,
            sensor_qos,
        )
        # 外部安全状态只参与“是否可执行”的诊断判定；缺失时按未验证处理。
        self.watchdog_subscription = self.create_subscription(
            Bool,
            self.watchdog_status_topic,
            self._on_watchdog_status,
            10,
        )
        self.estop_subscription = self.create_subscription(
            Bool,
            self.estop_status_topic,
            self._on_estop_status,
            10,
        )

        # 唯一发布器是诊断 String；本节点没有速度、Unitree API 或执行握手接口。
        self.status_publisher = self.create_publisher(
            String,
            self.status_topic,
            10,
        )
        self.evaluation_timer = self.create_timer(
            1.0 / self.evaluation_rate,
            self._on_evaluation,
        )
        self.print_timer = self.create_timer(
            1.0 / self.print_rate,
            self._on_print,
        )

        self.get_logger().warn(
            'B2.1 first-turn planner ready: '
            f'cloud={self.cloud_topic}, odom={self.odom_topic}, '
            f'status={self.status_topic}; DRY RUN ONLY, no motion output'
        )

    def _on_cloud(self, msg):
        """从单帧 PointCloud2 重建局部图，不跨帧积累过期障碍。"""
        now = time.monotonic()
        field_names = {field.name for field in msg.fields}
        missing = {'x', 'y', 'z'} - field_names
        if missing:
            with self._lock:
                self._cloud_sequence += 1
                self._last_cloud_time = now
                self._cloud_error = (
                    'missing_fields:' + ','.join(sorted(missing))
                )
            return
        if str(msg.header.frame_id) != self.expected_cloud_frame:
            with self._lock:
                self._cloud_sequence += 1
                self._last_cloud_time = now
                self._cloud_frame = str(msg.header.frame_id)
                self._cloud_error = 'unexpected_cloud_frame'
            return

        try:
            points = point_cloud2.read_points(
                msg,
                field_names=('x', 'y', 'z'),
                skip_nans=False,
            )
            grid = self.map_builder.build(points)
        except Exception as error:
            with self._lock:
                self._cloud_sequence += 1
                self._last_cloud_time = now
                self._cloud_error = f'cloud_parse_error:{error}'
            return

        with self._lock:
            self._cloud_sequence += 1
            self._last_cloud_time = now
            self._cloud_timestamps.append(now)
            self._cloud_timestamps = self._cloud_timestamps[-50:]
            self._last_map_build_time_ms = (
                time.monotonic() - now
            ) * 1000.0
            self._cloud_frame = str(msg.header.frame_id)
            self._cloud_error = ''
            self._grid = grid
            self.sector_tracker.update(grid.sector_stats, now)

    def _on_odom(self, msg):
        """读取姿态、速度和累计左转角，并检测不可能的 Yaw 跳变。"""
        now = time.monotonic()
        orientation = msg.pose.pose.orientation
        try:
            roll, pitch, yaw = quaternion_to_rpy(
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w,
            )
        except ValueError as error:
            with self._lock:
                self._last_odom_time = now
                self._odom_error = f'odom_orientation_invalid:{error}'
            return

        linear = msg.twist.twist.linear
        angular = msg.twist.twist.angular
        linear_speed = math.sqrt(
            linear.x * linear.x
            + linear.y * linear.y
            + linear.z * linear.z
        )
        angular_speed = math.sqrt(
            angular.x * angular.x
            + angular.y * angular.y
            + angular.z * angular.z
        )
        with self._lock:
            if self._previous_yaw is not None:
                delta = normalize_angle(yaw - self._previous_yaw)
                if abs(math.degrees(delta)) > self.planner_config.yaw_jump_limit_deg:
                    self._yaw_jump = True
                else:
                    self._turn_progress += delta
            self._previous_yaw = yaw
            self._yaw = yaw
            self._roll = roll
            self._pitch = pitch
            self._linear_speed = linear_speed
            self._angular_speed = angular_speed
            self._last_odom_time = now
            self._odom_frame = str(msg.header.frame_id)
            self._odom_child_frame = str(msg.child_frame_id)
            self._odom_error = ''

    def _on_watchdog_status(self, msg):
        with self._lock:
            self._watchdog_ok = bool(msg.data)
            self._watchdog_time = time.monotonic()

    def _on_estop_status(self, msg):
        with self._lock:
            self._estop_triggered = bool(msg.data)
            self._estop_time = time.monotonic()

    def _on_evaluation(self):
        self._evaluate_and_emit(force=False)

    def _on_print(self):
        # 打印定时器只复用最近结果，不能再次运行六候选规划并阻塞点云回调。
        with self._lock:
            output = self._last_output
        if output is not None:
            self._log_output(output)

    def _evaluate_and_emit(self, force):
        now = time.monotonic()
        with self._lock:
            cloud_age = self._age(now, self._last_cloud_time)
            odom_age = self._age(now, self._last_odom_time)
            watchdog_age = self._age(now, self._watchdog_time)
            estop_age = self._age(now, self._estop_time)
            sector_status = self.sector_tracker.snapshot(now)
            context = self._safety_context(
                cloud_age,
                odom_age,
                watchdog_age,
                estop_age,
                sector_status,
            )
            observation = {
                'front_distance_m': self._sector_distance(
                    sector_status, 'front'
                ),
                'left_open_distance_m': self._sector_distance(
                    sector_status, 'left'
                ),
                'left_distance_m': self._sector_distance(
                    sector_status, 'left'
                ),
                'right_distance_m': self._sector_distance(
                    sector_status, 'right'
                ),
                'turn_progress_rad': self._turn_progress,
            }
            output = self.state_machine.update(
                self._grid,
                context,
                observation,
                now,
                self._cloud_sequence,
            )
            output.update({
                'timestamp_monotonic_sec': now,
                'cloud_topic': self.cloud_topic,
                'odom_topic': self.odom_topic,
                'cloud_frame': self._cloud_frame,
                'odom_frame': self._odom_frame,
                'odom_child_frame': self._odom_child_frame,
                'cloud_sequence': self._cloud_sequence,
                'map_update_hz': self._map_update_hz(),
                'map_build_time_ms': self._last_map_build_time_ms,
                'cloud_age_sec': self._json_age(cloud_age),
                'odom_age_sec': self._json_age(odom_age),
                'cloud_error': self._cloud_error,
                'odom_error': self._odom_error,
                'roll_deg': math.degrees(self._roll),
                'pitch_deg': math.degrees(self._pitch),
                'yaw_deg': (
                    math.degrees(self._yaw)
                    if self._yaw is not None
                    else None
                ),
                'turn_progress_deg': math.degrees(self._turn_progress),
                'linear_speed_mps': self._linear_speed,
                'angular_speed_radps': self._angular_speed,
                'watchdog_ok': self._watchdog_ok,
                'watchdog_age_sec': self._json_age(watchdog_age),
                'estop_triggered': self._estop_triggered,
                'estop_age_sec': self._json_age(estop_age),
                'dynamic_footprint_m': self.footprint.expanded_extents(),
                'stop_tail_margin_m': self.footprint.stop_tail_margin_m,
                'target_physical_clearance_m': (
                    self.footprint.target_physical_clearance_m
                ),
                'margins_calibrated': self.footprint.margins_calibrated,
            })
            signature = (
                output['state'],
                output['reason'],
                (
                    output['selected_candidate']['name']
                    if output.get('selected_candidate')
                    else None
                ),
            )
            self._last_signature = signature
            self._last_output = output

        message = String()
        message.data = json.dumps(
            output,
            allow_nan=False,
            separators=(',', ':'),
            sort_keys=True,
        )
        self.status_publisher.publish(message)
        self._log_output(output)

    def _safety_context(
        self,
        cloud_age,
        odom_age,
        watchdog_age,
        estop_age,
        sector_status,
    ):
        return SafetyContext(
            cloud_age_sec=(cloud_age if cloud_age is not None else 1.0e9),
            odom_age_sec=(odom_age if odom_age is not None else 1.0e9),
            sector_status=sector_status,
            cloud_received=self._last_cloud_time is not None,
            odom_received=self._last_odom_time is not None,
            cloud_valid=(self._cloud_error == '' and self._grid is not None),
            odom_valid=(self._odom_error == '' and self._yaw is not None),
            watchdog_ok=self._watchdog_ok,
            watchdog_age_sec=watchdog_age,
            estop_triggered=self._estop_triggered,
            estop_age_sec=estop_age,
            roll_rad=self._roll,
            pitch_rad=self._pitch,
            yaw_jump=self._yaw_jump,
            linear_speed_mps=self._linear_speed,
            angular_speed_radps=self._angular_speed,
            stop_stable=(
                self._linear_speed
                <= self.planner_config.stationary_linear_speed_mps
                and self._angular_speed
                <= self.planner_config.stationary_angular_speed_radps
            ),
        )

    def _log_output(self, output):
        selected = output.get('selected_candidate')
        selected_text = 'none'
        if selected is not None:
            selected_text = (
                f'{selected["name"]}/{selected["verdict"]} '
                f'clearance={selected["sweep"]["minimum_clearance_m"]:.3f}m '
                f'danger_t={selected["sweep"]["danger_time_sec"]:.3f}s '
                f'part={selected["sweep"]["collision_part"]}'
            )
        rear = output['sector_status'].get('rear', {})
        top_names = ','.join(
            f'{item["rank"]}:{item["name"]}/{item["verdict"]}'
            for item in output.get('top_candidates', [])
        ) or 'none'
        text = (
            f'state={output["state"]} reason={output["reason"]} '
            f'route={output["route_index"]}/5 selected={selected_text} '
            f'rear_valid={rear.get("valid", False)} '
            f'rear_stale={rear.get("stale", True)} '
            f'rear_coverage={output["rear_coverage_status"]} '
            f'map_hz={output.get("map_update_hz") or 0.0:.2f} '
            f'top5={top_names} '
            f'execution_allowed={output["execution_allowed"]}'
        )
        if output['state'] == 'FAULT_STOP':
            self.get_logger().error(text)
        else:
            self.get_logger().info(text)

    def _load_dataclass(self, data_class):
        defaults = data_class()
        values = {}
        for field in fields(data_class):
            default = getattr(defaults, field.name)
            raw = self.declare_parameter(field.name, default).value
            if isinstance(default, bool):
                values[field.name] = bool(raw)
            elif isinstance(default, int):
                values[field.name] = int(raw)
            else:
                values[field.name] = float(raw)
        return data_class(**values)

    def _load_primitives(self):
        primitives = []
        for name, prefix in PRIMITIVE_PREFIXES.items():
            default_vx, default_vy, default_wz, default_duration = (
                PRIMITIVE_DEFAULTS[name]
            )
            primitives.append(MotionPrimitive(
                name=name,
                vx_mps=self._finite_float_parameter(
                    prefix + '_vx_mps', default_vx
                ),
                vy_mps=self._finite_float_parameter(
                    prefix + '_vy_mps', default_vy
                ),
                wz_radps=self._finite_float_parameter(
                    prefix + '_wz_radps', default_wz
                ),
                duration_sec=self._positive_float_parameter(
                    prefix + '_duration_sec', default_duration
                ),
                calibrated=bool(self.declare_parameter(
                    prefix + '_calibrated', False
                ).value),
                calibration_id=self._string_parameter(
                    prefix + '_calibration_id', 'UNVALIDATED'
                ),
            ))
        return tuple(primitives)

    @staticmethod
    def _sector_distance(sector_status, name):
        value = sector_status.get(name, {}).get('distance_m')
        return float(value) if value is not None else None

    @staticmethod
    def _age(now, timestamp):
        if timestamp is None:
            return None
        return max(0.0, now - timestamp)

    @staticmethod
    def _json_age(age):
        return float(age) if age is not None else None

    def _map_update_hz(self):
        """用最近成功构图时间估算频率，便于验证不低于约10Hz。"""
        if len(self._cloud_timestamps) < 2:
            return None
        elapsed = self._cloud_timestamps[-1] - self._cloud_timestamps[0]
        if elapsed <= 0.0:
            return None
        return (len(self._cloud_timestamps) - 1) / elapsed

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


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MazeFirstTurnDryRun()
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
