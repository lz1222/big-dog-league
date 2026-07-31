#!/usr/bin/env python3

"""B1 迷宫感知干跑节点：只读取传感器并发布诊断，不控制机器人。"""

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
from std_msgs.msg import String

from maze_perception_core import (
    ADVICE_STOP,
    DryRunDecisionEngine,
    SECTOR_NAMES,
    SideDistanceStabilizer,
    STATE_STALE,
    SectorExtractor,
    normalize_angle,
    quaternion_to_rpy,
)


class MazePerceptionDryRun(Node):
    """融合五扇区距离、里程计 Yaw 和新鲜度，输出抽象避障建议。"""

    def __init__(self):
        super().__init__('maze_perception_dry_run')

        # 输入 Topic 与只读诊断 Topic 均可通过 ROS 参数覆盖。
        self.cloud_topic = self._string_parameter(
            'cloud_topic', '/utlidar/cloud_base'
        )
        self.odom_topic = self._string_parameter(
            'odom_topic', '/utlidar/robot_odom'
        )
        self.status_topic = self._string_parameter(
            'status_topic', '/maze/perception/dry_run_status'
        )

        # 点云提取参数决定参与迷宫判断的空间范围。
        front_max_range = self._positive_float_parameter(
            'front_max_range', 3.0
        )
        side_max_range = self._positive_float_parameter(
            'side_max_range', 2.0
        )
        self.extractor = SectorExtractor(
            z_min=self._finite_float_parameter('z_min', -0.15),
            z_max=self._finite_float_parameter('z_max', 0.50),
            body_x_min=self._finite_float_parameter(
                'body_x_min', -0.45
            ),
            body_x_max=self._finite_float_parameter(
                'body_x_max', 0.45
            ),
            body_y_min=self._finite_float_parameter(
                'body_y_min', -0.25
            ),
            body_y_max=self._finite_float_parameter(
                'body_y_max', 0.25
            ),
            front_angle_deg=self._finite_float_parameter(
                # 57cm 窄通道中，45度扇区会把两侧挡板混入正前方；
                # 20度值来自两处已知距离的真机静态标定。
                'front_angle', 20.0
            ),
            min_range=self._nonnegative_float_parameter(
                'min_range', 0.05
            ),
            front_max_range=front_max_range,
            side_max_range=side_max_range,
            distance_percentile=self._finite_float_parameter(
                'distance_percentile', 10.0
            ),
            # 正前角度只控制 front，斜前和正侧边界独立配置。
            diagonal_angle_max_deg=self._finite_float_parameter(
                'diagonal_angle_max', 30.0
            ),
            side_angle_max_deg=self._finite_float_parameter(
                'side_angle_max', 120.0
            ),
            # 真机侧墙回波集中在斜前方，投影参数用于估算横向净距。
            side_projection_angle_min_deg=self._finite_float_parameter(
                'side_projection_angle_min', 15.0
            ),
            side_projection_angle_max_deg=self._finite_float_parameter(
                'side_projection_angle_max', 60.0
            ),
            side_projection_x_min=self._finite_float_parameter(
                'side_projection_x_min', 0.45
            ),
            side_projection_x_max=self._finite_float_parameter(
                'side_projection_x_max', 1.50
            ),
            side_projection_min_x_span=self._positive_float_parameter(
                'side_projection_min_x_span', 0.12
            ),
            side_projection_lateral_tolerance=(
                self._positive_float_parameter(
                    'side_projection_lateral_tolerance', 0.04
                )
            ),
            side_min_points=self._positive_int_parameter(
                'side_min_points', 3
            ),
        )
        self.side_stabilizer = SideDistanceStabilizer(
            hold_frames=self._nonnegative_int_parameter(
                'side_hold_frames', 2
            ),
            rise_tolerance_m=self._nonnegative_float_parameter(
                'side_rise_tolerance', 0.08
            ),
        )

        # 决策引擎只产生建议字符串，持续帧和滞回参数在此注入。
        self.engine = DryRunDecisionEngine(
            front_max_range=front_max_range,
            side_max_range=side_max_range,
            front_block_enter=self._positive_float_parameter(
                'front_block_enter', 0.65
            ),
            front_block_exit=self._positive_float_parameter(
                'front_block_exit', 0.80
            ),
            diagonal_block_enter=self._positive_float_parameter(
                'diagonal_block_enter', 0.45
            ),
            diagonal_block_exit=self._positive_float_parameter(
                'diagonal_block_exit', 0.60
            ),
            blocked_confirm_frames=self._positive_int_parameter(
                'blocked_confirm_frames', 3
            ),
            clear_confirm_frames=self._positive_int_parameter(
                'clear_confirm_frames', 5
            ),
            turn_min_clearance=self._positive_float_parameter(
                'turn_min_clearance', 0.55
            ),
            turn_switch_margin=self._nonnegative_float_parameter(
                'turn_switch_margin', 0.15
            ),
            preferred_turn=self._string_parameter(
                'preferred_turn', 'left'
            ),
        )

        # 点云与里程计均为必需输入，任一路超时都会进入 STALE/STOP。
        self.cloud_stale_timeout = self._positive_float_parameter(
            'cloud_stale_timeout', 0.50
        )
        self.odom_stale_timeout = self._positive_float_parameter(
            'odom_stale_timeout', 0.20
        )
        self.evaluation_rate = self._positive_float_parameter(
            'evaluation_rate', 20.0
        )
        self.print_rate = self._positive_float_parameter(
            # B2持续帧判断需要约10Hz的B1 JSON；该频率仍仅发布诊断。
            'print_rate', 10.0
        )
        self.min_cloud_points = self._positive_int_parameter(
            'min_cloud_points', 1
        )
        self.min_finite_points = self._positive_int_parameter(
            'min_finite_points', 1
        )

        # 回调和定时器可能并发访问以下快照，统一由锁保护。
        self._lock = threading.Lock()
        self._last_cloud_time = None
        self._last_odom_time = None
        self._last_cloud_error = ''
        self._last_odom_error = ''
        self._cloud_frame_id = ''
        self._odom_frame_id = ''
        self._odom_child_frame_id = ''
        self._distances = {
            name: None
            for name in SECTOR_NAMES
        }
        self._counts = {
            name: 0
            for name in SECTOR_NAMES
        }
        self._sources = {
            name: 'none'
            for name in SECTOR_NAMES
        }
        self._hold_frames = {
            name: 0
            for name in SECTOR_NAMES
        }
        self._valid_points = 0
        self._finite_points = 0
        self._total_points = 0
        self._yaw = None
        self._previous_yaw = None
        self._accumulated_yaw = 0.0
        self._last_emitted_signature = None

        # 雷达和里程计是高频传感器，使用 BEST_EFFORT 避免可靠传输积压。
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

        # 唯一输出是 JSON 诊断 String，不创建 Twist 或 Unitree API 发布器。
        self.status_publisher = self.create_publisher(
            String,
            self.status_topic,
            10,
        )
        self.watchdog_timer = self.create_timer(
            1.0 / self.evaluation_rate,
            self._on_watchdog,
        )
        self.print_timer = self.create_timer(
            1.0 / self.print_rate,
            self._on_print,
        )

        self.get_logger().info(
            'Maze perception dry run ready: '
            f'cloud={self.cloud_topic}, '
            f'odom={self.odom_topic}, '
            f'status={self.status_topic}; '
            'diagnostic output only'
        )

    def _on_cloud(self, msg):
        """解析 PointCloud2 并在每个有效点云帧上推进持续帧判断。"""
        receive_time = time.monotonic()
        field_names = {field.name for field in msg.fields}
        missing_fields = {'x', 'y', 'z'} - field_names
        if missing_fields:
            error = (
                'PointCloud2 missing fields: '
                + ', '.join(sorted(missing_fields))
            )
            with self._lock:
                self.side_stabilizer.reset()
                self._last_cloud_time = receive_time
                self._last_cloud_error = error
                self._evaluate_locked(receive_time, update_decision=False)
            self._emit_status(force=False)
            return

        # 真机和 rosbag 数据可能损坏；解析异常必须转为失效保护状态。
        try:
            points = point_cloud2.read_points(
                msg,
                field_names=('x', 'y', 'z'),
                skip_nans=False,
            )
            result = self.extractor.extract(points)
        except Exception as error:
            with self._lock:
                self.side_stabilizer.reset()
                self._last_cloud_time = receive_time
                self._last_cloud_error = (
                    f'failed to read PointCloud2: {error}'
                )
                self._evaluate_locked(receive_time, update_decision=False)
            self._emit_status(force=False)
            return

        with self._lock:
            if (
                self._last_cloud_time is None
                or receive_time - self._last_cloud_time
                > self.cloud_stale_timeout
            ):
                self.side_stabilizer.reset()
            result = self.side_stabilizer.update(result)
            self._last_cloud_time = receive_time
            self._last_cloud_error = ''
            self._cloud_frame_id = str(msg.header.frame_id)
            self._distances = result['distances']
            self._counts = result['counts']
            self._sources = result['sources']
            self._hold_frames = result['hold_frames']
            self._valid_points = result['valid_points']
            self._finite_points = result['finite_points']
            self._total_points = result['total_points']
            if self._total_points < self.min_cloud_points:
                self._last_cloud_error = (
                    f'cloud has {self._total_points} points; '
                    f'minimum is {self.min_cloud_points}'
                )
            elif self._finite_points < self.min_finite_points:
                self._last_cloud_error = (
                    f'cloud has {self._finite_points} finite points; '
                    f'minimum is {self.min_finite_points}'
                )
            self._evaluate_locked(receive_time, update_decision=True)
        self._emit_status(force=False)

    def _on_odom(self, msg):
        """读取姿态四元数，并累计经过正负 pi 边界后的连续转角。"""
        receive_time = time.monotonic()
        orientation = msg.pose.pose.orientation
        try:
            _, _, yaw = quaternion_to_rpy(
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w,
            )
        except ValueError as error:
            with self._lock:
                self._last_odom_time = receive_time
                self._last_odom_error = str(error)
                self._evaluate_locked(receive_time, update_decision=False)
            self._emit_status(force=False)
            return

        with self._lock:
            if self._previous_yaw is None:
                # 首帧只建立零点，不把初始绝对朝向计入累计转角。
                self._previous_yaw = yaw
                self._accumulated_yaw = 0.0
            else:
                # 归一化相邻角差，避免 +pi 到 -pi 时出现约 2pi 跳变。
                self._accumulated_yaw += normalize_angle(
                    yaw - self._previous_yaw
                )
                self._previous_yaw = yaw
            self._last_odom_time = receive_time
            self._last_odom_error = ''
            self._odom_frame_id = str(msg.header.frame_id)
            self._odom_child_frame_id = str(msg.child_frame_id)
            self._yaw = yaw
            self._evaluate_locked(receive_time, update_decision=False)
        self._emit_status(force=False)

    def _on_watchdog(self):
        # 即使传感器停止发布，也由看门狗主动触发 STALE/STOP。
        now = time.monotonic()
        with self._lock:
            self._evaluate_locked(now, update_decision=False)
        self._emit_status(force=False)

    def _on_print(self):
        now = time.monotonic()
        with self._lock:
            self._evaluate_locked(now, update_decision=False)
        self._emit_status(force=True)

    def _evaluate_locked(self, now, update_decision):
        """先检查两路输入新鲜度，再决定是否允许推进障碍状态机。"""
        cloud_fresh, cloud_reason = self._sensor_freshness(
            now,
            self._last_cloud_time,
            self.cloud_stale_timeout,
            self._last_cloud_error,
            'cloud',
        )
        odom_fresh, odom_reason = self._sensor_freshness(
            now,
            self._last_odom_time,
            self.odom_stale_timeout,
            self._last_odom_error,
            'odom',
        )
        if not cloud_fresh:
            self.engine.mark_stale(cloud_reason)
            return
        if not odom_fresh:
            self.engine.mark_stale(odom_reason)
            return
        if update_decision:
            # 持续“帧”确认只由新点云推进，定时器和 odom 不重复计数。
            self.engine.update(self._distances)

    @staticmethod
    def _sensor_freshness(now, last_time, timeout, error, name):
        """将未收到、内容无效和接收超时统一转换为失效原因。"""
        if last_time is None:
            return False, f'{name}_missing'
        if error:
            return False, f'{name}_invalid'
        age = now - last_time
        if age > timeout:
            return False, f'{name}_stale'
        return True, f'{name}_fresh'

    def _emit_status(self, force):
        """状态变化时立即输出，并按 print_rate 周期输出完整快照。"""
        now = time.monotonic()
        with self._lock:
            cloud_age = self._age(now, self._last_cloud_time)
            odom_age = self._age(now, self._last_odom_time)
            signature = (
                self.engine.state,
                self.engine.advice,
                self.engine.reason,
            )
            if not force and signature == self._last_emitted_signature:
                return
            self._last_emitted_signature = signature
            payload = {
                'dry_run': True,
                'state': self.engine.state,
                'advice': self.engine.advice,
                'reason': self.engine.reason,
                'cloud_age_sec': cloud_age,
                'odom_age_sec': odom_age,
                'cloud_frame': self._cloud_frame_id,
                'odom_frame': self._odom_frame_id,
                'odom_child_frame': self._odom_child_frame_id,
                'yaw_rad': self._yaw,
                'yaw_deg': (
                    math.degrees(self._yaw)
                    if self._yaw is not None
                    else None
                ),
                'turn_rad': self._accumulated_yaw,
                'turn_deg': math.degrees(self._accumulated_yaw),
                'distances_m': dict(self._distances),
                'sector_counts': dict(self._counts),
                'sector_sources': dict(self._sources),
                'sector_hold_frames': dict(self._hold_frames),
                'valid_points': self._valid_points,
                'finite_points': self._finite_points,
                'total_points': self._total_points,
                'blocked_streak': self.engine.blocked_streak,
                'clear_streak': self.engine.clear_streak,
            }

        # dry_run=true 供录包分析或下游工具明确识别“非控制输出”。
        message = String()
        message.data = json.dumps(
            payload,
            allow_nan=False,
            separators=(',', ':'),
            sort_keys=True,
        )
        self.status_publisher.publish(message)

        text = (
            f'state={payload["state"]} '
            f'advice={payload["advice"]} '
            f'reason={payload["reason"]} '
            f'cloud_age={self._format_age(cloud_age)} '
            f'odom_age={self._format_age(odom_age)} '
            f'yaw={self._format_yaw(payload["yaw_deg"])} '
            + ' '.join(
                f'{name}='
                f'{self._format_distance(payload["distances_m"][name])}'
                f'(n={payload["sector_counts"][name]},'
                f'src={payload["sector_sources"][name]},'
                f'hold={payload["sector_hold_frames"][name]})'
                for name in SECTOR_NAMES
            )
        )
        if payload['state'] == STATE_STALE:
            self.get_logger().warn(text)
        else:
            self.get_logger().info(text)

    @staticmethod
    def _age(now, last_time):
        if last_time is None:
            return None
        return max(0.0, now - last_time)

    @staticmethod
    def _format_age(age):
        if age is None:
            return 'n/a'
        return f'{age:.3f}s'

    @staticmethod
    def _format_yaw(yaw_deg):
        if yaw_deg is None:
            return 'n/a'
        return f'{yaw_deg:.2f}deg'

    @staticmethod
    def _format_distance(distance):
        if distance is None:
            return 'n/a'
        return f'{distance:.3f}m'

    def _string_parameter(self, name, default):
        """声明并读取非空字符串参数。"""
        value = str(self.declare_parameter(name, default).value)
        if not value:
            raise ValueError(f'{name} must not be empty')
        return value

    def _finite_float_parameter(self, name, default):
        """声明并读取有限浮点参数，拒绝 NaN 和 Inf。"""
        value = float(self.declare_parameter(name, default).value)
        if not math.isfinite(value):
            raise ValueError(f'{name} must be finite')
        return value

    def _positive_float_parameter(self, name, default):
        value = self._finite_float_parameter(name, default)
        if value <= 0.0:
            raise ValueError(f'{name} must be positive')
        return value

    def _nonnegative_float_parameter(self, name, default):
        value = self._finite_float_parameter(name, default)
        if value < 0.0:
            raise ValueError(f'{name} must be nonnegative')
        return value

    def _positive_int_parameter(self, name, default):
        """声明并读取正整数参数。"""
        value = int(self.declare_parameter(name, default).value)
        if value <= 0:
            raise ValueError(f'{name} must be positive')
        return value

    def _nonnegative_int_parameter(self, name, default):
        """声明并读取允许为零的整数参数。"""
        value = int(self.declare_parameter(name, default).value)
        if value < 0:
            raise ValueError(f'{name} must be nonnegative')
        return value


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MazePerceptionDryRun()
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
