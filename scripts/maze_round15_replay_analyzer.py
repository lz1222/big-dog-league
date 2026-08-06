#!/usr/bin/env python3

"""只读采集 Round15 点云和 Odom，并输出旧实际轨迹碰撞回放证据。"""

from dataclasses import fields
import json
import math
from pathlib import Path
import threading
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String

from maze_first_turn_core import (
    DynamicFootprint,
    LocalMapBuilder,
    LocalMapConfig,
    PlannerConfig,
)
from maze_perception_core import quaternion_to_rpy
from maze_round15_replay_core import (
    ReplayMapSnapshot,
    ReplayOdomSample,
    analyze_round15_actual_trajectory,
)


class Round15ReplayAnalyzer(Node):
    """缓存回放传感器数据；不创建任何 ROS Publisher 或运动接口。"""

    def __init__(self):
        super().__init__('maze_round15_replay_analyzer')
        self.cloud_topic = self._string_parameter(
            'cloud_topic', '/utlidar/cloud_base'
        )
        self.odom_topic = self._string_parameter(
            'odom_topic', '/utlidar/robot_odom'
        )
        self.expected_cloud_frame = self._string_parameter(
            'expected_cloud_frame', 'base_link'
        )
        self.candidate_status_topic = self._string_parameter(
            'candidate_status_topic',
            '/maze/first_turn/dry_run_status',
        )
        self.output_path = Path(self._string_parameter(
            'output_path',
            '/tmp/b2_1_a_round15_replay_summary.json',
        )).expanduser()
        self.contact_progress_deg = self._positive_float_parameter(
            'contact_progress_deg', 44.0
        )
        self.inactivity_finalize_sec = self._positive_float_parameter(
            'inactivity_finalize_sec', 2.0
        )
        self.max_map_snapshots = self._positive_int_parameter(
            'max_map_snapshots', 2000
        )

        self.map_config = self._load_dataclass(LocalMapConfig)
        self.footprint = self._load_dataclass(DynamicFootprint)
        self.planner_config = self._load_dataclass(PlannerConfig)
        self.map_builder = LocalMapBuilder(self.map_config)
        self._snapshots = []
        self._odom = []
        self._candidate_statuses = []
        self._last_receive_monotonic = None
        self._finalized = False
        self._result = None
        self._lock = threading.Lock()

        self.cloud_subscription = self.create_subscription(
            PointCloud2,
            self.cloud_topic,
            self._on_cloud,
            qos_profile_sensor_data,
        )
        self.odom_subscription = self.create_subscription(
            Odometry,
            self.odom_topic,
            self._on_odom,
            qos_profile_sensor_data,
        )
        self.candidate_subscription = self.create_subscription(
            String,
            self.candidate_status_topic,
            self._on_candidate_status,
            10,
        )
        self.finalize_timer = self.create_timer(0.5, self._on_timer)
        self.get_logger().warn(
            'Round15 replay analyzer ready; READ ONLY, no publishers, '
            f'output={self.output_path}'
        )

    def _on_cloud(self, msg):
        """每帧独立构图，保留当时 base_link 下的有限墙段和障碍点。"""
        if self._finalized:
            return
        if str(msg.header.frame_id) != self.expected_cloud_frame:
            self.get_logger().error(
                f'ignore cloud frame={msg.header.frame_id!r}, '
                f'expected={self.expected_cloud_frame!r}'
            )
            return
        field_names = {field.name for field in msg.fields}
        if not {'x', 'y', 'z'}.issubset(field_names):
            self.get_logger().error('ignore PointCloud2 without x/y/z')
            return
        try:
            points = point_cloud2.read_points(
                msg,
                field_names=('x', 'y', 'z'),
                skip_nans=False,
            )
            grid = self.map_builder.build(points)
        except Exception as error:
            self.get_logger().error(f'cloud build failed: {error}')
            return
        snapshot = ReplayMapSnapshot(
            self._stamp_sec(msg.header.stamp),
            grid,
        )
        with self._lock:
            if len(self._snapshots) >= self.max_map_snapshots:
                # 丢弃新帧会破坏接触前证据，因此容量不足直接结束为 BLOCKED。
                self.get_logger().error('map snapshot limit reached')
                self._finalized = True
                return
            self._snapshots.append(snapshot)
            self._last_receive_monotonic = time.monotonic()

    def _on_odom(self, msg):
        if self._finalized:
            return
        orientation = msg.pose.pose.orientation
        try:
            _, _, yaw = quaternion_to_rpy(
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w,
            )
        except ValueError as error:
            self.get_logger().error(f'ignore invalid odom: {error}')
            return
        position = msg.pose.pose.position
        sample = ReplayOdomSample(
            self._stamp_sec(msg.header.stamp),
            float(position.x),
            float(position.y),
            float(yaw),
        )
        with self._lock:
            self._odom.append(sample)
            self._last_receive_monotonic = time.monotonic()

    def _on_candidate_status(self, msg):
        """缓存候选排名；损坏 JSON 不能影响旧实际轨迹几何分析。"""
        if self._finalized:
            return
        try:
            payload = json.loads(msg.data)
        except (TypeError, ValueError):
            self.get_logger().error('ignore invalid candidate status JSON')
            return
        with self._lock:
            self._candidate_statuses.append(payload)

    def _on_timer(self):
        with self._lock:
            last_receive = self._last_receive_monotonic
            has_data = bool(self._snapshots and self._odom)
        if (
            not self._finalized
            and has_data
            and last_receive is not None
            and time.monotonic() - last_receive
            >= self.inactivity_finalize_sec
        ):
            self.finalize()

    def finalize(self):
        """分析并落盘一次；结论只评价旧轨迹拒绝能力，不授权运动。"""
        with self._lock:
            if self._result is not None:
                return self._result
            snapshots = tuple(self._snapshots)
            odom = tuple(self._odom)
            candidate_statuses = tuple(self._candidate_statuses)
            self._finalized = True
        result = analyze_round15_actual_trajectory(
            snapshots,
            odom,
            self.footprint,
            self.planner_config,
            contact_progress_deg=self.contact_progress_deg,
            turn_direction_sign=1,
        )
        result.update({
            'dry_run': True,
            'motion_output': False,
            'cloud_topic': self.cloud_topic,
            'odom_topic': self.odom_topic,
            'candidate_status_topic': self.candidate_status_topic,
            'candidate_status_count': len(candidate_statuses),
            'candidate_snapshot_near_contact': (
                self._candidate_snapshot(candidate_statuses)
            ),
            'output_path': str(self.output_path),
        })
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        self.output_path.write_text(
            json.dumps(result, ensure_ascii=False, indent=2) + '\n',
            encoding='utf-8',
        )
        with self._lock:
            self._result = result
        self.get_logger().warn(
            f'Round15 replay result={result["result"]} '
            f'gate={result["gate_status"]} '
            f'output={self.output_path}'
        )
        return result

    def _candidate_snapshot(self, statuses):
        """选择事故角度前最近一帧，并压缩为可审计的前五名证据。"""
        eligible = [
            item
            for item in statuses
            if item.get('turn_progress_deg') is not None
            and float(item['turn_progress_deg']) <= self.contact_progress_deg
        ]
        if not eligible:
            return None
        selected = max(
            eligible,
            key=lambda item: float(item['turn_progress_deg']),
        )
        top_candidates = []
        for candidate in selected.get('top_candidates', [])[:5]:
            sweep = candidate.get('sweep') or {}
            top_candidates.append({
                'rank': candidate.get('rank'),
                'name': candidate.get('name'),
                'verdict': candidate.get('verdict'),
                'minimum_clearance_m': sweep.get('minimum_clearance_m'),
                'danger_time_sec': sweep.get('danger_time_sec'),
                'danger_pose': sweep.get('danger_pose'),
                'danger_part': sweep.get('collision_part'),
                'danger_wall_segment_id': sweep.get(
                    'danger_wall_segment_id'
                ),
                'robustness_ratio': candidate.get('robustness_ratio'),
                'legacy_0413_guard_pass': candidate.get(
                    'legacy_0413_guard_pass'
                ),
            })
        return {
            'turn_progress_deg': selected.get('turn_progress_deg'),
            'state': selected.get('state'),
            'reason': selected.get('reason'),
            'rear_coverage_status': selected.get('rear_coverage_status'),
            'wall_segment_count': (selected.get('map_statistics') or {}).get(
                'wall_segment_count'
            ),
            'has_robust_safe_candidate': selected.get(
                'has_robust_safe_candidate'
            ),
            'top_candidates': top_candidates,
        }

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

    def _positive_int_parameter(self, name, default):
        value = int(self.declare_parameter(name, default).value)
        if value <= 0:
            raise ValueError(f'{name} must be positive')
        return value

    @staticmethod
    def _stamp_sec(stamp):
        return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Round15ReplayAnalyzer()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.finalize()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
