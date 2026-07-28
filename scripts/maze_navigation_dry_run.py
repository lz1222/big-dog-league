#!/usr/bin/env python3

"""B2 迷宫导航干跑节点：消费 B1 JSON，只发布只读策略诊断。"""

from dataclasses import fields
import json
import math
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from maze_navigation_core import (
    MazeNavigationPolicy,
    MazeObservation,
    MazePolicyConfig,
    STATE_FAULT_STOP,
    STATE_FINISHED,
    STATE_WAIT_SENSOR,
)


DEFAULT_ROUTE = [
    'LEFT',
    'LEFT',
    'RIGHT',
    'RIGHT',
    'LEFT',
]


class MazeNavigationDryRun(Node):
    """将 B1 五扇区快照转换为可回放的 B2 状态机输出。"""

    def __init__(self):
        super().__init__('maze_navigation_dry_run')

        self.perception_status_topic = self._string_parameter(
            'perception_status_topic',
            '/maze/perception/dry_run_status',
        )
        self.navigation_status_topic = self._string_parameter(
            'navigation_status_topic',
            '/maze/navigation/dry_run_status',
        )
        self.input_stale_timeout_sec = self._positive_float_parameter(
            'input_stale_timeout_sec',
            0.80,
        )
        self.evaluation_rate = self._positive_float_parameter(
            'evaluation_rate',
            20.0,
        )
        self.print_rate = self._positive_float_parameter(
            'print_rate',
            2.0,
        )
        route = list(
            self.declare_parameter(
                'route_directions',
                DEFAULT_ROUTE,
            ).value
        )

        # 数据类字段均映射为同名 ROS 参数，避免隐藏硬编码。
        default_config = MazePolicyConfig()
        config_values = {}
        for field in fields(MazePolicyConfig):
            default = getattr(default_config, field.name)
            raw_value = self.declare_parameter(
                field.name,
                default,
            ).value
            if isinstance(default, int):
                config_values[field.name] = int(raw_value)
            else:
                config_values[field.name] = float(raw_value)

        self.policy = MazeNavigationPolicy(
            MazePolicyConfig(**config_values),
            route,
        )

        self._lock = threading.Lock()
        self._last_input_time = None
        self._last_output = self.policy.snapshot(time.monotonic())
        self._last_emitted_signature = None

        self.perception_subscription = self.create_subscription(
            String,
            self.perception_status_topic,
            self._on_perception_status,
            10,
        )

        # 唯一发布类型是 std_msgs/String，不创建速度发布器。
        self.status_publisher = self.create_publisher(
            String,
            self.navigation_status_topic,
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
            'Maze navigation policy dry run ready: '
            f'input={self.perception_status_topic}, '
            f'output={self.navigation_status_topic}, '
            f'route={route}; diagnostic JSON only'
        )
        if not self.policy.in_place_rotation_fits_corridor:
            self.get_logger().warn(
                'In-place rotation does not fit configured corridor: '
                f'sweep_diameter='
                f'{self.policy.sweep_diameter_with_margin_m:.3f}m, '
                f'corridor={self.policy.config.corridor_width_m:.3f}m; '
                'policy will require moving-turn clearance'
            )

    def _on_perception_status(self, msg):
        """解析一帧 B1 JSON；格式错误按传感器失效处理。"""
        now = time.monotonic()
        try:
            payload = json.loads(msg.data)
            observation = self._observation_from_payload(payload)
        except (TypeError, ValueError, KeyError) as error:
            with self._lock:
                self._last_input_time = now
                self._last_output = self.policy.mark_input_stale(
                    f'b1_payload_invalid:{error}',
                    now,
                )
            self._emit_status(force=False)
            return

        with self._lock:
            self._last_input_time = now
            self._last_output = self.policy.update(observation, now)
        self._emit_status(force=False)

    def _on_watchdog(self):
        # B1 Topic 完全断流时也必须进入 WAIT_SENSOR 或 FAULT_STOP。
        now = time.monotonic()
        with self._lock:
            if self._last_input_time is None:
                self._last_output = self.policy.mark_input_stale(
                    'b1_input_missing',
                    now,
                )
            elif (
                now - self._last_input_time
                > self.input_stale_timeout_sec
            ):
                self._last_output = self.policy.mark_input_stale(
                    'b1_input_stale',
                    now,
                )
            else:
                self._last_output = self.policy.snapshot(now)
        self._emit_status(force=False)

    def _on_print(self):
        now = time.monotonic()
        with self._lock:
            self._last_output = self.policy.snapshot(now)
        self._emit_status(force=True)

    def _emit_status(self, force):
        """发布机器可读 JSON，并将关键状态写入 ROS 日志。"""
        with self._lock:
            payload = dict(self._last_output)
            signature = (
                payload['state'],
                payload['reason'],
                payload['route_index'],
                round(float(payload['desired_vx']), 4),
                round(float(payload['desired_wz']), 4),
            )
            if (
                not force
                and signature == self._last_emitted_signature
            ):
                return
            self._last_emitted_signature = signature

        message = String()
        message.data = json.dumps(
            payload,
            allow_nan=False,
            separators=(',', ':'),
            sort_keys=True,
        )
        self.status_publisher.publish(message)

        turn_error_text = self._format_optional(
            payload['turn_error_deg'],
            'deg',
        )
        center_error_text = self._format_optional(
            payload['center_error_m'],
            'm',
        )
        text = (
            f'state={payload["state"]} '
            f'reason={payload["reason"]} '
            f'route={payload["route_index"]}/'
            f'{payload["route_total"]} '
            f'expected={payload["expected_turn"]} '
            f'desired_vx={payload["desired_vx"]:.3f} '
            f'desired_wz={payload["desired_wz"]:.3f} '
            f'turn_error={turn_error_text} '
            f'center_error={center_error_text}'
        )
        if payload['state'] == STATE_FAULT_STOP:
            self.get_logger().error(text)
        elif payload['state'] in (
            STATE_WAIT_SENSOR,
            STATE_FINISHED,
        ):
            self.get_logger().warn(text)
        else:
            self.get_logger().info(text)

    @staticmethod
    def _observation_from_payload(payload):
        if not isinstance(payload, dict):
            raise ValueError('root must be a JSON object')
        if payload.get('dry_run') is not True:
            raise ValueError('dry_run must be true')

        distances = payload.get('distances_m')
        if not isinstance(distances, dict):
            raise ValueError('distances_m must be an object')

        return MazeObservation(
            sensor_state=str(payload['state']),
            cloud_age_sec=payload.get('cloud_age_sec'),
            odom_age_sec=payload.get('odom_age_sec'),
            yaw_rad=payload.get('yaw_rad'),
            turn_rad=payload.get('turn_rad'),
            distances=dict(distances),
        )

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

    @staticmethod
    def _format_optional(value, suffix):
        if value is None:
            return 'n/a'
        return f'{float(value):.3f}{suffix}'


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MazeNavigationDryRun()
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
