#!/usr/bin/env python3

"""B2 输入模拟器。

发布合成 B1 JSON，不生成点云或任何运动命令。
"""

import json
import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


DEFAULT_ROUTE = (
    'LEFT',
    'LEFT',
    'RIGHT',
    'RIGHT',
    'LEFT',
)
VALID_SCENARIOS = {
    'nominal',
    'sensor_stale',
    'turn_timeout',
    'blocked_turn',
}


class MazeNavigationSimulator(Node):
    """按固定时间轴模拟五次雷达转角及故障输入。"""

    def __init__(self):
        super().__init__('maze_navigation_simulator')

        self.status_topic = self._string_parameter(
            'perception_status_topic',
            '/maze/perception/dry_run_status',
        )
        self.publish_rate = self._positive_float_parameter(
            'sim_publish_rate',
            15.0,
        )
        self.scenario = self._string_parameter(
            'sim_scenario',
            'nominal',
        ).lower()
        if self.scenario not in VALID_SCENARIOS:
            raise ValueError(
                'sim_scenario must be one of: '
                + ', '.join(sorted(VALID_SCENARIOS))
            )

        self.warmup_sec = self._positive_float_parameter(
            'sim_warmup_sec',
            0.6,
        )
        self.corridor_sec = self._positive_float_parameter(
            'sim_corridor_sec',
            1.2,
        )
        self.approach_sec = self._positive_float_parameter(
            'sim_approach_sec',
            1.0,
        )
        self.turn_sec = self._positive_float_parameter(
            'sim_turn_sec',
            2.0,
        )
        self.reacquire_sec = self._positive_float_parameter(
            'sim_reacquire_sec',
            1.2,
        )
        self.fault_after_sec = self._positive_float_parameter(
            'sim_fault_after_sec',
            2.0,
        )

        # 唯一输出是 B1 格式的诊断 String，供 B2 节点和 rosbag 消费。
        self.publisher = self.create_publisher(
            String,
            self.status_topic,
            10,
        )
        self.timer = self.create_timer(
            1.0 / self.publish_rate,
            self._on_timer,
        )
        self._start_time = time.monotonic()
        self._last_label = ''

        self.get_logger().info(
            'Maze navigation simulator ready: '
            f'topic={self.status_topic}, scenario={self.scenario}; '
            'B1-style JSON only'
        )

    def _on_timer(self):
        elapsed = time.monotonic() - self._start_time
        sample = self._sample(elapsed)
        if sample['label'] != self._last_label:
            self._last_label = sample['label']
            self.get_logger().info(
                f'sim_phase={self._last_label}'
            )

        payload = {
            'dry_run': True,
            'state': sample['sensor_state'],
            'advice': 'STOP',
            'reason': f'simulator_{sample["label"]}',
            'cloud_age_sec': sample['cloud_age_sec'],
            'odom_age_sec': sample['odom_age_sec'],
            'cloud_frame': 'base_link',
            'odom_frame': 'odom',
            'odom_child_frame': 'base_link',
            'yaw_rad': self._normalize_angle(sample['turn_rad']),
            'yaw_deg': math.degrees(
                self._normalize_angle(sample['turn_rad'])
            ),
            'turn_rad': sample['turn_rad'],
            'turn_deg': math.degrees(sample['turn_rad']),
            'distances_m': sample['distances'],
            'sector_counts': {
                name: 20
                for name in (
                    'front',
                    'left_front',
                    'right_front',
                    'left',
                    'right',
                )
            },
            'valid_points': 100,
            'finite_points': 100,
            'total_points': 100,
            'blocked_streak': 0,
            'clear_streak': 0,
        }
        message = String()
        message.data = json.dumps(
            payload,
            allow_nan=False,
            separators=(',', ':'),
            sort_keys=True,
        )
        self.publisher.publish(message)

    def _sample(self, elapsed):
        if (
            self.scenario == 'sensor_stale'
            and elapsed >= self.fault_after_sec
        ):
            return {
                'label': 'sensor_stale',
                'sensor_state': 'STALE',
                'cloud_age_sec': 1.0,
                'odom_age_sec': 1.0,
                'turn_rad': 0.0,
                'distances': self._corridor_distances(),
            }

        if self.scenario == 'turn_timeout':
            first_turn_start = (
                self.warmup_sec
                + self.corridor_sec
                + self.approach_sec
            )
            if elapsed >= first_turn_start:
                return {
                    'label': 'turn_timeout_hold',
                    'sensor_state': 'BLOCKED',
                    'cloud_age_sec': 0.01,
                    'odom_age_sec': 0.01,
                    'turn_rad': 0.0,
                    'distances': self._turn_distances('LEFT'),
                }

        if self.scenario == 'blocked_turn':
            blocked_start = self.warmup_sec + self.corridor_sec
            if elapsed >= blocked_start:
                distances = self._turn_distances('LEFT')
                distances.update({
                    'front': 0.40,
                    'left_front': 0.24,
                    'left': 0.24,
                })
                return {
                    'label': 'blocked_turn_envelope',
                    'sensor_state': 'BLOCKED',
                    'cloud_age_sec': 0.01,
                    'odom_age_sec': 0.01,
                    'turn_rad': 0.0,
                    'distances': distances,
                }

        return self._nominal_sample(elapsed)

    def _nominal_sample(self, elapsed):
        if elapsed < self.warmup_sec:
            return self._fresh_sample(
                'warmup',
                0.0,
                self._corridor_distances(),
            )

        cycle_duration = (
            self.corridor_sec
            + self.approach_sec
            + self.turn_sec
            + self.reacquire_sec
        )
        route_elapsed = elapsed - self.warmup_sec
        route_index = int(route_elapsed / cycle_duration)
        if route_index >= len(DEFAULT_ROUTE):
            final_yaw = self._completed_route_yaw(len(DEFAULT_ROUTE))
            return self._fresh_sample(
                'exit_open',
                final_yaw,
                {
                    'front': 2.20,
                    'left_front': 1.60,
                    'right_front': 1.60,
                    'left': 1.20,
                    'right': 1.20,
                },
            )

        phase_elapsed = route_elapsed % cycle_duration
        direction = DEFAULT_ROUTE[route_index]
        base_yaw = self._completed_route_yaw(route_index)

        if phase_elapsed < self.corridor_sec:
            return self._fresh_sample(
                f'route_{route_index + 1}_corridor',
                base_yaw,
                self._corridor_distances(),
            )
        phase_elapsed -= self.corridor_sec

        if phase_elapsed < self.approach_sec:
            ratio = phase_elapsed / self.approach_sec
            front = 0.78 - 0.18 * ratio
            distances = self._turn_distances(direction)
            distances['front'] = front
            return self._fresh_sample(
                f'route_{route_index + 1}_approach_{direction.lower()}',
                base_yaw,
                distances,
                sensor_state='BLOCKED',
            )
        phase_elapsed -= self.approach_sec

        if phase_elapsed < self.turn_sec:
            # 80% 时间完成 90 度，剩余时间保持目标角供持续帧确认。
            progress = min(
                1.0,
                phase_elapsed / (0.80 * self.turn_sec),
            )
            sign = 1.0 if direction == 'LEFT' else -1.0
            turn_rad = base_yaw + sign * progress * (0.5 * math.pi)
            return self._fresh_sample(
                f'route_{route_index + 1}_turn_{direction.lower()}',
                turn_rad,
                self._turn_distances(direction),
                sensor_state='BLOCKED',
            )

        final_yaw = self._completed_route_yaw(route_index + 1)
        return self._fresh_sample(
            f'route_{route_index + 1}_reacquire',
            final_yaw,
            self._corridor_distances(),
        )

    @staticmethod
    def _fresh_sample(
        label,
        turn_rad,
        distances,
        sensor_state='CLEAR',
    ):
        return {
            'label': label,
            'sensor_state': sensor_state,
            'cloud_age_sec': 0.01,
            'odom_age_sec': 0.01,
            'turn_rad': turn_rad,
            'distances': distances,
        }

    @staticmethod
    def _corridor_distances():
        return {
            'front': 2.20,
            'left_front': 1.10,
            'right_front': 1.10,
            'left': 0.285,
            'right': 0.285,
        }

    @staticmethod
    def _turn_distances(direction):
        distances = {
            'front': 0.60,
            'left_front': 0.80,
            'right_front': 0.80,
            'left': 0.30,
            'right': 0.30,
        }
        if direction == 'LEFT':
            distances['left_front'] = 1.00
            distances['left'] = 1.20
        else:
            distances['right_front'] = 1.00
            distances['right'] = 1.20
        return distances

    @staticmethod
    def _completed_route_yaw(count):
        yaw = 0.0
        for direction in DEFAULT_ROUTE[:count]:
            yaw += 0.5 * math.pi if direction == 'LEFT' else -0.5 * math.pi
        return yaw

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
    def _normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MazeNavigationSimulator()
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
