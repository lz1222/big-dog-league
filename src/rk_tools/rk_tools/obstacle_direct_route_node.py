#!/usr/bin/env python3

import math
import time
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


@dataclass(frozen=True)
class RouteStage:
    name: str
    description: str
    forward_steps: int = 0
    turn_direction: str = 'none'
    turn_degrees: float = 0.0
    action_order: str = 'forward_then_turn'
    forward_speed_mps: float = 0.35
    turn_speed_radps: float = 0.80


# ========================= 用户调试修改区 =========================
#
# 这个文件会直接发布 /navigation/cmd_vel，不经过 gait_control_node、
# action server 或 rk_interfaces。你现场调试时优先只改本区域。
#
# 1. 先标定“一步”的长度：
#    下面的 FORWARD_STEP_LENGTH_M 不是机械狗真实腿步长，而是我们给
#    硬编码路线定义的一个小距离单位。
#    例子：
#      FORWARD_STEP_LENGTH_M = 0.10
#      forward_steps = 5
#      实际直行距离约为 0.10 * 5 = 0.50 m
#
#    如果所有直行段都偏短，就把 FORWARD_STEP_LENGTH_M 调大；
#    如果所有直行段都偏长，就把 FORWARD_STEP_LENGTH_M 调小。
#
# 2. 再调每一个阶段：
#    每个 RouteStage 都可以同时写：
#      - forward_steps：这个阶段先走多少步；0 表示不走。
#      - turn_direction：这个阶段是否转弯。
#          'none'  = 不转
#          'left'  = 左转
#          'right' = 右转
#      - turn_degrees：转多少度；turn_direction='none' 时填 0.0。
#      - action_order：如果这个阶段既要走又要转，决定先后顺序。
#          'forward_then_turn' = 先走，再转
#          'turn_then_forward' = 先转，再走
#      - forward_speed_mps：这一阶段直行速度，可不填，默认 0.35。
#      - turn_speed_radps：这一阶段转弯速度，可不填，默认 0.80。
#
#    例子：
#      RouteStage(
#          name='turn_left_to_top',
#          description='第2阶段：先左转进入顶部横向通道，再向前走',
#          forward_steps=5,
#          turn_direction='left',
#          turn_degrees=90.0,
#          action_order='turn_then_forward',
#      )
#    表示：先左转 90 度，然后向前走 5 步。
#
# 3. 推荐调试顺序：
#    - 先把每个阶段的 forward_steps 设小一点，确认方向顺序正确。
#    - 再逐个阶段增加 forward_steps。
#    - 最后微调每个阶段的 turn_degrees，例如 85 / 90 / 95。
#
# 4. 当前路线顺序按你图右上角的蛇形避障顺序：
#    entry_up -> 左转 -> 顶部左走 -> 左转向下 -> 中间下走
#    -> 右转向左下 -> 底部左走 -> 右转向上 -> 左侧上走
#    -> 左转出避障区 -> exit_left
#
# 急停：
#   Ctrl+C 停 launch 后，再发一次 0 速度：
#   ros2 topic pub --once /navigation/cmd_vel geometry_msgs/msg/Twist \
#     "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {z: 0.0}}"

FORWARD_STEP_LENGTH_M = 0.10
DEFAULT_FORWARD_SPEED_MPS = 0.35
DEFAULT_TURN_SPEED_RADPS = 0.80


# 每一行就是一个阶段。你可以给任何阶段同时设置“走几步”和“转多少”。
# forward_steps 必须是整数，方便你现场按“多一步/少一步”调。
ROUTE_STAGES = [
    # 第1阶段：从入口沿竖直通道向上走，靠近顶部墙前的直行段。
    RouteStage(
        name='entry_up',
        description='第1阶段：入口向上直走',
        forward_steps=5,
        turn_direction='none',
        turn_degrees=0.0,
        action_order='forward_then_turn',
        forward_speed_mps=DEFAULT_FORWARD_SPEED_MPS,
    ),
    # 第2阶段：按你的要求，先左转90度，再沿顶部通道向前走5步。
    RouteStage(
        name='turn_left_to_top',
        description='第2阶段：先左转90度，再向前走5步',
        forward_steps=5,
        turn_direction='left',
        turn_degrees=90.0,
        action_order='turn_then_forward',
        forward_speed_mps=DEFAULT_FORWARD_SPEED_MPS,
        turn_speed_radps=DEFAULT_TURN_SPEED_RADPS,
    ),
    # 第3阶段：到顶部横向通道末端后，左转进入中间向下通道。
    RouteStage(
        name='turn_left_to_middle_down',
        description='第3阶段：左转进入中间向下通道',
        forward_steps=0,
        turn_direction='left',
        turn_degrees=90.0,
        action_order='forward_then_turn',
        forward_speed_mps=DEFAULT_FORWARD_SPEED_MPS,
        turn_speed_radps=DEFAULT_TURN_SPEED_RADPS,
    ),
    # 第4阶段：沿中间通道向下走。
    RouteStage(
        name='middle_down',
        description='第4阶段：中间通道向下直走',
        forward_steps=4,
        turn_direction='none',
        turn_degrees=0.0,
        action_order='forward_then_turn',
        forward_speed_mps=DEFAULT_FORWARD_SPEED_MPS,
    ),
    # 第5阶段：中间通道末端右转，准备走底部横向通道。
    RouteStage(
        name='turn_right_to_bottom_left',
        description='第5阶段：右转进入底部横向通道',
        forward_steps=0,
        turn_direction='right',
        turn_degrees=90.0,
        action_order='forward_then_turn',
        forward_speed_mps=DEFAULT_FORWARD_SPEED_MPS,
        turn_speed_radps=DEFAULT_TURN_SPEED_RADPS,
    ),
    # 第6阶段：沿底部通道向左走。
    RouteStage(
        name='bottom_left',
        description='第6阶段：底部通道向左直走',
        forward_steps=3,
        turn_direction='none',
        turn_degrees=0.0,
        action_order='forward_then_turn',
        forward_speed_mps=DEFAULT_FORWARD_SPEED_MPS,
    ),
    # 第7阶段：底部通道末端右转，准备沿左侧通道向上走。
    RouteStage(
        name='turn_right_to_left_up',
        description='第7阶段：右转进入左侧向上通道',
        forward_steps=0,
        turn_direction='right',
        turn_degrees=90.0,
        action_order='forward_then_turn',
        forward_speed_mps=DEFAULT_FORWARD_SPEED_MPS,
        turn_speed_radps=DEFAULT_TURN_SPEED_RADPS,
    ),
    # 第8阶段：沿左侧通道向上走。
    RouteStage(
        name='left_up',
        description='第8阶段：左侧通道向上直走',
        forward_steps=4,
        turn_direction='none',
        turn_degrees=0.0,
        action_order='forward_then_turn',
        forward_speed_mps=DEFAULT_FORWARD_SPEED_MPS,
    ),
    # 第9阶段：左侧通道末端左转，对准出口方向。
    RouteStage(
        name='turn_left_to_exit',
        description='第9阶段：左转对准出口',
        forward_steps=0,
        turn_direction='left',
        turn_degrees=90.0,
        action_order='forward_then_turn',
        forward_speed_mps=DEFAULT_FORWARD_SPEED_MPS,
        turn_speed_radps=DEFAULT_TURN_SPEED_RADPS,
    ),
    # 第10阶段：向出口方向直走，离开避障区。
    RouteStage(
        name='exit_left',
        description='第10阶段：向出口直走离开避障区',
        forward_steps=3,
        turn_direction='none',
        turn_degrees=0.0,
        action_order='forward_then_turn',
        forward_speed_mps=DEFAULT_FORWARD_SPEED_MPS,
    ),
]

# ======================= 用户调试修改区结束 =======================


class ObstacleDirectRouteNode(Node):
    """Publish a hardcoded obstacle-zone route directly to cmd_vel."""

    VALID_TURN_DIRECTIONS = {'none', 'left', 'right'}
    VALID_ACTION_ORDERS = {'forward_then_turn', 'turn_then_forward'}

    def __init__(self):
        super().__init__('obstacle_direct_route_node')

        self.cmd_vel_topic = self.declare_parameter(
            'cmd_vel_topic',
            '/navigation/cmd_vel'
        ).value
        self.publish_rate_hz = float(self.declare_parameter(
            'publish_rate_hz',
            20.0
        ).value)
        self.countdown_sec = float(self.declare_parameter(
            'countdown_sec',
            3.0
        ).value)
        self.pre_stop_sec = float(self.declare_parameter(
            'pre_stop_sec',
            0.8
        ).value)
        self.step_stop_sec = float(self.declare_parameter(
            'step_stop_sec',
            0.25
        ).value)
        self.final_stop_sec = float(self.declare_parameter(
            'final_stop_sec',
            1.0
        ).value)
        self.distance_scale = float(self.declare_parameter(
            'distance_scale',
            1.0
        ).value)
        self.turn_scale = float(self.declare_parameter(
            'turn_scale',
            1.0
        ).value)
        self.speed_scale = float(self.declare_parameter(
            'speed_scale',
            1.0
        ).value)

        self._validate_parameters()
        self.publisher = self.create_publisher(Twist, self.cmd_vel_topic, 10)

    def _validate_parameters(self):
        if not self.cmd_vel_topic:
            raise ValueError('cmd_vel_topic must not be empty')

        positive = {
            'publish_rate_hz': self.publish_rate_hz,
            'distance_scale': self.distance_scale,
            'turn_scale': self.turn_scale,
            'speed_scale': self.speed_scale,
        }
        for name, value in positive.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f'{name} must be a finite positive number')

        nonnegative = {
            'countdown_sec': self.countdown_sec,
            'pre_stop_sec': self.pre_stop_sec,
            'step_stop_sec': self.step_stop_sec,
            'final_stop_sec': self.final_stop_sec,
        }
        for name, value in nonnegative.items():
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(
                    f'{name} must be a finite nonnegative number'
                )

        for stage in ROUTE_STAGES:
            if not stage.description:
                raise ValueError(f'{stage.name} description must not be empty')

            if stage.turn_direction not in self.VALID_TURN_DIRECTIONS:
                raise ValueError(
                    f'{stage.name} invalid turn_direction: '
                    f'{stage.turn_direction}'
                )

            if stage.action_order not in self.VALID_ACTION_ORDERS:
                raise ValueError(
                    f'{stage.name} invalid action_order: '
                    f'{stage.action_order}'
                )

            if stage.forward_steps < 0:
                raise ValueError(
                    f'{stage.name} forward_steps must be >= 0'
                )

            if stage.forward_steps > 0 and stage.forward_speed_mps <= 0.0:
                raise ValueError(
                    f'{stage.name} forward_speed_mps must be positive'
                )

            if stage.turn_direction == 'none':
                if stage.turn_degrees != 0.0:
                    raise ValueError(
                        f'{stage.name} turn_degrees must be 0.0 when '
                        "turn_direction is 'none'"
                    )
            else:
                if stage.turn_degrees <= 0.0:
                    raise ValueError(
                        f'{stage.name} turn_degrees must be positive'
                    )
                if stage.turn_speed_radps <= 0.0:
                    raise ValueError(
                        f'{stage.name} turn_speed_radps must be positive'
                    )

            has_forward = stage.forward_steps > 0
            has_turn = stage.turn_direction != 'none'
            if not has_forward and not has_turn:
                raise ValueError(
                    f'{stage.name} does nothing; set forward_steps or turn'
                )

            if not has_forward and stage.action_order == 'turn_then_forward':
                raise ValueError(
                    f'{stage.name} action_order turn_then_forward requires '
                    'forward_steps > 0'
                )

            if not has_turn and stage.action_order == 'turn_then_forward':
                raise ValueError(
                    f'{stage.name} action_order turn_then_forward requires '
                    "turn_direction other than 'none'"
                )

    def run(self):
        self.get_logger().warn(
            'Direct hardcoded obstacle route will start. '
            f'cmd_vel={self.cmd_vel_topic}, stages={len(ROUTE_STAGES)}, '
            f'countdown={self.countdown_sec:.1f}s'
        )

        self._publish_stop('countdown stop', self.countdown_sec)
        self._publish_stop('pre-route stop', self.pre_stop_sec)

        try:
            for index, stage in enumerate(ROUTE_STAGES, start=1):
                self._run_stage(index, len(ROUTE_STAGES), stage)
                self._publish_stop(
                    f'after {stage.name}',
                    self.step_stop_sec
                )
        finally:
            self._publish_stop('final stop', self.final_stop_sec)

        self.get_logger().info('Direct obstacle route completed.')

    def _run_stage(self, index, total, stage):
        self.get_logger().info(
            f'route stage {index}/{total}: {stage.description}, '
            f'order={stage.action_order}'
        )

        if stage.action_order == 'turn_then_forward':
            self._run_turn(index, total, stage)
            self._publish_stop(
                f'between {stage.name} turn and forward',
                self.step_stop_sec
            )
            self._run_forward(index, total, stage)
            return

        if stage.forward_steps > 0:
            self._run_forward(index, total, stage)
            if stage.turn_direction != 'none':
                self._publish_stop(
                    f'between {stage.name} forward and turn',
                    self.step_stop_sec
                )

        if stage.turn_direction != 'none':
            self._run_turn(index, total, stage)

    def _run_forward(self, index, total, stage):
        cmd = Twist()
        duration_sec = self._forward_duration_sec(stage)
        distance_m = self._forward_distance_m(stage)
        cmd.linear.x = stage.forward_speed_mps * self.speed_scale

        self.get_logger().info(
            f'route stage {index}/{total}: {stage.name} forward, '
            f'steps={stage.forward_steps}, '
            f'step_len={FORWARD_STEP_LENGTH_M:.3f}m, '
            f'distance={distance_m:.3f}m, '
            f'vx={cmd.linear.x:.3f}m/s, '
            f'duration={duration_sec:.2f}s'
        )
        self._publish_for_duration(cmd, duration_sec)

    def _run_turn(self, index, total, stage):
        cmd = Twist()
        duration_sec = self._turn_duration_sec(stage)
        turn_angle_deg = stage.turn_degrees * self.turn_scale

        if stage.turn_direction == 'left':
            cmd.angular.z = stage.turn_speed_radps * self.speed_scale
        else:
            cmd.angular.z = -stage.turn_speed_radps * self.speed_scale

        self.get_logger().info(
            f'route stage {index}/{total}: {stage.name} turn '
            f'{stage.turn_direction}, angle={turn_angle_deg:.1f}deg, '
            f'wz={cmd.angular.z:.3f}rad/s, '
            f'duration={duration_sec:.2f}s'
        )
        self._publish_for_duration(cmd, duration_sec)

    def _forward_duration_sec(self, stage):
        distance_m = self._forward_distance_m(stage)
        speed_mps = stage.forward_speed_mps * self.speed_scale
        return distance_m / speed_mps

    def _turn_duration_sec(self, stage):
        angle_rad = math.radians(stage.turn_degrees * self.turn_scale)
        speed_radps = stage.turn_speed_radps * self.speed_scale
        return angle_rad / speed_radps

    def _forward_distance_m(self, stage):
        return (
            float(stage.forward_steps)
            * FORWARD_STEP_LENGTH_M
            * self.distance_scale
        )

    def _publish_stop(self, label, duration_sec):
        if duration_sec <= 0.0:
            return

        self.get_logger().info(
            f'{label}: zero cmd_vel for {duration_sec:.2f}s'
        )
        self._publish_for_duration(Twist(), duration_sec)

    def _publish_for_duration(self, cmd, duration_sec):
        period_sec = 1.0 / self.publish_rate_hz
        end_time = time.monotonic() + duration_sec

        while rclpy.ok() and time.monotonic() < end_time:
            self.publisher.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period_sec)


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = ObstacleDirectRouteNode()
        node.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        if node is not None:
            node.get_logger().warn('Interrupted by user')
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
