#!/usr/bin/env python3

import math
import os
import subprocess
import sys
import time
from dataclasses import dataclass
from importlib import import_module

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool


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
    turn_forward_speed_mps: float = 0.28
    sdk_action: str = 'none'
    sdk_wait_sec: float = 0.0
    line_follow_steps: int = 0
    line_follow_duration_sec: float = 0.0
    line_follow_speed_mps: float = 0.30
    enabled: bool = True


# ========================= 用户调试修改区 =========================
#
# 这个文件现在统一负责：
#   启停区出发 -> 走两步 -> 跳一下 -> 再走几步进入避障区 -> 避障区蛇形路线
#
# 直走和转弯仍然通过 /navigation/cmd_vel 走你已经验证能动的 SDK UDP
# 桥接；FrontJump / RecoveryStand 这种底层动作则由小工具
# go2_sdk_motion_action 调 Unitree SDK2。
#
# 现场调试时优先只改本区域。
#
# 1. “一步”的长度：
#    FORWARD_STEP_LENGTH_M 不是机械狗真实腿步长，而是硬编码路线的小距离单位。
#    例子：FORWARD_STEP_LENGTH_M=0.10, forward_steps=5，直行约 0.50m。
#
# 2. 普通行走阶段 RouteStage 的字段：
#    - forward_steps：走几步；0 表示不走。
#    - turn_direction：'none' / 'left' / 'right'。
#    - turn_degrees：转多少度。
#    - action_order：
#        'forward_then_turn' = 先走再转
#        'turn_then_forward' = 先转再走
#    - forward_speed_mps：直行速度。
#    - turn_speed_radps：转弯角速度。
#    - turn_forward_speed_mps：转弯时同时给多少前进速度。
#        0.00 = 原地转，不推荐。
#        0.26~0.35 = 边小步前进边转，推荐先从 0.28 试。
#        如果还是像原地转，就改到 0.30 / 0.35。
#        如果转弯半径太大、容易撞墙，就降到 0.25 / 0.26。
#
# 3. 巡线阶段 RouteStage 的字段：
#    - line_follow_steps：巡线走几步；比如赛前段填 3，跳后填 4。
#    - line_follow_duration_sec：手动指定巡线多久；0 表示按步数自动算。
#    - line_follow_speed_mps：自动计算时间时使用的巡线估算速度。
#      你的 Go2 低于 0.27m/s 基本不动，所以默认按 0.30m/s 计算。
#
# 4. SDK动作阶段 RouteStage 的字段：
#    - sdk_action：'balance_stand' / 'economic_gait' / 'front_jump' /
#                  'recovery_stand' / 'stop_move'
#    - sdk_wait_sec：动作发出后等待多久。
#    - enabled：False 表示临时跳过这个阶段。
#    - 跳跃是比赛必需动作，RUN_WITHOUT_SDK_ACTIONS 默认 False：
#      如果 C++ 小工具和 ROS2 Sport API 都不可用，节点会明确报错并停住，
#      不会静默跳过跳跃。
#    - 你这台 Go2 当前 /api/sport/request 的 unitree_api typesupport
#      容易和 ROS2 默认 RMW 冲突，所以 ALLOW_ROS_TOPIC_SDK_ACTIONS
#      默认 False。比赛动作优先使用 go2_sdk_motion_action 这个 C++ 小工具。
#
# 5. 推荐调试顺序：
#    - 先单独确认巡线系统 line_visible=true。
#    - 再测：站稳 -> 巡线3步 -> 停。
#    - 再打开跳跃，确认 FrontJump。
#    - 最后接巡线4步和避障区。
#
# 急停：
#   Ctrl+C 停 launch 后，再发一次 0 速度：
#   ros2 topic pub --once /navigation/cmd_vel geometry_msgs/msg/Twist \
#     "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {z: 0.0}}"

FORWARD_STEP_LENGTH_M = 0.10
LINE_FOLLOW_STEP_LENGTH_M = 0.10
LINE_FOLLOW_ESTIMATED_SPEED_MPS = 0.30
LINE_FOLLOW_START_SETTLE_SEC = 0.30
LINE_FOLLOW_STOP_SETTLE_SEC = 0.50
DEFAULT_FORWARD_SPEED_MPS = 0.35
DEFAULT_TURN_SPEED_RADPS = 0.80
DEFAULT_TURN_FORWARD_SPEED_MPS = 0.16

SDK_NETWORK_INTERFACE = 'eth0'
SDK_ACTION_EXECUTABLE = ''
SDK_ACTION_TIMEOUT_PADDING_SEC = 6.0
SDK_ACTION_PRE_STOP_SEC = 0.4
RUN_WITHOUT_SDK_ACTIONS = False
ALLOW_ROS_TOPIC_SDK_ACTIONS = False
SPORT_REQUEST_TOPIC = '/api/sport/request'
SDK_ACTION_API_IDS = {
    'stand_up': 1004,
    'balance_stand': 1002,
    'stop_move': 1003,
    'recovery_stand': 1006,
    'front_jump': 1031,
    'economic_gait': 1063,
}
SDK_LD_LIBRARY_PATH_PREFIX = (
    '/home/unitree/rk_inspection_ws/third_party/unitree_sdk2_official/'
    'thirdparty/lib/aarch64',
    '/home/unitree/rk_inspection_ws/install/rk_go2_sdk_bridge/lib',
    '/home/unitree/rk_inspection_ws/third_party/unitree_sdk2_official/'
    'thirdparty/lib/x86_64',
    '/usr/local/lib',
    '/home/unitree/cyclonedds_ws/install/cyclonedds/lib',
)

ENABLE_START_SEQUENCE = True
START_FROM_PRONE = False
ENABLE_FRONT_JUMP = True
ENABLE_OBSTACLE_ROUTE = True

MISSION_START_TOPIC = '/mission/start'
MISSION_STOP_TOPIC = '/mission/stop'


# 每一行就是一个阶段。你可以在同一个表里同时调：
#   可选趴下起步、巡线三步、前跳、巡线四步、避障区。
ROUTE_STAGES = [
    # 第0-1阶段：如果比赛要求趴下起步，就把 START_FROM_PRONE 改成 True。
    # 你现在说开始不用蹲下，所以默认跳过这一段。
    RouteStage(
        name='start_recovery_stand',
        description='第0-1阶段：可选从趴下状态恢复站立',
        sdk_action='recovery_stand',
        sdk_wait_sec=2.0,
        enabled=ENABLE_START_SEQUENCE and START_FROM_PRONE,
    ),
    # 第0-2阶段：站稳，准备接受 Move 速度控制。
    RouteStage(
        name='start_balance_stand',
        description='第0-2阶段：站稳准备巡线',
        sdk_action='balance_stand',
        sdk_wait_sec=0.8,
        enabled=ENABLE_START_SEQUENCE,
    ),
    # 第0-3阶段：切换续航步态，后面巡线和转弯更柔和。
    RouteStage(
        name='start_economic_gait',
        description='第0-3阶段：切换续航步态',
        sdk_action='economic_gait',
        sdk_wait_sec=0.3,
        enabled=ENABLE_START_SEQUENCE,
    ),
    # 第0-4阶段：调用已经跑通过的巡线系统，一边巡线一边向前走。
    # 如果这里看起来只是原地踏步，就优先加 line_follow_steps。
    RouteStage(
        name='line_follow_before_jump',
        description='第0-4阶段：巡线向前走10步，到跳跃前位置',
        line_follow_steps=10,
        line_follow_speed_mps=LINE_FOLLOW_ESTIMATED_SPEED_MPS,
        enabled=ENABLE_START_SEQUENCE,
    ),
    # 第0-5阶段：执行一次前跳。
    RouteStage(
        name='front_jump',
        description='第0-5阶段：执行一次前跳',
        sdk_action='front_jump',
        sdk_wait_sec=2.5,
        enabled=ENABLE_START_SEQUENCE and ENABLE_FRONT_JUMP,
    ),
    # 第0-6阶段：跳完后恢复站立，再切回续航步态。
    RouteStage(
        name='recover_after_front_jump',
        description='第0-6阶段：跳完后恢复站立',
        sdk_action='recovery_stand',
        sdk_wait_sec=1.0,
        enabled=ENABLE_START_SEQUENCE and ENABLE_FRONT_JUMP,
    ),
    RouteStage(
        name='economic_after_front_jump',
        description='第0-7阶段：跳完后重新切换续航步态',
        sdk_action='economic_gait',
        sdk_wait_sec=0.3,
        enabled=ENABLE_START_SEQUENCE and ENABLE_FRONT_JUMP,
    ),
    # 第0-8阶段：继续调用巡线系统，一边巡线一边向前走，进入避障区入口。
    # 如果跳完后进入避障区距离不够，就优先加 line_follow_steps。
    RouteStage(
        name='line_follow_to_obstacle_entry',
        description='第0-8阶段：跳完后巡线向前走12步，进入避障区入口',
        line_follow_steps=12,
        line_follow_speed_mps=LINE_FOLLOW_ESTIMATED_SPEED_MPS,
        enabled=ENABLE_START_SEQUENCE,
    ),

    # ========================= 避障区路线 =========================
    # 第1阶段：从入口沿竖直通道向上走，靠近顶部墙前的直行段。
    RouteStage(
        name='entry_up',
        description='第1阶段：入口向上直走',
        forward_steps=13,
        turn_direction='none',
        turn_degrees=0.0,
        action_order='forward_then_turn',
        forward_speed_mps=DEFAULT_FORWARD_SPEED_MPS,
        enabled=ENABLE_OBSTACLE_ROUTE,
    ),
    # 第2阶段：按你的要求，先左转90度，再沿顶部通道向前走5步。
    RouteStage(
        name='turn_left_to_top',
        description='第2阶段：先左转90度，再向前走5步',
        forward_steps=5,
        turn_direction='left',
        turn_degrees=140.0,
        action_order='turn_then_forward',
        forward_speed_mps=DEFAULT_FORWARD_SPEED_MPS,
        turn_speed_radps=DEFAULT_TURN_SPEED_RADPS,
        turn_forward_speed_mps=DEFAULT_TURN_FORWARD_SPEED_MPS,
        enabled=ENABLE_OBSTACLE_ROUTE,
    ),
    # 第3阶段：到顶部横向通道末端后，左转进入中间向下通道。
    RouteStage(
        name='turn_left_to_middle_down',
        description='第3阶段：左转进入中间向下通道',
        forward_steps=0,
        turn_direction='left',
        turn_degrees=140.0,
        action_order='forward_then_turn',
        forward_speed_mps=DEFAULT_FORWARD_SPEED_MPS,
        turn_speed_radps=DEFAULT_TURN_SPEED_RADPS,
        turn_forward_speed_mps=DEFAULT_TURN_FORWARD_SPEED_MPS,
        enabled=ENABLE_OBSTACLE_ROUTE,
    ),
    # 第4阶段：沿中间通道向下走。
    RouteStage(
        name='middle_down',
        description='第4阶段：中间通道向下直走',
        forward_steps=3,
        turn_direction='none',
        turn_degrees=0.0,
        action_order='forward_then_turn',
        forward_speed_mps=DEFAULT_FORWARD_SPEED_MPS,
        enabled=ENABLE_OBSTACLE_ROUTE,
    ),
    # 第5阶段：中间通道末端右转，准备走底部横向通道。
    RouteStage(
        name='turn_right_to_bottom_left',
        description='第5阶段：右转进入底部横向通道',
        forward_steps=0,
        turn_direction='right',
        turn_degrees=140.0,
        action_order='forward_then_turn',
        forward_speed_mps=DEFAULT_FORWARD_SPEED_MPS,
        turn_speed_radps=DEFAULT_TURN_SPEED_RADPS,
        turn_forward_speed_mps=DEFAULT_TURN_FORWARD_SPEED_MPS,
        enabled=ENABLE_OBSTACLE_ROUTE,
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
        enabled=ENABLE_OBSTACLE_ROUTE,
    ),
    # 第7阶段：底部通道末端右转，准备沿左侧通道向上走。
    RouteStage(
        name='turn_right_to_left_up',
        description='第7阶段：右转进入左侧向上通道',
        forward_steps=0,
        turn_direction='right',
        turn_degrees=160.0,
        action_order='forward_then_turn',
        forward_speed_mps=DEFAULT_FORWARD_SPEED_MPS,
        turn_speed_radps=DEFAULT_TURN_SPEED_RADPS,
        turn_forward_speed_mps=DEFAULT_TURN_FORWARD_SPEED_MPS,
        enabled=ENABLE_OBSTACLE_ROUTE,
    ),
    # 第8阶段：沿左侧通道向上走。
    RouteStage(
        name='left_up',
        description='第8阶段：左侧通道向上直走',
        forward_steps=3,
        turn_direction='none',
        turn_degrees=0.0,
        action_order='forward_then_turn',
        forward_speed_mps=DEFAULT_FORWARD_SPEED_MPS,
        enabled=ENABLE_OBSTACLE_ROUTE,
    ),
    # 第9阶段：左侧通道末端左转，对准出口方向。
    RouteStage(
        name='turn_left_to_exit',
        description='第9阶段：左转对准出口',
        forward_steps=0,
        turn_direction='left',
        turn_degrees=140.0,
        action_order='forward_then_turn',
        forward_speed_mps=DEFAULT_FORWARD_SPEED_MPS,
        turn_speed_radps=DEFAULT_TURN_SPEED_RADPS,
        turn_forward_speed_mps=DEFAULT_TURN_FORWARD_SPEED_MPS,
        enabled=ENABLE_OBSTACLE_ROUTE,
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
        enabled=ENABLE_OBSTACLE_ROUTE,
    ),
]

# ======================= 用户调试修改区结束 =======================


class ObstacleDirectRouteNode(Node):
    """Run the hardcoded start-zone and obstacle-zone route."""

    VALID_TURN_DIRECTIONS = {'none', 'left', 'right'}
    VALID_ACTION_ORDERS = {'forward_then_turn', 'turn_then_forward'}
    VALID_SDK_ACTIONS = {
        'none',
        'stand_up',
        'balance_stand',
        'economic_gait',
        'front_jump',
        'recovery_stand',
        'stop_move',
    }

    def __init__(self):
        super().__init__('obstacle_direct_route_node')

        self.cmd_vel_topic = self.declare_parameter(
            'cmd_vel_topic',
            '/navigation/cmd_vel'
        ).value
        self.mission_start_topic = self.declare_parameter(
            'mission_start_topic',
            MISSION_START_TOPIC
        ).value
        self.mission_stop_topic = self.declare_parameter(
            'mission_stop_topic',
            MISSION_STOP_TOPIC
        ).value
        self.publish_rate_hz = float(self.declare_parameter(
            'publish_rate_hz',
            20.0
        ).value)
        self.countdown_sec = float(self.declare_parameter(
            'countdown_sec',
            0.0
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
        self.sdk_network_interface = self.declare_parameter(
            'sdk_network_interface',
            SDK_NETWORK_INTERFACE
        ).value
        self.sport_request_topic = self.declare_parameter(
            'sport_request_topic',
            SPORT_REQUEST_TOPIC
        ).value
        self.sdk_action_executable = self.declare_parameter(
            'sdk_action_executable',
            SDK_ACTION_EXECUTABLE
        ).value
        self.run_without_sdk_actions = bool(self.declare_parameter(
            'run_without_sdk_actions',
            RUN_WITHOUT_SDK_ACTIONS
        ).value)
        self.allow_ros_topic_sdk_actions = bool(self.declare_parameter(
            'allow_ros_topic_sdk_actions',
            ALLOW_ROS_TOPIC_SDK_ACTIONS
        ).value)
        self.sdk_action_timeout_padding_sec = float(self.declare_parameter(
            'sdk_action_timeout_padding_sec',
            SDK_ACTION_TIMEOUT_PADDING_SEC
        ).value)
        self.sdk_action_pre_stop_sec = float(self.declare_parameter(
            'sdk_action_pre_stop_sec',
            SDK_ACTION_PRE_STOP_SEC
        ).value)
        self._sdk_action_executable_resolved = None
        self._sdk_action_missing_reason = ''
        self._sdk_actions_available = False
        self._sdk_action_backend = 'none'
        self._sport_request_msg_cls = None
        self._sport_request_publisher = None

        self._validate_parameters()
        self.publisher = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.mission_start_publisher = self.create_publisher(
            Bool,
            self.mission_start_topic,
            10
        )
        self.mission_stop_publisher = self.create_publisher(
            Bool,
            self.mission_stop_topic,
            10
        )

    def _validate_parameters(self):
        if not self.cmd_vel_topic:
            raise ValueError('cmd_vel_topic must not be empty')
        if not self.mission_start_topic:
            raise ValueError('mission_start_topic must not be empty')
        if not self.mission_stop_topic:
            raise ValueError('mission_stop_topic must not be empty')

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
            'sdk_action_timeout_padding_sec': (
                self.sdk_action_timeout_padding_sec
            ),
            'sdk_action_pre_stop_sec': self.sdk_action_pre_stop_sec,
        }
        for name, value in nonnegative.items():
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(
                    f'{name} must be a finite nonnegative number'
                )

        if not self.sdk_network_interface:
            raise ValueError('sdk_network_interface must not be empty')
        if not self.sport_request_topic:
            raise ValueError('sport_request_topic must not be empty')

        enabled_stages = [stage for stage in ROUTE_STAGES if stage.enabled]
        if not enabled_stages:
            raise ValueError('ROUTE_STAGES has no enabled stage')

        for stage in ROUTE_STAGES:
            if not stage.enabled:
                continue

            if not stage.description:
                raise ValueError(f'{stage.name} description must not be empty')

            if stage.sdk_action not in self.VALID_SDK_ACTIONS:
                raise ValueError(
                    f'{stage.name} invalid sdk_action: {stage.sdk_action}'
                )

            if stage.sdk_wait_sec < 0.0:
                raise ValueError(f'{stage.name} sdk_wait_sec must be >= 0')
            if stage.line_follow_steps < 0:
                raise ValueError(
                    f'{stage.name} line_follow_steps must be >= 0'
                )
            if stage.line_follow_duration_sec < 0.0:
                raise ValueError(
                    f'{stage.name} line_follow_duration_sec must be >= 0'
                )
            if (
                stage.line_follow_steps > 0
                and stage.line_follow_duration_sec <= 0.0
                and stage.line_follow_speed_mps <= 0.0
            ):
                raise ValueError(
                    f'{stage.name} line_follow_speed_mps must be positive '
                    'when using line_follow_steps'
                )

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
                if stage.turn_forward_speed_mps < 0.0:
                    raise ValueError(
                        f'{stage.name} turn_forward_speed_mps must be >= 0'
                    )

            has_forward = stage.forward_steps > 0
            has_turn = stage.turn_direction != 'none'
            has_sdk_action = stage.sdk_action != 'none'
            has_line_follow = (
                stage.line_follow_steps > 0
                or stage.line_follow_duration_sec > 0.0
            )

            if has_sdk_action and (has_forward or has_turn):
                raise ValueError(
                    f'{stage.name} sdk_action stages cannot also move/turn'
                )
            if has_sdk_action and has_line_follow:
                raise ValueError(
                    f'{stage.name} sdk_action stages cannot also line_follow'
                )
            if has_line_follow and (has_forward or has_turn):
                raise ValueError(
                    f'{stage.name} line_follow stages cannot also move/turn'
                )

            if (
                not has_forward
                and not has_turn
                and not has_sdk_action
                and not has_line_follow
            ):
                raise ValueError(
                    f'{stage.name} does nothing; set forward_steps, turn, '
                    'line_follow, or sdk_action'
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

        if any(stage.sdk_action != 'none' for stage in enabled_stages):
            self._detect_sdk_action_backend()

    def run(self):
        active_stages = [
            stage for stage in ROUTE_STAGES
            if stage.enabled and not self._should_skip_stage(stage)
        ]
        skipped_sdk_stages = [
            stage for stage in ROUTE_STAGES
            if stage.enabled and self._should_skip_stage(stage)
        ]
        if skipped_sdk_stages:
            skipped_names = ', '.join(stage.name for stage in skipped_sdk_stages)
            self.get_logger().warn(
                'SDK action helper is missing; skipping SDK action stages: '
                f'{skipped_names}. Reason: {self._sdk_action_missing_reason}'
            )
        if not active_stages:
            raise RuntimeError('No runnable stages after filtering SDK actions')

        self.get_logger().warn(
            'Direct hardcoded full route will start. '
            f'cmd_vel={self.cmd_vel_topic}, stages={len(active_stages)}, '
            f'countdown={self.countdown_sec:.1f}s'
        )

        self._publish_stop('countdown stop', self.countdown_sec)
        self._publish_stop('pre-route stop', self.pre_stop_sec)

        try:
            for index, stage in enumerate(active_stages, start=1):
                self._run_stage(index, len(active_stages), stage)
                self._publish_stop(
                    f'after {stage.name}',
                    self.step_stop_sec
                )
        finally:
            self._publish_stop('final stop', self.final_stop_sec)

        self.get_logger().info('Direct full route completed.')

    def _should_skip_stage(self, stage):
        return (
            stage.sdk_action != 'none'
            and not self._sdk_actions_available
            and self.run_without_sdk_actions
        )

    def _run_stage(self, index, total, stage):
        self.get_logger().info(
            f'route stage {index}/{total}: {stage.description}, '
            f'order={stage.action_order}'
        )

        if stage.sdk_action != 'none':
            self._run_sdk_action(index, total, stage)
            return

        if (
            stage.line_follow_steps > 0
            or stage.line_follow_duration_sec > 0.0
        ):
            self._run_line_follow(index, total, stage)
            return

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

    def _run_line_follow(self, index, total, stage):
        duration_sec = self._line_follow_duration_sec(stage)
        distance_m = self._line_follow_distance_m(stage)

        self.get_logger().warn(
            f'route stage {index}/{total}: {stage.name} line_follow, '
            f'steps={stage.line_follow_steps}, '
            f'step_len={LINE_FOLLOW_STEP_LENGTH_M:.3f}m, '
            f'est_distance={distance_m:.3f}m, '
            f'est_speed={stage.line_follow_speed_mps:.3f}m/s, '
            f'duration={duration_sec:.2f}s'
        )

        self._publish_stop(
            f'before line follow {stage.name}',
            LINE_FOLLOW_START_SETTLE_SEC
        )
        self._publish_mission_command(
            self.mission_start_publisher,
            True,
            f'{stage.name} mission_start',
            LINE_FOLLOW_START_SETTLE_SEC
        )
        self._wait_for_duration(duration_sec)
        self._publish_mission_command(
            self.mission_stop_publisher,
            True,
            f'{stage.name} mission_stop',
            LINE_FOLLOW_STOP_SETTLE_SEC
        )
        self._publish_stop(
            f'after line follow {stage.name}',
            LINE_FOLLOW_STOP_SETTLE_SEC
        )

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
        cmd.linear.x = stage.turn_forward_speed_mps * self.speed_scale

        self.get_logger().info(
            f'route stage {index}/{total}: {stage.name} turn '
            f'{stage.turn_direction}, angle={turn_angle_deg:.1f}deg, '
            f'turn_vx={cmd.linear.x:.3f}m/s, '
            f'wz={cmd.angular.z:.3f}rad/s, '
            f'duration={duration_sec:.2f}s'
        )
        self._publish_for_duration(cmd, duration_sec)

    def _run_sdk_action(self, index, total, stage):
        if not self._sdk_actions_available and self.run_without_sdk_actions:
            self.get_logger().warn(
                f'route stage {index}/{total}: skip {stage.name} '
                f'sdk_action={stage.sdk_action}; '
                f'{self._sdk_action_missing_reason}'
            )
            return

        if not self._sdk_actions_available:
            raise RuntimeError(
                f'SDK action backend is unavailable for {stage.name} '
                f'({stage.sdk_action}). {self._sdk_action_missing_reason}'
            )

        timeout_sec = (
            stage.sdk_wait_sec
            + self.sdk_action_timeout_padding_sec
        )

        self._publish_stop(
            f'before sdk action {stage.name}',
            self.sdk_action_pre_stop_sec
        )
        self.get_logger().warn(
            f'route stage {index}/{total}: {stage.name} sdk_action, '
            f'action={stage.sdk_action}, wait={stage.sdk_wait_sec:.2f}s, '
            f'backend={self._sdk_action_backend}, '
            f'interface={self.sdk_network_interface}, '
            f'timeout={timeout_sec:.2f}s'
        )

        if self._sdk_action_backend == 'ros_topic':
            self._publish_sport_request_action(stage)
            self._wait_for_duration(stage.sdk_wait_sec)
            return

        executable = self._resolve_sdk_action_executable()
        command = [
            executable,
            self.sdk_network_interface,
            stage.sdk_action,
            f'{stage.sdk_wait_sec:.3f}',
        ]

        try:
            result = subprocess.run(
                command,
                env=self._sdk_action_env(),
                timeout=timeout_sec,
                check=False,
            )
        except subprocess.TimeoutExpired as error:
            raise RuntimeError(
                f'{stage.name} sdk_action timeout after '
                f'{timeout_sec:.2f}s'
            ) from error
        except OSError as error:
            raise RuntimeError(
                f'{stage.name} failed to start sdk action helper: {error}'
            ) from error

        if result.returncode != 0:
            raise RuntimeError(
                f'{stage.name} sdk_action {stage.sdk_action} failed with '
                f'exit code {result.returncode}'
            )

    def _resolve_sdk_action_executable(self):
        if self._sdk_action_executable_resolved:
            return self._sdk_action_executable_resolved

        explicit = str(self.sdk_action_executable).strip()
        if explicit:
            candidates = [os.path.expanduser(explicit)]
        else:
            candidates = [
                os.environ.get('RK_GO2_SDK_MOTION_ACTION', ''),
                os.path.join(
                    os.getcwd(),
                    'install',
                    'rk_go2_sdk_bridge',
                    'lib',
                    'rk_go2_sdk_bridge',
                    'go2_sdk_motion_action'
                ),
                os.path.expanduser(
                    '~/rk_inspection_ws/install/rk_go2_sdk_bridge/lib/'
                    'rk_go2_sdk_bridge/go2_sdk_motion_action'
                ),
                (
                    '/home/unitree/rk_inspection_ws/install/'
                    'rk_go2_sdk_bridge/lib/rk_go2_sdk_bridge/'
                    'go2_sdk_motion_action'
                ),
                (
                    '/home/lzbb/rk_inspection_ws/install/'
                    'rk_go2_sdk_bridge/lib/rk_go2_sdk_bridge/'
                    'go2_sdk_motion_action'
                ),
            ]

        for candidate in candidates:
            if (
                candidate
                and os.path.isfile(candidate)
                and os.access(candidate, os.X_OK)
            ):
                return candidate

        checked = ', '.join(candidate for candidate in candidates if candidate)
        raise FileNotFoundError(
            'go2_sdk_motion_action not found or not executable. '
            f'Checked: {checked}'
        )

    def _detect_sdk_action_backend(self):
        errors = []

        try:
            self._sdk_action_executable_resolved = (
                self._resolve_sdk_action_executable()
            )
            self._sdk_action_backend = 'sdk_helper'
            self._sdk_actions_available = True
            return
        except FileNotFoundError as error:
            errors.append(str(error))

        if not self.allow_ros_topic_sdk_actions:
            errors.append(
                'ROS topic SDK action backend is disabled by default. '
                'Use go2_sdk_motion_action, or pass '
                'allow_ros_topic_sdk_actions:=true only after confirming '
                'unitree_api/RMW typesupport works on this robot.'
            )
            self._sdk_action_backend = 'none'
            self._sdk_actions_available = False
            self._sdk_action_missing_reason = ' | '.join(errors)
            return

        try:
            self._sport_request_msg_cls = self._load_sport_request_msg_class()
            self._sdk_action_backend = 'ros_topic'
            self._sdk_actions_available = True
            return
        except Exception as error:
            errors.append(f'unitree_api ROS topic backend unavailable: {error}')

        self._sdk_action_backend = 'none'
        self._sdk_actions_available = False
        self._sdk_action_missing_reason = ' | '.join(errors)

    def _load_sport_request_msg_class(self):
        self._add_unitree_api_python_paths()
        module = import_module('unitree_api.msg')
        return getattr(module, 'Request')

    def _add_unitree_api_python_paths(self):
        py_major = sys.version_info.major
        py_minor = sys.version_info.minor
        python_versions = [
            f'python{py_major}.{py_minor}',
            'python3.8',
            'python3.10',
        ]
        prefixes = [
            '/home/unitree/rk_inspection_ws/third_party/unitree_ros2/'
            'cyclonedds_ws/install/unitree_api',
            '/home/unitree/cyclonedds_ws/install/unitree_api',
            '/home/unitree/unitree_ros2/cyclonedds_ws/install/unitree_api',
            '/home/lzbb/rk_inspection_ws/third_party/unitree_ros2/'
            'cyclonedds_ws/install/unitree_api',
        ]

        for prefix in prefixes:
            for version in python_versions:
                candidate = os.path.join(
                    prefix,
                    'lib',
                    version,
                    'site-packages'
                )
                if os.path.isdir(candidate) and candidate not in sys.path:
                    sys.path.append(candidate)

    def _publish_sport_request_action(self, stage):
        api_id = SDK_ACTION_API_IDS.get(stage.sdk_action)
        if api_id is None:
            raise RuntimeError(
                f'No Sport API id configured for action {stage.sdk_action}'
            )

        if self._sport_request_msg_cls is None:
            self._sport_request_msg_cls = self._load_sport_request_msg_class()
        if self._sport_request_publisher is None:
            self._sport_request_publisher = self.create_publisher(
                self._sport_request_msg_cls,
                self.sport_request_topic,
                10
            )

        request = self._sport_request_msg_cls()
        request.header.identity.api_id = int(api_id)
        request.parameter = ''
        self.get_logger().warn(
            f'publish Sport API request: action={stage.sdk_action}, '
            f'api_id={api_id}, topic={self.sport_request_topic}'
        )

        period_sec = 1.0 / self.publish_rate_hz
        end_time = time.monotonic() + max(0.30, period_sec)
        while rclpy.ok() and time.monotonic() < end_time:
            self._sport_request_publisher.publish(request)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period_sec)

    def _sdk_action_env(self):
        env = os.environ.copy()
        paths = list(SDK_LD_LIBRARY_PATH_PREFIX)
        current = env.get('LD_LIBRARY_PATH', '')
        if current:
            paths.extend(current.split(':'))

        merged_paths = []
        seen = set()
        for path in paths:
            if path and path not in seen:
                merged_paths.append(path)
                seen.add(path)

        env['LD_LIBRARY_PATH'] = ':'.join(merged_paths)
        return env

    def _forward_duration_sec(self, stage):
        distance_m = self._forward_distance_m(stage)
        speed_mps = stage.forward_speed_mps * self.speed_scale
        return distance_m / speed_mps

    def _line_follow_duration_sec(self, stage):
        if stage.line_follow_duration_sec > 0.0:
            return stage.line_follow_duration_sec

        distance_m = self._line_follow_distance_m(stage)
        return distance_m / stage.line_follow_speed_mps

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

    def _line_follow_distance_m(self, stage):
        return (
            float(stage.line_follow_steps)
            * LINE_FOLLOW_STEP_LENGTH_M
            * self.distance_scale
        )

    def _publish_stop(self, label, duration_sec):
        if duration_sec <= 0.0:
            return

        self.get_logger().info(
            f'{label}: zero cmd_vel for {duration_sec:.2f}s'
        )
        self._publish_for_duration(Twist(), duration_sec)

    def _publish_mission_command(self, publisher, value, label, duration_sec):
        msg = Bool()
        msg.data = bool(value)
        period_sec = 1.0 / self.publish_rate_hz
        end_time = time.monotonic() + max(duration_sec, period_sec)

        self.get_logger().info(
            f'{label}: publish {msg.data} for {max(duration_sec, period_sec):.2f}s'
        )
        while rclpy.ok() and time.monotonic() < end_time:
            publisher.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period_sec)

    def _wait_for_duration(self, duration_sec):
        period_sec = 1.0 / self.publish_rate_hz
        end_time = time.monotonic() + duration_sec

        while rclpy.ok() and time.monotonic() < end_time:
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period_sec)

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
    except Exception as error:
        if node is not None:
            node.get_logger().error(f'Route aborted: {error}')
            node._publish_stop('abort final stop', node.final_stop_sec)
        else:
            raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
