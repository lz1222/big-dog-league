#!/usr/bin/env python3

import math
import os
import signal
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
# 2. 避障区现在只改 OBSTACLE_ROUTE：
#    forward(步数) = 直走几步。
#    left(角度) / right(角度) = 续航步态左/右转多少度。
#    你想怎么跑，就按顺序写一行一行动作。
#
# 3. 巡线阶段 RouteStage 的字段：
#    - line_follow_steps：巡线走几步；比如赛前段填 3，跳后填 4。
#    - line_follow_duration_sec：手动指定巡线多久；0 表示按步数自动算。
#    - line_follow_speed_mps：自动计算时间时使用的巡线估算速度。
#      你的 Go2 低于 0.27m/s 基本不动，所以默认按 0.30m/s 计算。
#
# 3.1 启停区前进：
#    如果巡线阶段只原地踏步，就先把启停区改成写死前进。当前默认：
#    - USE_DIRECT_START_FORWARD = True
#    - line_follow_before_jump / line_follow_to_obstacle_entry 用
#      forward_steps，不依赖视觉巡线。
#    以后黑线识别稳定后，再改成 False，恢复巡线前进。
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
#   Ctrl+C 后本节点会立刻连续发布 0 速度和 mission_stop，并尽量调用
#   SDK stop_move。现场仍建议手放急停/遥控器，真机调试不要只依赖软件。

FORWARD_STEP_LENGTH_M = 0.10
LINE_FOLLOW_STEP_LENGTH_M = 0.10
LINE_FOLLOW_ESTIMATED_SPEED_MPS = 0.30
LINE_FOLLOW_START_SETTLE_SEC = 0.0
LINE_FOLLOW_STOP_SETTLE_SEC = 0.0
DEFAULT_FORWARD_SPEED_MPS = 0.25
DEFAULT_TURN_SPEED_RADPS = 0.80
# 当前避障区先切续航步态，所以转弯默认不额外给前进速度。
# 如果后面想恢复边走边转，把这里改成 0.25/0.30，或单独给 left/right 传 vx_mps。
DEFAULT_TURN_FORWARD_SPEED_MPS = 0.0
START_FORWARD_SPEED_MPS = 0.35
USE_DIRECT_START_FORWARD = True

SDK_NETWORK_INTERFACE = 'eth0'
SDK_ACTION_EXECUTABLE = ''
SDK_ACTION_TIMEOUT_PADDING_SEC = 6.0
# 普通 SDK 动作前先发一小段 0 速度，避免 UDP Move 还在持续上一次速度。
SDK_ACTION_PRE_STOP_SEC = 0.25
# 前跳对状态要求更严格：必须完全停住、站稳后再调用 FrontJump。
FRONT_JUMP_CMD_STOP_SEC = 1.00
EMERGENCY_STOP_SEC = 1.20
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

ENABLE_START_SEQUENCE = False
START_FROM_PRONE = False
ENABLE_FRONT_JUMP = False
ENABLE_OBSTACLE_ROUTE = True

MISSION_START_TOPIC = '/mission/start'
MISSION_STOP_TOPIC = '/mission/stop'


def forward(steps, speed_mps=DEFAULT_FORWARD_SPEED_MPS):
    """避障区：直走多少步。"""
    return {
        'type': 'forward',
        'steps': int(steps),
        'speed_mps': float(speed_mps),
    }


def left(degrees, vx_mps=DEFAULT_TURN_FORWARD_SPEED_MPS,
         wz_radps=DEFAULT_TURN_SPEED_RADPS):
    """避障区：左转多少度。默认续航步态原地转。"""
    return {
        'type': 'turn',
        'direction': 'left',
        'degrees': float(degrees),
        'vx_mps': float(vx_mps),
        'wz_radps': float(wz_radps),
    }


def right(degrees, vx_mps=DEFAULT_TURN_FORWARD_SPEED_MPS,
          wz_radps=DEFAULT_TURN_SPEED_RADPS):
    """避障区：右转多少度。默认续航步态原地转。"""
    return {
        'type': 'turn',
        'direction': 'right',
        'degrees': float(degrees),
        'vx_mps': float(vx_mps),
        'wz_radps': float(wz_radps),
    }


def build_obstacle_route_stages(commands):
    stages = []
    for index, command in enumerate(commands, start=1):
        command_type = command.get('type')
        if command_type == 'forward':
            steps = int(command['steps'])
            stages.append(RouteStage(
                name=f'obstacle_{index:02d}_forward',
                description=f'避障动作{index}：直走 {steps} 步',
                forward_steps=steps,
                turn_direction='none',
                turn_degrees=0.0,
                action_order='forward_then_turn',
                forward_speed_mps=float(command['speed_mps']),
                enabled=ENABLE_OBSTACLE_ROUTE,
            ))
        elif command_type == 'turn':
            direction = str(command['direction'])
            degrees = float(command['degrees'])
            stages.append(RouteStage(
                name=f'obstacle_{index:02d}_{direction}',
                description=(
                    f'避障动作{index}：'
                    f'{"左" if direction == "left" else "右"}转 '
                    f'{degrees:.1f} 度'
                ),
                forward_steps=0,
                turn_direction=direction,
                turn_degrees=degrees,
                action_order='forward_then_turn',
                forward_speed_mps=DEFAULT_FORWARD_SPEED_MPS,
                turn_speed_radps=float(command['wz_radps']),
                turn_forward_speed_mps=float(command['vx_mps']),
                enabled=ENABLE_OBSTACLE_ROUTE,
            ))
        else:
            raise ValueError(f'Unknown obstacle command: {command}')
    return stages


# ========================= 避障区路线，只改这里 =========================
# 写法非常直接：直走、左转、直走、右转……
# 例子：
#   forward(13)      # 直走13步
#   left(140)        # 续航步态左转140度，不额外前进
#   right(120, vx_mps=0.25)  # 如果想边走边转，可以单独加 vx_mps
#
# 调参建议：
#   - 直走距离不够：改 forward(...) 里的步数。
#   - 转弯角度不够：改 left/right(...) 里的角度。
#   - 默认是原地转；如果需要弧线转弯，就给 left/right 加 vx_mps=0.25。
OBSTACLE_ROUTE = [
    forward(13),      # 入口向上直走
    left(140),        # 左转进入顶部横向通道
    forward(5),       # 顶部横向通道前进
    left(140),        # 左转进入中间向下通道
    forward(3),       # 中间通道向下
    right(140),       # 右转进入底部横向通道
    forward(3),       # 底部通道前进
    right(160),       # 右转进入左侧向上通道
    forward(3),       # 左侧通道向上
    left(140),        # 左转对准出口
    forward(3),       # 出口直走
]


# 下面是程序内部用的阶段表；避障区会由 OBSTACLE_ROUTE 自动生成。
# 一般调试避障时不要改这里。
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
        sdk_wait_sec=0.0,
        enabled=ENABLE_START_SEQUENCE,
    ),
    # 第0-3阶段：切换续航步态，后面巡线和转弯更柔和。
    RouteStage(
        name='start_economic_gait',
        description='第0-3阶段：切换续航步态',
        sdk_action='economic_gait',
        sdk_wait_sec=0.0,
        enabled=ENABLE_START_SEQUENCE,
    ),
    # 第0-4阶段：跳前向前走。当前默认写死前进，不再依赖巡线。
    # 要改距离就改 forward_steps；要改速度就改 START_FORWARD_SPEED_MPS。
    RouteStage(
        name='line_follow_before_jump',
        description='第0-4阶段：写死向前走10步，到跳跃前位置',
        forward_steps=10 if USE_DIRECT_START_FORWARD else 0,
        forward_speed_mps=START_FORWARD_SPEED_MPS,
        line_follow_steps=0 if USE_DIRECT_START_FORWARD else 10,
        line_follow_speed_mps=LINE_FOLLOW_ESTIMATED_SPEED_MPS,
        enabled=ENABLE_START_SEQUENCE,
    ),
    # 第0-5阶段：执行一次前跳。
    RouteStage(
        name='front_jump',
        description='第0-5阶段：执行一次前跳',
        sdk_action='front_jump',
        sdk_wait_sec=1.2,
        enabled=ENABLE_START_SEQUENCE and ENABLE_FRONT_JUMP,
    ),
    # 第0-6阶段：跳完后恢复站立，再切回续航步态。
    RouteStage(
        name='recover_after_front_jump',
        description='第0-6阶段：跳完后恢复站立',
        sdk_action='recovery_stand',
        sdk_wait_sec=0.4,
        enabled=ENABLE_START_SEQUENCE and ENABLE_FRONT_JUMP,
    ),
    RouteStage(
        name='economic_after_front_jump',
        description='第0-7阶段：跳完后重新切换续航步态',
        sdk_action='economic_gait',
        sdk_wait_sec=0.0,
        enabled=ENABLE_START_SEQUENCE and ENABLE_FRONT_JUMP,
    ),
    # 第0-8阶段：跳后进入避障区。当前默认写死前进，不再依赖巡线。
    # 要改距离就改 forward_steps；要改速度就改 START_FORWARD_SPEED_MPS。
    RouteStage(
        name='line_follow_to_obstacle_entry',
        description='第0-8阶段：跳完后写死向前走12步，进入避障区入口',
        forward_steps=12 if USE_DIRECT_START_FORWARD else 0,
        forward_speed_mps=START_FORWARD_SPEED_MPS,
        line_follow_steps=0 if USE_DIRECT_START_FORWARD else 12,
        line_follow_speed_mps=LINE_FOLLOW_ESTIMATED_SPEED_MPS,
        enabled=ENABLE_START_SEQUENCE,
    ),

    # 避障区开始前：切换续航步态，让转弯和低速行走更柔和。
    RouteStage(
        name='obstacle_economic_gait',
        description='避障区开始前：切换续航步态',
        sdk_action='economic_gait',
        sdk_wait_sec=0.30,
        enabled=ENABLE_OBSTACLE_ROUTE,
    ),

    *build_obstacle_route_stages(OBSTACLE_ROUTE),
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
            0.0
        ).value)
        self.step_stop_sec = float(self.declare_parameter(
            'step_stop_sec',
            0.0
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
        self.stop_requested = False
        self._active_sdk_process = None

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
                self._raise_if_stop_requested()
                self._run_stage(index, len(active_stages), stage)
                self._publish_stop(
                    f'after {stage.name}',
                    self.step_stop_sec
                )
        finally:
            if self.stop_requested:
                self._send_sdk_stop_move_best_effort()
                self._publish_emergency_stop(
                    'final emergency stop',
                    max(self.final_stop_sec, EMERGENCY_STOP_SEC)
                )
            else:
                self._publish_stop('final stop', self.final_stop_sec)

        if not self.stop_requested:
            self.get_logger().info('Direct full route completed.')

    def _should_skip_stage(self, stage):
        return (
            stage.sdk_action != 'none'
            and not self._sdk_actions_available
            and self.run_without_sdk_actions
        )

    def _run_stage(self, index, total, stage):
        self._raise_if_stop_requested()
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
            process = subprocess.Popen(command, env=self._sdk_action_env())
            self._active_sdk_process = process
            deadline = time.monotonic() + timeout_sec

            while rclpy.ok():
                self._raise_if_stop_requested()
                return_code = process.poll()
                if return_code is not None:
                    break
                if time.monotonic() >= deadline:
                    self._terminate_active_sdk_process()
                    raise RuntimeError(
                        f'{stage.name} sdk_action timeout after '
                        f'{timeout_sec:.2f}s'
                    )
                rclpy.spin_once(self, timeout_sec=0.0)
                time.sleep(0.05)
            else:
                self._terminate_active_sdk_process()
                raise KeyboardInterrupt()
        except OSError as error:
            raise RuntimeError(
                f'{stage.name} failed to start sdk action helper: {error}'
            ) from error
        finally:
            self._active_sdk_process = None

        if return_code != 0:
            raise RuntimeError(
                f'{stage.name} sdk_action {stage.sdk_action} failed with '
                f'exit code {return_code}'
            )

    def request_stop(self, reason):
        if self.stop_requested:
            return

        self.stop_requested = True
        self.get_logger().warn(f'Emergency stop requested: {reason}')
        self._terminate_active_sdk_process()
        self._publish_emergency_stop('immediate emergency stop', 0.35)

    def _raise_if_stop_requested(self):
        if self.stop_requested:
            raise KeyboardInterrupt()

    def _terminate_active_sdk_process(self):
        process = self._active_sdk_process
        if process is None or process.poll() is not None:
            return

        try:
            process.terminate()
            process.wait(timeout=0.30)
        except subprocess.TimeoutExpired:
            try:
                process.kill()
                process.wait(timeout=0.30)
            except (OSError, subprocess.TimeoutExpired):
                pass
        except OSError:
            pass

    def _send_sdk_stop_move_best_effort(self):
        try:
            executable = self._resolve_sdk_action_executable()
        except FileNotFoundError as error:
            self.get_logger().warn(
                f'skip SDK stop_move during emergency stop: {error}'
            )
            return

        command = [
            executable,
            self.sdk_network_interface,
            'stop_move',
            '0.000',
        ]
        try:
            subprocess.run(
                command,
                env=self._sdk_action_env(),
                timeout=2.0,
                check=False,
            )
        except (OSError, subprocess.TimeoutExpired) as error:
            self.get_logger().warn(
                f'SDK stop_move during emergency stop failed: {error}'
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
            self._raise_if_stop_requested()
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
            self._raise_if_stop_requested()
            publisher.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period_sec)

    def _wait_for_duration(self, duration_sec):
        period_sec = 1.0 / self.publish_rate_hz
        end_time = time.monotonic() + duration_sec

        while rclpy.ok() and time.monotonic() < end_time:
            self._raise_if_stop_requested()
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period_sec)

    def _publish_for_duration(self, cmd, duration_sec):
        period_sec = 1.0 / self.publish_rate_hz
        end_time = time.monotonic() + duration_sec

        while rclpy.ok() and time.monotonic() < end_time:
            self._raise_if_stop_requested()
            self.publisher.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period_sec)

    def _publish_zero_once(self):
        self.publisher.publish(Twist())
        msg = Bool()
        msg.data = True
        self.mission_stop_publisher.publish(msg)

    def _publish_emergency_stop(self, label, duration_sec):
        duration_sec = max(float(duration_sec), 0.10)
        period_sec = 1.0 / max(self.publish_rate_hz, 10.0)
        end_time = time.monotonic() + duration_sec

        self.get_logger().warn(
            f'{label}: publish zero cmd_vel and mission_stop for '
            f'{duration_sec:.2f}s'
        )
        while time.monotonic() < end_time:
            self._publish_zero_once()
            if rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period_sec)


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = ObstacleDirectRouteNode()

        def handle_stop_signal(signum, _frame):
            node.request_stop(f'signal {signum}')
            raise KeyboardInterrupt()

        signal.signal(signal.SIGINT, handle_stop_signal)
        signal.signal(signal.SIGTERM, handle_stop_signal)

        node.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        if node is not None:
            node.request_stop('KeyboardInterrupt')
            node._send_sdk_stop_move_best_effort()
            node._publish_emergency_stop(
                'interrupt final stop',
                max(node.final_stop_sec, EMERGENCY_STOP_SEC)
            )
            node.get_logger().warn('Interrupted by user; robot stop sent')
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
