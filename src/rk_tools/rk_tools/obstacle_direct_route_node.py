#!/usr/bin/env python3

import math
import os
import subprocess
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
    turn_forward_speed_mps: float = 0.28
    sdk_action: str = 'none'
    sdk_wait_sec: float = 0.0
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
# 3. SDK动作阶段 RouteStage 的字段：
#    - sdk_action：'balance_stand' / 'economic_gait' / 'front_jump' /
#                  'recovery_stand' / 'stop_move'
#    - sdk_wait_sec：动作发出后等待多久。
#    - enabled：False 表示临时跳过这个阶段。
#
# 4. 推荐调试顺序：
#    - 第一次先把 ENABLE_FRONT_JUMP 改成 False，确认前后直走和避障方向。
#    - 再打开 ENABLE_FRONT_JUMP 单独测跳。
#    - 最后逐段调 forward_steps 和 turn_degrees。
#
# 急停：
#   Ctrl+C 停 launch 后，再发一次 0 速度：
#   ros2 topic pub --once /navigation/cmd_vel geometry_msgs/msg/Twist \
#     "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {z: 0.0}}"

FORWARD_STEP_LENGTH_M = 0.10
DEFAULT_FORWARD_SPEED_MPS = 0.35
DEFAULT_TURN_SPEED_RADPS = 0.80
DEFAULT_TURN_FORWARD_SPEED_MPS = 0.16

SDK_NETWORK_INTERFACE = 'eth0'
SDK_ACTION_EXECUTABLE = ''
SDK_ACTION_TIMEOUT_PADDING_SEC = 6.0
SDK_ACTION_PRE_STOP_SEC = 0.4
SDK_LD_LIBRARY_PATH_PREFIX = (
    '/home/unitree/rk_inspection_ws/third_party/unitree_sdk2/install/lib',
    '/usr/local/lib',
    '/home/unitree/cyclonedds_ws/install/cyclonedds/lib',
)

ENABLE_START_SEQUENCE = True
ENABLE_FRONT_JUMP = True


# 每一行就是一个阶段。你可以在同一个表里同时调启动区和避障区。
# 普通行走阶段改 forward_steps / turn_degrees；
# 跳跃或恢复阶段改 sdk_action / sdk_wait_sec / enabled。
ROUTE_STAGES = [
    # 第0-1阶段：进入比赛后先切到较省电、较平稳的步态。
    RouteStage(
        name='prepare_balance_stand',
        description='第0-1阶段：准备站稳',
        sdk_action='balance_stand',
        sdk_wait_sec=1.0,
        enabled=ENABLE_START_SEQUENCE,
    ),
    RouteStage(
        name='prepare_economic_gait',
        description='第0-2阶段：切换续航步态',
        sdk_action='economic_gait',
        sdk_wait_sec=0.3,
        enabled=ENABLE_START_SEQUENCE,
    ),
    # 第0-3阶段：从启停区出发，先向前走2步。
    RouteStage(
        name='start_zone_forward',
        description='第0-3阶段：启停区出发向前走2步',
        forward_steps=2,
        turn_direction='none',
        turn_degrees=0.0,
        action_order='forward_then_turn',
        forward_speed_mps=DEFAULT_FORWARD_SPEED_MPS,
        enabled=ENABLE_START_SEQUENCE,
    ),
    # 第0-4阶段：前跳一次。第一次联调可把 ENABLE_FRONT_JUMP 改 False。
    RouteStage(
        name='front_jump',
        description='第0-4阶段：执行一次前跳',
        sdk_action='front_jump',
        sdk_wait_sec=2.5,
        enabled=ENABLE_START_SEQUENCE and ENABLE_FRONT_JUMP,
    ),
    # 第0-5阶段：跳完后恢复站立，再切回续航步态。
    RouteStage(
        name='recover_after_front_jump',
        description='第0-5阶段：跳完后恢复站立',
        sdk_action='recovery_stand',
        sdk_wait_sec=1.0,
        enabled=ENABLE_START_SEQUENCE and ENABLE_FRONT_JUMP,
    ),
    RouteStage(
        name='economic_after_front_jump',
        description='第0-6阶段：跳完后重新切换续航步态',
        sdk_action='economic_gait',
        sdk_wait_sec=0.3,
        enabled=ENABLE_START_SEQUENCE and ENABLE_FRONT_JUMP,
    ),
    # 第0-7阶段：继续向前走几步，进入避障区入口。
    RouteStage(
        name='enter_obstacle_forward',
        description='第0-7阶段：跳完后继续向前走进入避障区',
        forward_steps=5,
        turn_direction='none',
        turn_degrees=0.0,
        action_order='forward_then_turn',
        forward_speed_mps=DEFAULT_FORWARD_SPEED_MPS,
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
        turn_degrees=160.0,
        action_order='forward_then_turn',
        forward_speed_mps=DEFAULT_FORWARD_SPEED_MPS,
        turn_speed_radps=DEFAULT_TURN_SPEED_RADPS,
        turn_forward_speed_mps=DEFAULT_TURN_FORWARD_SPEED_MPS,
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
    """Run the hardcoded start-zone and obstacle-zone route."""

    VALID_TURN_DIRECTIONS = {'none', 'left', 'right'}
    VALID_ACTION_ORDERS = {'forward_then_turn', 'turn_then_forward'}
    VALID_SDK_ACTIONS = {
        'none',
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
        self.sdk_network_interface = self.declare_parameter(
            'sdk_network_interface',
            SDK_NETWORK_INTERFACE
        ).value
        self.sdk_action_executable = self.declare_parameter(
            'sdk_action_executable',
            SDK_ACTION_EXECUTABLE
        ).value
        self.sdk_action_timeout_padding_sec = float(self.declare_parameter(
            'sdk_action_timeout_padding_sec',
            SDK_ACTION_TIMEOUT_PADDING_SEC
        ).value)
        self.sdk_action_pre_stop_sec = float(self.declare_parameter(
            'sdk_action_pre_stop_sec',
            SDK_ACTION_PRE_STOP_SEC
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

            if has_sdk_action and (has_forward or has_turn):
                raise ValueError(
                    f'{stage.name} sdk_action stages cannot also move/turn'
                )

            if not has_forward and not has_turn and not has_sdk_action:
                raise ValueError(
                    f'{stage.name} does nothing; set forward_steps, turn, '
                    'or sdk_action'
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
            self._resolve_sdk_action_executable()

    def run(self):
        active_stages = [stage for stage in ROUTE_STAGES if stage.enabled]
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

    def _run_stage(self, index, total, stage):
        self.get_logger().info(
            f'route stage {index}/{total}: {stage.description}, '
            f'order={stage.action_order}'
        )

        if stage.sdk_action != 'none':
            self._run_sdk_action(index, total, stage)
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
        executable = self._resolve_sdk_action_executable()
        timeout_sec = (
            stage.sdk_wait_sec
            + self.sdk_action_timeout_padding_sec
        )
        command = [
            executable,
            self.sdk_network_interface,
            stage.sdk_action,
            f'{stage.sdk_wait_sec:.3f}',
        ]

        self._publish_stop(
            f'before sdk action {stage.name}',
            self.sdk_action_pre_stop_sec
        )
        self.get_logger().warn(
            f'route stage {index}/{total}: {stage.name} sdk_action, '
            f'action={stage.sdk_action}, wait={stage.sdk_wait_sec:.2f}s, '
            f'interface={self.sdk_network_interface}, '
            f'timeout={timeout_sec:.2f}s'
        )

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
