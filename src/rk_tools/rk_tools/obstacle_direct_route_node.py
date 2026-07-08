#!/usr/bin/env python3

import math
import os
import signal
import subprocess
import sys
import time
from dataclasses import dataclass
from importlib import import_module

try:
    import cv2
    import numpy as np
    from sensor_msgs.msg import Image
except ImportError:
    cv2 = None
    np = None
    Image = None

import rclpy
from geometry_msgs.msg import Twist
from rk_interfaces.msg import LineTrack
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
    line_follow_until_lost: bool = False
    line_follow_until_white_line: bool = False
    line_follow_require_visible: bool = True
    line_follow_finish_centered: bool = False
    enabled: bool = True


# ========================= 用户调试修改区 =========================
#
# 这个文件现在统一负责：
#   启停区出发 -> 调用已跑通的 SDK 巡线，直到检测到长白线
#   -> 向前直走2步 -> 前跳 -> 继续启动巡线自动找线
#   -> 巡线3秒
#   -> 切换续航步态 -> 避障区蛇形路线
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
#    left(角度) / right(角度) = 边走边左/右转多少度。
#    你想怎么跑，就按顺序写一行一行动作。
#
# 3. 巡线阶段 RouteStage 的字段：
#    - line_follow_steps：巡线走几步；比如赛前段填 3，跳后填 4。
#    - line_follow_duration_sec：手动指定巡线多久；0 表示按步数自动算。
#    - line_follow_speed_mps：自动计算时间时使用的巡线估算速度。
#      你的 Go2 低于 0.27m/s 基本不动，所以默认按 0.30m/s 计算。
#
# 3.1 启停区巡线与跳跃：
#    - START_LINE_FOLLOW_UNTIL_WHITE_MAX_SEC：
#      起步后最多巡线多久等待长白线；检测到长白线会提前停巡线。
#    - START_FORWARD_AFTER_WHITE_STEPS：
#      检测到长白线后，先向前直走几步再跳。你这次要求为 1 步。
#    - POST_JUMP_LINE_FOLLOW_SEC：
#      跳完后继续巡线多久再切入避障区。你这次要求为 3 秒。
#    - WHITE_LINE_*：
#      长白线识别参数。默认会同时用图像里的横向白色条带，以及
#      “黑线突然持续不可用”作为兜底触发，因为长白线通常会让黑线
#      检测中断。
#    - LINE_LOST_SWITCH_SEC：
#      如果图像白线识别不稳定，黑线持续看不到多久后也认为到了长白线。
#      过早触发就调大，触发太晚就调小。
#    - FrontJump 是 Unitree SDK 内置动作，本接口没有蹲下速度参数；
#      这里通过跳前停稳和跳后恢复等待来降低冲击。
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
#    - 最后接跳后巡线和避障区。
#
# 急停：
#   Ctrl+C 后本节点会立刻连续发布 0 速度和 mission_stop，并尽量调用
#   SDK stop_move。现场仍建议手放急停/遥控器，真机调试不要只依赖软件。

# ------------------------- 距离和巡线时间参数 -------------------------
# 写死路线里“一步”对应的距离，forward(8) 就是约 8 * 0.10 = 0.80m。
FORWARD_STEP_LENGTH_M = 0.10

# 巡线阶段如果按“步数”估算距离时使用的步长，目前主要保留给调试备用。
LINE_FOLLOW_STEP_LENGTH_M = 0.10

# 巡线速度估计值，只用于把 line_follow_steps 换算成持续时间。
LINE_FOLLOW_ESTIMATED_SPEED_MPS = 0.30

# 每次启动巡线前先停稳多久；太小容易带着上一段速度进入巡线。
LINE_FOLLOW_START_SETTLE_SEC = 0.35

# 每次停止巡线后再停稳多久；太小可能还没停住就进入下一个动作。
LINE_FOLLOW_STOP_SETTLE_SEC = 0.25

# 启停区最多巡线等待长白线的时间，超过后会继续执行后面的跳跃流程。
START_LINE_FOLLOW_UNTIL_WHITE_MAX_SEC = 16.0

# 检测到长白线后向前直走几步再跳；你现在要求为 1 步。
START_FORWARD_AFTER_WHITE_STEPS = 1

# 跳完之后继续巡线多久，然后再进入“居中确认”。
POST_JUMP_LINE_FOLLOW_SEC = 3.0

# 开始巡线前最多等待黑线可见多久，超时会报错停住。
LINE_VISIBLE_WAIT_TIMEOUT_SEC = 10.0

# 等待长白线时，如果黑线连续丢失这么久，就认为到达白泡沫条。
LINE_LOST_SWITCH_SEC = 0.45

# /perception/line_track 消息超过这个时间没更新，就认为相机/识别卡住。
LINE_TRACK_STALE_SEC = 0.80

# 本路线节点认为“黑线可用”的最低置信度。
ROUTE_LINE_MIN_CONFIDENCE = 0.35

# 本路线节点允许的最大黑线横向偏差；超过说明不适合直接巡线。
ROUTE_LINE_MAX_ABS_LATERAL_ERROR = 0.95

# ------------------------- 白色泡沫条识别参数 -------------------------
# 是否启用长白线/白泡沫条识别。
WHITE_LINE_DETECTION_ENABLED = True

# 用哪个彩色图像话题识别白泡沫条。
WHITE_LINE_IMAGE_TOPIC = '/camera/color/image_raw'

# 白泡沫条识别的 ROI 上边界，0.25 表示从画面 25% 高度以下开始看。
WHITE_LINE_ROI_TOP_FRACTION = 0.25

# 白泡沫条识别的 ROI 下边界，0.98 表示看到接近画面底部。
WHITE_LINE_ROI_BOTTOM_FRACTION = 0.98

# 白泡沫条识别时左右两边裁掉的比例，避免边缘杂物干扰。
WHITE_LINE_ROI_SIDE_MARGIN_FRACTION = 0.02

# 颜色法要求白色候选横向至少占 ROI 宽度多少。
WHITE_LINE_MIN_WIDTH_FRACTION = 0.22

# 颜色法要求白色候选高度至少占 ROI 高度多少。
WHITE_LINE_MIN_HEIGHT_FRACTION = 0.010

# 颜色法允许白色候选最大高度，太高通常是整块白地板误识别。
WHITE_LINE_MAX_HEIGHT_FRACTION = 0.45

# 颜色法要求白色候选是横条形，宽高比越大越像长白线。
WHITE_LINE_MIN_ASPECT_RATIO = 2.0

# HSV 里 V 亮度阈值，越低越容易把灰白地板也识别成白线。
WHITE_LINE_MIN_VALUE = 160

# HSV 里 S 饱和度阈值，越高越容易把有颜色的区域也当白线。
WHITE_LINE_MAX_SATURATION = 130

# 是否启用泡沫条“凸起横向边缘”检测，白地白条时这个比颜色更可靠。
WHITE_LINE_EDGE_DETECTION_ENABLED = True

# 边缘法要求某一行的横向边缘像素至少占 ROI 宽度多少。
WHITE_LINE_EDGE_MIN_ROW_FRACTION = 0.32

# 边缘法的最小梯度强度，越低越灵敏但更容易受反光/纹理影响。
WHITE_LINE_EDGE_MIN_GRADIENT = 14.0

# 边缘法认为泡沫条上下两条边之间的最小距离比例。
WHITE_LINE_EDGE_MIN_GAP_FRACTION = 0.012

# 边缘法认为泡沫条上下两条边之间的最大距离比例。
WHITE_LINE_EDGE_MAX_GAP_FRACTION = 0.20

# 只有黑线置信度低于这个值，白泡沫候选才会被接受，防止白地板误触发。
WHITE_LINE_MAX_LINE_CONFIDENCE = 0.65

# 白泡沫候选需要连续稳定多久才算真的看到了。
WHITE_LINE_STABLE_SEC = 0.08

# 图像超过多久没有更新，就不再相信上一帧白泡沫检测结果。
WHITE_LINE_STALE_SEC = 1.00

# ------------------------- 进入避障区前居中确认参数 -------------------------
# 跳后巡线结束后，最多继续居中确认多久；超时会停住报错，不盲进避障。
CENTER_BEFORE_OBSTACLE_TIMEOUT_SEC = 8.0

# 黑线居中状态需要连续稳定多久，才允许切入避障区。
CENTER_BEFORE_OBSTACLE_STABLE_SEC = 0.60

# 进入避障区前要求黑线最低置信度，越高越保守。
CENTER_BEFORE_OBSTACLE_MIN_CONFIDENCE = 0.55

# 进入避障区前允许的最大横向偏差，越小越要求站在黑线正中间。
CENTER_BEFORE_OBSTACLE_MAX_LATERAL_ERROR = 0.10

# 进入避障区前允许的最大方向偏差，越小越要求身体朝向和线一致。
CENTER_BEFORE_OBSTACLE_MAX_HEADING_ERROR = 0.22

# ------------------------- 避障区和普通直走参数 -------------------------
# 切换续航步态后等待多久再开始避障动作。
ECONOMIC_GAIT_WAIT_SEC = 0.30

# 写死直走动作默认前进速度。
DEFAULT_FORWARD_SPEED_MPS = 0.35

# 写死转弯动作默认角速度。
DEFAULT_TURN_SPEED_RADPS = 0.80
# 避障区转弯时也要给前进速度，避免原地扭腿。
# 你的 Go2 实测 0.27m/s 以下基本不往前走，所以这里默认用 0.27。
# 如果转弯半径太大容易撞墙，先降到 0.27；如果还是像原地转，升到 0.35。
DEFAULT_TURN_FORWARD_SPEED_MPS = 0.27

# 长白线后直走一小步再跳时使用的前进速度。
START_FORWARD_SPEED_MPS = 0.35

# 备用开关：目前不使用直接起步直走，保持 False。
USE_DIRECT_START_FORWARD = False

# ------------------------- Unitree SDK 动作参数 -------------------------
# Go2 SDK2 使用的网卡名；你的真机是 eth0。
SDK_NETWORK_INTERFACE = 'eth0'

# SDK 动作小工具路径。空字符串表示自动查找 install 里的 go2_sdk_motion_action。
SDK_ACTION_EXECUTABLE = ''

# SDK 动作等待超时时间会在动作等待时间基础上额外加这一段。
SDK_ACTION_TIMEOUT_PADDING_SEC = 6.0

# 普通 SDK 动作前先发一小段 0 速度，避免 UDP Move 还在持续上一次速度。
SDK_ACTION_PRE_STOP_SEC = 0.25

# 前跳对状态要求更严格：必须完全停住、站稳后再调用 FrontJump。
# Unitree SDK 的 FrontJump 不暴露“慢速蹲下”参数，这里用更长停稳时间保护关节。
FRONT_JUMP_CMD_STOP_SEC = 2.00

# 前跳前先 balance_stand 等待多久。
FRONT_JUMP_PRE_BALANCE_WAIT_SEC = 0.80

# FrontJump 动作发出后等待多久。
FRONT_JUMP_ACTION_WAIT_SEC = 2.00

# 前跳完成后 recovery_stand 等待多久。
FRONT_JUMP_RECOVERY_WAIT_SEC = 1.00

# Ctrl+C 或异常时持续发送停止命令的时间。
EMERGENCY_STOP_SEC = 1.20

# 如果 SDK 动作小工具不存在，是否允许跳过跳跃/站立动作。比赛必须保持 False。
RUN_WITHOUT_SDK_ACTIONS = False

# 是否允许退回 ROS topic 调 Sport API。你这台容易 typesupport 冲突，保持 False。
ALLOW_ROS_TOPIC_SDK_ACTIONS = False

# ROS topic 方式调用 Sport API 时的话题名，默认不用改。
SPORT_REQUEST_TOPIC = '/api/sport/request'

# Unitree Sport API 的动作编号，一般不要改。
SDK_ACTION_API_IDS = {
    'stand_up': 1004,
    'balance_stand': 1002,
    'stop_move': 1003,
    'recovery_stand': 1006,
    'economic_gait': 1063,
    'front_jump': 1031,
}

# SDK 动作小工具运行时需要的动态库路径，一般不要改。
SDK_LD_LIBRARY_PATH_PREFIX = (
    '/home/unitree/rk_inspection_ws/third_party/unitree_sdk2_official/'
    'thirdparty/lib/aarch64',
    '/home/unitree/rk_inspection_ws/install/rk_go2_sdk_bridge/lib',
    '/home/unitree/rk_inspection_ws/third_party/unitree_sdk2_official/'
    'thirdparty/lib/x86_64',
    '/usr/local/lib',
    '/home/unitree/cyclonedds_ws/install/cyclonedds/lib',
)

# ------------------------- 整体流程开关 -------------------------
# 是否启用启停区到避障区的完整起步流程。
ENABLE_START_SEQUENCE = True

# 是否从趴下状态恢复站立；现在你说开始不用蹲下，所以保持 False。
START_FROM_PRONE = False

# 是否执行前跳动作；调试纯巡线/纯避障时可以临时改 False。
ENABLE_FRONT_JUMP = True

# 是否执行避障区写死路线。
ENABLE_OBSTACLE_ROUTE = True

# ------------------------- ROS 话题名 -------------------------
# 给巡线节点发开始巡线的 topic。
MISSION_START_TOPIC = '/mission/start'

# 给巡线节点发停止巡线的 topic。
MISSION_STOP_TOPIC = '/mission/stop'

# 黑线识别结果 topic。
LINE_TRACK_TOPIC = '/perception/line_track'


def forward(steps, speed_mps=DEFAULT_FORWARD_SPEED_MPS):
    """避障区：直走多少步。"""
    return {
        'type': 'forward',
        'steps': int(steps),
        'speed_mps': float(speed_mps),
    }


def left(degrees, vx_mps=DEFAULT_TURN_FORWARD_SPEED_MPS,
         wz_radps=DEFAULT_TURN_SPEED_RADPS):
    """避障区：边走边左转多少度。"""
    return {
        'type': 'turn',
        'direction': 'left',
        'degrees': float(degrees),
        'vx_mps': float(vx_mps),
        'wz_radps': float(wz_radps),
    }


def right(degrees, vx_mps=DEFAULT_TURN_FORWARD_SPEED_MPS,
          wz_radps=DEFAULT_TURN_SPEED_RADPS):
    """避障区：边走边右转多少度。"""
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
                    f'避障动作{index}：边走边'
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
#   left(140)        # 边走边左转140度
#   right(120, vx_mps=0.35)  # 右转120度，转弯时前进速度0.35
#
# 调参建议：
#   - 直走距离不够：改 forward(...) 里的步数。
#   - 转弯角度不够：改 left/right(...) 里的角度。
#   - 转弯像原地转：把 vx_mps 加到 0.35。
#   - 转弯半径太大：把 vx_mps 降到 0.27。
OBSTACLE_ROUTE = [
    forward(8),      # 入口向上直走
    left(130),        # 左转进入顶部横向通道
    forward(3),
    left(130),        # 左转进入中间向下通道
    forward(9),       # 中间通道向下
    right(120),       # 右转进入底部横向通道
    forward(9),       # 底部通道前进
    right(160),       # 右转进入左侧向上通道
    forward(9),       # 左侧通道向上
    left(160),        # 左转对准出口
    forward(9),       # 出口直走
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
    # 第0-2阶段：启停区直接开始 SDK 巡线，直到识别到长白线。
    # 这里不切续航步态，避免每次切换步态造成急停和动作不连贯。
    # 要改等待长白线的最长时间，就改 START_LINE_FOLLOW_UNTIL_WHITE_MAX_SEC。
    RouteStage(
        name='line_follow_until_white_line',
        description=(
            '第0-2阶段：启停区 SDK 巡线，检测到长白线后停下'
        ),
        line_follow_until_white_line=True,
        line_follow_duration_sec=START_LINE_FOLLOW_UNTIL_WHITE_MAX_SEC,
        line_follow_speed_mps=LINE_FOLLOW_ESTIMATED_SPEED_MPS,
        enabled=ENABLE_START_SEQUENCE and ENABLE_FRONT_JUMP,
    ),
    # 第0-3阶段：检测到长白线后，再向前走一步，然后接前跳。
    RouteStage(
        name='forward_after_white_line',
        description=(
            f'第0-3阶段：长白线后向前直走 '
            f'{START_FORWARD_AFTER_WHITE_STEPS} 步，再准备前跳'
        ),
        forward_steps=START_FORWARD_AFTER_WHITE_STEPS,
        forward_speed_mps=START_FORWARD_SPEED_MPS,
        enabled=ENABLE_START_SEQUENCE and ENABLE_FRONT_JUMP,
    ),
    # 第0-4阶段：前跳前先站稳，降低从巡线速度直接进入跳跃动作的冲击。
    RouteStage(
        name='prepare_front_jump_balance',
        description='第0-4阶段：前跳前站稳，保护关节',
        sdk_action='balance_stand',
        sdk_wait_sec=FRONT_JUMP_PRE_BALANCE_WAIT_SEC,
        enabled=ENABLE_START_SEQUENCE and ENABLE_FRONT_JUMP,
    ),
    # 第0-5阶段：执行一次前跳。
    RouteStage(
        name='front_jump',
        description='第0-5阶段：执行一次前跳，跳前已停稳',
        sdk_action='front_jump',
        sdk_wait_sec=FRONT_JUMP_ACTION_WAIT_SEC,
        enabled=ENABLE_START_SEQUENCE and ENABLE_FRONT_JUMP,
    ),
    # 第0-6阶段：跳完后恢复站立。
    RouteStage(
        name='recover_after_front_jump',
        description='第0-6阶段：跳完后恢复站立',
        sdk_action='recovery_stand',
        sdk_wait_sec=FRONT_JUMP_RECOVERY_WAIT_SEC,
        enabled=ENABLE_START_SEQUENCE and ENABLE_FRONT_JUMP,
    ),
    # 第0-7阶段：跳后继续调用现有 SDK 巡线3秒。
    # 这里允许一开始没看见线：巡线节点会自己进入找线状态，找到线后继续跟线。
    RouteStage(
        name='line_follow_after_jump',
        description=(
            f'第0-7阶段：跳后继续巡线 {POST_JUMP_LINE_FOLLOW_SEC:.1f} 秒，'
            '然后确认位于黑线中间'
        ),
        line_follow_duration_sec=POST_JUMP_LINE_FOLLOW_SEC,
        line_follow_speed_mps=LINE_FOLLOW_ESTIMATED_SPEED_MPS,
        line_follow_require_visible=False,
        line_follow_finish_centered=True,
        enabled=ENABLE_START_SEQUENCE,
    ),

    # 避障区开始前切换续航步态，后面的写死路线全程沿用这个步态。
    RouteStage(
        name='obstacle_economic_gait',
        description='避障区开始前：切换续航步态',
        sdk_action='economic_gait',
        sdk_wait_sec=ECONOMIC_GAIT_WAIT_SEC,
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
        self.line_track_topic = self.declare_parameter(
            'line_track_topic',
            LINE_TRACK_TOPIC
        ).value
        self.white_line_detection_enabled = bool(self.declare_parameter(
            'white_line_detection_enabled',
            WHITE_LINE_DETECTION_ENABLED
        ).value)
        self.white_line_image_topic = self.declare_parameter(
            'white_line_image_topic',
            WHITE_LINE_IMAGE_TOPIC
        ).value
        self.white_line_roi_top_fraction = float(self.declare_parameter(
            'white_line_roi_top_fraction',
            WHITE_LINE_ROI_TOP_FRACTION
        ).value)
        self.white_line_roi_bottom_fraction = float(self.declare_parameter(
            'white_line_roi_bottom_fraction',
            WHITE_LINE_ROI_BOTTOM_FRACTION
        ).value)
        self.white_line_roi_side_margin_fraction = float(
            self.declare_parameter(
                'white_line_roi_side_margin_fraction',
                WHITE_LINE_ROI_SIDE_MARGIN_FRACTION
            ).value
        )
        self.white_line_min_width_fraction = float(self.declare_parameter(
            'white_line_min_width_fraction',
            WHITE_LINE_MIN_WIDTH_FRACTION
        ).value)
        self.white_line_min_height_fraction = float(self.declare_parameter(
            'white_line_min_height_fraction',
            WHITE_LINE_MIN_HEIGHT_FRACTION
        ).value)
        self.white_line_max_height_fraction = float(self.declare_parameter(
            'white_line_max_height_fraction',
            WHITE_LINE_MAX_HEIGHT_FRACTION
        ).value)
        self.white_line_min_aspect_ratio = float(self.declare_parameter(
            'white_line_min_aspect_ratio',
            WHITE_LINE_MIN_ASPECT_RATIO
        ).value)
        self.white_line_min_value = int(self.declare_parameter(
            'white_line_min_value',
            WHITE_LINE_MIN_VALUE
        ).value)
        self.white_line_max_saturation = int(self.declare_parameter(
            'white_line_max_saturation',
            WHITE_LINE_MAX_SATURATION
        ).value)
        self.white_line_edge_detection_enabled = bool(self.declare_parameter(
            'white_line_edge_detection_enabled',
            WHITE_LINE_EDGE_DETECTION_ENABLED
        ).value)
        self.white_line_edge_min_row_fraction = float(self.declare_parameter(
            'white_line_edge_min_row_fraction',
            WHITE_LINE_EDGE_MIN_ROW_FRACTION
        ).value)
        self.white_line_edge_min_gradient = float(self.declare_parameter(
            'white_line_edge_min_gradient',
            WHITE_LINE_EDGE_MIN_GRADIENT
        ).value)
        self.white_line_edge_min_gap_fraction = float(self.declare_parameter(
            'white_line_edge_min_gap_fraction',
            WHITE_LINE_EDGE_MIN_GAP_FRACTION
        ).value)
        self.white_line_edge_max_gap_fraction = float(self.declare_parameter(
            'white_line_edge_max_gap_fraction',
            WHITE_LINE_EDGE_MAX_GAP_FRACTION
        ).value)
        self.white_line_max_line_confidence = float(self.declare_parameter(
            'white_line_max_line_confidence',
            WHITE_LINE_MAX_LINE_CONFIDENCE
        ).value)
        self.white_line_stable_sec = float(self.declare_parameter(
            'white_line_stable_sec',
            WHITE_LINE_STABLE_SEC
        ).value)
        self.white_line_stale_sec = float(self.declare_parameter(
            'white_line_stale_sec',
            WHITE_LINE_STALE_SEC
        ).value)
        self.center_before_obstacle_timeout_sec = float(
            self.declare_parameter(
                'center_before_obstacle_timeout_sec',
                CENTER_BEFORE_OBSTACLE_TIMEOUT_SEC
            ).value
        )
        self.center_before_obstacle_stable_sec = float(self.declare_parameter(
            'center_before_obstacle_stable_sec',
            CENTER_BEFORE_OBSTACLE_STABLE_SEC
        ).value)
        self.center_before_obstacle_min_confidence = float(
            self.declare_parameter(
                'center_before_obstacle_min_confidence',
                CENTER_BEFORE_OBSTACLE_MIN_CONFIDENCE
            ).value
        )
        self.center_before_obstacle_max_lateral_error = float(
            self.declare_parameter(
                'center_before_obstacle_max_lateral_error',
                CENTER_BEFORE_OBSTACLE_MAX_LATERAL_ERROR
            ).value
        )
        self.center_before_obstacle_max_heading_error = float(
            self.declare_parameter(
                'center_before_obstacle_max_heading_error',
                CENTER_BEFORE_OBSTACLE_MAX_HEADING_ERROR
            ).value
        )
        self.line_visible_wait_timeout_sec = float(self.declare_parameter(
            'line_visible_wait_timeout_sec',
            LINE_VISIBLE_WAIT_TIMEOUT_SEC
        ).value)
        self.line_lost_switch_sec = float(self.declare_parameter(
            'line_lost_switch_sec',
            LINE_LOST_SWITCH_SEC
        ).value)
        self.line_track_stale_sec = float(self.declare_parameter(
            'line_track_stale_sec',
            LINE_TRACK_STALE_SEC
        ).value)
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
        self._last_line_track_msg = None
        self._last_line_track_time = None
        self._last_white_line_time = None
        self._last_white_line_detected = False
        self._white_line_candidate_since = None
        self._last_white_line_score = 0.0
        self._last_white_line_detail = 'not_checked'

        self._validate_parameters()
        self.publisher = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.line_track_subscription = self.create_subscription(
            LineTrack,
            self.line_track_topic,
            self._on_line_track,
            10
        )
        self.white_line_image_subscription = None
        if self.white_line_detection_enabled and Image is not None:
            self.white_line_image_subscription = self.create_subscription(
                Image,
                self.white_line_image_topic,
                self._on_white_line_image,
                5
            )
        elif self.white_line_detection_enabled:
            self.get_logger().warn(
                'white line image detection requested but sensor_msgs/Image '
                'or OpenCV/Numpy is unavailable; line-lost fallback remains.'
            )
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
        if not self.line_track_topic:
            raise ValueError('line_track_topic must not be empty')
        if self.white_line_detection_enabled and not self.white_line_image_topic:
            raise ValueError('white_line_image_topic must not be empty')

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
            'line_visible_wait_timeout_sec': (
                self.line_visible_wait_timeout_sec
            ),
            'line_lost_switch_sec': self.line_lost_switch_sec,
            'line_track_stale_sec': self.line_track_stale_sec,
            'white_line_roi_top_fraction': self.white_line_roi_top_fraction,
            'white_line_roi_bottom_fraction': (
                self.white_line_roi_bottom_fraction
            ),
            'white_line_roi_side_margin_fraction': (
                self.white_line_roi_side_margin_fraction
            ),
            'white_line_min_width_fraction': (
                self.white_line_min_width_fraction
            ),
            'white_line_min_height_fraction': (
                self.white_line_min_height_fraction
            ),
            'white_line_max_height_fraction': (
                self.white_line_max_height_fraction
            ),
            'white_line_min_aspect_ratio': self.white_line_min_aspect_ratio,
            'white_line_edge_min_row_fraction': (
                self.white_line_edge_min_row_fraction
            ),
            'white_line_edge_min_gradient': self.white_line_edge_min_gradient,
            'white_line_edge_min_gap_fraction': (
                self.white_line_edge_min_gap_fraction
            ),
            'white_line_edge_max_gap_fraction': (
                self.white_line_edge_max_gap_fraction
            ),
            'white_line_max_line_confidence': (
                self.white_line_max_line_confidence
            ),
            'white_line_stable_sec': self.white_line_stable_sec,
            'white_line_stale_sec': self.white_line_stale_sec,
            'center_before_obstacle_timeout_sec': (
                self.center_before_obstacle_timeout_sec
            ),
            'center_before_obstacle_stable_sec': (
                self.center_before_obstacle_stable_sec
            ),
            'center_before_obstacle_min_confidence': (
                self.center_before_obstacle_min_confidence
            ),
            'center_before_obstacle_max_lateral_error': (
                self.center_before_obstacle_max_lateral_error
            ),
            'center_before_obstacle_max_heading_error': (
                self.center_before_obstacle_max_heading_error
            ),
        }
        for name, value in nonnegative.items():
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(
                    f'{name} must be a finite nonnegative number'
                )
        if self.white_line_roi_bottom_fraction <= (
            self.white_line_roi_top_fraction
        ):
            raise ValueError(
                'white_line_roi_bottom_fraction must be greater than '
                'white_line_roi_top_fraction'
            )
        if self.white_line_min_height_fraction > (
            self.white_line_max_height_fraction
        ):
            raise ValueError(
                'white_line_min_height_fraction must be <= '
                'white_line_max_height_fraction'
            )
        if self.white_line_edge_min_gap_fraction > (
            self.white_line_edge_max_gap_fraction
        ):
            raise ValueError(
                'white_line_edge_min_gap_fraction must be <= '
                'white_line_edge_max_gap_fraction'
            )
        if not 0.0 <= self.white_line_edge_min_row_fraction <= 1.0:
            raise ValueError(
                'white_line_edge_min_row_fraction must be in [0.0, 1.0]'
            )
        if not 0.0 <= self.white_line_max_line_confidence <= 1.0:
            raise ValueError(
                'white_line_max_line_confidence must be in [0.0, 1.0]'
            )
        if not 0.0 <= self.center_before_obstacle_min_confidence <= 1.0:
            raise ValueError(
                'center_before_obstacle_min_confidence must be in [0.0, 1.0]'
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
                stage.line_follow_until_lost
                or stage.line_follow_until_white_line
                or stage.line_follow_steps > 0
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
            if stage.line_follow_finish_centered and not has_line_follow:
                raise ValueError(
                    f'{stage.name} line_follow_finish_centered requires a '
                    'line_follow stage'
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

        if stage.line_follow_until_lost:
            self._run_line_follow_until_lost(index, total, stage)
            return

        if stage.line_follow_until_white_line:
            self._run_line_follow_until_white_line(index, total, stage)
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

    def _on_line_track(self, msg):
        self._last_line_track_msg = msg
        self._last_line_track_time = time.monotonic()

    def _on_white_line_image(self, msg):
        if not self.white_line_detection_enabled:
            return

        image = self._image_msg_to_bgr(msg)
        if image is None:
            self._last_white_line_detected = False
            self._white_line_candidate_since = None
            self._last_white_line_detail = 'image_decode_failed'
            self._last_white_line_time = time.monotonic()
            return

        detected, score, detail = self._detect_long_white_line(image)
        now = time.monotonic()
        self._last_white_line_detected = detected
        self._last_white_line_score = score
        self._last_white_line_detail = detail
        self._last_white_line_time = now

        if detected:
            if self._white_line_candidate_since is None:
                self._white_line_candidate_since = now
        else:
            self._white_line_candidate_since = None

    def _image_msg_to_bgr(self, msg):
        if cv2 is None or np is None:
            return None

        encoding = str(msg.encoding or '').lower()
        height = int(msg.height)
        width = int(msg.width)
        step = int(msg.step)
        if height <= 0 or width <= 0 or step <= 0:
            return None

        try:
            data = np.frombuffer(msg.data, dtype=np.uint8)
            rows = data.reshape((height, step))
            if encoding in ('bgr8', 'rgb8'):
                channels = 3
                image = rows[:, :width * channels].reshape(
                    (height, width, channels)
                )
                if encoding == 'rgb8':
                    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
                return image
            if encoding in ('bgra8', 'rgba8'):
                channels = 4
                image = rows[:, :width * channels].reshape(
                    (height, width, channels)
                )
                if encoding == 'rgba8':
                    return cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
                return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
            if encoding in ('mono8', '8uc1'):
                gray = rows[:, :width].reshape((height, width))
                return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        except (ValueError, cv2.error):
            return None

        return None

    def _detect_long_white_line(self, image_bgr):
        height, width = image_bgr.shape[:2]
        y0 = int(max(0.0, min(0.98, self.white_line_roi_top_fraction)) * height)
        y1 = int(max(0.01, min(1.0, self.white_line_roi_bottom_fraction)) * height)
        x_margin = int(
            max(0.0, min(0.45, self.white_line_roi_side_margin_fraction))
            * width
        )
        x0 = x_margin
        x1 = max(x0 + 1, width - x_margin)
        if y1 <= y0 + 1:
            return False, 0.0, 'empty_roi'

        roi = image_bgr[y0:y1, x0:x1]
        roi_h, roi_w = roi.shape[:2]
        if roi_h <= 0 or roi_w <= 0:
            return False, 0.0, 'empty_roi'

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        lower = np.array([0, 0, self.white_line_min_value], dtype=np.uint8)
        upper = np.array([180, self.white_line_max_saturation, 255],
                         dtype=np.uint8)
        mask = cv2.inRange(hsv, lower, upper)

        kernel_w = max(9, int(roi_w * 0.035))
        if kernel_w % 2 == 0:
            kernel_w += 1
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (kernel_w, 3))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))

        result = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )
        contours = result[0] if len(result) == 2 else result[1]

        best_score = 0.0
        best_detail = 'no_candidate'
        min_w = self.white_line_min_width_fraction
        min_h = self.white_line_min_height_fraction
        max_h = self.white_line_max_height_fraction

        for contour in contours:
            x, y, box_w, box_h = cv2.boundingRect(contour)
            width_fraction = float(box_w) / float(max(1, roi_w))
            height_fraction = float(box_h) / float(max(1, roi_h))
            aspect = float(box_w) / float(max(1, box_h))
            area_fraction = float(cv2.contourArea(contour)) / float(
                max(1, roi_w * roi_h)
            )
            score = width_fraction * min(1.0, aspect / 10.0)
            if score > best_score:
                best_score = score
                best_detail = (
                    f'w={width_fraction:.2f},h={height_fraction:.2f},'
                    f'aspect={aspect:.1f},area={area_fraction:.3f}'
                )

            if width_fraction < min_w:
                continue
            if height_fraction < min_h or height_fraction > max_h:
                continue
            if aspect < self.white_line_min_aspect_ratio:
                continue

            return True, score, (
                f'accepted:{best_detail},roi=({x0},{y0})-({x1},{y1})'
            )

        edge_detected, edge_score, edge_detail = (
            self._detect_raised_white_strip_edges(roi)
        )
        if edge_detected:
            return True, edge_score, (
                f'edge_accepted:{edge_detail},roi=({x0},{y0})-({x1},{y1})'
            )

        return False, max(best_score, edge_score), (
            f'{best_detail};edge={edge_detail}'
        )

    def _detect_raised_white_strip_edges(self, roi_bgr):
        if not self.white_line_edge_detection_enabled:
            return False, 0.0, 'edge_disabled'

        roi_h, roi_w = roi_bgr.shape[:2]
        if roi_h < 12 or roi_w < 20:
            return False, 0.0, 'edge_roi_too_small'

        hsv = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)
        saturation = hsv[:, :, 1]
        value = hsv[:, :, 2]
        gray = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)

        gradient_y = cv2.Sobel(gray, cv2.CV_16S, 0, 1, ksize=3)
        gradient_y = cv2.convertScaleAbs(gradient_y)
        adaptive_threshold = float(np.percentile(gradient_y, 92))
        threshold = max(
            float(self.white_line_edge_min_gradient),
            adaptive_threshold
        )
        edge_mask = gradient_y >= threshold

        bright_mask = (
            (value >= max(0, self.white_line_min_value - 25))
            & (saturation <= min(255, self.white_line_max_saturation + 45))
        )
        edge_mask = edge_mask & bright_mask

        row_scores = edge_mask.mean(axis=1).astype(np.float32)
        if row_scores.size < 5:
            return False, 0.0, 'edge_no_rows'

        row_scores = cv2.blur(row_scores.reshape(-1, 1), (1, 5)).ravel()
        candidate_rows = np.flatnonzero(
            row_scores >= self.white_line_edge_min_row_fraction
        )
        if candidate_rows.size == 0:
            return False, float(row_scores.max(initial=0.0)), 'edge_no_row'

        clusters = []
        cluster_start = int(candidate_rows[0])
        previous = int(candidate_rows[0])
        for row in candidate_rows[1:]:
            row = int(row)
            if row == previous + 1:
                previous = row
                continue
            clusters.append((cluster_start, previous))
            cluster_start = row
            previous = row
        clusters.append((cluster_start, previous))

        centers = [
            (start + end) // 2
            for start, end in clusters
            if end >= start
        ]
        best_score = 0.0
        best_detail = f'edge_clusters={len(centers)}'

        min_gap = self.white_line_edge_min_gap_fraction
        max_gap = self.white_line_edge_max_gap_fraction
        for first_index, top in enumerate(centers):
            for bottom in centers[first_index + 1:]:
                gap_fraction = float(bottom - top) / float(max(1, roi_h))
                if gap_fraction < min_gap or gap_fraction > max_gap:
                    continue

                y0 = max(0, min(top, bottom))
                y1 = min(roi_h, max(top, bottom) + 1)
                band_bright_fraction = float(bright_mask[y0:y1, :].mean())
                edge_width_fraction = min(
                    float(row_scores[top]),
                    float(row_scores[bottom])
                )
                score = edge_width_fraction * min(1.0, band_bright_fraction)
                detail = (
                    f'edge_pair=({top},{bottom}),gap={gap_fraction:.3f},'
                    f'edge_w={edge_width_fraction:.2f},'
                    f'bright={band_bright_fraction:.2f}'
                )
                if score > best_score:
                    best_score = score
                    best_detail = detail

                if (
                    edge_width_fraction >= self.white_line_edge_min_row_fraction
                    and band_bright_fraction >= 0.65
                ):
                    return True, score, detail

        return False, best_score, best_detail

    def _white_line_is_visible_now(self):
        if not self.white_line_detection_enabled:
            return False, 'white_detection_disabled'
        if self.white_line_image_subscription is None:
            return False, 'white_image_subscription_unavailable'
        if self._last_white_line_time is None:
            return False, 'no_white_image'

        now = time.monotonic()
        age = now - float(self._last_white_line_time)
        if age > self.white_line_stale_sec:
            return False, f'white_image_stale_{age:.2f}s'
        if not self._last_white_line_detected:
            return False, f'white_not_detected:{self._last_white_line_detail}'

        stable_for = now - float(self._white_line_candidate_since or now)
        if stable_for < self.white_line_stable_sec:
            return False, f'white_candidate_{stable_for:.2f}s'

        return True, (
            f'white_line score={self._last_white_line_score:.2f}, '
            f'stable={stable_for:.2f}s, {self._last_white_line_detail}'
        )

    def _line_is_visible_now(self):
        now = time.monotonic()
        if self._last_line_track_msg is None:
            return False, 'no_line_track'

        age = now - float(self._last_line_track_time or now)
        if age > self.line_track_stale_sec:
            return False, f'line_track_stale_{age:.2f}s'

        if not bool(self._last_line_track_msg.line_visible):
            return False, 'line_visible_false'

        confidence = float(self._last_line_track_msg.confidence)
        if confidence < ROUTE_LINE_MIN_CONFIDENCE:
            return False, f'confidence_low_{confidence:.2f}'

        lateral_error = abs(float(self._last_line_track_msg.lateral_error))
        if lateral_error > ROUTE_LINE_MAX_ABS_LATERAL_ERROR:
            return False, f'lateral_error_large_{lateral_error:.2f}'

        return True, 'line_visible'

    def _line_is_centered_now(self):
        now = time.monotonic()
        if self._last_line_track_msg is None:
            return False, 'center_no_line_track'

        age = now - float(self._last_line_track_time or now)
        if age > self.line_track_stale_sec:
            return False, f'center_line_track_stale_{age:.2f}s'

        msg = self._last_line_track_msg
        if not bool(msg.line_visible):
            return False, 'center_line_visible_false'

        confidence = float(msg.confidence)
        if confidence < self.center_before_obstacle_min_confidence:
            return False, (
                f'center_confidence_low_{confidence:.2f}<'
                f'{self.center_before_obstacle_min_confidence:.2f}'
            )

        lateral_error = abs(float(msg.lateral_error))
        if lateral_error > self.center_before_obstacle_max_lateral_error:
            return False, (
                f'center_lateral_large_{lateral_error:.3f}>'
                f'{self.center_before_obstacle_max_lateral_error:.3f}'
            )

        heading_error = abs(float(msg.heading_error))
        if heading_error > self.center_before_obstacle_max_heading_error:
            return False, (
                f'center_heading_large_{heading_error:.3f}>'
                f'{self.center_before_obstacle_max_heading_error:.3f}'
            )

        return True, (
            f'center_ok confidence={confidence:.2f}, '
            f'lateral={float(msg.lateral_error):.3f}, '
            f'heading={float(msg.heading_error):.3f}, age={age:.2f}s'
        )

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

        if stage.line_follow_require_visible:
            if not self._wait_for_line_visible(stage.name):
                raise RuntimeError(
                    f'{stage.name}: cannot start line follow because no '
                    'usable black line is visible'
                )
        else:
            visible, reason = self._line_is_visible_now()
            if visible:
                msg = self._last_line_track_msg
                self.get_logger().info(
                    f'{stage.name}: black line already visible before '
                    f'line-follow restart, confidence={msg.confidence:.3f}, '
                    f'lateral={msg.lateral_error:.3f}, '
                    f'heading={msg.heading_error:.3f}'
                )
            else:
                self.get_logger().warn(
                    f'{stage.name}: black line is not visible yet '
                    f'({reason}); start the existing line follower anyway '
                    'so it can search and reacquire.'
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
        if stage.line_follow_finish_centered:
            self._wait_for_centered_line_before_obstacle(stage.name)
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

    def _run_line_follow_until_white_line(self, index, total, stage):
        max_duration_sec = stage.line_follow_duration_sec

        self.get_logger().warn(
            f'route stage {index}/{total}: {stage.name} '
            f'line_follow_until_white_line, '
            f'max_duration={max_duration_sec:.2f}s, '
            f'white_topic={self.white_line_image_topic}, '
            f'line_lost_fallback={self.line_lost_switch_sec:.2f}s'
        )

        if not self._wait_for_line_visible(stage.name):
            raise RuntimeError(
                f'{stage.name}: cannot start white-line search because no '
                'usable black line is visible'
            )

        self._publish_stop(
            f'before line follow until white line {stage.name}',
            LINE_FOLLOW_START_SETTLE_SEC
        )
        self._publish_mission_command(
            self.mission_start_publisher,
            True,
            f'{stage.name} mission_start',
            LINE_FOLLOW_START_SETTLE_SEC
        )

        period_sec = 1.0 / self.publish_rate_hz
        start_time = time.monotonic()
        lost_since = None
        last_report_time = 0.0
        stop_reason = 'white_line_not_found'

        while rclpy.ok():
            self._raise_if_stop_requested()
            now = time.monotonic()

            if max_duration_sec > 0.0 and now - start_time >= max_duration_sec:
                stop_reason = 'max_duration_reached'
                self.get_logger().warn(
                    f'{stage.name}: max duration reached without stable '
                    f'white-line detection ({max_duration_sec:.2f}s); '
                    'continue to the two-step-forward jump sequence'
                )
                break

            visible, line_reason = self._line_is_visible_now()
            msg = self._last_line_track_msg
            line_confidence = float(msg.confidence) if msg is not None else 0.0
            line_weak_for_white = (
                not visible
                or line_confidence <= self.white_line_max_line_confidence
            )

            white_visible, white_reason = self._white_line_is_visible_now()
            if white_visible and line_weak_for_white:
                stop_reason = 'white_line_detected'
                self.get_logger().warn(
                    f'{stage.name}: long white foam line detected: '
                    f'{white_reason}; line_confidence={line_confidence:.2f}, '
                    f'line_reason={line_reason}'
                )
                break
            if white_visible and not line_weak_for_white:
                if now - last_report_time >= 1.0:
                    self.get_logger().info(
                        f'{stage.name}: ignore white candidate while black '
                        f'line is still strong, '
                        f'confidence={line_confidence:.2f}, {white_reason}'
                    )
                    last_report_time = now

            if visible:
                lost_since = None
            else:
                if lost_since is None:
                    lost_since = now
                    self.get_logger().warn(
                        f'{stage.name}: black line lost candidate while '
                        f'waiting white line, reason={line_reason}'
                    )
                lost_for = now - lost_since
                if lost_for >= self.line_lost_switch_sec:
                    stop_reason = 'line_lost_assume_white_line'
                    self.get_logger().warn(
                        f'{stage.name}: black line lost for {lost_for:.2f}s '
                        f'(reason={line_reason}); assume long white line and '
                        'continue to two-step-forward jump sequence'
                    )
                    break

            if now - last_report_time >= 1.0:
                msg = self._last_line_track_msg
                if msg is None:
                    line_summary = 'line=no_msg'
                else:
                    age = now - float(self._last_line_track_time or now)
                    line_summary = (
                        f'line_visible={bool(msg.line_visible)}, '
                        f'conf={msg.confidence:.2f}, '
                        f'lateral={msg.lateral_error:.2f}, age={age:.2f}s'
                    )
                self.get_logger().info(
                    f'{stage.name}: searching white line, '
                    f'{line_summary}, white={white_reason}, '
                    f'white_score={self._last_white_line_score:.2f}'
                )
                last_report_time = now

            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period_sec)

        self._publish_mission_command(
            self.mission_stop_publisher,
            True,
            f'{stage.name} mission_stop ({stop_reason})',
            LINE_FOLLOW_STOP_SETTLE_SEC
        )
        self._publish_stop(
            f'after line follow until white line {stage.name}',
            LINE_FOLLOW_STOP_SETTLE_SEC
        )

    def _run_line_follow_until_lost(self, index, total, stage):
        max_duration_sec = stage.line_follow_duration_sec

        self.get_logger().warn(
            f'route stage {index}/{total}: {stage.name} line_follow_until_lost, '
            f'max_duration={max_duration_sec:.2f}s, '
            f'lost_switch={self.line_lost_switch_sec:.2f}s, '
            f'stale_timeout={self.line_track_stale_sec:.2f}s'
        )

        if not self._wait_for_line_visible(stage.name):
            self.get_logger().warn(
                f'{stage.name}: line not visible before line_follow_until_lost; '
                'skip blind line following and switch to hardcoded route'
            )
            self._publish_mission_command(
                self.mission_stop_publisher,
                True,
                f'{stage.name} mission_stop no visible line',
                LINE_FOLLOW_STOP_SETTLE_SEC
            )
            return
        self._publish_stop(
            f'before line follow until lost {stage.name}',
            LINE_FOLLOW_START_SETTLE_SEC
        )
        self._publish_mission_command(
            self.mission_start_publisher,
            True,
            f'{stage.name} mission_start',
            LINE_FOLLOW_START_SETTLE_SEC
        )

        period_sec = 1.0 / self.publish_rate_hz
        start_time = time.monotonic()
        lost_since = None
        last_report_time = 0.0
        stop_reason = 'line_lost'

        while rclpy.ok():
            self._raise_if_stop_requested()
            now = time.monotonic()

            if max_duration_sec > 0.0 and now - start_time >= max_duration_sec:
                stop_reason = 'max_duration_reached'
                self.get_logger().warn(
                    f'{stage.name}: max line-follow duration reached '
                    f'({max_duration_sec:.2f}s); switching to obstacle route'
                )
                break

            visible, reason = self._line_is_visible_now()
            if visible:
                if lost_since is not None:
                    self.get_logger().info(
                        f'{stage.name}: line recovered before switch, '
                        f'lost_for={now - lost_since:.2f}s'
                    )
                lost_since = None
            else:
                if lost_since is None:
                    lost_since = now
                    self.get_logger().warn(
                        f'{stage.name}: line lost candidate, reason={reason}'
                    )

                lost_for = now - lost_since
                if lost_for >= self.line_lost_switch_sec:
                    stop_reason = reason
                    self.get_logger().warn(
                        f'{stage.name}: line lost for {lost_for:.2f}s '
                        f'(reason={reason}); switching to hardcoded obstacle '
                        'route'
                    )
                    break

            if now - last_report_time >= 1.0:
                msg = self._last_line_track_msg
                if msg is None:
                    self.get_logger().info(
                        f'{stage.name}: running line follow, no line_track yet'
                    )
                else:
                    age = now - float(self._last_line_track_time or now)
                    self.get_logger().info(
                        f'{stage.name}: running line follow, '
                        f'visible={bool(msg.line_visible)}, '
                        f'confidence={msg.confidence:.3f}, '
                        f'lateral={msg.lateral_error:.3f}, '
                        f'heading={msg.heading_error:.3f}, '
                        f'age={age:.2f}s'
                    )
                last_report_time = now

            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period_sec)

        self._publish_mission_command(
            self.mission_stop_publisher,
            True,
            f'{stage.name} mission_stop ({stop_reason})',
            LINE_FOLLOW_STOP_SETTLE_SEC
        )
        self._publish_stop(
            f'after line follow until lost {stage.name}',
            LINE_FOLLOW_STOP_SETTLE_SEC
        )

    def _wait_for_line_visible(self, stage_name):
        timeout_sec = self.line_visible_wait_timeout_sec
        if timeout_sec <= 0.0:
            return True

        start_time = time.monotonic()
        last_report_time = 0.0
        self.get_logger().info(
            f'{stage_name}: waiting for line_visible on '
            f'{self.line_track_topic}, timeout={timeout_sec:.1f}s'
        )

        while rclpy.ok():
            self._raise_if_stop_requested()
            now = time.monotonic()
            visible, _reason = self._line_is_visible_now()
            if visible:
                msg = self._last_line_track_msg
                self.get_logger().info(
                    f'{stage_name}: line visible, '
                    f'confidence={msg.confidence:.3f}, '
                    f'lateral={msg.lateral_error:.3f}, '
                    f'heading={msg.heading_error:.3f}'
                )
                return True

            elapsed = now - start_time
            if elapsed >= timeout_sec:
                if self._last_line_track_msg is None:
                    self.get_logger().warn(
                        f'{stage_name}: no line_track message received '
                        f'within {timeout_sec:.1f}s'
                    )
                else:
                    self.get_logger().warn(
                        f'{stage_name}: line_track received but '
                        f'line not usable for {timeout_sec:.1f}s'
                    )
                return False

            if now - last_report_time >= 1.0:
                if self._last_line_track_msg is None:
                    self.get_logger().info(
                        f'{stage_name}: still waiting for first line_track'
                    )
                else:
                    self.get_logger().info(
                        f'{stage_name}: still waiting for visible line, '
                        f'last_visible={self._last_line_track_msg.line_visible}, '
                        f'confidence={self._last_line_track_msg.confidence:.3f}'
                    )
                last_report_time = now

            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.05)

    def _wait_for_centered_line_before_obstacle(self, stage_name):
        timeout_sec = self.center_before_obstacle_timeout_sec
        stable_sec = self.center_before_obstacle_stable_sec
        start_time = time.monotonic()
        centered_since = None
        last_report_time = 0.0

        self.get_logger().warn(
            f'{stage_name}: timed line-follow finished; keep line follower '
            'running until robot is centered on black line before obstacle, '
            f'timeout={timeout_sec:.2f}s, stable={stable_sec:.2f}s, '
            f'max_lateral={self.center_before_obstacle_max_lateral_error:.3f}, '
            f'max_heading={self.center_before_obstacle_max_heading_error:.3f}'
        )

        while rclpy.ok():
            self._raise_if_stop_requested()
            now = time.monotonic()
            centered, reason = self._line_is_centered_now()

            if centered:
                if centered_since is None:
                    centered_since = now
                    self.get_logger().info(
                        f'{stage_name}: center candidate started: {reason}'
                    )
                stable_for = now - centered_since
                if stable_for >= stable_sec:
                    self.get_logger().warn(
                        f'{stage_name}: centered and stable for '
                        f'{stable_for:.2f}s; switch to obstacle route. '
                        f'{reason}'
                    )
                    return
            else:
                if centered_since is not None:
                    self.get_logger().info(
                        f'{stage_name}: center candidate reset: {reason}'
                    )
                centered_since = None

            elapsed = now - start_time
            if timeout_sec > 0.0 and elapsed >= timeout_sec:
                raise RuntimeError(
                    f'{stage_name}: cannot enter obstacle route because robot '
                    f'is not centered on black line after {timeout_sec:.2f}s; '
                    f'last_reason={reason}'
                )

            if now - last_report_time >= 1.0:
                self.get_logger().info(
                    f'{stage_name}: centering before obstacle, '
                    f'elapsed={elapsed:.2f}s, {reason}'
                )
                last_report_time = now

            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.05)

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

        pre_stop_sec = self.sdk_action_pre_stop_sec
        if stage.sdk_action == 'front_jump':
            pre_stop_sec = max(pre_stop_sec, FRONT_JUMP_CMD_STOP_SEC)

        self._publish_stop(
            f'before sdk action {stage.name}',
            pre_stop_sec
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
