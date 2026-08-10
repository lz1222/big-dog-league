#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""楼梯任务唯一运行入口：巡线直到 T 标记，再交接到经典步态 Phase 3。

本脚本把相机、巡线、T 型识别、安全停车和 Phase 3 参数收敛为一条命令。
它不重复实现 Unitree SDK 通信：巡线仍经独立 UDP 安全链，Phase 3 仍调用已
编译验证的经典步态工具，避免一个巨型 Python 文件绕过速度上限、watchdog 或
停车确认。默认是纯观测验证，只有 ``--execute`` 才会让机器狗运动。
"""

import argparse
import os
import subprocess
import sys


# ===== 楼梯完整流程参数总表：现场只需改本表或对应命令行参数 =====
# UVC 的 /dev/videoN 编号会随重插或重启变化；使用序列号稳定链接固定选择楼梯相机。
DEFAULT_CAMERA_DEVICE = (
    '/dev/v4l/by-id/'
    'usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera_SN0001-video-index0'
)                                                # 当前指向 /dev/video6，禁止假定仍是 /dev/video0
DEFAULT_CAMERA_WIDTH = 320                       # 相机宽度（px），T 标记按此画面标定
DEFAULT_CAMERA_HEIGHT = 240                      # 相机高度（px）
DEFAULT_CAMERA_FPS = 15.0                        # 相机帧率（Hz）
DEFAULT_SDK_INTERFACE = 'eth1'                  # Go2 控制网卡，不使用无链路 eth0
DEFAULT_LINE_SPEED_MPS = 0.25                   # T 前持续巡线速度（m/s）
DEFAULT_LINE_DARK_THRESHOLD = 70                # 巡线黑色二值阈值，过大易把阴影当线
DEFAULT_LINE_KP_LATERAL = 0.85                   # 巡线横向偏差转向增益
DEFAULT_LINE_KP_HEADING = 0.35                   # 巡线朝向偏差转向增益
DEFAULT_LINE_MAX_WZ_RADPS = 0.28                # 巡线最大转向角速度（rad/s）
DEFAULT_T_CONFIRM_FRAMES = 5                     # T 标记连续确认帧数
DEFAULT_T_DARK_THRESHOLD = 70                    # T 标记黑色阈值
DEFAULT_T_BAND_MIN_WIDTH_RATIO = 0.65            # 顶部横带最小画面宽度比例
DEFAULT_T_LANE_MIN_WIDTH_RATIO = 0.04            # 下方竖带最小宽度比例
DEFAULT_T_LANE_MAX_WIDTH_RATIO = 0.28            # 下方竖带最大宽度比例
DEFAULT_ZERO_CONFIRM_MESSAGES = 3                # 交接前连续最终零速度消息数
DEFAULT_ZERO_CONFIRM_TIMEOUT_SEC = 2.0           # 等待零速度的失败关闭时限（s）
DEFAULT_STAIRS_UP_SPEED_MPS = 0.55              # T 后上台阶速度（m/s）
DEFAULT_STAIRS_UP_DURATION_SEC = 3.9            # 上台阶时间（s）
DEFAULT_TURN_LEFT_ANGLE_DEG = 79.0              # 左转目标角（度，实时 yaw 完成）
DEFAULT_TURN_LEFT_WZ_RADPS = 1.0                # 左转角速度（rad/s）
DEFAULT_STAIRS_DOWN_SPEED_MPS = 0.55            # 下台阶速度（m/s）
DEFAULT_STAIRS_DOWN_DURATION_SEC = 2.9          # 下台阶时间（s）
DEFAULT_INTER_STAGE_STOP_SEC = 0.4              # 各动作之间 StopMove 稳定时间（s）


def bounded_float(lower, upper):
    """为运动参数建立启动前边界，拒绝 NaN、负速度和超过 SDK 上限的值。"""
    def parse(raw):
        value = float(raw)
        if value != value or value < lower or value > upper:
            raise argparse.ArgumentTypeError(
                '参数必须在 {} 到 {} 之间'.format(lower, upper))
        return value
    return parse


def build_parser():
    """定义唯一入口的全部参数；参数名与底层独立 launch 一一对应。"""
    parser = argparse.ArgumentParser(
        description='独立楼梯完整流程：巡线 → T 识别 → 安全停车 → Phase 3')
    parser.add_argument('--start-camera', action='store_true',
                        help='由本流程打开 /dev/video0；已有相机节点时不要添加')
    parser.add_argument('--device', default=DEFAULT_CAMERA_DEVICE,
                        help='楼梯 Sonix 相机稳定设备链接；不要固定猜测 /dev/videoN')
    parser.add_argument('--camera-width', type=int, default=DEFAULT_CAMERA_WIDTH,
                        help='相机宽度 px，默认 320')
    parser.add_argument('--camera-height', type=int, default=DEFAULT_CAMERA_HEIGHT,
                        help='相机高度 px，默认 240')
    parser.add_argument('--camera-fps', type=bounded_float(1.0, 60.0),
                        default=DEFAULT_CAMERA_FPS, help='相机帧率 Hz')
    parser.add_argument('--interface', default=DEFAULT_SDK_INTERFACE,
                        help='Go2 SDK 网卡，默认 eth1')
    parser.add_argument('--line-speed-mps', type=bounded_float(0.0, 0.25),
                        default=DEFAULT_LINE_SPEED_MPS,
                        help='T 前持续巡线速度 m/s，范围 0 至 0.25')
    parser.add_argument('--line-dark-threshold', type=int,
                        default=DEFAULT_LINE_DARK_THRESHOLD,
                        help='巡线黑色阈值，范围 0 至 255')
    parser.add_argument('--line-kp-lateral', type=bounded_float(0.0, 5.0),
                        default=DEFAULT_LINE_KP_LATERAL,
                        help='巡线横向偏差转向增益')
    parser.add_argument('--line-kp-heading', type=bounded_float(0.0, 5.0),
                        default=DEFAULT_LINE_KP_HEADING,
                        help='巡线朝向偏差转向增益')
    parser.add_argument('--line-max-wz-radps', type=bounded_float(0.01, 0.60),
                        default=DEFAULT_LINE_MAX_WZ_RADPS,
                        help='巡线最大转向角速度 rad/s')
    parser.add_argument('--t-confirm-frames', type=int,
                        default=DEFAULT_T_CONFIRM_FRAMES,
                        help='连续多少帧确认 T 标记，默认 5')
    parser.add_argument('--t-dark-threshold', type=int,
                        default=DEFAULT_T_DARK_THRESHOLD,
                        help='T 标记黑色阈值，范围 0 至 255')
    parser.add_argument('--t-band-min-width-ratio', type=bounded_float(0.0, 1.0),
                        default=DEFAULT_T_BAND_MIN_WIDTH_RATIO,
                        help='T 顶部横带最小宽度比例')
    parser.add_argument('--t-lane-min-width-ratio', type=bounded_float(0.0, 1.0),
                        default=DEFAULT_T_LANE_MIN_WIDTH_RATIO,
                        help='T 下方竖带最小宽度比例')
    parser.add_argument('--t-lane-max-width-ratio', type=bounded_float(0.0, 1.0),
                        default=DEFAULT_T_LANE_MAX_WIDTH_RATIO,
                        help='T 下方竖带最大宽度比例')
    parser.add_argument('--zero-confirm-messages', type=int,
                        default=DEFAULT_ZERO_CONFIRM_MESSAGES,
                        help='交接前连续最终零速度消息数')
    parser.add_argument('--zero-confirm-timeout-sec', type=bounded_float(0.1, 10.0),
                        default=DEFAULT_ZERO_CONFIRM_TIMEOUT_SEC,
                        help='等待停车确认超时 s')
    parser.add_argument('--stairs-up-speed-mps', type=bounded_float(0.0, 0.60),
                        default=DEFAULT_STAIRS_UP_SPEED_MPS,
                        help='T 后上台阶速度 m/s')
    parser.add_argument('--stairs-up-duration-sec', type=bounded_float(0.01, 5.0),
                        default=DEFAULT_STAIRS_UP_DURATION_SEC,
                        help='T 后上台阶时间 s')
    parser.add_argument('--stairs-turn-angle-deg', type=bounded_float(0.01, 90.0),
                        default=DEFAULT_TURN_LEFT_ANGLE_DEG,
                        help='T 后左转目标角度')
    parser.add_argument('--stairs-turn-wz-radps', type=bounded_float(0.01, 1.20),
                        default=DEFAULT_TURN_LEFT_WZ_RADPS,
                        help='T 后左转角速度 rad/s')
    parser.add_argument('--stairs-down-speed-mps', type=bounded_float(0.0, 0.60),
                        default=DEFAULT_STAIRS_DOWN_SPEED_MPS,
                        help='T 后下台阶速度 m/s')
    parser.add_argument('--stairs-down-duration-sec', type=bounded_float(0.01, 5.0),
                        default=DEFAULT_STAIRS_DOWN_DURATION_SEC,
                        help='T 后下台阶时间 s')
    parser.add_argument('--inter-stage-stop-sec', type=bounded_float(0.0, 2.0),
                        default=DEFAULT_INTER_STAGE_STOP_SEC,
                        help='每段动作 StopMove 后稳定时间 s')
    parser.add_argument('--execute', action='store_true',
                        help='明确授权真机执行巡线和 T 后 Phase 3；默认仅观测')
    return parser


def launch_command(args):
    """将单程序参数转换为独立 launch 参数，不让调用者手工拼多节点命令。"""
    def flag(value):
        return 'true' if value else 'false'

    for name in ('camera_width', 'camera_height', 't_confirm_frames',
                 'zero_confirm_messages'):
        if getattr(args, name) <= 0:
            raise ValueError('{} 必须为正整数'.format(name))
    for name in ('line_dark_threshold', 't_dark_threshold'):
        if not 0 <= getattr(args, name) <= 255:
            raise ValueError('{} 必须在 0 至 255'.format(name))
    if args.t_lane_min_width_ratio > args.t_lane_max_width_ratio:
        raise ValueError('t-lane-min-width-ratio 不能大于 t-lane-max-width-ratio')

    return [
        'ros2', 'launch', 'rk_bringup', 'stair_line_to_t.launch.py',
        'start_camera:={}'.format(flag(args.start_camera)),
        'camera_device:={}'.format(args.device),
        'camera_width:={}'.format(args.camera_width),
        'camera_height:={}'.format(args.camera_height),
        'camera_fps:={:.3f}'.format(args.camera_fps),
        'sdk_interface:={}'.format(args.interface),
        'line_speed_mps:={:.3f}'.format(args.line_speed_mps),
        'line_dark_threshold:={}'.format(args.line_dark_threshold),
        'line_kp_lateral:={:.3f}'.format(args.line_kp_lateral),
        'line_kp_heading:={:.3f}'.format(args.line_kp_heading),
        'line_max_wz_radps:={:.3f}'.format(args.line_max_wz_radps),
        't_confirm_frames:={}'.format(args.t_confirm_frames),
        't_dark_threshold:={}'.format(args.t_dark_threshold),
        't_band_min_width_ratio:={:.3f}'.format(args.t_band_min_width_ratio),
        't_lane_min_width_ratio:={:.3f}'.format(args.t_lane_min_width_ratio),
        't_lane_max_width_ratio:={:.3f}'.format(args.t_lane_max_width_ratio),
        'zero_confirm_messages:={}'.format(args.zero_confirm_messages),
        'zero_confirm_timeout_sec:={:.3f}'.format(args.zero_confirm_timeout_sec),
        'stairs_up_speed_mps:={:.3f}'.format(args.stairs_up_speed_mps),
        'stairs_up_duration_sec:={:.3f}'.format(args.stairs_up_duration_sec),
        'stairs_turn_angle_deg:={:.3f}'.format(args.stairs_turn_angle_deg),
        'stairs_turn_wz_radps:={:.3f}'.format(args.stairs_turn_wz_radps),
        'stairs_down_speed_mps:={:.3f}'.format(args.stairs_down_speed_mps),
        'stairs_down_duration_sec:={:.3f}'.format(args.stairs_down_duration_sec),
        'inter_stage_stop_sec:={:.3f}'.format(args.inter_stage_stop_sec),
        'execute_line_motion:={}'.format(flag(args.execute)),
        'execute_phase3:={}'.format(flag(args.execute)),
    ]


def main(argv=None):
    """运行完整流程；环境未 source ROS 时明确失败，不伪造本地控制能力。"""
    args = build_parser().parse_args(argv)
    if not os.environ.get('AMENT_PREFIX_PATH'):
        print('错误：请先 source /opt/ros/foxy/setup.bash 和 install/setup.bash',
              file=sys.stderr)
        return 2
    command = launch_command(args)
    mode = '真机执行' if args.execute else '纯观测验证（不会运动）'
    print('楼梯完整流程模式：{}'.format(mode), flush=True)
    print('启动命令：{}'.format(' '.join(command)), flush=True)
    try:
        return subprocess.call(command)
    except KeyboardInterrupt:
        # Ctrl+C 会同时交给 ros2 launch 与本父进程；launch 负责向所有子节点
        # 广播关闭和停车，本入口仅将 Python traceback 转为明确的人工停止提示。
        print('\n已收到 Ctrl+C：独立楼梯流程正在停止，未授权任何真机运动。',
              flush=True)
        return 130
    except FileNotFoundError:
        print('错误：未找到 ros2；请确认 ROS Foxy 环境已 source', file=sys.stderr)
        return 2


if __name__ == '__main__':
    sys.exit(main())
