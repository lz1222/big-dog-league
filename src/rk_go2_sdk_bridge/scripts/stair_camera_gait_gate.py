#!/usr/bin/env python3
"""USB 楼梯入口相机门禁：确认入口标记后才调用楼梯专用经典步态工具。

本程序不直接发布 ``cmd_vel``，也不调用已被固件移除的 ``SwitchGait(3)``。
只有视觉门禁通过并添加 ``--execute`` 时，才启动
``go2_sdk_stair_classic_action``。该独立工具不切换步态；它在当前经典步态下
复现参考程序 Phase 3 的上台阶、左转和下台阶速度流程，并在结束时停车。
"""

import argparse
import json
from pathlib import Path
import subprocess
import sys
import time

import cv2
import numpy as np


# 楼梯任务与已验证的 Sonix USB 相机绑定；不扫描 /dev/video*，避免误用机械臂相机。
DEFAULT_STAIR_CAMERA_DEVICE = '/dev/video0'
# Go2 当前控制网卡地址为 192.168.123.18；eth0 无链路时绝不能作为动作默认值。
DEFAULT_STAIR_NETWORK_INTERFACE = 'eth1'

# ===== 楼梯运动参数总表：修改速度、时间、转弯或段数只改这里 =====
# T 型标记尚未出现时的接近动作：每段后停车并重新识别，而不是连续盲走。
DEFAULT_APPROACH_SPEED_MPS = 0.25       # 接近 T 的前进速度（m/s）
DEFAULT_APPROACH_SEGMENT_SEC = 1.6      # 单次接近前进时间（s）；本次改为 1.6
DEFAULT_APPROACH_MAX_SEGMENTS = 3       # 最多接近段数；总前进时间上限 = 4.8 秒
# T 型标记确认后的参考程序 Phase 3 三段动作参数。
DEFAULT_STAIRS_UP_SPEED_MPS = 0.55      # 上台阶前进速度（m/s）
DEFAULT_STAIRS_UP_DURATION_SEC = 3.9    # 上台阶前进时间（s）
DEFAULT_TURN_LEFT_ANGLE_DEG = 79.0      # 左转目标角度（度），按实时 yaw 判断完成
DEFAULT_TURN_LEFT_WZ_RADPS = 1.0        # 左转角速度（rad/s），正值为左转
DEFAULT_STAIRS_DOWN_SPEED_MPS = 0.55    # 下台阶前进速度（m/s）
DEFAULT_STAIRS_DOWN_DURATION_SEC = 2.9  # 下台阶前进时间（s）
INTER_STAGE_STOP_SEC = 0.4              # 三段动作之间停车稳定时间（s），由 C++ 再次执行


def positive_int(raw):
    """解析正整数参数，拒绝零和负数以保持采样、门限含义明确。"""
    value = int(raw)
    if value <= 0:
        raise argparse.ArgumentTypeError('必须是正整数')
    return value


def nonnegative_float(raw):
    """解析有限非负浮点数，用于几何阈值，禁止 NaN 进入安全判断。"""
    value = float(raw)
    if not np.isfinite(value) or value < 0.0:
        raise argparse.ArgumentTypeError('必须是有限非负数')
    return value


def finite_float(raw):
    """解析有限浮点数，用于允许左右转向的角速度，拒绝 NaN 和无穷大。"""
    value = float(raw)
    if not np.isfinite(value):
        raise argparse.ArgumentTypeError('必须是有限数')
    return value


def positive_float(raw):
    """解析有限正浮点数，保证经典步态动作有明确且非零的持续时间。"""
    value = float(raw)
    if not np.isfinite(value) or value <= 0.0:
        raise argparse.ArgumentTypeError('必须是有限正数')
    return value


def normalize_device(raw):
    """只接受明确 V4L2 设备，禁止为寻找可用相机而切换到其它传感器。"""
    value = str(raw).strip()
    if value.isdigit():
        return '/dev/video{}'.format(int(value))
    if value.startswith('/dev/'):
        return value
    raise argparse.ArgumentTypeError('设备必须是 /dev/videoN 或数字索引')


def horizontal_step_edges(frame, canny_low, canny_high, min_width_ratio):
    """提取近水平且足够长的边缘，作为楼梯踏步的保守候选。

    只返回图像下半部的线段中点 y 坐标。地面远处的单条横线不应触发：调用方
    还要求多条彼此分离的候选边缘，且所有帧均需达标。
    """
    if frame is None or frame.ndim != 3 or frame.shape[2] != 3:
        return []
    height, width = frame.shape[:2]
    roi_top = height // 3
    gray = cv2.cvtColor(frame[roi_top:, :], cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blurred, canny_low, canny_high)
    segments = cv2.HoughLinesP(
        edges,
        rho=1,
        theta=np.pi / 180.0,
        threshold=max(20, width // 12),
        minLineLength=max(24, int(width * min_width_ratio)),
        maxLineGap=max(8, width // 32),
    )
    if segments is None:
        return []

    candidates = []
    minimum_length = width * min_width_ratio
    for x1, y1, x2, y2 in segments.reshape(-1, 4):
        delta_x = float(x2 - x1)
        delta_y = float(y2 - y1)
        length = float(np.hypot(delta_x, delta_y))
        if length < minimum_length:
            continue
        # 允许轻微透视倾斜，但排除黑线转弯处的竖直边。
        if abs(delta_y) > max(3.0, 0.12 * abs(delta_x)):
            continue
        candidates.append(roi_top + 0.5 * (float(y1) + float(y2)))
    return candidates


def distinct_edge_count(candidates, minimum_separation_px):
    """合并同一踏步被重复检测到的线段，避免 Hough 重复计数绕过门禁。"""
    levels = []
    for level in sorted(candidates):
        if not levels or level - levels[-1] >= minimum_separation_px:
            levels.append(level)
    return len(levels), levels


def stairs_entry_marker(frame, dark_threshold, band_min_width_ratio,
                        lane_min_width_ratio, lane_max_width_ratio):
    """识别赛道楼梯入口的 T 形黑色标记，而不是把普通楼梯边缘当作入口。

    目标画面由上半部分的宽黑色横带（台阶立面/挡板）和下半部分居中的窄黑色
    引导带组成。两项同时成立才返回 true；单独的地面黑线、阴影或横向物体均
    不能放行。该算法只作保守起点门禁，不用于估计台阶高度或控制行走速度。
    """
    if frame is None or frame.ndim != 3 or frame.shape[2] != 3:
        return False, {'reason': 'invalid_frame'}

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    height, width = gray.shape[:2]
    dark = gray <= dark_threshold

    # 入口横带应处于图像上半部；取最大行覆盖率来容忍两端被画面裁切。
    band_start = max(0, int(height * 0.08))
    band_end = max(band_start + 1, int(height * 0.55))
    row_coverage = dark[band_start:band_end, :].mean(axis=1)
    widest_band_ratio = float(row_coverage.max()) if row_coverage.size else 0.0

    # 中央引导带必须延伸进下半画面，宽度远小于横带且中心接近相机中心。
    lane_start = max(0, int(height * 0.55))
    lane = dark[lane_start:, :]
    column_coverage = lane.mean(axis=0) if lane.size else np.zeros(width)
    lane_columns = np.flatnonzero(column_coverage >= 0.55)
    lane_width_ratio = float(len(lane_columns)) / float(max(1, width))
    lane_center_ratio = (
        float(lane_columns.mean()) / float(max(1, width - 1))
        if len(lane_columns) else None
    )
    lane_is_centered = (
        lane_center_ratio is not None and 0.35 <= lane_center_ratio <= 0.65
    )
    band_found = widest_band_ratio >= band_min_width_ratio
    lane_found = (
        lane_min_width_ratio <= lane_width_ratio <= lane_max_width_ratio
        and lane_is_centered
    )
    return band_found and lane_found, {
        'band_width_ratio': round(widest_band_ratio, 3),
        'lane_width_ratio': round(lane_width_ratio, 3),
        'lane_center_ratio': (
            None if lane_center_ratio is None else round(lane_center_ratio, 3)
        ),
        'band_found': band_found,
        'lane_found': lane_found,
    }


def inspect_frame(frame, config):
    """把一帧转换为统一审计记录，保证设备输入与 ROS 图像输入使用同一门禁算法。"""
    if frame is None:
        return {'frame_ok': False, 'edge_count': 0}
    sample = {
        'frame_ok': True,
        'width': int(frame.shape[1]),
        'height': int(frame.shape[0]),
    }
    if config.trigger_profile == 't_marker':
        marker_ready, marker = stairs_entry_marker(
            frame,
            config.dark_threshold,
            config.band_min_width_ratio,
            config.lane_min_width_ratio,
            config.lane_max_width_ratio,
        )
        sample['marker_ready'] = marker_ready
        sample['marker'] = marker
    else:
        candidates = horizontal_step_edges(
            frame,
            config.canny_low,
            config.canny_high,
            config.min_width_ratio,
        )
        edge_count, levels = distinct_edge_count(
            candidates, config.min_separation_px
        )
        sample['edge_count'] = edge_count
        sample['levels_y_px'] = [round(value, 1) for value in levels]
    return sample


def samples_ready(samples, trigger_profile, minimum_edges):
    """所有采样帧必须通过，避免一张旧画面或短暂误检触发速度覆盖。"""
    if trigger_profile == 't_marker':
        return bool(samples) and all(
            sample['frame_ok'] and sample.get('marker_ready', False)
            for sample in samples
        )
    return bool(samples) and all(
        sample['frame_ok'] and sample.get('edge_count', 0) >= minimum_edges
        for sample in samples
    )


def inspect_camera(config):
    """连续采样指定相机；任何读帧异常均返回未就绪而不是沿用旧画面。"""
    capture = cv2.VideoCapture(config.device, cv2.CAP_V4L2)
    if not capture.isOpened():
        raise RuntimeError('无法打开指定相机 {}'.format(config.device))
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, float(config.width))
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, float(config.height))
    capture.set(cv2.CAP_PROP_BUFFERSIZE, 1.0)

    samples = []
    try:
        for _unused_index in range(config.frames):
            ok, frame = capture.read()
            if not ok or frame is None:
                samples.append({'frame_ok': False, 'edge_count': 0})
                continue
            samples.append(inspect_frame(frame, config))
            if config.frame_interval_sec:
                time.sleep(config.frame_interval_sec)
    finally:
        capture.release()

    return samples_ready(samples, config.trigger_profile, config.minimum_edges), samples


def inspect_image_topic(config):
    """订阅巡线相机已发布的画面，避免与 line_camera_node 争抢同一 V4L2 设备。

    ROS 依赖延迟导入，使 ``--device`` 模式仍可在未 source ROS 环境时做相机诊断。
    订阅端只读图像并复用同一帧门禁；它不发布巡线话题，也不干预巡线节点。
    """
    try:
        import rclpy
        from cv_bridge import CvBridge
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image
    except ImportError as error:
        raise RuntimeError(
            '使用 --image-topic 前必须 source ROS 2 工作区: {}'.format(error)
        ) from error

    class TopicFrameCollector(Node):
        """只收集固定数量的新帧；不缓存历史图像，确保 T 标记判断是实时画面。"""

        def __init__(self):
            super().__init__('stair_camera_gait_gate')
            self.samples = []
            self.bridge = CvBridge()
            self.subscription = self.create_subscription(
                Image, config.image_topic, self._on_image, qos_profile_sensor_data
            )

        def _on_image(self, message):
            if len(self.samples) >= config.frames:
                return
            try:
                frame = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
                self.samples.append(inspect_frame(frame, config))
            except Exception as error:  # 图像编码异常只能拒绝动作，不能沿用前帧。
                self.samples.append({
                    'frame_ok': False,
                    'edge_count': 0,
                    'error': '{}: {}'.format(type(error).__name__, error),
                })

    rclpy.init(args=None)
    node = TopicFrameCollector()
    deadline = time.monotonic() + config.topic_timeout_sec
    try:
        while len(node.samples) < config.frames and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.20)
        return samples_ready(
            node.samples, config.trigger_profile, config.minimum_edges
        ), node.samples
    finally:
        node.destroy_node()
        rclpy.shutdown()


def default_stair_classic_helper():
    """优先使用安装后的工具；源码运行时回退到当前工作区的构建产物。"""
    installed = Path(__file__).resolve().parent / 'go2_sdk_stair_classic_action'
    if installed.is_file():
        return installed
    workspace = Path(__file__).resolve().parents[3]
    return workspace / 'build' / 'rk_go2_sdk_bridge' / 'go2_sdk_stair_classic_action'


def execute_stair_classic_motion(helper, interface, config):
    """调用 T 型标记后的完整 Phase 3 三段动作，不混入迷宫路线或旧步态 API。"""
    # ===== T 型标记确认后实际发送的 Phase 3 运动参数 =====
    # 上台阶前进 → 按 yaw 左转 → 下台阶前进；速度和时间来自下方 --stairs-* 参数。
    command = [
        str(helper), interface,
        '--stairs-up-speed-mps', '{:.3f}'.format(config.stairs_up_speed_mps),
        '--stairs-up-duration-sec', '{:.3f}'.format(config.stairs_up_duration_sec),
        '--stairs-turn-angle-deg', '{:.3f}'.format(config.stairs_turn_angle_deg),
        '--stairs-turn-wz-radps', '{:.3f}'.format(config.stairs_turn_wz_radps),
        '--stairs-down-speed-mps', '{:.3f}'.format(config.stairs_down_speed_mps),
        '--stairs-down-duration-sec', '{:.3f}'.format(config.stairs_down_duration_sec),
        '--execute',
    ]
    completed = subprocess.run(
        command,
        check=False,
        text=True,
        capture_output=True,
    )
    return {
        'command': command,
        'returncode': completed.returncode,
        'stdout': completed.stdout.strip(),
        'stderr': completed.stderr.strip(),
    }


def execute_approach_motion(helper, interface, speed_mps, duration_sec):
    """在未识别到 T 时只前进一个短段；短段结束后必须重新取图，而非持续盲走。"""
    command = [
        str(helper), interface,
        '--approach-only',
        '--approach-speed-mps', '{:.3f}'.format(speed_mps),
        '--approach-duration-sec', '{:.3f}'.format(duration_sec),
        '--execute',
    ]
    completed = subprocess.run(
        command, check=False, text=True, capture_output=True,
    )
    return {
        'command': command,
        'returncode': completed.returncode,
        'stdout': completed.stdout.strip(),
        'stderr': completed.stderr.strip(),
    }


def parse_arguments():
    """提供默认相机 dry-run；实机经典步态必须显式给出速度参数和 --execute。"""
    parser = argparse.ArgumentParser(description=__doc__)
    input_source = parser.add_mutually_exclusive_group()
    input_source.add_argument(
        '--device', type=normalize_device,
        help='独占 USB 相机的 V4L2 设备；默认 /dev/video0；仅适用于巡线相机节点未运行时',
    )
    input_source.add_argument(
        '--image-topic',
        help='订阅已有 ROS 图像流；巡线时使用 /line_camera/image_raw，避免抢占相机',
    )
    parser.add_argument(
        '--interface', default=DEFAULT_STAIR_NETWORK_INTERFACE,
        help='Go2 SDK 控制网卡，默认 eth1（192.168.123.18）',
    )
    parser.add_argument('--frames', type=positive_int, default=5)
    parser.add_argument('--frame-interval-sec', type=nonnegative_float,
                        default=0.10)
    parser.add_argument('--topic-timeout-sec', type=positive_float, default=5.0,
                        help='等待 --image-topic 收齐采样帧的总超时秒数')
    parser.add_argument('--width', type=positive_int, default=640)
    parser.add_argument('--height', type=positive_int, default=480)
    parser.add_argument('--minimum-edges', type=positive_int, default=3)
    parser.add_argument('--min-separation-px', type=positive_int, default=18)
    parser.add_argument('--min-width-ratio', type=nonnegative_float,
                        default=0.30)
    parser.add_argument('--canny-low', type=positive_int, default=50)
    parser.add_argument('--canny-high', type=positive_int, default=150)
    parser.add_argument(
        '--trigger-profile', choices=('step_edges', 't_marker'),
        default='step_edges',
        help='step_edges 检测多级踏步；t_marker 检测白底黑色 T 形楼梯入口',
    )
    parser.add_argument('--dark-threshold', type=positive_int, default=70,
                        help='t_marker 的黑色灰度上限，默认适配黑色胶带')
    parser.add_argument('--band-min-width-ratio', type=nonnegative_float,
                        default=0.65,
                        help='t_marker 上方横黑带最小画面宽度比例')
    parser.add_argument('--lane-min-width-ratio', type=nonnegative_float,
                        default=0.04,
                        help='t_marker 中央引导带最小画面宽度比例')
    parser.add_argument('--lane-max-width-ratio', type=nonnegative_float,
                        default=0.28,
                        help='t_marker 中央引导带最大画面宽度比例')
    # ===== T 型标记确认后才会生效的旧 Phase 3 三段动作参数 =====
    # 默认复现参考程序：上 0.55 m/s×3.9 s，左转 79°@1.0 rad/s，
    # 下 0.55 m/s×2.9 s。未满足 T 型标记门禁时不会启动运动工具。
    parser.add_argument(
        '--execute', action='store_true',
        help='门禁通过后运行完整 Phase 3 三段动作；始终保持当前经典步态',
    )
    # 接收 T 型标记确认后速度命令的独立工具；不引用巡线或迷宫动作程序。
    parser.add_argument(
        '--stair-classic-helper', type=Path,
        default=default_stair_classic_helper(),
        help='楼梯专用经典步态工具路径（默认安装树或当前工作区构建产物）',
    )
    # T 型标记后的第 1 段：上台阶前进速度与持续时间。
    parser.add_argument(
        '--stairs-up-speed-mps', '--stair-vx-mps', dest='stairs_up_speed_mps',
        type=nonnegative_float, default=DEFAULT_STAIRS_UP_SPEED_MPS,
        help='T 后上台阶前进速度 m/s；默认 0.55（参考 Phase 3）',
    )
    parser.add_argument(
        '--stairs-up-duration-sec', '--stair-duration-sec',
        dest='stairs_up_duration_sec', type=positive_float,
        default=DEFAULT_STAIRS_UP_DURATION_SEC,
        help='T 后上台阶前进时间 s；默认 3.9（参考 Phase 3）',
    )
    # T 型标记后的第 2 段：按实时 yaw 左转，不能以固定时间替代角度判定。
    parser.add_argument(
        '--stairs-turn-angle-deg', type=positive_float,
        default=DEFAULT_TURN_LEFT_ANGLE_DEG,
        help='T 后左转目标角度（度）；默认 79（参考 Phase 3）',
    )
    parser.add_argument(
        '--stairs-turn-wz-radps', '--stair-wz-radps',
        dest='stairs_turn_wz_radps', type=positive_float,
        default=DEFAULT_TURN_LEFT_WZ_RADPS,
        help='T 后左转角速度 rad/s；默认 1.0（参考 Phase 3）',
    )
    # T 型标记后的第 3 段：下台阶前进速度与持续时间。
    parser.add_argument(
        '--stairs-down-speed-mps', type=nonnegative_float,
        default=DEFAULT_STAIRS_DOWN_SPEED_MPS,
        help='T 后下台阶前进速度 m/s；默认 0.55（参考 Phase 3）',
    )
    parser.add_argument(
        '--stairs-down-duration-sec', type=positive_float,
        default=DEFAULT_STAIRS_DOWN_DURATION_SEC,
        help='T 后下台阶前进时间 s；默认 2.9（参考 Phase 3）',
    )
    # ===== 未识别 T 型标记时的接近速度 =====
    # 每个短段完成都会停车并重新采样，max-segments 限制总行走距离，防止持续盲走。
    parser.add_argument(
        '--approach-speed-mps', type=positive_float,
        default=DEFAULT_APPROACH_SPEED_MPS,
        help='识别到 T 前的前进速度 m/s；默认 0.25',
    )
    parser.add_argument(
        '--approach-segment-sec', type=positive_float,
        default=DEFAULT_APPROACH_SEGMENT_SEC,
        help='每次接近短段的持续时间 s；默认 1.6，结束即停车并重新识别',
    )
    parser.add_argument(
        '--approach-max-segments', type=positive_int,
        default=DEFAULT_APPROACH_MAX_SEGMENTS,
        help='未识别 T 时最多接近短段数；默认 3，即累计前进最多 4.8 秒',
    )
    config = parser.parse_args()
    # 未指定 ROS 图像话题时固定使用楼梯相机 /dev/video0，不探测或回退其它设备。
    if config.device is None and config.image_topic is None:
        config.device = DEFAULT_STAIR_CAMERA_DEVICE
    return config


def main():
    """输出结构化判定，供人工和上层任务状态机同时审计。"""
    config = parse_arguments()
    if config.canny_low >= config.canny_high:
        raise ValueError('--canny-low 必须小于 --canny-high')
    if not 0.05 <= config.min_width_ratio <= 1.0:
        raise ValueError('--min-width-ratio 必须在 [0.05, 1.0] 内')
    if not 0 <= config.dark_threshold <= 255:
        raise ValueError('--dark-threshold 必须在 [0, 255] 内')
    if not 0.05 <= config.band_min_width_ratio <= 1.0:
        raise ValueError('--band-min-width-ratio 必须在 [0.05, 1.0] 内')
    if not 0 < config.lane_min_width_ratio <= config.lane_max_width_ratio:
        raise ValueError('中央引导带宽度比例范围无效')

    def inspect_selected_input():
        """每次接近短段后重新读取新画面，禁止用运动前的旧帧确认 T 型标记。"""
        if config.image_topic:
            ready, samples = inspect_image_topic(config)
            return ready, samples, {'image_topic': config.image_topic}
        ready, samples = inspect_camera(config)
        return ready, samples, {'device': config.device}

    ready, samples, input_source = inspect_selected_input()
    approach_results = []
    if config.execute and not config.stair_classic_helper.is_file():
        raise RuntimeError(
            '楼梯专用经典步态工具不存在: {}'.format(
                config.stair_classic_helper
            )
        )
    # ===== T 型标记前：0.25 m/s 接近 → 停车 → 新画面复检 =====
    # 初始画面已识别到 T 时跳过该阶段，直接进入下方完整 Phase 3。
    while config.execute and not ready and (
            len(approach_results) < config.approach_max_segments):
        approach_result = execute_approach_motion(
            config.stair_classic_helper,
            config.interface,
            config.approach_speed_mps,
            config.approach_segment_sec,
        )
        approach_results.append(approach_result)
        if approach_result['returncode'] != 0:
            break
        ready, samples, input_source = inspect_selected_input()
    report = {
        **input_source,
        'interface': config.interface,
        'frames': samples,
        'trigger_profile': config.trigger_profile,
        'minimum_edges': config.minimum_edges,
        'stairs_ready': ready,
        'executed': False,
    }
    if config.execute:
        report['approach_before_t_marker'] = {
            'vx_mps': config.approach_speed_mps,
            'segment_duration_sec': config.approach_segment_sec,
            'maximum_segments': config.approach_max_segments,
            'completed_segments': len(approach_results),
            'results': approach_results,
        }
    # ===== T 型标记已确认：从这里开始执行完整旧 Phase 3 三段动作 =====
    # ready 由所有采样帧 marker_ready=true 得到；未确认时会进入下方 blocked 分支。
    if config.execute and ready:
        # T 型标记后的完整旧 Phase 3 参数在此汇总并写入输出，便于逐段调参。
        # C++ 工具用 DDS 实时 yaw 判断左转完成；这里仍不切换步态。
        report['stair_classic_motion'] = {
            'stairs_up': {
                'vx_mps': config.stairs_up_speed_mps,
                'duration_sec': config.stairs_up_duration_sec,
            },
            'turn_left': {
                'angle_deg': config.stairs_turn_angle_deg,
                'wz_radps': config.stairs_turn_wz_radps,
            },
            'stairs_down': {
                'vx_mps': config.stairs_down_speed_mps,
                'duration_sec': config.stairs_down_duration_sec,
            },
            'command_rate_hz': 20,
            'gait_switch': False,
        }
        report['stair_classic_result'] = execute_stair_classic_motion(
            config.stair_classic_helper, config.interface, config,
        )
        report['executed'] = report['stair_classic_result']['returncode'] == 0
    elif config.execute:
        report['blocked_reason'] = (
            'approach_motion_failed'
            if approach_results and approach_results[-1]['returncode'] != 0
            else 't_marker_not_confirmed_before_approach_limit'
        )

    print(json.dumps(report, ensure_ascii=False, sort_keys=True))
    return 0 if (ready and (not config.execute or report['executed'])) else 1


if __name__ == '__main__':
    try:
        sys.exit(main())
    except Exception as error:
        print(json.dumps({
            'stairs_ready': False,
            'executed': False,
            'error': '{}: {}'.format(type(error).__name__, error),
        }, ensure_ascii=False), file=sys.stderr)
        sys.exit(2)
