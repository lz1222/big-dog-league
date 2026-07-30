#!/usr/bin/env python3
"""
Go2 多阶段整合任务程序
======================
Phase 1: 黑线循迹 + 白线检测 (来自 track_and_control.py)
Phase 2: 迷宫定序运动 (来自 迷宫.py)
Phase 3: 巡线 + 黑区FreeWalk + 二次居中 (来自 step.py)
Phase 4: 巡线 + 红圆检测 + 终段巡线 (来自 1.py)
Phase 5: 站立 + arm left + 白线检测单次跳跃 + 直走3s + 左转85° + 右平移2s

用法:
  python3 integrated_mission.py <networkInterface>
  python3 integrated_mission.py eth0
"""

import sys
import os
import time
import math
import threading
import signal
import subprocess
import re

import json
from dataclasses import dataclass

import cv2
import numpy as np
import pyrealsense2 as rs
import cyclonedds.idl as idl
import cyclonedds.idl.annotations as annotate
import cyclonedds.idl.types as types

from unitree_sdk2py.core.channel import (
    ChannelFactoryInitialize,
    ChannelPublisher,
    ChannelSubscriber,
)
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
from unitree_sdk2py.go2.sport.sport_client import SportClient
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import (
    MotionSwitcherClient,
)
from unitree_sdk2py.a2.audio.audio_client import AudioClient
from unitree_sdk2py.go2.vui.vui_client import VuiClient

# ============================================================
# 机械臂控制 (来自 Arm.py)
# ============================================================

@dataclass
@annotate.final
@annotate.autoid("sequential")
class ArmString_(idl.IdlStruct, typename="unitree_arm.msg.dds_.ArmString_"):
    data_: str


ARM_ACTIONS = {
    "夹住走": [-6.9, -40.9, 89.7, -0.3, 3.8, 0.3, 25],
    "放左抬起1": [-81.7, -18.5, 19, 1.8, -32, 0, 25],
    "放左": [-81.7, 33.2, 55.7, 1.6, -81.7, 0, 25],
    "放左松开": [-81.7, 33.2, 55.7, 1.6, -81.7, 0, 60],
    "放左抬起2": [-81.7, -18.5, 19, 1.8, -32, 0, 60],
    "放右抬起1": [78.8, -18.5, 19, 1.8, -32, 0, 25],
    "放右": [77.1, 21.7, 80.1, -6.4, -90, 0, 25],
    "放右松开": [77.1, 21.7, 80.1, -6.4, -90, 0, 60],
    "放右抬起2": [77.1, -18.5, 80.1, -6.4, -90, 0, 60],
    "抬起2": [56.7, 10.6, -3.8, -1.3, -37.5, 0.8, 25],
    "放2": [56.7, 55, 1.7, -0.6, -44.6, 0.8, 25],
    "松手2": [56.7, 55, 1.7, -0.6, -44.6, 0.8, 60],
    "小抬2": [56.7, 37, 1.7, -0.6, -44.6, 0.8, 60],
    "小转2": [42.5, 37, 1.7, -0.6, -44.6, 0.8, 60],
    "小放2": [42.5, 64.4, -3.7, 1.2, -52.7, 0.4, 60],
    "小抓2": [42.5, 64.4, -3.7, 1.2, -52.7, 0.4, 25],
    "小抬抓2": [42.5, 22, -3.7, 1.2, -52.7, 0.4, 25],
    "抬1": [-10.6, 39.9, -19, -0.3, -21.5, 0.2, 60],
    "伸1": [-10.6, 58.2, -14, -0.3, -33.1, 0.6, 60],
    "抓1": [-10.6, 58.2, -14, -0.3, -33.1, 0.6, 25],
    "放扫1": [-20, 20.9, -11.0, -0.3, -33.5, 0.2, 25],
    "放扫2": [-20, 30.9, -20.0, -0.3, -21.5, 0.2, 25],
    "扫1": [-42, 82.0, -55.0, -3.3, -28.4, 0.4, 25],
    "扫2": [28, 82.0, -55.0, -3.3, -28.4, 0.4, 25],
}


# ============================================================
# 网络接口自动检测
# ============================================================
def auto_detect_interface():
    """自动检测连接 Go2 机器人的网络接口"""
    go2_subnet = "192.168.123."
    candidates = []
    try:
        result = subprocess.run(
            ["ip", "-o", "addr", "show"],
            capture_output=True, text=True, timeout=5,
        )
        for line in result.stdout.splitlines():
            parts = line.split()
            if len(parts) < 4:
                continue
            iface = parts[1]
            if iface == "lo" or iface.startswith("docker") or iface == "l4tbr0":
                continue
            inet_match = re.search(r'inet\s+([\d.]+)', line)
            if inet_match:
                ip = inet_match.group(1)
                if ip.startswith(go2_subnet):
                    print(f"[AUTO] Go2 网络接口: {iface} ({ip})")
                    return iface
                candidates.append((iface, ip))
    except Exception:
        pass
    try:
        for iface in sorted(os.listdir("/sys/class/net")):
            if iface == "lo" or iface.startswith("docker"):
                continue
            operstate_file = f"/sys/class/net/{iface}/operstate"
            carrier_file = f"/sys/class/net/{iface}/carrier"
            if os.path.exists(operstate_file):
                with open(operstate_file) as f:
                    if f.read().strip() != "up":
                        continue
            if os.path.exists(carrier_file):
                with open(carrier_file) as f:
                    if f.read().strip() != "1":
                        continue
            addr_path = f"/sys/class/net/{iface}/address"
            if os.path.exists(addr_path):
                print(f"[AUTO] 活动接口: {iface}")
                return iface
    except Exception:
        pass
    if candidates:
        iface, ip = candidates[0]
        print(f"[AUTO] 备选接口: {iface} ({ip})")
        return iface
    return None


# ============================================================
# DDS Topic
# ============================================================
TOPIC_HIGHSTATE = "rt/sportmodestate"


# ============================================================
# 🔧 可调参数汇总 — 所有需要微调的参数都在这里 🔧
# ============================================================

# ──────────────────────────────────────────────────────────────
# 📷 摄像头参数
# ──────────────────────────────────────────────────────────────
FRAME_WIDTH = 640          # 图像宽度
FRAME_HEIGHT = 480         # 图像高度
FPS = 30                   # 帧率

# ──────────────────────────────────────────────────────────────
# ⚙️ 控制参数
# ──────────────────────────────────────────────────────────────
CTRL_HZ = 100              # 控制循环频率
CTRL_DT = 1.0 / CTRL_HZ    # 控制周期 (自动计算)

# ================================================================
#  Phase 1 — 黑线循迹 + 白线检测 + 两次前跳
# ================================================================

# --- 黑线检测 ---
P1_THRESHOLD = 80           # 黑线二值化阈值 (越大检测越严格)
P1_MIN_AREA = 200.0         # 最小轮廓面积

# --- PID 巡线控制 ---
P1_PID_KP = 0.80            # 比例系数
P1_PID_KI = 0.04            # 积分系数
P1_PID_KD = 0.08            # 微分系数
P1_PID_MAX_YAW = 2.0        # PID 输出最大偏航角速度
P1_PID_INTEGRAL_MAX = 1.5   # 积分限幅

# --- 巡线速度 ---
P1_BASE_SPEED = 0.22        # 基础前进速度 (m/s)

# --- 急转弯处理 ---
P1_SHARP_TURN_THRESHOLD = 30 # 连续大偏移帧数触发急转弯
P1_FORWARD_DIST = 0.25      # 急转弯前冲距离 (m)
P1_ROTATION_ANGLE = math.radians(80)  # 急转弯旋转角度 (rad)
P1_COOLDOWN_FRAMES = 15     # 急转弯冷却帧数

# --- 白线检测 ---
P1_WHITE_BRIGHTNESS = 140   # 白色亮度阈值
P1_WHITE_GAP_MIN = 4        # 白线最小行高 (像素)
P1_WHITE_CROSS_CONFIRM = 3  # 白线连续确认帧数
P1_WHITE_BGR_THRESH = 155   # BGR 三通道白色阈值
P1_WHITE_LINE_RATIO = 0.30  # 白线行像素占比阈值

# --- 白线后居中校准 ---
P1_CENTER_THRESHOLD = 0.12  # 居中判定偏移阈值
P1_CENTER_CONFIRM = 8       # 居中连续确认帧数
P1_ALIGN_MAX_YAW = 1.0      # 校准最大偏航角速度
P1_ALIGN_TIMEOUT = 3.0      # 校准超时 (秒)

# --- 前跳 ---
P1_JUMP_FORWARD_DUR = 0.5   # 跳跃前冲时长 (秒)
P1_JUMP_FORWARD_SPEED = 0.3 # 跳跃前冲速度 (m/s)
P1_JUMP_STOP_DUR = 0.5      # 跳跃前停顿 (秒)

# --- 第二次白线后向前走 ---
P1_SECOND_WHITE_FORWARD_TIME = 1.5  # 第二次检测到白线后向前走的时长 (秒)
P1_SECOND_WHITE_FORWARD_SPEED = 0.25  # 第二次检测到白线后向前走的速度 (m/s)

# ================================================================
#  Phase 2 — 迷宫定序运动 (盲走)
# ================================================================

P2_FORWARD_SPEED = 0.25     # 前进速度 (m/s)
P2_TURN_SPEED = 1.0         # 转向速度 (rad/s)
P2_TURN_TIMEOUT = 10.0      # 转向超时 (秒)
P2_STRAFE_SPEED = 0.08      # 横移速度 (m/s)，正数=左移，负数=右移
P2_ARC_FORWARD_SPEED = 0.04 # 圆弧转弯前进速度 (m/s)，小一点，避免弯内冲过头
P2_ARC_LATERAL_SPEED = 0.06 # 圆弧转弯横移速度 (m/s)，左转默认左移，右转默认右移
P2_ARC_TURN_SPEED = 0.70    # 圆弧转弯角速度 (rad/s)，80°约2秒完成

# 动作序列:
#   ("forward", 秒)
#   ("strafe_left", 秒) / ("strafe_right", 秒)
#   ("turn_left", 度) / ("turn_right", 度)                 # 老的原地转，保留兼容
#   ("arc_left", (角度, vx, vy, |vyaw|))                  # 推荐：带横移的左弧转
#   ("arc_right", (角度, vx, vy, |vyaw|))                 # 推荐：带横移的右弧转
# 说明：
#   vy 是机器人自身坐标系横移速度，正数=左移，负数=右移。
#   80cm 窄段内不建议原地 90° 转，Go2 机身 70cm 转动扫掠空间太大。
P2_ACTION_SEQUENCE = [
    ("forward",     7.0),
    ("arc_left",    (80, 0.04,  0.06, 0.70)),
    ("forward",     2.5),
    ("arc_left",    (80, 0.04,  0.06, 0.70)),
    ("forward",     3.6),
    ("arc_right",   (80, 0.04, -0.06, 0.70)),
    ("forward",     2.6),
    ("arc_right",   (87, 0.04, -0.06, 0.70)),
    ("forward",     4.2),
    ("arc_left",    (70, 0.04,  0.06, 0.70)),
    ("forward",     0.4)
]

# ================================================================
#  Phase 3 — 巡线 + 黑区 FreeWalk + 后退
# ================================================================

# --- 黑线检测 ---
P3_BLACK_THRESHOLD = 130    # 黑线二值化阈值
P3_MIN_AREA = 150.0         # 最小轮廓面积

# --- PID 巡线控制 ---
P3_PID_KP = 0.70
P3_PID_KI = 0.04
P3_PID_KD = 0.08
P3_PID_MAX_YAW = 2.0
P3_PID_INTEGRAL_MAX = 1.5

# --- 巡线速度 ---
P3_BASE_SPEED = 0.17       # 基础前进速度 (m/s)

# --- 初始居中 ---
P3_CENTER_THRESHOLD = 0.12      # 居中判定偏移阈值
P3_CENTER_CONFIRM_FRAMES = 6    # 居中连续确认帧数
P3_CENTER_LATERAL_KP = 0.40     # 横向居中 P 系数
P3_CENTER_LATERAL_MAX = 0.25    # 横向速度上限 (m/s)
P3_CENTER2_SEARCH_SPEED = -0.18 # 二次居中丢线时右平移搜索速度 (负=右, m/s)

# --- 黑区检测 (触发 FreeWalk 上台阶) ---
P3_BLACK_RATIO_THRESHOLD = 0.80    # 黑色占比触发阈值
P3_BLACK_CONFIRM_FRAMES = 3        # 黑色连续确认帧数

# --- 台阶动作 ---
P3_STAIRS_UP_DUR = 3.9         # 上台阶前进时长 (秒)
P3_STAIRS_SPEED = 0.55          # 台阶前进速度 (m/s)
P3_TURN_ANGLE_DEG = 79         # 台阶上转向角度 (度)
P3_TURN_SPEED = 1.0            # 台阶上转向速度 (rad/s)
P3_STAIRS_DOWN_DUR = 2.9       # 下台阶前进时长 (秒)

# ================================================================
#  Phase 4 — 巡线 + 红圆检测 + 终段巡线
# ================================================================

# --- 黑线检测 ---
P4_THRESHOLD = 80
P4_MIN_AREA = 200.0

# --- PID 巡线控制 ---
P4_PID_KP = 0.80
P4_PID_KI = 0.04
P4_PID_KD = 0.08
P4_PID_MAX_YAW = 2.0
P4_PID_INTEGRAL_MAX = 1.5

# --- 巡线速度 ---
P4_BASE_SPEED = 0.25

# --- 第一阶段巡线 ---
P4_LINE_TRACK_DURATION = 16.5     # 巡线时长 (秒)
P4_FORWARD_DURATION = 0.5          # 巡线后前进时长 (秒)

# --- 第一次左转 ---
P4_TURN_LEFT_ANGLE = math.radians(88)   # 左转角度 (rad)
P4_TURN_LEFT_SPEED = 1.0                # 左转速度 (rad/s)

# --- 站立 ---
P4_STAND_UP_DURATION = 2.0         # 站立时长 (秒)

# --- 右转 ---
P4_TURN_RIGHT_ANGLE = math.radians(70)  # 右转角度 (rad)
P4_TURN_RIGHT_SPEED = -1.0              # 右转速度 (负=右)

# --- 丢线恢复 ---
P4_LOST_TURN_ANGLE = math.radians(30)   # 丢线后左转角度 (rad)
P4_LOST_TURN_SPEED = 1.0                # 丢线后左转速度 (rad/s)

# --- 红圆检测 (HSV) ---
P4_RED_HSV_LOWER1 = np.array([0, 120, 70])
P4_RED_HSV_UPPER1 = np.array([10, 255, 255])
P4_RED_HSV_LOWER2 = np.array([170, 120, 70])
P4_RED_HSV_UPPER2 = np.array([180, 255, 255])
P4_RED_BLUR_KERNEL = (5, 5)        # 高斯模糊核
P4_RED_MORPH_KERNEL = 5            # 形态学核大小
P4_TARGET_AREA_MIN = 0.02          # 红圆最小面积占比
P4_CIRCULARITY_THRESH = 0.6        # 圆形度阈值

# --- 红圆后计时 + 转向 ---
P4_RED_TIMER_DURATION = 3.9            # 红圆检测后等待 (秒)
P4_RED_TURN_LEFT_ANGLE = math.radians(83)   # 红圆后左转角度 (rad)
P4_RED_TURN_LEFT_SPEED = 1.0               # 红圆后左转速度 (rad/s)
P4_RED_TURN_RIGHT_ANGLE = math.radians(80) # 红圆后右转角度 (rad)
P4_RED_TURN_RIGHT_SPEED = -1.0             # 红圆后右转速度 (rad/s)

# --- 最终巡线 ---
P4_FINAL_TRACK_DURATION = 10.1    # 最终巡线时长 (秒)

# ================================================================
#  Phase 5 — 站立 + 白线单跳 + 线消失左转 + 右平移
# ================================================================

# --- 黑线检测 ---
P5_THRESHOLD = 80
P5_MIN_AREA = 200.0

# --- PID 巡线控制 ---
P5_PID_KP = 0.80
P5_PID_KI = 0.04
P5_PID_KD = 0.08
P5_PID_MAX_YAW = 2.0
P5_PID_INTEGRAL_MAX = 1.5

# --- 巡线速度 ---
P5_BASE_SPEED = 0.25

# --- 白线检测 ---
P5_WHITE_BRIGHTNESS = 140
P5_WHITE_GAP_MIN = 4
P5_WHITE_CROSS_CONFIRM = 4
P5_WHITE_BGR_THRESH = 155
P5_WHITE_LINE_RATIO = 0.30

# --- 白线后居中校准 ---
P5_CENTER_THRESHOLD = 0.12
P5_CENTER_CONFIRM = 8
P5_ALIGN_MAX_YAW = 1.0
P5_ALIGN_TIMEOUT = 3.0

# --- 前跳 (单次) ---
P5_JUMP_FORWARD_DUR = 0.5       # 跳跃前冲时长 (秒)
P5_JUMP_FORWARD_SPEED = 0.3     # 跳跃前冲速度 (m/s)
P5_JUMP_STOP_DUR = 0.5          # 跳跃前停顿 (秒)

# --- 线消失左转 ---
P5_TURN_LEFT_ANGLE = math.radians(85)   # 左转角度 (rad)
P5_TURN_LEFT_SPEED = 1.0                # 左转速度 (rad/s)
P5_TURN_TIMEOUT = 5.0                   # 转向超时 (秒)

# --- 右平移 ---
P5_RIGHT_SPEED = -0.18          # 横向速度 (负=右, 正=左, m/s)
P5_RIGHT_DURATION = 2.4       # 平移时长 (秒)
P5_POST_FORWARD_SPEED = 0.25    # 跳跃后直走速度 (m/s)
P5_POST_FORWARD_DUR = 3.0       # 跳跃后直走时长 (秒)


# ================================================================
# 🔩 状态机常量 — 以下为各 Phase 的子状态编号，一般不需要修改
# ================================================================

# --- Phase 1 子状态 ---
P1S_NORMAL = 0
P1S_GO_FORWARD = 1
P1S_ROTATING = 2
P1S_JUMP = 3
P1S_JUMP_ALIGN = 4
P1S_DONE = 5
P1S_POST_FORWARD = 6

# --- Phase 3 子状态 ---
P3S_CENTER_1 = 0
P3S_LINE_TRACK = 1
P3S_STAIRS_UP = 2
P3S_TURN_LEFT = 3
P3S_STAIRS_DOWN = 4
P3S_CENTER_2 = 5
P3S_DONE = 6

# --- Phase 4 子状态 ---
P4S_LINE_TRACK_1 = 0
P4S_TURN_LEFT = 1
P4S_STAND_UP = 2
P4S_TURN_RIGHT = 3
P4S_LINE_TRACK_2 = 4
P4S_DONE = 5
P4S_LOST_STAND = 6
P4S_LOST_TURN_LEFT = 7
P4S_RED_TURN_LEFT = 8
P4S_RED_TURN_RIGHT = 9
P4S_FINAL_TRACK = 10
P4S_FORWARD_1S = 11

# --- Phase 5 子状态 ---
P5S_STAND_5S = 0
P5S_LINE_TRACK = 1
P5S_JUMP_ALIGN = 2
P5S_JUMP = 3
P5S_POST_FORWARD = 4
P5S_TURN_LEFT_85 = 5
P5S_MOVE_RIGHT = 6
P5S_DONE = 7


# ============================================================
# PID 控制器
# ============================================================
class PIDController:
    """离散 PID 控制器，带积分抗饱和与输出限幅"""

    def __init__(self, kp, ki, kd, output_min, output_max,
                 integral_min=-1.0, integral_max=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max
        self.integral_min = integral_min
        self.integral_max = integral_max
        self.reset()

    def reset(self, initial_error=0.0):
        self._prev_error = initial_error
        self._integral = 0.0

    def update(self, error, dt):
        if dt <= 0.0 or dt > 0.5:
            dt = 0.02
        self._integral += 0.5 * (error + self._prev_error) * dt
        if self._integral > self.integral_max:
            self._integral = self.integral_max
        elif self._integral < self.integral_min:
            self._integral = self.integral_min
        derivative = (error - self._prev_error) / dt
        self._prev_error = error
        output = (self.kp * error +
                  self.ki * self._integral +
                  self.kd * derivative)
        if output > self.output_max:
            output = self.output_max
            self._integral -= self.ki * error * dt * 0.5
        elif output < self.output_min:
            output = self.output_min
            self._integral -= self.ki * error * dt * 0.5
        return output


# ============================================================
# 统一黑线检测器 (合并 track_and_control.py + step.py)
# ============================================================
class LineDetector:
    """黑线检测，支持白线横切检测、黑色占比、底部黑边检测"""

    def __init__(self, black_threshold=80, min_area=200.0,
                 white_threshold=180, min_white_gap=6,
                 white_bgr_threshold=155, white_ratio=0.30):
        self.black_threshold = black_threshold
        self.white_threshold = white_threshold
        self.min_white_gap = min_white_gap
        self.min_area = min_area
        self.avg_area = 800.0
        self.white_bgr_threshold = white_bgr_threshold
        self.white_ratio_threshold = white_ratio

    def detect(self, gray_frame):
        """检测黑线，返回 (found, offset, cx, cy, area,
           touches_left, touches_right, significant_count)"""
        h, w = gray_frame.shape
        img_center_x = w // 2
        _, binary = cv2.threshold(gray_frame, self.black_threshold, 255,
                                   cv2.THRESH_BINARY_INV)
        roi_top = h * 2 // 3
        roi = binary[roi_top:h, :]
        contours, _ = cv2.findContours(roi, cv2.RETR_EXTERNAL,
                                        cv2.CHAIN_APPROX_SIMPLE)
        found = False
        best_cx = 0
        best_cy = 0
        largest_area = 0.0
        touches_left = False
        touches_right = False
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > largest_area and area > self.min_area:
                M = cv2.moments(cnt)
                if M["m00"] != 0:
                    largest_area = area
                    best_cx = int(M["m10"] / M["m00"])
                    best_cy = int(M["m01"] / M["m00"]) + roi_top
                    found = True
                    for pt in cnt:
                        if pt[0][0] <= 2:
                            touches_left = True
                        if pt[0][0] >= w - 3:
                            touches_right = True
        significant_count = sum(1 for cnt in contours
                                if cv2.contourArea(cnt) > 300)
        if found:
            offset = float(img_center_x - best_cx) / img_center_x
            self.avg_area = self.avg_area * 0.85 + largest_area * 0.15
        else:
            offset = 0.0
        return (found, offset, best_cx, best_cy, largest_area,
                touches_left, touches_right, significant_count)

    def detect_white_line_cross(self, gray_frame, color_frame=None):
        """检测白线横切（双模式）"""
        h, w = gray_frame.shape
        roi_top = h * 2 // 3
        roi_h = h - roi_top
        if roi_h < 20:
            return False
        center_x = w // 2

        # 模式1: 直接检测白色像素
        if color_frame is not None:
            roi_color = color_frame[roi_top:h, :]
            b, g, r = cv2.split(roi_color)
            bright_mask = (b > self.white_bgr_threshold) & \
                          (g > self.white_bgr_threshold) & \
                          (r > self.white_bgr_threshold)
            white_mask = bright_mask.astype(np.uint8) * 255
            half_win = 70
            left = max(0, center_x - half_win)
            right = min(w, center_x + half_win)
            win_w = right - left
            row_white = np.count_nonzero(white_mask[:, left:right], axis=1)
            white_ratio = row_white.astype(float) / max(win_w, 1)
            white_line_rows = white_ratio > self.white_ratio_threshold
            segments = []
            in_seg = False
            seg_start = 0
            for r in range(roi_h):
                if white_line_rows[r] and not in_seg:
                    in_seg = True
                    seg_start = r
                elif not white_line_rows[r] and in_seg:
                    in_seg = False
                    h_seg = r - seg_start
                    if h_seg >= self.min_white_gap:
                        segments.append((seg_start, r, h_seg))
            if in_seg:
                h_seg = roi_h - seg_start
                if h_seg >= self.min_white_gap:
                    segments.append((seg_start, roi_h, h_seg))
            if segments:
                best = max(segments, key=lambda s: s[2])
                gs, ge, gh = best
                margin = int(roi_h * 0.15)
                if gs > margin and ge < roi_h - margin:
                    return True

        # 模式2: 黑线投影间隙
        _, black_binary = cv2.threshold(
            gray_frame, self.black_threshold, 255, cv2.THRESH_BINARY_INV)
        roi_black = black_binary[roi_top:h, :]
        half_win = 50
        left = max(0, center_x - half_win)
        right = min(w, center_x + half_win)
        row_black = np.count_nonzero(roi_black[:, left:right], axis=1)
        kernel = np.ones(3) / 3.0
        row_black_smooth = np.convolve(row_black, kernel, mode='same')
        max_black = np.max(row_black_smooth)
        if max_black < 5:
            return False
        gap_threshold = max(2.0, max_black * 0.05)
        gap_segments = []
        in_gap = False
        gap_start = 0
        for r in range(roi_h):
            is_gap = row_black_smooth[r] < gap_threshold
            if is_gap and not in_gap:
                in_gap = True
                gap_start = r
            elif not is_gap and in_gap:
                in_gap = False
                if r - gap_start >= self.min_white_gap:
                    gap_segments.append((gap_start, r, r - gap_start))
        if in_gap and roi_h - gap_start >= self.min_white_gap:
            gap_segments.append((gap_start, roi_h, roi_h - gap_start))
        margin = int(roi_h * 0.20)
        for gs, ge, gh in gap_segments:
            if gs > margin and ge < roi_h - margin:
                return True
        return False

    def get_black_ratio(self, gray_frame):
        """计算画面中黑色像素占比"""
        h, w = gray_frame.shape
        _, binary = cv2.threshold(gray_frame, self.black_threshold, 255,
                                   cv2.THRESH_BINARY_INV)
        black_pixels = cv2.countNonZero(binary)
        ratio = black_pixels / binary.size
        return ratio


# ============================================================
# 红色同心圆检测器 (Phase 4 用)
# ============================================================
class TargetDetector:
    """HSV 红色椭圆/圆检测"""

    def __init__(self):
        self._frame_h = 0
        self._frame_w = 0

    def detect(self, frame):
        self._frame_h, self._frame_w = frame.shape[:2]
        frame_area = self._frame_h * self._frame_w

        blurred = cv2.GaussianBlur(frame, P4_RED_BLUR_KERNEL, 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(hsv, P4_RED_HSV_LOWER1, P4_RED_HSV_UPPER1)
        mask2 = cv2.inRange(hsv, P4_RED_HSV_LOWER2, P4_RED_HSV_UPPER2)
        mask = cv2.bitwise_or(mask1, mask2)

        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (P4_RED_MORPH_KERNEL, P4_RED_MORPH_KERNEL))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        candidates = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            area_ratio = area / frame_area
            if area_ratio < P4_TARGET_AREA_MIN:
                continue
            perimeter = cv2.arcLength(cnt, True)
            if perimeter == 0:
                continue
            circularity = 4 * np.pi * area / (perimeter ** 2)
            if circularity < P4_CIRCULARITY_THRESH:
                continue
            if len(cnt) < 5:
                continue
            candidates.append({
                "contour": cnt, "area": area, "area_ratio": area_ratio,
                "circularity": circularity,
            })

        if not candidates:
            return None

        candidates.sort(key=lambda c: c["area"], reverse=True)
        best = candidates[0]
        cnt = best["contour"]

        ellipse = cv2.fitEllipse(cnt)
        (cx, cy), (axes_w, axes_h), angle = ellipse

        major_axis = max(axes_w, axes_h)
        minor_axis = min(axes_w, axes_h)
        axis_ratio = minor_axis / major_axis
        center_error_norm = (cx / self._frame_w) - 0.5

        return {
            "center_x": cx, "center_y": cy,
            "center_error_norm": center_error_norm,
            "axis_ratio": axis_ratio,
            "ellipse_angle_deg": angle,
            "ellipse": ellipse,
        }


# ============================================================
# 多阶段整合主控类
# ============================================================
class IntegratedMission:
    """Go2 五阶段任务流水线"""

    def __init__(self, network_interface="", action_mode=1):
        # ---- 共享命令 ----
        self._cmd_lock = threading.Lock()
        self._target_vx = 0.0
        self._target_vy = 0.0
        self._target_vyaw = 0.0

        # ---- 机器人状态 ----
        self._robot_state = None
        self._robot_state_lock = threading.Lock()

        # ---- 动作模式 ----
        self._action_mode = action_mode

        # ---- 运行标志 ----
        self._running = True
        self._control_running = True
        self._init_done = False     # 初始化完成标志，防止控制线程提前发指令

        # ---- 检测器 ----
        self._detector = LineDetector(
            black_threshold=P1_THRESHOLD,
            min_area=P1_MIN_AREA,
            white_threshold=P1_WHITE_BRIGHTNESS,
            min_white_gap=P1_WHITE_GAP_MIN,
            white_bgr_threshold=P1_WHITE_BGR_THRESH,
            white_ratio=P1_WHITE_LINE_RATIO,
        )
        self._pid = PIDController(
            kp=P1_PID_KP, ki=P1_PID_KI, kd=P1_PID_KD,
            output_min=-P1_PID_MAX_YAW, output_max=P1_PID_MAX_YAW,
            integral_min=-P1_PID_INTEGRAL_MAX, integral_max=P1_PID_INTEGRAL_MAX,
        )
        self._red_detector = TargetDetector()

        # ---- 摄像头 ----
        self._pipeline = None

        # ---- DDS 初始化 ----
        print("[INIT] 初始化 ChannelFactory...")
        if network_interface:
            ChannelFactoryInitialize(0, network_interface)
        else:
            ChannelFactoryInitialize(0)

        self._msc = MotionSwitcherClient()
        self._msc.SetTimeout(5.0)
        self._msc.Init()

        self._sport = SportClient()
        self._sport.SetTimeout(10.0)
        self._sport.Init()

        self._audio = AudioClient()
        self._audio.SetTimeout(5.0)
        self._audio.Init()
        print("[INIT] AudioClient 初始化完成")

        # 前照灯控制 (通过 VuiClient)
        self._vui = VuiClient()
        self._vui.SetTimeout(5.0)
        self._vui.Init()
        print("[INIT] VuiClient 初始化完成")

        print(f"[INIT] 订阅状态主题: {TOPIC_HIGHSTATE}")
        self._state_suber = ChannelSubscriber(TOPIC_HIGHSTATE, SportModeState_)
        self._state_suber.Init(self._on_high_state, 10)

        # 机械臂控制发布器
        self._arm_pub = ChannelPublisher("rt/arm_Command", ArmString_)
        self._arm_pub.Init()
        self._arm_seq = 0
        print("[INIT] 机械臂发布器已初始化")
        self._arm_enable()
        # 使能后立即进入"夹住走"默认姿态，保持关节上力
        self._arm_set_angles(ARM_ACTIONS["夹住走"])
        time.sleep(0.3)
        print("[ARM] 默认姿态: 夹住走")

        print("[INIT] 等待首帧机器人状态...")
        waited = 0
        while self._robot_state is None and waited < 30:
            time.sleep(0.1)
            waited += 1
        if self._robot_state is None:
            print("[WARN] 未收到机器人状态，里程计数据将不可用")

    # ================================================================
    # 机器人状态
    # ================================================================
    def _on_high_state(self, msg):
        with self._robot_state_lock:
            self._robot_state = msg

    def _get_yaw(self):
        with self._robot_state_lock:
            if self._robot_state is None:
                return 0.0
            return float(self._robot_state.imu_state.rpy[2])

    def _get_position(self):
        with self._robot_state_lock:
            if self._robot_state is None:
                return (0.0, 0.0, 0.0)
            p = self._robot_state.position
            return (float(p[0]), float(p[1]), float(p[2]))

    @staticmethod
    def _normalize_angle(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    # ================================================================
    # 命令接口
    # ================================================================
    def set_command(self, vx, vy, vyaw):
        with self._cmd_lock:
            self._target_vx = vx
            self._target_vy = vy
            self._target_vyaw = vyaw

    def _get_command(self):
        with self._cmd_lock:
            return (self._target_vx, self._target_vy, self._target_vyaw)

    # ================================================================
    # 控制线程 (100Hz)
    # ================================================================
    def _control_loop(self):
        print("[CTRL] 控制线程启动 (@~100Hz)")
        time.sleep(0.1)  # 短暂等待，确保 sport 接口就绪
        period = 1.0 / CTRL_HZ
        while self._control_running:
            if self._init_done:
                vx, vy, vyaw = self._get_command()
                self._sport.Move(vx, vy, vyaw)
            time.sleep(period)
        print("[CTRL] 控制线程退出")

    # ================================================================
    # 机器人初始化 (仅一次)
    # ================================================================
    def _init_robot(self):
        print("[INIT] SelectMode('normal')...")
        self._msc.SelectMode("normal")
        time.sleep(2.0)

        print("[INIT] StandUp...")
        self._sport.StandUp()
        time.sleep(4.0)

        print("[INIT] SpeedLevel(1)...")
        self._sport.SpeedLevel(1)
        time.sleep(1.0)

        print("[INIT] ClassicWalk(True)...")
        self._sport.ClassicWalk(True)
        time.sleep(1.0)
        print("[INIT] Move(0,0,0) 锁定位置...")
        self._sport.Move(0.0, 0.0, 0.0)

    # ================================================================
    # 摄像头管理
    # ================================================================
    def _start_camera(self):
        self._pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, FRAME_WIDTH, FRAME_HEIGHT,
                             rs.format.bgr8, FPS)
        try:
            self._pipeline.start(config)
            print("[CAM] D435i 摄像头已启动")
        except RuntimeError as e:
            print(f"[ERROR] 摄像头启动失败: {e}")
            raise

    def _restart_camera(self):
        """重启摄像头管道 (Phase 4 TURN_RIGHT 后使用)"""
        print("[CAM] 重启 D435i...")
        if self._pipeline:
            try:
                self._pipeline.stop()
            except Exception:
                pass
        self._pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, FRAME_WIDTH, FRAME_HEIGHT,
                             rs.format.bgr8, FPS)
        try:
            self._pipeline.start(config)
            print("[CAM] D435i 重启成功")
        except RuntimeError as e:
            print(f"[ERROR] D435i 重启失败: {e}")

    # ================================================================
    # 步态切换
    # ================================================================
    def _switch_to_classic(self):
        # 先清零目标速度，让控制线程发 Move(0,0,0)，确保机器人完全静止再切步态
        self.set_command(0.0, 0.0, 0.0)
        time.sleep(0.15)
        self._sport.StopMove()
        time.sleep(0.3)
        self._sport.ClassicWalk(True)
        time.sleep(0.5)
        print("[GAIT] 切换经典步态 ClassicWalk")

    def _switch_to_free(self):
        # 先清零目标速度，让控制线程发 Move(0,0,0)，确保机器人完全静止再切步态
        self.set_command(0.0, 0.0, 0.0)
        time.sleep(0.15)
        self._sport.StopMove()
        time.sleep(0.3)
        self._sport.FreeWalk()
        time.sleep(0.5)
        print("[GAIT] 切换灵动步态 FreeWalk")

    # ================================================================
    # 机械臂控制 (来自 Arm.py)
    # ================================================================
    def _arm_send(self, funcode: int, data: dict = None):
        self._arm_seq += 1
        cmd = {"seq": self._arm_seq, "address": 1, "funcode": funcode}
        if data:
            cmd["data"] = data
        self._arm_pub.Write(ArmString_(json.dumps(cmd, ensure_ascii=False)))

    def _arm_set_angles(self, angles: list):
        self._arm_send(2, {
            "mode": 1,
            "angle0": angles[0], "angle1": angles[1], "angle2": angles[2],
            "angle3": angles[3], "angle4": angles[4], "angle5": angles[5],
            "angle6": angles[6],
        })

    def _arm_enable(self):
        """使能机械臂关节 (funcode=5, mode=0)"""
        print("[ARM] 使能关节...")
        self._arm_send(5, {"mode": 0})
        time.sleep(1.0)

    def _arm_move_sequence(self, action_names, delay=30.0):
        # 先让机器人停稳
        self.set_command(0.0, 0.0, 0.0)
        time.sleep(0.5)
        for name in action_names:
            print(f"[ARM] 执行动作: {name}")
            self._arm_set_angles(ARM_ACTIONS[name])
            time.sleep(0.8)
        # 等待机械臂物理上完成动作（关节运动需要时间）
        print(f"[ARM] 等待机械臂完成动作 ({delay:.0f}s)...")
        time.sleep(delay)
        print("[ARM] 机械臂动作完成")
        self.set_command(0.0, 0.0, 0.0)

    def _arm_put1(self):
        print("[ARM] 执行 put1 序列...")
        self._arm_move_sequence([
    
            "扫1", "扫2", "抬1", "夹住走"
        ], delay=30.0)
        print("[ARM] put1 序列发送完毕，等待机械臂完成动作...")

    def _arm_put2(self):
        print("[ARM] 执行 put2 序列...")
        self._arm_move_sequence([
            "抬起2", "小转2",
            "小放2", "小抓2", "小抬抓2", "夹住走"
        ], delay=32.0)
        time.sleep(0.5)
        print("[ARM] put2 序列发送完毕，等待机械臂完成动作...")

    def _arm_left(self):
        print("[ARM] 执行 left 序列...")
        self._arm_move_sequence([
             "放左抬起1", "放左", "放左松开", "放左抬起2", "夹住走"
        ], delay=30.0)
        time.sleep(0.5)
        print("[ARM] left 序列发送完毕，等待机械臂完成动作...")

    def _arm_right(self):
        print("[ARM] 执行 right 序列...")
        self._arm_move_sequence([
             "放右抬起1", "放右", "放右松开", "放右抬起2", "夹住走"
        ], delay=30.0)
        time.sleep(0.5)
        print("[ARM] right 序列发送完毕，等待机械臂完成动作...")

    def _set_headlight(self, on):
        level = 10 if on else 0
        code = self._vui.SetBrightness(level)
        if code != 0:
            print(f"[WARN] 前照灯控制失败，错误码: {code}")
        else:
            print(f"[LED] 前照灯 {'开启' if on else '关闭'} (亮度: {level})")

    def _blink_led(self, times=3):
        print(f"[LED] 闪烁前照灯 {times} 次")
        for i in range(times):
            self._set_headlight(True)
            time.sleep(0.3)
            self._set_headlight(False)
            time.sleep(0.3)
        self._set_headlight(False)
        print("[LED] 闪烁完成")

    # ================================================================
    # 检测器/PID 重配置
    # ================================================================
    def _configure_detector(self, black_threshold, min_area):
        self._detector.black_threshold = black_threshold
        self._detector.min_area = min_area

    def _configure_pid(self, kp, ki, kd, max_yaw, integral_max):
        self._pid.kp = kp
        self._pid.ki = ki
        self._pid.kd = kd
        self._pid.output_min = -max_yaw
        self._pid.output_max = max_yaw
        self._pid.integral_min = -integral_max
        self._pid.integral_max = integral_max
        self._pid.reset()

    # ================================================================
    # Phase 1: 黑线循迹 + 白线检测 + 两次前跳
    # ================================================================
    def _phase1_track_and_jump(self):
        print("\n" + "=" * 60)
        print("  Phase 1: 黑线循迹 + 白线检测 + 两次前跳")
        print("=" * 60)

        # 配置 Phase 1 参数
        self._configure_detector(P1_THRESHOLD, P1_MIN_AREA)
        self._configure_pid(P1_PID_KP, P1_PID_KI, P1_PID_KD,
                            P1_PID_MAX_YAW, P1_PID_INTEGRAL_MAX)
        self._sport.SpeedLevel(1)

        # Phase 1 状态变量
        state = P1S_NORMAL
        state_start_time = 0.0
        line_lost_count = 0
        sharp_turn_count = 0
        turn_direction = 0
        cooldown_frames = 0
        forward_frame_count = 0
        forward_start_x = 0.0
        forward_start_y = 0.0
        rotation_start_yaw = 0.0
        rotation_frame_count = 0
        white_cross_count = 0
        jump_frame_count = 0
        jump_start_time = 0.0
        jump_triggered = False
        jump_count = 0
        align_ok_count = 0
        align_start_time = 0.0
        stored_offset = 0.0
        self._pid.reset()
        startup_pid_pending = True

        phase1_done = False
        loop_frame = 0
        start_protect_frames = 20

        while self._running and not phase1_done:
            loop_frame += 1

            frames = self._pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            color_img = np.asanyarray(color_frame.get_data())
            gray = cv2.cvtColor(color_img, cv2.COLOR_BGR2GRAY)

            (found, offset, best_cx, best_cy, largest_area,
             touches_left, touches_right, sig_count) = self._detector.detect(gray)

            img_center_x = FRAME_WIDTH // 2
            is_cross = False
            is_sharp_turn = False

            if found:
                area_spike = (largest_area > self._detector.avg_area * 3.0
                              and self._detector.avg_area > 0)
                multi_branch = (sig_count >= 3)
                is_cross = (area_spike and multi_branch
                            and abs(offset) < 0.4)
                large_offset = abs(offset) > 0.75
                near_left = best_cx < img_center_x * 0.08
                near_right = best_cx > img_center_x * 1.92
                at_edge = touches_left or touches_right or near_left or near_right
                is_sharp_turn = False  # 禁用急转弯检测

            # 白线检测 (跳完后不依赖found，白线本身会遮挡黑线)
            is_white_cross = False
            if state == P1S_NORMAL and (found or jump_count >= 1):
                is_white_cross = self._detector.detect_white_line_cross(
                    gray, color_img)
                if is_white_cross:
                    white_cross_count += 1
                else:
                    white_cross_count = max(0, white_cross_count - 1)

            if cooldown_frames > 0:
                cooldown_frames -= 1

            # ---- Phase 1 状态机 ----
            if state == P1S_NORMAL:
                if loop_frame <= start_protect_frames:
                    self.set_command(0.0, 0.0, 0.0)
                    if loop_frame == start_protect_frames:
                        print("[P1] 启动保护期结束，开始正常巡线")
                    continue

                # 首个可信检测值只用于初始化 PID，避免启动时 D 项瞬间冲击。
                if startup_pid_pending and found and abs(offset) <= 0.80:
                    self._pid.reset(initial_error=offset)
                    startup_pid_pending = False

                if abs(offset) > 0.80:
                    line_lost_count += 1
                    self._pid.reset()
                    self.set_command(0.0, 0.0, 0.0)
                    continue
                else:
                    line_lost_count = 0

                # 第二次白线：不需要等cooldown，提高检测优先级
                if jump_count >= 1 and white_cross_count >= P1_WHITE_CROSS_CONFIRM:
                    print(f"[DETECT] 第二次白线！连续 {white_cross_count} 帧 "
                          f"-> 向前走 {P1_SECOND_WHITE_FORWARD_TIME}s -> Phase 2")
                    state = P1S_POST_FORWARD
                    state_start_time = time.time()
                    self.set_command(P1_SECOND_WHITE_FORWARD_SPEED, 0.0, 0.0)
                    white_cross_count = 0
                elif (jump_count < 1 and white_cross_count >= P1_WHITE_CROSS_CONFIRM
                        and cooldown_frames == 0):
                    print(f"[DETECT] 白线横切！连续 {white_cross_count} 帧 -> 校准居中")
                    state = P1S_JUMP_ALIGN
                    white_cross_count = 0
                    align_ok_count = 0
                    align_start_time = time.time()
                    stored_offset = offset
                    self._pid.reset()
                elif not found:
                    line_lost_count += 1
                    self._pid.reset()
                    if line_lost_count < 30:
                        self.set_command(0.05, 0.0, 0.0)
                    else:
                        search_yaw = 0.3 if line_lost_count % 120 < 60 else -0.3
                        self.set_command(0.0, 0.0, search_yaw)
                elif jump_count < 1 and is_sharp_turn and cooldown_frames == 0:
                    sharp_turn_count += 1
                    if sharp_turn_count >= P1_SHARP_TURN_THRESHOLD:
                        state = P1S_GO_FORWARD
                        sharp_turn_count = 0
                        turn_direction = 1 if offset > 0 else -1
                        forward_frame_count = 0
                        fx, fy, _ = self._get_position()
                        forward_start_x = fx
                        forward_start_y = fy
                        direction_name = "LEFT" if turn_direction > 0 else "RIGHT"
                        print(f"[STATE] -> GO_FORWARD dir={direction_name}")
                        self.set_command(0.5, 0.0, 0.0)
                    else:
                        yaw = self._pid.update(offset, CTRL_DT)
                        self.set_command(P1_BASE_SPEED, 0.0, yaw)
                elif is_cross:
                    line_lost_count = 0
                    self._pid.reset()
                    self.set_command(0.35, 0.0, 0.0)
                else:
                    sharp_turn_count = 0
                    line_lost_count = 0
                    yaw = self._pid.update(offset, CTRL_DT)
                    if abs(offset) > 0.5:
                        vx = P1_BASE_SPEED * 0.6
                    else:
                        vx = P1_BASE_SPEED
                    self.set_command(vx, 0.0, yaw)

            elif state == P1S_GO_FORWARD:
                forward_frame_count += 1
                fx, fy, _ = self._get_position()
                dx = fx - forward_start_x
                dy = fy - forward_start_y
                dist = math.sqrt(dx * dx + dy * dy)
                if dist >= P1_FORWARD_DIST or forward_frame_count >= 80:
                    state = P1S_ROTATING
                    rotation_start_yaw = self._get_yaw()
                    rotation_frame_count = 0
                    vyaw = 1.0 if turn_direction > 0 else -1.0
                    self.set_command(0.0, 0.0, vyaw)
                    direction_name = "LEFT" if turn_direction > 0 else "RIGHT"
                    print(f"[STATE] -> ROTATING dir={direction_name} "
                          f"dist={int(dist * 100)}cm")
                else:
                    self.set_command(0.5, 0.0, 0.0)

            elif state == P1S_ROTATING:
                rotation_frame_count += 1
                vyaw = 1.0 if turn_direction > 0 else -1.0
                self.set_command(0.0, 0.0, vyaw)
                current_yaw = self._get_yaw()
                yaw_diff = self._normalize_angle(current_yaw - rotation_start_yaw)
                if (abs(yaw_diff) >= P1_ROTATION_ANGLE
                        or rotation_frame_count >= 80):
                    state = P1S_NORMAL
                    line_lost_count = 0
                    sharp_turn_count = 0
                    cooldown_frames = P1_COOLDOWN_FRAMES
                    self._pid.reset()
                    self.set_command(0.0, 0.0, 0.0)

            elif state == P1S_JUMP_ALIGN:
                elapsed = time.time() - align_start_time
                if not found:
                    if elapsed > P1_ALIGN_TIMEOUT:
                        print(f"[ALIGN] 丢线超时 -> 直接跳")
                        state = P1S_JUMP
                        jump_frame_count = 0
                        jump_start_time = time.time()
                        jump_triggered = False
                        self.set_command(0.0, 0.0, 0.0)
                    else:
                        turn_dir = 0.4 if stored_offset > 0 else -0.4
                        self.set_command(0.0, 0.0, turn_dir)
                else:
                    stored_offset = offset
                    yaw = self._pid.update(offset, CTRL_DT)
                    yaw = max(-P1_ALIGN_MAX_YAW, min(P1_ALIGN_MAX_YAW, yaw))
                    self.set_command(0.0, 0.0, yaw)
                    if abs(offset) < P1_CENTER_THRESHOLD:
                        align_ok_count += 1
                        if align_ok_count >= P1_CENTER_CONFIRM:
                            print(f"[ALIGN] 居中完成 -> 前跳")
                            state = P1S_JUMP
                            jump_frame_count = 0
                            jump_start_time = time.time()
                            jump_triggered = False
                            self.set_command(0.0, 0.0, 0.0)
                    else:
                        if align_ok_count > 0:
                            align_ok_count = max(0, align_ok_count - 2)
                if elapsed > P1_ALIGN_TIMEOUT:
                    print(f"[ALIGN] 校准超时 ({elapsed:.1f}s) -> 直接跳")
                    state = P1S_JUMP
                    jump_frame_count = 0
                    jump_start_time = time.time()
                    jump_triggered = False
                    self.set_command(0.0, 0.0, 0.0)

            elif state == P1S_JUMP:
                elapsed = time.time() - jump_start_time
                t_forward_end = P1_JUMP_FORWARD_DUR
                t_stop_end = t_forward_end + P1_JUMP_STOP_DUR
                t_jump_end = t_stop_end + 2.5
                if elapsed < t_forward_end:
                    self.set_command(P1_JUMP_FORWARD_SPEED, 0.0, 0.0)
                elif elapsed < t_stop_end:
                    self.set_command(0.0, 0.0, 0.0)
                elif elapsed < t_jump_end:
                    if not jump_triggered:
                        jump_count += 1
                        code = self._sport.FrontJump()
                        jump_triggered = True
                        if code == 0:
                            print(f"[JUMP] FrontJump() #{jump_count} OK")
                        else:
                            print(f"[JUMP] FrontJump() #{jump_count} 失败, code={code}")
                    self.set_command(0.0, 0.0, 0.0)
                else:
                    state = P1S_NORMAL
                    cooldown_frames = P1_COOLDOWN_FRAMES
                    self._pid.reset()
                    self._sport.ClassicWalk(True)
                    print(f"[STATE] -> NORMAL (跳跃完成, 等待第二次白线)")

            elif state == P1S_POST_FORWARD:
                elapsed = time.time() - state_start_time
                if elapsed >= P1_SECOND_WHITE_FORWARD_TIME:
                    state = P1S_DONE
                    self.set_command(0.0, 0.0, 0.0)
                    self._sport.StopMove()
                    print(f"[STATE] Phase 1 DONE (向前走 {elapsed:.1f}s 完成, 保持站立)")
                    phase1_done = True
                else:
                    self.set_command(P1_SECOND_WHITE_FORWARD_SPEED, 0.0, 0.0)

            elif state == P1S_DONE:
                self.set_command(0.0, 0.0, 0.0)
                phase1_done = True

            # ---- 可视化 ----
            display = color_img.copy()
            if found:
                cv2.circle(display, (best_cx, best_cy), 6, (0, 255, 0), -1)
            cv2.line(display, (img_center_x, display.shape[0]),
                     (img_center_x, display.shape[0] - 40), (255, 0, 0), 2)
            roi_top = display.shape[0] * 2 // 3
            cv2.line(display, (0, roi_top), (display.shape[1], roi_top),
                     (0, 255, 255), 1)
            state_names = {P1S_NORMAL: "NORMAL", P1S_GO_FORWARD: "GO_FORWARD",
                           P1S_ROTATING: "ROTATING", P1S_JUMP_ALIGN: "JUMP_ALIGN",
                           P1S_JUMP: "JUMP", P1S_POST_FORWARD: "POST_FORWARD",
                           P1S_DONE: "DONE"}
            cv2.putText(display, f"Phase1 {state_names[state]} jumps={jump_count}",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            vx, _, vyaw = self._get_command()
            cv2.putText(display, f"vx={vx:.2f} vyaw={vyaw:+.2f}",
                        (10, display.shape[0] - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 0), 1)
            cv2.imshow("Integrated Mission", display)

            key = cv2.waitKey(1) & 0xFF
            if key == 27 or key == ord('q'):
                self._running = False
                return

            if loop_frame % 30 == 0 and state == P1S_NORMAL:
                print(f"[P1] offset={offset:+.3f} found={found}")

        print("[Phase1] 完成，进入 Phase 2")

    # ================================================================
    # Phase 2: 迷宫定序运动
    # ================================================================
    def _phase2_maze(self):
        print("\n" + "=" * 60)
        print("  Phase 2: 迷宫定序运动")
        print("=" * 60)

        self._switch_to_classic()
        total_steps = len(P2_ACTION_SEQUENCE)

        for idx, (action, param) in enumerate(P2_ACTION_SEQUENCE, 1):
            if not self._running:
                return
            print(f"\n===== Phase2 步骤 {idx}/{total_steps} =====")

            if action == "forward":
                print(f"[动作] 前进 {param}s, 速度 {P2_FORWARD_SPEED}m/s")
                self.set_command(P2_FORWARD_SPEED, 0.0, 0.0)
                time.sleep(param)
                self.set_command(0.0, 0.0, 0.0)
                time.sleep(0.1)
                print("[动作] 前进完成")

            elif action in ("strafe_left", "strafe_right"):
                vy = P2_STRAFE_SPEED if action == "strafe_left" else -P2_STRAFE_SPEED
                direction_name = "左横移" if action == "strafe_left" else "右横移"
                print(f"[动作] {direction_name} {param}s, vy={vy:+.2f}m/s")
                self.set_command(0.0, vy, 0.0)
                time.sleep(param)
                self.set_command(0.0, 0.0, 0.0)
                time.sleep(0.1)
                print(f"[动作] {direction_name}完成")

            elif action in ("arc_left", "arc_right"):
                if isinstance(param, (tuple, list)):
                    if len(param) == 3:
                        angle_deg, vx, yaw_speed = param
                        vy = 0.0
                    elif len(param) == 4:
                        angle_deg, vx, vy, yaw_speed = param
                    else:
                        print(f"[WARN] 圆弧参数数量错误: {param}")
                        continue
                else:
                    angle_deg = param
                    vx = P2_ARC_FORWARD_SPEED
                    turn_sign = 1.0 if action == "arc_left" else -1.0
                    vy = turn_sign * P2_ARC_LATERAL_SPEED
                    yaw_speed = P2_ARC_TURN_SPEED

                turn_sign = 1.0 if action == "arc_left" else -1.0
                vyaw = turn_sign * abs(yaw_speed)
                angle_rad = math.radians(abs(angle_deg))
                est_duration = angle_rad / max(abs(yaw_speed), 1e-6)
                timeout = max(P2_TURN_TIMEOUT, est_duration + 2.0)
                direction_name = "左弧转" if action == "arc_left" else "右弧转"

                if self._robot_state is None:
                    print(
                        f"[动作] 无IMU，估算{direction_name} {est_duration:.2f}s "
                        f"angle={angle_deg}°, vx={vx:.2f}, vy={vy:+.2f}, "
                        f"vyaw={vyaw:+.2f}"
                    )
                    self.set_command(vx, vy, vyaw)
                    time.sleep(est_duration)
                    self.set_command(0.0, 0.0, 0.0)
                    time.sleep(0.1)
                else:
                    start_yaw = self._get_yaw()
                    start_time = time.time()
                    print(
                        f"[动作] {direction_name} {angle_deg}° "
                        f"vx={vx:.2f}, vy={vy:+.2f}, vyaw={vyaw:+.2f}"
                    )
                    self.set_command(vx, vy, vyaw)
                    while self._running:
                        if time.time() - start_time > timeout:
                            print(f"[WARN] {direction_name}超时，强制停止")
                            break
                        current_yaw = self._get_yaw()
                        yaw_diff = self._normalize_angle(current_yaw - start_yaw)
                        if abs(yaw_diff) >= angle_rad:
                            break
                        time.sleep(0.01)
                    self.set_command(0.0, 0.0, 0.0)
                    time.sleep(0.1)
                    actual_deg = math.degrees(abs(
                        self._normalize_angle(self._get_yaw() - start_yaw)))
                    print(f"[动作] {direction_name}完成，实际转过 {actual_deg:.1f}°")

            elif action == "turn_left":
                angle_rad = math.radians(param)
                if self._robot_state is None:
                    est_duration = angle_rad / P2_TURN_SPEED
                    print(f"[动作] 无IMU，估算左转 {est_duration:.2f}s")
                    self.set_command(0.0, 0.0, P2_TURN_SPEED)
                    time.sleep(est_duration)
                    self.set_command(0.0, 0.0, 0.0)
                    time.sleep(0.1)
                else:
                    start_yaw = self._get_yaw()
                    start_time = time.time()
                    print(f"[动作] 左转 {param}°")
                    self.set_command(0.0, 0.0, P2_TURN_SPEED)
                    while self._running:
                        if time.time() - start_time > P2_TURN_TIMEOUT:
                            print("[WARN] 左转超时，强制停止")
                            break
                        current_yaw = self._get_yaw()
                        yaw_diff = self._normalize_angle(current_yaw - start_yaw)
                        if abs(yaw_diff) >= angle_rad:
                            break
                        time.sleep(0.01)
                    self.set_command(0.0, 0.0, 0.0)
                    time.sleep(0.1)
                    actual_deg = math.degrees(abs(
                        self._normalize_angle(self._get_yaw() - start_yaw)))
                    print(f"[动作] 左转完成，实际转过 {actual_deg:.1f}°")

            elif action == "turn_right":
                angle_rad = math.radians(param)
                if self._robot_state is None:
                    est_duration = angle_rad / P2_TURN_SPEED
                    print(f"[动作] 无IMU，估算右转 {est_duration:.2f}s")
                    self.set_command(0.0, 0.0, -P2_TURN_SPEED)
                    time.sleep(est_duration)
                    self.set_command(0.0, 0.0, 0.0)
                    time.sleep(0.1)
                else:
                    start_yaw = self._get_yaw()
                    start_time = time.time()
                    print(f"[动作] 右转 {param}°")
                    self.set_command(0.0, 0.0, -P2_TURN_SPEED)
                    while self._running:
                        if time.time() - start_time > P2_TURN_TIMEOUT:
                            print("[WARN] 右转超时，强制停止")
                            break
                        current_yaw = self._get_yaw()
                        yaw_diff = self._normalize_angle(current_yaw - start_yaw)
                        if abs(yaw_diff) >= angle_rad:
                            break
                        time.sleep(0.01)
                    self.set_command(0.0, 0.0, 0.0)
                    time.sleep(0.1)
                    actual_deg = math.degrees(abs(
                        self._normalize_angle(self._get_yaw() - start_yaw)))
                    print(f"[动作] 右转完成，实际转过 {actual_deg:.1f}°")

        print("\n[Phase2] 全部动作执行完毕，进入 Phase 3")

    # ================================================================
    # Phase 3: 巡线 + 黑区FreeWalk + 后退
    # ================================================================
    def _phase3_stairs(self):
        print("\n" + "=" * 60)
        print("  Phase 3: 巡线 + 黑区FreeWalk + 后退")
        print("=" * 60)

        # 配置 Phase 3 参数
        self._configure_detector(P3_BLACK_THRESHOLD, P3_MIN_AREA)
        self._configure_pid(P3_PID_KP, P3_PID_KI, P3_PID_KD,
                            P3_PID_MAX_YAW, P3_PID_INTEGRAL_MAX)
        self._sport.SpeedLevel(2)
        print("[P3] 速度档位 = 2")

        # Phase 3 状态变量
        state = P3S_CENTER_1
        state_start_time = time.time()
        state_start_yaw = self._get_yaw()
        black_confirm_count = 0
        center_confirm = 0
        self._pid.reset()

        while self._running:
            # FreeWalk 纯延时状态不需要摄像头
            if state in (P3S_STAIRS_UP, P3S_TURN_LEFT, P3S_STAIRS_DOWN):
                if state == P3S_STAIRS_UP:
                    elapsed = time.time() - state_start_time
                    if elapsed < P3_STAIRS_UP_DUR:
                        self.set_command(P3_STAIRS_SPEED, 0, 0)
                    else:
                        self.set_command(0, 0, 0)
                        time.sleep(0.4)
                        state = P3S_TURN_LEFT
                        state_start_time = time.time()
                        state_start_yaw = self._get_yaw()
                        print("[P3] STAIRS_UP 完成 -> TURN_LEFT(60°)")

                elif state == P3S_TURN_LEFT:
                    current_yaw = self._get_yaw()
                    diff = abs(self._normalize_angle(current_yaw - state_start_yaw))
                    target = math.radians(P3_TURN_ANGLE_DEG)
                    if diff < target:
                        self.set_command(0, 0, P3_TURN_SPEED)
                    else:
                        self.set_command(0, 0, 0)
                        time.sleep(0.4)
                        state = P3S_STAIRS_DOWN
                        state_start_time = time.time()
                        print("[P3] 左转60°完成 -> STAIRS_DOWN")

                elif state == P3S_STAIRS_DOWN:
                    elapsed = time.time() - state_start_time
                    if elapsed < P3_STAIRS_DOWN_DUR:
                        self.set_command(P3_STAIRS_SPEED, 0, 0)
                    else:
                        self.set_command(0, 0, 0)
                        time.sleep(0.4)
                        print("[P3] FreeWalk动作完成，切回ClassicWalk，强行向右搜索黑线")
                        self._switch_to_classic()
                        state = P3S_CENTER_2
                        state_start_time = time.time()
                        center_confirm = -30     # 强制右移30帧后才允许居中
                        self._pid.reset()

                time.sleep(CTRL_DT)
                continue

            # 需要视觉处理的状态
            frames = self._pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                time.sleep(0.01)
                continue

            color_img = np.asanyarray(color_frame.get_data())
            gray = cv2.cvtColor(color_img, cv2.COLOR_BGR2GRAY)
            (found, offset, best_cx, best_cy, largest_area,
             _touches_left, _touches_right, _sig_count) = self._detector.detect(gray)

            if state == P3S_CENTER_1:
                if found:
                    vy = P3_CENTER_LATERAL_KP * offset
                    vy = np.clip(vy, -P3_CENTER_LATERAL_MAX, P3_CENTER_LATERAL_MAX)
                    self.set_command(0.0, vy, 0.0)
                    if abs(offset) < P3_CENTER_THRESHOLD:
                        center_confirm += 1
                    else:
                        center_confirm = max(0, center_confirm - 1)
                    if center_confirm >= P3_CENTER_CONFIRM_FRAMES:
                        print(f"[P3] 初始居中完成 offset={offset:.3f}，开始巡线")
                        self.set_command(0, 0, 0)
                        time.sleep(0.3)
                        state = P3S_LINE_TRACK
                        state_start_time = time.time()
                        black_confirm_count = 0
                        self._pid.reset()
                else:
                    self.set_command(0, 0, 0)
                    center_confirm = max(0, center_confirm - 1)

            elif state == P3S_LINE_TRACK:
                black_ratio = self._detector.get_black_ratio(gray)

                if black_ratio > P3_BLACK_RATIO_THRESHOLD:
                    black_confirm_count += 1
                    if black_confirm_count >= P3_BLACK_CONFIRM_FRAMES:
                        print(f"[P3] 检测大面积黑色({black_ratio:.2f})，切换FreeWalk!")
                        self.set_command(0, 0, 0)
                        time.sleep(0.4)
                        self._sport.StopMove()
                        self._switch_to_free()
                        state = P3S_STAIRS_UP
                        state_start_time = time.time()
                        continue
                else:
                    black_confirm_count = max(0, black_confirm_count - 1)

                if found:
                    yaw = self._pid.update(offset, CTRL_DT)
                    slow_coeff = 0.6 if abs(offset) > 0.5 else 1.0
                    vx = P3_BASE_SPEED * slow_coeff
                    self.set_command(vx, 0.0, yaw)
                else:
                    self._pid.reset()
                    self.set_command(0.03, 0, 0)

            elif state == P3S_CENTER_2:
                # 先强制向右平移搜索，找到线后再居中
                if center_confirm < 0:
                    # 强制右移阶段，无视 found，先扫一遍
                    self.set_command(0, P3_CENTER2_SEARCH_SPEED, 0)
                    center_confirm += 1
                elif found:
                    vy = P3_CENTER_LATERAL_KP * offset
                    vy = np.clip(vy, -P3_CENTER_LATERAL_MAX, P3_CENTER_LATERAL_MAX)
                    self.set_command(0.0, vy, 0.0)
                    if abs(offset) < P3_CENTER_THRESHOLD:
                        center_confirm += 1
                    else:
                        center_confirm = max(0, center_confirm - 1)
                    if center_confirm >= P3_CENTER_CONFIRM_FRAMES:
                        print("[P3] 二次居中完成，Phase 3 结束")
                        self.set_command(0, 0, 0)
                        time.sleep(0.3)
                        state = P3S_DONE
                else:
                    self.set_command(0, P3_CENTER2_SEARCH_SPEED, 0)
                    center_confirm = max(0, center_confirm - 1)

            elif state == P3S_DONE:
                self.set_command(0, 0, 0)
                break

            # ---- 可视化 ----
            display = color_img.copy()
            h, w = display.shape[:2]
            cv2.line(display, (w // 2, 0), (w // 2, h), (255, 0, 0), 2)
            cv2.line(display, (0, h * 2 // 3), (w, h * 2 // 3), (0, 255, 255), 2)
            if found:
                cv2.circle(display, (int(best_cx), int(best_cy)), 8, (0, 255, 0), -1)
            state_names = {P3S_CENTER_1: "CENTER_1", P3S_LINE_TRACK: "LINE_TRACK",
                           P3S_STAIRS_UP: "STAIRS_UP", P3S_TURN_LEFT: "TURN_LEFT",
                           P3S_STAIRS_DOWN: "STAIRS_DOWN", P3S_CENTER_2: "CENTER_2",
                           P3S_DONE: "DONE"}
            cv2.putText(display, f"Phase3 {state_names[state]}",
                        (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            if state == P3S_LINE_TRACK:
                br = self._detector.get_black_ratio(gray)
                cv2.putText(display, f"Black:{br:.2f}", (10, 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                            (0, 0, 255) if br > 0.8 else (255, 255, 255), 2)
            cv2.imshow("Integrated Mission", display)

            key = cv2.waitKey(1) & 0xFF
            if key == 27 or key == ord('q'):
                self._running = False
                return

            time.sleep(CTRL_DT)

        print("[Phase3] 完成，进入 Phase 4")

    # ================================================================
    # Phase 4: 巡线 + 红圆检测 + 终段巡线
    # ================================================================
    def _phase4_final_track(self):
        print("\n" + "=" * 60)
        print("  Phase 4: 巡线 + 红圆检测 + 终段巡线")
        print("=" * 60)

        # 配置 Phase 4 参数
        self._configure_detector(P4_THRESHOLD, P4_MIN_AREA)
        self._configure_pid(P4_PID_KP, P4_PID_KI, P4_PID_KD,
                            P4_PID_MAX_YAW, P4_PID_INTEGRAL_MAX)
        self._sport.SpeedLevel(1)
        self._switch_to_classic()

        # Phase 4 状态变量
        state = P4S_LINE_TRACK_1
        phase_start_time = time.time()

        # 转向状态
        turn_left_start_yaw = 0.0
        turn_left_frame_count = 0
        turn_right_start_yaw = 0.0
        turn_right_frame_count = 0

        # 丢线恢复
        lost_turn_start_yaw = 0.0
        lost_turn_frame_count = 0

        # 红圆检测
        red_timer_start = None
        red_turn_left_start_yaw = 0.0
        red_turn_left_frame_count = 0
        red_turn_right_start_yaw = 0.0
        red_turn_right_frame_count = 0

        self._pid.reset()
        offset = 0.0
        found = False

        while self._running:
            # 不需要摄像头视觉的状态
            if state == P4S_STAND_UP:
                print(f"\n[P4] 执行 put1 机械臂动作\n")
                self._arm_put1()
                self._switch_to_classic()
                state = P4S_TURN_RIGHT
                phase_start_time = time.time()
                turn_right_start_yaw = self._get_yaw()
                turn_right_frame_count = 0
                self.set_command(0.0, 0.0, P4_TURN_RIGHT_SPEED)
                continue

            # 需要摄像头视觉的状态
            frames = self._pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            color_img = np.asanyarray(color_frame.get_data())
            gray = cv2.cvtColor(color_img, cv2.COLOR_BGR2GRAY)
            (found, offset, best_cx, best_cy, largest_area,
             _tl, _tr, _sc) = self._detector.detect(gray)

            # ---- Phase 4 状态机 ----
            if state == P4S_LINE_TRACK_1:
                elapsed = time.time() - phase_start_time
                if elapsed >= P4_LINE_TRACK_DURATION:
                    state = P4S_FORWARD_1S
                    phase_start_time = time.time()
                    self.set_command(P4_BASE_SPEED, 0.0, 0.0)
                    print(f"\n[P4] -> FORWARD_1S (前进1秒) 巡线用时: {elapsed:.1f}s\n")
                elif not found:
                    self._pid.reset()
                    self.set_command(0.05, 0.0, 0.0)
                else:
                    yaw = self._pid.update(offset, CTRL_DT)
                    vx = P4_BASE_SPEED * 0.6 if abs(offset) > 0.5 else P4_BASE_SPEED
                    self.set_command(vx, 0.0, yaw)

                state_text = f"LINE_TRACK_1 {max(0, P4_LINE_TRACK_DURATION - elapsed):.1f}s"
                state_color = (0, 255, 0)

            elif state == P4S_FORWARD_1S:
                elapsed = time.time() - phase_start_time
                if elapsed >= P4_FORWARD_DURATION:
                    state = P4S_TURN_LEFT
                    phase_start_time = time.time()
                    turn_left_start_yaw = self._get_yaw()
                    turn_left_frame_count = 0
                    self.set_command(0.0, 0.0, P4_TURN_LEFT_SPEED)
                    print(f"\n[P4] -> TURN_LEFT (左转90°) 前进用时: {elapsed:.1f}s\n")
                else:
                    self.set_command(P4_BASE_SPEED, 0.0, 0.0)

                state_text = f"FORWARD_1S {max(0, P4_FORWARD_DURATION - elapsed):.1f}s"
                state_color = (255, 255, 0)

            elif state == P4S_TURN_LEFT:
                turn_left_frame_count += 1
                current_yaw = self._get_yaw()
                yaw_diff = self._normalize_angle(current_yaw - turn_left_start_yaw)
                if abs(yaw_diff) >= P4_TURN_LEFT_ANGLE or turn_left_frame_count >= 150:
                    self.set_command(0.0, 0.0, 0.0)
                    time.sleep(0.2)
                    # 左转完成后向前走0.2秒
                    print(f"\n[P4] 左转完成，向前走0.2s")
                    self.set_command(0.25, 0.0, 0.0)
                    time.sleep(0.2)
                    self.set_command(0.0, 0.0, 0.0)
                    self._sport.StopMove()
                    self._sport.StandUp()
                    self._switch_to_classic()
                    state = P4S_STAND_UP
                    phase_start_time = time.time()
                    print(f"\n[P4] -> STAND_UP (站立2s)\n")
                else:
                    self.set_command(0.0, 0.0, P4_TURN_LEFT_SPEED)
                state_text = f"TURN_LEFT {int(abs(yaw_diff) * 180 / math.pi)}deg"
                state_color = (0, 0, 255)

            elif state == P4S_TURN_RIGHT:
                turn_right_frame_count += 1
                current_yaw = self._get_yaw()
                yaw_diff = self._normalize_angle(current_yaw - turn_right_start_yaw)
                turn_elapsed = time.time() - phase_start_time
                if abs(yaw_diff) >= P4_TURN_RIGHT_ANGLE or turn_elapsed >= 5.0:
                    self.set_command(0.0, 0.0, 0.0)
                    time.sleep(0.2)
                    self._sport.StopMove()
                    print(f"\n[P4] 右转完成 {int(abs(yaw_diff) * 180 / math.pi)}deg\n")
                    self._restart_camera()
                    self._pid.reset()
                    self._switch_to_classic()
                    state = P4S_LINE_TRACK_2
                    phase_start_time = time.time()
                    red_timer_start = None
                    print(f"\n[P4] -> LINE_TRACK_2 (持续巡线 + 红圆检测)\n")
                    continue
                else:
                    self.set_command(0.0, 0.0, P4_TURN_RIGHT_SPEED)
                state_text = f"TURN_RIGHT {int(abs(yaw_diff) * 180 / math.pi)}deg"
                state_color = (255, 165, 0)

            elif state == P4S_LINE_TRACK_2:
                if not found:
                    print(f"\n[P4] -> LOST_STAND (线丢失，站立2s)\n")
                    state = P4S_LOST_STAND
                    phase_start_time = time.time()
                    self.set_command(0.0, 0.0, 0.0)
                    self._sport.StopMove()
                    self._sport.StandUp()
                else:
                    yaw = self._pid.update(offset, CTRL_DT)
                    vx = P4_BASE_SPEED * 0.6 if abs(offset) > 0.5 else P4_BASE_SPEED
                    self.set_command(vx, 0.0, yaw)

                    # 红圆检测
                    red_target = self._red_detector.detect(color_img)
                    if red_target is not None and red_timer_start is None:
                        red_timer_start = time.time()
                        print(f"[P4] 检测到红色圆，开始计时...")

                    if red_timer_start is not None:
                        red_elapsed = time.time() - red_timer_start
                        if red_elapsed >= P4_RED_TIMER_DURATION:
                            print(f"\n[P4] -> RED_TURN_LEFT (红圆计时{red_elapsed:.1f}s，左转83°)\n")
                            state = P4S_RED_TURN_LEFT
                            phase_start_time = time.time()
                            red_turn_left_start_yaw = self._get_yaw()
                            red_turn_left_frame_count = 0
                            red_timer_start = None
                            self.set_command(0.0, 0.0, P4_RED_TURN_LEFT_SPEED)

                red_info = ""
                if red_timer_start is not None:
                    red_info = f"  RED:{time.time() - red_timer_start:.1f}s"
                state_text = f"LINE_TRACK_2{red_info}"
                state_color = (0, 255, 0)

            elif state == P4S_LOST_STAND:
                elapsed = time.time() - phase_start_time
                if elapsed >= P4_STAND_UP_DURATION:
                    print(f"\n[P4] 站立完成，执行 put2 机械臂动作\n")
                    self._arm_put2()
                    print(f"\n[P4] -> LOST_TURN_LEFT (put2完成，左转30°)\n")
                    self._switch_to_classic()
                    state = P4S_LOST_TURN_LEFT
                    phase_start_time = time.time()
                    lost_turn_start_yaw = self._get_yaw()
                    lost_turn_frame_count = 0
                    self.set_command(0.0, 0.0, P4_LOST_TURN_SPEED)
                    continue
                state_text = f"LOST_STAND {elapsed:.1f}s"
                state_color = (255, 0, 255)

            elif state == P4S_LOST_TURN_LEFT:
                lost_turn_frame_count += 1
                current_yaw = self._get_yaw()
                yaw_diff = self._normalize_angle(current_yaw - lost_turn_start_yaw)
                if abs(yaw_diff) >= P4_LOST_TURN_ANGLE or lost_turn_frame_count >= 150:
                    self._pid.reset()
                    state = P4S_LINE_TRACK_2
                    phase_start_time = time.time()
                    print(f"\n[P4] -> LINE_TRACK_2 (左转完成，继续巡线)\n")
                else:
                    self.set_command(0.0, 0.0, P4_LOST_TURN_SPEED)
                state_text = f"LOST_TURN_LEFT {int(abs(yaw_diff) * 180 / math.pi)}deg"
                state_color = (0, 0, 255)

            elif state == P4S_RED_TURN_LEFT:
                red_turn_left_frame_count += 1
                current_yaw = self._get_yaw()
                yaw_diff = self._normalize_angle(current_yaw - red_turn_left_start_yaw)
                if abs(yaw_diff) >= P4_RED_TURN_LEFT_ANGLE or red_turn_left_frame_count >= 150:
                    print(f"\n[P4] -> RED_TURN_RIGHT (左转完成，右转90°)\n")
                    self.set_command(0.0, 0.0, 0.0)
                    time.sleep(0.3)
                    if self._action_mode in (1, 4):
                        print("[P4] 执行伸懒腰动作")
                        self._sport.Stretch()
                        time.sleep(2.0)
                    elif self._action_mode in (2, 5):
                        print("[P4] 执行打招呼动作")
                        self._sport.Hello()
                        time.sleep(2.0)
                    else:
                        self._blink_led(3)
                    self._switch_to_classic()
                    state = P4S_RED_TURN_RIGHT
                    phase_start_time = time.time()
                    red_turn_right_start_yaw = self._get_yaw()
                    red_turn_right_frame_count = 0
                    self.set_command(0.0, 0.0, P4_RED_TURN_RIGHT_SPEED)
                else:
                    self.set_command(0.0, 0.0, P4_RED_TURN_LEFT_SPEED)
                state_text = f"RED_TURN_LEFT {int(abs(yaw_diff) * 180 / math.pi)}deg"
                state_color = (0, 0, 255)

            elif state == P4S_RED_TURN_RIGHT:
                red_turn_right_frame_count += 1
                current_yaw = self._get_yaw()
                yaw_diff = self._normalize_angle(current_yaw - red_turn_right_start_yaw)
                if abs(yaw_diff) >= P4_RED_TURN_RIGHT_ANGLE or red_turn_right_frame_count >= 150:
                    self._pid.reset()
                    red_timer_start = None
                    state = P4S_FINAL_TRACK
                    phase_start_time = time.time()
                    print(f"\n[P4] -> FINAL_TRACK (巡线{P4_FINAL_TRACK_DURATION}s后结束)\n")
                else:
                    self.set_command(0.0, 0.0, P4_RED_TURN_RIGHT_SPEED)
                state_text = f"RED_TURN_RIGHT {int(abs(yaw_diff) * 180 / math.pi)}deg"
                state_color = (255, 165, 0)

            elif state == P4S_FINAL_TRACK:
                elapsed = time.time() - phase_start_time
                if elapsed >= P4_FINAL_TRACK_DURATION:
                    print(f"\n[P4] -> DONE (计时{elapsed:.1f}s，全部任务完成)\n")
                    self.set_command(0.0, 0.0, 0.0)
                    self._sport.StopMove()
                    state = P4S_DONE
                elif not found:
                    self._pid.reset()
                    self.set_command(0.05, 0.0, 0.0)
                else:
                    yaw = self._pid.update(offset, CTRL_DT)
                    vx = P4_BASE_SPEED * 0.6 if abs(offset) > 0.5 else P4_BASE_SPEED
                    self.set_command(vx, 0.0, yaw)
                state_text = f"FINAL_TRACK {elapsed:.1f}/{P4_FINAL_TRACK_DURATION}s"
                state_color = (0, 255, 0)

            elif state == P4S_DONE:
                print("[P4] Phase 4 完成，进入 Phase 5")
                break

            # ---- 可视化 ----
            display = color_img.copy()
            img_center_x = FRAME_WIDTH // 2
            if found:
                cv2.circle(display, (best_cx, best_cy), 6, (0, 255, 0), -1)
            cv2.line(display, (img_center_x, 0), (img_center_x, display.shape[0]),
                     (255, 0, 0), 1)
            roi_top = display.shape[0] * 2 // 3
            cv2.line(display, (0, roi_top), (display.shape[1], roi_top),
                     (0, 255, 255), 1)
            # 红圆可视化
            if state == P4S_LINE_TRACK_2:
                red_target = self._red_detector.detect(color_img)
                if red_target is not None:
                    cv2.ellipse(display, red_target["ellipse"], (0, 0, 255), 2)
                    cv2.circle(display, (int(red_target["center_x"]),
                               int(red_target["center_y"])), 5, (0, 0, 255), -1)

            cv2.putText(display, f"Phase4 {state_text}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, state_color, 2)
            vx, _, vyaw = self._get_command()
            cv2.putText(display, f"vx={vx:.2f} vyaw={vyaw:+.2f}",
                        (10, display.shape[0] - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 0), 1)
            cv2.imshow("Integrated Mission", display)

            key = cv2.waitKey(1) & 0xFF
            if key == 27 or key == ord('q'):
                self._running = False
                return

        print("[Phase4] 完成")

    # ================================================================
    # Phase 5: 站立5s + 白线检测单次跳跃 + 巡线 + 线消失左转85° + 右平移2s
    # ================================================================
    def _phase5_final_jump(self):
        print("\n" + "=" * 60)
        print("  Phase 5: 站立 + 白线检测单次跳跃 + 线消失左转 + 右平移")
        print("=" * 60)

        # 配置 Phase 5 参数
        self._configure_detector(P5_THRESHOLD, P5_MIN_AREA)
        self._configure_pid(P5_PID_KP, P5_PID_KI, P5_PID_KD,
                            P5_PID_MAX_YAW, P5_PID_INTEGRAL_MAX)
        self._sport.SpeedLevel(1)
        self._switch_to_classic()

        # 临时覆盖白线检测参数
        old_white_threshold = self._detector.white_threshold
        old_min_white_gap = self._detector.min_white_gap
        old_white_bgr = self._detector.white_bgr_threshold
        old_white_ratio = self._detector.white_ratio_threshold
        self._detector.white_threshold = P5_WHITE_BRIGHTNESS
        self._detector.min_white_gap = P5_WHITE_GAP_MIN
        self._detector.white_bgr_threshold = P5_WHITE_BGR_THRESH
        self._detector.white_ratio_threshold = P5_WHITE_LINE_RATIO

        # Phase 5 状态变量
        state = P5S_STAND_5S
        state_start_time = time.time()
        white_cross_count = 0
        align_ok_count = 0
        align_start_time = 0.0
        stored_offset = 0.0
        jump_triggered = False
        jump_start_time = 0.0
        turn_left_start_yaw = 0.0
        move_right_start_time = 0.0
        self._pid.reset()

        offset = 0.0
        found = False

        while self._running:
            # ---- 纯延时状态（不需要摄像头） ----
            if state == P5S_STAND_5S:
                self.set_command(0.0, 0.0, 0.0)
                if self._action_mode <= 3:
                    print(f"[P5] 执行 arm_left 机械臂动作...")
                    self._arm_left()
                    print(f"[P5] arm_left 完成，开始巡线+白线检测")
                else:
                    print(f"[P5] 执行 arm_right 机械臂动作...")
                    self._arm_right()
                    print(f"[P5] arm_right 完成，开始巡线+白线检测")
                state = P5S_LINE_TRACK
                white_cross_count = 0
                self._pid.reset()
                continue

            if state == P5S_TURN_LEFT_85:
                current_yaw = self._get_yaw()
                yaw_diff = abs(self._normalize_angle(current_yaw - turn_left_start_yaw))
                if yaw_diff >= P5_TURN_LEFT_ANGLE or (time.time() - state_start_time) > P5_TURN_TIMEOUT:
                    print(f"[P5] 左转85°完成 (实际{math.degrees(yaw_diff):.1f}°)，开始右平移2s")
                    state = P5S_MOVE_RIGHT
                    state_start_time = time.time()
                    move_right_start_time = time.time()
                    self.set_command(0.0, P5_RIGHT_SPEED, 0.0)
                else:
                    self.set_command(0.0, 0.0, P5_TURN_LEFT_SPEED)
                time.sleep(CTRL_DT)
                continue

            if state == P5S_MOVE_RIGHT:
                elapsed = time.time() - move_right_start_time
                if elapsed >= P5_RIGHT_DURATION:
                    print(f"[P5] 右平移{P5_RIGHT_DURATION}s完成，全部任务结束!")
                    self.set_command(0.0, 0.0, 0.0)
                    self._sport.StopMove()
                    state = P5S_DONE
                else:
                    self.set_command(0.0, P5_RIGHT_SPEED, 0.0)
                time.sleep(CTRL_DT)
                continue

            if state == P5S_POST_FORWARD:
                elapsed = time.time() - state_start_time
                if elapsed >= P5_POST_FORWARD_DUR:
                    print(f"[P5] 直走{P5_POST_FORWARD_DUR}s完成 -> TURN_LEFT_85 (左转85°)")
                    state = P5S_TURN_LEFT_85
                    state_start_time = time.time()
                    turn_left_start_yaw = self._get_yaw()
                    self.set_command(0.0, 0.0, P5_TURN_LEFT_SPEED)
                else:
                    self.set_command(P5_POST_FORWARD_SPEED, 0.0, 0.0)
                time.sleep(CTRL_DT)
                continue

            if state == P5S_DONE:
                print("[P5] 全部任务结束")
                self._running = False
                break

            # ---- 需要摄像头视觉的状态 ----
            frames = self._pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            color_img = np.asanyarray(color_frame.get_data())
            gray = cv2.cvtColor(color_img, cv2.COLOR_BGR2GRAY)
            (found, offset, best_cx, best_cy, largest_area,
             _tl, _tr, _sc) = self._detector.detect(gray)

            img_center_x = FRAME_WIDTH // 2

            # ---- Phase 5 状态机 ----
            if state == P5S_LINE_TRACK:
                # 白线检测
                is_white = self._detector.detect_white_line_cross(gray, color_img)
                if is_white:
                    white_cross_count += 1
                else:
                    white_cross_count = max(0, white_cross_count - 1)

                if white_cross_count >= P5_WHITE_CROSS_CONFIRM:
                    print(f"[P5] 检测到白线横切！(连续{white_cross_count}帧) -> 校准居中")
                    state = P5S_JUMP_ALIGN
                    white_cross_count = 0
                    align_ok_count = 0
                    align_start_time = time.time()
                    stored_offset = offset if found else 0.0
                    self.set_command(0.0, 0.0, 0.0)
                    self._pid.reset()
                elif not found:
                    self._pid.reset()
                    self.set_command(0.05, 0.0, 0.0)
                else:
                    yaw = self._pid.update(offset, CTRL_DT)
                    vx = P5_BASE_SPEED * 0.6 if abs(offset) > 0.5 else P5_BASE_SPEED
                    self.set_command(vx, 0.0, yaw)

                state_text = "LINE_TRACK"
                state_color = (0, 255, 0)

            elif state == P5S_JUMP_ALIGN:
                elapsed = time.time() - align_start_time
                if not found:
                    if elapsed > P5_ALIGN_TIMEOUT:
                        print(f"[P5] 丢线超时 -> 直接跳")
                        state = P5S_JUMP
                        jump_start_time = time.time()
                        jump_triggered = False
                        self.set_command(0.0, 0.0, 0.0)
                    else:
                        turn_dir = 0.4 if stored_offset > 0 else -0.4
                        self.set_command(0.0, 0.0, turn_dir)
                else:
                    stored_offset = offset
                    yaw = self._pid.update(offset, CTRL_DT)
                    yaw = max(-P5_ALIGN_MAX_YAW, min(P5_ALIGN_MAX_YAW, yaw))
                    self.set_command(0.0, 0.0, yaw)
                    if abs(offset) < P5_CENTER_THRESHOLD:
                        align_ok_count += 1
                        if align_ok_count >= P5_CENTER_CONFIRM:
                            print(f"[P5] 居中完成 -> 前跳")
                            state = P5S_JUMP
                            jump_start_time = time.time()
                            jump_triggered = False
                            self.set_command(0.0, 0.0, 0.0)
                    else:
                        if align_ok_count > 0:
                            align_ok_count = max(0, align_ok_count - 2)
                if elapsed > P5_ALIGN_TIMEOUT:
                    print(f"[P5] 校准超时 ({elapsed:.1f}s) -> 直接跳")
                    state = P5S_JUMP
                    jump_start_time = time.time()
                    jump_triggered = False
                    self.set_command(0.0, 0.0, 0.0)

                state_text = f"JUMP_ALIGN offset={stored_offset:+.3f}"
                state_color = (255, 255, 0)

            elif state == P5S_JUMP:
                elapsed = time.time() - jump_start_time
                t_forward_end = P5_JUMP_FORWARD_DUR
                t_stop_end = t_forward_end + P5_JUMP_STOP_DUR
                t_jump_end = t_stop_end + 2.5
                if elapsed < t_forward_end:
                    self.set_command(P5_JUMP_FORWARD_SPEED, 0.0, 0.0)
                elif elapsed < t_stop_end:
                    self.set_command(0.0, 0.0, 0.0)
                elif elapsed < t_jump_end:
                    if not jump_triggered:
                        code = self._sport.FrontJump()
                        jump_triggered = True
                        if code == 0:
                            print(f"[P5] FrontJump() OK (单次)")
                        else:
                            print(f"[P5] FrontJump() 失败, code={code}")
                    self.set_command(0.0, 0.0, 0.0)
                else:
                    # 跳跃完成，进入跳跃后巡线阶段
                    print(f"[P5] 跳跃完成，进入 POST_FORWARD (直走3s)")
                    self._sport.ClassicWalk(True)
                    self._pid.reset()
                    state = P5S_POST_FORWARD
                    state_start_time = time.time()
                    self.set_command(0.0, 0.0, 0.0)

                state_text = f"JUMP {'fwd' if elapsed < t_forward_end else 'jumping...'}"
                state_color = (0, 0, 255)


            # ---- 可视化 ----
            display = color_img.copy()
            if found:
                cv2.circle(display, (best_cx, best_cy), 6, (0, 255, 0), -1)
            cv2.line(display, (img_center_x, 0), (img_center_x, display.shape[0]),
                     (255, 0, 0), 1)
            roi_top = display.shape[0] * 2 // 3
            cv2.line(display, (0, roi_top), (display.shape[1], roi_top),
                     (0, 255, 255), 1)

            state_names = {P5S_STAND_5S: "ARM_ACTION", P5S_LINE_TRACK: "LINE_TRACK",
                           P5S_JUMP_ALIGN: "JUMP_ALIGN", P5S_JUMP: "JUMP",
                           P5S_POST_FORWARD: "POST_FORWARD",
                           P5S_TURN_LEFT_85: "TURN_LEFT_85",
                           P5S_MOVE_RIGHT: "MOVE_RIGHT", P5S_DONE: "DONE"}
            cv2.putText(display, f"Phase5 {state_names.get(state, state_text)}",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, state_color, 2)
            vx, vy, vyaw = self._get_command()
            cv2.putText(display, f"vx={vx:.2f} vy={vy:+.2f} vyaw={vyaw:+.2f}",
                        (10, display.shape[0] - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 0), 1)
            cv2.imshow("Integrated Mission", display)

            key = cv2.waitKey(1) & 0xFF
            if key == 27 or key == ord('q'):
                self._running = False
                return

            time.sleep(CTRL_DT)

        # 恢复白线检测参数
        self._detector.white_threshold = old_white_threshold
        self._detector.min_white_gap = old_min_white_gap
        self._detector.white_bgr_threshold = old_white_bgr
        self._detector.white_ratio_threshold = old_white_ratio

        print("[Phase5] 完成")

    # ================================================================
    # 关闭
    # ================================================================
    def _shutdown(self):
        print("\n[SHUTDOWN] 停止所有系统...")
        self._control_running = False
        time.sleep(0.3)

        try:
            self._sport.StopMove()
            time.sleep(0.4)
        except Exception:
            pass

        try:
            if self._pipeline:
                self._pipeline.stop()
                print("[CAM] 摄像头已关闭")
        except Exception:
            pass

        cv2.destroyAllWindows()
        print("[SHUTDOWN] 程序安全退出")

    # ================================================================
    # 顶层编排
    # ================================================================
    def run(self):
        # 一次性机器人初始化
        self._init_robot()

        # 启动控制线程（在摄像头之前，确保尽快开始发送 Move(0,0,0)）
        ctrl_thread = threading.Thread(
            target=self._control_loop, daemon=True, name="ctrl")
        ctrl_thread.start()

        # 启动摄像头
        self._start_camera()

        # 初始化完成，允许控制线程发指令
        self._init_done = True

        # 顺序执行全部阶段
        try:
            self._phase1_track_and_jump()
            if not self._running:
                print("[INFO] Phase 1 中被用户终止")
                self._shutdown()
                return

            self._phase2_maze()
            if not self._running:
                print("[INFO] Phase 2 中被用户终止")
                self._shutdown()
                return

            self._phase3_stairs()
            if not self._running:
                print("[INFO] Phase 3 中被用户终止")
                self._shutdown()
                return

            self._phase4_final_track()
            if not self._running:
                print("[INFO] Phase 4 中被用户终止")
                self._shutdown()
                return

            self._phase5_final_jump()

        except KeyboardInterrupt:
            print("\n[INFO] 用户中断")
        except Exception as e:
            print(f"\n[ERROR] {e}")
            import traceback
            traceback.print_exc()
        finally:
            self._shutdown()


# ============================================================
# 主入口
# ============================================================
def main():
    sys.stdout = os.fdopen(sys.stdout.fileno(), 'w', buffering=1)

    if len(sys.argv) >= 2:
        network_iface = sys.argv[1]
    else:
        print("[INFO] 未指定网络接口，自动检测...")
        network_iface = auto_detect_interface()
        if network_iface is None:
            print("用法: python3 integrated_mission.py <networkInterface>")
            print("示例: python3 integrated_mission.py eth0")
            sys.exit(-1)

    print("=" * 60)
    print("  Go2 多阶段整合任务")
    print("  Phase 1: 循迹 + 白线检测 + 两次前跳")
    print("  Phase 2: 迷宫定序运动")
    print("  Phase 3: 巡线 + 黑区FreeWalk + 后退")
    print("  Phase 4: 巡线 + 红圆检测 + 终段巡线")
    print("  Phase 5: 站立 + 机械臂 + 白线单跳 + 线消失左转 + 右平移")
    print(f"  网络接口: {network_iface}")
    print("=" * 60)
    print("  动作模式选择:")
    print("    1: Phase5 arm_left   + Phase4 伸懒腰")
    print("    2: Phase5 arm_left   + Phase4 打招呼")
    print("    3: Phase5 arm_left   + Phase4 闪烁灯3次")
    print("    4: Phase5 arm_right  + Phase4 伸懒腰")
    print("    5: Phase5 arm_right  + Phase4 打招呼")
    print("    6: Phase5 arm_right  + Phase4 闪烁灯3次")
    print("=" * 60)

    while True:
        try:
            action_mode = int(input("请输入动作模式(1-6): "))
            if 1 <= action_mode <= 6:
                break
            print("输入无效，请输入1-6之间的数字")
        except ValueError:
            print("输入无效，请输入数字")

    print(f"[INFO] 选择模式 {action_mode}")

    # 信号处理
    def on_signal(sig, frame):
        print("\n[STOP] 收到终止信号")
        sys.exit(0)

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    try:
        mission = IntegratedMission(network_interface=network_iface, action_mode=action_mode)
    except Exception as e:
        print(f"\n[ERROR] 使用接口 {network_iface} 初始化失败: {e}")
        auto_iface = auto_detect_interface()
        if auto_iface and auto_iface != network_iface:
            print(f"[INFO] 重试接口: {auto_iface}")
            mission = IntegratedMission(network_interface=auto_iface, action_mode=action_mode)
        else:
            print("[FATAL] 无法找到可用的网络接口")
            sys.exit(-1)

    mission.run()


if __name__ == "__main__":
    main()
