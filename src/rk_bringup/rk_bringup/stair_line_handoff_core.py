# -*- coding: utf-8 -*-
"""楼梯独立验证链使用的纯视觉 T 标记门禁。

此模块不依赖 ROS、相机设备或运动 SDK，便于在不接触真机的情况下验证
T 型画面判定。它只输出视觉判断，绝不直接生成速度命令。
"""

import cv2
import numpy as np


def detect_t_stair_marker(frame, dark_threshold=70,
                          band_min_width_ratio=0.65,
                          lane_min_width_ratio=0.04,
                          lane_max_width_ratio=0.28):
    """保守识别“上宽横带＋下方居中竖带”的 T 型楼梯入口。

    横带只在上半画面检查，竖带只在下半画面检查；两个黑色区域同时成立才
    通过，避免单独黑线、阴影或台阶边缘误触发。输入必须是 BGR 三通道图像。
    """
    if frame is None or frame.ndim != 3 or frame.shape[2] != 3:
        return False, {'reason': 'invalid_frame'}

    threshold = int(max(0, min(255, dark_threshold)))
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    height, width = gray.shape[:2]
    if height < 4 or width < 4:
        return False, {'reason': 'frame_too_small'}
    dark = gray <= threshold

    # 取横带区域的最大单行黑色覆盖率，容忍两端被相机画面裁切。
    band_start = max(0, int(height * 0.08))
    band_end = max(band_start + 1, int(height * 0.55))
    row_coverage = dark[band_start:band_end, :].mean(axis=1)
    band_width_ratio = float(row_coverage.max()) if row_coverage.size else 0.0

    # 竖向引导带应进入图像下半部，且中心必须接近机器人前方。
    lane = dark[max(0, int(height * 0.55)):, :]
    column_coverage = lane.mean(axis=0) if lane.size else np.zeros(width)
    lane_columns = np.flatnonzero(column_coverage >= 0.55)
    lane_width_ratio = float(len(lane_columns)) / float(max(1, width))
    lane_center_ratio = (
        float(lane_columns.mean()) / float(max(1, width - 1))
        if len(lane_columns) else None
    )
    band_found = band_width_ratio >= band_min_width_ratio
    lane_found = (
        lane_min_width_ratio <= lane_width_ratio <= lane_max_width_ratio
        and lane_center_ratio is not None
        and 0.35 <= lane_center_ratio <= 0.65
    )
    return band_found and lane_found, {
        'band_found': band_found,
        'band_width_ratio': round(band_width_ratio, 3),
        'lane_found': lane_found,
        'lane_width_ratio': round(lane_width_ratio, 3),
        'lane_center_ratio': (
            None if lane_center_ratio is None else round(lane_center_ratio, 3)
        ),
    }
