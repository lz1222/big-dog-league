# -*- coding: utf-8 -*-
"""不接 ROS 或真机的 T 型楼梯入口门禁单元测试。"""

import numpy as np

from rk_bringup.stair_line_handoff_core import detect_t_stair_marker


def test_t_marker_requires_wide_band_and_center_lane():
    """目标 T 形黑带通过，缺失横带或居中竖带均不得通过。"""
    image = np.full((240, 320, 3), 220, dtype=np.uint8)
    image[25:110, :] = 0
    image[132:, 138:184] = 0
    found, details = detect_t_stair_marker(image)
    assert found
    assert details['band_found']
    assert details['lane_found']

    no_lane = image.copy()
    no_lane[132:, :] = 220
    assert not detect_t_stair_marker(no_lane)[0]

    off_center = image.copy()
    off_center[132:, :] = 220
    off_center[132:, 10:56] = 0
    assert not detect_t_stair_marker(off_center)[0]
