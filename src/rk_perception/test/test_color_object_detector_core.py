"""颜色 RGB-D 核心的纯 Python 回归测试。"""

from __future__ import division

import os

import cv2
import numpy as np
import pytest
import yaml

from rk_perception.color_object_detector_core import (
    ColorObjectDetectorCore,
    ConfirmationTracker,
    DetectionCandidate,
    classify_contour_shape,
)


@pytest.fixture
def config():
    """读取正式默认配置，测试不会把未标定数值复制进源码。"""
    path = os.path.join(os.path.dirname(__file__), '..', 'config',
                        'color_object_detector.yaml')
    with open(path, 'r') as stream:
        return yaml.safe_load(stream)['color_object_detector']['ros__parameters']


@pytest.fixture
def camera_info():
    return {'fx': 100.0, 'fy': 100.0, 'cx': 100.0, 'cy': 100.0}


def _scene(color, center=(100, 100), radius=30, depth=500):
    """合成 BGR 彩色圆与 16UC1 深度，模拟对齐 RGB-D 图像。"""
    image = np.zeros((200, 200, 3), dtype=np.uint8)
    cv2.circle(image, center, radius, color, -1)
    depth_image = np.full((200, 200), depth, dtype=np.uint16)
    return image, depth_image


@pytest.mark.parametrize('color_name,bgr', [
    ('red', (0, 0, 255)), ('blue', (255, 0, 0)), ('green', (0, 255, 0)),
])
def test_primary_colors_segment_and_project(config, camera_info, color_name, bgr):
    image, depth = _scene(bgr)
    result = ColorObjectDetectorCore(config).detect(
        image, depth, '16UC1', camera_info, [color_name])
    assert result.detected
    assert result.color == color_name
    assert result.depth_m == pytest.approx(0.5)
    assert result.position_camera[2] == pytest.approx(0.5)


def test_red_hue_wraparound_merges_two_ranges(config):
    core = ColorObjectDetectorCore(config)
    hsv = np.full((200, 200, 3), (175, 220, 220), dtype=np.uint8)
    image = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
    assert np.count_nonzero(core.build_color_mask(image, 'red')) > 0


def test_non_target_and_empty_images_do_not_detect(config, camera_info):
    core = ColorObjectDetectorCore(config)
    blue, depth = _scene((255, 0, 0))
    assert not core.detect(blue, depth, '16UC1', camera_info, ['red']).detected
    empty = np.zeros_like(blue)
    assert not core.detect(empty, depth, '16UC1', camera_info, ['green']).detected


def test_noise_and_roi_outside_are_filtered(config, camera_info):
    core = ColorObjectDetectorCore(config)
    noisy, depth = _scene((0, 0, 0))
    cv2.circle(noisy, (100, 100), 4, (0, 0, 255), -1)
    assert not core.detect(noisy, depth, '16UC1', camera_info, ['red']).detected
    outside, depth = _scene((0, 0, 255), center=(15, 15))
    assert not core.detect(outside, depth, '16UC1', camera_info, ['red']).detected


def test_selection_prefers_grasp_image_center_not_largest_area(config, camera_info):
    image = np.zeros((200, 200, 3), dtype=np.uint8)
    cv2.circle(image, (105, 110), 24, (0, 0, 255), -1)
    cv2.circle(image, (160, 160), 36, (0, 0, 255), -1)
    depth = np.full((200, 200), 500, dtype=np.uint16)
    result = ColorObjectDetectorCore(config).detect(
        image, depth, '16UC1', camera_info, ['red'])
    assert result.detected
    assert result.center_x < 130


def test_region_depth_handles_zero_center_and_mad_outliers(config, camera_info):
    image, depth = _scene((0, 0, 255))
    depth[100, 100] = 0
    depth[90:94, 90:94] = 1400
    result = ColorObjectDetectorCore(config).detect(
        image, depth, '16UC1', camera_info, ['red'])
    assert result.detected
    assert result.depth_m == pytest.approx(0.5)
    assert result.valid_depth_pixels > config['min_valid_depth_pixels']


def test_depth_encoding_and_insufficient_depth(config, camera_info):
    image, depth16 = _scene((0, 255, 0), depth=600)
    core = ColorObjectDetectorCore(config)
    result16 = core.detect(image, depth16, '16UC1', camera_info, ['green'])
    assert result16.depth_m == pytest.approx(0.6)
    depth32 = depth16.astype(np.float32) * 0.001
    result32 = core.detect(image, depth32, '32FC1', camera_info, ['green'])
    assert result32.depth_m == pytest.approx(0.6)
    depth16[:] = 0
    assert not core.detect(image, depth16, '16UC1', camera_info, ['green']).detected


def test_projection_rejects_invalid_intrinsics_and_keeps_optical_axes(config, camera_info):
    core = ColorObjectDetectorCore(config)
    assert core.project_pixel(110, 120, 0.5, camera_info) == pytest.approx((0.05, 0.1, 0.5))
    image, depth = _scene((0, 0, 255))
    invalid_info = {'fx': 0, 'fy': 1, 'cx': 0, 'cy': 0}
    assert not core.detect(image, depth, '16UC1', invalid_info, ['red']).detected


def test_confirmation_requires_continuity_and_resets_on_jump():
    tracker = ConfirmationTracker(3, 2, 0.08, 0.05)
    candidate = DetectionCandidate(
        detected=True, color='red', depth_m=0.5, position_camera=(0.0, 0.0, 0.5))
    assert not tracker.update(candidate, 'red')
    assert not tracker.update(candidate, 'red')
    assert tracker.update(candidate, 'red')
    jumped = DetectionCandidate(
        detected=True, color='red', depth_m=0.7, position_camera=(0.0, 0.0, 0.7))
    assert not tracker.update(jumped, 'red')
    assert not tracker.update(candidate, 'none')


def test_invalid_configuration_fails_fast(config):
    config['min_depth_m'] = config['max_depth_m']
    with pytest.raises(ValueError):
        ColorObjectDetectorCore(config)


def _contour(points):
    """将合成二维点转换为 OpenCV 标准 contour 格式。"""
    return np.asarray(points, dtype=np.int32).reshape((-1, 1, 2))


def _rotated_rectangle(center, size, angle_deg):
    """生成旋转矩形，验证分类不依赖轴对齐 bbox。"""
    return _contour(cv2.boxPoints((center, size, angle_deg)))


@pytest.mark.parametrize('points', [
    [(100, 25), (35, 155), (165, 155)],
    [(100, 45), (55, 145), (145, 145)],
])
def test_triangles_are_classified_across_sizes(config, points):
    result = classify_contour_shape(
        _contour(points), config['shape_detection'])
    assert result.shape == 'triangle'
    assert 0.0 <= result.confidence <= 1.0


def test_axis_aligned_and_rotated_squares_are_classified(config):
    axis_aligned = _contour([(50, 50), (150, 50), (150, 150), (50, 150)])
    rotated = _rotated_rectangle((100, 100), (90, 90), 32.0)
    for contour in (axis_aligned, rotated):
        result = classify_contour_shape(contour, config['shape_detection'])
        assert result.shape == 'square'
        assert result.polygon_vertices == 4
        assert result.rotated_aspect_ratio == pytest.approx(1.0, abs=0.03)


def test_axis_aligned_and_rotated_rectangles_are_classified(config):
    horizontal = _contour([(35, 70), (165, 70), (165, 130), (35, 130)])
    rotated = _rotated_rectangle((100, 100), (120, 60), 28.0)
    for contour in (horizontal, rotated):
        result = classify_contour_shape(contour, config['shape_detection'])
        assert result.shape == 'rectangle'
        assert 1.2 <= result.rotated_aspect_ratio <= 2.5


def _circle_contour(radius=55, jagged=False):
    """以合成轮廓验证规则，不依赖外部图片或相机。"""
    angles = np.linspace(0.0, 2.0 * np.pi, 72, endpoint=False)
    radii = np.full_like(angles, radius, dtype=np.float64)
    if jagged:
        radii[::6] -= 3.0
    points = np.column_stack((100.0 + radii * np.cos(angles),
                              100.0 + radii * np.sin(angles)))
    return _contour(np.round(points))


@pytest.mark.parametrize('jagged', [False, True])
def test_circles_and_slightly_jagged_circles_are_classified(config, jagged):
    result = classify_contour_shape(_circle_contour(jagged=jagged), config['shape_detection'])
    assert result.shape == 'circle'
    assert result.polygon_vertices >= config['shape_detection']['circle_min_vertices']


def test_elongated_and_irregular_contours(config):
    elongated = _contour([(20, 80), (180, 80), (180, 120), (20, 120)])
    result = classify_contour_shape(elongated, config['shape_detection'])
    assert result.shape == 'elongated'
    irregular = _contour([
        (30, 30), (105, 65), (170, 30), (140, 100), (170, 170),
        (100, 135), (30, 170), (60, 100),
    ])
    assert classify_contour_shape(irregular, config['shape_detection']).shape == 'unknown'


@pytest.mark.parametrize(
    'contour', [
        None,
        np.empty((0, 1, 2), dtype=np.int32),
        _contour([(10, 10)]),
        _contour([(10, 10), (20, 20)]),
    ])
def test_empty_and_degenerate_contours_are_safe(config, contour):
    result = classify_contour_shape(contour, config['shape_detection'])
    assert result.shape == 'unknown'
    assert result.confidence == 0.0
    assert result.rotated_aspect_ratio == 0.0


def test_shape_config_validates_nan_and_disabled_mode(config, camera_info):
    config['shape_detection']['circle_min_circularity'] = float('nan')
    with pytest.raises(ValueError):
        ColorObjectDetectorCore(config)
    config['shape_detection']['circle_min_circularity'] = 0.78
    config['shape_detection']['enabled'] = False
    image, depth = _scene((0, 0, 255))
    candidate = ColorObjectDetectorCore(config).detect(
        image, depth, '16UC1', camera_info, ['red'])
    assert candidate.detected
    assert candidate.shape == 'unknown'
    assert candidate.shape_confidence == 0.0


def test_unknown_shape_does_not_change_confirmation_or_ready_inputs():
    candidate = DetectionCandidate(
        detected=True, color='red', shape='unknown', shape_confidence=0.0,
        depth_m=0.5, position_camera=(0.0, 0.0, 0.5))
    tracker = ConfirmationTracker(1, 2, 0.08, 0.05)
    assert tracker.update(candidate, 'red')


def test_shape_outputs_are_bounded_and_finite(config):
    contours = [_circle_contour(), _rotated_rectangle((100, 100), (120, 60), 0.0)]
    for contour in contours:
        result = classify_contour_shape(contour, config['shape_detection'])
        assert 0.0 <= result.confidence <= 1.0
        assert np.isfinite(result.rotated_aspect_ratio)
        assert result.rotated_aspect_ratio >= 1.0
