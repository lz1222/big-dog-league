import cv2
import numpy as np

from rk_perception.real_line_tracker_node import (
    LineTrackerConfig,
    detect_blue_stop_zone,
    detect_line_in_image,
    detect_red_circle,
    detect_white_bar,
)


def make_image(width=640, height=480, color=220):
    return np.full((height, width, 3), color, dtype=np.uint8)


def draw_line(image, bottom_x, top_x=None, line_width=70):
    if top_x is None:
        top_x = bottom_x

    height = image.shape[0]
    cv2.line(
        image,
        (bottom_x, height - 1),
        (top_x, int(height * 0.45)),
        (0, 0, 0),
        line_width
    )
    return image


def default_config():
    return LineTrackerConfig(
        threshold_value=100,
        max_line_width_fraction=0.35,
        max_dark_fraction=0.35,
        visible_min_confidence=0.30,
    )


def test_center_line_is_visible():
    image = draw_line(make_image(), bottom_x=320)

    result = detect_line_in_image(image, default_config())

    assert result.line_visible is True
    assert result.reason == 'ok'
    assert result.confidence >= 0.30
    assert abs(result.lateral_error) < 0.08


def test_robot_center_offset_changes_lateral_error_reference():
    image = draw_line(make_image(), bottom_x=320)

    result = detect_line_in_image(
        image,
        default_config(),
        robot_center_x=352.0
    )

    assert result.line_visible is True
    assert result.robot_center_x == 352.0
    assert result.lateral_error < -0.05


def test_right_line_has_positive_lateral_error():
    image = draw_line(make_image(), bottom_x=430)

    result = detect_line_in_image(image, default_config())

    assert result.line_visible is True
    assert result.lateral_error > 0.20


def test_slanted_line_has_heading_error():
    image = draw_line(make_image(), bottom_x=420, top_x=280)

    result = detect_line_in_image(image, default_config())

    assert result.line_visible is True
    assert abs(result.heading_error) > 0.15


def test_blank_floor_is_not_visible():
    result = detect_line_in_image(make_image(), default_config())

    assert result.line_visible is False
    assert result.confidence == 0.0


def test_full_black_occlusion_is_not_visible():
    image = make_image(color=0)

    result = detect_line_in_image(image, default_config())

    assert result.line_visible is False
    assert result.reason == 'too_dark'


def test_large_black_block_is_not_visible():
    image = make_image()
    cv2.rectangle(image, (120, 250), (520, 479), (0, 0, 0), -1)

    result = detect_line_in_image(image, default_config())

    assert result.line_visible is False
    assert result.reason in ('too_dark', 'too_wide')


def test_small_noise_is_not_visible():
    image = make_image()
    rng = np.random.default_rng(1)
    for x, y in rng.integers((0, 240), (640, 480), size=(40, 2)):
        cv2.circle(image, (int(x), int(y)), 2, (0, 0, 0), -1)

    result = detect_line_in_image(image, default_config())

    assert result.line_visible is False


def test_red_circle_detection_reports_normalized_geometry():
    image = make_image()
    cv2.circle(image, (320, 300), 55, (0, 0, 255), -1)

    result = detect_red_circle(image)

    assert result.visible is True
    assert result.target_type == 'red_circle'
    assert 0.45 < result.center_x < 0.55
    assert result.area_ratio > 0.002


def test_blue_stop_zone_detects_robot_reference_inside():
    image = make_image()
    cv2.rectangle(image, (80, 300), (560, 479), (255, 0, 0), -1)

    result = detect_blue_stop_zone(
        image,
        robot_center_x=320.0,
        min_area_ratio=0.05
    )

    assert result.visible is True
    assert result.inside_candidate is True
    assert result.target_type == 'blue_stop_zone'


def test_white_horizontal_bar_is_detected_but_white_block_is_rejected():
    image = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.rectangle(image, (120, 300), (520, 335), (255, 255, 255), -1)

    result = detect_white_bar(image)

    assert result.visible is True
    assert result.width_ratio > 0.20
    assert result.height_ratio < 0.15

    square = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.rectangle(square, (250, 200), (390, 340), (255, 255, 255), -1)
    rejected = detect_white_bar(square)

    assert rejected.visible is False
