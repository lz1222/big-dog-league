from dataclasses import replace

import cv2
import numpy as np
from sensor_msgs.msg import Image

from rk_perception.real_line_tracker_node import (
    LineTrackerConfig,
    RealLineTrackerNode,
    detect_blue_stop_zone,
    detect_line_in_image,
    detect_red_circle,
    detect_white_bar,
)


class _RecordedPublisher:
    """最小 publisher 替身：只验证回调是否持续产生主输出。"""

    def __init__(self, failure=None):
        self.messages = []
        self.failure = failure

    def publish(self, message):
        if self.failure is not None:
            raise self.failure
        self.messages.append(message)


class _RecordedLogger:
    def __init__(self):
        self.warnings = []

    def warn(self, message):
        self.warnings.append(message)


class _ImageBridge:
    def __init__(self, image):
        self.image = image

    def imgmsg_to_cv2(self, _message, desired_encoding):
        assert desired_encoding == 'bgr8'
        return self.image.copy()


class _CallbackHarness:
    """不初始化 ROS 节点，隔离验证 image_callback 的主输出契约。"""

    image_callback = RealLineTrackerNode.image_callback
    publish_debug_images_safely = RealLineTrackerNode.publish_debug_images_safely

    def __init__(self, image, debug_enabled=True, debug_failure=None,
                 forced_result=None):
        self.bridge = _ImageBridge(image)
        self.tracker_config = default_config()
        self.max_track_jump_fraction = 0.30
        self.publisher = _RecordedPublisher()
        self.enable_debug_image = debug_enabled
        self.debug_failure = debug_failure
        self.forced_result = forced_result
        self.logger = _RecordedLogger()
        self.debug_calls = 0

    def refresh_parameters(self):
        pass

    def robot_center_x(self, image_width):
        return float(image_width) / 2.0

    def preferred_center_x(self, _image_width, robot_center_x):
        return robot_center_x

    def apply_route_lock(self, result, _image_width, _preferred_center_x):
        return self.forced_result if self.forced_result is not None else result

    def make_line_track_msg(self, _image_msg, lateral, heading, confidence,
                            visible):
        return {
            'lateral_error': lateral,
            'heading_error': heading,
            'confidence': confidence,
            'line_visible': visible,
        }

    def publish_special_detections(self, *_args):
        pass

    def publish_debug_images(self, *_args, **_kwargs):
        self.debug_calls += 1
        if self.debug_failure is not None:
            raise self.debug_failure
        return {
            'enabled': self.enable_debug_image,
            'mask_published': self.enable_debug_image,
            'overlay_published': self.enable_debug_image,
            'mask_shape': '1x1',
            'overlay_shape': '1x1x3',
            'stage': 'debug_publish',
        }

    def log_debug(self, *_args, **_kwargs):
        pass

    def get_logger(self):
        return self.logger


def make_image_message():
    message = Image()
    message.header.frame_id = 'line_camera_optical_frame'
    return message


def run_callback(image, **kwargs):
    harness = _CallbackHarness(image, **kwargs)
    harness.image_callback(make_image_message())
    assert len(harness.publisher.messages) == 1
    return harness.publisher.messages[-1], harness


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


def test_debug_image_messages_use_explicit_mono8_and_bgr8_layouts():
    source = make_image_message()
    mask = np.zeros((12, 20), dtype=np.uint8)
    overlay = np.zeros((12, 20, 3), dtype=np.uint8)

    mask_msg = RealLineTrackerNode.make_debug_image_msg(
        mask, 'mono8', source
    )
    overlay_msg = RealLineTrackerNode.make_debug_image_msg(
        overlay, 'bgr8', source
    )

    assert mask_msg.encoding == 'mono8'
    assert (mask_msg.height, mask_msg.width, mask_msg.step) == (12, 20, 20)
    assert len(mask_msg.data) == 12 * 20
    assert overlay_msg.encoding == 'bgr8'
    assert (overlay_msg.height, overlay_msg.width, overlay_msg.step) == (
        12, 20, 60
    )
    assert len(overlay_msg.data) == 12 * 20 * 3
    assert mask_msg.header.frame_id == source.header.frame_id
    assert overlay_msg.header.frame_id == source.header.frame_id


def test_debug_exception_does_not_block_primary_line_track_publish():
    message, harness = run_callback(
        make_image(), debug_failure=RuntimeError('debug publisher unavailable')
    )

    assert message['line_visible'] is False
    assert harness.debug_calls == 1
    assert harness.logger.warnings


def test_no_line_callback_still_publishes_line_track_fallback():
    message, _ = run_callback(make_image())

    assert message['line_visible'] is False
    assert message['confidence'] == 0.0


def test_too_wide_callback_still_publishes_line_track_fallback():
    image = make_image()
    cv2.rectangle(image, (140, 245), (500, 479), (0, 0, 0), -1)
    assert detect_line_in_image(image, default_config()).reason == 'too_wide'

    message, _ = run_callback(image)

    assert message['line_visible'] is False


def test_too_dark_callback_still_publishes_line_track_fallback():
    image = make_image(color=0)
    assert detect_line_in_image(image, default_config()).reason == 'too_dark'

    message, _ = run_callback(image)

    assert message['line_visible'] is False


def test_reacquire_rejected_callback_still_publishes_line_track_fallback():
    image = make_image()
    rejected = replace(
        detect_line_in_image(image, default_config()),
        line_visible=False,
        confidence=0.0,
        reason='reacquire_jump_rejected',
        candidate_rejected=True,
        candidate_rejection_reason='reacquire_jump>64.0px',
        track_jump_rejected=True,
    )

    message, _ = run_callback(image, forced_result=rejected)

    assert message['line_visible'] is False
    assert message['confidence'] == 0.0


def test_debug_disabled_does_not_change_primary_line_track_publish():
    message, harness = run_callback(make_image(), debug_enabled=False)

    assert message['line_visible'] is False
    assert harness.debug_calls == 1


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
