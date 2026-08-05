import array
import inspect
from pathlib import Path

from builtin_interfaces.msg import Time
import numpy as np
import pytest
from rclpy.qos import DurabilityPolicy, HistoryPolicy, ReliabilityPolicy

from rk_perception.usb_line_camera_node import (
    UsbLineCameraNode,
    make_bgr8_image,
    make_sensor_data_qos,
    normalize_bgr_frame,
    strictly_increasing_stamp,
)


def make_stamp(seconds, nanoseconds):
    stamp = Time()
    stamp.sec = seconds
    stamp.nanosec = nanoseconds
    return stamp


def test_bgr8_message_uses_actual_frame_layout():
    frame = np.full((4, 5, 3), 7, dtype=np.uint8)
    message = make_bgr8_image(
        frame, make_stamp(2, 3), 'line_camera_optical_frame'
    )

    assert message.height == 4
    assert message.width == 5
    assert message.encoding == 'bgr8'
    assert message.step == 15
    assert len(message.data) == message.height * message.step
    assert isinstance(message.data, array.array)
    assert message.header.frame_id == 'line_camera_optical_frame'


def test_non_contiguous_frame_is_normalized_before_serialization():
    frame = np.zeros((4, 10, 3), dtype=np.uint8)[:, ::2, :]

    normalized = normalize_bgr_frame(frame)

    assert normalized.flags.c_contiguous
    assert normalized.shape == (4, 5, 3)


@pytest.mark.parametrize('frame', [
    np.zeros((4, 5), dtype=np.uint8),
    np.zeros((4, 5, 4), dtype=np.uint8),
    np.zeros((4, 5, 3), dtype=np.float32),
])
def test_invalid_frames_are_rejected(frame):
    with pytest.raises(ValueError):
        normalize_bgr_frame(frame)


def test_timestamp_is_strictly_increasing_when_clock_repeats():
    first, first_ns = strictly_increasing_stamp(make_stamp(3, 5), -1)
    second, second_ns = strictly_increasing_stamp(make_stamp(3, 5), first_ns)

    assert first.sec == 3
    assert second_ns == first_ns + 1
    assert second.nanosec == first.nanosec + 1


def test_sensor_data_qos_drops_stale_images_instead_of_backlogging():
    qos = make_sensor_data_qos()

    assert qos.history == HistoryPolicy.KEEP_LAST
    assert qos.depth == 5
    assert qos.reliability == ReliabilityPolicy.BEST_EFFORT
    assert qos.durability == DurabilityPolicy.VOLATILE


class FakeLogger:
    def __init__(self):
        self.errors = []
        self.infos = []

    def error(self, message):
        self.errors.append(message)

    def info(self, message):
        self.infos.append(message)


def test_read_failure_limit_and_stall_are_classified():
    logger = FakeLogger()
    node = UsbLineCameraNode.__new__(UsbLineCameraNode)
    node.consecutive_read_failures = 0
    node.last_success_monotonic = 0.0
    node.frame_stall_timeout_sec = 1.0
    node.stall_reported = False
    node.read_failure_limit = 3
    node.exit_code = 0
    node.stop_requested = False
    node.get_logger = lambda: logger

    node._record_read_failure(1.1)
    node._record_read_failure(1.2)
    node._record_read_failure(1.3)

    assert logger.errors.count('CAMERA_FRAME_STALL') == 1
    assert logger.errors[-1] == 'CAMERA_READ_FAILURE_LIMIT'
    assert node.exit_code == 2
    assert node.stop_requested is True


class FakeCapture:
    def __init__(self):
        self.release_calls = 0

    def isOpened(self):
        return True

    def release(self):
        self.release_calls += 1


def test_close_releases_camera_once_when_called_repeatedly():
    logger = FakeLogger()
    capture = FakeCapture()
    node = UsbLineCameraNode.__new__(UsbLineCameraNode)
    node._camera_released = False
    node.capture = capture
    node.published_frames = 0
    node.get_logger = lambda: logger

    node.close()
    node.close()

    assert capture.release_calls == 1
    assert logger.infos == ['CAMERA_RELEASED published=0']


def test_camera_node_has_no_cv_bridge_or_control_interfaces():
    source = Path(inspect.getsourcefile(UsbLineCameraNode)).read_text(
        encoding='utf-8'
    )

    assert 'import cv_bridge' not in source
    assert 'from cv_bridge import' not in source
    assert 'cmd_vel' not in source
    assert 'Sport' not in source
