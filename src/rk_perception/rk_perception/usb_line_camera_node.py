#!/usr/bin/env python3
"""Sonix UVC 巡线摄像头的无 cv_bridge ROS Image 发布节点。"""

import array
import time

import cv2
import numpy as np

try:
    import rclpy
    from builtin_interfaces.msg import Time
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import (
        DurabilityPolicy,
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
    )
    from sensor_msgs.msg import Image
except ImportError:
    rclpy = None
    Time = None
    ExternalShutdownException = Exception
    Node = object
    Image = None


DEFAULT_DEVICE = (
    '/dev/v4l/by-id/'
    'usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera_SN0001-video-index0'
)
DEFAULT_IMAGE_TOPIC = '/camera/color/image_raw'
DEFAULT_FRAME_ID = 'line_camera_optical_frame'
NANOSECONDS_PER_SECOND = 1_000_000_000


def decode_fourcc(value):
    """将驱动回读的 FOURCC 数值转换为四字符，便于诊断实际格式。"""
    number = int(value)
    return ''.join(chr((number >> (8 * index)) & 0xFF) for index in range(4))


def normalize_bgr_frame(frame):
    """仅接受连续的 uint8 三通道帧，避免错误布局被发布为有效图像。"""
    if not isinstance(frame, np.ndarray):
        raise ValueError('frame must be a numpy.ndarray')
    if frame.ndim != 3 or frame.shape[2] != 3:
        raise ValueError(f'expected HxWx3 frame, got shape={frame.shape}')
    if frame.dtype != np.uint8:
        raise ValueError(f'expected uint8 frame, got dtype={frame.dtype}')
    if not frame.flags.c_contiguous:
        return np.ascontiguousarray(frame)
    return frame


def stamp_to_nanoseconds(stamp):
    """将 ROS 时间消息转为整数纳秒，用于保证发布序列严格递增。"""
    return (
        int(stamp.sec) * NANOSECONDS_PER_SECOND
        + int(stamp.nanosec)
    )


def strictly_increasing_stamp(stamp, previous_nanoseconds):
    """时钟重复或回退时前移1ns，保障图像流时序的单调性。"""
    nanoseconds = max(
        stamp_to_nanoseconds(stamp),
        int(previous_nanoseconds) + 1,
    )
    adjusted = Time()
    adjusted.sec = nanoseconds // NANOSECONDS_PER_SECOND
    adjusted.nanosec = nanoseconds % NANOSECONDS_PER_SECOND
    return adjusted, nanoseconds


def make_bgr8_image(frame, stamp, frame_id):
    """从已验证的连续 BGR 帧直接构造 Image，不经过 cv_bridge。"""
    frame = normalize_bgr_frame(frame)
    height, width, _ = frame.shape
    message = Image()
    message.header.stamp = stamp
    message.header.frame_id = str(frame_id)
    message.height = height
    message.width = width
    message.encoding = 'bgr8'
    message.is_bigendian = 0
    message.step = width * 3
    # Foxy 的 bytes setter 会逐字节校验；array 保持同一字节流并避免耗时。
    message.data = array.array('B', frame.tobytes())
    return message


def make_sensor_data_qos():
    """图像流使用尽力而为、易过期的 QoS，避免慢订阅者积压旧画面。"""
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=5,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


class UsbLineCameraNode(Node):
    """以稳定 by-id 路径采集新巡线摄像头，且只发布 Image 数据。"""

    def __init__(self):
        super().__init__('usb_line_camera_node')
        self._declare_parameters()
        self._load_parameters()
        self.capture = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        self._camera_released = False
        if not self.capture.isOpened():
            raise RuntimeError(f'V4L2_OPEN_FAILED:{self.device}')
        self._configure_capture()
        self.publisher = self.create_publisher(
            Image, self.image_topic, make_sensor_data_qos()
        )
        self.consecutive_read_failures = 0
        self.last_success_monotonic = time.monotonic()
        self.last_stamp_nanoseconds = -1
        self.stall_reported = False
        self.published_frames = 0
        self.exit_code = 0
        self.stop_requested = False
        self.timer = self.create_timer(1.0 / self.fps, self._on_timer)

    def _declare_parameters(self):
        """声明全部可覆盖的采集参数，默认值对应已验证 Sonix 配置。"""
        self.declare_parameter('device', DEFAULT_DEVICE)
        self.declare_parameter('image_topic', DEFAULT_IMAGE_TOPIC)
        self.declare_parameter('frame_id', DEFAULT_FRAME_ID)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 15.0)
        self.declare_parameter('fourcc', 'MJPG')
        self.declare_parameter('buffer_size', 1)
        self.declare_parameter('read_failure_limit', 10)
        self.declare_parameter('frame_stall_timeout_sec', 1.0)

    def _load_parameters(self):
        """读取并校验参数，错误值在打开硬件前失败，避免不受控的运行状态。"""
        self.device = str(self.get_parameter('device').value)
        self.image_topic = str(self.get_parameter('image_topic').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.fps = float(self.get_parameter('fps').value)
        self.fourcc = str(self.get_parameter('fourcc').value)
        self.buffer_size = int(self.get_parameter('buffer_size').value)
        self.read_failure_limit = int(
            self.get_parameter('read_failure_limit').value
        )
        self.frame_stall_timeout_sec = float(
            self.get_parameter('frame_stall_timeout_sec').value
        )
        if not self.device or not self.image_topic or not self.frame_id:
            raise ValueError('device, image_topic and frame_id must be non-empty')
        if self.width <= 0 or self.height <= 0 or self.fps <= 0.0:
            raise ValueError('width, height and fps must be positive')
        if len(self.fourcc) != 4:
            raise ValueError('fourcc must contain exactly four characters')
        if self.buffer_size <= 0 or self.read_failure_limit <= 0:
            raise ValueError('buffer_size and read_failure_limit must be positive')
        if self.frame_stall_timeout_sec <= 0.0:
            raise ValueError('frame_stall_timeout_sec must be positive')

    def _configure_capture(self):
        """显式请求并回读 V4L2 参数，记录驱动接受的真实配置。"""
        self.capture.set(
            cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.fourcc)
        )
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.capture.set(cv2.CAP_PROP_FPS, self.fps)
        self.capture.set(cv2.CAP_PROP_BUFFERSIZE, self.buffer_size)
        self.get_logger().info(
            'CAMERA_CONFIG '
            f'device={self.device} '
            f'backend={self.capture.getBackendName()} '
            f'fourcc={decode_fourcc(self.capture.get(cv2.CAP_PROP_FOURCC))} '
            f'width={int(self.capture.get(cv2.CAP_PROP_FRAME_WIDTH))} '
            f'height={int(self.capture.get(cv2.CAP_PROP_FRAME_HEIGHT))} '
            f'fps={self.capture.get(cv2.CAP_PROP_FPS):.3f} '
            f'buffersize={self.capture.get(cv2.CAP_PROP_BUFFERSIZE):.0f}'
        )

    def _on_timer(self):
        """只发布本次读取成功的新帧，读失败不会重发旧帧。"""
        try:
            success, frame = self.capture.read()
        except cv2.error as error:
            self.get_logger().error(f'CAMERA_READ_ERROR:{error}')
            success, frame = False, None
        now = time.monotonic()
        if not success or frame is None:
            self._record_read_failure(now)
            return

        try:
            frame = normalize_bgr_frame(frame)
        except ValueError as error:
            self.get_logger().error(f'INVALID_FRAME_LAYOUT:{error}')
            self.exit_code = 3
            self.stop_requested = True
            return

        self.consecutive_read_failures = 0
        self.last_success_monotonic = now
        self.stall_reported = False
        stamp, self.last_stamp_nanoseconds = strictly_increasing_stamp(
            self.get_clock().now().to_msg(),
            self.last_stamp_nanoseconds,
        )
        self.publisher.publish(make_bgr8_image(frame, stamp, self.frame_id))
        self.published_frames += 1
        if self.published_frames % 300 == 0:
            self.get_logger().info(
                f'PUBLISHED_FRAMES count={self.published_frames}'
            )

    def _record_read_failure(self, now):
        """分类停帧与连续失败；达到安全限制后请求节点非零退出。"""
        self.consecutive_read_failures += 1
        if (
            not self.stall_reported
            and now - self.last_success_monotonic
            > self.frame_stall_timeout_sec
        ):
            self.get_logger().error('CAMERA_FRAME_STALL')
            self.stall_reported = True
        if self.consecutive_read_failures >= self.read_failure_limit:
            self.get_logger().error('CAMERA_READ_FAILURE_LIMIT')
            self.exit_code = 2
            self.stop_requested = True

    def close(self):
        """幂等释放摄像头；此路径只回收资源，不发布控制消息。"""
        if self._camera_released:
            return
        self._camera_released = True
        if self.capture is not None and self.capture.isOpened():
            self.capture.release()
        self.get_logger().info(
            f'CAMERA_RELEASED published={self.published_frames}'
        )


def main():
    """运行节点直到 ROS 关闭或摄像头安全失败，随后始终释放设备。"""
    if rclpy is None:
        raise RuntimeError('rclpy and sensor_msgs are required')
    rclpy.init()
    node = None
    exit_code = 0
    try:
        node = UsbLineCameraNode()
        while rclpy.ok() and not node.stop_requested:
            rclpy.spin_once(node, timeout_sec=0.25)
        exit_code = node.exit_code
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as error:
        print(f'CAMERA_NODE_FATAL:{error}', flush=True)
        exit_code = 1
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == '__main__':
    main()
