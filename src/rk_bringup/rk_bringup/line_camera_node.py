#!/usr/bin/env python3
"""比赛 USB 巡线相机的单设备 ROS 图像源。

该节点只打开操作者明确指定的 ``/dev/videoN`` 索引，绝不扫描或回退到
另一台摄像头，避免把机械臂 D435i 或无关 UVC 设备误接入巡线控制链。
"""

from collections import deque
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String


LINE_IMAGE_TOPIC = '/line_camera/image_raw'
LINE_STATUS_TOPIC = '/line_camera/status'
LINE_FRAME_ID = 'line_camera_optical_frame'
LINE_CAMERA_BY_ID = (
    '/dev/v4l/by-id/'
    'usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera_SN0001-video-index0'
)


class LineCameraNode(Node):
    """将一个显式 USB UVC 设备稳定发布到固定的比赛巡线 topic。"""

    def __init__(self):
        super().__init__('line_camera_node')
        self.declare_parameter('device', LINE_CAMERA_BY_ID)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 15.0)
        self.declare_parameter('failure_log_period_sec', 2.0)
        self.declare_parameter('performance_stats_enabled', False)

        self.device = self._explicit_device_path('device')
        self.width = self._positive_int('width')
        self.height = self._positive_int('height')
        self.fps = self._positive_float('fps')
        self.failure_log_period_sec = self._positive_float(
            'failure_log_period_sec'
        )
        self.performance_stats_enabled = self._bool_parameter(
            'performance_stats_enabled'
        )
        # 图像是最新帧优先的传感器数据；best-effort 避免可靠队列拥塞把采集
        # 定时器拖慢，巡线订阅端同样使用 sensor-data QoS。
        self.image_publisher = self.create_publisher(
            Image, LINE_IMAGE_TOPIC, qos_profile_sensor_data
        )
        self.status_publisher = self.create_publisher(
            String, LINE_STATUS_TOPIC, 10
        )
        self._last_failure_log = 0.0
        self._consecutive_failures = 0
        self._last_capture_callback_time = None
        self._last_stats_log_time = time.monotonic()
        self._performance_samples = {
            'capture_read_ms': deque(maxlen=200),
            'image_wrap_ms': deque(maxlen=200),
            'publish_call_ms': deque(maxlen=200),
            'loop_period_ms': deque(maxlen=200),
        }

        # device 是唯一选取条件；强制 V4L2，打开失败时退出而不是枚举其它设备。
        self.capture = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        if not self.capture.isOpened():
            self._publish_status(
                'ERROR', 'open_failed device={}'.format(self.device)
            )
            self.get_logger().fatal(
                '无法打开指定巡线相机 {}；不会尝试其它设备。'.format(self.device)
            )
            self.capture.release()
            raise RuntimeError('line camera open failed')

        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, float(self.width))
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, float(self.height))
        self.capture.set(cv2.CAP_PROP_FPS, float(self.fps))
        self.capture.set(cv2.CAP_PROP_BUFFERSIZE, 1.0)
        self._log_capture_configuration()
        self._publish_status(
            'READY',
            'device={} requested={}x{}@{:.2f}'.format(
                self.device, self.width, self.height, self.fps
            ),
        )
        self.timer = self.create_timer(1.0 / self.fps, self._capture_once)

    def destroy_node(self):
        """关闭本节点打开的唯一设备，不影响其它相机进程。"""
        if hasattr(self, 'capture'):
            self.capture.release()
        return super().destroy_node()

    def _capture_once(self):
        """读取一帧；失效时持续暴露 ERROR，不重选或切换设备。"""
        loop_start = time.monotonic()
        if self._last_capture_callback_time is not None:
            self._record_performance(
                'loop_period_ms',
                (loop_start - self._last_capture_callback_time) * 1000.0,
            )
        self._last_capture_callback_time = loop_start

        capture_start = time.monotonic()
        ok, frame = self.capture.read()
        self._record_performance(
            'capture_read_ms', (time.monotonic() - capture_start) * 1000.0
        )
        if not ok or frame is None:
            self._consecutive_failures += 1
            now = time.monotonic()
            detail = 'read_failed device={} consecutive_failures={}'.format(
                self.device, self._consecutive_failures
            )
            self._publish_status('ERROR', detail)
            if now - self._last_failure_log >= self.failure_log_period_sec:
                self.get_logger().error(
                    'USB 巡线相机读帧失败：{}；不会切换到其它设备。'.format(
                        detail
                    )
                )
                self._last_failure_log = now
            self._maybe_log_performance(loop_start)
            return

        self._consecutive_failures = 0
        if frame.ndim != 3 or frame.shape[2] != 3:
            self._report_unsupported_frame(frame)
            self._maybe_log_performance(loop_start)
            return

        # 本机 OpenCV 与 Foxy cv_bridge 的 CV 类型常量版本不一致；直接按
        # sensor_msgs/Image 的 BGR8 布局封包，避免相机源因桥接库崩溃退出。
        wrap_start = time.monotonic()
        frame = np.ascontiguousarray(frame)
        message = Image()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = LINE_FRAME_ID
        message.height = int(frame.shape[0])
        message.width = int(frame.shape[1])
        message.encoding = 'bgr8'
        message.is_bigendian = False
        message.step = int(frame.strides[0])
        message.data = frame.tobytes()
        self._record_performance(
            'image_wrap_ms', (time.monotonic() - wrap_start) * 1000.0
        )
        publish_start = time.monotonic()
        self.image_publisher.publish(message)
        self._record_performance(
            'publish_call_ms', (time.monotonic() - publish_start) * 1000.0
        )
        self._publish_status(
            'STREAMING',
            'device={} {}x{} bgr8'.format(
                self.device, message.width, message.height
            ),
        )
        self._maybe_log_performance(loop_start)

    def _report_unsupported_frame(self, frame):
        """暴露非 BGR 三通道输入，禁止猜测格式后继续发布错误图像。"""
        self._consecutive_failures += 1
        detail = 'unsupported_frame shape={} dtype={}'.format(
            getattr(frame, 'shape', None), getattr(frame, 'dtype', None)
        )
        self._publish_status('ERROR', detail)
        self.get_logger().error('USB 巡线相机帧格式不受支持：{}'.format(detail))

    def _publish_status(self, state, detail):
        message = String()
        message.data = 'state={} {}'.format(state, detail)
        self.status_publisher.publish(message)

    def _explicit_device_path(self, name):
        """仅接受显式 video index 或 /dev 路径，禁止从候选设备中自动选择。"""
        value = self.get_parameter(name).value
        device = str(value).strip()
        if device.isdigit():
            return '/dev/video{}'.format(int(device))
        if device.startswith('/dev/'):
            return device
        raise ValueError('{} must be an explicit /dev path or video index'.format(name))

    def _bool_parameter(self, name):
        value = self.get_parameter(name).value
        if not isinstance(value, bool):
            raise ValueError('{} must be a bool'.format(name))
        return value

    def _positive_int(self, name):
        value = self.get_parameter(name).value
        if not isinstance(value, int) or value <= 0:
            raise ValueError('{} must be a positive integer'.format(name))
        return value

    def _positive_float(self, name):
        value = float(self.get_parameter(name).value)
        if value <= 0.0:
            raise ValueError('{} must be > 0'.format(name))
        return value

    def _log_capture_configuration(self):
        """记录实际 V4L2 回读值，便于真机验收发现驱动忽略请求参数。"""
        fourcc = int(self.capture.get(cv2.CAP_PROP_FOURCC))
        fourcc_text = ''.join(
            chr((fourcc >> (8 * index)) & 0xFF) for index in range(4)
        )
        self.get_logger().info(
            'USB_CAPTURE_CONFIG device={} requested={}x{}@{:.2f} '
            'actual={}x{}@{:.2f} fourcc={} buffer_size={}'.format(
                self.device,
                self.width,
                self.height,
                self.fps,
                self.capture.get(cv2.CAP_PROP_FRAME_WIDTH),
                self.capture.get(cv2.CAP_PROP_FRAME_HEIGHT),
                self.capture.get(cv2.CAP_PROP_FPS),
                fourcc_text,
                self.capture.get(cv2.CAP_PROP_BUFFERSIZE),
            )
        )

    def _record_performance(self, metric, value):
        """收集有限窗口指标；默认关闭日志，避免统计本身影响比赛运行。"""
        if self.performance_stats_enabled:
            self._performance_samples[metric].append(value)

    def _maybe_log_performance(self, now):
        """每五秒输出一次聚合时延，绝不逐帧打印。"""
        if (
            not self.performance_stats_enabled
            or now - self._last_stats_log_time < 5.0
        ):
            return
        fields = []
        for metric, samples in self._performance_samples.items():
            if not samples:
                fields.append('{}=no_samples'.format(metric))
                continue
            ordered = sorted(samples)
            p95_index = min(len(ordered) - 1, int(len(ordered) * 0.95))
            fields.append('{} mean={:.2f}ms p95={:.2f}ms max={:.2f}ms'.format(
                metric,
                sum(samples) / len(samples),
                ordered[p95_index],
                ordered[-1],
            ))
            samples.clear()
        self.get_logger().info('USB_CAMERA_PERF ' + '; '.join(fields))
        self._last_stats_log_time = now


def main(args=None):
    """启动巡线相机；启动失败保留非零退出状态供比赛脚本拒绝放行。"""
    rclpy.init(args=args)
    node = None
    try:
        node = LineCameraNode()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
