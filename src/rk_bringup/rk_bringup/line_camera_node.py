#!/usr/bin/env python3
"""比赛 USB 巡线相机的单设备 ROS 图像源。

该节点只打开操作者明确指定的 ``/dev/videoN`` 索引，绝不扫描或回退到
另一台摄像头，避免把机械臂 D435i 或无关 UVC 设备误接入巡线控制链。
"""

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


class LineCameraNode(Node):
    """将一个显式 USB UVC 设备稳定发布到固定的比赛巡线 topic。"""

    def __init__(self):
        super().__init__('line_camera_node')
        self.declare_parameter('device', 0)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 15.0)
        self.declare_parameter('failure_log_period_sec', 2.0)

        self.device = self._positive_or_zero_int('device')
        self.width = self._positive_int('width')
        self.height = self._positive_int('height')
        self.fps = self._positive_float('fps')
        self.failure_log_period_sec = self._positive_float(
            'failure_log_period_sec'
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

        # device 是唯一选取条件；打开失败时退出，而不是枚举其它设备。
        self.capture = cv2.VideoCapture(self.device)
        if not self.capture.isOpened():
            self._publish_status(
                'ERROR', 'open_failed device=/dev/video{}'.format(self.device)
            )
            self.get_logger().fatal(
                '无法打开指定巡线相机 /dev/video{}；不会尝试其它设备。'.format(
                    self.device
                )
            )
            self.capture.release()
            raise RuntimeError('line camera open failed')

        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, float(self.width))
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, float(self.height))
        self.capture.set(cv2.CAP_PROP_FPS, float(self.fps))
        self._publish_status(
            'READY',
            'device=/dev/video{} requested={}x{}@{:.2f}'.format(
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
        ok, frame = self.capture.read()
        if not ok or frame is None:
            self._consecutive_failures += 1
            now = time.monotonic()
            detail = 'read_failed device=/dev/video{} consecutive_failures={}'.format(
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
            return

        self._consecutive_failures = 0
        if frame.ndim != 3 or frame.shape[2] != 3:
            self._report_unsupported_frame(frame)
            return

        # 本机 OpenCV 与 Foxy cv_bridge 的 CV 类型常量版本不一致；直接按
        # sensor_msgs/Image 的 BGR8 布局封包，避免相机源因桥接库崩溃退出。
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
        self.image_publisher.publish(message)
        self._publish_status(
            'STREAMING',
            'device=/dev/video{} {}x{} bgr8'.format(
                self.device, message.width, message.height
            ),
        )

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

    def _positive_or_zero_int(self, name):
        value = self.get_parameter(name).value
        if not isinstance(value, int) or value < 0:
            raise ValueError('{} must be an integer >= 0'.format(name))
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
