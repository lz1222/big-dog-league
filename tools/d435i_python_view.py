#!/usr/bin/env python3

"""D435i 的 Python/OpenCV 图像调用验证工具。

本工具只订阅已经由 ``arm_d435i.launch.py`` 发布的 RGB topic，不直接打开
USB 设备，也不会控制机械臂。收到的 ``sensor_msgs/Image`` 被转换为 OpenCV
可直接送入物体识别算法的 BGR ``numpy.ndarray``，并实时显示。

用法：
    source /opt/ros/foxy/setup.bash
    source ~/rk_inspection_ws/install/setup.bash
    python3 tools/d435i_python_view.py

按 ``q`` 或 ``Esc`` 退出；按 ``s`` 将当前画面保存到 ``--save-dir``。
"""

import argparse
from datetime import datetime
from pathlib import Path
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class D435iPythonView(Node):
    """以传感器 QoS 接收 D435i RGB 图像，并保留最新一帧供 GUI 消费。"""

    def __init__(self, topic: str):
        super().__init__('d435i_python_view')
        self._topic = topic
        self._latest_bgr = None
        self._frame_count = 0
        self._fps_window_count = 0
        self._fps_window_start = time.monotonic()
        self._fps = 0.0
        self._unsupported_encoding_reported = None

        # RealSense 图像默认是 sensor-data QoS；保持一致才能可靠接收实时帧。
        self._subscription = self.create_subscription(
            Image, topic, self._on_image, qos_profile_sensor_data)
        self.get_logger().info(
            'Python OpenCV viewer subscribed to {0}; press q/Esc to quit, s to save.'.format(topic))

    @property
    def latest_bgr(self):
        """返回最近一帧 BGR 图像；识别器可在此处接入推理逻辑。"""
        return self._latest_bgr

    @property
    def fps(self):
        """返回最近约两秒窗口内的实际 Python 接收帧率。"""
        return self._fps

    def _on_image(self, message: Image):
        """将 ROS Image 安全复制为 BGR 数组，避免回调返回后消息缓冲区失效。"""
        image = self._image_to_bgr(message)
        if image is None:
            return

        self._latest_bgr = image
        self._frame_count += 1
        self._fps_window_count += 1
        now = time.monotonic()
        elapsed = now - self._fps_window_start
        if elapsed >= 2.0:
            self._fps = self._fps_window_count / elapsed
            self._fps_window_count = 0
            self._fps_window_start = now
            self.get_logger().info(
                'Received {0}x{1} {2}, Python callback FPS: {3:.1f}'.format(
                    image.shape[1], image.shape[0], message.encoding, self._fps))

    def _image_to_bgr(self, message: Image):
        """处理 RGB8/BGR8 常用格式及行对齐，未知格式只报告一次以防刷屏。"""
        encoding = message.encoding.lower()
        channel_counts = {
            'rgb8': 3,
            'bgr8': 3,
            'rgba8': 4,
            'bgra8': 4,
            'mono8': 1,
        }
        channels = channel_counts.get(encoding)
        if channels is None:
            self._report_unsupported_encoding(message.encoding)
            return None

        min_step = message.width * channels
        if message.height <= 0 or message.width <= 0 or message.step < min_step:
            self.get_logger().warn(
                'Invalid image layout: {0}x{1}, step={2}, encoding={3}'.format(
                    message.width, message.height, message.step, message.encoding))
            return None

        required_size = message.height * message.step
        raw = np.frombuffer(message.data, dtype=np.uint8)
        if raw.size < required_size:
            self.get_logger().warn(
                'Incomplete image data: got {0}, expected at least {1} bytes'.format(
                    raw.size, required_size))
            return None

        # step 可能含每行 padding；先按完整行切片，再去掉 padding。
        rows = raw[:required_size].reshape(message.height, message.step)
        pixels = rows[:, :min_step]
        if channels == 1:
            return cv2.cvtColor(pixels.reshape(message.height, message.width),
                                cv2.COLOR_GRAY2BGR)

        pixels = pixels.reshape(message.height, message.width, channels)
        if encoding == 'rgb8':
            return cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR)
        if encoding == 'rgba8':
            return cv2.cvtColor(pixels, cv2.COLOR_RGBA2BGR)
        if encoding == 'bgra8':
            return cv2.cvtColor(pixels, cv2.COLOR_BGRA2BGR)
        return pixels.copy()

    def _report_unsupported_encoding(self, encoding: str):
        """避免异常格式在 15 FPS 回调中反复打印，保留首条排查信息。"""
        if encoding != self._unsupported_encoding_reported:
            self._unsupported_encoding_reported = encoding
            self.get_logger().error(
                'Unsupported encoding {0}; expected rgb8/bgr8/rgba8/bgra8/mono8'.format(encoding))


def parse_args():
    """解析查看器参数；默认话题与机械臂 D435i 启动入口保持一致。"""
    parser = argparse.ArgumentParser(
        description='Verify D435i ROS images in Python/OpenCV before object recognition.')
    parser.add_argument('--topic', default='/arm_camera/color/image_raw',
                        help='ROS Image topic to subscribe to.')
    parser.add_argument('--window', default='D435i Python OpenCV View',
                        help='OpenCV window title.')
    parser.add_argument('--save-dir', default='.',
                        help='Directory used when pressing s (default: current directory).')
    return parser.parse_args()


def main():
    """运行 ROS 回调与 OpenCV 事件循环；无帧时仍保持窗口响应以便安全退出。"""
    args = parse_args()
    save_dir = Path(args.save_dir).expanduser()
    save_dir.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = D435iPythonView(args.topic)
    cv2.namedWindow(args.window, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(args.window, 960, 720)
    last_no_frame_notice = time.monotonic()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            frame = node.latest_bgr
            if frame is not None:
                display = frame.copy()
                # 感知调试 overlay 在左上角写检测字段；查看器自身提示移到底部防重叠。
                height = display.shape[0]
                cv2.putText(display, 'Python RGB callback: {0:.1f} FPS'.format(node.fps),
                            (12, max(30, height - 34)), cv2.FONT_HERSHEY_SIMPLEX, 0.72,
                            (0, 255, 0), 2, cv2.LINE_AA)
                cv2.putText(display, 'q/Esc: quit | s: save image',
                            (12, max(54, height - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                            (0, 255, 255), 1, cv2.LINE_AA)
                cv2.imshow(args.window, display)
            elif time.monotonic() - last_no_frame_notice >= 5.0:
                node.get_logger().warn(
                    'No image received yet. Keep arm_d435i.launch.py running and check topic: {0}'.format(args.topic))
                last_no_frame_notice = time.monotonic()

            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), 27):
                break
            if key == ord('s') and frame is not None:
                output = save_dir / 'd435i_python_{0}.jpg'.format(
                    datetime.now().strftime('%Y%m%d_%H%M%S'))
                if cv2.imwrite(str(output), frame):
                    node.get_logger().info('Saved current RGB frame to {0}'.format(output))
                else:
                    node.get_logger().error('Failed to save image to {0}'.format(output))
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
