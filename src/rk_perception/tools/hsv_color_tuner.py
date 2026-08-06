#!/usr/bin/env python3
"""D435i HSV 现场标定工具；仅显示和保存候选参数，不控制机器人或机械臂。"""

from __future__ import print_function

import copy
import os
import sys

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
import yaml

# 允许直接执行 tools/ 下脚本，同时保持已安装包的导入方式不变。
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from rk_perception.color_object_detector_core import ColorObjectDetectorCore  # noqa: E402


class HsvColorTuner(Node):
    """用六个滑块编辑当前 HSV 区间；红色可按 r 键切换两个区间。"""

    def __init__(self):
        super().__init__('hsv_color_tuner')
        self.config_file = self.declare_parameter('config_file', '').value
        self.color_topic = self.declare_parameter(
            'color_topic', '/camera/camera/color/image_raw').value
        self.depth_topic = self.declare_parameter(
            'depth_topic',
            '/camera/camera/aligned_depth_to_color/image_raw').value
        self.color = self.declare_parameter('color', 'red').value.lower()
        self.output_file = self.declare_parameter(
            'output_file', 'color_object_hsv_calibration.yaml').value
        self.enable_gui = self.declare_parameter('enable_gui', True).value
        if not self.config_file or not os.path.isfile(self.config_file):
            raise ValueError('config_file must reference color_object_detector.yaml')
        with open(self.config_file, 'r') as stream:
            document = yaml.safe_load(stream)
        self.config = document['color_object_detector']['ros__parameters']
        if self.color not in self.config['colors']:
            raise ValueError('unknown color: {0}'.format(self.color))
        self.range_index = 0
        self.latest_depth = None
        self.latest_depth_encoding = ''
        self.bridge = CvBridge()
        self.window_name = 'HSV Color Tuner'
        self.subscription = self.create_subscription(
            Image, self.color_topic, self._on_image, qos_profile_sensor_data)
        self.depth_subscription = self.create_subscription(
            Image, self.depth_topic, self._on_depth, qos_profile_sensor_data)
        if self.enable_gui:
            self._create_trackbars()
        else:
            self.get_logger().info('GUI disabled; tuner does not open windows.')

    def _on_depth(self, message):
        """缓存最近深度仅供标定显示，不向正式检测链注入未同步数据。"""
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(message, 'passthrough')
            self.latest_depth_encoding = message.encoding
        except Exception as exc:
            self.get_logger().warn('Cannot display depth: {0}'.format(exc))

    def _create_trackbars(self):
        """创建当前区间的 H/S/V 上下界六个滑块。"""
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        for name, maximum in (('H low', 179), ('S low', 255), ('V low', 255),
                              ('H high', 179), ('S high', 255), ('V high', 255)):
            cv2.createTrackbar(name, self.window_name, 0, maximum, lambda _value: None)
        self._set_trackbars_from_config()

    def _set_trackbars_from_config(self):
        current = self.config['colors'][self.color]['hsv_ranges'][self.range_index]
        for name, value in zip(('H low', 'S low', 'V low'), current['lower']):
            cv2.setTrackbarPos(name, self.window_name, int(value))
        for name, value in zip(('H high', 'S high', 'V high'), current['upper']):
            cv2.setTrackbarPos(name, self.window_name, int(value))

    def _read_trackbars(self):
        current = self.config['colors'][self.color]['hsv_ranges'][self.range_index]
        current['lower'] = [cv2.getTrackbarPos(name, self.window_name)
                            for name in ('H low', 'S low', 'V low')]
        current['upper'] = [cv2.getTrackbarPos(name, self.window_name)
                            for name in ('H high', 'S high', 'V high')]

    def _on_image(self, message):
        """叠加 mask 与形状指标；标定工具不发布任何控制 topic。"""
        if not self.enable_gui:
            return
        image = self.bridge.imgmsg_to_cv2(message, 'bgr8')
        self._read_trackbars()
        core = ColorObjectDetectorCore(self.config)
        mask = core.build_color_mask(image, self.color)
        contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        display = image.copy()
        for contour in contours:
            area = cv2.contourArea(contour)
            x, y, width, height = cv2.boundingRect(contour)
            perimeter = cv2.arcLength(contour, True)
            circularity = 4.0 * 3.1415926 * area / (perimeter * perimeter) if perimeter else 0.0
            hull_area = cv2.contourArea(cv2.convexHull(contour))
            solidity = area / hull_area if hull_area else 0.0
            center_x, center_y = x + width // 2, y + height // 2
            depth_text = self._contour_depth_text(contour, image.shape[:2], core)
            cv2.rectangle(display, (x, y), (x + width, y + height), (0, 255, 255), 2)
            label = 'area={0:.0f} circ={1:.2f} sol={2:.2f}'.format(
                area, circularity, solidity)
            cv2.putText(display, label, (x, max(18, y - 22)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
            cv2.putText(display, 'center=({0},{1}) {2}'.format(
                center_x, center_y, depth_text), (x, max(18, y - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
        combined = cv2.hconcat([display, cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)])
        cv2.imshow(self.window_name, combined)
        key = cv2.waitKey(1) & 0xff
        ranges = self.config['colors'][self.color]['hsv_ranges']
        if key == ord('r') and len(ranges) > 1:
            self.range_index = (self.range_index + 1) % len(ranges)
            self._set_trackbars_from_config()
        elif key == ord('s'):
            self._save_calibration()

    def _contour_depth_text(self, contour, image_shape, core):
        """显示当前轮廓内的深度中位数；尺寸不一致时明确标记不可用。"""
        if self.latest_depth is None or self.latest_depth.shape[:2] != image_shape:
            return 'depth=n/a'
        try:
            depth_m = core.depth_to_meters(
                self.latest_depth, self.latest_depth_encoding)
        except ValueError:
            return 'depth=n/a'
        mask = np.zeros(image_shape, dtype=np.uint8)
        cv2.drawContours(mask, [contour], -1, 255, thickness=-1)
        values = depth_m[mask > 0]
        values = values[np.isfinite(values) & (values > 0.0)]
        if not len(values):
            return 'depth=n/a'
        return 'depth={0:.3f}m'.format(float(np.median(values)))

    def _save_calibration(self):
        """写入单独的标定文件，绝不覆盖比赛正式 YAML。"""
        output = {'colors': {self.color: copy.deepcopy(self.config['colors'][self.color])}}
        with open(self.output_file, 'w') as stream:
            yaml.safe_dump(output, stream, default_flow_style=False)
        self.get_logger().info('Saved calibration candidate to {0}'.format(self.output_file))


def main(args=None):
    """控制台入口；关闭窗口后释放 GUI 资源。"""
    rclpy.init(args=args)
    node = HsvColorTuner()
    try:
        rclpy.spin(node)
    finally:
        if node.enable_gui:
            cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
