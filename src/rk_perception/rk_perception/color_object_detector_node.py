"""ROS 2 节点：同步 D435i RGB-D，发布颜色目标及相机/机械臂基座坐标。"""

from __future__ import division

import math
import os
import json

import cv2
import numpy as np
import message_filters
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformListener
import yaml
from ament_index_python.packages import get_package_share_directory

from rk_interfaces.msg import ColorObjectDetection
from rk_perception.color_object_detector_core import (
    ColorObjectDetectorCore,
    ConfirmationTracker,
    DetectionCandidate,
)


def _stamp_seconds(stamp):
    """将 ROS 时间转换为秒，供同步差、过期和 TF 年龄判断使用。"""
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _load_config(config_file):
    """读取保留 HSV 多区间结构的 YAML，而不是依赖 ROS 参数的扁平化限制。"""
    if not config_file or not os.path.isfile(config_file):
        raise ValueError('config_file does not exist: {0}'.format(config_file))
    with open(config_file, 'r') as stream:
        document = yaml.safe_load(stream) or {}
    config = document.get('color_object_detector', {}).get('ros__parameters')
    if not isinstance(config, dict):
        raise ValueError('color_object_detector.ros__parameters is required')
    return config


def _debug_image_message(image, encoding):
    """封装 uint8 调试图，绕过 OpenCV 5 与 Foxy cv_bridge 的类型表失配。"""
    if image.dtype != np.uint8:
        raise ValueError('debug image must use uint8 pixels')
    expected_channels = {'bgr8': 3, 'mono8': 1}
    if encoding not in expected_channels:
        raise ValueError('unsupported debug image encoding: {0}'.format(encoding))
    channels = expected_channels[encoding]
    if image.ndim != (2 if channels == 1 else 3):
        raise ValueError('debug image dimensions do not match {0}'.format(encoding))
    if channels == 3 and image.shape[2] != channels:
        raise ValueError('debug image channel count does not match bgr8')

    # tobytes() 会压紧非连续 NumPy 视图，step 必须按压紧后的每行字节数计算。
    message = Image()
    message.height, message.width = int(image.shape[0]), int(image.shape[1])
    message.encoding = encoding
    message.is_bigendian = False
    message.step = message.width * channels
    message.data.frombytes(image.tobytes())
    return message


def _image_message_to_numpy(message, desired_encoding=None):
    """按 ROS Image 的 step/字节序解码 D435i 图像，不依赖 cv_bridge。

    本机 Foxy 的 cv_bridge 与 OpenCV 5 存在类型映射不一致。这里仅覆盖 D435i
    RGB8、BGR8、16UC1 和 32FC1 输入；未知编码直接失败并由调用方发布状态，
    不以错误深度或错误颜色继续定位。
    """
    encoding = (message.encoding or '').lower()
    formats = {
        'rgb8': (np.dtype(np.uint8), 3),
        'bgr8': (np.dtype(np.uint8), 3),
        '16uc1': (np.dtype('>u2') if message.is_bigendian else np.dtype('<u2'), 1),
        'mono16': (np.dtype('>u2') if message.is_bigendian else np.dtype('<u2'), 1),
        '32fc1': (np.dtype('>f4') if message.is_bigendian else np.dtype('<f4'), 1),
    }
    if encoding not in formats:
        raise ValueError('unsupported image encoding: {0}'.format(message.encoding))
    dtype, channels = formats[encoding]
    itemsize = dtype.itemsize
    required_step = int(message.width) * channels * itemsize
    required_size = int(message.height) * int(message.step)
    if (message.width <= 0 or message.height <= 0 or
            message.step < required_step or len(message.data) < required_size):
        raise ValueError(
            'invalid image layout {0}x{1} step={2} encoding={3}'.format(
                message.width, message.height, message.step, message.encoding))

    # 先保留每行 padding，再提取像素部分；copy 使数组不依赖 ROS 消息生命周期。
    raw = np.frombuffer(message.data, dtype=np.uint8, count=required_size)
    rows = raw.reshape(int(message.height), int(message.step))[:, :required_step]
    pixels = rows.copy().reshape(int(message.height), int(message.width),
                                 channels * itemsize)
    if channels == 1:
        image = pixels.reshape(int(message.height), int(message.width), itemsize)
        image = image.view(dtype).reshape(int(message.height), int(message.width))
        # 运算使用本机字节序，避免大端相机时深度数值被错误解释。
        return image.astype(np.float32 if encoding == '32fc1' else np.uint16,
                            copy=False)

    image = pixels.reshape(int(message.height), int(message.width), channels)
    if desired_encoding == 'bgr8' and encoding == 'rgb8':
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    if desired_encoding and desired_encoding != encoding:
        raise ValueError('cannot convert {0} to {1}'.format(encoding, desired_encoding))
    return image


class ColorObjectDetectorNode(Node):
    """只订阅相机与任务颜色，不发布任何底盘或机械臂控制命令。"""

    def __init__(self):
        super().__init__('color_object_detector_node')
        try:
            default_config = os.path.join(
                get_package_share_directory('rk_perception'), 'config',
                'color_object_detector.yaml')
        except Exception:
            # 源码树直接运行时尚未安装 ament 索引，回退到相对源码位置。
            default_config = os.path.join(
                os.path.dirname(os.path.dirname(__file__)), 'config',
                'color_object_detector.yaml')
        config_file = self.declare_parameter('config_file', default_config).value
        self.config = _load_config(config_file)
        self._declare_scalar_overrides(self.config)
        self.core = ColorObjectDetectorCore(self.config)
        self.expected_color = 'none'
        self.last_candidate = None
        self.tracker = ConfirmationTracker(
            self.config['confirm_frames'], self.config['lost_frames'],
            self.config['max_position_jump_m'], self.config['max_depth_jump_m'])

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.detection_publisher = self.create_publisher(
            ColorObjectDetection, self.config['detection_topic'], 10)
        self.overlay_publisher = self.create_publisher(
            Image, self.config['overlay_topic'], 10)
        self.mask_publisher = self.create_publisher(
            Image, self.config['mask_topic'], 10)
        self.status_publisher = self.create_publisher(
            String, self.config['status_topic'], 10)
        self.object_list_publisher = self.create_publisher(
            String, self.config['object_list_topic'], 10)
        self.expected_color_subscription = self.create_subscription(
            String, self.config['expected_color_topic'], self._on_expected_color,
            qos_profile_sensor_data)

        # 三路均使用 sensor-data QoS；ATS 只负责近似配对，回调仍复核时间差。
        color_subscriber = message_filters.Subscriber(
            self, Image, self.config['color_topic'],
            qos_profile=qos_profile_sensor_data)
        depth_subscriber = message_filters.Subscriber(
            self, Image, self.config['depth_topic'],
            qos_profile=qos_profile_sensor_data)
        camera_info_subscriber = message_filters.Subscriber(
            self, CameraInfo, self.config['camera_info_topic'],
            qos_profile=qos_profile_sensor_data)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [color_subscriber, depth_subscriber, camera_info_subscriber],
            int(self.config['sync_queue_size']), float(self.config['sync_slop_sec']))
        self.sync.registerCallback(self._on_synced_images)
        self.get_logger().info(
            'Color RGB-D detector started: color={0}, depth={1}, info={2}'.format(
                self.config['color_topic'], self.config['depth_topic'],
                self.config['camera_info_topic']))

    def _declare_scalar_overrides(self, mapping, prefix=''):
        """声明标量参数以允许 launch/命令行覆盖，不破坏 YAML 内 HSV 列表。"""
        for key, value in mapping.items():
            name = '{0}.{1}'.format(prefix, key) if prefix else key
            if isinstance(value, dict):
                self._declare_scalar_overrides(value, name)
            elif not isinstance(value, list):
                parameter_value = self.declare_parameter(name, value).value
                parent, leaf = self._mapping_parent(mapping, key)
                parent[leaf] = parameter_value

    @staticmethod
    def _mapping_parent(mapping, key):
        # 该辅助函数使当前层的参数覆盖值直接回写到配置树。
        return mapping, key

    def _on_expected_color(self, message):
        """任务阶段切换立即清空旧确认，阻断上一目标跨阶段遗留。"""
        requested = message.data.strip().lower()
        if requested != self.expected_color:
            self.expected_color = requested
            self.tracker.reset()
            self.last_candidate = None
            self.get_logger().info(
                'expected_color changed to {0}; confirmation reset'.format(
                    requested))

    def _on_synced_images(self, color_message, depth_message, camera_info_message):
        """处理同一时刻 RGB-D-CameraInfo；所有输出沿用彩色图原始时间戳。"""
        stamps = [_stamp_seconds(message.header.stamp) for message in (
            color_message, depth_message, camera_info_message)]
        if max(stamps) - min(stamps) > float(self.config['sync_slop_sec']):
            self._publish_result(
                color_message, DetectionCandidate(reason='sync_time_difference_exceeded'),
                False, False, False, '', None,
                self._camera_frame(color_message, camera_info_message))
            return
        try:
            color_image = _image_message_to_numpy(color_message, 'bgr8')
            depth_image = _image_message_to_numpy(depth_message)
        except Exception as exc:
            self._publish_result(
                color_message, DetectionCandidate(reason='image_decode_error:{0}'.format(exc)),
                False, False, False, '', None,
                self._camera_frame(color_message, camera_info_message))
            return

        camera_info = {
            'fx': camera_info_message.k[0], 'fy': camera_info_message.k[4],
            'cx': camera_info_message.k[2], 'cy': camera_info_message.k[5],
        }
        valid_expected = self.expected_color in self.config['colors']
        if self.expected_color == 'none':
            allowed_colors = list(self.config['colors'].keys())
        elif valid_expected:
            allowed_colors = [self.expected_color]
        else:
            allowed_colors = list(self.config['colors'].keys())
        candidates = self.core.detect_all(
            color_image, depth_image, depth_message.encoding, camera_info,
            allowed_colors)
        candidate = (candidates[0] if candidates else
                     DetectionCandidate(reason='no_valid_contour'))
        self.last_candidate = candidate if candidate.detected else None
        stale = self._is_stale(color_message)
        if self.expected_color == 'none':
            confirmed = self.tracker.update(None, 'none')
            reason = 'expected_color_none'
        elif not valid_expected:
            confirmed = self.tracker.update(None, 'none')
            reason = 'invalid_expected_color'
        else:
            confirmed = self.tracker.update(candidate, self.expected_color)
            reason = candidate.reason
        arm_position, arm_reason = self._transform_to_arm_base(
            color_message, camera_info_message, candidate)
        grasp_ready = bool(
            candidate.detected and confirmed and not stale and
            self.expected_color == candidate.color and arm_position is not None and
            self._inside_arm_workspace(arm_position))
        if stale:
            reason = 'stale_detection'
        elif candidate.detected and arm_reason:
            reason = arm_reason
        elif (candidate.detected and arm_position is not None and
              not self._inside_arm_workspace(arm_position)):
            reason = 'outside_arm_workspace'
        self._publish_result(
            color_message, candidate, confirmed, grasp_ready, stale,
            reason, arm_position, self._camera_frame(
                color_message, camera_info_message), color_image, candidates)
        self._publish_object_list(color_message, candidates)

    def _is_stale(self, image_message):
        """使用输入图像时间判断失效，绝不以处理完成时间替代采集时间。"""
        now_seconds = self.get_clock().now().nanoseconds * 1e-9
        age = now_seconds - _stamp_seconds(image_message.header.stamp)
        max_age = min(float(self.config['max_image_age_sec']),
                      float(self.config['max_detection_age_sec']))
        return age > max_age

    def _camera_frame(self, image_message, camera_info_message):
        """优先保持实际 CameraInfo 光学 frame，兼容驱动仅在图像填写 frame 的情况。"""
        return (self.config['camera_frame'] or camera_info_message.header.frame_id or
                image_message.header.frame_id)

    def _transform_to_arm_base(self, image_message, camera_info_message, candidate):
        """仅通过 TF2 得到机械臂基座坐标；失败时绝不伪造外参结果。"""
        if not candidate.detected:
            return None, ''
        source_frame = self._camera_frame(image_message, camera_info_message)
        target_frame = self.config['arm_base_frame']
        if not source_frame or not target_frame:
            return None, 'missing_tf_frame'
        point = PointStamped()
        point.header = image_message.header
        point.header.frame_id = source_frame
        point.point.x, point.point.y, point.point.z = candidate.position_camera
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame, source_frame, image_message.header.stamp,
                timeout=Duration(seconds=float(self.config['tf_timeout_sec'])))
            transform_time = _stamp_seconds(transform.header.stamp)
            # 静态 TF 的零时间戳没有年龄概念；动态 TF 必须与图像同一时刻附近。
            image_time = _stamp_seconds(image_message.header.stamp)
            if (transform_time and abs(transform_time - image_time) >
                    float(self.config['tf_max_age_sec'])):
                return None, 'stale_tf'
            transformed = do_transform_point(point, transform)
            result = (transformed.point.x, transformed.point.y, transformed.point.z)
            if not all(math.isfinite(value) for value in result):
                return None, 'non_finite_arm_position'
            return result, ''
        except Exception as exc:
            if self.config['debug_log']:
                self.get_logger().debug('TF unavailable: {0}'.format(exc))
            return None, 'tf_unavailable'

    def _inside_arm_workspace(self, position):
        """仅做未标定工作空间门控，不生成或发送任何机械臂轨迹。"""
        workspace = self.config['arm_workspace']
        return (workspace['x_min_m'] <= position[0] <= workspace['x_max_m'] and
                workspace['y_min_m'] <= position[1] <= workspace['y_max_m'] and
                workspace['z_min_m'] <= position[2] <= workspace['z_max_m'])

    def _publish_result(self, image_message, candidate, confirmed, grasp_ready,
                        stale, reason, arm_position, camera_frame,
                        color_image=None, candidates=None):
        """在固定 topic 发布检测和可选调试图，消息头保持彩色图时间。"""
        result = ColorObjectDetection()
        result.header = image_message.header
        result.detected = candidate.detected
        result.confirmed = bool(confirmed)
        result.grasp_ready = bool(grasp_ready)
        result.stale = bool(stale)
        result.color = candidate.color
        result.confidence = float(candidate.confidence)
        result.shape = candidate.shape
        result.shape_confidence = float(candidate.shape_confidence)
        result.polygon_vertices = int(candidate.polygon_vertices)
        result.rotated_aspect_ratio = float(candidate.rotated_aspect_ratio)
        result.center_x, result.center_y = candidate.center_x, candidate.center_y
        result.bbox_x, result.bbox_y = candidate.bbox_x, candidate.bbox_y
        result.bbox_width, result.bbox_height = candidate.bbox_width, candidate.bbox_height
        result.contour_area, result.area_ratio = candidate.contour_area, candidate.area_ratio
        result.circularity, result.solidity = candidate.circularity, candidate.solidity
        result.depth_m, result.depth_mad_m = candidate.depth_m, candidate.depth_mad_m
        result.valid_depth_pixels = candidate.valid_depth_pixels
        (result.position_camera.x, result.position_camera.y,
         result.position_camera.z) = candidate.position_camera
        if arm_position is not None:
            (result.position_arm_base.x, result.position_arm_base.y,
             result.position_arm_base.z) = arm_position
        result.camera_frame = camera_frame
        result.arm_base_frame = self.config['arm_base_frame'] if arm_position is not None else ''
        result.reason = reason
        self.detection_publisher.publish(result)
        self.status_publisher.publish(String(data=reason))
        if self.config['publish_debug_image'] and color_image is not None:
            self._publish_debug_images(
                image_message, color_image, candidate, result, candidates or [])

    def _publish_object_list(self, image_message, candidates):
        """发布同色多物体的只读相机坐标清单，供标定与抓取规划人工核验。

        JSON 不携带任何执行命令；shape 是二维投影线索，不能替代三维类别验证。
        """
        objects = []
        for index, candidate in enumerate(candidates):
            objects.append({
                'index': index,
                'color': candidate.color,
                'shape_2d': candidate.shape,
                'shape_confidence': round(float(candidate.shape_confidence), 4),
                'pixel': {'u': int(candidate.center_x), 'v': int(candidate.center_y)},
                'depth_m': round(float(candidate.depth_m), 4),
                # ROS optical frame: x-right, y-down, z-forward; units are metres.
                'position_camera_m': {
                    'x': round(float(candidate.position_camera[0]), 4),
                    'y': round(float(candidate.position_camera[1]), 4),
                    'z': round(float(candidate.position_camera[2]), 4),
                },
                'bbox': {
                    'x': int(candidate.bbox_x), 'y': int(candidate.bbox_y),
                    'width': int(candidate.bbox_width), 'height': int(candidate.bbox_height),
                },
            })
        payload = {
            'stamp_sec': round(_stamp_seconds(image_message.header.stamp), 6),
            'camera_frame': image_message.header.frame_id,
            'objects': objects,
            'note': ('shape_2d is a projected contour clue, not a calibrated '
                     '3D class or a grasp command'),
        }
        self.object_list_publisher.publish(
            String(data=json.dumps(payload, separators=(',', ':'), sort_keys=True)))

    def _publish_debug_images(self, image_message, color_image, candidate, result,
                              candidates):
        """调试图只服务观测和排障，正式节点不调用任何 GUI API。"""
        overlay = color_image.copy()
        for other in candidates:
            if other is candidate:
                continue
            cv2.rectangle(overlay, (other.bbox_x, other.bbox_y),
                          (other.bbox_x + other.bbox_width,
                           other.bbox_y + other.bbox_height),
                          (0, 200, 0), 1)
            cv2.putText(overlay, '{0}/{1} z={2:.2f}'.format(
                other.color, other.shape, other.depth_m),
                (other.bbox_x, max(16, other.bbox_y - 5)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 200, 0), 1, cv2.LINE_AA)
        if candidate.detected:
            cv2.rectangle(overlay, (candidate.bbox_x, candidate.bbox_y),
                          (candidate.bbox_x + candidate.bbox_width,
                           candidate.bbox_y + candidate.bbox_height),
                          (0, 255, 255), 2)
            cv2.circle(overlay, (candidate.center_x, candidate.center_y), 4,
                       (255, 255, 255), -1)
            cv2.putText(overlay, 'selected {0}/{1} z={2:.2f}'.format(
                candidate.color, candidate.shape, candidate.depth_m),
                (candidate.bbox_x, max(16, candidate.bbox_y - 5)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 255, 255), 1, cv2.LINE_AA)
        camera_text = 'cam=({0:.3f},{1:.3f},{2:.3f})'.format(
            result.position_camera.x, result.position_camera.y, result.position_camera.z)
        arm_text = 'arm=({0:.3f},{1:.3f},{2:.3f})'.format(
            result.position_arm_base.x, result.position_arm_base.y, result.position_arm_base.z)
        lines = [
            'expected={0} color={1}'.format(self.expected_color, result.color),
            'detected={0} confirmed={1} ready={2} stale={3}'.format(
                result.detected, result.confirmed, result.grasp_ready, result.stale),
            'depth={0:.3f} valid={1} confidence={2:.2f}'.format(
                result.depth_m, result.valid_depth_pixels, result.confidence),
            'shape={0} conf={1:.2f} vertices={2} rot_aspect={3:.2f}'.format(
                result.shape, result.shape_confidence,
                result.polygon_vertices, result.rotated_aspect_ratio),
            camera_text, arm_text, 'reason={0}'.format(result.reason),
        ]
        for index, line in enumerate(lines):
            cv2.putText(overlay, line, (8, 22 + index * 21),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 255, 255), 1,
                        cv2.LINE_AA)
        mask = candidate.mask
        if mask is None:
            mask = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
            mask[:] = 0
        overlay_message = _debug_image_message(overlay, 'bgr8')
        overlay_message.header = image_message.header
        mask_message = _debug_image_message(mask, 'mono8')
        mask_message.header = image_message.header
        self.overlay_publisher.publish(overlay_message)
        self.mask_publisher.publish(mask_message)


def main(args=None):
    """ROS 2 控制台入口。"""
    rclpy.init(args=args)
    node = ColorObjectDetectorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
