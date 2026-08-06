"""基于 HSV 与区域深度统计的颜色目标检测核心。

本模块刻意不依赖 ROS，便于在没有相机或 ROS 图的环境中测试。它只输出相机
光学坐标系下的候选结果，不推测机械臂外参，也不包含任何执行机构控制逻辑。
"""

from __future__ import division

from dataclasses import dataclass, field
import math

import cv2
import numpy as np


@dataclass
class DetectionCandidate(object):
    """一帧中通过二维与深度过滤的目标候选，坐标单位均为米。"""

    detected: bool = False
    color: str = ''
    center_x: int = 0
    center_y: int = 0
    bbox_x: int = 0
    bbox_y: int = 0
    bbox_width: int = 0
    bbox_height: int = 0
    contour_area: float = 0.0
    area_ratio: float = 0.0
    circularity: float = 0.0
    solidity: float = 0.0
    depth_m: float = 0.0
    depth_mad_m: float = 0.0
    valid_depth_pixels: int = 0
    position_camera: tuple = (0.0, 0.0, 0.0)
    confidence: float = 0.0
    reason: str = 'no_candidate'
    contour: object = field(default=None, repr=False)
    mask: object = field(default=None, repr=False)


class ConfirmationTracker(object):
    """按同色、位置与深度连续性确认视觉结果的纯状态机。"""

    def __init__(self, confirm_frames, lost_frames,
                 max_position_jump_m, max_depth_jump_m):
        self.confirm_frames = int(confirm_frames)
        self.lost_frames = int(lost_frames)
        self.max_position_jump_m = float(max_position_jump_m)
        self.max_depth_jump_m = float(max_depth_jump_m)
        self.reset()

    def reset(self):
        """阶段切换或跳变时清除旧目标，防止历史结果误确认。"""
        self._count = 0
        self._lost_count = 0
        self._color = ''
        self._position = None
        self._depth_m = None
        self.confirmed = False

    def update(self, candidate, expected_color):
        """更新确认状态；none 和非预期颜色永远不产生确认。"""
        valid = (
            candidate is not None and candidate.detected and
            expected_color not in ('', 'none') and
            candidate.color == expected_color
        )
        if not valid:
            self._lost_count += 1
            if self._lost_count >= self.lost_frames:
                self.reset()
            return False

        position = np.asarray(candidate.position_camera, dtype=np.float64)
        jumped = self._color != candidate.color
        if self._position is not None:
            jumped = jumped or (
                np.linalg.norm(position - self._position) >
                self.max_position_jump_m
            )
            jumped = jumped or (
                abs(candidate.depth_m - self._depth_m) > self.max_depth_jump_m
            )
        if jumped:
            self._count = 0
            self.confirmed = False

        self._count += 1
        self._lost_count = 0
        self._color = candidate.color
        self._position = position
        self._depth_m = candidate.depth_m
        self.confirmed = self._count >= self.confirm_frames
        return self.confirmed


class ColorObjectDetectorCore(object):
    """执行 HSV 分割、轮廓过滤、区域深度估计和反投影。"""

    def __init__(self, config):
        self.config = config
        self._validate_config()

    def _validate_config(self):
        """在节点启动前拒绝危险或无法解释的参数组合。"""
        required = ('colors', 'min_depth_m', 'max_depth_m',
                    'min_contour_area_px', 'max_contour_area_px')
        for key in required:
            if key not in self.config:
                raise ValueError('missing required config: {0}'.format(key))
        if self.config['min_depth_m'] >= self.config['max_depth_m']:
            raise ValueError('min_depth_m must be smaller than max_depth_m')
        if self.config['min_contour_area_px'] > self.config['max_contour_area_px']:
            raise ValueError('min_contour_area_px must not exceed max_contour_area_px')
        for color, color_config in self.config['colors'].items():
            ranges = color_config.get('hsv_ranges', [])
            if not ranges:
                raise ValueError('color has no hsv_ranges: {0}'.format(color))
            for hsv_range in ranges:
                lower = hsv_range.get('lower', [])
                upper = hsv_range.get('upper', [])
                if len(lower) != 3 or len(upper) != 3:
                    raise ValueError('HSV range must contain three values')
                if any(value < 0 or value > 255 for value in lower + upper):
                    raise ValueError('HSV values must be in [0, 255]')

    @staticmethod
    def depth_to_meters(depth_image, encoding):
        """按 ROS 图像编码转换深度，避免把 32FC1 错当毫米。"""
        encoding = (encoding or '').upper()
        if encoding in ('16UC1', 'MONO16'):
            return depth_image.astype(np.float32) * 0.001
        if encoding == '32FC1':
            return depth_image.astype(np.float32, copy=False)
        raise ValueError('unsupported depth encoding: {0}'.format(encoding))

    def build_color_mask(self, bgr_image, color):
        """合并一个颜色的多个 HSV 区间，支持红色 Hue 跨界。"""
        color_config = self.config['colors'].get(color)
        if not color_config or not color_config.get('enabled', False):
            return np.zeros(bgr_image.shape[:2], dtype=np.uint8)
        kernel_size = self._odd_kernel(self.config['blur_kernel_size'])
        blurred = cv2.GaussianBlur(bgr_image, (kernel_size, kernel_size), 0)
        hsv_image = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        mask = np.zeros(hsv_image.shape[:2], dtype=np.uint8)
        for hsv_range in color_config['hsv_ranges']:
            lower = np.array(hsv_range['lower'], dtype=np.uint8)
            upper = np.array(hsv_range['upper'], dtype=np.uint8)
            mask = cv2.bitwise_or(mask, cv2.inRange(hsv_image, lower, upper))
        mask = self._morphology(mask, cv2.MORPH_OPEN,
                                self.config['morph_open_kernel'])
        return self._morphology(mask, cv2.MORPH_CLOSE,
                                self.config['morph_close_kernel'])

    def detect(self, bgr_image, depth_image, depth_encoding, camera_info,
               allowed_colors=None):
        """检测候选并优先选择最靠近预设抓取图像中心的有效目标。"""
        if bgr_image is None or depth_image is None:
            return DetectionCandidate(reason='missing_image')
        if bgr_image.ndim != 3 or bgr_image.shape[2] != 3:
            return DetectionCandidate(reason='invalid_color_image')
        if depth_image.shape[:2] != bgr_image.shape[:2]:
            return DetectionCandidate(reason='color_depth_size_mismatch')
        if not camera_info or not self._valid_intrinsics(camera_info):
            return DetectionCandidate(reason='missing_or_invalid_camera_info')
        self._image_height, self._image_width = bgr_image.shape[:2]
        try:
            depth_m = self.depth_to_meters(depth_image, depth_encoding)
        except ValueError as exc:
            return DetectionCandidate(reason=str(exc))

        colors = allowed_colors if allowed_colors is not None else self.config['colors'].keys()
        candidates = []
        for color in colors:
            if color not in self.config['colors']:
                continue
            mask = self.build_color_mask(bgr_image, color)
            contours_info = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            contours = contours_info[-2]
            for contour in contours:
                candidate = self._candidate_from_contour(
                    contour, mask, depth_m, camera_info, color)
                if candidate.detected:
                    candidates.append(candidate)
        if not candidates:
            return DetectionCandidate(reason='no_valid_contour')
        candidates.sort(key=self._candidate_sort_key)
        return candidates[0]

    def _candidate_from_contour(self, contour, mask, depth_m, camera_info, color):
        """以轮廓内部深度而非单个中心像素构建候选。"""
        image_area = float(mask.shape[0] * mask.shape[1])
        area = float(cv2.contourArea(contour))
        x, y, width, height = cv2.boundingRect(contour)
        area_ratio = area / image_area if image_area else 0.0
        perimeter = float(cv2.arcLength(contour, True))
        circularity = (4.0 * math.pi * area / (perimeter * perimeter)
                       if perimeter > 0.0 else 0.0)
        hull = cv2.convexHull(contour)
        hull_area = float(cv2.contourArea(hull))
        solidity = area / hull_area if hull_area > 0.0 else 0.0
        if not self._passes_shape_filters(
                x, y, width, height, area, area_ratio, circularity, solidity,
                mask.shape[1], mask.shape[0]):
            return DetectionCandidate(reason='contour_filtered')

        region_mask = np.zeros(mask.shape, dtype=np.uint8)
        cv2.drawContours(region_mask, [contour], -1, 255, thickness=-1)
        erode_pixels = int(self.config['mask_erode_pixels'])
        if erode_pixels > 0:
            erode_size = 2 * erode_pixels + 1
            kernel = cv2.getStructuringElement(
                cv2.MORPH_ELLIPSE, (erode_size, erode_size))
            region_mask = cv2.erode(region_mask, kernel, iterations=1)
        depth_result = self._robust_depth(region_mask, depth_m)
        if depth_result is None:
            return DetectionCandidate(reason='insufficient_valid_depth')
        depth_value, depth_mad, valid_count, representative_x, representative_y = depth_result
        position = self.project_pixel(
            representative_x, representative_y, depth_value, camera_info)
        if not np.all(np.isfinite(position)):
            return DetectionCandidate(reason='non_finite_camera_position')
        confidence = self._confidence(area_ratio, circularity, solidity, depth_mad)
        return DetectionCandidate(
            detected=True, color=color, center_x=representative_x,
            center_y=representative_y, bbox_x=x, bbox_y=y,
            bbox_width=width, bbox_height=height, contour_area=area,
            area_ratio=area_ratio, circularity=circularity, solidity=solidity,
            depth_m=depth_value, depth_mad_m=depth_mad,
            valid_depth_pixels=valid_count, position_camera=tuple(position),
            confidence=confidence, reason='candidate_valid', contour=contour,
            mask=region_mask)

    def _passes_shape_filters(self, x, y, width, height, area, area_ratio,
                              circularity, solidity, image_width, image_height):
        """先使用二维规则剔除噪点、边界外目标和不可信形状。"""
        aspect_ratio = float(width) / float(height) if height else float('inf')
        checks = (
            self.config['min_contour_area_px'] <= area <= self.config['max_contour_area_px'],
            self.config['min_area_ratio'] <= area_ratio <= self.config['max_area_ratio'],
            width >= self.config['min_width_px'],
            height >= self.config['min_height_px'],
            self.config['min_aspect_ratio'] <= aspect_ratio <= self.config['max_aspect_ratio'],
            circularity >= self.config['min_circularity'],
            solidity >= self.config['min_solidity'],
            self._inside_roi(x, y, width, height, image_width, image_height),
        )
        return all(checks)

    def _inside_roi(self, x, y, width, height, image_width, image_height):
        """要求整个候选框处于允许 ROI，避免场景边缘误触发。"""
        roi_x_min = self.config['roi_x_min_fraction'] * image_width
        roi_x_max = self.config['roi_x_max_fraction'] * image_width
        roi_y_min = self.config['roi_y_min_fraction'] * image_height
        roi_y_max = self.config['roi_y_max_fraction'] * image_height
        return x >= roi_x_min and y >= roi_y_min and (
            x + width <= roi_x_max and y + height <= roi_y_max)

    def _robust_depth(self, region_mask, depth_m):
        """用中位数与 MAD 抑制轮廓边缘和背景深度异常值。"""
        rows, cols = np.where(region_mask > 0)
        region_pixel_count = len(rows)
        samples = depth_m[rows, cols]
        valid = np.isfinite(samples)
        valid &= samples >= self.config['min_depth_m']
        valid &= samples <= self.config['max_depth_m']
        rows, cols, samples = rows[valid], cols[valid], samples[valid]
        original_count = len(samples)
        min_count = int(self.config['min_valid_depth_pixels'])
        if (original_count < min_count or not region_pixel_count or
                float(original_count) / float(region_pixel_count) <
                float(self.config['min_valid_depth_fraction'])):
            return None
        median = float(np.median(samples))
        mad = float(np.median(np.abs(samples - median)))
        epsilon = float(self.config.get('depth_mad_epsilon_m', 0.000001))
        if mad > epsilon:
            inlier = np.abs(samples - median) <= self.config['depth_mad_scale'] * mad
            rows, cols, samples = rows[inlier], cols[inlier], samples[inlier]
        if len(samples) < min_count:
            return None
        final_median = float(np.median(samples))
        final_mad = float(np.median(np.abs(samples - final_median)))
        if final_mad > self.config['max_depth_mad_m']:
            return None
        # 使用有效深度内点的中位像素，避免 bbox 中心落在空洞或背景上。
        representative_x = int(round(float(np.median(cols))))
        representative_y = int(round(float(np.median(rows))))
        return final_median, final_mad, int(len(samples)), representative_x, representative_y

    def _candidate_sort_key(self, candidate):
        """严格按抓取中心、三维距离、面积的优先级选择同色候选。"""
        center_x = self.config['grasp_center_x_fraction']
        center_y = self.config['grasp_center_y_fraction']
        image_distance = math.hypot(
            candidate.center_x - center_x * self._image_width,
            candidate.center_y - center_y * self._image_height)
        point_distance = math.sqrt(sum(value * value for value in candidate.position_camera))
        return (image_distance, point_distance, -candidate.contour_area)

    def _morphology(self, mask, operation, configured_size):
        size = int(configured_size)
        if size <= 1:
            return mask
        size = self._odd_kernel(size)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (size, size))
        return cv2.morphologyEx(mask, operation, kernel,
                                iterations=int(self.config['morph_iterations']))

    @staticmethod
    def _odd_kernel(size):
        size = max(1, int(size))
        return size if size % 2 else size + 1

    @staticmethod
    def _valid_intrinsics(camera_info):
        return all(math.isfinite(float(camera_info.get(key, 0.0))) and
                   float(camera_info.get(key, 0.0)) > 0.0
                   for key in ('fx', 'fy'))

    @staticmethod
    def project_pixel(u, v, depth_m, camera_info):
        """遵守 ROS optical frame：X 右、Y 下、Z 前。"""
        fx, fy = float(camera_info['fx']), float(camera_info['fy'])
        cx, cy = float(camera_info['cx']), float(camera_info['cy'])
        return ((float(u) - cx) * depth_m / fx,
                (float(v) - cy) * depth_m / fy, depth_m)

    def _confidence(self, area_ratio, circularity, solidity, depth_mad):
        """为调试提供可解释的保守置信度，不作为机械臂安全判据。"""
        area_span = max(self.config['max_area_ratio'] - self.config['min_area_ratio'], 1e-9)
        area_score = min(1.0, max(0.0, (area_ratio - self.config['min_area_ratio']) / area_span))
        shape_score = min(1.0, (circularity + solidity) * 0.5)
        depth_score = max(0.0, 1.0 - depth_mad / self.config['max_depth_mad_m'])
        return float((area_score + shape_score + depth_score) / 3.0)

    @property
    def _image_width(self):
        return getattr(self, '__image_width', 1)

    @_image_width.setter
    def _image_width(self, value):
        self.__image_width = value

    @property
    def _image_height(self):
        return getattr(self, '__image_height', 1)

    @_image_height.setter
    def _image_height(self, value):
        self.__image_height = value
