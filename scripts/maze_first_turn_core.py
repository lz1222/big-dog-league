#!/usr/bin/env python3

"""B2.1 第一左弯局部规划纯逻辑核心，不依赖 ROS 或 Unitree SDK。"""

from dataclasses import dataclass, replace
import math


SECTOR_NAMES = (
    'front',
    'left_front',
    'right_front',
    'left',
    'right',
    'left_rear',
    'right_rear',
    'rear',
)

STATE_APPROACH_TURN = 'APPROACH_TURN'
STATE_STOP_AND_SCAN = 'STOP_AND_SCAN'
STATE_SELECT_TRAJECTORY = 'SELECT_TRAJECTORY'
STATE_EXECUTE_SHORT_SEGMENT = 'EXECUTE_SHORT_SEGMENT'
STATE_STOP_AND_RESCAN = 'STOP_AND_RESCAN'
STATE_REVERSE_RECOVERY = 'REVERSE_RECOVERY'
STATE_TURN_FINE_ALIGN = 'TURN_FINE_ALIGN'
STATE_CORRIDOR_REACQUIRE = 'CORRIDOR_REACQUIRE'
STATE_TURN_COMPLETE = 'TURN_COMPLETE'
STATE_FAULT_STOP = 'FAULT_STOP'

VERDICT_UNSAFE = 'UNSAFE'
VERDICT_UNKNOWN = 'UNKNOWN'
VERDICT_GEOMETRY_SAFE_UNCALIBRATED = 'GEOMETRY_SAFE_UNCALIBRATED'
VERDICT_NOMINAL_SAFE = 'NOMINAL_SAFE'
VERDICT_ROBUST_SAFE = 'ROBUST_SAFE'
VERDICT_EXECUTABLE_SAFE = 'EXECUTABLE_SAFE'

# 兼容首版测试导入；对外 JSON 统一输出正式六级名称。
VERDICT_DRY_RUN_SAFE = VERDICT_GEOMETRY_SAFE_UNCALIBRATED

VERDICT_PRIORITY = {
    VERDICT_UNSAFE: 0,
    VERDICT_UNKNOWN: 1,
    VERDICT_GEOMETRY_SAFE_UNCALIBRATED: 2,
    VERDICT_NOMINAL_SAFE: 3,
    VERDICT_ROBUST_SAFE: 4,
    VERDICT_EXECUTABLE_SAFE: 5,
}

PRIMITIVE_FORWARD = 'FORWARD_SHORT'
PRIMITIVE_OUTSIDE_DIAGONAL = 'OUTSIDE_DIAGONAL_SHORT'
PRIMITIVE_LEFT_ARC = 'LEFT_ARC'
PRIMITIVE_LEFT_ARC_OUTSIDE = 'LEFT_ARC_OUTSIDE_VY'
PRIMITIVE_REVERSE = 'REVERSE_SHORT'
PRIMITIVE_FINE_LEFT_ARC = 'FINE_LEFT_ARC'

PRIMITIVE_ORDER = (
    PRIMITIVE_FORWARD,
    PRIMITIVE_OUTSIDE_DIAGONAL,
    PRIMITIVE_LEFT_ARC,
    PRIMITIVE_LEFT_ARC_OUTSIDE,
    PRIMITIVE_REVERSE,
    PRIMITIVE_FINE_LEFT_ARC,
)


@dataclass(frozen=True)
class LocalMapConfig:
    """定义 base_link 周围局部地图、点云过滤和角向覆盖判定。"""

    x_min_m: float = -1.5
    x_max_m: float = 1.5
    y_min_m: float = -1.5
    y_max_m: float = 1.5
    resolution_m: float = 0.02
    z_min_m: float = -0.15
    z_max_m: float = 0.50
    body_x_min_m: float = -0.40
    body_x_max_m: float = 0.40
    body_y_min_m: float = -0.18
    body_y_max_m: float = 0.18
    min_range_m: float = 0.05
    coverage_bin_deg: float = 5.0
    sector_min_coverage_points: int = 3
    sector_min_coverage_bins: int = 2
    wall_inlier_tolerance_m: float = 0.035
    wall_cluster_gap_m: float = 0.10
    wall_min_points: int = 6
    wall_min_length_m: float = 0.12
    wall_max_residual_m: float = 0.04
    wall_max_segments: int = 16
    wall_ransac_sample_limit: int = 8
    wall_endpoint_uncertainty_m: float = 0.03


@dataclass(frozen=True)
class DynamicFootprint:
    """描述相对 base_link 的非对称机身边界和动态安全余量。"""

    footprint_front_m: float = 0.35
    footprint_rear_m: float = 0.35
    footprint_left_m: float = 0.155
    footprint_right_m: float = 0.155
    gait_sway_margin_m: float = 0.03
    cloud_uncertainty_margin_m: float = 0.015
    odom_uncertainty_margin_m: float = 0.015
    model_uncertainty_margin_m: float = 0.015
    stop_tail_margin_m: float = 0.05
    target_physical_clearance_m: float = 0.05
    margins_calibrated: bool = False

    def expanded_extents(self):
        """返回加入点云、里程计和步态横摆后的四向边界。"""
        common = (
            self.cloud_uncertainty_margin_m
            + self.odom_uncertainty_margin_m
            + self.model_uncertainty_margin_m
        )
        return {
            'front': self.footprint_front_m + common,
            'rear': self.footprint_rear_m + common,
            'left': (
                self.footprint_left_m
                + common
                + self.gait_sway_margin_m
            ),
            'right': (
                self.footprint_right_m
                + common
                + self.gait_sway_margin_m
            ),
        }


@dataclass(frozen=True)
class MotionPrimitive:
    """单个已标定或待标定的短段运动模型。"""

    name: str
    vx_mps: float
    vy_mps: float
    wz_radps: float
    duration_sec: float
    calibrated: bool = False
    calibration_id: str = 'UNVALIDATED'


@dataclass(frozen=True)
class PlannerConfig:
    """第一弯规划阈值；不包含后续四个弯的拓扑。"""

    cloud_stale_timeout_sec: float = 0.50
    odom_stale_timeout_sec: float = 0.20
    sector_stale_timeout_sec: float = 0.50
    safety_status_stale_timeout_sec: float = 0.30
    trajectory_sample_dt_sec: float = 0.02
    trajectory_max_translation_step_m: float = 0.01
    trajectory_max_yaw_step_deg: float = 1.0
    clearance_search_m: float = 0.20
    hard_clearance_m: float = 0.01
    robust_clearance_m: float = 0.02
    legacy_turn_sweep_clearance_m: float = 0.413
    minimum_wall_segments_for_turn: int = 1
    wall_confidence_min: float = 0.45
    trajectory_vx_uncertainty_ratio: float = 0.15
    trajectory_vy_uncertainty_mps: float = 0.03
    trajectory_wz_uncertainty_ratio: float = 0.15
    trajectory_duration_uncertainty_sec: float = 0.05
    approach_stop_distance_m: float = 0.80
    left_open_distance_m: float = 0.42
    turn_target_deg: float = 90.0
    fine_align_enter_error_deg: float = 20.0
    turn_complete_tolerance_deg: float = 5.0
    reacquire_side_difference_m: float = 0.06
    stable_confirm_frames: int = 3
    max_roll_deg: float = 8.0
    max_pitch_deg: float = 8.0
    stationary_linear_speed_mps: float = 0.04
    stationary_angular_speed_radps: float = 0.04
    yaw_jump_limit_deg: float = 20.0
    max_tracking_position_error_m: float = 0.06
    max_tracking_yaw_error_deg: float = 8.0
    stage_timeout_sec: float = 30.0
    max_reverse_segments: int = 3
    max_reverse_distance_m: float = 0.30


@dataclass(frozen=True)
class Pose2D:
    """局部轨迹中的二维位姿。"""

    x_m: float
    y_m: float
    yaw_rad: float
    time_sec: float


@dataclass(frozen=True)
class SafetyContext:
    """把传感器、外部安全链和姿态状态显式传给规划器。"""

    cloud_age_sec: float
    odom_age_sec: float
    sector_status: dict
    cloud_received: bool = True
    odom_received: bool = True
    cloud_valid: bool = True
    odom_valid: bool = True
    watchdog_ok: object = None
    watchdog_age_sec: object = None
    estop_triggered: object = None
    estop_age_sec: object = None
    roll_rad: float = 0.0
    pitch_rad: float = 0.0
    yaw_jump: bool = False
    linear_speed_mps: float = 0.0
    angular_speed_radps: float = 0.0
    stop_stable: bool = True


class LocalOccupancyGrid:
    """稀疏二维占据图；每个占据单元只保留中心坐标。"""

    def __init__(
        self,
        config,
        occupied_cells,
        statistics,
        sector_stats,
        wall_segments=(),
    ):
        self.config = config
        self.occupied_cells = frozenset(occupied_cells)
        self.statistics = dict(statistics)
        self.sector_stats = {
            name: dict(sector_stats[name])
            for name in SECTOR_NAMES
        }
        # 降采样单元中心是最终碰撞检查的原始证据，墙模型不能替代它们。
        self.obstacle_points = tuple(
            self.cell_center(cell)
            for cell in sorted(self.occupied_cells)
        )
        self.wall_segments = tuple(dict(item) for item in wall_segments)
        # 已被可靠有限墙段解释的点仍保留在 obstacle_points，名义轨迹会再次
        # 检查；扰动批量检查可只扫描未建模点，避免对同一墙重复数十次计算。
        self.obstacle_wall_segment_ids = {
            point: self.nearest_wall_segment_id(point)
            for point in self.obstacle_points
        }
        self.unmodeled_obstacle_points = tuple(
            point
            for point in self.obstacle_points
            if self.obstacle_wall_segment_ids[point] is None
        )
        self._bucket_size_m = max(0.20, 10.0 * self.config.resolution_m)
        self._obstacle_buckets = self._build_point_buckets(
            self.obstacle_points
        )
        self._unmodeled_buckets = self._build_point_buckets(
            self.unmodeled_obstacle_points
        )

    def cell_center(self, cell):
        ix, iy = cell
        return (
            self.config.x_min_m
            + (ix + 0.5) * self.config.resolution_m,
            self.config.y_min_m
            + (iy + 0.5) * self.config.resolution_m,
        )

    def nearby_points(self, pose, radius_m, include_modeled_points=True):
        """遍历稀疏降采样点；局部图较稀时比扫描矩形网格更稳定。"""
        radius_squared = radius_m * radius_m
        points = (
            self._obstacle_buckets
            if include_modeled_points
            else self._unmodeled_buckets
        )
        min_bx = int(math.floor(
            (pose.x_m - radius_m) / self._bucket_size_m
        ))
        max_bx = int(math.floor(
            (pose.x_m + radius_m) / self._bucket_size_m
        ))
        min_by = int(math.floor(
            (pose.y_m - radius_m) / self._bucket_size_m
        ))
        max_by = int(math.floor(
            (pose.y_m + radius_m) / self._bucket_size_m
        ))
        for bucket_x in range(min_bx, max_bx + 1):
            for bucket_y in range(min_by, max_by + 1):
                for point in points.get((bucket_x, bucket_y), ()):
                    dx = point[0] - pose.x_m
                    dy = point[1] - pose.y_m
                    if dx * dx + dy * dy <= radius_squared:
                        yield point

    def _build_point_buckets(self, points):
        buckets = {}
        for point in points:
            key = (
                int(math.floor(point[0] / self._bucket_size_m)),
                int(math.floor(point[1] / self._bucket_size_m)),
            )
            buckets.setdefault(key, []).append(point)
        return {
            key: tuple(bucket_points)
            for key, bucket_points in buckets.items()
        }

    def nearby_evidence(self, pose, radius_m):
        """输出降采样点、有限墙段和墙端三类独立碰撞证据。"""
        for point in self.nearby_points(pose, radius_m):
            yield {
                'point_m': point,
                'geometry_type': 'obstacle_point',
                'wall_segment_id': self.nearest_wall_segment_id(point),
                'uncertainty_m': 0.0,
            }

        for segment in self.wall_segments:
            start = tuple(segment['start_m'])
            end = tuple(segment['end_m'])
            if self._point_segment_distance(
                (pose.x_m, pose.y_m),
                start,
                end,
            ) > radius_m:
                continue
            length = float(segment['length_m'])
            step = max(0.005, 0.5 * self.config.resolution_m)
            count = max(1, int(math.ceil(length / step)))
            for index in range(count + 1):
                fraction = index / count
                point = (
                    start[0] + fraction * (end[0] - start[0]),
                    start[1] + fraction * (end[1] - start[1]),
                )
                is_endpoint = index in (0, count)
                yield {
                    'point_m': point,
                    'geometry_type': (
                        'wall_endpoint' if is_endpoint else 'wall_segment'
                    ),
                    'wall_segment_id': segment['id'],
                    'wall_endpoint': (
                        'start' if index == 0 else 'end' if is_endpoint else None
                    ),
                    # 墙端位置比墙面内点更不稳定，碰撞检查必须显式扣除
                    # 端点不确定度，不能把拟合端点当作精确真值。
                    'uncertainty_m': (
                        float(segment['endpoint_uncertainty_m'])
                        if is_endpoint
                        else float(segment['residual_rms_m'])
                    ),
                }

    def nearest_wall_segment_id(self, point):
        best_id = None
        best_distance = None
        for segment in self.wall_segments:
            distance = self._point_segment_distance(
                point,
                tuple(segment['start_m']),
                tuple(segment['end_m']),
            )
            if best_distance is None or distance < best_distance:
                best_distance = distance
                best_id = segment['id']
        if (
            best_distance is None
            or best_distance > 2.0 * self.config.wall_inlier_tolerance_m
        ):
            return None
        return best_id

    @staticmethod
    def _point_segment_distance(point, start, end):
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        length_squared = dx * dx + dy * dy
        if length_squared <= 1.0e-12:
            return math.hypot(point[0] - start[0], point[1] - start[1])
        projection = (
            (point[0] - start[0]) * dx
            + (point[1] - start[1]) * dy
        ) / length_squared
        projection = max(0.0, min(1.0, projection))
        closest = (
            start[0] + projection * dx,
            start[1] + projection * dy,
        )
        return math.hypot(point[0] - closest[0], point[1] - closest[1])


class LocalMapBuilder:
    """将 PointCloud2 的 xyz 迭代器转换为局部占据图与八扇区覆盖统计。"""

    def __init__(self, config):
        self.config = config
        self._validate_config()

    def build(self, points):
        occupied_cells = set()
        total_points = 0
        finite_points = 0
        map_points = 0
        height_points = 0
        body_filtered_points = 0
        sector_occupied = {name: 0 for name in SECTOR_NAMES}
        sector_distances = {name: [] for name in SECTOR_NAMES}
        sector_coverage_points = {name: 0 for name in SECTOR_NAMES}
        sector_bins = {name: set() for name in SECTOR_NAMES}

        for point in points:
            total_points += 1
            try:
                x, y, z = (float(point[0]), float(point[1]), float(point[2]))
            except (IndexError, TypeError, ValueError):
                continue
            if not all(math.isfinite(value) for value in (x, y, z)):
                continue
            finite_points += 1
            if not self._inside_map(x, y):
                continue
            if math.hypot(x, y) < self.config.min_range_m:
                continue
            if self._inside_body(x, y):
                body_filtered_points += 1
                continue

            # 覆盖统计在高度过滤前完成：地面回波可以证明该角度被雷达扫描，
            # 但不会被写成挡板占据点。
            angle_deg = math.degrees(math.atan2(y, x))
            sector = classify_sector(angle_deg)
            sector_coverage_points[sector] += 1
            sector_bins[sector].add(self._coverage_bin(angle_deg))

            if z < self.config.z_min_m or z > self.config.z_max_m:
                height_points += 1
                continue

            cell = self._cell_for_point(x, y)
            if cell not in occupied_cells:
                sector_occupied[sector] += 1
            occupied_cells.add(cell)
            sector_distances[sector].append(
                self._sector_distance(sector, x, y)
            )
            map_points += 1

        sector_stats = {}
        for name in SECTOR_NAMES:
            coverage_count = sector_coverage_points[name]
            bin_count = len(sector_bins[name])
            valid = (
                coverage_count
                >= self.config.sector_min_coverage_points
                and bin_count >= self.config.sector_min_coverage_bins
            )
            sector_stats[name] = {
                'point_count': sector_occupied[name],
                'coverage_point_count': coverage_count,
                'coverage_bin_count': bin_count,
                'distance_m': self._percentile(
                    sector_distances[name],
                    10.0,
                ),
                'valid': valid,
            }

        statistics = {
            'total_points': total_points,
            'finite_points': finite_points,
            'map_points': map_points,
            'occupied_cells': len(occupied_cells),
            'height_filtered_points': height_points,
            'body_filtered_points': body_filtered_points,
        }
        wall_segments = self._extract_wall_segments(occupied_cells)
        statistics['wall_segment_count'] = len(wall_segments)
        grid = LocalOccupancyGrid(
            self.config,
            occupied_cells,
            statistics,
            sector_stats,
            wall_segments,
        )
        grid.statistics['unmodeled_obstacle_points'] = len(
            grid.unmodeled_obstacle_points
        )
        return grid

    def _extract_wall_segments(self, occupied_cells):
        """用确定性 RANSAC 提取有限墙段，保留端点和拟合置信度。"""
        points = [
            (
                self.config.x_min_m
                + (cell[0] + 0.5) * self.config.resolution_m,
                self.config.y_min_m
                + (cell[1] + 0.5) * self.config.resolution_m,
            )
            for cell in sorted(occupied_cells)
        ]
        remaining = list(points)
        segments = []
        while (
            len(remaining) >= self.config.wall_min_points
            and len(segments) < self.config.wall_max_segments
        ):
            cluster = self._best_wall_cluster(remaining)
            if cluster is None:
                break
            fitted = self._fit_wall_segment(cluster)
            if fitted is None:
                break
            fitted['id'] = f'wall_{len(segments):03d}'
            segments.append(fitted)

            start = tuple(fitted['start_m'])
            end = tuple(fitted['end_m'])
            # 只移除该有限段附近的内点，交叉处另一方向的墙仍可继续拟合。
            remaining = [
                point
                for point in remaining
                if LocalOccupancyGrid._point_segment_distance(
                    point,
                    start,
                    end,
                ) > self.config.wall_inlier_tolerance_m
            ]
        return tuple(segments)

    def _best_wall_cluster(self, points):
        sample = self._deterministic_sample(
            points,
            self.config.wall_ransac_sample_limit,
        )
        best_cluster = None
        best_score = None
        for first_index in range(len(sample)):
            for second_index in range(first_index + 1, len(sample)):
                first = sample[first_index]
                second = sample[second_index]
                dx = second[0] - first[0]
                dy = second[1] - first[1]
                length = math.hypot(dx, dy)
                if length < self.config.wall_min_length_m:
                    continue
                direction = (dx / length, dy / length)
                normal = (-direction[1], direction[0])
                inliers = []
                for point in points:
                    residual = abs(
                        (point[0] - first[0]) * normal[0]
                        + (point[1] - first[1]) * normal[1]
                    )
                    if residual <= self.config.wall_inlier_tolerance_m:
                        projection = (
                            (point[0] - first[0]) * direction[0]
                            + (point[1] - first[1]) * direction[1]
                        )
                        inliers.append((projection, point))
                for cluster in self._split_projection_clusters(inliers):
                    if len(cluster) < self.config.wall_min_points:
                        continue
                    span = cluster[-1][0] - cluster[0][0]
                    if span < self.config.wall_min_length_m:
                        continue
                    score = (len(cluster), span)
                    if best_score is None or score > best_score:
                        best_cluster = [point for _, point in cluster]
                        best_score = score
        return best_cluster

    def _split_projection_clusters(self, projected_points):
        if not projected_points:
            return ()
        ordered = sorted(projected_points, key=lambda item: item[0])
        clusters = [[ordered[0]]]
        for item in ordered[1:]:
            if item[0] - clusters[-1][-1][0] > self.config.wall_cluster_gap_m:
                clusters.append([item])
            else:
                clusters[-1].append(item)
        return tuple(tuple(cluster) for cluster in clusters)

    def _fit_wall_segment(self, points):
        count = len(points)
        center_x = sum(point[0] for point in points) / count
        center_y = sum(point[1] for point in points) / count
        covariance_xx = sum(
            (point[0] - center_x) ** 2 for point in points
        ) / count
        covariance_yy = sum(
            (point[1] - center_y) ** 2 for point in points
        ) / count
        covariance_xy = sum(
            (point[0] - center_x) * (point[1] - center_y)
            for point in points
        ) / count
        angle = 0.5 * math.atan2(
            2.0 * covariance_xy,
            covariance_xx - covariance_yy,
        )
        direction = (math.cos(angle), math.sin(angle))
        normal = (-direction[1], direction[0])
        projections = [
            (point[0] - center_x) * direction[0]
            + (point[1] - center_y) * direction[1]
            for point in points
        ]
        residuals = [
            abs(
                (point[0] - center_x) * normal[0]
                + (point[1] - center_y) * normal[1]
            )
            for point in points
        ]
        start_projection = min(projections)
        end_projection = max(projections)
        length = end_projection - start_projection
        residual_rms = math.sqrt(
            sum(value * value for value in residuals) / count
        )
        if (
            count < self.config.wall_min_points
            or length < self.config.wall_min_length_m
            or residual_rms > self.config.wall_max_residual_m
        ):
            return None
        start = (
            center_x + start_projection * direction[0],
            center_y + start_projection * direction[1],
        )
        end = (
            center_x + end_projection * direction[0],
            center_y + end_projection * direction[1],
        )
        point_score = min(
            1.0,
            count / max(1.0, 2.0 * self.config.wall_min_points),
        )
        length_score = min(
            1.0,
            length / max(1.0e-6, 2.0 * self.config.wall_min_length_m),
        )
        residual_score = max(
            0.0,
            1.0 - residual_rms / self.config.wall_max_residual_m,
        )
        confidence = point_score * length_score * residual_score
        endpoint_uncertainty = max(
            self.config.wall_endpoint_uncertainty_m,
            self.config.resolution_m,
            2.0 * residual_rms,
        )
        return {
            'id': '',
            'start_m': start,
            'end_m': end,
            'length_m': length,
            'angle_rad': angle,
            'angle_deg': math.degrees(angle),
            'point_count': count,
            'residual_rms_m': residual_rms,
            'confidence': confidence,
            'endpoint_uncertainty_m': endpoint_uncertainty,
        }

    @staticmethod
    def _deterministic_sample(points, limit):
        if len(points) <= limit:
            return list(points)
        indexes = {
            int(round(index * (len(points) - 1) / (limit - 1)))
            for index in range(limit)
        }
        return [points[index] for index in sorted(indexes)]

    def _inside_map(self, x, y):
        cfg = self.config
        return (
            cfg.x_min_m <= x <= cfg.x_max_m
            and cfg.y_min_m <= y <= cfg.y_max_m
        )

    def _inside_body(self, x, y):
        cfg = self.config
        return (
            cfg.body_x_min_m <= x <= cfg.body_x_max_m
            and cfg.body_y_min_m <= y <= cfg.body_y_max_m
        )

    def _cell_for_point(self, x, y):
        cfg = self.config
        return (
            int(math.floor((x - cfg.x_min_m) / cfg.resolution_m)),
            int(math.floor((y - cfg.y_min_m) / cfg.resolution_m)),
        )

    def _coverage_bin(self, angle_deg):
        return int(math.floor(
            (angle_deg + 180.0) / self.config.coverage_bin_deg
        ))

    @staticmethod
    def _sector_distance(sector, x, y):
        """前后使用纵向距离、正侧使用横向距离、斜区使用径向距离。"""
        if sector in ('front', 'rear'):
            return abs(x)
        if sector in ('left', 'right'):
            return abs(y)
        return math.hypot(x, y)

    @staticmethod
    def _percentile(values, percentile):
        if not values:
            return None
        ordered = sorted(values)
        rank = (len(ordered) - 1) * float(percentile) / 100.0
        low = int(math.floor(rank))
        high = int(math.ceil(rank))
        if low == high:
            return ordered[low]
        fraction = rank - low
        return ordered[low] * (1.0 - fraction) + ordered[high] * fraction

    def _validate_config(self):
        cfg = self.config
        finite_values = (
            cfg.x_min_m,
            cfg.x_max_m,
            cfg.y_min_m,
            cfg.y_max_m,
            cfg.resolution_m,
            cfg.z_min_m,
            cfg.z_max_m,
            cfg.body_x_min_m,
            cfg.body_x_max_m,
            cfg.body_y_min_m,
            cfg.body_y_max_m,
            cfg.min_range_m,
            cfg.coverage_bin_deg,
            cfg.wall_inlier_tolerance_m,
            cfg.wall_cluster_gap_m,
            cfg.wall_min_length_m,
            cfg.wall_max_residual_m,
            cfg.wall_endpoint_uncertainty_m,
        )
        if not all(math.isfinite(value) for value in finite_values):
            raise ValueError('local map parameters must be finite')
        if not cfg.x_min_m < cfg.x_max_m:
            raise ValueError('x_min_m must be less than x_max_m')
        if not cfg.y_min_m < cfg.y_max_m:
            raise ValueError('y_min_m must be less than y_max_m')
        if not cfg.z_min_m < cfg.z_max_m:
            raise ValueError('z_min_m must be less than z_max_m')
        if not cfg.body_x_min_m < cfg.body_x_max_m:
            raise ValueError(
                'body_x_min_m must be less than body_x_max_m'
            )
        if not cfg.body_y_min_m < cfg.body_y_max_m:
            raise ValueError(
                'body_y_min_m must be less than body_y_max_m'
            )
        if cfg.resolution_m <= 0.0:
            raise ValueError('resolution_m must be positive')
        if cfg.min_range_m < 0.0:
            raise ValueError('min_range_m must be nonnegative')
        if not 0.0 < cfg.coverage_bin_deg <= 45.0:
            raise ValueError('coverage_bin_deg must be in (0, 45]')
        if cfg.sector_min_coverage_points <= 0:
            raise ValueError('sector_min_coverage_points must be positive')
        if cfg.sector_min_coverage_bins <= 0:
            raise ValueError('sector_min_coverage_bins must be positive')
        if cfg.wall_inlier_tolerance_m <= 0.0:
            raise ValueError('wall_inlier_tolerance_m must be positive')
        if cfg.wall_cluster_gap_m <= 0.0:
            raise ValueError('wall_cluster_gap_m must be positive')
        if cfg.wall_min_points <= 1:
            raise ValueError('wall_min_points must be greater than one')
        if cfg.wall_min_length_m <= 0.0:
            raise ValueError('wall_min_length_m must be positive')
        if cfg.wall_max_residual_m <= 0.0:
            raise ValueError('wall_max_residual_m must be positive')
        if cfg.wall_max_segments <= 0:
            raise ValueError('wall_max_segments must be positive')
        if cfg.wall_ransac_sample_limit < 2:
            raise ValueError('wall_ransac_sample_limit must be at least two')
        if cfg.wall_endpoint_uncertainty_m < 0.0:
            raise ValueError(
                'wall_endpoint_uncertainty_m must be nonnegative'
            )


class SectorFreshnessTracker:
    """独立记录每个角区最后一次有效覆盖，禁止用前向帧龄代替后向帧龄。"""

    def __init__(self, stale_timeout_sec):
        self.stale_timeout_sec = float(stale_timeout_sec)
        if (
            not math.isfinite(self.stale_timeout_sec)
            or self.stale_timeout_sec <= 0.0
        ):
            raise ValueError('sector stale timeout must be positive')
        self._last_valid_time = {name: None for name in SECTOR_NAMES}
        self._latest_stats = {
            name: {
                'point_count': 0,
                'coverage_point_count': 0,
                'coverage_bin_count': 0,
                'distance_m': None,
                'valid': False,
            }
            for name in SECTOR_NAMES
        }

    def update(self, sector_stats, now):
        for name in SECTOR_NAMES:
            stats = dict(sector_stats[name])
            self._latest_stats[name] = stats
            if stats.get('valid') is True:
                self._last_valid_time[name] = float(now)

    def snapshot(self, now):
        result = {}
        for name in SECTOR_NAMES:
            last_time = self._last_valid_time[name]
            age = None if last_time is None else max(0.0, now - last_time)
            stats = dict(self._latest_stats[name])
            stats['age_sec'] = age
            stats['stale'] = (
                age is None or age > self.stale_timeout_sec
            )
            stats['usable'] = (
                stats.get('valid') is True
                and stats['stale'] is False
            )
            result[name] = stats
        return result


class TrajectoryGenerator:
    """按常值机体系速度生成连续短段，并追加停止尾程采样。"""

    def __init__(self, config, footprint):
        self.config = config
        self.footprint = footprint

    def generate(self, primitive):
        self._validate_primitive(primitive)
        linear_speed = math.hypot(primitive.vx_mps, primitive.vy_mps)
        expanded = self.footprint.expanded_extents()
        corner_radius = math.hypot(
            max(expanded['front'], expanded['rear']),
            max(expanded['left'], expanded['right']),
        )
        center_path_m = linear_speed * primitive.duration_sec
        corner_rotation_path_m = (
            corner_radius
            * abs(primitive.wz_radps)
            * primitive.duration_sec
        )
        sample_count = max(
            1,
            int(math.ceil(
                primitive.duration_sec
                / self.config.trajectory_sample_dt_sec
            )),
            int(math.ceil(
                (center_path_m + corner_rotation_path_m)
                / self.config.trajectory_max_translation_step_m
            )),
            int(math.ceil(
                abs(primitive.wz_radps) * primitive.duration_sec
                / math.radians(self.config.trajectory_max_yaw_step_deg)
            )),
        )
        poses = [
            self._pose_at(
                primitive,
                primitive.duration_sec * index / sample_count,
            )
            for index in range(sample_count + 1)
        ]

        # StopMove 不是零距离瞬停。沿末端速度方向追加实测尾程，确保候选
        # 在命令结束后仍不会让扩大足迹撞到挡板。
        if linear_speed > 1.0e-9 and self.footprint.stop_tail_margin_m > 0.0:
            tail_steps = max(
                1,
                int(math.ceil(
                    self.footprint.stop_tail_margin_m
                    / self.config.trajectory_max_translation_step_m
                )),
            )
            end_pose = poses[-1]
            direction = math.atan2(primitive.vy_mps, primitive.vx_mps)
            global_direction = end_pose.yaw_rad + direction
            for index in range(1, tail_steps + 1):
                distance = (
                    self.footprint.stop_tail_margin_m
                    * index / tail_steps
                )
                poses.append(Pose2D(
                    end_pose.x_m + distance * math.cos(global_direction),
                    end_pose.y_m + distance * math.sin(global_direction),
                    end_pose.yaw_rad,
                    primitive.duration_sec
                    + distance / max(linear_speed, 1.0e-9),
                ))
        return tuple(poses)

    @staticmethod
    def _pose_at(primitive, time_sec):
        yaw = primitive.wz_radps * time_sec
        if abs(primitive.wz_radps) <= 1.0e-9:
            return Pose2D(
                primitive.vx_mps * time_sec,
                primitive.vy_mps * time_sec,
                yaw,
                time_sec,
            )
        wz = primitive.wz_radps
        sin_yaw = math.sin(yaw)
        cos_yaw = math.cos(yaw)
        x_m = (
            primitive.vx_mps * sin_yaw
            - primitive.vy_mps * (1.0 - cos_yaw)
        ) / wz
        y_m = (
            primitive.vx_mps * (1.0 - cos_yaw)
            + primitive.vy_mps * sin_yaw
        ) / wz
        return Pose2D(x_m, y_m, yaw, time_sec)

    @staticmethod
    def _validate_primitive(primitive):
        values = (
            primitive.vx_mps,
            primitive.vy_mps,
            primitive.wz_radps,
            primitive.duration_sec,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError(f'{primitive.name} contains NaN or Inf')
        if primitive.duration_sec <= 0.0:
            raise ValueError(f'{primitive.name} duration must be positive')
        if (
            abs(primitive.vx_mps) <= 1.0e-9
            and abs(primitive.vy_mps) <= 1.0e-9
            and abs(primitive.wz_radps) > 1.0e-9
        ):
            raise ValueError('in-place rotation is forbidden')


class SweptFootprintChecker:
    """在轨迹每个采样位姿检查非对称矩形足迹及最危险时刻。"""

    def __init__(self, footprint, config):
        self.footprint = footprint
        self.config = config

    def check(
        self,
        grid,
        poses,
        command_duration_sec=None,
        include_modeled_points=True,
    ):
        extents = self.footprint.expanded_extents()
        radius = math.hypot(
            max(extents['front'], extents['rear']),
            max(extents['left'], extents['right']),
        ) + self.config.clearance_search_m
        minimum_clearance = self.config.clearance_search_m
        dangerous_time = 0.0
        dangerous_part = 'none'
        dangerous_point = None
        dangerous_pose = None
        dangerous_geometry_type = 'none'
        dangerous_wall_segment_id = None
        dangerous_wall_endpoint = None
        collision = False
        censored = True
        first_unsafe_time = None
        first_unsafe_pose = None
        first_unsafe_part = None
        first_unsafe_wall_segment_id = None
        first_unsafe_geometry_type = None
        first_unsafe_point = None
        first_unsafe_by_part = {}
        minimum_endpoint_clearance = self.config.clearance_search_m
        minimum_stop_tail_clearance = self.config.clearance_search_m

        for pose in poses:
            cos_yaw = math.cos(pose.yaw_rad)
            sin_yaw = math.sin(pose.yaw_rad)
            pose_minimum = self.config.clearance_search_m
            pose_part = 'none'
            pose_wall_segment_id = None
            pose_geometry_type = 'none'
            pose_danger_point = None
            for evidence in self._pose_evidence(
                grid,
                pose,
                radius,
                cos_yaw,
                sin_yaw,
                extents,
                include_modeled_points,
            ):
                point_x, point_y = evidence['point_m']
                clearance = evidence['clearance_m']
                part = evidence['part']
                inside = evidence['inside']
                evidence_uncertainty = max(
                    0.0,
                    float(evidence.get('uncertainty_m', 0.0)),
                )
                clearance = max(0.0, clearance - evidence_uncertainty)
                inside = inside or clearance <= 0.0
                if clearance <= minimum_clearance:
                    minimum_clearance = clearance
                    dangerous_time = pose.time_sec
                    dangerous_part = part
                    dangerous_point = (point_x, point_y)
                    dangerous_pose = pose_to_dict(pose)
                    dangerous_geometry_type = evidence['geometry_type']
                    dangerous_wall_segment_id = evidence.get(
                        'wall_segment_id'
                    )
                    dangerous_wall_endpoint = evidence.get('wall_endpoint')
                    censored = False
                if evidence['geometry_type'] == 'wall_endpoint':
                    minimum_endpoint_clearance = min(
                        minimum_endpoint_clearance,
                        clearance,
                    )
                if (
                    clearance < self.footprint.target_physical_clearance_m
                    and part not in first_unsafe_by_part
                ):
                    first_unsafe_by_part[part] = {
                        'time_sec': pose.time_sec,
                        'pose': pose_to_dict(pose),
                        'clearance_m': clearance,
                        'geometry_type': evidence['geometry_type'],
                        'wall_segment_id': evidence.get('wall_segment_id'),
                        'wall_endpoint': evidence.get('wall_endpoint'),
                        'point_m': (point_x, point_y),
                    }
                if (
                    command_duration_sec is not None
                    and pose.time_sec > command_duration_sec
                ):
                    minimum_stop_tail_clearance = min(
                        minimum_stop_tail_clearance,
                        clearance,
                    )
                if clearance < pose_minimum:
                    pose_minimum = clearance
                    pose_part = part
                    pose_wall_segment_id = evidence.get('wall_segment_id')
                    pose_geometry_type = evidence['geometry_type']
                    pose_danger_point = (point_x, point_y)
                if inside:
                    collision = True
            if (
                first_unsafe_time is None
                and pose_minimum
                < self.footprint.target_physical_clearance_m
            ):
                first_unsafe_time = pose.time_sec
                first_unsafe_pose = pose_to_dict(pose)
                first_unsafe_part = pose_part
                first_unsafe_wall_segment_id = pose_wall_segment_id
                first_unsafe_geometry_type = pose_geometry_type
                first_unsafe_point = pose_danger_point

        return {
            'collision': collision,
            'minimum_clearance_m': minimum_clearance,
            'clearance_censored': censored,
            'danger_time_sec': dangerous_time,
            'collision_part': dangerous_part,
            'danger_point_m': dangerous_point,
            'danger_pose': dangerous_pose,
            'danger_geometry_type': dangerous_geometry_type,
            'danger_wall_segment_id': dangerous_wall_segment_id,
            'danger_wall_endpoint': dangerous_wall_endpoint,
            'minimum_wall_endpoint_clearance_m': (
                minimum_endpoint_clearance
            ),
            'wall_endpoint_clearance_censored': (
                minimum_endpoint_clearance == self.config.clearance_search_m
            ),
            'minimum_stop_tail_clearance_m': minimum_stop_tail_clearance,
            'first_unsafe_time_sec': first_unsafe_time,
            'first_unsafe_pose': first_unsafe_pose,
            'first_unsafe_part': first_unsafe_part,
            'first_unsafe_wall_segment_id': first_unsafe_wall_segment_id,
            'first_unsafe_geometry_type': first_unsafe_geometry_type,
            'first_unsafe_point_m': first_unsafe_point,
            'first_unsafe_by_part': first_unsafe_by_part,
            'sample_count': len(poses),
        }

    def _pose_evidence(
        self,
        grid,
        pose,
        radius,
        cos_yaw,
        sin_yaw,
        extents,
        include_modeled_points,
    ):
        """解析检查原始点、有限墙段及墙端，避免反复离散整条墙。"""
        for point_x, point_y in grid.nearby_points(
            pose,
            radius,
            include_modeled_points=include_modeled_points,
        ):
            local_x, local_y = self._to_local(
                point_x,
                point_y,
                pose,
                cos_yaw,
                sin_yaw,
            )
            clearance, part, inside = self._point_clearance(
                local_x,
                local_y,
                extents,
            )
            yield {
                'point_m': (point_x, point_y),
                'clearance_m': clearance,
                'part': part,
                'inside': inside,
                'geometry_type': 'obstacle_point',
                'wall_segment_id': grid.obstacle_wall_segment_ids.get(
                    (point_x, point_y)
                ),
                'wall_endpoint': None,
                'uncertainty_m': 0.0,
            }

        for segment in grid.wall_segments:
            start_global = tuple(segment['start_m'])
            end_global = tuple(segment['end_m'])
            if LocalOccupancyGrid._point_segment_distance(
                (pose.x_m, pose.y_m),
                start_global,
                end_global,
            ) > radius:
                continue
            start_local = self._to_local(
                start_global[0],
                start_global[1],
                pose,
                cos_yaw,
                sin_yaw,
            )
            end_local = self._to_local(
                end_global[0],
                end_global[1],
                pose,
                cos_yaw,
                sin_yaw,
            )
            clearance, part, inside, local_point = (
                self._segment_rectangle_clearance(
                    start_local,
                    end_local,
                    extents,
                )
            )
            yield {
                'point_m': self._to_global(
                    local_point,
                    pose,
                    cos_yaw,
                    sin_yaw,
                ),
                'clearance_m': clearance,
                'part': part,
                'inside': inside,
                'geometry_type': 'wall_segment',
                'wall_segment_id': segment['id'],
                'wall_endpoint': None,
                'uncertainty_m': float(segment['residual_rms_m']),
            }

            for endpoint_name, endpoint_global, endpoint_local in (
                ('start', start_global, start_local),
                ('end', end_global, end_local),
            ):
                endpoint_clearance, endpoint_part, endpoint_inside = (
                    self._point_clearance(
                        endpoint_local[0],
                        endpoint_local[1],
                        extents,
                    )
                )
                yield {
                    'point_m': endpoint_global,
                    'clearance_m': endpoint_clearance,
                    'part': endpoint_part,
                    'inside': endpoint_inside,
                    'geometry_type': 'wall_endpoint',
                    'wall_segment_id': segment['id'],
                    'wall_endpoint': endpoint_name,
                    'uncertainty_m': float(
                        segment['endpoint_uncertainty_m']
                    ),
                }

    @staticmethod
    def _to_local(point_x, point_y, pose, cos_yaw, sin_yaw):
        dx = point_x - pose.x_m
        dy = point_y - pose.y_m
        return (
            cos_yaw * dx + sin_yaw * dy,
            -sin_yaw * dx + cos_yaw * dy,
        )

    @staticmethod
    def _to_global(local_point, pose, cos_yaw, sin_yaw):
        return (
            pose.x_m
            + cos_yaw * local_point[0]
            - sin_yaw * local_point[1],
            pose.y_m
            + sin_yaw * local_point[0]
            + cos_yaw * local_point[1],
        )

    @classmethod
    def _segment_rectangle_clearance(cls, start, end, extents):
        """计算有限线段到轴对齐矩形的精确二维距离和危险边。"""
        intersection = cls._segment_rectangle_intersection(
            start,
            end,
            extents,
        )
        if intersection is not None:
            return (
                0.0,
                cls._contact_part(intersection[0], intersection[1], extents),
                True,
                intersection,
            )

        candidates = []
        for endpoint in (start, end):
            clearance, part, _ = cls._point_clearance(
                endpoint[0],
                endpoint[1],
                extents,
            )
            candidates.append((clearance, part, endpoint))

        corners = (
            (extents['front'], extents['left']),
            (extents['front'], -extents['right']),
            (-extents['rear'], extents['left']),
            (-extents['rear'], -extents['right']),
        )
        for corner in corners:
            closest = cls._closest_point_on_segment(corner, start, end)
            clearance = math.hypot(
                corner[0] - closest[0],
                corner[1] - closest[1],
            )
            candidates.append((
                clearance,
                cls._contact_part(corner[0], corner[1], extents),
                closest,
            ))
        clearance, part, wall_point = min(
            candidates,
            key=lambda item: item[0],
        )
        return clearance, part, False, wall_point

    @staticmethod
    def _segment_rectangle_intersection(start, end, extents):
        """Liang-Barsky 裁剪；返回最先进入矩形的线段点。"""
        x_min = -extents['rear']
        x_max = extents['front']
        y_min = -extents['right']
        y_max = extents['left']
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        lower = 0.0
        upper = 1.0
        for coefficient, offset in (
            (-dx, start[0] - x_min),
            (dx, x_max - start[0]),
            (-dy, start[1] - y_min),
            (dy, y_max - start[1]),
        ):
            if abs(coefficient) <= 1.0e-12:
                if offset < 0.0:
                    return None
                continue
            ratio = offset / coefficient
            if coefficient < 0.0:
                lower = max(lower, ratio)
            else:
                upper = min(upper, ratio)
            if lower > upper:
                return None
        return (
            start[0] + lower * dx,
            start[1] + lower * dy,
        )

    @staticmethod
    def _closest_point_on_segment(point, start, end):
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        length_squared = dx * dx + dy * dy
        if length_squared <= 1.0e-12:
            return start
        ratio = (
            (point[0] - start[0]) * dx
            + (point[1] - start[1]) * dy
        ) / length_squared
        ratio = max(0.0, min(1.0, ratio))
        return (
            start[0] + ratio * dx,
            start[1] + ratio * dy,
        )

    @staticmethod
    def _point_clearance(local_x, local_y, extents):
        outside_x = max(
            -extents['rear'] - local_x,
            0.0,
            local_x - extents['front'],
        )
        outside_y = max(
            -extents['right'] - local_y,
            0.0,
            local_y - extents['left'],
        )
        inside = outside_x == 0.0 and outside_y == 0.0
        if not inside:
            part = SweptFootprintChecker._contact_part(
                local_x,
                local_y,
                extents,
            )
            return math.hypot(outside_x, outside_y), part, False

        part = SweptFootprintChecker._contact_part(
            local_x,
            local_y,
            extents,
        )
        return 0.0, part, True

    @staticmethod
    def _contact_part(local_x, local_y, extents):
        """按危险点相对矩形的位置输出固定的角点或边名称。"""
        longitudinal_name = 'front' if local_x >= 0.0 else 'rear'
        lateral_name = 'left' if local_y >= 0.0 else 'right'
        longitudinal_extent = extents[longitudinal_name]
        lateral_extent = extents[lateral_name]
        longitudinal_ratio = abs(local_x) / max(longitudinal_extent, 1.0e-9)
        lateral_ratio = abs(local_y) / max(lateral_extent, 1.0e-9)
        if longitudinal_ratio >= 0.65 and lateral_ratio >= 0.65:
            return f'{longitudinal_name}_{lateral_name}'
        if longitudinal_ratio >= lateral_ratio:
            return f'{longitudinal_name}_edge'
        return f'{lateral_name}_side'


class FirstTurnTrajectoryPlanner:
    """只生成第一左弯六类候选，并按完整安全门控排序。"""

    REQUIRED_SECTORS = {
        PRIMITIVE_FORWARD: (
            'front', 'left_front', 'right_front', 'left', 'right',
        ),
        PRIMITIVE_OUTSIDE_DIAGONAL: (
            'front', 'left_front', 'right_front', 'left', 'right',
        ),
        PRIMITIVE_LEFT_ARC: SECTOR_NAMES,
        PRIMITIVE_LEFT_ARC_OUTSIDE: SECTOR_NAMES,
        PRIMITIVE_REVERSE: (
            'rear', 'left_rear', 'right_rear', 'left', 'right',
        ),
        PRIMITIVE_FINE_LEFT_ARC: SECTOR_NAMES,
    }

    def __init__(self, config, footprint, primitives):
        self.config = config
        self.footprint = footprint
        self.primitives = {
            primitive.name: primitive
            for primitive in primitives
        }
        missing = set(PRIMITIVE_ORDER) - set(self.primitives)
        if missing:
            raise ValueError(
                'missing motion primitives: ' + ', '.join(sorted(missing))
            )
        self._validate_configuration()
        self.generator = TrajectoryGenerator(config, footprint)
        self.checker = SweptFootprintChecker(footprint, config)
        self.reverse_segments = 0
        self.reverse_distance_m = 0.0

    def plan(self, grid, safety, phase=STATE_SELECT_TRAJECTORY):
        # 每帧始终评估六类第一弯候选，再按当前阶段限制可选择集合。
        # 这样日志能稳定输出安全优先的前五名，而不是只显示被选中的一条。
        evaluations = []
        for name in PRIMITIVE_ORDER:
            evaluations.append(self.evaluate_candidate(
                self.primitives[name],
                grid,
                safety,
            ))
        ranked = self._rank_candidates(evaluations)
        allowed_names = set(self._candidate_names_for_phase(phase))
        selected = self._select(ranked, allowed_names)
        rear_unavailable = [
            name
            for name in ('rear', 'left_rear', 'right_rear')
            if safety.sector_status.get(name, {}).get('usable') is not True
        ]
        return {
            'selected': selected,
            'candidates': ranked,
            'top_candidates': ranked[:5],
            'has_robust_safe': any(
                item['verdict'] == VERDICT_ROBUST_SAFE
                for item in ranked
            ),
            'has_dry_run_safe': any(
                item['verdict']
                in (
                    VERDICT_GEOMETRY_SAFE_UNCALIBRATED,
                    VERDICT_NOMINAL_SAFE,
                    VERDICT_ROBUST_SAFE,
                )
                for item in ranked
            ),
            'rear_coverage_status': (
                'SUFFICIENT'
                if not rear_unavailable
                else 'rear_coverage_insufficient'
            ),
            'rear_unavailable_sectors': rear_unavailable,
        }

    def current_footprint_safety(self, grid):
        """检查当前静止足迹，供状态机执行硬距离近障保护。"""
        return self.checker.check(
            grid,
            (Pose2D(0.0, 0.0, 0.0, 0.0),),
            command_duration_sec=0.0,
        )

    def evaluate_candidate(self, primitive, grid, safety):
        poses = self.generator.generate(primitive)
        sweep = self.checker.check(
            grid,
            poses,
            command_duration_sec=primitive.duration_sec,
        )
        unsafe_reasons = []
        unknown_reasons = []
        execution_blockers = []

        if safety.cloud_age_sec > self.config.cloud_stale_timeout_sec:
            unknown_reasons.append('cloud_stale')
        if safety.odom_age_sec > self.config.odom_stale_timeout_sec:
            unknown_reasons.append('odom_stale')
        if not safety.cloud_valid:
            unknown_reasons.append('cloud_invalid')
        if not safety.odom_valid:
            unknown_reasons.append('odom_invalid')
        if safety.yaw_jump:
            unknown_reasons.append('yaw_jump')
        if not self._posture_stable(safety):
            unknown_reasons.append('posture_unstable')
        if not safety.stop_stable:
            unknown_reasons.append('stop_not_stable')

        unavailable = []
        for name in self.REQUIRED_SECTORS[primitive.name]:
            status = safety.sector_status.get(name, {})
            if status.get('usable') is not True:
                unavailable.append(name)
        if unavailable:
            unknown_reasons.append(
                'sector_unavailable:' + ','.join(unavailable)
            )

        if primitive.name == PRIMITIVE_REVERSE:
            reverse_unsafe, reverse_unknown = self._reverse_reasons(
                primitive,
                safety,
            )
            unsafe_reasons.extend(reverse_unsafe)
            unknown_reasons.extend(reverse_unknown)

        if sweep['collision']:
            unsafe_reasons.append('predicted_collision')
        elif sweep['minimum_clearance_m'] < self.config.hard_clearance_m:
            unsafe_reasons.append('hard_clearance_too_small')
        elif (
            sweep['minimum_clearance_m']
            < self.footprint.target_physical_clearance_m
        ):
            unsafe_reasons.append('target_physical_clearance_not_met')

        reliable_walls = [
            segment
            for segment in grid.wall_segments
            if segment['confidence'] >= self.config.wall_confidence_min
        ]
        is_turning = primitive.name in (
            PRIMITIVE_LEFT_ARC,
            PRIMITIVE_LEFT_ARC_OUTSIDE,
            PRIMITIVE_FINE_LEFT_ARC,
        )
        wall_model_reliable = (
            not is_turning
            or len(reliable_walls)
            >= self.config.minimum_wall_segments_for_turn
        )
        if not wall_model_reliable:
            unknown_reasons.append('wall_model_insufficient')

        left_distance = safety.sector_status.get('left', {}).get('distance_m')
        legacy_guard_pass = (
            left_distance is not None
            and float(left_distance)
            >= self.config.legacy_turn_sweep_clearance_m
        )
        legacy_geometry_override = False
        if is_turning and not legacy_guard_pass:
            if wall_model_reliable:
                # 可靠有限墙端允许几何模型替代过度保守的固定半径，但必须
                # 保留并输出旧门限比较，不能把它从安全证据中删除。
                legacy_geometry_override = True
            else:
                unknown_reasons.append(
                    'legacy_turn_guard_failed_without_reliable_geometry'
                )

        perturbation_sweeps = []
        for scenario_name, perturbed in self._perturbed_primitives(primitive):
            perturbed_poses = self.generator.generate(perturbed)
            result = self.checker.check(
                grid,
                perturbed_poses,
                command_duration_sec=perturbed.duration_sec,
                # 名义轨迹已逐点复核全部降采样点；扰动轨迹继续检查墙段、
                # 墙端及所有未被墙模型解释的独立障碍，避免重复扫描墙点。
                include_modeled_points=False,
            )
            scenario_safe = (
                not result['collision']
                and result['minimum_clearance_m']
                >= self.footprint.target_physical_clearance_m
            )
            perturbation_sweeps.append({
                'scenario': scenario_name,
                'safe': scenario_safe,
                'minimum_clearance_m': result['minimum_clearance_m'],
                'danger_time_sec': result['danger_time_sec'],
                'danger_pose': result['danger_pose'],
                'danger_part': result['collision_part'],
                'danger_wall_segment_id': result['danger_wall_segment_id'],
            })
        robust_pass_count = sum(
            1 for item in perturbation_sweeps if item['safe']
        )
        robust_total = len(perturbation_sweeps)
        all_perturbations_safe = robust_pass_count == robust_total

        if not primitive.calibrated:
            execution_blockers.append('motion_model_uncalibrated')
        if not self.footprint.margins_calibrated:
            execution_blockers.append('dynamic_margins_uncalibrated')
        if safety.watchdog_ok is not True:
            execution_blockers.append('watchdog_unverified')
        if safety.estop_triggered is not False:
            execution_blockers.append('estop_unverified')
        if not self._safety_status_fresh(safety.watchdog_age_sec):
            execution_blockers.append('watchdog_status_stale')
        if not self._safety_status_fresh(safety.estop_age_sec):
            execution_blockers.append('estop_status_stale')

        if unsafe_reasons:
            verdict = VERDICT_UNSAFE
        elif unknown_reasons:
            verdict = VERDICT_UNKNOWN
        elif not primitive.calibrated or not self.footprint.margins_calibrated:
            verdict = VERDICT_GEOMETRY_SAFE_UNCALIBRATED
        elif not all_perturbations_safe:
            verdict = VERDICT_NOMINAL_SAFE
        else:
            verdict = VERDICT_ROBUST_SAFE

        predicted_end = pose_to_dict(poses[-1])
        final_yaw_error = abs(
            math.radians(self.config.turn_target_deg)
            - poses[-1].yaw_rad
        ) if is_turning else abs(poses[-1].yaw_rad)
        return {
            'name': primitive.name,
            'verdict': verdict,
            'blockers': unsafe_reasons,
            'unknown_reasons': unknown_reasons,
            'unverified': execution_blockers,
            'execution_blockers': execution_blockers,
            # B2.1-A 永远禁止 EXECUTABLE_SAFE，后续执行层必须另行验收。
            'executable_safe': False,
            'calibrated': primitive.calibrated,
            'calibration_id': primitive.calibration_id,
            'vx_mps': primitive.vx_mps,
            'vy_mps': primitive.vy_mps,
            'wz_radps': primitive.wz_radps,
            'duration_sec': primitive.duration_sec,
            'predicted_end': predicted_end,
            'sweep': sweep,
            'perturbation_sweeps': perturbation_sweeps,
            'robustness_pass_count': robust_pass_count,
            'robustness_total': robust_total,
            'robustness_ratio': (
                robust_pass_count / robust_total if robust_total else 0.0
            ),
            'wall_model_reliable': wall_model_reliable,
            'reliable_wall_segment_count': len(reliable_walls),
            'legacy_0413_guard_pass': legacy_guard_pass,
            'legacy_0413_geometry_override': legacy_geometry_override,
            'legacy_0413_clearance_m': (
                float(left_distance) if left_distance is not None else None
            ),
            'final_yaw_error_deg': math.degrees(final_yaw_error),
            'final_lateral_error_m': abs(poses[-1].y_m),
            'required_sectors': list(self.REQUIRED_SECTORS[primitive.name]),
        }

    def record_reverse_execution(self, primitive_name):
        """仅由未来执行层在真实完成一个后退脉冲后调用。"""
        if primitive_name != PRIMITIVE_REVERSE:
            raise ValueError('only REVERSE_SHORT increments recovery limits')
        primitive = self.primitives[primitive_name]
        self.reverse_segments += 1
        self.reverse_distance_m += abs(
            primitive.vx_mps * primitive.duration_sec
        )

    def _reverse_reasons(self, primitive, safety):
        unsafe = []
        unknown = []
        if self.reverse_segments >= self.config.max_reverse_segments:
            unsafe.append('reverse_segment_limit')
        predicted = abs(primitive.vx_mps * primitive.duration_sec)
        if (
            self.reverse_distance_m + predicted
            > self.config.max_reverse_distance_m
        ):
            unsafe.append('reverse_distance_limit')
        if safety.watchdog_ok is not True:
            unknown.append('reverse_watchdog_not_ok')
        if safety.estop_triggered is not False:
            unknown.append('reverse_estop_not_clear')
        if not self._posture_stable(safety):
            unknown.append('reverse_posture_unstable')
        return unsafe, unknown

    def _perturbed_primitives(self, primitive):
        """生成速度、曲率和持续时间的独立边界扰动场景。"""
        vx_delta = abs(primitive.vx_mps) * self.config.trajectory_vx_uncertainty_ratio
        wz_delta = abs(primitive.wz_radps) * self.config.trajectory_wz_uncertainty_ratio
        vy_delta = self.config.trajectory_vy_uncertainty_mps
        duration_delta = self.config.trajectory_duration_uncertainty_sec
        scenarios = (
            ('vx_low', replace(
                primitive,
                vx_mps=primitive.vx_mps - math.copysign(vx_delta, primitive.vx_mps),
            )),
            ('vx_high', replace(
                primitive,
                vx_mps=primitive.vx_mps + math.copysign(vx_delta, primitive.vx_mps),
            )),
            ('vy_left_bias', replace(
                primitive,
                vy_mps=primitive.vy_mps + vy_delta,
            )),
            ('vy_right_bias', replace(
                primitive,
                vy_mps=primitive.vy_mps - vy_delta,
            )),
            ('wz_low', replace(
                primitive,
                wz_radps=primitive.wz_radps - math.copysign(
                    wz_delta,
                    primitive.wz_radps,
                ) if abs(primitive.wz_radps) > 1.0e-9 else 0.0,
            )),
            ('wz_high', replace(
                primitive,
                wz_radps=primitive.wz_radps + math.copysign(
                    wz_delta,
                    primitive.wz_radps,
                ) if abs(primitive.wz_radps) > 1.0e-9 else 0.0,
            )),
            ('duration_long', replace(
                primitive,
                duration_sec=primitive.duration_sec + duration_delta,
            )),
        )
        return scenarios

    def _posture_stable(self, safety):
        return (
            abs(math.degrees(safety.roll_rad)) <= self.config.max_roll_deg
            and abs(math.degrees(safety.pitch_rad)) <= self.config.max_pitch_deg
            and safety.linear_speed_mps
            <= self.config.stationary_linear_speed_mps
            and abs(safety.angular_speed_radps)
            <= self.config.stationary_angular_speed_radps
        )

    def _safety_status_fresh(self, age):
        return (
            age is not None
            and math.isfinite(float(age))
            and 0.0 <= float(age)
            <= self.config.safety_status_stale_timeout_sec
        )

    def _validate_configuration(self):
        """拒绝会削弱连续扫掠或把第一左弯方向写反的配置。"""
        footprint_values = (
            self.footprint.footprint_front_m,
            self.footprint.footprint_rear_m,
            self.footprint.footprint_left_m,
            self.footprint.footprint_right_m,
            self.footprint.gait_sway_margin_m,
            self.footprint.cloud_uncertainty_margin_m,
            self.footprint.odom_uncertainty_margin_m,
            self.footprint.model_uncertainty_margin_m,
            self.footprint.stop_tail_margin_m,
            self.footprint.target_physical_clearance_m,
        )
        if not all(math.isfinite(value) for value in footprint_values):
            raise ValueError('footprint parameters must be finite')
        if any(value < 0.0 for value in footprint_values):
            raise ValueError('footprint parameters must be nonnegative')
        if any(value <= 0.0 for value in footprint_values[:4]):
            raise ValueError('base footprint extents must be positive')

        positive_config = (
            self.config.cloud_stale_timeout_sec,
            self.config.odom_stale_timeout_sec,
            self.config.sector_stale_timeout_sec,
            self.config.safety_status_stale_timeout_sec,
            self.config.trajectory_sample_dt_sec,
            self.config.trajectory_max_translation_step_m,
            self.config.trajectory_max_yaw_step_deg,
            self.config.clearance_search_m,
            self.config.robust_clearance_m,
            self.config.legacy_turn_sweep_clearance_m,
            self.config.wall_confidence_min,
            self.config.trajectory_vy_uncertainty_mps,
            self.config.trajectory_duration_uncertainty_sec,
            self.config.approach_stop_distance_m,
            self.config.left_open_distance_m,
            self.config.turn_target_deg,
            self.config.stage_timeout_sec,
            self.config.max_reverse_distance_m,
        )
        if not all(
            math.isfinite(value) and value > 0.0
            for value in positive_config
        ):
            raise ValueError('planner positive parameters are invalid')
        if not 0.0 <= self.config.hard_clearance_m:
            raise ValueError('hard_clearance_m must be nonnegative')
        if self.config.robust_clearance_m < self.config.hard_clearance_m:
            raise ValueError(
                'robust_clearance_m must not be below hard_clearance_m'
            )
        if self.config.stable_confirm_frames <= 0:
            raise ValueError('stable_confirm_frames must be positive')
        if self.config.max_reverse_segments <= 0:
            raise ValueError('max_reverse_segments must be positive')
        if self.config.minimum_wall_segments_for_turn <= 0:
            raise ValueError(
                'minimum_wall_segments_for_turn must be positive'
            )
        for ratio_name, ratio in (
            (
                'trajectory_vx_uncertainty_ratio',
                self.config.trajectory_vx_uncertainty_ratio,
            ),
            (
                'trajectory_wz_uncertainty_ratio',
                self.config.trajectory_wz_uncertainty_ratio,
            ),
        ):
            if not math.isfinite(ratio) or not 0.0 <= ratio < 1.0:
                raise ValueError(f'{ratio_name} must be in [0, 1)')
        if not 0.0 < self.config.wall_confidence_min <= 1.0:
            raise ValueError('wall_confidence_min must be in (0, 1]')

        for primitive in self.primitives.values():
            TrajectoryGenerator._validate_primitive(primitive)
        if self.primitives[PRIMITIVE_FORWARD].vx_mps <= 0.0:
            raise ValueError('FORWARD_SHORT vx must be positive')
        if self.primitives[PRIMITIVE_REVERSE].vx_mps >= 0.0:
            raise ValueError('REVERSE_SHORT vx must be negative')
        for name in (
            PRIMITIVE_LEFT_ARC,
            PRIMITIVE_LEFT_ARC_OUTSIDE,
            PRIMITIVE_FINE_LEFT_ARC,
        ):
            if self.primitives[name].wz_radps <= 0.0:
                raise ValueError(f'{name} wz must be positive for a left turn')
        for name in (
            PRIMITIVE_OUTSIDE_DIAGONAL,
            PRIMITIVE_LEFT_ARC_OUTSIDE,
        ):
            if self.primitives[name].vy_mps >= 0.0:
                raise ValueError(
                    f'{name} vy must be negative toward the outer side'
                )

    @staticmethod
    def _candidate_names_for_phase(phase):
        if phase == STATE_APPROACH_TURN:
            return (PRIMITIVE_FORWARD, PRIMITIVE_OUTSIDE_DIAGONAL)
        if phase == STATE_REVERSE_RECOVERY:
            return (PRIMITIVE_REVERSE,)
        if phase == STATE_TURN_FINE_ALIGN:
            return (PRIMITIVE_FINE_LEFT_ARC, PRIMITIVE_REVERSE)
        if phase == STATE_CORRIDOR_REACQUIRE:
            return (PRIMITIVE_FORWARD, PRIMITIVE_FINE_LEFT_ARC)
        return (
            PRIMITIVE_LEFT_ARC,
            PRIMITIVE_LEFT_ARC_OUTSIDE,
            PRIMITIVE_OUTSIDE_DIAGONAL,
            PRIMITIVE_REVERSE,
        )

    @staticmethod
    def _rank_candidates(evaluations):
        """按安全等级、余量和终点误差做稳定的字典序排序。"""
        def finite_or(value, fallback):
            if value is None:
                return fallback
            converted = float(value)
            return converted if math.isfinite(converted) else fallback

        def ranking_key(item):
            sweep = item['sweep']
            selectable = item['verdict'] not in (
                VERDICT_UNSAFE,
                VERDICT_UNKNOWN,
            )
            eligibility = (
                2
                if selectable
                else 1 if item['verdict'] == VERDICT_UNKNOWN else 0
            )
            return (
                -eligibility,
                -finite_or(sweep.get('minimum_clearance_m'), -1.0),
                -finite_or(item.get('robustness_ratio'), 0.0),
                -finite_or(
                    sweep.get('minimum_stop_tail_clearance_m'),
                    -1.0,
                ),
                -finite_or(
                    sweep.get('minimum_wall_endpoint_clearance_m'),
                    -1.0,
                ),
                finite_or(item.get('final_yaw_error_deg'), math.inf),
                finite_or(item.get('final_lateral_error_m'), math.inf),
                finite_or(item.get('duration_sec'), math.inf),
                -VERDICT_PRIORITY[item['verdict']],
                PRIMITIVE_ORDER.index(item['name']),
            )

        ranked = sorted(evaluations, key=ranking_key)
        for rank, item in enumerate(ranked, start=1):
            item['rank'] = rank
        return ranked

    @staticmethod
    def _select(evaluations, allowed_names):
        """只从当前阶段允许且已通过几何门控的候选中选择。"""
        selectable = {
            VERDICT_GEOMETRY_SAFE_UNCALIBRATED,
            VERDICT_NOMINAL_SAFE,
            VERDICT_ROBUST_SAFE,
            VERDICT_EXECUTABLE_SAFE,
        }
        for evaluation in evaluations:
            if (
                evaluation['name'] in allowed_names
                and evaluation['verdict'] in selectable
            ):
                return evaluation
        return None


class FirstTurnDryRunStateMachine:
    """第一弯闭环状态机；节点只调用分析路径，不确认任何真实执行。"""

    def __init__(self, config, planner):
        self.config = config
        self.planner = planner
        self.state = STATE_APPROACH_TURN
        self.reason = 'startup'
        self.route_index = 0
        self._state_start_time = None
        self._last_update_time = 0.0
        self._stable_frames = 0
        self._last_cloud_sequence = None
        self._selected = None
        self._execution_start = None

    def update(self, grid, safety, observation, now, cloud_sequence):
        """根据新地图和 Odom 更新 Dry Run 状态，不产生硬件命令。"""
        self._last_update_time = float(now)
        if not safety.cloud_received or not safety.odom_received:
            self.reason = 'waiting_initial_sensors'
            return self._output(None, grid, safety, None)
        if self._state_start_time is None:
            # 初始传感器等待不计入阶段超时，避免设备启动顺序造成假故障。
            self._state_start_time = float(now)
        global_fault = self._global_fault(grid, safety)
        if global_fault is not None:
            return self._fault(global_fault, grid, safety, now)
        if (
            self.state not in (STATE_TURN_COMPLETE, STATE_FAULT_STOP)
            and now - self._state_start_time > self.config.stage_timeout_sec
        ):
            return self._fault('stage_timeout', grid, safety, now)

        new_cloud = cloud_sequence != self._last_cloud_sequence
        if new_cloud:
            self._last_cloud_sequence = cloud_sequence

        front = observation.get('front_distance_m')
        left_open = observation.get('left_open_distance_m')
        turn_progress = float(observation.get('turn_progress_rad', 0.0))
        turn_error = math.radians(self.config.turn_target_deg) - turn_progress

        if self.state == STATE_APPROACH_TURN:
            if (
                front is not None
                and front <= self.config.approach_stop_distance_m
            ) or (
                left_open is not None
                and left_open >= self.config.left_open_distance_m
            ):
                self._transition(STATE_STOP_AND_SCAN, 'turn_zone_reached', now)
            else:
                plan = self.planner.plan(grid, safety, STATE_APPROACH_TURN)
                return self._selection_output(
                    plan,
                    grid,
                    safety,
                    turn_error,
                )

        if self.state in (STATE_STOP_AND_SCAN, STATE_STOP_AND_RESCAN):
            if safety.stop_stable and new_cloud:
                self._stable_frames += 1
            elif not safety.stop_stable:
                self._stable_frames = 0
            if self._stable_frames < self.config.stable_confirm_frames:
                self.reason = 'waiting_for_stable_rescan'
                return self._output(None, grid, safety, turn_error)
            self._transition(STATE_SELECT_TRAJECTORY, 'stable_scan_ready', now)

        if self.state == STATE_SELECT_TRAJECTORY:
            if abs(math.degrees(turn_error)) <= self.config.turn_complete_tolerance_deg:
                self._transition(
                    STATE_CORRIDOR_REACQUIRE,
                    'turn_angle_reached',
                    now,
                )
            elif abs(math.degrees(turn_error)) <= self.config.fine_align_enter_error_deg:
                self._transition(
                    STATE_TURN_FINE_ALIGN,
                    'fine_alignment_required',
                    now,
                )
            else:
                plan = self.planner.plan(
                    grid,
                    safety,
                    STATE_SELECT_TRAJECTORY,
                )
                return self._selection_output(plan, grid, safety, turn_error)

        if self.state == STATE_TURN_FINE_ALIGN:
            if abs(math.degrees(turn_error)) <= self.config.turn_complete_tolerance_deg:
                self._transition(
                    STATE_CORRIDOR_REACQUIRE,
                    'fine_alignment_complete',
                    now,
                )
            else:
                plan = self.planner.plan(
                    grid,
                    safety,
                    STATE_TURN_FINE_ALIGN,
                )
                return self._selection_output(plan, grid, safety, turn_error)

        if self.state == STATE_CORRIDOR_REACQUIRE:
            left = observation.get('left_distance_m')
            right = observation.get('right_distance_m')
            corridor_centered = (
                left is not None
                and right is not None
                and abs(left - right)
                <= self.config.reacquire_side_difference_m
            )
            yaw_aligned = (
                abs(math.degrees(turn_error))
                <= self.config.turn_complete_tolerance_deg
            )
            if corridor_centered and yaw_aligned:
                self.route_index = 1
                self._transition(STATE_TURN_COMPLETE, 'first_turn_complete', now)
            else:
                plan = self.planner.plan(
                    grid,
                    safety,
                    STATE_CORRIDOR_REACQUIRE,
                )
                return self._selection_output(plan, grid, safety, turn_error)

        return self._output(None, grid, safety, turn_error)

    def acknowledge_segment_start(self, candidate, now, start_pose):
        """为未来执行层保留显式握手；当前 ROS Dry Run 节点不会调用。"""
        if candidate is None or candidate.get('verdict') != VERDICT_ROBUST_SAFE:
            raise ValueError('only ROBUST_SAFE candidate may start')
        if candidate['name'] == PRIMITIVE_REVERSE:
            next_state = STATE_REVERSE_RECOVERY
        else:
            next_state = STATE_EXECUTE_SHORT_SEGMENT
        self._selected = dict(candidate)
        self._execution_start = {
            'time_sec': float(now),
            'pose': start_pose,
        }
        self._transition(next_state, 'segment_started', now)

    def check_tracking(self, current_pose, now):
        """比较实际 Odom 与候选预测，偏差超限立即锁止。"""
        if self.state not in (
            STATE_EXECUTE_SHORT_SEGMENT,
            STATE_REVERSE_RECOVERY,
        ):
            return None
        if self._selected is None or self._execution_start is None:
            return self._fault('execution_context_missing', None, None, now)
        start = self._execution_start['pose']
        elapsed = max(0.0, now - self._execution_start['time_sec'])
        primitive = self.planner.primitives[self._selected['name']]
        expected_relative = TrajectoryGenerator._pose_at(
            primitive,
            min(elapsed, primitive.duration_sec),
        )
        expected = compose_pose(start, expected_relative)
        position_error = math.hypot(
            current_pose.x_m - expected.x_m,
            current_pose.y_m - expected.y_m,
        )
        yaw_error = abs(normalize_angle(
            current_pose.yaw_rad - expected.yaw_rad
        ))
        if position_error > self.config.max_tracking_position_error_m:
            return self._fault('trajectory_position_deviation', None, None, now)
        if math.degrees(yaw_error) > self.config.max_tracking_yaw_error_deg:
            return self._fault('trajectory_yaw_deviation', None, None, now)
        return {
            'position_error_m': position_error,
            'yaw_error_deg': math.degrees(yaw_error),
        }

    def acknowledge_segment_stopped(self, now):
        """短段后必须 StopMove、稳定和重扫；不得连续执行下一段。"""
        if self.state == STATE_REVERSE_RECOVERY:
            self.planner.record_reverse_execution(PRIMITIVE_REVERSE)
        if self.state not in (
            STATE_EXECUTE_SHORT_SEGMENT,
            STATE_REVERSE_RECOVERY,
        ):
            raise ValueError('no segment is active')
        self._selected = None
        self._execution_start = None
        self._transition(STATE_STOP_AND_RESCAN, 'segment_stopped', now)

    def _selection_output(self, plan, grid, safety, turn_error):
        selected = plan.get('selected') if plan else None
        if selected is None:
            return self._fault(
                'no_safe_candidate',
                grid,
                safety,
                self._last_update_time,
                plan=plan,
                turn_error=turn_error,
            )
        if selected['name'] == PRIMITIVE_REVERSE:
            # 只有执行层明确确认后退短段已开始，才进入 REVERSE_RECOVERY；
            # Dry Run 选择结果本身不能伪造成执行状态。
            self.reason = 'reverse_recovery_candidate'
        elif selected['verdict'] == VERDICT_ROBUST_SAFE:
            self.reason = 'robust_safe_candidate_waiting_for_executor'
        else:
            self.reason = 'dry_run_candidate_execution_forbidden'
        return self._output(plan, grid, safety, turn_error)

    def _global_fault(self, grid, safety):
        if grid is None:
            return 'cloud_missing'
        if not safety.cloud_valid:
            return 'cloud_invalid'
        if not safety.odom_valid:
            return 'odom_invalid'
        if safety.cloud_age_sec > self.config.cloud_stale_timeout_sec:
            return 'cloud_stale'
        if safety.odom_age_sec > self.config.odom_stale_timeout_sec:
            return 'odom_stale'
        if safety.yaw_jump:
            return 'yaw_jump'
        if abs(math.degrees(safety.roll_rad)) > self.config.max_roll_deg:
            return 'roll_limit_exceeded'
        if abs(math.degrees(safety.pitch_rad)) > self.config.max_pitch_deg:
            return 'pitch_limit_exceeded'
        if (
            safety.watchdog_ok is False
            and self.planner._safety_status_fresh(safety.watchdog_age_sec)
        ):
            return 'watchdog_fault'
        if (
            safety.estop_triggered is True
            and self.planner._safety_status_fresh(safety.estop_age_sec)
        ):
            return 'estop_triggered'
        if self.state in (
            STATE_EXECUTE_SHORT_SEGMENT,
            STATE_REVERSE_RECOVERY,
        ):
            if self._selected is None:
                return 'execution_context_missing'
            if safety.watchdog_ok is not True:
                return 'execution_watchdog_not_ok'
            if not self.planner._safety_status_fresh(
                safety.watchdog_age_sec
            ):
                return 'execution_watchdog_stale'
            if safety.estop_triggered is not False:
                return 'execution_estop_not_clear'
            if not self.planner._safety_status_fresh(safety.estop_age_sec):
                return 'execution_estop_stale'
            for name in self.planner.REQUIRED_SECTORS[self._selected['name']]:
                if safety.sector_status.get(name, {}).get('usable') is not True:
                    return f'execution_sector_unavailable:{name}'
        footprint_safety = self.planner.current_footprint_safety(grid)
        if footprint_safety['collision']:
            return 'current_footprint_collision'
        if (
            footprint_safety['minimum_clearance_m']
            < self.config.hard_clearance_m
        ):
            return 'current_hard_clearance_too_small'
        return None

    def _fault(
        self,
        reason,
        grid,
        safety,
        now,
        plan=None,
        turn_error=None,
    ):
        self._transition(STATE_FAULT_STOP, reason, now)
        return self._output(plan, grid, safety, turn_error)

    def _transition(self, state, reason, now):
        if state != self.state:
            self.state = state
            self._state_start_time = float(now)
            self._stable_frames = 0
        self.reason = str(reason)

    def _output(self, plan, grid, safety, turn_error):
        selected = (
            plan.get('selected')
            if plan
            else self._selected
        )
        robust = bool(plan and plan.get('has_robust_safe'))
        execution_allowed = (
            False
            # 本轮节点永远是 Dry Run。未来执行器必须另行实现控制权握手，
            # 不能通过修改一个参数把此诊断节点变成运动节点。
        )
        return {
            'dry_run': True,
            'scope': 'FIRST_TURN_ONLY',
            'state': self.state,
            'reason': self.reason,
            'safety_action': 'STOP_MOVE_REQUIRED',
            'motion_output': False,
            'execution_allowed': execution_allowed,
            'route_index': self.route_index,
            'route_total': 5,
            'turn_direction': 'LEFT',
            'turn_error_rad': turn_error,
            'turn_error_deg': (
                math.degrees(turn_error)
                if turn_error is not None
                else None
            ),
            'selected_candidate': selected,
            'candidates': plan.get('candidates', []) if plan else [],
            'top_candidates': plan.get('top_candidates', []) if plan else [],
            'has_robust_safe_candidate': robust,
            'has_dry_run_safe_candidate': bool(
                plan and plan.get('has_dry_run_safe')
            ),
            'map_statistics': (
                dict(grid.statistics) if grid is not None else {}
            ),
            'wall_segments': (
                list(grid.wall_segments) if grid is not None else []
            ),
            'rear_coverage_status': (
                plan.get('rear_coverage_status')
                if plan
                else self._rear_coverage_status(safety)
            ),
            'rear_unavailable_sectors': (
                plan.get('rear_unavailable_sectors', [])
                if plan
                else self._rear_unavailable_sectors(safety)
            ),
            'current_footprint_safety': (
                self.planner.current_footprint_safety(grid)
                if grid is not None
                else None
            ),
            'sector_status': (
                safety.sector_status if safety is not None else {}
            ),
            'reverse_segments': self.planner.reverse_segments,
            'reverse_distance_m': self.planner.reverse_distance_m,
        }

    @staticmethod
    def _rear_unavailable_sectors(safety):
        """显式列出后方不可用扇区，禁止把前向新鲜度当作后向证据。"""
        if safety is None:
            return ['rear', 'left_rear', 'right_rear']
        return [
            name
            for name in ('rear', 'left_rear', 'right_rear')
            if safety.sector_status.get(name, {}).get('usable') is not True
        ]

    @classmethod
    def _rear_coverage_status(cls, safety):
        return (
            'SUFFICIENT'
            if not cls._rear_unavailable_sectors(safety)
            else 'rear_coverage_insufficient'
        )


def classify_sector(angle_deg):
    """把水平角分成前、侧、后八区，正角为机器人左侧。"""
    angle = normalize_degrees(angle_deg)
    absolute = abs(angle)
    if absolute <= 10.0:
        return 'front'
    if 10.0 < angle <= 70.0:
        return 'left_front'
    if -70.0 <= angle < -10.0:
        return 'right_front'
    if 70.0 < angle <= 110.0:
        return 'left'
    if -110.0 <= angle < -70.0:
        return 'right'
    if 110.0 < angle < 170.0:
        return 'left_rear'
    if -170.0 < angle < -110.0:
        return 'right_rear'
    return 'rear'


def normalize_degrees(angle_deg):
    """将角度归一化到 [-180, 180)。"""
    return (float(angle_deg) + 180.0) % 360.0 - 180.0


def normalize_angle(angle_rad):
    """将弧度归一化到 [-pi, pi]。"""
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def pose_to_dict(pose):
    return {
        'x_m': pose.x_m,
        'y_m': pose.y_m,
        'yaw_rad': pose.yaw_rad,
        'yaw_deg': math.degrees(pose.yaw_rad),
        'time_sec': pose.time_sec,
    }


def compose_pose(origin, relative):
    """将机器人起点坐标与机体系相对轨迹组合为 Odom 位姿。"""
    cos_yaw = math.cos(origin.yaw_rad)
    sin_yaw = math.sin(origin.yaw_rad)
    return Pose2D(
        origin.x_m
        + cos_yaw * relative.x_m
        - sin_yaw * relative.y_m,
        origin.y_m
        + sin_yaw * relative.x_m
        + cos_yaw * relative.y_m,
        normalize_angle(origin.yaw_rad + relative.yaw_rad),
        origin.time_sec + relative.time_sec,
    )


def default_motion_primitives():
    """返回仅供几何 Dry Run 的占位模型，全部明确标记为未标定。"""
    return (
        MotionPrimitive(PRIMITIVE_FORWARD, 0.25, 0.0, 0.0, 0.50),
        MotionPrimitive(PRIMITIVE_OUTSIDE_DIAGONAL, 0.18, -0.05, 0.0, 0.50),
        MotionPrimitive(PRIMITIVE_LEFT_ARC, 0.18, 0.0, 0.35, 0.50),
        MotionPrimitive(
            PRIMITIVE_LEFT_ARC_OUTSIDE,
            0.18,
            -0.04,
            0.35,
            0.50,
        ),
        MotionPrimitive(PRIMITIVE_REVERSE, -0.18, 0.0, 0.0, 0.40),
        MotionPrimitive(PRIMITIVE_FINE_LEFT_ARC, 0.12, 0.0, 0.20, 0.35),
    )
