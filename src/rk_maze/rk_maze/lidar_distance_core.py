#!/usr/bin/env python3
"""F1+F2+F3: LiDAR point cloud filtering, effective obstacle clustering,
hard_distance and navigation_distance dual channels.

Design constraints:
- hard_distance: single-frame, conservative, cluster-based (never min(all_points))
- navigation_distance: multi-frame, temporally filtered, combined with wall lines
- All distances are clearance from robot body edge, NOT Euclidean range
"""

from __future__ import annotations
import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

UNKNOWN = 'UNKNOWN'
FREE = 'FREE'
OCCUPIED = 'OCCUPIED'

# Sector labels
SECTOR_FRONT = 'front'
SECTOR_LEFT_FRONT = 'left_front'
SECTOR_RIGHT_FRONT = 'right_front'
SECTOR_LEFT = 'left'
SECTOR_RIGHT = 'right'
SECTOR_REAR = 'rear'
SECTOR_LEFT_REAR = 'left_rear'
SECTOR_RIGHT_REAR = 'right_rear'

ALL_SECTORS = [
    SECTOR_FRONT, SECTOR_LEFT_FRONT, SECTOR_RIGHT_FRONT,
    SECTOR_LEFT, SECTOR_RIGHT,
    SECTOR_REAR, SECTOR_LEFT_REAR, SECTOR_RIGHT_REAR,
]

# Speed classification
SPEED_CLEAR = 'CLEAR'
SPEED_CAUTION = 'CAUTION'
SPEED_BRAKE = 'BRAKE'
SPEED_EMERGENCY = 'EMERGENCY'
SPEED_UNKNOWN = 'UNKNOWN'


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class LidarDistanceConfig:
    """All tunable parameters for LiDAR distance processing."""

    # ---- point cloud filtering ----
    min_range_m: float = 0.08          # minimum valid range
    max_range_m: float = 3.00          # maximum valid range
    ground_z_min_m: float = -0.30      # ground plane lower bound (base_link)
    ground_z_max_m: float = 0.03       # ground plane upper bound
    obstacle_z_min_m: float = 0.03     # minimum obstacle height
    obstacle_z_max_m: float = 0.80     # maximum obstacle height (above this ignored)

    # ---- body / self-reflection filter ----
    body_x_min_m: float = -0.40
    body_x_max_m: float = 0.40
    body_y_min_m: float = -0.18
    body_y_max_m: float = 0.18
    leg_self_filter_enabled: bool = False
    leg_radius_m: float = 0.08

    # ---- voxel downsampling ----
    voxel_size_m: float = 0.02

    # ---- effective obstacle clustering ----
    cluster_tolerance_m: float = 0.05   # max gap between points in same cluster
    min_cluster_points: int = 5         # discard clusters with fewer points
    cluster_percentile: float = 0.10    # use P10 distance within cluster (conservative)

    # ---- sector / ROI definition ----
    front_half_angle_deg: float = 10.0    # front sector ±10°
    diagonal_min_angle_deg: float = 10.0  # left_front / right_front start
    diagonal_max_angle_deg: float = 45.0  # left_front / right_front end
    side_min_angle_deg: float = 45.0      # left / right start
    side_max_angle_deg: float = 90.0      # left / right end

    # ---- footprint dimensions (for clearance calculation) ----
    # T0卷尺标定: LiDAR传感器位于base_link前方0.28m处
    # Go2机身半长约0.35m, LiDAR在机身最前端
    footprint_front_m: float = 0.28   # T0校准: LiDAR到base_link距离
    footprint_rear_m: float = 0.42    # base_link到尾部 (0.70-0.28)
    footprint_left_m: float = 0.18
    footprint_right_m: float = 0.18
    perception_margin_m: float = 0.03

    # ---- T0 calibration record ----
    lidar_offset_x_m: float = 0.281  # T0实测: base_link原点在LiDAR后方0.281m(记录用)

    # ---- hard_distance specific ----
    hard_max_age_sec: float = 0.15      # STALE if older than this

    # ---- navigation_distance specific ----
    nav_temporal_window: int = 5        # median over last N valid frames
    nav_ema_alpha: float = 0.3          # EMA smoothing factor
    nav_max_age_sec: float = 0.50       # STALE if older than this
    nav_max_jump_m: float = 0.30        # reject sudden distance jumps

    # ---- dynamic stop distance ----
    total_latency_sec: float = 0.20     # command + network + SDK latency
    min_deceleration: float = 0.80      # minimum braking deceleration (m/s²)
    measured_stop_tail_95_m: float = 0.15  # 95th percentile measured stop tail


# ---------------------------------------------------------------------------
# Data types
# ---------------------------------------------------------------------------

@dataclass
class Point3D:
    x: float
    y: float
    z: float


@dataclass
class EffectiveCluster:
    """One valid obstacle cluster with diagnostic fields."""
    points: List[Point3D] = field(default_factory=list)
    centroid_x: float = 0.0
    centroid_y: float = 0.0
    centroid_z: float = 0.0
    distance_2d: float = float('inf')     # min Euclidean distance
    conservative_distance: float = float('inf')  # low-percentile distance
    point_count: int = 0
    spread_m: float = 0.0                # max point-to-point distance
    valid: bool = False
    sector: str = ''


@dataclass
class SectorDistance:
    """Single-sector distance diagnostic output."""
    sector: str = ''
    hard_distance: float = float('inf')
    navigation_distance: float = float('inf')
    front_clearance: float = float('inf')   # along x, from body front
    rear_clearance: float = float('inf')
    left_clearance: float = float('inf')    # along y, from body left
    right_clearance: float = float('inf')
    range_m: float = float('inf')           # Euclidean range
    forward_clearance: float = float('inf') # diagonal forward component
    lateral_clearance: float = float('inf') # diagonal lateral component
    point_count: int = 0
    cluster_point_count: int = 0
    spread: float = 0.0
    confidence: float = 0.0
    valid: bool = False
    stale: bool = True
    reason: str = 'uninitialized'
    data_age: float = float('inf')


@dataclass
class DistanceSnapshot:
    """Complete per-frame distance output for all sectors."""
    sectors: dict = field(default_factory=dict)
    hard_front: float = float('inf')
    nav_front: float = float('inf')
    speed_class: str = SPEED_UNKNOWN
    dynamic_stop_distance: float = float('inf')
    remaining_margin: float = float('inf')
    current_speed: float = 0.0
    cloud_timestamp: float = 0.0
    valid: bool = False
    stale: bool = True


# ---------------------------------------------------------------------------
# F1: Point cloud filtering
# ---------------------------------------------------------------------------

def filter_point_cloud(
    points: List[Point3D],
    config: LidarDistanceConfig,
) -> List[Point3D]:
    """Filter raw PointCloud2 points: NaN/Inf, range, ground, self, height."""
    filtered = []

    for p in points:
        # NaN / Inf check
        if not (math.isfinite(p.x) and math.isfinite(p.y) and math.isfinite(p.z)):
            continue

        # Range check (Euclidean)
        dist = math.sqrt(p.x * p.x + p.y * p.y + p.z * p.z)
        if dist < config.min_range_m or dist > config.max_range_m:
            continue

        # Ground filter: points near or below ground plane
        if config.ground_z_min_m <= p.z <= config.ground_z_max_m:
            continue

        # Height filter: only keep points at obstacle-relevant heights
        if p.z < config.obstacle_z_min_m or p.z > config.obstacle_z_max_m:
            continue

        # Self-reflection: body bounding box
        if (config.body_x_min_m <= p.x <= config.body_x_max_m
                and config.body_y_min_m <= p.y <= config.body_y_max_m):
            continue

        filtered.append(p)

    return filtered


def voxel_downsample(
    points: List[Point3D],
    voxel_size_m: float,
) -> List[Point3D]:
    """Simple voxel grid downsampling — one point per occupied voxel."""
    if not points or voxel_size_m <= 0.0:
        return list(points)

    voxels = {}
    for p in points:
        vx = int(math.floor(p.x / voxel_size_m))
        vy = int(math.floor(p.y / voxel_size_m))
        vz = int(math.floor(p.z / voxel_size_m))
        key = (vx, vy, vz)
        if key not in voxels:
            voxels[key] = p
    return list(voxels.values())


# ---------------------------------------------------------------------------
# F2: Effective obstacle clustering and hard_distance
# ---------------------------------------------------------------------------

def _cluster_points(
    points: List[Point3D],
    tolerance_m: float,
) -> List[List[Point3D]]:
    """Simple Euclidean-distance clustering (greedy)."""
    if not points:
        return []
    if tolerance_m <= 0.0:
        return [[p] for p in points]

    remaining = list(points)
    clusters = []

    while remaining:
        seed = remaining.pop(0)
        cluster = [seed]
        changed = True
        while changed:
            changed = False
            i = 0
            while i < len(remaining):
                p = remaining[i]
                # Check if near any point already in cluster
                for cp in cluster:
                    dx = p.x - cp.x
                    dy = p.y - cp.y
                    dz = p.z - cp.z
                    if math.sqrt(dx * dx + dy * dy + dz * dz) <= tolerance_m:
                        cluster.append(p)
                        remaining.pop(i)
                        changed = True
                        break
                else:
                    i += 1
        clusters.append(cluster)

    return clusters


def _cluster_statistics(
    cluster: List[Point3D],
    percentile: float,
) -> dict:
    """Compute centroid, spread, and conservative distance for a cluster."""
    if not cluster:
        return {
            'centroid_x': 0.0, 'centroid_y': 0.0, 'centroid_z': 0.0,
            'distance_2d': float('inf'), 'conservative_distance': float('inf'),
            'point_count': 0, 'spread_m': 0.0,
        }

    n = len(cluster)
    cx = sum(p.x for p in cluster) / n
    cy = sum(p.y for p in cluster) / n
    cz = sum(p.z for p in cluster) / n

    distances = sorted(math.sqrt(p.x * p.x + p.y * p.y) for p in cluster)
    dist_2d = math.sqrt(cx * cx + cy * cy)

    # Conservative: low percentile (closer = more dangerous)
    idx = max(0, min(len(distances) - 1, int(len(distances) * percentile)))
    conservative = distances[idx]

    # Spread: max distance between any two points
    spread = 0.0
    for i in range(min(n, 50)):  # cap for performance
        for j in range(i + 1, min(n, 50)):
            dx = cluster[i].x - cluster[j].x
            dy = cluster[i].y - cluster[j].y
            dz = cluster[i].z - cluster[j].z
            d = math.sqrt(dx * dx + dy * dy + dz * dz)
            if d > spread:
                spread = d

    return {
        'centroid_x': cx, 'centroid_y': cy, 'centroid_z': cz,
        'distance_2d': dist_2d, 'conservative_distance': conservative,
        'point_count': n, 'spread_m': spread,
    }


def find_effective_clusters(
    points: List[Point3D],
    config: LidarDistanceConfig,
) -> List[EffectiveCluster]:
    """Find valid obstacle clusters, sorted by conservative distance (nearest first)."""
    raw_clusters = _cluster_points(points, config.cluster_tolerance_m)
    result = []

    for raw in raw_clusters:
        if len(raw) < config.min_cluster_points:
            continue

        stats = _cluster_statistics(raw, config.cluster_percentile)

        cluster = EffectiveCluster(
            points=raw,
            centroid_x=stats['centroid_x'],
            centroid_y=stats['centroid_y'],
            centroid_z=stats['centroid_z'],
            distance_2d=stats['distance_2d'],
            conservative_distance=stats['conservative_distance'],
            point_count=stats['point_count'],
            spread_m=stats['spread_m'],
            valid=True,
        )
        result.append(cluster)

    result.sort(key=lambda c: c.conservative_distance)
    return result


def _classify_sector(x: float, y: float, config: LidarDistanceConfig) -> str:
    """Assign a point to a sector based on angle in base_link frame."""
    angle_deg = math.degrees(math.atan2(y, x))

    ad = abs(angle_deg)
    front_half = config.front_half_angle_deg
    diag_min = config.diagonal_min_angle_deg
    diag_max = config.diagonal_max_angle_deg
    side_min = config.side_min_angle_deg
    side_max = config.side_max_angle_deg

    if ad <= front_half:
        return SECTOR_FRONT
    if diag_min < angle_deg <= diag_max:
        return SECTOR_LEFT_FRONT
    if -diag_max <= angle_deg < -diag_min:
        return SECTOR_RIGHT_FRONT
    if side_min < angle_deg <= side_max:
        return SECTOR_LEFT
    if -side_max <= angle_deg < -side_min:
        return SECTOR_RIGHT
    if angle_deg > 90.0 or angle_deg < -90.0:
        return SECTOR_REAR  # simplified
    return SECTOR_FRONT  # fallback


def _compute_clearance(
    cluster: EffectiveCluster,
    config: LidarDistanceConfig,
) -> Tuple[float, float, float, float]:
    """Compute front/rear/left/right clearance from body edge.

    LiDAR is positioned at body front (x = +footprint_front_m from base_link).
    T0 calibration confirmed LiDAR ~0.28m from base_link, consistent with
    Go2 body half-length. Clearance = centroid - body_edge - margin.
    """
    cx = cluster.centroid_x
    cy = cluster.centroid_y
    margin = config.perception_margin_m

    front_clearance = cx - config.footprint_front_m - margin
    rear_clearance = -cx - config.footprint_rear_m - margin
    left_clearance = cy - config.footprint_left_m - margin
    right_clearance = -cy - config.footprint_right_m - margin

    return front_clearance, rear_clearance, left_clearance, right_clearance


def compute_hard_distance(
    sector_points: List[Point3D],
    sector: str,
    config: LidarDistanceConfig,
    timestamp: float,
    now: float,
) -> SectorDistance:
    """Compute hard_distance for one sector using effective obstacle clustering."""
    data_age = now - timestamp if timestamp > 0 else float('inf')

    sd = SectorDistance(
        sector=sector,
        data_age=data_age,
        stale=(data_age > config.hard_max_age_sec),
    )

    if not sector_points:
        sd.reason = 'no_points_in_sector'
        sd.valid = False
        return sd

    clusters = find_effective_clusters(sector_points, config)

    if not clusters:
        sd.reason = 'no_effective_cluster'
        sd.point_count = len(sector_points)
        sd.valid = False
        return sd

    # Use nearest effective cluster
    nearest = clusters[0]

    # Distances
    sd.hard_distance = nearest.conservative_distance
    sd.range_m = nearest.distance_2d
    sd.point_count = len(sector_points)
    sd.cluster_point_count = nearest.point_count
    sd.spread = nearest.spread_m
    sd.valid = True

    # Clearances from body edge
    fc, rc, lc, right_c = _compute_clearance(nearest, config)
    sd.front_clearance = fc
    sd.rear_clearance = rc
    sd.left_clearance = lc
    sd.right_clearance = right_c

    # For diagonal sectors, also compute forward/lateral components
    if sector in (SECTOR_LEFT_FRONT, SECTOR_RIGHT_FRONT):
        sd.forward_clearance = nearest.centroid_x - config.footprint_front_m - config.perception_margin_m
        sd.lateral_clearance = abs(nearest.centroid_y) - config.footprint_left_m - config.perception_margin_m

    # Confidence based on cluster quality
    # More points + wider spread = more confident (real object, not noise)
    point_score = min(1.0, nearest.point_count / 20.0)
    spread_score = min(1.0, nearest.spread_m / 0.10)
    sd.confidence = 0.5 * point_score + 0.5 * spread_score

    sd.stale = (data_age > config.hard_max_age_sec)
    sd.reason = 'ok' if sd.valid else 'no_effective_cluster'

    return sd


def compute_all_hard_distances(
    points: List[Point3D],
    config: LidarDistanceConfig,
    timestamp: float,
    now: float,
) -> dict:
    """Compute hard_distance for all 8 sectors from filtered point cloud."""
    # Assign points to sectors
    sector_points = {s: [] for s in ALL_SECTORS}
    for p in points:
        sector = _classify_sector(p.x, p.y, config)
        sector_points[sector].append(p)

    # Also allocate diagonal points to front for comprehensive coverage
    for s in (SECTOR_LEFT_FRONT, SECTOR_RIGHT_FRONT):
        front_points = list(sector_points.get(SECTOR_FRONT, []))
        front_points.extend(sector_points[s])
        sector_points[SECTOR_FRONT] = front_points

    result = {}
    for sector in ALL_SECTORS:
        result[sector] = compute_hard_distance(
            sector_points[sector], sector, config, timestamp, now
        )

    return result


# ---------------------------------------------------------------------------
# F3: navigation_distance (temporal filtering)
# ---------------------------------------------------------------------------

class NavigationDistanceFilter:
    """Multi-frame temporal filter for stable navigation distances."""

    def __init__(self, config: LidarDistanceConfig):
        self._config = config
        self._history: dict = {s: [] for s in ALL_SECTORS}
        self._ema: dict = {s: float('inf') for s in ALL_SECTORS}
        self._last_update: dict = {s: 0.0 for s in ALL_SECTORS}

    def update(
        self,
        hard_distances: dict,
        timestamp: float,
    ) -> dict:
        """Update navigation distances with new hard_distance readings."""
        window = self._config.nav_temporal_window
        alpha = self._config.nav_ema_alpha
        max_jump = self._config.nav_max_jump_m
        max_age = self._config.nav_max_age_sec

        result = {}
        for sector in ALL_SECTORS:
            hd = hard_distances.get(sector)
            if hd is None or not hd.valid:
                # Keep old value but mark aging
                continue

            distance = hd.hard_distance

            # Reject sudden jumps
            if (math.isfinite(self._ema[sector])
                    and abs(distance - self._ema[sector]) > max_jump):
                # Suspicious jump — use EMA value but don't update history
                nav_dist = self._ema[sector]
                stale = (timestamp - self._last_update[sector]) > max_age
            else:
                # Add to history window
                self._history[sector].append(distance)
                if len(self._history[sector]) > window:
                    self._history[sector] = self._history[sector][-window:]

                # Median of recent valid values
                if self._history[sector]:
                    sorted_vals = sorted(self._history[sector])
                    median = sorted_vals[len(sorted_vals) // 2]
                else:
                    median = distance

                # EMA smoothing
                if math.isfinite(self._ema[sector]):
                    self._ema[sector] = alpha * median + (1.0 - alpha) * self._ema[sector]
                else:
                    self._ema[sector] = median

                nav_dist = self._ema[sector]
                self._last_update[sector] = timestamp
                stale = False

            sd = SectorDistance(
                sector=sector,
                hard_distance=hd.hard_distance if hd else float('inf'),
                navigation_distance=nav_dist,
                front_clearance=hd.front_clearance if hd else float('inf'),
                rear_clearance=hd.rear_clearance if hd else float('inf'),
                left_clearance=hd.left_clearance if hd else float('inf'),
                right_clearance=hd.right_clearance if hd else float('inf'),
                point_count=hd.point_count if hd else 0,
                valid=math.isfinite(nav_dist) and not stale,
                stale=stale,
                reason='ok' if not stale else 'navigation_stale',
                data_age=timestamp - self._last_update[sector] if self._last_update[sector] > 0 else float('inf'),
            )
            result[sector] = sd

        return result

    def reset(self):
        """Clear all history (e.g. on odom jump or sensor reset)."""
        self._history = {s: [] for s in ALL_SECTORS}
        self._ema = {s: float('inf') for s in ALL_SECTORS}
        self._last_update = {s: 0.0 for s in ALL_SECTORS}


# ---------------------------------------------------------------------------
# Dynamic stop distance
# ---------------------------------------------------------------------------

def compute_dynamic_stop_distance(
    current_speed: float,
    config: LidarDistanceConfig,
) -> float:
    """Compute speed-dependent stopping distance."""
    if current_speed <= 0.0:
        return config.footprint_front_m + config.measured_stop_tail_95_m + config.perception_margin_m

    latency_dist = current_speed * config.total_latency_sec
    braking_dist = (current_speed * current_speed) / (2.0 * config.min_deceleration)

    return (
        config.footprint_front_m
        + latency_dist
        + braking_dist
        + config.measured_stop_tail_95_m
        + config.perception_margin_m
        + 0.05  # uncertainty margin
    )


def classify_speed(
    hard_front: float,
    nav_front: float,
    dynamic_stop: float,
    config: LidarDistanceConfig,
) -> str:
    """Classify speed level based on available clearances."""
    if not math.isfinite(hard_front) or not math.isfinite(nav_front):
        return SPEED_UNKNOWN

    # Use the more conservative (smaller) distance
    effective = min(hard_front, nav_front)

    if effective <= config.footprint_front_m + config.perception_margin_m:
        return SPEED_EMERGENCY
    if effective <= dynamic_stop:
        return SPEED_BRAKE
    if effective <= dynamic_stop * 1.5:
        return SPEED_CAUTION
    return SPEED_CLEAR


def compute_distance_snapshot(
    filtered_points: List[Point3D],
    config: LidarDistanceConfig,
    nav_filter: NavigationDistanceFilter,
    timestamp: float,
    now: float,
    speed_estimator: Optional[LiDARSpeedEstimator] = None,
    odom_speed: float = 0.0,
) -> DistanceSnapshot:
    """Full per-frame processing: filter → hard → nav → speed classification.

    Speed priority: LiDAR range-rate > odom twist (odom twist is unreliable on Go2).
    """
    # Hard distances
    hard = compute_all_hard_distances(filtered_points, config, timestamp, now)

    # Navigation distances
    nav = nav_filter.update(hard, timestamp)

    # Front distances for safety
    hard_front = hard[SECTOR_FRONT].hard_distance if hard[SECTOR_FRONT].valid else float('inf')
    nav_front = nav.get(SECTOR_FRONT, SectorDistance()).navigation_distance

    # Speed estimation: prefer LiDAR range-rate over odom (odom twist = 0 on Go2)
    lidar_speed = speed_estimator.speed(now) if speed_estimator else 0.0
    # Use LiDAR speed if valid, otherwise fall back to odom (usually 0)
    current_speed = lidar_speed if abs(lidar_speed) > 0.001 else max(0.0, odom_speed)
    # Update speed estimator with latest front distance
    if speed_estimator and hard[SECTOR_FRONT].valid:
        speed_estimator.update(timestamp, hard[SECTOR_FRONT].hard_distance)

    # Dynamic stop distance
    dyn_stop = compute_dynamic_stop_distance(current_speed, config)

    # Speed classification
    speed = classify_speed(hard_front, nav_front, dyn_stop, config)

    # Remaining margin
    effective_front = min(hard_front, nav_front)
    remaining = effective_front - dyn_stop if math.isfinite(effective_front) else float('-inf')

    valid = hard[SECTOR_FRONT].valid
    stale = (now - timestamp) > config.hard_max_age_sec if valid else True
    return DistanceSnapshot(
        sectors={s: nav.get(s, SectorDistance(sector=s)) for s in ALL_SECTORS},
        hard_front=hard_front,
        nav_front=nav_front,
        speed_class=speed,
        dynamic_stop_distance=dyn_stop,
        remaining_margin=remaining,
        current_speed=current_speed,
        cloud_timestamp=timestamp,
        valid=valid,
        stale=stale,
    )


# ---------------------------------------------------------------------------
# LiDAR range-rate speed estimator (replaces unreliable odom twist)
# ---------------------------------------------------------------------------

# Odom twist.linear.x on Go2 is always 0.0 (T1 verified).
# We estimate forward speed from LiDAR front distance change rate.


class LiDARSpeedEstimator:
    """Estimate forward speed from rate of change of front LiDAR distance.

    Uses a short history of front_x median values, fits a linear slope
    to estimate approach speed. Positive = approaching obstacle.

    Noise handling:
    - Requires minimum samples before reporting non-zero speed
    - Uses median of recent samples to reduce single-frame noise
    - Caps maximum plausible speed
    - Returns 0.0 when data is stale or insufficient
    """

    def __init__(
        self,
        history_duration_sec: float = 0.50,
        min_samples: int = 5,
        max_speed_m_s: float = 0.80,
        stale_timeout_sec: float = 0.30,
    ):
        self._history_duration = history_duration_sec
        self._min_samples = min_samples
        self._max_speed = max_speed_m_s
        self._stale_timeout = stale_timeout_sec
        self._samples: list = []  # list of (timestamp, front_x_median)
        self._last_speed: float = 0.0

    def update(self, timestamp: float, front_x_median: float):
        """Add a new front distance sample."""
        if not math.isfinite(front_x_median):
            return
        self._samples.append((timestamp, front_x_median))
        # Evict old samples
        cutoff = timestamp - self._history_duration
        while self._samples and self._samples[0][0] < cutoff:
            self._samples.pop(0)
        # Limit buffer size
        if len(self._samples) > 50:
            self._samples = self._samples[-50:]

    def speed(self, now: float) -> float:
        """Estimate current forward speed from distance change rate.

        Positive = approaching obstacle (distance decreasing).
        Returns 0.0 if insufficient data.
        """
        # Stale check
        if not self._samples:
            return 0.0
        if now - self._samples[-1][0] > self._stale_timeout:
            return 0.0
        if len(self._samples) < self._min_samples:
            return 0.0

        # Use two medians: first half vs second half
        n = len(self._samples)
        mid = n // 2
        first_half = [s[1] for s in self._samples[:mid]]
        second_half = [s[1] for s in self._samples[mid:]]

        if not first_half or not second_half:
            return 0.0

        first_med = sorted(first_half)[len(first_half) // 2]
        second_med = sorted(second_half)[len(second_half) // 2]

        first_time = self._samples[0][0]
        second_time = self._samples[-1][0]
        dt = second_time - first_time

        if dt < 0.05:
            return 0.0

        # Speed = -Δx/Δt (distance decreasing → positive speed)
        speed = -(second_med - first_med) / dt

        # Only trust the estimate if distance is actually changing
        if abs(second_med - first_med) < 0.01:
            return 0.0

        # Clamp
        speed = max(-self._max_speed, min(self._max_speed, speed))
        self._last_speed = speed
        return speed

    def reset(self):
        self._samples.clear()
        self._last_speed = 0.0

    @property
    def sample_count(self) -> int:
        return len(self._samples)

    @property
    def last_speed(self) -> float:
        return self._last_speed
