#!/usr/bin/env python3
"""F4c: Local 2D occupancy grid (3m x 3m, configurable resolution).

Cell states: UNKNOWN, FREE, OCCUPIED.
No-point areas remain UNKNOWN — never assumed FREE.

Also supports sector coverage statistics for odometry buffer points.
"""

import math
from dataclasses import dataclass, field
from typing import List, Optional, Set, Tuple

from rk_maze.lidar_distance_core import Point3D

# Cell states
CELL_UNKNOWN = 0
CELL_FREE = 1
CELL_OCCUPIED = 2


@dataclass
class LocalGridConfig:
    """Configuration for the local occupancy grid."""

    # Map bounds (base_link frame)
    x_min_m: float = -1.50
    x_max_m: float = 1.50
    y_min_m: float = -1.50
    y_max_m: float = 1.50

    # Resolution
    resolution_m: float = 0.03

    # Point filters
    min_range_m: float = 0.08
    max_range_m: float = 3.00
    z_min_m: float = 0.03    # obstacle height minimum
    z_max_m: float = 0.80    # obstacle height maximum

    # Body / self-reflection filter
    body_x_min_m: float = -0.40
    body_x_max_m: float = 0.40
    body_y_min_m: float = -0.18
    body_y_max_m: float = 0.18

    # Sector coverage
    sector_min_coverage_points: int = 3
    sector_min_coverage_bins: int = 2
    coverage_bin_width_deg: float = 10.0

    # Wall extraction
    wall_min_points: int = 8
    wall_min_length_m: float = 0.15
    wall_inlier_tolerance_m: float = 0.04
    wall_max_residual_m: float = 0.06
    wall_ransac_sample_limit: int = 200
    wall_max_segments: int = 4
    wall_cluster_gap_m: float = 0.08
    wall_endpoint_uncertainty_m: float = 0.03

    # Fragment extraction (lower thresholds for sparse data)
    wall_fragment_min_points: int = 4
    wall_fragment_min_length_m: float = 0.08
    wall_fragment_max_segments: int = 6
    wall_fragment_association_confidence_min: float = 0.30


@dataclass
class WallSegment:
    """One extracted wall line segment (finite)."""

    segment_id: str = ''
    start: Tuple[float, float] = (0.0, 0.0)
    end: Tuple[float, float] = (0.0, 0.0)
    length_m: float = 0.0
    angle_rad: float = 0.0
    angle_deg: float = 0.0
    direction: Tuple[float, float] = (1.0, 0.0)
    normal: Tuple[float, float] = (0.0, 1.0)
    point_count: int = 0
    residual_rms_m: float = 0.0
    confidence: float = 0.0
    endpoint_uncertainty_m: float = 0.03
    evidence_kind: str = 'wall_fragment'  # 'full_wall' or 'wall_fragment'
    reliable_for_turn: bool = False
    data_age: float = 0.0


@dataclass
class SectorStats:
    """Coverage and distance statistics for one angular sector."""
    name: str = ''
    occupied_cells: int = 0
    coverage_points: int = 0
    coverage_bins: int = 0
    distance_p10_m: float = float('inf')
    valid: bool = False


@dataclass
class GridOutput:
    """Complete output of one grid build cycle."""
    occupied_cells: int = 0
    total_points: int = 0
    finite_points: int = 0
    map_points: int = 0
    body_filtered: int = 0
    height_filtered: int = 0
    wall_segments: List[WallSegment] = field(default_factory=list)
    sector_stats: dict = field(default_factory=dict)
    unmodeled_points: List[Tuple[float, float]] = field(default_factory=list)


class LocalOccupancyGrid:
    """2D occupancy grid with UNKNOWN/FREE/OCCUPIED cell states."""

    def __init__(self, config: LocalGridConfig):
        self._config = config
        self._cols = int((config.x_max_m - config.x_min_m) / config.resolution_m) + 1
        self._rows = int((config.y_max_m - config.y_min_m) / config.resolution_m) + 1
        self._cells: List[int] = [CELL_UNKNOWN] * (self._cols * self._rows)

    @property
    def cols(self) -> int:
        return self._cols

    @property
    def rows(self) -> int:
        return self._rows

    def cell_index(self, x: float, y: float) -> Optional[int]:
        """Convert world coordinates to cell index. Returns None if outside map."""
        col = int((x - self._config.x_min_m) / self._config.resolution_m)
        row = int((y - self._config.y_min_m) / self._config.resolution_m)
        if 0 <= col < self._cols and 0 <= row < self._rows:
            return row * self._cols + col
        return None

    def cell_center(self, index: int) -> Tuple[float, float]:
        """Convert cell index back to world coordinates (cell center)."""
        row = index // self._cols
        col = index % self._cols
        cx = self._config.x_min_m + (col + 0.5) * self._config.resolution_m
        cy = self._config.y_min_m + (row + 0.5) * self._config.resolution_m
        return (cx, cy)

    def mark_occupied(self, x: float, y: float):
        idx = self.cell_index(x, y)
        if idx is not None:
            self._cells[idx] = CELL_OCCUPIED

    def is_occupied(self, x: float, y: float) -> bool:
        idx = self.cell_index(x, y)
        return idx is not None and self._cells[idx] == CELL_OCCUPIED

    def is_unknown(self, x: float, y: float) -> bool:
        idx = self.cell_index(x, y)
        return idx is None or self._cells[idx] == CELL_UNKNOWN

    def occupied_cells(self) -> Set[int]:
        """Return set of cell indices marked OCCUPIED."""
        return {i for i, v in enumerate(self._cells) if v == CELL_OCCUPIED}

    def cell_state(self, x: float, y: float) -> int:
        idx = self.cell_index(x, y)
        if idx is None:
            return CELL_UNKNOWN
        return self._cells[idx]

    def occupied_cell_points(self) -> List[Tuple[float, float]]:
        """Return world coordinates of all occupied cells."""
        points = []
        for idx in self.occupied_cells():
            points.append(self.cell_center(idx))
        return points

    def clearance_to_point(
        self, px: float, py: float, footprint_extents: dict
    ) -> float:
        """Compute minimum distance from footprint to an occupied cell near the given point."""
        min_dist = float('inf')
        for idx in self.occupied_cells():
            cx, cy = self.cell_center(idx)
            dx = cx - px
            dy = cy - py
            dist = math.sqrt(dx * dx + dy * dy)
            if dist < min_dist:
                min_dist = dist
        return min_dist

    @staticmethod
    def point_to_segment_distance(
        point: Tuple[float, float],
        seg_start: Tuple[float, float],
        seg_end: Tuple[float, float],
    ) -> float:
        """Minimum distance from point to finite line segment."""
        px, py = point
        sx, sy = seg_start
        ex, ey = seg_end
        dx = ex - sx
        dy = ey - sy
        seg_len_sq = dx * dx + dy * dy
        if seg_len_sq < 1e-12:
            return math.hypot(px - sx, py - sy)
        t = max(0.0, min(1.0, ((px - sx) * dx + (py - sy) * dy) / seg_len_sq))
        proj_x = sx + t * dx
        proj_y = sy + t * dy
        return math.hypot(px - proj_x, py - proj_y)
