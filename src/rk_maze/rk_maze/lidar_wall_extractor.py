#!/usr/bin/env python3
"""F4b: LiDAR wall line extraction using RANSAC + PCA refinement.

Extracts finite wall line segments from the local occupancy grid.
Two-tier extraction: full walls (high confidence) then wall fragments (sparse).

Outputs per wall segment:
- direction, normal, distance
- segment start/end (finite)
- point_count, fit_rmse, confidence
- data_age

Used for:
- corridor_heading estimation
- lateral_error = (left - right) / 2
- wall-endpoint collision detection
"""

import math
from dataclasses import dataclass, field
from typing import List, Optional, Set, Tuple

from rk_maze.local_occupancy_grid import (
    LocalGridConfig, LocalOccupancyGrid, WallSegment, GridOutput,
)


@dataclass
class CorridorModel:
    """Corridor geometry extracted from wall lines."""

    corridor_heading: Optional[float] = None  # radians, long-axis direction
    left_wall_distance: float = float('inf')
    right_wall_distance: float = float('inf')
    front_wall_distance: float = float('inf')
    left_wall: Optional[WallSegment] = None
    right_wall: Optional[WallSegment] = None
    front_wall: Optional[WallSegment] = None
    wall_segments: List[WallSegment] = field(default_factory=list)
    confidence: float = 0.0
    valid: bool = False
    reason: str = 'uninitialized'
    data_age: float = 0.0


class LidarWallExtractor:
    """Extract wall line segments from occupied grid cells."""

    def __init__(self, config: LocalGridConfig):
        self._config = config

    def extract(self, grid: LocalOccupancyGrid, timestamp: float) -> GridOutput:
        """Build grid from point cloud and extract wall segments."""
        occupied_cells = grid.occupied_cells()
        wall_segments = self._extract_wall_segments(occupied_cells)

        return GridOutput(
            occupied_cells=len(occupied_cells),
            wall_segments=wall_segments,
        )

    def build_corridor_model(
        self,
        wall_segments: List[WallSegment],
        timestamp: float,
        odom_yaw: float = 0.0,
    ) -> CorridorModel:
        """Classify wall segments into left/right/front and compute corridor model."""
        model = CorridorModel(
            wall_segments=wall_segments,
            data_age=0.0,
        )

        if not wall_segments:
            model.reason = 'no_wall_segments'
            return model

        # Classify walls by position relative to robot
        # side walls: roughly parallel to robot x-axis (|angle| < 45°)
        #   left: positive y (robot left side)
        #   right: negative y (robot right side)
        # front/back walls: roughly perpendicular to robot x-axis (|angle| > 45°)
        left_walls = []
        right_walls = []
        front_walls = []

        for seg in wall_segments:
            if not seg.reliable_for_turn and seg.evidence_kind != 'full_wall':
                continue
            sx, sy = seg.start
            ex, ey = seg.end
            mid_x = (sx + ex) / 2.0
            mid_y = (sy + ey) / 2.0

            # Wall orientation relative to robot x-axis
            abs_angle = abs(seg.angle_deg)
            # Normalize to [0, 90]
            if abs_angle > 90.0:
                abs_angle = 180.0 - abs_angle

            if abs_angle <= 45.0:
                # Wall runs roughly along x → side wall
                if mid_y >= 0.0:
                    left_walls.append(seg)
                else:
                    right_walls.append(seg)
            else:
                # Wall runs roughly along y → front/back wall
                if mid_x >= 0.0:
                    front_walls.append(seg)
                # else: back wall (ignore for corridor model)

        # Select best wall per side (highest confidence)
        left_walls.sort(key=lambda s: -s.confidence)
        right_walls.sort(key=lambda s: -s.confidence)
        front_walls.sort(key=lambda s: -s.confidence)

        if left_walls:
            model.left_wall = left_walls[0]
            model.left_wall_distance = math.hypot(
                (left_walls[0].start[0] + left_walls[0].end[0]) / 2.0,
                (left_walls[0].start[1] + left_walls[0].end[1]) / 2.0,
            )
        if right_walls:
            model.right_wall = right_walls[0]
            model.right_wall_distance = math.hypot(
                (right_walls[0].start[0] + right_walls[0].end[0]) / 2.0,
                (right_walls[0].start[1] + right_walls[0].end[1]) / 2.0,
            )
        if front_walls:
            model.front_wall = front_walls[0]
            model.front_wall_distance = math.hypot(
                (front_walls[0].start[0] + front_walls[0].end[0]) / 2.0,
                (front_walls[0].start[1] + front_walls[0].end[1]) / 2.0,
            )

        # Compute corridor heading from left+right walls
        if model.left_wall and model.right_wall:
            # Average of left and right wall directions
            heading_sum = model.left_wall.angle_rad + model.right_wall.angle_rad
            model.corridor_heading = heading_sum / 2.0
            model.confidence = (
                model.left_wall.confidence + model.right_wall.confidence
            ) / 2.0
            model.valid = True
            model.reason = 'two_walls'
        elif model.left_wall:
            model.corridor_heading = model.left_wall.angle_rad
            model.confidence = model.left_wall.confidence * 0.7
            model.valid = model.left_wall.confidence >= 0.5
            model.reason = 'left_wall_only'
        elif model.right_wall:
            model.corridor_heading = model.right_wall.angle_rad
            model.confidence = model.right_wall.confidence * 0.7
            model.valid = model.right_wall.confidence >= 0.5
            model.reason = 'right_wall_only'
        else:
            model.reason = 'no_side_walls'

        return model

    # ------------------------------------------------------------------
    # Wall extraction algorithm (RANSAC + PCA refinement)
    # ------------------------------------------------------------------

    def _extract_wall_segments(
        self, occupied_cells: Set[int]
    ) -> List[WallSegment]:
        """Two-tier extraction: full walls first, then sparse fragments."""
        # Convert cell indices back to world coordinates
        points = []
        for idx in sorted(occupied_cells):
            row = idx // self._cols()
            col = idx % self._cols()
            cx = self._config.x_min_m + (col + 0.5) * self._config.resolution_m
            cy = self._config.y_min_m + (row + 0.5) * self._config.resolution_m
            points.append((cx, cy))

        remaining = list(points)
        segments = []

        # Tier 1: full walls
        full = self._extract_tier(
            remaining,
            min_points=self._config.wall_min_points,
            min_length_m=self._config.wall_min_length_m,
            max_segments=self._config.wall_max_segments,
            prefix='wall',
            evidence='full_wall',
            reliable=True,
        )
        segments.extend(full)
        remaining = self._remove_inliers(remaining, full)

        # Tier 2: fragments from remaining sparse points
        fragments = self._extract_tier(
            remaining,
            min_points=self._config.wall_fragment_min_points,
            min_length_m=self._config.wall_fragment_min_length_m,
            max_segments=self._config.wall_fragment_max_segments,
            prefix='fragment',
            evidence='wall_fragment',
            reliable=False,
        )
        segments.extend(fragments)

        return segments

    def _extract_tier(
        self,
        points: List[Tuple[float, float]],
        *,
        min_points: int,
        min_length_m: float,
        max_segments: int,
        prefix: str,
        evidence: str,
        reliable: bool,
    ) -> List[WallSegment]:
        """Extract one tier of wall segments from remaining points."""
        remaining = list(points)
        segments = []

        while len(remaining) >= min_points and len(segments) < max_segments:
            cluster = self._best_wall_cluster(
                remaining, min_points=min_points, min_length_m=min_length_m,
            )
            if cluster is None:
                break
            seg = self._fit_wall_segment(
                cluster, min_points=min_points, min_length_m=min_length_m,
            )
            if seg is None:
                break
            seg.segment_id = f'{prefix}_{len(segments):03d}'
            seg.evidence_kind = evidence
            seg.reliable_for_turn = reliable
            segments.append(seg)
            remaining = self._remove_inliers(remaining, [seg])
        return segments

    def _remove_inliers(
        self,
        points: List[Tuple[float, float]],
        segments: List[WallSegment],
    ) -> List[Tuple[float, float]]:
        """Remove points that are explained by the given wall segments."""
        remaining = list(points)
        for seg in segments:
            remaining = [
                p for p in remaining
                if LocalOccupancyGrid.point_to_segment_distance(
                    p, seg.start, seg.end,
                ) > self._config.wall_inlier_tolerance_m
            ]
        return remaining

    def _best_wall_cluster(
        self,
        points: List[Tuple[float, float]],
        *,
        min_points: int,
        min_length_m: float,
    ) -> Optional[List[Tuple[float, float]]]:
        """Deterministic RANSAC: sample point pairs, find best inlier cluster."""
        sample = self._deterministic_sample(
            points, self._config.wall_ransac_sample_limit,
        )
        best_cluster = None
        best_score = None

        for i in range(len(sample)):
            for j in range(i + 1, len(sample)):
                p1 = sample[i]
                p2 = sample[j]
                dx = p2[0] - p1[0]
                dy = p2[1] - p1[1]
                length = math.hypot(dx, dy)
                if length < min_length_m:
                    continue

                # Line model from p1, p2
                direction = (dx / length, dy / length)
                normal = (-direction[1], direction[0])

                # Find inliers within tolerance
                inliers = []
                for point in points:
                    residual = abs(
                        (point[0] - p1[0]) * normal[0]
                        + (point[1] - p1[1]) * normal[1]
                    )
                    if residual <= self._config.wall_inlier_tolerance_m:
                        projection = (
                            (point[0] - p1[0]) * direction[0]
                            + (point[1] - p1[1]) * direction[1]
                        )
                        inliers.append((projection, point))

                # Split into contiguous projection clusters
                for cluster in self._split_projection_clusters(inliers):
                    if len(cluster) < min_points:
                        continue
                    span = cluster[-1][0] - cluster[0][0]
                    if span < min_length_m:
                        continue
                    # Score: (point_count, span_length)
                    score = (len(cluster), span)
                    if best_score is None or score > best_score:
                        best_cluster = [p for _, p in cluster]
                        best_score = score

        return best_cluster

    def _split_projection_clusters(
        self, projected: List[Tuple[float, Tuple[float, float]]]
    ) -> List[List[Tuple[float, Tuple[float, float]]]]:
        """Split inliers into contiguous clusters along projection axis."""
        if not projected:
            return []
        ordered = sorted(projected, key=lambda item: item[0])
        clusters = [[ordered[0]]]
        for item in ordered[1:]:
            if item[0] - clusters[-1][-1][0] > self._config.wall_cluster_gap_m:
                clusters.append([item])
            else:
                clusters[-1].append(item)
        return clusters

    def _fit_wall_segment(
        self,
        points: List[Tuple[float, float]],
        *,
        min_points: int,
        min_length_m: float,
    ) -> Optional[WallSegment]:
        """PCA-based refinement: fit principal axis to cluster points."""
        n = len(points)
        if n < min_points:
            return None

        cx = sum(p[0] for p in points) / n
        cy = sum(p[1] for p in points) / n

        # Covariance
        cxx = sum((p[0] - cx) ** 2 for p in points) / n
        cyy = sum((p[1] - cy) ** 2 for p in points) / n
        cxy = sum((p[0] - cx) * (p[1] - cy) for p in points) / n

        angle = 0.5 * math.atan2(2.0 * cxy, cxx - cyy)
        direction = (math.cos(angle), math.sin(angle))
        normal = (-direction[1], direction[0])

        projections = [
            (p[0] - cx) * direction[0] + (p[1] - cy) * direction[1]
            for p in points
        ]
        residuals = [
            abs((p[0] - cx) * normal[0] + (p[1] - cy) * normal[1])
            for p in points
        ]

        start_proj = min(projections)
        end_proj = max(projections)
        length = end_proj - start_proj
        residual_rms = math.sqrt(sum(r * r for r in residuals) / n)

        if (
            n < min_points
            or length < min_length_m
            or residual_rms > self._config.wall_max_residual_m
        ):
            return None

        start = (
            cx + start_proj * direction[0],
            cy + start_proj * direction[1],
        )
        end = (
            cx + end_proj * direction[0],
            cy + end_proj * direction[1],
        )

        # Confidence
        point_score = min(1.0, n / max(1.0, 2.0 * min_points))
        length_score = min(1.0, length / max(1e-6, 2.0 * min_length_m))
        residual_score = max(0.0, 1.0 - residual_rms / self._config.wall_max_residual_m)
        confidence = point_score * length_score * residual_score

        return WallSegment(
            start=start,
            end=end,
            length_m=length,
            angle_rad=angle,
            angle_deg=math.degrees(angle),
            direction=direction,
            normal=normal,
            point_count=n,
            residual_rms_m=residual_rms,
            confidence=confidence,
            endpoint_uncertainty_m=max(
                self._config.wall_endpoint_uncertainty_m,
                self._config.resolution_m,
                2.0 * residual_rms,
            ),
        )

    def _deterministic_sample(
        self, points: List[Tuple[float, float]], limit: int
    ) -> List[Tuple[float, float]]:
        """Uniform deterministic sampling for reproducible RANSAC."""
        if len(points) <= limit:
            return list(points)
        indices = {
            int(round(i * (len(points) - 1) / (limit - 1)))
            for i in range(limit)
        }
        return [points[i] for i in sorted(indices)]

    def _cols(self) -> int:
        return int(
            (self._config.x_max_m - self._config.x_min_m) / self._config.resolution_m
        ) + 1
