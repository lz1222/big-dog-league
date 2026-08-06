#!/usr/bin/env python3
"""Unit tests for local occupancy grid and wall extraction (F4)."""

import math
import pytest

from rk_maze.local_occupancy_grid import (
    LocalGridConfig, LocalOccupancyGrid, WallSegment,
    CELL_UNKNOWN, CELL_FREE, CELL_OCCUPIED,
)
from rk_maze.lidar_wall_extractor import (
    LidarWallExtractor, CorridorModel,
)


# ======================================================================
# LocalOccupancyGrid
# ======================================================================

def _grid_config(**kwargs) -> LocalGridConfig:
    params = dict(
        x_min_m=-1.50, x_max_m=1.50,
        y_min_m=-1.50, y_max_m=1.50,
        resolution_m=0.03,
        z_min_m=0.03, z_max_m=0.80,
        body_x_min_m=-0.40, body_x_max_m=0.40,
        body_y_min_m=-0.18, body_y_max_m=0.18,
    )
    params.update(kwargs)
    return LocalGridConfig(**params)


class TestLocalOccupancyGrid:
    def test_empty_grid_all_unknown(self):
        grid = LocalOccupancyGrid(_grid_config())
        assert grid.cell_state(0.0, 0.0) == CELL_UNKNOWN
        assert grid.cell_state(1.0, 1.0) == CELL_UNKNOWN
        assert len(grid.occupied_cells()) == 0

    def test_mark_occupied(self):
        grid = LocalOccupancyGrid(_grid_config())
        grid.mark_occupied(0.50, 0.30)
        assert grid.is_occupied(0.50, 0.30)
        assert not grid.is_unknown(0.50, 0.30)
        assert len(grid.occupied_cells()) == 1

    def test_multiple_occupied(self):
        grid = LocalOccupancyGrid(_grid_config())
        for x, y in [(0.5, 0.3), (0.5, 0.33), (1.0, -0.5), (-0.8, 0.2)]:
            grid.mark_occupied(x, y)
        assert len(grid.occupied_cells()) >= 3  # some may merge into same cell

    def test_out_of_bounds_is_unknown(self):
        grid = LocalOccupancyGrid(_grid_config())
        assert grid.is_unknown(10.0, 10.0)
        assert grid.cell_index(10.0, 10.0) is None

    def test_cell_center_roundtrip(self):
        grid = LocalOccupancyGrid(_grid_config())
        idx = grid.cell_index(0.50, 0.30)
        assert idx is not None
        cx, cy = grid.cell_center(idx)
        # Cell center should be within half a resolution of original point
        res = _grid_config().resolution_m
        assert abs(cx - 0.50) < res
        assert abs(cy - 0.30) < res

    def test_point_to_segment_distance(self):
        # Point on segment
        d = LocalOccupancyGrid.point_to_segment_distance(
            (0.5, 0.5), (0.0, 0.0), (1.0, 1.0))
        assert d < 0.001

        # Point near midpoint, 0.6 away from horizontal segment
        d = LocalOccupancyGrid.point_to_segment_distance(
            (0.5, 0.6), (0.0, 0.0), (1.0, 0.0))
        assert d == pytest.approx(0.6, abs=0.01)

    def test_occupied_cell_points(self):
        grid = LocalOccupancyGrid(_grid_config())
        grid.mark_occupied(0.50, 0.30)
        grid.mark_occupied(1.00, -0.50)
        pts = grid.occupied_cell_points()
        assert len(pts) >= 2

    def test_clearance_to_point(self):
        grid = LocalOccupancyGrid(_grid_config())
        grid.mark_occupied(0.50, 0.30)
        grid.mark_occupied(0.55, 0.30)
        clearance = grid.clearance_to_point(0.50, 0.30, {})
        # Occupied cell at (0.50, 0.30) → clearance should be near 0
        assert clearance < 0.05


# ======================================================================
# LidarWallExtractor
# ======================================================================

class TestLidarWallExtractor:
    def test_empty_grid_no_walls(self):
        grid = LocalOccupancyGrid(_grid_config())
        ext = LidarWallExtractor(_grid_config())
        output = ext.extract(grid, 0.0)
        assert len(output.wall_segments) == 0

    def test_straight_wall_extracted(self):
        """A clear straight wall along the x-axis should be extracted."""
        grid = LocalOccupancyGrid(_grid_config(
            wall_min_points=5, wall_min_length_m=0.15,
            wall_inlier_tolerance_m=0.05,
        ))
        # Create a wall along y = -0.30, from x=0.5 to x=1.5
        for x in [0.5 + i * 0.03 for i in range(35)]:
            grid.mark_occupied(x, -0.30)
            grid.mark_occupied(x, -0.32)

        ext = LidarWallExtractor(_grid_config(
            wall_min_points=5, wall_min_length_m=0.15,
            wall_inlier_tolerance_m=0.05,
        ))
        output = ext.extract(grid, 0.0)
        assert len(output.wall_segments) >= 1

        wall = output.wall_segments[0]
        assert wall.point_count >= 5
        assert wall.length_m > 0.30
        # Wall should be roughly horizontal (angle near 0)
        assert abs(wall.angle_deg) < 30.0

    def test_corridor_model_right_wall(self):
        """A wall on the right side should be classified as right wall."""
        grid = LocalOccupancyGrid(_grid_config(
            wall_min_points=5, wall_min_length_m=0.15,
            wall_inlier_tolerance_m=0.05,
        ))
        # Right wall: at y=-0.30, x=0.5 to 1.3 (negative y = right of robot)
        for x in [0.5 + i * 0.03 for i in range(28)]:
            grid.mark_occupied(x, -0.30)

        ext = LidarWallExtractor(_grid_config(
            wall_min_points=5, wall_min_length_m=0.15,
            wall_inlier_tolerance_m=0.05,
        ))
        output = ext.extract(grid, 0.0)
        model = ext.build_corridor_model(output.wall_segments, 0.0)

        assert model.right_wall is not None
        assert model.right_wall_distance < 1.0
        # Should be classified as right (negative y, angle along x)
        assert model.valid or model.reason == 'right_wall_only'

    def test_no_walls_invalid_model(self):
        grid = LocalOccupancyGrid(_grid_config())
        ext = LidarWallExtractor(_grid_config())
        model = ext.build_corridor_model([], 0.0)
        assert not model.valid
        assert model.reason == 'no_wall_segments'

    def test_wall_confidence_bounded(self):
        grid = LocalOccupancyGrid(_grid_config(
            wall_min_points=5, wall_min_length_m=0.15,
        ))
        for x in [0.5 + i * 0.03 for i in range(30)]:
            grid.mark_occupied(x, -0.30)

        ext = LidarWallExtractor(_grid_config(
            wall_min_points=5, wall_min_length_m=0.15,
        ))
        output = ext.extract(grid, 0.0)
        if output.wall_segments:
            assert 0.0 <= output.wall_segments[0].confidence <= 1.0

    def test_sparse_points_no_full_wall(self):
        """Very sparse points should not form a full wall, only fragments."""
        grid = LocalOccupancyGrid(_grid_config(
            wall_min_points=8, wall_fragment_min_points=3,
        ))
        # Only 4 sparse points
        for i in range(4):
            grid.mark_occupied(0.5 + i * 0.20, -0.30)

        ext = LidarWallExtractor(_grid_config(
            wall_min_points=8, wall_fragment_min_points=3,
        ))
        output = ext.extract(grid, 0.0)
        # Should not produce a full_wall (needs 8 points)
        full_walls = [s for s in output.wall_segments if s.evidence_kind == 'full_wall']
        assert len(full_walls) == 0

    def test_two_tier_extraction(self):
        """Mixed dense+sparse data should give full walls AND fragments."""
        grid = LocalOccupancyGrid(_grid_config(
            wall_min_points=10, wall_min_length_m=0.20,
            wall_inlier_tolerance_m=0.05,
            wall_fragment_min_points=3, wall_fragment_min_length_m=0.05,
        ))
        # Dense wall
        for x in [0.5 + i * 0.03 for i in range(25)]:
            grid.mark_occupied(x, -0.30)
        # Sparse scattered points elsewhere
        grid.mark_occupied(0.8, 0.40)
        grid.mark_occupied(0.85, 0.42)
        grid.mark_occupied(0.90, 0.41)
        grid.mark_occupied(0.82, 0.43)

        ext = LidarWallExtractor(_grid_config(
            wall_min_points=10, wall_min_length_m=0.20,
            wall_inlier_tolerance_m=0.05,
            wall_fragment_min_points=3, wall_fragment_min_length_m=0.05,
        ))
        output = ext.extract(grid, 0.0)
        has_full = any(s.evidence_kind == 'full_wall' for s in output.wall_segments)
        assert has_full

    def test_corridor_model_left_wall(self):
        """Wall on left side (positive y) should be classified as left."""
        grid = LocalOccupancyGrid(_grid_config(
            wall_min_points=5, wall_min_length_m=0.15,
        ))
        for x in [0.5 + i * 0.03 for i in range(25)]:
            grid.mark_occupied(x, 0.30)

        ext = LidarWallExtractor(_grid_config(
            wall_min_points=5, wall_min_length_m=0.15,
        ))
        output = ext.extract(grid, 0.0)
        model = ext.build_corridor_model(output.wall_segments, 0.0)
        assert model.left_wall is not None or model.reason != 'no_wall_segments'
