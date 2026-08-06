#!/usr/bin/env python3
"""Unit tests for LiDAR distance processing (F1+F2+F3)."""

import math
import pytest

from rk_maze.lidar_distance_core import (
    Point3D, LidarDistanceConfig, EffectiveCluster, SectorDistance,
    NavigationDistanceFilter, DistanceSnapshot, LiDARSpeedEstimator,
    filter_point_cloud, voxel_downsample, _cluster_points,
    find_effective_clusters, compute_hard_distance, compute_all_hard_distances,
    compute_distance_snapshot, compute_dynamic_stop_distance, classify_speed,
    _classify_sector, _compute_clearance,
    SECTOR_FRONT, SECTOR_LEFT, SECTOR_RIGHT, SECTOR_REAR,
    SPEED_CLEAR, SPEED_CAUTION, SPEED_BRAKE, SPEED_EMERGENCY, SPEED_UNKNOWN,
)


def _default_config(**kwargs) -> LidarDistanceConfig:
    params = {
        'min_range_m': 0.08, 'max_range_m': 3.00,
        'ground_z_min_m': -0.30, 'ground_z_max_m': 0.03,
        'obstacle_z_min_m': 0.03, 'obstacle_z_max_m': 0.80,
        'body_x_min_m': -0.40, 'body_x_max_m': 0.40,
        'body_y_min_m': -0.18, 'body_y_max_m': 0.18,
        'voxel_size_m': 0.02, 'cluster_tolerance_m': 0.05,
        'min_cluster_points': 5, 'cluster_percentile': 0.10,
        'footprint_front_m': 0.35, 'footprint_rear_m': 0.35,
        'footprint_left_m': 0.18, 'footprint_right_m': 0.18,
        'perception_margin_m': 0.03,
        'hard_max_age_sec': 0.15, 'nav_max_age_sec': 0.50,
    }
    params.update(kwargs)
    return LidarDistanceConfig(**params)


# -----------------------------------------------------------------------
# F1: Point cloud filtering
# -----------------------------------------------------------------------

class TestFilterPointCloud:
    def test_nan_filtered(self):
        c = _default_config()
        pts = [Point3D(float('nan'), 1.0, 0.5)]
        assert len(filter_point_cloud(pts, c)) == 0

    def test_inf_filtered(self):
        c = _default_config()
        pts = [Point3D(float('inf'), 1.0, 0.5)]
        assert len(filter_point_cloud(pts, c)) == 0

    def test_neg_inf_filtered(self):
        c = _default_config()
        pts = [Point3D(-float('inf'), 1.0, 0.5)]
        assert len(filter_point_cloud(pts, c)) == 0

    def test_empty_cloud_returns_empty(self):
        c = _default_config()
        assert len(filter_point_cloud([], c)) == 0

    def test_range_too_close_filtered(self):
        c = _default_config(min_range_m=0.10)
        pts = [Point3D(0.05, 0.0, 0.5)]  # 5cm < 10cm
        assert len(filter_point_cloud(pts, c)) == 0

    def test_range_too_far_filtered(self):
        c = _default_config(max_range_m=2.00)
        pts = [Point3D(3.0, 0.0, 0.5)]
        assert len(filter_point_cloud(pts, c)) == 0

    def test_ground_points_filtered(self):
        c = _default_config(ground_z_min_m=-0.30, ground_z_max_m=0.03)
        pts = [Point3D(1.0, 0.0, 0.0)]  # on ground
        assert len(filter_point_cloud(pts, c)) == 0

    def test_body_self_reflection_filtered(self):
        c = _default_config(body_x_min_m=-0.40, body_x_max_m=0.40,
                            body_y_min_m=-0.18, body_y_max_m=0.18)
        pts = [Point3D(0.10, 0.05, 0.50)]  # inside body box
        assert len(filter_point_cloud(pts, c)) == 0

    def test_height_below_obstacle_filtered(self):
        c = _default_config(obstacle_z_min_m=0.03)
        pts = [Point3D(1.0, 0.0, 0.01)]  # below obstacle height
        assert len(filter_point_cloud(pts, c)) == 0

    def test_valid_point_passes(self):
        c = _default_config()
        pts = [Point3D(1.5, 0.3, 0.40)]  # valid obstacle point
        assert len(filter_point_cloud(pts, c)) == 1

    def test_multiple_points_filtered_correctly(self):
        c = _default_config()
        pts = [
            Point3D(1.0, 0.0, 0.40),    # valid
            Point3D(float('nan'), 0, 0), # NaN
            Point3D(0.0, 0.0, 0.0),      # ground
            Point3D(0.1, 0.05, 0.40),    # body
            Point3D(4.0, 0.0, 0.40),     # too far
        ]
        result = filter_point_cloud(pts, c)
        assert len(result) == 1
        assert result[0].x == 1.0


# -----------------------------------------------------------------------
# F2: Effective obstacle clustering
# -----------------------------------------------------------------------

class TestClustering:
    def test_empty_returns_empty(self):
        c = _default_config()
        assert len(find_effective_clusters([], c)) == 0

    def test_single_cluster_detected(self):
        c = _default_config(cluster_tolerance_m=0.10, min_cluster_points=3)
        pts = [
            Point3D(1.0, 0.0, 0.5),
            Point3D(1.02, 0.01, 0.5),
            Point3D(1.04, -0.01, 0.5),
            Point3D(1.01, 0.02, 0.5),
        ]
        clusters = find_effective_clusters(pts, c)
        assert len(clusters) == 1
        assert clusters[0].point_count == 4
        assert clusters[0].valid

    def test_small_cluster_filtered(self):
        c = _default_config(min_cluster_points=5)
        pts = [Point3D(1.0, 0.0, 0.5), Point3D(1.01, 0.0, 0.5)]
        clusters = find_effective_clusters(pts, c)
        assert len(clusters) == 0  # too small

    def test_separated_clusters(self):
        c = _default_config(cluster_tolerance_m=0.10, min_cluster_points=3)
        pts = [
            # Cluster A at 1m
            Point3D(1.0, 0.0, 0.5), Point3D(1.02, 0.01, 0.5),
            Point3D(1.01, -0.01, 0.5),
            # Cluster B at 2m
            Point3D(2.0, 0.0, 0.5), Point3D(2.01, 0.0, 0.5),
            Point3D(2.02, 0.0, 0.5),
        ]
        clusters = find_effective_clusters(pts, c)
        assert len(clusters) == 2
        # Nearest cluster first
        assert clusters[0].conservative_distance < clusters[1].conservative_distance

    def test_single_outlier_does_not_form_cluster(self):
        c = _default_config(min_cluster_points=3)
        pts = [
            Point3D(0.10, 0.0, 0.5),  # isolated — should be discarded
            Point3D(1.5, 0.0, 0.5), Point3D(1.52, 0.01, 0.5),
            Point3D(1.51, -0.01, 0.5),
        ]
        clusters = find_effective_clusters(pts, c)
        assert len(clusters) == 1
        assert clusters[0].conservative_distance > 1.0  # not the outlier

    def test_conservative_distance_uses_percentile(self):
        c = _default_config(cluster_tolerance_m=0.20, min_cluster_points=4,
                            cluster_percentile=0.25)
        pts = [
            Point3D(0.90, 0.0, 0.5),   # closest
            Point3D(0.95, 0.0, 0.5),
            Point3D(1.00, 0.0, 0.5),
            Point3D(1.10, 0.0, 0.5),   # farthest
        ]
        clusters = find_effective_clusters(pts, c)
        # P25 of [0.90, 0.95, 1.00, 1.10] ≈ 0.90 or 0.95
        assert 0.89 < clusters[0].conservative_distance < 0.97


# -----------------------------------------------------------------------
# F2: Hard distance per sector
# -----------------------------------------------------------------------

class TestHardDistance:
    def test_empty_sector_returns_invalid(self):
        c = _default_config()
        sd = compute_hard_distance([], SECTOR_FRONT, c, 100.0, 100.05)
        assert not sd.valid
        assert sd.reason == 'no_points_in_sector'

    def test_valid_sector_with_cluster(self):
        c = _default_config(min_cluster_points=3, cluster_tolerance_m=0.10)
        pts = [
            Point3D(1.20, 0.01, 0.50),
            Point3D(1.21, 0.00, 0.50),
            Point3D(1.22, -0.01, 0.50),
        ]
        sd = compute_hard_distance(pts, SECTOR_FRONT, c, 100.0, 100.01)
        assert sd.valid
        assert sd.point_count == 3
        assert 1.1 < sd.hard_distance < 1.3

    def test_stale_detection(self):
        c = _default_config(hard_max_age_sec=0.15)
        pts = [Point3D(1.0, 0.0, 0.5), Point3D(1.01, 0.0, 0.5),
               Point3D(1.02, 0.0, 0.5)]
        sd = compute_hard_distance(pts, SECTOR_FRONT, c, 100.0, 100.30)
        assert sd.stale

    def test_clearance_from_body_edge(self):
        c = _default_config(footprint_front_m=0.28, perception_margin_m=0.03)
        cluster = EffectiveCluster(
            centroid_x=1.00, centroid_y=0.0, centroid_z=0.5,
            conservative_distance=1.0, point_count=5,
            valid=True,
        )
        fc, rc, lc, right_c = _compute_clearance(cluster, c)
        assert fc == pytest.approx(1.00 - 0.28 - 0.03)

    def test_clearance_with_t0_footprint(self):
        """T0校准: footprint_front从0.35更新为0.28m (LiDAR实际位置)."""
        c = _default_config(footprint_front_m=0.28, perception_margin_m=0.03)
        cluster = EffectiveCluster(
            centroid_x=1.00, centroid_y=0.0, centroid_z=0.5,
            conservative_distance=1.0, point_count=5,
            valid=True,
        )
        fc, rc, lc, right_c = _compute_clearance(cluster, c)
        # fc = 1.00 - 0.28 - 0.03 = 0.69 (vs 旧值 1.00-0.35-0.03=0.62)
        assert fc == pytest.approx(0.69, abs=0.01)


# -----------------------------------------------------------------------
# F3: Navigation distance (temporal filtering)
# -----------------------------------------------------------------------

class TestNavigationDistanceFilter:
    def test_initial_values_infinite(self):
        c = _default_config()
        nf = NavigationDistanceFilter(c)
        result = nf.update({}, 100.0)
        # All should be stale/invalid initially
        for s, sd in result.items():
            assert sd.stale or not sd.valid

    def test_median_filtering(self):
        c = _default_config(nav_temporal_window=3, nav_ema_alpha=1.0, nav_max_jump_m=10.0)
        nf = NavigationDistanceFilter(c)

        # Feed 5 frames with distances to build up EMA and history
        for dist, stamp in [(1.5, 1.0), (1.8, 1.1), (2.0, 1.2), (1.9, 1.3), (2.1, 1.4)]:
            sd = SectorDistance(
                sector=SECTOR_FRONT, hard_distance=dist,
                valid=True, stale=False,
            )
            nf.update({SECTOR_FRONT: sd}, stamp)

        # After multiple frames, the EMA should stabilize near ~1.9
        sd = SectorDistance(sector=SECTOR_FRONT, hard_distance=2.0, valid=True)
        result = nf.update({SECTOR_FRONT: sd}, 1.5)
        nav = result.get(SECTOR_FRONT, SectorDistance()).navigation_distance
        assert 1.0 < nav < 3.0

    def test_ema_smoothing(self):
        c = _default_config(nav_temporal_window=5, nav_ema_alpha=0.5)
        nf = NavigationDistanceFilter(c)

        # Build up with consistent value first to establish EMA
        for dist, stamp in [(2.0, 1.0), (2.0, 1.1), (2.0, 1.2), (2.0, 1.3), (2.0, 1.4)]:
            sd = SectorDistance(sector=SECTOR_FRONT, hard_distance=dist, valid=True)
            nf.update({SECTOR_FRONT: sd}, stamp)

        # Now feed a different value and check EMA smoothing
        sd = SectorDistance(sector=SECTOR_FRONT, hard_distance=1.0, valid=True)
        result = nf.update({SECTOR_FRONT: sd}, 1.5)
        nav = result.get(SECTOR_FRONT, SectorDistance()).navigation_distance
        # After 5 frames of 2.0, EMA should be near 2.0. A single 1.0 should pull it down.
        # With alpha=0.5: 0.5*2.0 + 0.5*1.5 = 1.75 (approximate)
        assert 1.0 < nav <= 2.5  # smoothed

    def test_reset_clears_history(self):
        c = _default_config()
        nf = NavigationDistanceFilter(c)
        sd = SectorDistance(sector=SECTOR_FRONT, hard_distance=1.5, valid=True)
        nf.update({SECTOR_FRONT: sd}, 1.0)
        nf.reset()
        result = nf.update({}, 2.0)
        assert result.get(SECTOR_FRONT, SectorDistance()).stale


# -----------------------------------------------------------------------
# Dynamic stop distance and speed classification
# -----------------------------------------------------------------------

class TestDynamicStopDistance:
    def test_zero_speed(self):
        c = _default_config()
        d = compute_dynamic_stop_distance(0.0, c)
        assert d > 0.0
        assert d < 1.0  # just footprint + margins

    def test_increases_with_speed(self):
        c = _default_config()
        d_slow = compute_dynamic_stop_distance(0.1, c)
        d_fast = compute_dynamic_stop_distance(0.5, c)
        assert d_fast > d_slow


class TestSpeedClassification:
    def test_unknown_on_infinite(self):
        c = _default_config()
        assert classify_speed(float('inf'), float('inf'), 1.0, c) == SPEED_UNKNOWN

    def test_emergency_when_too_close(self):
        c = _default_config(footprint_front_m=0.35, perception_margin_m=0.03)
        dyn_stop = 0.50
        assert classify_speed(0.30, 0.30, dyn_stop, c) == SPEED_EMERGENCY

    def test_brake_when_below_stop_distance(self):
        c = _default_config()
        dyn_stop = 0.60
        assert classify_speed(0.55, 0.55, dyn_stop, c) == SPEED_BRAKE

    def test_caution_when_below_150pct(self):
        c = _default_config()
        dyn_stop = 0.60
        assert classify_speed(0.85, 0.85, dyn_stop, c) == SPEED_CAUTION

    def test_clear_when_sufficient(self):
        c = _default_config()
        dyn_stop = 0.60
        assert classify_speed(1.50, 1.50, dyn_stop, c) == SPEED_CLEAR


# -----------------------------------------------------------------------
# Sector classification
# -----------------------------------------------------------------------

class TestSectorClassification:
    def test_front(self):
        c = _default_config(front_half_angle_deg=10.0)
        assert _classify_sector(1.0, 0.0, c) == SECTOR_FRONT
        assert _classify_sector(1.0, 0.05, c) == SECTOR_FRONT  # ~2.9°

    def test_left_front(self):
        c = _default_config(diagonal_min_angle_deg=10.0, diagonal_max_angle_deg=45.0)
        assert _classify_sector(1.0, 0.5, c) == 'left_front'  # ~26.6°

    def test_right_front(self):
        c = _default_config(diagonal_min_angle_deg=10.0, diagonal_max_angle_deg=45.0)
        assert _classify_sector(1.0, -0.5, c) == 'right_front'

    def test_left(self):
        c = _default_config(side_min_angle_deg=45.0, side_max_angle_deg=90.0)
        assert _classify_sector(0.1, 1.0, c) == SECTOR_LEFT  # ~84.3°

    def test_right(self):
        c = _default_config(side_min_angle_deg=45.0, side_max_angle_deg=90.0)
        assert _classify_sector(0.1, -1.0, c) == SECTOR_RIGHT

    def test_rear(self):
        c = _default_config()
        assert _classify_sector(-1.0, 0.0, c) == SECTOR_REAR


# -----------------------------------------------------------------------
# Voxel downsampling
# -----------------------------------------------------------------------

class TestVoxelDownsample:
    def test_merges_nearby_points(self):
        pts = [
            Point3D(1.000, 0.000, 0.500),
            Point3D(1.005, 0.005, 0.505),  # within same 2cm voxel
            Point3D(1.100, 0.000, 0.500),  # different voxel
        ]
        result = voxel_downsample(pts, 0.02)
        assert 2 <= len(result) <= 3


# -----------------------------------------------------------------------
# Full distance snapshot
# -----------------------------------------------------------------------

class TestLiDARSpeedEstimator:
    def test_insufficient_samples_returns_zero(self):
        est = LiDARSpeedEstimator(min_samples=5, history_duration_sec=0.50)
        est.update(1.0, 1.00)
        est.update(1.1, 0.99)
        assert est.speed(1.2) == 0.0  # only 2 samples < min 5

    def test_approaching_detected(self):
        est = LiDARSpeedEstimator(min_samples=5, stale_timeout_sec=10.0)
        # Distance decreasing: 1.00 → 0.95 over 0.5s = speed 0.10 m/s
        for i in range(8):
            est.update(1.0 + i * 0.05, 1.00 - i * 0.01)
        speed = est.speed(1.5)
        assert 0.05 < speed < 0.20  # approaching ~0.1 m/s

    def test_stationary_returns_zero(self):
        est = LiDARSpeedEstimator(min_samples=5, stale_timeout_sec=10.0)
        for i in range(8):
            est.update(1.0 + i * 0.05, 1.00)  # no change
        speed = est.speed(1.5)
        assert speed == 0.0  # distance not changing

    def test_moving_away_negative_speed(self):
        est = LiDARSpeedEstimator(min_samples=5, stale_timeout_sec=10.0)
        for i in range(8):
            est.update(1.0 + i * 0.05, 1.00 + i * 0.02)  # distance increasing
        speed = est.speed(1.5)
        assert speed < 0.0  # moving away → negative

    def test_stale_returns_zero(self):
        est = LiDARSpeedEstimator(min_samples=5, stale_timeout_sec=0.10)
        for i in range(8):
            est.update(1.0 + i * 0.01, 1.00 - i * 0.01)
        speed = est.speed(2.0)  # 1.0s after last update, > stale_timeout
        assert speed == 0.0

    def test_max_speed_clamped(self):
        est = LiDARSpeedEstimator(min_samples=5, max_speed_m_s=0.30)
        for i in range(8):
            est.update(1.0 + i * 0.01, 1.50 - i * 0.10)  # very fast approach
        speed = est.speed(1.1)
        assert abs(speed) <= 0.30  # clamped

    def test_reset_clears_all(self):
        est = LiDARSpeedEstimator(min_samples=5)
        for i in range(8):
            est.update(1.0 + i * 0.05, 1.00 - i * 0.01)
        est.reset()
        assert est.sample_count == 0
        assert est.speed(1.5) == 0.0


class TestDistanceSnapshot:
    def test_invalid_when_no_points(self):
        c = _default_config()
        nf = NavigationDistanceFilter(c)
        snap = compute_distance_snapshot([], c, nf, 100.0, 100.05)
        assert not snap.valid
        assert snap.stale
