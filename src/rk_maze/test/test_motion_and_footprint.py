#!/usr/bin/env python3
"""Unit tests for motion-compensated cloud buffer and swept footprint checker."""

import math, pytest
from rk_maze.motion_compensated_cloud import (
    MotionCompensatedCloudBuffer, MotionCompensatedCloudConfig, OdomPose,
)
from rk_maze.swept_footprint_checker import (
    DynamicFootprint, SweptFootprintChecker, SweptFootprintConfig,
    VelocityCandidate, Pose2D, Point3D,
    VERDICT_ROBUST_SAFE, VERDICT_NOMINAL_SAFE, VERDICT_UNSAFE, VERDICT_ORDER,
)


# ============== Motion Compensated Cloud ==============
class TestMotionCompensatedCloud:
    def test_empty(self):
        buf = MotionCompensatedCloudBuffer(MotionCompensatedCloudConfig())
        assert buf.get_compensated_cloud() == []

    def test_single_cloud(self):
        buf = MotionCompensatedCloudBuffer(MotionCompensatedCloudConfig())
        pts = [Point3D(1.0, 0.0, 0.5)]
        buf.add_cloud(pts, 100.0, OdomPose(stamp_sec=100.0))
        result = buf.get_compensated_cloud()
        assert len(result) == 1 and abs(result[0].x - 1.0) < 0.001

    def test_odom_jump_clears(self):
        buf = MotionCompensatedCloudBuffer(MotionCompensatedCloudConfig(odom_max_jump_m=0.20))
        buf.update_odom(OdomPose(stamp_sec=100.0))
        buf.add_cloud([Point3D(1.0,0,0.5)], 100.0, OdomPose(stamp_sec=100.0))
        buf.update_odom(OdomPose(stamp_sec=100.05, x=2.0))
        assert buf.accumulated_frames == 0

    def test_timestamp_regression_clears(self):
        buf = MotionCompensatedCloudBuffer(MotionCompensatedCloudConfig())
        buf.add_cloud([Point3D(1.0,0,0.5)], 100.0, OdomPose(stamp_sec=100.0))
        buf.add_cloud([Point3D(1.0,0,0.5)], 99.0, OdomPose(stamp_sec=99.0))
        assert 'regression' in buf.last_clear_reason.lower()

    def test_is_odom_fresh(self):
        buf = MotionCompensatedCloudBuffer(MotionCompensatedCloudConfig(odom_max_age_sec=0.10))
        assert not buf.is_odom_fresh(100.0)
        buf.update_odom(OdomPose(stamp_sec=100.0))
        assert buf.is_odom_fresh(100.05) and not buf.is_odom_fresh(100.20)


# ============== Swept Footprint Checker ==============
def _mk_fp():
    return DynamicFootprint(footprint_front_m=0.28, footprint_rear_m=0.42,
                            footprint_left_m=0.18, footprint_right_m=0.18)

def _mk_cfg():
    return SweptFootprintConfig()

class TestSweptFootprintChecker:
    def test_clear_path_robust(self):
        checker = SweptFootprintChecker(_mk_fp(), _mk_cfg())
        cand = VelocityCandidate(name='FWD', vx=0.20, vy=0.0, wz=0.0, duration_sec=1.0)
        result = checker.check(cand, [Point3D(5.0, 0.0, 0.5)], Pose2D())
        assert result.robust_safe and not result.collision

    def test_collision_detected(self):
        checker = SweptFootprintChecker(_mk_fp(), _mk_cfg())
        cand = VelocityCandidate(name='FWD', vx=0.30, vy=0.0, wz=0.0, duration_sec=1.0)
        # Obstacle directly at front-left corner position
        ext = _mk_fp().expanded_extents()
        result = checker.check(cand, [Point3D(ext['front'], ext['left']-0.01, 0.5)], Pose2D())
        assert result.collision or not result.robust_safe

    def test_stop_safe(self):
        checker = SweptFootprintChecker(_mk_fp(), _mk_cfg())
        cand = VelocityCandidate(name='STOP', vx=0.0, vy=0.0, wz=0.0, duration_sec=0.5)
        result = checker.check(cand, [Point3D(0.50, 0.0, 0.5)], Pose2D())
        assert result.robust_safe or not result.collision

    def test_turn_clearance(self):
        checker = SweptFootprintChecker(_mk_fp(), _mk_cfg())
        cand = VelocityCandidate(name='L_ARC', vx=0.10, vy=0.0, wz=0.30, duration_sec=1.5)
        result = checker.check(cand, [Point3D(0.30, -1.0, 0.5)], Pose2D())
        assert result.robust_safe

    def test_expanded_extents(self):
        fp = DynamicFootprint(footprint_front_m=0.28, gait_sway_margin_m=0.03,
                              cloud_uncertainty_margin_m=0.03, odom_uncertainty_margin_m=0.02,
                              motion_model_margin_m=0.02)
        ext = fp.expanded_extents()
        assert abs(ext['front'] - (0.28 + 0.03 + 0.03 + 0.02 + 0.02)) < 0.001

    def test_corners_generated(self):
        checker = SweptFootprintChecker(_mk_fp(), _mk_cfg())
        corners = checker._corners(Pose2D(0, 0, 0), _mk_fp().expanded_extents())
        assert len(corners) == 4

    def test_verdict_ordering(self):
        assert VERDICT_ORDER[VERDICT_ROBUST_SAFE] > VERDICT_ORDER[VERDICT_NOMINAL_SAFE]
        assert VERDICT_ORDER[VERDICT_UNSAFE] == 0
