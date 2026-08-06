#!/usr/bin/env python3
"""Unit tests for heading controller, safety arbiter, JointHealthGuard, stop bias estimator."""

import math, pytest
from random import random as _random

from rk_maze.heading_controller import HeadingController, HeadingControllerConfig, HeadingState
from rk_maze.safety_arbiter import SafetyArbiter, SafetyVerdict
from rk_maze.joint_health_guard import (
    JointHealthGuard, JointHealthConfig, JointHealthStatus, MotorState,
    HEALTH_NORMAL, HEALTH_LIMITED, HEALTH_COOLDOWN_REQUIRED,
    HEALTH_HARDWARE_FAULT, HEALTH_SENSOR_INVALID,
)
from rk_maze.stop_bias_estimator import StopBiasEstimator, StopBiasConfig
from rk_maze.lidar_distance_core import SPEED_CLEAR, SPEED_EMERGENCY
from rk_maze.swept_footprint_checker import VelocityCandidate, VERDICT_ROBUST_SAFE


def _make_motor(idx, temp, tau, timestamp=100.0):
    return MotorState(index=idx, temperature=temp, tau_est=tau, timestamp=timestamp)


# ============== Heading Controller ==============
class TestHeadingController:
    def test_yaw_wrap(self):
        hc = HeadingController(HeadingControllerConfig())
        assert abs(hc._normalize_angle(3.5)) < math.pi
        assert abs(hc._normalize_angle(-3.5)) < math.pi

    def test_zero_error_aligned(self):
        hc = HeadingController(HeadingControllerConfig())
        s = hc.compute(corridor_heading=0.0, wall_confidence=1.0, wall_age_sec=0.0,
                       odom_yaw=0.0, odom_age_sec=0.0, imu_wz=0.0, imu_age_sec=0.0,
                       left_clearance=1.0, right_clearance=1.0, now_sec=1.0)
        assert s.valid and abs(s.heading_error_deg) < 0.1

    def test_corrects_heading_error(self):
        hc = HeadingController(HeadingControllerConfig(kp_heading=2.0, kd_gyro=0.0))
        s = hc.compute(corridor_heading=0.0, wall_confidence=1.0, wall_age_sec=0.0,
                       odom_yaw=0.1, odom_age_sec=0.0, imu_wz=0.0, imu_age_sec=0.0,
                       left_clearance=1.0, right_clearance=1.0, now_sec=1.0)
        assert s.wz_reference < 0.0 and s.valid

    def test_deadband(self):
        hc = HeadingController(HeadingControllerConfig(kp_heading=2.0, kd_gyro=0.0, deadband_deg=2.0))
        s = hc.compute(corridor_heading=0.0, wall_confidence=1.0, wall_age_sec=0.0,
                       odom_yaw=0.01, odom_age_sec=0.0, imu_wz=0.0, imu_age_sec=0.0,
                       left_clearance=1.0, right_clearance=1.0, now_sec=1.0)
        assert abs(s.wz_reference) < 0.01

    def test_wz_limiting(self):
        hc = HeadingController(HeadingControllerConfig(kp_heading=10.0, max_wz=0.30, kd_gyro=0.0))
        s = hc.compute(corridor_heading=0.0, wall_confidence=1.0, wall_age_sec=0.0,
                       odom_yaw=1.0, odom_age_sec=0.0, imu_wz=0.0, imu_age_sec=0.0,
                       left_clearance=1.0, right_clearance=1.0, now_sec=1.0)
        assert abs(s.wz_reference) <= 0.30

    def test_imu_gyro_damping(self):
        no_d = HeadingController(HeadingControllerConfig(kd_gyro=0.0))
        with_d = HeadingController(HeadingControllerConfig(kd_gyro=1.0))
        s1 = no_d.compute(corridor_heading=0.0, wall_confidence=1.0, wall_age_sec=0.0,
                          odom_yaw=0.0, odom_age_sec=0.0, imu_wz=0.30, imu_age_sec=0.0,
                          left_clearance=1.0, right_clearance=1.0, now_sec=1.0)
        s2 = with_d.compute(corridor_heading=0.0, wall_confidence=1.0, wall_age_sec=0.0,
                            odom_yaw=0.0, odom_age_sec=0.0, imu_wz=0.30, imu_age_sec=0.0,
                            left_clearance=1.0, right_clearance=1.0, now_sec=2.0)
        assert s2.wz_reference < s1.wz_reference  # damped version more negative

    def test_lateral_center(self):
        hc = HeadingController(HeadingControllerConfig(kp_center=1.0, kp_heading=0.0, kd_gyro=0.0))
        s = hc.compute(corridor_heading=0.0, wall_confidence=1.0, wall_age_sec=0.0,
                       odom_yaw=0.0, odom_age_sec=0.0, imu_wz=0.0, imu_age_sec=0.0,
                       left_clearance=0.20, right_clearance=0.60, now_sec=1.0)
        assert s.wz_reference < 0.0  # closer to left wall → move right

    def test_odom_stale(self):
        hc = HeadingController(HeadingControllerConfig(odom_max_age_sec=0.10))
        s = hc.compute(corridor_heading=0.0, wall_confidence=1.0, wall_age_sec=0.0,
                       odom_yaw=0.0, odom_age_sec=0.20, imu_wz=0.0, imu_age_sec=0.0,
                       left_clearance=1.0, right_clearance=1.0, now_sec=1.0)
        assert s.stale and not s.valid

    def test_nan_protection(self):
        hc = HeadingController(HeadingControllerConfig())
        s = hc.compute(corridor_heading=float('nan'), wall_confidence=1.0, wall_age_sec=0.0,
                       odom_yaw=0.0, odom_age_sec=0.0, imu_wz=0.0, imu_age_sec=0.0,
                       left_clearance=1.0, right_clearance=1.0, now_sec=1.0)
        assert s.stale

    def test_low_confidence_fallback(self):
        hc = HeadingController(HeadingControllerConfig(min_wall_confidence=0.5))
        s = hc.compute(corridor_heading=0.5, wall_confidence=0.3, wall_age_sec=0.0,
                       odom_yaw=0.0, odom_age_sec=0.0, imu_wz=0.0, imu_age_sec=0.0,
                       left_clearance=1.0, right_clearance=1.0, now_sec=1.0)
        assert s.valid and abs(s.heading_error_deg) < 1.0

    def test_wz_rate_limit(self):
        hc = HeadingController(HeadingControllerConfig(kp_heading=5.0, max_wz_rate=0.10))
        s1 = hc.compute(corridor_heading=0.0, wall_confidence=1.0, wall_age_sec=0.0,
                        odom_yaw=0.5, odom_age_sec=0.0, imu_wz=0.0, imu_age_sec=0.0,
                        left_clearance=1.0, right_clearance=1.0, now_sec=1.0)
        s2 = hc.compute(corridor_heading=0.0, wall_confidence=1.0, wall_age_sec=0.0,
                        odom_yaw=1.0, odom_age_sec=0.0, imu_wz=0.0, imu_age_sec=0.0,
                        left_clearance=1.0, right_clearance=1.0, now_sec=1.01)
        assert abs(s2.wz_reference - s1.wz_reference) <= 0.10 * 0.01 + 0.02


# ============== Safety Arbiter ==============
class TestSafetyArbiter:
    def test_estop_overrides(self):
        sa = SafetyArbiter(); sa.set_estop(True)
        v = sa.evaluate(HEALTH_NORMAL, False, False, False, 5.0, SPEED_CLEAR, None)
        assert not v.can_move and v.override_priority == sa.PRIORITY_ESTOP

    def test_hardware_fault_stops(self):
        sa = SafetyArbiter()
        v = sa.evaluate(HEALTH_HARDWARE_FAULT, False, False, False, 5.0, SPEED_CLEAR, None)
        assert not v.can_move

    def test_stale_cloud_stops(self):
        sa = SafetyArbiter()
        v = sa.evaluate(HEALTH_NORMAL, True, False, False, 5.0, SPEED_CLEAR, None)
        assert not v.can_move and 'Cloud' in v.reason

    def test_emergency_stops(self):
        sa = SafetyArbiter()
        v = sa.evaluate(HEALTH_NORMAL, False, False, False, 0.30, SPEED_EMERGENCY,
                        VelocityCandidate(robust_safe=True))
        assert not v.can_move

    def test_no_robust_safe_stops(self):
        sa = SafetyArbiter()
        v = sa.evaluate(HEALTH_NORMAL, False, False, False, 3.0, SPEED_CLEAR,
                        VelocityCandidate(robust_safe=False))
        assert not v.can_move

    def test_limited_allows(self):
        sa = SafetyArbiter()
        v = sa.evaluate(HEALTH_LIMITED, False, False, False, 3.0, SPEED_CLEAR,
                        VelocityCandidate(vx=0.10, robust_safe=True, minimum_clearance=0.20))
        assert v.can_move

    def test_all_clear(self):
        sa = SafetyArbiter()
        v = sa.evaluate(HEALTH_NORMAL, False, False, False, 3.0, SPEED_CLEAR,
                        VelocityCandidate(vx=0.25, wz=0.05, robust_safe=True, minimum_clearance=0.30))
        assert v.can_move and v.command_vx == 0.25


# ============== JointHealthGuard ==============
class TestJointHealthGuard:
    def test_normal(self):
        jh = JointHealthGuard(JointHealthConfig())
        s = jh.update({9: _make_motor(9, 35.0, 1.0), 6: _make_motor(6, 35.0, 1.0)}, 100.0)
        assert s.state == HEALTH_NORMAL and s.valid

    def test_limited_warm(self):
        jh = JointHealthGuard(JointHealthConfig(temp_warn_deg_c=70.0))
        s = jh.update({9: _make_motor(9, 75.0, 3.0), 6: _make_motor(6, 35.0, 1.0)}, 100.0)
        assert s.state == HEALTH_LIMITED

    def test_cooldown_hot(self):
        jh = JointHealthGuard(JointHealthConfig(temp_critical_deg_c=85.0))
        s = jh.update({9: _make_motor(9, 90.0, 3.0), 6: _make_motor(6, 35.0, 1.0)}, 100.0)
        assert s.state == HEALTH_COOLDOWN_REQUIRED

    def test_hardware_fault_extreme(self):
        jh = JointHealthGuard(JointHealthConfig(temp_hardware_fault_deg_c=105.0))
        s = jh.update({9: _make_motor(9, 110.0, 3.0), 6: _make_motor(6, 35.0, 1.0)}, 100.0)
        assert s.state == HEALTH_HARDWARE_FAULT and jh.locked

    def test_sensor_invalid_stale(self):
        jh = JointHealthGuard(JointHealthConfig(max_state_age_sec=0.50))
        s = jh.update({9: _make_motor(9, 35.0, 1.0, 99.0), 6: _make_motor(6, 35.0, 1.0, 99.0)}, 100.0)
        assert s.state == HEALTH_SENSOR_INVALID

    def test_sensor_invalid_missing(self):
        jh = JointHealthGuard(JointHealthConfig())
        s = jh.update({}, 100.0)
        assert s.state == HEALTH_SENSOR_INVALID

    def test_high_torque_cooldown(self):
        jh = JointHealthGuard(JointHealthConfig(torque_critical_nm=8.0))
        s = jh.update({9: _make_motor(9, 35.0, 10.0), 6: _make_motor(6, 35.0, 1.0)}, 100.0)
        assert s.state == HEALTH_COOLDOWN_REQUIRED

    def test_lost_count(self):
        jh = JointHealthGuard(JointHealthConfig(max_lost_count=3))
        s = jh.update({9: MotorState(index=9, temperature=35.0, tau_est=1.0, lost=5, timestamp=100.0),
                        6: _make_motor(6, 35.0, 1.0)}, 100.0)
        assert s.state == HEALTH_SENSOR_INVALID


# ============== Stop Bias Estimator ==============
class TestStopBiasEstimator:
    def test_insufficient_samples(self):
        sbe = StopBiasEstimator(StopBiasConfig(min_samples=10))
        for i in range(5): sbe.record_stop(0.0, 0.0, 0.02, 0.5, 35.0, 35.0, 1.0, 1.0)
        est = sbe.get_estimate()
        assert est is not None and not est.enabled and est.sample_count == 5

    def test_enabled_with_stable_data(self):
        sbe = StopBiasEstimator(StopBiasConfig(min_samples=10, max_std_dev_deg=5.0, direction_stability_threshold=0.6))
        for i in range(12): sbe.record_stop(0.0, 0.0, 0.05 + (i % 3) * 0.002, 0.5, 35.0, 35.0, 1.0, 1.0)
        assert sbe.get_estimate().enabled and sbe.get_estimate().bias_rad > 0.0

    def test_unstable_not_enabled(self):
        sbe = StopBiasEstimator(StopBiasConfig(min_samples=10, direction_stability_threshold=0.8))
        for i in range(12): sbe.record_stop(0.0, 0.0, 0.05 if i % 2 == 0 else -0.05, 0.5, 35.0, 35.0, 1.0, 1.0)
        assert not sbe.get_estimate().enabled

    def test_compensation_reduces_error(self):
        sbe = StopBiasEstimator(StopBiasConfig(min_samples=10, compensation_ratio=0.30))
        for i in range(12): sbe.record_stop(0.0, 0.0, 0.05 + _random() * 0.005, 0.5, 35.0, 35.0, 1.0, 1.0)
        if sbe.get_estimate().enabled:
            pre, applied = sbe.get_compensation_angle(0.0)
            assert applied and pre < 0.0

    def test_final_error_classification(self):
        sbe = StopBiasEstimator(StopBiasConfig())
        assert sbe.classify_final_error(math.radians(0.5))[0] == 'complete'
        assert sbe.classify_final_error(math.radians(2.0))[0] == 'correct_next_move'
        assert sbe.classify_final_error(math.radians(4.0))[0] == 'one_arc_correction'
        assert sbe.classify_final_error(math.radians(6.0))[0] == 'fault_stop'

    def test_reset(self):
        sbe = StopBiasEstimator(StopBiasConfig(min_samples=10))
        for i in range(15): sbe.record_stop(0.0, 0.0, 0.05, 0.5, 35.0, 35.0, 1.0, 1.0)
        sbe.reset()
        assert sbe.sample_count == 0 and sbe.get_estimate() is None
