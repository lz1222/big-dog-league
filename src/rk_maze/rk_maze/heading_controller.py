#!/usr/bin/env python3
"""F7: LiDAR wall-line + Odom + IMU heading fusion controller.

Computes wz_reference from:
  wz_ref = K_heading * heading_error + K_center * lateral_error - K_gyro * imu_wz

The heading reference is primarily from LiDAR wall lines (corridor_heading).
Odom provides short-term yaw tracking and relative displacement.
IMU angular_velocity.z provides fast yaw damping and overshoot suppression.

Key rule: heading controller produces wz_reference as INPUT to the planner.
It MUST NOT modify the selected candidate's wz after planning.
"""

import math
from dataclasses import dataclass
from typing import Optional


@dataclass
class HeadingControllerConfig:
    """Tunable parameters for the heading fusion controller."""

    # PD gains
    kp_heading: float = 1.5        # proportional to heading error
    kp_center: float = 0.8         # proportional to lateral (center) error
    kd_gyro: float = 0.3           # derivative: IMU gyro damping
    ki_heading: float = 0.0        # integral (disabled in v1)

    # Limits
    max_wz: float = 0.50           # rad/s — absolute cap on wz reference
    max_wz_rate: float = 0.30      # rad/s² — max change in wz per second
    deadband_deg: float = 1.5      # heading error deadband in degrees

    # Confidence gates
    min_wall_confidence: float = 0.4    # wall line confidence threshold
    wall_max_age_sec: float = 0.50      # wall data STALE timeout
    odom_max_age_sec: float = 0.10      # odom STALE timeout
    imu_max_age_sec: float = 0.10       # IMU STALE timeout

    # Smoothing
    wz_smoothing_alpha: float = 0.5     # exponential moving average for output


@dataclass
class HeadingState:
    """Output of the heading controller — INPUT to trajectory planner."""

    wz_reference: float = 0.0           # suggested angular velocity (rad/s)
    heading_error_deg: float = 0.0      # corridor_heading - odom_yaw (degrees)
    lateral_error_m: float = 0.0        # (left_clearance - right_clearance) / 2
    corridor_heading: Optional[float] = None  # from LiDAR wall lines (rad)
    odom_yaw: float = 0.0              # from odometry (rad)
    imu_wz: float = 0.0               # from IMU angular_velocity.z (rad/s)
    valid: bool = False
    stale: bool = True
    reason: str = 'uninitialized'

    # Component contributions
    heading_component: float = 0.0
    center_component: float = 0.0
    gyro_component: float = 0.0


class HeadingController:
    """LiDAR/Odom/IMU heading fusion with gyro damping."""

    def __init__(self, config: HeadingControllerConfig):
        self._config = config
        self._prev_wz: float = 0.0
        self._prev_time: float = 0.0
        self._smoothed_wz: float = 0.0
        self._integral_error: float = 0.0

    def compute(
        self,
        corridor_heading: Optional[float],
        wall_confidence: float,
        wall_age_sec: float,
        odom_yaw: float,
        odom_age_sec: float,
        imu_wz: float,
        imu_age_sec: float,
        left_clearance: float,
        right_clearance: float,
        now_sec: float,
        in_turn: bool = False,
    ) -> HeadingState:
        """Compute wz_reference from fused sensor inputs."""

        state = HeadingState(
            corridor_heading=corridor_heading,
            odom_yaw=odom_yaw,
            imu_wz=imu_wz,
        )

        # ---- STALE checks ----
        odom_stale = odom_age_sec > self._config.odom_max_age_sec
        imu_stale = imu_age_sec > self._config.imu_max_age_sec
        wall_stale = wall_age_sec > self._config.wall_max_age_sec

        if odom_stale:
            state.reason = 'odom_stale'
            state.stale = True
            return state
        # IMU is optional: missing/old IMU → PD-only mode (no gyro damping).
        # Only odom staleness is a hard fault because heading error needs odom_yaw.
        if imu_stale or not math.isfinite(imu_wz):
            imu_wz = 0.0  # fallback: no gyro damping

        # ---- NaN/Inf protection ----
        if not math.isfinite(odom_yaw):
            state.reason = 'nan_or_inf_odom'
            state.stale = True
            return state
        if corridor_heading is not None and not math.isfinite(corridor_heading):
            state.reason = 'nan_or_inf_corridor_heading'
            state.stale = True
            return state

        # ---- Determine effective heading reference ----
        use_wall = (
            corridor_heading is not None
            and not wall_stale
            and wall_confidence >= self._config.min_wall_confidence
            and not in_turn
        )

        if use_wall:
            heading_ref = corridor_heading
            state.reason = 'wall_based'
        else:
            # Fallback: use odom yaw as heading reference (no correction)
            heading_ref = odom_yaw
            state.reason = 'odom_fallback' if not wall_stale else 'wall_stale'

        # ---- Heading error ----
        heading_error = self._normalize_angle(heading_ref - odom_yaw)
        heading_error_deg = math.degrees(heading_error)

        # Deadband
        if abs(heading_error_deg) < self._config.deadband_deg:
            heading_error = 0.0
            heading_error_deg = 0.0

        # ---- Lateral (center) error ----
        lateral_error = 0.0
        if math.isfinite(left_clearance) and math.isfinite(right_clearance):
            lateral_error = (left_clearance - right_clearance) / 2.0

        # ---- Compute wz components ----
        heading_component = self._config.kp_heading * heading_error
        center_component = self._config.kp_center * lateral_error
        gyro_component = -self._config.kd_gyro * imu_wz

        # Integral (disabled in v1)
        if self._config.ki_heading > 0.0:
            dt = now_sec - self._prev_time if self._prev_time > 0.0 else 0.0
            if 0.0 < dt < 0.5:
                self._integral_error += heading_error * dt
                # Anti-windup: clamp integral
                max_integral = self._config.max_wz / max(self._config.ki_heading, 0.01)
                self._integral_error = max(-max_integral, min(max_integral, self._integral_error))
            heading_component += self._config.ki_heading * self._integral_error
        else:
            self._integral_error = 0.0

        raw_wz = heading_component + center_component + gyro_component

        # ---- Wz limiting ----
        raw_wz = max(-self._config.max_wz, min(self._config.max_wz, raw_wz))

        # ---- Wz rate limiting ----
        if self._prev_time > 0.0:
            dt = now_sec - self._prev_time
            if dt > 0.0 and dt < 0.5:
                max_change = self._config.max_wz_rate * dt
                change = raw_wz - self._prev_wz
                if abs(change) > max_change:
                    raw_wz = self._prev_wz + math.copysign(max_change, change)

        # ---- Smoothing ----
        alpha = self._config.wz_smoothing_alpha
        self._smoothed_wz = alpha * raw_wz + (1.0 - alpha) * self._smoothed_wz

        self._prev_wz = self._smoothed_wz
        self._prev_time = now_sec

        # ---- Populate state ----
        state.wz_reference = self._smoothed_wz
        state.heading_error_deg = heading_error_deg
        state.lateral_error_m = lateral_error
        state.heading_component = heading_component
        state.center_component = center_component
        state.gyro_component = gyro_component
        state.valid = True
        state.stale = False

        return state

    def reset(self):
        """Reset internal state on sensor fault or mode change."""
        self._prev_wz = 0.0
        self._prev_time = 0.0
        self._smoothed_wz = 0.0
        self._integral_error = 0.0

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
