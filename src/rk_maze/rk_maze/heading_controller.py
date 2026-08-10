#!/usr/bin/env python3
"""轻量航向稳定器：墙线重对齐、IMU 阻尼与异常偏航保护。

该模块不再试图以单侧墙距长期“纠正左偏/右偏”。巡航只使用可信墙线的
航向误差；侧向居中仅能在转弯后的短暂重捕获阶段显式启用。IMU 只承担
yaw-rate 阻尼和异常偏航保护，最终轨迹仍由 planner 的碰撞检查决定。
"""

import math
from dataclasses import dataclass
from typing import Optional


@dataclass
class HeadingControllerConfig:
    """Tunable parameters for the heading fusion controller."""

    # PD gains
    kp_heading: float = 1.5        # proportional to heading error
    kp_center: float = 0.0         # 默认关闭：不以墙距差持续制造偏航
    kd_gyro: float = 0.3           # derivative: IMU gyro damping
    ki_heading: float = 0.0        # integral (disabled in v1)

    # Limits
    max_wz: float = 0.50           # rad/s — absolute cap on wz reference
    max_wz_rate: float = 0.30      # rad/s² — max change in wz per second
    deadband_deg: float = 1.5      # heading error deadband in degrees
    post_turn_kp_heading: float = 0.8
    post_turn_max_wz: float = 0.20
    enable_lateral_centering: bool = False
    max_abs_imu_wz: float = 1.50   # 超过正常转弯范围即拒绝给规划器候选

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
    abnormal_yaw_rate: bool = False


class HeadingController:
    """只输出受限航向参考；输入失真时 fail-closed，不生成补偿性转向。"""

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
        post_turn_realign: bool = False,
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
        # IMU 暂时不可用时不猜测角速度；全局安全仲裁仍会把 IMU stale 当作停止条件。
        if imu_stale or not math.isfinite(imu_wz):
            imu_wz = 0.0  # fallback: no gyro damping
        elif abs(imu_wz) > self._config.max_abs_imu_wz:
            # 异常 yaw-rate 可能来自跌倒、碰撞或坐标约定错误，绝不能继续叠加墙线补偿。
            state.reason = 'imu_yaw_rate_excess'
            state.abnormal_yaw_rate = True
            return state

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

        # ---- 侧向误差只服务于转弯后重对齐 ----
        lateral_error = 0.0
        if (post_turn_realign and self._config.enable_lateral_centering
                and math.isfinite(left_clearance) and math.isfinite(right_clearance)):
            lateral_error = (left_clearance - right_clearance) / 2.0

        # ---- Compute wz components ----
        heading_gain = (self._config.post_turn_kp_heading if post_turn_realign
                        else self._config.kp_heading)
        heading_component = heading_gain * heading_error
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
        max_wz = (min(self._config.max_wz, self._config.post_turn_max_wz)
                  if post_turn_realign else self._config.max_wz)
        raw_wz = max(-max_wz, min(max_wz, raw_wz))

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
