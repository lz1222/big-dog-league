#!/usr/bin/env python3
"""F5: Dynamic rectangular footprint and swept-footprint collision checking."""
import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

VERDICT_UNSAFE = 'UNSAFE'; VERDICT_UNKNOWN = 'UNKNOWN'; VERDICT_GEOMETRY_SAFE_UNCALIBRATED = 'GEOMETRY_SAFE_UNCALIBRATED'
VERDICT_NOMINAL_SAFE = 'NOMINAL_SAFE'; VERDICT_ROBUST_SAFE = 'ROBUST_SAFE'; VERDICT_EXECUTABLE_SAFE = 'EXECUTABLE_SAFE'
VERDICT_ORDER = {VERDICT_UNSAFE:0, VERDICT_UNKNOWN:1, VERDICT_GEOMETRY_SAFE_UNCALIBRATED:2,
                 VERDICT_NOMINAL_SAFE:3, VERDICT_ROBUST_SAFE:4, VERDICT_EXECUTABLE_SAFE:5}

class Point3D:
    __slots__ = ('x','y','z')
    def __init__(self, x=0.0, y=0.0, z=0.0): self.x=x; self.y=y; self.z=z

@dataclass
class Pose2D: x: float = 0.0; y: float = 0.0; yaw: float = 0.0

@dataclass
class VelocityCandidate:
    name: str = ''; vx: float = 0.0; vy: float = 0.0; wz: float = 0.0
    duration_sec: float = 1.0; verdict: str = VERDICT_UNKNOWN
    minimum_clearance: float = float('inf'); minimum_clearance_time: float = 0.0
    dangerous_obstacle = None; dangerous_footprint_part: str = ''
    collision: bool = False; robust_safe: bool = False
    stop_tail_clearance: float = float('inf'); perturbation_margin: float = 0.0

@dataclass
class DynamicFootprint:
    footprint_front_m: float = 0.28; footprint_rear_m: float = 0.42
    footprint_left_m: float = 0.18; footprint_right_m: float = 0.18
    gait_sway_margin_m: float = 0.03; cloud_uncertainty_margin_m: float = 0.03
    odom_uncertainty_margin_m: float = 0.02; motion_model_margin_m: float = 0.02
    stop_tail_margin_m: float = 0.05
    def expanded_extents(self):
        m = self.gait_sway_margin_m+self.cloud_uncertainty_margin_m+self.odom_uncertainty_margin_m+self.motion_model_margin_m
        return {'front':self.footprint_front_m+m,'rear':self.footprint_rear_m+m,'left':self.footprint_left_m+m,'right':self.footprint_right_m+m}

@dataclass
class SweptFootprintConfig:
    max_yaw_step_deg: float = 1.0; max_corner_displacement_m: float = 0.01
    max_dt_sec: float = 0.05; stop_delay_sec: float = 0.20
    min_braking_deceleration: float = 0.80; stop_tail_duration_sec: float = 0.80
    perturbation_vx: float = 0.02; perturbation_wz: float = 0.03
    cloud_uncertainty_margin_m: float = 0.03

class SweptFootprintChecker:
    def __init__(self, footprint: DynamicFootprint, config: SweptFootprintConfig):
        self._fp = footprint; self._cfg = config

    def check(self, candidate: VelocityCandidate, obstacles, start_pose=None):
        if start_pose is None: start_pose = Pose2D()
        ext = self._fp.expanded_extents(); result = candidate
        result.minimum_clearance = float('inf'); result.collision = False
        samples = self._sample_traj(result.vx, result.vy, result.wz, result.duration_sec, start_pose)
        tail = self._sample_tail(result.vx, result.vy, result.wz, result.duration_sec, start_pose)
        for i, pose in enumerate(samples + tail):
            for label, cx, cy in self._corners(pose, ext):
                min_d = float('inf')
                for op in obstacles:
                    d = math.hypot(cx-op.x, cy-op.y)
                    if d < min_d: min_d = d
                if min_d < result.minimum_clearance:
                    result.minimum_clearance = min_d
                    result.dangerous_footprint_part = label
                if min_d <= 0.0: result.collision = True; result.dangerous_footprint_part = label; break
            if result.collision: break
        return self._assign_verdict(result)

    def check_with_perturbation(self, candidate, obstacles, start_pose=None):
        base = self.check(candidate, obstacles, start_pose)
        if not base.robust_safe: return base
        for pvx, pwz in [(candidate.vx+self._cfg.perturbation_vx, candidate.wz),
                          (candidate.vx-self._cfg.perturbation_vx, candidate.wz),
                          (candidate.vx, candidate.wz+self._cfg.perturbation_wz),
                          (candidate.vx, candidate.wz-self._cfg.perturbation_wz)]:
            p = VelocityCandidate(name=candidate.name+'_p', vx=pvx, vy=candidate.vy, wz=pwz, duration_sec=candidate.duration_sec)
            c = self.check(p, obstacles, start_pose)
            if c.collision: base.robust_safe = False; base.verdict = VERDICT_NOMINAL_SAFE
        return base

    def _sample_traj(self, vx, vy, wz, duration, start):
        if duration <= 0: return [start]
        dt = self._cfg.max_dt_sec
        if abs(wz) > 0: dt = min(dt, math.radians(self._cfg.max_yaw_step_deg)/abs(wz))
        if abs(vx) > 0 or abs(vy) > 0: dt = min(dt, self._cfg.max_corner_displacement_m/max(abs(vx), abs(vy), 0.001))
        dt = max(0.01, dt); n = max(1, int(duration/dt)+1)
        samples = []
        for i in range(n):
            t = i * dt
            if t > duration:
                t = duration
            if abs(wz) > 0.001:
                y = start.yaw + wz*t
                r = math.hypot(vx, vy)/abs(wz) if abs(wz) > 0.001 else float('inf')
                dx = r*(math.sin(y)-math.sin(start.yaw)) if r < 1e9 else vx*t
                dy = -r*(math.cos(y)-math.cos(start.yaw)) if r < 1e9 else vy*t
            else:
                y = start.yaw; dx = vx*t; dy = vy*t
            samples.append(Pose2D(x=start.x+dx, y=start.y+dy, yaw=y))
            if t >= duration: break
        return samples

    def _sample_tail(self, vx, vy, wz, duration, start):
        speed = math.hypot(vx, vy)
        if speed > 0:
            bd = speed*self._cfg.stop_delay_sec + speed*speed/(2*self._cfg.min_braking_deceleration)
            bt = speed/self._cfg.min_braking_deceleration + self._cfg.stop_delay_sec
        else: bd = 0.0; bt = 0.0
        tt = min(bt+0.1, self._cfg.stop_tail_duration_sec); n = max(1, int(tt/self._cfg.max_dt_sec))
        final = samples[-1] if (samples := self._sample_traj(vx, vy, wz, duration, Pose2D())) else start
        result = []
        for i in range(1, n+1):
            f = i/n; result.append(Pose2D(x=final.x+bd*f*math.cos(final.yaw), y=final.y+bd*f*math.sin(final.yaw), yaw=final.yaw+wz*0.2*f))
        return result

    def _corners(self, pose, ext):
        c = math.cos(pose.yaw); s = math.sin(pose.yaw)
        return [('FL', pose.x+ext['front']*c-ext['left']*s, pose.y+ext['front']*s+ext['left']*c),
                ('FR', pose.x+ext['front']*c+ext['right']*s, pose.y+ext['front']*s-ext['right']*c),
                ('RL', pose.x-ext['rear']*c-ext['left']*s, pose.y-ext['rear']*s+ext['left']*c),
                ('RR', pose.x-ext['rear']*c+ext['right']*s, pose.y-ext['rear']*s-ext['right']*c)]

    def _assign_verdict(self, c):
        if c.collision: c.verdict = VERDICT_UNSAFE; c.robust_safe = False; return c
        m = self._cfg.cloud_uncertainty_margin_m + 0.02
        if c.minimum_clearance >= 0.10: c.verdict = VERDICT_ROBUST_SAFE; c.robust_safe = True
        elif c.minimum_clearance >= m: c.verdict = VERDICT_NOMINAL_SAFE; c.robust_safe = False
        elif c.minimum_clearance > 0: c.verdict = VERDICT_GEOMETRY_SAFE_UNCALIBRATED
        else: c.verdict = VERDICT_UNSAFE
        return c
