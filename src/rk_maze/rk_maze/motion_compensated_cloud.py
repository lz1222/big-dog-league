#!/usr/bin/env python3
"""F4a: Odom-compensated multi-frame point cloud buffer."""
import math
from collections import deque
from dataclasses import dataclass, field
from typing import List, Optional

@dataclass
class OdomPose:
    stamp_sec: float = 0.0; x: float = 0.0; y: float = 0.0; yaw: float = 0.0
    vx: float = 0.0; vy: float = 0.0; wz: float = 0.0

@dataclass
class TimestampedCloud:
    points: list = field(default_factory=list); stamp_sec: float = 0.0
    pose_x: float = 0.0; pose_y: float = 0.0; pose_yaw: float = 0.0

@dataclass
class MotionCompensatedCloudConfig:
    accumulation_window_sec: float = 0.30; max_buffer_frames: int = 10
    odom_max_jump_m: float = 0.30; odom_max_jump_yaw_rad: float = 0.30
    odom_max_age_sec: float = 0.10; odom_max_stale_age_sec: float = 0.10
    max_accumulated_displacement_m: float = 0.50; max_accumulated_yaw_rad: float = 0.50

class MotionCompensatedCloudBuffer:
    def __init__(self, config: MotionCompensatedCloudConfig):
        self._cfg = config; self._buffer: deque = deque()
        self._ref_pose: Optional[OdomPose] = None; self._ref_stamp: float = 0.0
        self._last_odom: Optional[OdomPose] = None
        self._acc_dist: float = 0.0; self._acc_yaw: float = 0.0
        self._cleared_count: int = 0; self._last_clear_reason: str = 'init'

    @property
    def accumulated_frames(self): return len(self._buffer)
    @property
    def cleared_count(self): return self._cleared_count
    @property
    def last_clear_reason(self): return self._last_clear_reason

    def update_odom(self, pose: OdomPose):
        if self._last_odom:
            dt = pose.stamp_sec - self._last_odom.stamp_sec
            if dt <= 0: self._clear('odom_timestamp_regression'); return
            d = math.hypot(pose.x-self._last_odom.x, pose.y-self._last_odom.y)
            dy = self._norm(pose.yaw - self._last_odom.yaw)
            if dt > 0 and d/dt > self._cfg.odom_max_jump_m/0.05: self._clear('odom_jump'); return
            if dt > 0 and abs(dy)/dt > self._cfg.odom_max_jump_yaw_rad/0.05: self._clear('yaw_jump'); return
            self._acc_dist += d; self._acc_yaw += abs(dy)
            if self._acc_dist > self._cfg.max_accumulated_displacement_m: self._clear('acc_dist'); return
            if self._acc_yaw > self._cfg.max_accumulated_yaw_rad: self._clear('acc_yaw'); return
        self._last_odom = pose

    def add_cloud(self, points, stamp, pose):
        if self._ref_pose is None: self._ref_pose = pose; self._ref_stamp = stamp
        if self._buffer and stamp < self._buffer[-1].stamp_sec: self._clear('ts_regression'); return
        cutoff = stamp - self._cfg.accumulation_window_sec
        while self._buffer and self._buffer[0].stamp_sec < cutoff:
            old = self._buffer.popleft()
            if self._buffer: self._ref_pose = OdomPose(stamp_sec=self._buffer[0].stamp_sec, x=self._buffer[0].pose_x, y=self._buffer[0].pose_y, yaw=self._buffer[0].pose_yaw); self._ref_stamp = self._buffer[0].stamp_sec
        self._buffer.append(TimestampedCloud(points=list(points), stamp_sec=stamp, pose_x=pose.x, pose_y=pose.y, pose_yaw=pose.yaw))
        if len(self._buffer) == 1: self._ref_pose = pose; self._ref_stamp = stamp

    def get_compensated_cloud(self):
        if not self._buffer or self._ref_pose is None: return []
        ref = self._ref_pose; all_pts = []
        for c in self._buffer:
            if abs(c.stamp_sec - self._ref_stamp) < 0.001: all_pts.extend(c.points); continue
            dx = ref.x - c.pose_x; dy = ref.y - c.pose_y; dya = self._norm(ref.yaw - c.pose_yaw)
            cos_y = math.cos(-dya); sin_y = math.sin(-dya)
            for p in c.points:
                all_pts.append(type(p)(x=p.x*cos_y - p.y*sin_y + dx, y=p.x*sin_y + p.y*cos_y + dy, z=p.z))
        return all_pts

    def is_odom_fresh(self, now):
        if self._last_odom is None: return False
        return (now - self._last_odom.stamp_sec) <= self._cfg.odom_max_age_sec

    def _clear(self, reason): self._buffer.clear(); self._ref_pose = None; self._ref_stamp = 0.0; self._acc_dist = 0.0; self._acc_yaw = 0.0; self._cleared_count += 1; self._last_clear_reason = reason

    @staticmethod
    def _norm(a):
        while a > math.pi: a -= 2*math.pi
        while a < -math.pi: a += 2*math.pi
        return a
