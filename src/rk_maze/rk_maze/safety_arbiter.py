#!/usr/bin/env python3
"""Safety arbiter — unified 12-level safety priority."""
from dataclasses import dataclass, field
from typing import Optional
from rk_maze.joint_health_guard import HEALTH_HARDWARE_FAULT, HEALTH_SENSOR_INVALID, HEALTH_COOLDOWN_REQUIRED
from rk_maze.lidar_distance_core import SPEED_EMERGENCY

@dataclass
class SafetyVerdict:
    can_move: bool = False; command_vx: float = 0.0; command_vy: float = 0.0
    command_wz: float = 0.0; active_override: str = ''; override_priority: int = 0
    reason: str = ''; diagnostic: dict = field(default_factory=dict)

class SafetyArbiter:
    PRIORITY_ESTOP = 1; PRIORITY_WATCHDOG = 2; PRIORITY_JOINT_HARDWARE_FAULT = 3
    PRIORITY_SENSOR_STALE = 4; PRIORITY_HARD_DISTANCE = 5
    PRIORITY_FOOTPRINT_COLLISION = 6; PRIORITY_NO_ROBUST_SAFE = 7
    PRIORITY_TRAJECTORY_DEVIATION = 8

    def __init__(self): self._estop: bool = False; self._watchdog_fault: bool = False

    def set_estop(self, active: bool): self._estop = active
    def set_watchdog_fault(self, fault: bool): self._watchdog_fault = fault

    def evaluate(self, joint_health_state, cloud_stale, odom_stale, imu_stale,
                 hard_front_distance, speed_class, selected_candidate,
                 actual_deviation=False, robot_moving=False):
        if self._estop:
            return SafetyVerdict(active_override='estop', override_priority=self.PRIORITY_ESTOP,
                                 reason='emergency_stop_active')
        if self._watchdog_fault:
            return SafetyVerdict(active_override='watchdog', override_priority=self.PRIORITY_WATCHDOG,
                                 reason='watchdog_fault')
        if joint_health_state in (HEALTH_HARDWARE_FAULT, HEALTH_SENSOR_INVALID):
            return SafetyVerdict(active_override='joint_health', override_priority=self.PRIORITY_JOINT_HARDWARE_FAULT,
                                 reason=f'joint_{joint_health_state}')
        if joint_health_state == HEALTH_COOLDOWN_REQUIRED:
            return SafetyVerdict(active_override='joint_cooldown', override_priority=self.PRIORITY_JOINT_HARDWARE_FAULT,
                                 reason='cooldown_required')
        stale = [n for n, s in [('Cloud', cloud_stale), ('Odom', odom_stale), ('IMU', imu_stale)] if s]
        if stale:
            return SafetyVerdict(active_override='sensor_stale', override_priority=self.PRIORITY_SENSOR_STALE,
                                 reason=f'stale_{",".join(stale)}')
        if speed_class == SPEED_EMERGENCY:
            return SafetyVerdict(active_override='hard_distance', override_priority=self.PRIORITY_HARD_DISTANCE,
                                 reason=f'front={hard_front_distance:.3f}m')
        if selected_candidate is not None and selected_candidate.collision:
            return SafetyVerdict(active_override='footprint', override_priority=self.PRIORITY_FOOTPRINT_COLLISION,
                                 reason='collision')
        if selected_candidate is None or not selected_candidate.robust_safe:
            return SafetyVerdict(active_override='no_robust', override_priority=self.PRIORITY_NO_ROBUST_SAFE,
                                 reason='no_robust_safe')
        if actual_deviation:
            return SafetyVerdict(active_override='deviation', override_priority=self.PRIORITY_TRAJECTORY_DEVIATION,
                                 reason='path_deviation')
        return SafetyVerdict(can_move=True, command_vx=selected_candidate.vx,
                             command_vy=selected_candidate.vy, command_wz=selected_candidate.wz,
                             reason='safe')
