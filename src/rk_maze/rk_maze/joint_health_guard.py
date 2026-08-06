#!/usr/bin/env python3
"""F9: JointHealthGuard — realtime motor health monitoring for rear hip joints.

Monitors RL_hip[9], RR_hip[6], RL_thigh[10], RR_thigh[7], RL_calf[11], RR_calf[8].
Health states: NORMAL, LIMITED, COOLDOWN_REQUIRED, HARDWARE_FAULT, SENSOR_INVALID.
All temperature thresholds UNVERIFIED — confirm with Unitree official.
"""
import math
from collections import deque
from dataclasses import dataclass, field
from typing import Optional

HEALTH_NORMAL = 'NORMAL'
HEALTH_LIMITED = 'LIMITED'
HEALTH_COOLDOWN_REQUIRED = 'COOLDOWN_REQUIRED'
HEALTH_HARDWARE_FAULT = 'HARDWARE_FAULT'
HEALTH_SENSOR_INVALID = 'SENSOR_INVALID'

@dataclass
class MotorState:
    index: int = -1; name: str = ''; q: float = 0.0; dq: float = 0.0
    tau_est: float = 0.0; temperature: float = 0.0; lost: int = 0; timestamp: float = 0.0

@dataclass
class JointHealthConfig:
    temp_warn_deg_c: float = 70.0; temp_critical_deg_c: float = 85.0
    temp_hardware_fault_deg_c: float = 105.0; temp_rise_rate_warn: float = 2.0
    torque_warn_nm: float = 5.0; torque_critical_nm: float = 8.0
    torque_consistency_samples: int = 5; max_lost_count: int = 3
    max_state_age_sec: float = 0.50; limited_max_wz: float = 0.20
    limited_max_vx: float = 0.15; limited_max_consecutive_left_turns: int = 2
    limited_max_left_turn_duration_sec: float = 3.0
    temperature_history_size: int = 20; torque_history_size: int = 20

@dataclass
class JointHealthStatus:
    state: str = HEALTH_NORMAL; rl_hip_temp: float = 0.0; rr_hip_temp: float = 0.0
    rl_hip_tau: float = 0.0; rr_hip_tau: float = 0.0
    rl_hip_temp_rise_rate: float = 0.0; rr_hip_temp_rise_rate: float = 0.0
    rl_hip_torque_consistent: bool = False; rr_hip_torque_consistent: bool = False
    any_lost: bool = False; any_stale: bool = False; reason: str = ''
    limited_restrictions: dict = field(default_factory=dict); valid: bool = False

class JointHealthGuard:
    MONITORED_JOINTS = {9:'RL_hip',6:'RR_hip',10:'RL_thigh',7:'RR_thigh',11:'RL_calf',8:'RR_calf'}
    _PRIMARY_JOINTS = {9, 6}

    def __init__(self, config: JointHealthConfig):
        self._config = config
        self._temp_history = {idx: deque(maxlen=config.temperature_history_size) for idx in self.MONITORED_JOINTS}
        self._torque_history = {idx: deque(maxlen=config.torque_history_size) for idx in self.MONITORED_JOINTS}
        self._last_update = 0.0; self._consecutive_left_turns = 0
        self._left_turn_duration = 0.0; self._locked = False

    @property
    def locked(self) -> bool: return self._locked

    def update(self, motor_states: dict, now_sec: float) -> JointHealthStatus:
        status = JointHealthStatus()
        for idx in self._PRIMARY_JOINTS:
            if idx not in motor_states:
                status.state = HEALTH_SENSOR_INVALID; status.reason = f'motor_state[{idx}] missing'; return status
        rl = motor_states.get(9); rr = motor_states.get(6)
        if rl is None or rr is None:
            status.state = HEALTH_SENSOR_INVALID; status.reason = 'primary_joint_null'; return status
        age = now_sec - rl.timestamp if rl.timestamp > 0 else float('inf')
        if age > self._config.max_state_age_sec:
            status.state = HEALTH_SENSOR_INVALID; status.reason = f'stale {age:.3f}s'; status.any_stale = True; return status
        if rl.lost > self._config.max_lost_count or rr.lost > self._config.max_lost_count:
            status.state = HEALTH_SENSOR_INVALID; status.reason = f'lost RL={rl.lost} RR={rr.lost}'; status.any_lost = True; return status
        for idx, motor in motor_states.items():
            if idx in self.MONITORED_JOINTS:
                self._temp_history[idx].append((now_sec, motor.temperature))
                self._torque_history[idx].append((now_sec, motor.tau_est))
        self._last_update = now_sec
        status.rl_hip_temp = rl.temperature; status.rr_hip_temp = rr.temperature
        status.rl_hip_tau = rl.tau_est; status.rr_hip_tau = rr.tau_est; status.valid = True
        status.rl_hip_temp_rise_rate = self._compute_rise_rate(9, now_sec)
        status.rr_hip_temp_rise_rate = self._compute_rise_rate(6, now_sec)
        if rl.temperature >= self._config.temp_hardware_fault_deg_c:
            self._locked = True; status.state = HEALTH_HARDWARE_FAULT
            status.reason = f'RL temp {rl.temperature:.0f}C'; return status
        if rr.temperature >= self._config.temp_hardware_fault_deg_c:
            self._locked = True; status.state = HEALTH_HARDWARE_FAULT
            status.reason = f'RR temp {rr.temperature:.0f}C'; return status
        if rl.temperature >= self._config.temp_critical_deg_c:
            status.state = HEALTH_COOLDOWN_REQUIRED; return status
        if rr.temperature >= self._config.temp_critical_deg_c:
            status.state = HEALTH_COOLDOWN_REQUIRED; return status
        if abs(rl.tau_est) >= self._config.torque_critical_nm:
            status.state = HEALTH_COOLDOWN_REQUIRED; return status
        if abs(rr.tau_est) >= self._config.torque_critical_nm:
            status.state = HEALTH_COOLDOWN_REQUIRED; return status
        if rl.temperature >= self._config.temp_warn_deg_c or rr.temperature >= self._config.temp_warn_deg_c:
            status.state = HEALTH_LIMITED
            status.limited_restrictions = {'max_wz':self._config.limited_max_wz,'max_vx':self._config.limited_max_vx}
            return status
        if abs(rl.tau_est) >= self._config.torque_warn_nm or abs(rr.tau_est) >= self._config.torque_warn_nm:
            status.state = HEALTH_LIMITED
            status.limited_restrictions = {'max_wz':self._config.limited_max_wz,'max_vx':self._config.limited_max_vx}
            return status
        status.state = HEALTH_NORMAL; status.reason = 'all_normal'; return status

    def _compute_rise_rate(self, idx, now_sec):
        history = list(self._temp_history.get(idx, []))
        if len(history) < 2: return 0.0
        n = len(history); s_t = s_v = s_tt = s_tv = 0.0
        for t, v in history: s_t += t; s_v += v; s_tt += t*t; s_tv += t*v
        denom = n*s_tt - s_t*s_t
        return (n*s_tv - s_t*s_v)/denom if abs(denom) > 1e-9 else 0.0

    def reset(self):
        self._temp_history = {idx: deque(maxlen=self._config.temperature_history_size) for idx in self.MONITORED_JOINTS}
        self._torque_history = {idx: deque(maxlen=self._config.torque_history_size) for idx in self.MONITORED_JOINTS}
        self._consecutive_left_turns = 0; self._left_turn_duration = 0.0; self._locked = False
