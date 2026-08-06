#!/usr/bin/env python3
"""F6: Short-term velocity candidate planning for maze corridors."""
import math
from dataclasses import dataclass, field
from typing import List, Optional
from rk_maze.swept_footprint_checker import (DynamicFootprint, SweptFootprintChecker, SweptFootprintConfig,
    VelocityCandidate, Pose2D, VERDICT_ROBUST_SAFE, VERDICT_UNSAFE, VERDICT_UNKNOWN)

STATE_IDLE = 'IDLE'; STATE_CORRIDOR_CRUISE = 'CORRIDOR_CRUISE'; STATE_TURN_APPROACH = 'TURN_APPROACH'
STATE_ARC_TURN_ENTRY = 'ARC_TURN_ENTRY'; STATE_ARC_TURN_MAIN = 'ARC_TURN_MAIN'
STATE_TURN_FINE_ALIGN = 'TURN_FINE_ALIGN'; STATE_CORRIDOR_REACQUIRE = 'CORRIDOR_REACQUIRE'
STATE_PRE_STOP_COMPENSATE = 'PRE_STOP_COMPENSATE'; STATE_STOP_AND_SETTLE = 'STOP_AND_SETTLE'
STATE_FAULT_STOP = 'FAULT_STOP'; STATE_THERMAL_COOLDOWN = 'THERMAL_COOLDOWN'

@dataclass
class PlannerConfig:
    corridor_cruise_vx: float = 0.25; corridor_slow_vx: float = 0.15
    delta_wz: float = 0.05; arc_vx_small: float = 0.10; arc_vx_medium: float = 0.18
    arc_wz_small: float = 0.30; arc_wz_medium: float = 0.50; arc_vy_outward: float = 0.05
    decelerate_vx: float = 0.12; fine_align_vx: float = 0.08; fine_align_wz: float = 0.15
    reverse_short_vx: float = -0.10; reverse_short_duration: float = 0.5
    turn_decision_distance_m: float = 0.70; corner_approach_distance_m: float = 1.00
    compensation_ratio: float = 0.30; allow_in_place_rotation: bool = False

@dataclass
class PlannerOutput:
    selected_candidate: Optional[VelocityCandidate] = None
    all_candidates: List[VelocityCandidate] = field(default_factory=list)
    wz_reference: float = 0.0; state: str = STATE_IDLE
    reason: str = ''; stop_required: bool = False; stop_reason: str = ''

class MazeLocalPlanner:
    def __init__(self, config: PlannerConfig, footprint: DynamicFootprint, checker_config: SweptFootprintConfig):
        self._cfg = config; self._fp = footprint; self._checker = SweptFootprintChecker(footprint, checker_config)

    def plan(self, heading, obstacles, state, left_cl, right_cl, front_cl, rear_ok, joint_health,
             stop_bias_avail=False, stop_bias_rad=0.0, start_pose=None):
        output = PlannerOutput(wz_reference=heading.wz_reference if heading.valid else 0.0, state=state)
        if joint_health.state in ('HARDWARE_FAULT', 'SENSOR_INVALID'):
            output.stop_required = True; output.stop_reason = f'joint_{joint_health.state}'; output.state = STATE_FAULT_STOP; return output
        if joint_health.state == 'COOLDOWN_REQUIRED':
            output.stop_required = True; output.stop_reason = 'cooldown'; output.state = STATE_THERMAL_COOLDOWN; return output
        if not heading.valid or heading.stale:
            output.stop_required = True; output.stop_reason = 'heading_stale'; return output
        cands = self._gen(heading, state, joint_health)
        checked = [self._checker.check_with_perturbation(c, obstacles, start_pose) for c in cands]
        output.all_candidates = checked
        robust = [c for c in checked if c.robust_safe]
        if robust:
            robust.sort(key=lambda c: (-c.minimum_clearance, -c.vx))
            output.selected_candidate = robust[0]; output.reason = f'selected_{robust[0].name}'; return output
        output.stop_required = True; uns = sum(1 for c in checked if c.verdict == VERDICT_UNSAFE)
        output.stop_reason = f'no_robust_{uns}_unsafe'; return output

    def _gen(self, heading, state, jh):
        wz = heading.wz_reference; dw = self._cfg.delta_wz
        mwz = jh.limited_restrictions.get('max_wz', self._cfg.arc_wz_medium) if jh.state == 'LIMITED' else self._cfg.arc_wz_medium
        mvx = jh.limited_restrictions.get('max_vx', self._cfg.corridor_cruise_vx) if jh.state == 'LIMITED' else self._cfg.corridor_cruise_vx
        cands = []
        cvx = min(self._cfg.corridor_cruise_vx, mvx); svx = min(self._cfg.corridor_slow_vx, mvx)
        for n, vx, w in [('STOP',0.0,0.0), ('FORWARD_CRUISE',cvx,wz), ('FORWARD_SLOW',svx,wz),
                          ('CORRECT_LEFT',cvx,wz+dw), ('CORRECT_RIGHT',cvx,wz-dw)]:
            if abs(w) <= mwz: cands.append(VelocityCandidate(name=n, vx=vx, vy=0.0, wz=w, duration_sec=1.0))
        return cands
