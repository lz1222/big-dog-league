#!/usr/bin/env python3
"""F8: Stop deviation statistics and adaptive feedforward."""
import math
from collections import deque
from dataclasses import dataclass, field
from typing import Optional, Tuple

@dataclass
class StopBiasConfig:
    min_samples: int = 10; max_history: int = 50
    direction_stability_threshold: float = 0.7; max_std_dev_deg: float = 3.0
    compensation_ratio: float = 0.30; max_compensation_deg: float = 5.0
    final_error_ok_deg: float = 1.5; final_error_correct_next_deg: float = 3.0
    final_error_one_arc_deg: float = 5.0

@dataclass
class StopRecord:
    yaw_before_stop: float = 0.0; yaw_at_stop_request: float = 0.0
    yaw_after_final_settle: float = 0.0; stop_yaw_shift: float = 0.0
    settling_time_sec: float = 0.0; rl_hip_temperature: float = 0.0
    rr_hip_temperature: float = 0.0; rl_hip_tau: float = 0.0; rr_hip_tau: float = 0.0
    gait_type: str = ''; speed_m_s: float = 0.0; action_type: str = ''

@dataclass
class StopBiasEstimate:
    bias_rad: float = 0.0; bias_deg: float = 0.0; sample_count: int = 0
    std_dev_deg: float = 0.0; direction_consistency: float = 0.0
    enabled: bool = False; reason: str = 'insufficient_samples'

class StopBiasEstimator:
    def __init__(self, config: StopBiasConfig):
        self._config = config
        self._records: deque = deque(maxlen=config.max_history)
        self._estimate: Optional[StopBiasEstimate] = None

    def record_stop(self, yaw_before_stop, yaw_at_stop_request, yaw_after_final_settle,
                    settling_time_sec, rl_hip_temp, rr_hip_temp, rl_hip_tau, rr_hip_tau,
                    gait_type='', speed_m_s=0.0, action_type=''):
        stop_yaw_shift = yaw_after_final_settle - yaw_before_stop
        record = StopRecord(yaw_before_stop=yaw_before_stop, yaw_at_stop_request=yaw_at_stop_request,
                            yaw_after_final_settle=yaw_after_final_settle, stop_yaw_shift=stop_yaw_shift,
                            settling_time_sec=settling_time_sec, rl_hip_temperature=rl_hip_temp,
                            rr_hip_temperature=rr_hip_temp, rl_hip_tau=rl_hip_tau, rr_hip_tau=rr_hip_tau,
                            gait_type=gait_type, speed_m_s=speed_m_s, action_type=action_type)
        self._records.append(record)
        self._update_estimate()

    def get_estimate(self) -> Optional[StopBiasEstimate]: return self._estimate

    def get_compensation_angle(self, target_yaw: float) -> Tuple[float, bool]:
        if self._estimate is None or not self._estimate.enabled: return target_yaw, False
        compensation = self._config.compensation_ratio * self._estimate.bias_rad
        max_comp = math.radians(self._config.max_compensation_deg)
        compensation = max(-max_comp, min(max_comp, compensation))
        return target_yaw - compensation, True

    def classify_final_error(self, final_error_rad: float) -> Tuple[str, str]:
        error_deg = abs(math.degrees(final_error_rad))
        if error_deg <= self._config.final_error_ok_deg: return 'complete', f'{error_deg:.1f}deg_ok'
        elif error_deg <= self._config.final_error_correct_next_deg: return 'correct_next_move', f'{error_deg:.1f}deg'
        elif error_deg <= self._config.final_error_one_arc_deg: return 'one_arc_correction', f'{error_deg:.1f}deg'
        else: return 'fault_stop', f'{error_deg:.1f}deg_fault'

    @property
    def sample_count(self) -> int: return len(self._records)

    def _update_estimate(self):
        n = len(self._records)
        if n < self._config.min_samples:
            self._estimate = StopBiasEstimate(sample_count=n, reason=f'need_{self._config.min_samples-n}_more'); return
        shifts = [r.stop_yaw_shift for r in self._records]
        ss = sorted(shifts); mid = n // 2
        med = ss[mid] if n % 2 else (ss[mid-1] + ss[mid]) / 2
        mean_s = sum(shifts) / n; variance = sum((s-mean_s)**2 for s in shifts) / n
        std_dev = math.sqrt(variance)
        dominant = 1 if med >= 0 else -1
        consistent = sum(1 for s in shifts if (s>=0)==(dominant>=0))
        enabled = (math.degrees(std_dev) <= self._config.max_std_dev_deg and
                   consistent/n >= self._config.direction_stability_threshold)
        self._estimate = StopBiasEstimate(bias_rad=med, bias_deg=math.degrees(med),
                                          sample_count=n, std_dev_deg=math.degrees(std_dev),
                                          direction_consistency=consistent/n,
                                          enabled=enabled, reason='ok' if enabled else 'unstable')

    def reset(self): self._records.clear(); self._estimate = None
