#!/usr/bin/env python3
"""验证 V2 仅把 IMU 用于转弯/阻尼/停稳和姿态异常保护。"""

import ast
from pathlib import Path


SOURCE = Path(__file__).resolve().parents[1] / 'scripts' / 'maze_full_auto_v2.py'


def test_imu_snapshot_carries_yaw_rate_and_attitude_for_safety_only():
    """roll/pitch 必须进入不可变快照，避免跨线程读取半更新姿态。"""
    text = SOURCE.read_text(encoding='utf-8')
    ast.parse(text)
    assert 'imu_roll: float = 0.0' in text
    assert 'imu_pitch: float = 0.0' in text
    assert 'self._imu_roll_baseline' in text
    assert 'imu_roll_delta' in text
    assert 'def _imu_safety_fault' in text


def test_turn_checks_direction_overshoot_and_stationary_with_imu():
    """IMU 不承担持续偏置，而负责方向、过冲和停稳三个安全判定。"""
    text = SOURCE.read_text(encoding='utf-8')
    assert 'turn_imu_min_wz' in text
    assert 'turn_overshoot_deg' in text
    assert 'abs(snap.imu_wz) <= self.cfg.imu_settle_wz' in text
    assert 'WRONG_TURN_DIRECTION_IMU' in text
    assert 'TURN_OVERSHOOT' in text
