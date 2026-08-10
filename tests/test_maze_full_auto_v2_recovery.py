#!/usr/bin/env python3
"""maze_full_auto_v2 传感器短断流恢复策略的纯状态测试。"""

import importlib.util
from pathlib import Path


SCRIPT_PATH = Path(__file__).resolve().parents[1] / 'scripts' / 'maze_full_auto_v2.py'


def _load_module():
    """仅加载类型和静态策略，不实例化 ROS 节点或创建任何 publisher。"""
    spec = importlib.util.spec_from_file_location('maze_full_auto_v2_recovery', SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_short_cloud_outage_stops_motion_before_long_outage_latches_fault():
    """一秒无云即停止；较长阈值只决定是否禁止自动恢复。"""
    module = _load_module()
    config = module.MazeConfig()
    snapshot = module.SensorSnapshot(
        monotonic=1.0,
        cloud_valid=True,
        cloud_fresh=False,
        odom_valid=True,
        odom_fresh=True,
        imu_valid=True,
        imu_fresh=True,
    )

    assert config.cloud_stop_age == 1.0
    assert config.sensor_fault_latch_age == 6.0
    assert not snapshot.all_sensors_ok


def test_short_pre_turn_outage_reassesses_but_active_turn_remains_fail_closed():
    """预转只可重新决策；主转弯中断绝不能根据旧 yaw 自动续转。"""
    module = _load_module()
    state = module.MazeState
    resume_target = module.MazeAutoNode._resume_target_after_sensor_stale

    assert resume_target(state.SYSTEM_PRECHECK) == state.WAIT_FOR_SENSORS
    assert resume_target(state.WAIT_FOR_SENSORS) == state.WAIT_FOR_SENSORS
    assert resume_target(state.CRUISE) == state.CRUISE
    assert resume_target(state.CORNER_CANDIDATE) == state.CRUISE
    assert resume_target(state.TURN_APPROACH) == state.RECOVERY_DECISION
    assert resume_target(state.ARC_TURN_MAIN) is None
    assert resume_target(state.TURN_FINE_ALIGN) is None
    assert resume_target(state.CORRIDOR_REACQUIRE) is None
