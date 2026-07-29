from rk_mission.position_gates import (
    ConsecutiveDetectionGate,
    CurvedApexGate,
    RedMarkerBodyOffsetGate,
    StableLineGate,
    TimedDistanceGate,
)
from rk_mission.segment_progress import SegmentProgress


def test_timed_distance_gate_requires_minimum_and_target_conditions():
    progress = SegmentProgress(commanded_forward_distance=0.20,
                               effective_forward_time=0.30)
    gate = TimedDistanceGate(0.10, 0.18, 0.40, 0.20, 0.25, 1.0)
    assert gate.check(progress, 0.50).reached
    assert gate.check(progress, 1.00).exceeded


def test_curved_apex_locks_direction_and_rejects_reverse_turn():
    progress = SegmentProgress(commanded_forward_distance=0.25,
                               signed_turn_progress=0.35)
    gate = CurvedApexGate(0.10, 0.20, 0.40, 0.10, 0.05, 0.20)
    gate.update(0.10, 0.50)
    gate.update(0.10, -0.50)
    assert gate.locked_turn_sign == 1
    assert gate.reverse_duration == 0.10
    assert gate.check(progress, 0.40).reached


def test_red_offset_and_consecutive_gates_reset_on_noise():
    progress = SegmentProgress(commanded_forward_distance=0.12,
                               effective_forward_time=0.10)
    red_gate = RedMarkerBodyOffsetGate(0.10, 0.50, 1.0)
    assert not red_gate.check(progress, 0.10).reached
    red_gate.anchor()
    assert red_gate.check(progress, 0.10).reached

    line_gate = StableLineGate(2, 0.7)
    assert not line_gate.update(True, 0.8)
    assert not line_gate.update(False, 0.8)
    assert not line_gate.update(True, 0.8)
    assert line_gate.update(True, 0.8)

    detection_gate = ConsecutiveDetectionGate[int](2, 0.7)
    assert not detection_gate.update(1, 0.9)
    assert not detection_gate.update(2, 0.9)
    assert detection_gate.update(2, 0.9)
