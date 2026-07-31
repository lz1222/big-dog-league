"""START_READY 稳定门的离线回归测试。"""

from rk_navigation.start_ready_core import StartReadyGate


def test_start_ready_requires_configured_consecutive_valid_frames():
    gate = StartReadyGate(confirm_frames=3)

    first = gate.observe(True, 0.80, 0.10, -0.10)
    second = gate.observe(True, 0.80, 0.10, -0.10)
    third = gate.observe(True, 0.80, 0.10, -0.10)

    assert not first.ready
    assert first.confirm_count == 1
    assert not second.ready
    assert second.confirm_count == 2
    assert third.ready
    assert third.reason == 'start_ready_confirmed'


def test_start_ready_rejects_low_confidence_and_resets_streak():
    gate = StartReadyGate(confirm_frames=2, min_confidence=0.70)

    first = gate.observe(True, 0.80, 0.0, 0.0)
    rejected = gate.observe(True, 0.69, 0.0, 0.0)
    recovered = gate.observe(True, 0.80, 0.0, 0.0)

    assert first.confirm_count == 1
    assert not rejected.ready
    assert rejected.confirm_count == 0
    assert rejected.reason == 'start_ready_confidence_low'
    assert recovered.confirm_count == 1


def test_start_ready_rejects_nonfinite_and_out_of_bounds_errors():
    gate = StartReadyGate(confirm_frames=1, max_lateral_error=0.2)

    nonfinite = gate.observe(True, float('nan'), 0.0, 0.0)
    out_of_bounds = gate.observe(True, 1.0, 0.21, 0.0)

    assert not nonfinite.ready
    assert nonfinite.reason == 'start_ready_non_finite_line_track'
    assert not out_of_bounds.ready
    assert out_of_bounds.reason == 'start_ready_lateral_error_large'


def test_start_ready_reset_prevents_old_task_frames_from_unlocking():
    gate = StartReadyGate(confirm_frames=2)

    gate.observe(True, 1.0, 0.0, 0.0)
    gate.reset()
    decision = gate.observe(True, 1.0, 0.0, 0.0)

    assert not decision.ready
    assert decision.confirm_count == 1
