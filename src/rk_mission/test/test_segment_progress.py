from rk_mission.mission_types import MotionCommand
from rk_mission.segment_progress import SegmentProgress


def test_progress_only_accumulates_authorized_finite_forward_motion():
    progress = SegmentProgress()
    accepted = progress.update(
        0.5, MotionCommand(0.2, 0.4), line_visible=True,
        line_confidence=0.9, min_line_confidence=0.7,
        min_effective_speed=0.1, estop=False, gait_lock=False,
        arm_lock=False, state_allows_forward=True, searching_in_place=False,
    )
    assert accepted
    assert progress.commanded_forward_distance == 0.1
    assert progress.absolute_turn_progress == 0.2

    assert not progress.update(
        0.5, MotionCommand(0.2, 0.4), line_visible=True,
        line_confidence=0.9, min_line_confidence=0.7,
        min_effective_speed=0.1, estop=False, gait_lock=True,
        arm_lock=False, state_allows_forward=True, searching_in_place=False,
    )
    assert progress.commanded_forward_distance == 0.1
