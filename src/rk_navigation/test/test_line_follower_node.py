from rk_navigation.line_follower_node import (
    LINE_FOLLOW,
    STOP,
    VALID_STATES,
    WAIT_START,
    LineFollowerNode,
)


def test_line_follower_states_are_declared():
    assert WAIT_START in VALID_STATES
    assert LINE_FOLLOW in VALID_STATES
    assert STOP in VALID_STATES


def test_direction_from_value_uses_sign():
    assert LineFollowerNode.direction_from_value(1.0) == 1
    assert LineFollowerNode.direction_from_value(0.0) == 1
    assert LineFollowerNode.direction_from_value(-1.0) == -1
