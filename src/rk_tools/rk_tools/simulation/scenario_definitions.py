"""Named nominal and fault routes for national integrated mission v1."""

from dataclasses import dataclass
from typing import Dict, Tuple


@dataclass(frozen=True)
class ScenarioDefinition:
    name: str
    marker_id: int = 1
    inspection_type: str = 'electric_shock'
    fault: str = ''
    expected_terminal: str = 'MISSION_COMPLETE'


NOMINAL_SCENARIOS: Tuple[ScenarioDefinition, ...] = tuple(
    ScenarioDefinition(
        'marker{}_{}'.format(marker, inspection), marker, inspection
    )
    for marker in (1, 2)
    for inspection in ('electric_shock', 'oxidizer', 'radiation')
)


_FAULTS = (
    ('start_white_single_noise', 'SAFE_STOP'),
    ('start_white_duplicate', 'MISSION_COMPLETE'),
    ('start_jump_failure', 'SAFE_STOP'),
    ('jump_line_reacquire_timeout', 'SAFE_STOP'),
    ('maze_failure', 'SAFE_STOP'),
    ('maze_timeout', 'SAFE_STOP'),
    ('stairs_failure', 'SAFE_STOP'),
    ('pick_arc_not_reached', 'SAFE_STOP'),
    ('pick_pattern_not_found', 'SAFE_STOP'),
    ('marker_alternates', 'SAFE_STOP'),
    ('marker_repeat_after_latch', 'MISSION_COMPLETE'),
    ('arm_pick_failure', 'SAFE_STOP'),
    ('transfer_place_failure', 'SAFE_STOP'),
    ('transfer_pick_failure', 'SAFE_STOP'),
    ('red_single_noise', 'SAFE_STOP'),
    ('red_early_ignored', 'MISSION_COMPLETE'),
    ('red_post_offset_timeout', 'SAFE_STOP'),
    ('inspection_sign_missing', 'SAFE_STOP'),
    ('inspection_sign_alternates', 'SAFE_STOP'),
    ('inspection_action_failure', 'SAFE_STOP'),
    ('place_target_missing', 'SAFE_STOP'),
    ('place_action_failure', 'SAFE_STOP'),
    ('finish_jump_failure', 'SAFE_STOP'),
    ('mid_route_estop', 'ESTOP'),
    ('final_cmd_stale', 'SAFE_STOP'),
    ('action_duplicate_result', 'MISSION_COMPLETE'),
    ('second_context_reset', 'MISSION_COMPLETE'),
    ('state_timeout', 'SAFE_STOP'),
    ('nonfinite_mission_candidate', 'SAFE_STOP'),
    ('action_and_json_repeat_start', 'MISSION_COMPLETE'),
)

FAULT_SCENARIOS: Tuple[ScenarioDefinition, ...] = tuple(
    ScenarioDefinition(name, 1, 'radiation', name, expected)
    for name, expected in _FAULTS
)

ALL_SCENARIOS: Dict[str, ScenarioDefinition] = {
    scenario.name: scenario
    for scenario in NOMINAL_SCENARIOS + FAULT_SCENARIOS
}


def get_scenario(name: str) -> ScenarioDefinition:
    try:
        return ALL_SCENARIOS[str(name)]
    except KeyError as exc:
        raise ValueError('unknown national scenario: {}'.format(name)) from exc
