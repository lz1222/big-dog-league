from rk_mission.mission_state_machine_node import STAGES, STAGE_PLAN


def test_stage_plan_names_are_unique_and_ordered():
    names = [stage.name for stage in STAGE_PLAN]

    assert names == STAGES
    assert len(names) == len(set(names))
    assert names[0] == 'PRECHECK'
    assert names[-1] == 'DONE'


def test_action_stage_specs_have_commands():
    command_kinds = {
        'motion',
        'navigation',
        'arm',
        'arm_wait_item',
        'final_stop',
    }

    for stage in STAGE_PLAN:
        if stage.kind in command_kinds:
            assert stage.command, stage.name
