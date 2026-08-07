import inspect

from rk_mission.national_mission_node import NationalMissionNode


def test_national_node_keeps_single_final_velocity_owner_contract_in_source():
    source = inspect.getsource(NationalMissionNode)
    assert '/control/mission_cmd' in source
    assert '/navigation/cmd_vel' in source
