"""正式全局经典步态 owner 合同的无硬件回归测试。"""

import sys
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT / 'scripts'))

from sdk_motion_status import decode_status_datagram  # noqa: E402


def _status(event, sequence=1, ret=0):
    return (
        '{"server_instance_id":"current","sequence":%d,'
        '"event":"%s","ret":%d,"reason":"test",'
        '"vx":0,"vy":0,"yaw":0,"server_monotonic_ns":1}'
    ) % (sequence, event, ret)


def test_classic_verified_status_event_is_protocol_valid():
    """readiness 必须观测当前实例的经典步态 command ACK。"""
    decoded = decode_status_datagram(_status('CLASSIC_VERIFIED', 2).encode())
    assert decoded['event'] == 'CLASSIC_VERIFIED'
    assert decoded['ret'] == 0


def test_single_owner_action_status_events_are_protocol_valid():
    """动作代理的事件必须能穿过 forwarder，供现场审计单写者执行结果。"""
    for event in ('ACTION_REQUESTED', 'ACTION_READY', 'ACTION_FAILED'):
        decoded = decode_status_datagram(_status(event, 3).encode())
        assert decoded is not None
        assert decoded['event'] == event


def test_startup_classic_ack_precedes_udp_bind_and_fails_closed():
    """SDK 经典步态序列成功前，不得绑定 UDP Move socket。"""
    source = (PACKAGE_ROOT / 'src' / 'go2_sdk_udp_server.cpp').read_text(
        encoding='utf-8'
    )
    startup = source.index('SendStartupStopWithRetry(client, status)')
    classic = source.index('ApplyGaitRequest(', startup)
    udp_bind = source.index('CreateUdpSocket(config)')
    assert startup < classic < udp_bind
    assert 'client.ClassicWalk(true)' in source
    assert 'client.FreeWalk()' in source
    assert 'verification_source=command_ack' in source
    assert 'SelectMode(' not in source
    assert 'ReleaseMode(' not in source
    assert 'error_code() ==' not in source


def test_move_loop_does_not_contain_gait_entry_calls():
    """20 Hz ExecuteDecision 不得包含任何步态入口或人工按键模拟。"""
    source = (PACKAGE_ROOT / 'src' / 'go2_sdk_udp_server.cpp').read_text(
        encoding='utf-8'
    )
    execute_decision = source.rsplit('void ExecuteDecision(', 1)[1].split(
        'int CreateUdpSocket', 1
    )[0]
    assert 'ClassicWalk' not in execute_decision
    assert 'wirelesscontroller' not in execute_decision


def test_motion_helper_delegates_all_sdk_writes_to_server():
    """正式 helper 只能收发有 request_id 的 UDP ACK，不能创建 SDK client。"""
    helper = (PACKAGE_ROOT / 'src' / 'go2_sdk_motion_action.cpp').read_text(
        encoding='utf-8'
    )
    for forbidden in (
        '#include <unitree/', 'ClassicWalk(', 'FreeWalk(', 'FrontJump()',
        'StandUp()', 'StopMove()',
    ):
        assert forbidden not in helper
    assert 'GAIT_ACK ' in helper
    assert 'ACTION_ACK ' in helper
    assert 'RK_GO2_SDK_UDP_HOST' in helper
    assert 'RK_GO2_SDK_UDP_PORT' in helper

    server = (PACKAGE_ROOT / 'src' / 'go2_sdk_udp_server.cpp').read_text(
        encoding='utf-8'
    )
    assert 'ParseActionRequest' in server
    assert 'ApplyActionRequest' in server
    assert 'ACTION_ACK' in server


def test_formal_start_waits_for_current_instance_classic_ack():
    """启动脚本在 ROS 图之前等待 server 的经典步态 ACK。"""
    start_source = (
        PACKAGE_ROOT.parent / 'rk_bringup' / 'scripts' /
        'start_non_arm_competition.sh'
    ).read_text(encoding='utf-8')
    classic_ack = start_source.index('--event CLASSIC_VERIFIED')
    startup_ack = start_source.index('--event STARTUP_STOP')
    listener = start_source.index('wait_for_udp_listener_count', startup_ack)
    assert classic_ack < startup_ack < listener
    assert 'global_gait_owner.py' in (
        PACKAGE_ROOT.parent / 'rk_bringup' / 'launch' /
        'competition_non_arm.launch.py'
    ).read_text(encoding='utf-8')
    launch_source = (
        PACKAGE_ROOT.parent / 'rk_bringup' / 'launch' /
        'competition_non_arm.launch.py'
    ).read_text(encoding='utf-8')
    assert launch_source.count("'RK_GO2_SDK_UDP_HOST': sdk_udp_host") == 2
    assert launch_source.count("'RK_GO2_SDK_UDP_PORT': sdk_udp_port") == 2
