"""正式人工经典确认合同的无硬件回归测试。"""

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
    """readiness 必须观测当前实例的人工经典签名验证结果。"""
    decoded = decode_status_datagram(_status('CLASSIC_VERIFIED', 2).encode())
    assert decoded['event'] == 'CLASSIC_VERIFIED'
    assert decoded['ret'] == 0


def test_manual_classic_confirmation_precedes_udp_bind_and_fails_closed():
    """缺少人工确认时，不得绑定 UDP Move socket。"""
    source = (PACKAGE_ROOT / 'src' / 'go2_sdk_udp_server.cpp').read_text(
        encoding='utf-8'
    )
    classic = source.index('config.manual_classic_confirmed')
    startup = source.index('SendStartupStopWithRetry(client, status)')
    udp_bind = source.index('CreateUdpSocket(config)')
    assert classic < startup < udp_bind
    assert 'MANUAL_CLASSIC_CONFIRMATION_REQUIRED' in source
    assert 'manual_classic_confirmation_missing' in source
    assert 'client.ClassicWalk(' not in source
    assert '--manual-classic-confirmed' in source
    assert 'error_code() ==' not in source


def test_move_loop_does_not_contain_gait_entry_calls():
    """20 Hz ExecuteDecision 不得包含任何步态入口或人工按键模拟。"""
    source = (PACKAGE_ROOT / 'src' / 'go2_sdk_udp_server.cpp').read_text(
        encoding='utf-8'
    )
    execute_decision = source.split('void ExecuteDecision(', 1)[1].split(
        'int CreateUdpSocket', 1
    )[0]
    assert 'ClassicWalk' not in execute_decision
    assert 'wirelesscontroller' not in execute_decision


def test_formal_start_records_current_instance_manual_classic_confirmation():
    """启动脚本在 UDP socket/ROS 图之前记录人工经典确认。"""
    start_source = (
        PACKAGE_ROOT.parent / 'rk_bringup' / 'scripts' /
        'start_non_arm_competition.sh'
    ).read_text(encoding='utf-8')
    classic_ack = start_source.index('--event CLASSIC_VERIFIED')
    startup_ack = start_source.index('--event STARTUP_STOP')
    listener = start_source.index('wait_for_udp_listener_count', startup_ack)
    assert classic_ack < startup_ack < listener
    assert '--manual-classic-confirmed "$MANUAL_CLASSIC_CONFIRMED"' in (
        start_source
    )
