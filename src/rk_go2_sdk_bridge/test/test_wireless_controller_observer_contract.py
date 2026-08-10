"""遥控器按键被动取证工具的无硬件安全回归测试。"""

from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_wireless_observer_is_subscriber_only():
    """遥控器取证不得携带任何可向机器人写入的控制路径。"""
    source = (
        PACKAGE_ROOT / 'src' / 'go2_sdk_wireless_controller_observer.cpp'
    ).read_text(encoding='utf-8')
    assert 'rt/wirelesscontroller' in source
    assert 'ChannelSubscriber' in source
    assert '#include <unitree/robot/channel/channel_publisher.hpp>' not in source
    assert '#include <unitree/robot/go2/sport/sport_client.hpp>' not in source
    assert '.Move(' not in source
    assert '.StopMove(' not in source


def test_wireless_observer_preserves_raw_key_evidence():
    """按键位图必须原样保留，不允许把组合键猜测为某种步态。"""
    source = (
        PACKAGE_ROOT / 'src' / 'go2_sdk_wireless_controller_observer.cpp'
    ).read_text(encoding='utf-8')
    for field in ('keys_dec=', 'keys_hex=', 'pressed=[', 'lx=', 'ly=', 'rx=', 'ry='):
        assert field in source
    assert '不推断步态语义' in source
