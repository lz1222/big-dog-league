"""SportModeCmd 被动观察器的无硬件安全回归测试。"""

from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_sport_mode_command_observer_is_subscriber_only():
    """人工入口取证工具不得同时拥有任何控制面写入能力。"""
    source = (
        PACKAGE_ROOT / 'src' / 'go2_sdk_sport_mode_command_observer.cpp'
    ).read_text(encoding='utf-8')
    assert 'rt/sportmodecmd' in source
    assert 'ChannelSubscriber' in source
    assert '#include <unitree/robot/go2/sport/sport_client.hpp>' not in source
    assert '#include <unitree/robot/channel/channel_publisher.hpp>' not in source
    assert '.Move(' not in source
    assert '.StopMove(' not in source


def test_sport_mode_command_observer_logs_raw_gait_fields():
    """先记录原始命令字段，禁止在观察阶段猜测 gait 数值的语义。"""
    source = (
        PACKAGE_ROOT / 'src' / 'go2_sdk_sport_mode_command_observer.cpp'
    ).read_text(encoding='utf-8')
    for field in ('mode=', 'gait_type=', 'speed_level=', 'foot_raise_height='):
        assert field in source
    assert 'gait 数值预先解释为任何步态名称' in source
