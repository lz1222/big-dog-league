"""隔离动态 adapter 的运动时长命令行安全边界。"""

import pytest

from rk_bringup.validation_motion_adapter_node import parse_arguments


def test_eight_second_window_and_hard_watchdog_are_accepted():
    """现场直线确认只能使用固定 8.0 s 与不超过 8.2 s 的硬上限。"""
    arguments = parse_arguments([
        '--server-instance-id', 'validation-instance',
        '--motion-sec', '8.0',
        '--watchdog-sec', '8.2',
    ])
    assert arguments.motion_sec == 8.0
    assert arguments.watchdog_sec == 8.2


@pytest.mark.parametrize(
    'arguments',
    [
        ['--motion-sec', '8.01', '--watchdog-sec', '8.2'],
        ['--motion-sec', '8.0', '--watchdog-sec', '8.21'],
        ['--motion-sec', '8.0', '--watchdog-sec', '8.0'],
    ],
)
def test_motion_window_limits_remain_fail_closed(arguments):
    with pytest.raises(SystemExit):
        parse_arguments(['--server-instance-id', 'validation-instance'] + arguments)
