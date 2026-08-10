"""固定放置 executable 的纯软件接口测试；绝不启动真实 D1 程序。"""

from pathlib import Path
import subprocess
from types import SimpleNamespace

from rk_arm_control.fixed_platform_action_runner import (
    FixedPlatformActionRunner,
)


def test_disabled_fixed_runner_never_invokes_process(tmp_path):
    called = []
    runner = FixedPlatformActionRunner(
        {'enabled': False, 'binary_directory': str(tmp_path)},
        executor=lambda *args, **kwargs: called.append((args, kwargs)))
    result = runner.execute('PLACE_PLATFORM_1')
    assert not result.success
    assert called == []


def test_platform_tasks_select_distinct_existing_executables(tmp_path):
    commands = []
    for name in (
            'd1_platform1_complete_action',
            'd1_platform2_complete_action'):
        path = Path(tmp_path) / name
        path.touch()
        path.chmod(0o755)

    def executor(command, **kwargs):
        commands.append((command, kwargs))
        return SimpleNamespace(returncode=0, stderr='')

    runner = FixedPlatformActionRunner(
        {'enabled': True, 'binary_directory': str(tmp_path),
         'network_interface': 'eth1'}, executor=executor)
    assert runner.execute('PLACE_PLATFORM_1').success
    assert runner.execute('PLACE_PLATFORM_2').success
    assert commands[0][0][0].endswith('d1_platform1_complete_action')
    assert commands[1][0][0].endswith('d1_platform2_complete_action')
    assert all(command[0][1:] == ('eth1', '--execute', '--power-on')
               for command in commands)


def test_timeout_and_failed_process_never_report_action_success(tmp_path):
    path = Path(tmp_path) / 'd1_platform1_complete_action'
    path.touch()
    path.chmod(0o755)
    runner = FixedPlatformActionRunner(
        {'enabled': True, 'binary_directory': str(tmp_path)},
        executor=lambda *args, **kwargs: (_ for _ in ()).throw(
            subprocess.TimeoutExpired(args[0], kwargs['timeout'])))
    assert not runner.execute('PLACE_PLATFORM_1').success

    failed = FixedPlatformActionRunner(
        {'enabled': True, 'binary_directory': str(tmp_path)},
        executor=lambda *args, **kwargs: SimpleNamespace(
            returncode=5, stderr='hardware rejected'))
    assert not failed.execute('PLACE_PLATFORM_1').success
