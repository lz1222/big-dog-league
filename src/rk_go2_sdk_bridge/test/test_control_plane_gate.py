"""冷启动控制面门禁的纯软件测试，不需要网卡、DDS 或实体机器人。"""

import importlib.util
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
GATE_PATH = PACKAGE_ROOT / 'scripts' / 'go2_control_plane_gate.py'
WORKSPACE_ROOT = PACKAGE_ROOT.parents[1]


def _load_gate_module():
    spec = importlib.util.spec_from_file_location(
        'go2_control_plane_gate', str(GATE_PATH)
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_gate_requires_measured_thresholds():
    """缺少帧数/时窗不得悄悄采用固定 sleep 或猜测默认值。"""
    gate = _load_gate_module()
    try:
        gate.parse_arguments([
            '--interface', 'eth0', '--robot-ip', '192.168.123.161',
            '--runtime-wrapper', '/wrapper', '--probe', '/probe',
        ])
    except SystemExit as error:
        assert error.code != 0
    else:
        raise AssertionError('gate must reject unmeasured threshold defaults')


def test_stable_ping_rejects_single_success_then_loss(monkeypatch):
    """网络可达但只成功一次不是稳定网络，不能进入 DDS probe。"""
    gate = _load_gate_module()
    calls = iter([True, False, False])
    monkeypatch.setattr(
        gate, 'interface_and_route_ready', lambda interface, ip: next(calls, False)
    )
    monkeypatch.setattr(gate.time, 'sleep', lambda seconds: None)
    monkeypatch.setattr(
        gate, '_run_read_only',
        lambda command: type('Result', (), {'returncode': 0})(),
    )
    ticks = iter([0.0, 0.0, 0.1, 2.0])
    monkeypatch.setattr(gate.time, 'monotonic', lambda: next(ticks, 2.0))
    assert not gate.stable_ping('eth0', '192.168.123.161', 2, 1.0, 0.1)


def test_run_probe_uses_only_read_only_monitor(monkeypatch):
    """门禁只 exec monitor 的 --gate 参数，命令行没有任何 Sport 动作。"""
    gate = _load_gate_module()
    captured = {}

    def fake_run(command, check):
        captured['command'] = command
        return type('Result', (), {'returncode': 0})()

    monkeypatch.setattr(gate.subprocess, 'run', fake_run)
    assert gate.run_probe('/runtime', '/monitor', 'eth0', 10, 5, 200) == 0
    command = captured['command']
    assert command[:4] == ['/runtime', '/monitor', 'eth0', '--gate']
    assert not any(
        action in ' '.join(command)
        for action in ('StopMove', 'Move', 'BalanceStand')
    )


def test_gate_source_has_no_sport_action_names():
    """代码级回归保护：冷启动门禁禁止引入任何控制接口调用。"""
    source = GATE_PATH.read_text(encoding='utf-8')
    for forbidden in ('.StopMove(', '.Move(', '.BalanceStand('):
        assert forbidden not in source


def test_formal_start_scripts_keep_probe_sdk_ros_ordering():
    """阶段 A/B/C 的文本顺序防止未来把相机或 ROS 图移回冷启动前。"""
    line_source = (
        WORKSPACE_ROOT / 'src' / 'rk_bringup' / 'scripts' /
        'start_line_system.sh'
    ).read_text(encoding='utf-8')
    competition_source = (
        WORKSPACE_ROOT / 'src' / 'rk_bringup' / 'scripts' /
        'start_non_arm_competition.sh'
    ).read_text(encoding='utf-8')

    assert line_source.index('run_step "control_plane_gate"') < (
        line_source.index('start_background "sdk_server"')
    ) < line_source.index('start_background "realsense_camera"')
    assert '"start_sdk_server:=false"' in competition_source
    assert competition_source.index('GATE_COMMAND=') < (
        competition_source.index('SERVER_COMMAND=')
    ) < competition_source.index("'UDP server listening on'")


def test_retry_exhaustion_precedes_udp_bind_and_has_no_extra_guard_stop():
    """三次启动失败时 socket 尚未建立，析构保护也不得隐式第四次停车。"""
    source = (PACKAGE_ROOT / 'src' / 'go2_sdk_udp_server.cpp').read_text(
        encoding='utf-8'
    )
    assert source.index('SendStartupStopWithRetry(client)') < source.index(
        'CreateUdpSocket(config)'
    )
    assert 'bool armed_{false};' in source
    assert 'stop_guard.Arm();' in source
