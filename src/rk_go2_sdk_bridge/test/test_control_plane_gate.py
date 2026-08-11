"""冷启动控制面门禁的纯软件测试，不需要网卡、DDS 或实体机器人。"""

import importlib.util
import json
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


def _pass_evidence(instance_id, **overrides):
    """构造完整观测 PASS，测试 parent 不会只信任子进程返回码。"""
    evidence = {
        'probe_instance_id': instance_id,
        'classification': 'CONTROL_PLANE_OBSERVATION_PASS',
        'total_frames': 201,
        'valid_frames': 201,
        'invalid_frames': 0,
        'max_gap_ms': 7,
        'first_monotonic_ns': 100,
        'last_monotonic_ns': 200,
        'pass_monotonic_ns': 300,
        'motion_calls': 0,
        'mutating_calls': 0,
    }
    evidence.update(overrides)
    return evidence


def test_run_probe_uses_only_read_only_monitor(monkeypatch, tmp_path):
    """门禁只 exec monitor 的 --gate 参数，命令行没有任何 Sport 动作。"""
    gate = _load_gate_module()
    captured = {}

    def fake_run(command, check):
        captured['command'] = command
        return type('Result', (), {'returncode': 0})()

    monkeypatch.setattr(gate.subprocess, 'run', fake_run)
    assert gate.run_probe(
        '/runtime', '/monitor', 'eth0', 10, 5, 200, 'probe-id',
        tmp_path / 'evidence.json', 'controlled',
    ) == 0
    command = captured['command']
    assert command[:4] == ['/runtime', '/monitor', 'eth0', '--gate']
    assert '--controlled-terminal-success' in command
    assert not any(
        action in ' '.join(command)
        for action in ('StopMove', 'Move', 'BalanceStand')
    )


def test_pass_evidence_requires_this_instance_and_contract(tmp_path):
    """PASS 必须同时满足实例、帧数、gap、零调用及单调时间戳合同。"""
    gate = _load_gate_module()
    path = tmp_path / 'pass.json'
    path.write_text(json.dumps(_pass_evidence('current')), encoding='utf-8')
    assert gate.validate_pass_evidence(path, 'current', 200, 20) == (
        True, 'valid_pass_evidence',
    )


def test_invalid_or_stale_pass_evidence_fails_closed(tmp_path):
    """RC=0 也不能复用旧文件、错误实例或低质量 observation。"""
    gate = _load_gate_module()
    cases = (
        _pass_evidence('old'),
        _pass_evidence('current', valid_frames=199),
        _pass_evidence('current', max_gap_ms=21),
        _pass_evidence('current', motion_calls=1),
    )
    for index, evidence in enumerate(cases):
        path = tmp_path / '{}.json'.format(index)
        path.write_text(json.dumps(evidence), encoding='utf-8')
        assert gate.validate_pass_evidence(path, 'current', 200, 20)[0] is False


def test_main_rejects_rc_zero_without_valid_pass_evidence(monkeypatch, tmp_path):
    """子进程 RC=0、RC=-6 都不能替代同实例 PASS evidence。"""
    gate = _load_gate_module()
    monkeypatch.setattr(gate, 'stable_ping', lambda *args: True)
    monkeypatch.setattr(gate, 'run_probe', lambda *args: 0)
    arguments = [
        '--interface', 'eth0', '--robot-ip', '192.168.123.161',
        '--runtime-wrapper', '/wrapper', '--probe', '/probe',
        '--network-timeout-sec', '1', '--ping-count', '1',
        '--ping-poll-sec', '0.1', '--dds-timeout-sec', '1',
        '--required-frames', '200', '--max-frame-gap-ms', '20',
        '--evidence-dir', str(tmp_path),
    ]
    assert gate.main(arguments) == 1


def test_main_rejects_teardown_abort_even_with_valid_evidence(
        monkeypatch, tmp_path):
    """旧式 SIGABRT/-6 有 evidence 也必须 fail-closed，而非 parent 伪造成功。"""
    gate = _load_gate_module()
    monkeypatch.setattr(gate, 'stable_ping', lambda *args: True)

    def fake_probe(*args):
        path = args[7]
        instance_id = args[6]
        Path(path).write_text(
            json.dumps(_pass_evidence(instance_id)), encoding='utf-8'
        )
        return -6

    monkeypatch.setattr(gate, 'run_probe', fake_probe)
    arguments = [
        '--interface', 'eth0', '--robot-ip', '192.168.123.161',
        '--runtime-wrapper', '/wrapper', '--probe', '/probe',
        '--network-timeout-sec', '1', '--ping-count', '1',
        '--ping-poll-sec', '0.1', '--dds-timeout-sec', '1',
        '--required-frames', '200', '--max-frame-gap-ms', '20',
        '--evidence-dir', str(tmp_path),
    ]
    assert gate.main(arguments) == -6


def test_gate_source_has_no_sport_action_names():
    """代码级回归保护：冷启动门禁禁止引入任何控制接口调用。"""
    source = GATE_PATH.read_text(encoding='utf-8')
    for forbidden in ('.StopMove(', '.Move(', '.BalanceStand('):
        assert forbidden not in source


def test_probe_source_limits_controlled_exit_to_read_only_gate():
    """受控退出只能位于观测 PASS evidence 已落盘后的独立只读 probe。"""
    source = (PACKAGE_ROOT / 'src' / 'go2_sdk_sport_state_monitor.cpp').read_text(
        encoding='utf-8'
    )
    assert 'subscriber_->CloseChannel()' in source
    assert 'ChannelFactory::Instance()->Release()' in source
    assert 'WritePassEvidence(config, statistics' in source
    assert 'std::_Exit(0)' in source
    assert 'if (config.controlled_terminal_success)' in source
    for forbidden in (
            'sport/sport_client.hpp', '.StopMove(', '.BalanceStand(', '.Move('):
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
    assert '"start_udp_forwarder:=false"' in competition_source
    forwarder_start = competition_source.index('FORWARDER_ARGS=')
    owner_gate = competition_source.index(
        'wait_for_global_gait_owner_status_subscriber', forwarder_start
    )
    assert competition_source.index('CONTROL_GATE_COMMAND=') < forwarder_start < (
        competition_source.index('--mode receiver')
    ) < competition_source.index(
        '-n ros_graph', forwarder_start
    ) < owner_gate < (
        competition_source.index('SERVER_ARGS=')
    ) < competition_source.index('--mode status')
    assert "grep -Fq 'UDP server listening on'" not in competition_source


def test_retry_exhaustion_precedes_udp_bind_and_has_no_extra_guard_stop():
    """三次启动失败时 socket 尚未建立，析构保护也不得隐式第四次停车。"""
    source = (PACKAGE_ROOT / 'src' / 'go2_sdk_udp_server.cpp').read_text(
        encoding='utf-8'
    )
    assert source.index('SendStartupStopWithRetry(client, status)') < source.index(
        'CreateUdpSocket(config)'
    )
    assert 'bool armed_{false};' in source
    assert 'stop_guard.Arm();' in source
