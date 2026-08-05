"""只读冷启动标定模式的纯软件回归测试。"""

import importlib.util
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
MEASURE_PATH = PACKAGE_ROOT / 'scripts' / 'go2_control_plane_measure.py'
GATE_PATH = PACKAGE_ROOT / 'scripts' / 'go2_control_plane_gate.py'


def _load_measure_module():
    spec = importlib.util.spec_from_file_location(
        'go2_control_plane_measure', str(MEASURE_PATH)
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_calibration_parser_does_not_accept_production_thresholds(tmp_path):
    """标定入口只要求采集身份/路径，不出现六个 production 参数。"""
    measure = _load_measure_module()
    arguments = measure.parse_args([
        '--run-root', str(tmp_path), '--runtime-wrapper', '/wrapper',
        '--monitor', '/monitor', '--branch', 'test', '--commit', 'abc',
    ])
    assert arguments.interface == 'eth0'
    source = MEASURE_PATH.read_text(encoding='utf-8')
    for production_flag in (
            '--network-timeout-sec', '--ping-count', '--ping-poll-sec',
            '--dds-timeout-sec', '--required-frames', '--max-frame-gap-ms'):
        assert production_flag not in source


def test_production_gate_remains_fail_closed_without_thresholds():
    """新增标定模式不能为 production gate 注入隐式默认值。"""
    spec = importlib.util.spec_from_file_location('gate', str(GATE_PATH))
    gate = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(gate)
    try:
        gate.parse_arguments([
            '--interface', 'eth0', '--robot-ip', '192.168.123.161',
            '--runtime-wrapper', '/wrapper', '--probe', '/monitor',
        ])
    except SystemExit as error:
        assert error.code != 0
    else:
        raise AssertionError('production gate must reject missing thresholds')


def test_frame_statistics_cover_average_percentiles_and_interruptions():
    """统计依据原始接收时刻，不把 200 帧范围误写成生产新鲜度阈值。"""
    measure = _load_measure_module()
    frames = [
        {'valid': True, 'monotonic_ns': 0},
        {'valid': True, 'monotonic_ns': 10_000_000},
        {'valid': False, 'monotonic_ns': 15_000_000},
        {'valid': True, 'monotonic_ns': 20_000_000},
        {'valid': True, 'monotonic_ns': 100_000_000},
    ]
    stats = measure.valid_statistics(frames)
    assert stats['average_frame_period_ms'] == 100.0 / 3.0
    assert stats['median_frame_period_ms'] == 10.0
    assert stats['p95_frame_period_ms'] > 60.0
    assert stats['max_frame_gap_ms'] == 80.0
    assert stats['state_interruptions'] == 1
    assert stats['longest_state_interruption_ms'] == 80.0


def test_ping_rtt_parser_records_iputils_time_value():
    """每次成功 ping 必须保留真实 RTT，不能把“time=… ms”误写为空。"""
    measure = _load_measure_module()
    output = ('64 bytes from 192.168.123.161: icmp_seq=1 ttl=64 '
              'time=0.482 ms\n')
    assert measure.parse_rtt_ms(output) == 0.482
    chinese_output = '64 字节，来自 192.168.123.161: icmp_seq=1 时间=0.213 毫秒\n'
    assert measure.parse_rtt_ms(chinese_output) == 0.213
    assert measure.parse_rtt_ms('timeout') == ''


def test_measurement_classification_never_becomes_production_readiness():
    """硬上限和帧数只表达采集完成度，三种结果均不等于生产 gate 成功。"""
    measure = _load_measure_module()
    assert measure.classify_measurement([], 0, None) == (
        'MEASUREMENT_INCOMPLETE_NO_DDS_STATE')
    assert measure.classify_measurement([{'valid': True}], 1, 0) == (
        'MEASUREMENT_INCOMPLETE_INSUFFICIENT_FRAMES')
    complete_frames = [{'valid': True}] * measure.TARGET_VALID_FRAMES
    assert measure.classify_measurement(
        complete_frames, measure.TARGET_VALID_FRAMES, 0) == 'MEASUREMENT_COMPLETE'


def test_post_exit_snapshots_are_read_only_process_observations(tmp_path, monkeypatch):
    """退出审计只读取 ps/ss，既不启动 SDK Server 也不尝试网络控制。"""
    measure = _load_measure_module()
    calls = []

    class Result:
        returncode = 0
        stdout = 'snapshot\n'
        stderr = ''

    def fake_read_only(command):
        calls.append(command)
        return Result()

    monkeypatch.setattr(measure, 'run_read_only', fake_read_only)
    measure.write_post_exit_snapshots(str(tmp_path))
    assert calls == [
        ['ps', '-eo', 'pid=,ppid=,args='],
        ['ss', '-lunp'],
    ]
    assert 'snapshot' in (tmp_path / 'processes_after.txt').read_text()
    assert 'snapshot' in (tmp_path / 'udp_after.txt').read_text()


def test_measurement_source_cannot_start_control_or_sport_processes():
    """测量子进程 argv 只能是 runtime wrapper 加只读 monitor。"""
    source = MEASURE_PATH.read_text(encoding='utf-8')
    for forbidden in (
            'go2_sdk_udp_server', 'cmd_vel_udp_forwarder', '.StopMove(',
            '.Move(', '.BalanceStand(', 'ros2 topic pub', 'mission/start'):
        assert forbidden not in source
    assert '--calibration-stream' in source
    assert 'production_readiness_conclusion' in source
