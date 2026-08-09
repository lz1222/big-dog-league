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
    # 统计结果可以合法包含同名字段，但 CLI 绝不能接收 production 阈值。
    for production_option in (
            '--network-timeout-sec', '--ping-count', '--dds-timeout-sec',
            '--required-frames', '--max-frame-gap-ms'):
        try:
            measure.parse_args([
                '--run-root', str(tmp_path), '--runtime-wrapper', '/wrapper',
                '--monitor', '/monitor', '--branch', 'test', '--commit', 'abc',
                production_option, '1',
            ])
        except SystemExit as error:
            assert error.code != 0
        else:
            raise AssertionError(
                '{} must not be a calibration CLI option'.format(
                    production_option
                )
            )


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


def test_measurement_source_cannot_start_control_or_sport_processes():
    """测量子进程 argv 只能是 runtime wrapper 加只读 monitor。"""
    source = MEASURE_PATH.read_text(encoding='utf-8')
    for forbidden in (
            'go2_sdk_udp_server', 'cmd_vel_udp_forwarder', 'StopMove',
            '.Move(', '.BalanceStand(', 'ros2 topic pub', 'mission/start'):
        assert forbidden not in source
    assert '--calibration-stream' in source
    assert 'production_readiness_conclusion' in source
