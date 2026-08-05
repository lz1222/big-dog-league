#!/usr/bin/env python3
"""Go2 冷启动控制面的只读标定采集器。

这不是 production gate：300 秒和 200 帧只是本次采集协议的有界范围，绝不
生成 production readiness，也不接受或推断 production 的六个门禁参数。
它只执行网络查询、ICMP 和只读 SportModeState 订阅。
"""

import argparse
import csv
import datetime
import hashlib
import json
import os
import queue
import re
import shutil
import statistics
import subprocess
import sys
import threading
import time


SCHEMA_VERSION = '1.0'
MEASUREMENT_HARD_TIMEOUT_SEC = 300.0
TARGET_VALID_FRAMES = 200
PING_INTERVAL_SEC = 0.25


def now_record(start_ns):
    """统一生成墙钟、单调钟和相对耗时，避免系统校时破坏时间线。"""
    monotonic_ns = time.monotonic_ns()
    return {
        'wall_time': datetime.datetime.now(
            datetime.timezone.utc).astimezone().isoformat(),
        'monotonic_ns': monotonic_ns,
        'elapsed_ms': (monotonic_ns - start_ns) / 1000000.0,
    }


def run_read_only(command):
    """仅允许 ip/ping 查询；该 helper 不得用于 SDK 或 ROS 控制命令。"""
    return subprocess.run(
        command, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
        stderr=subprocess.PIPE, text=True, check=False,
    )


def sha256_file(path):
    """记录实际执行物哈希，保证测量证据可追溯而不改变任何可执行文件。"""
    digest = hashlib.sha256()
    try:
        with open(path, 'rb') as source:
            for block in iter(lambda: source.read(1024 * 1024), b''):
                digest.update(block)
    except OSError as error:
        return 'unavailable: {}'.format(error)
    return digest.hexdigest()


def write_post_exit_snapshots(run_root):
    """只读保存退出后进程与 UDP 快照，供残留审计，绝不清理或终止其他进程。"""
    snapshots = (
        ('processes_after.txt', ['ps', '-eo', 'pid=,ppid=,args=']),
        ('udp_after.txt', ['ss', '-lunp']),
    )
    for filename, command in snapshots:
        result = run_read_only(command)
        with open(os.path.join(run_root, filename), 'w') as output:
            output.write(result.stdout)
            if result.stderr:
                output.write('\n# stderr\n{}'.format(result.stderr))
            output.write('\n# exit_code={}\n'.format(result.returncode))


def parse_rtt_ms(text):
    """从 iputils ping 输出提取 RTT；缺失时保留空值而非猜测零延迟。"""
    match = re.search(r'(?:time|时间)\s*[=<]\s*([0-9]+(?:\.[0-9]+)?)\s*ms', text)
    if match:
        return float(match.group(1))
    return ''


def percentile(values, ratio):
    """线性插值百分位；空序列显式返回 None。"""
    if not values:
        return None
    ordered = sorted(values)
    position = (len(ordered) - 1) * ratio
    lower = int(position)
    upper = min(lower + 1, len(ordered) - 1)
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (position - lower)


def key_values(line):
    """解析 monitor 的空格分隔诊断字段，字段值均不含空格。"""
    result = {}
    for token in line.strip().split()[1:]:
        if '=' in token:
            key, value = token.split('=', 1)
            result[key] = value
    return result


def reader(stream, destination, output_queue):
    """后台逐行落盘并入队，防止 DDS 回调输出因管道未读而阻塞。"""
    try:
        for line in iter(stream.readline, ''):
            destination.write(line)
            destination.flush()
            output_queue.put(line.rstrip('\n'))
    finally:
        stream.close()


def write_event(writer, start_ns, event, **details):
    record = now_record(start_ns)
    writer.writerow([
        event, record['wall_time'], record['monotonic_ns'],
        '{:.3f}'.format(record['elapsed_ms']), json.dumps(details, sort_keys=True),
    ])


def valid_statistics(frames):
    """依据实际有效帧接收时刻统计周期；状态中断定义为大于三倍观测中位周期。"""
    valid_times = [frame['monotonic_ns'] for frame in frames if frame['valid']]
    gaps_ms = [
        (current - previous) / 1000000.0
        for previous, current in zip(valid_times, valid_times[1:])
    ]
    median_gap = statistics.median(gaps_ms) if gaps_ms else None
    interruptions = []
    if median_gap is not None:
        interruptions = [gap for gap in gaps_ms if gap > 3.0 * median_gap]
    return {
        'average_frame_period_ms': statistics.mean(gaps_ms) if gaps_ms else None,
        'median_frame_period_ms': median_gap,
        'p90_frame_period_ms': percentile(gaps_ms, 0.90),
        'p95_frame_period_ms': percentile(gaps_ms, 0.95),
        'p99_frame_period_ms': percentile(gaps_ms, 0.99),
        'max_frame_gap_ms': max(gaps_ms) if gaps_ms else None,
        'state_interruptions': len(interruptions),
        'longest_state_interruption_ms': max(interruptions) if interruptions else 0.0,
    }


def classify_measurement(frames, valid_frames, monitor_returncode):
    """仅给出采集完整性，不把帧数或时限解释为 production readiness。"""
    if valid_frames >= TARGET_VALID_FRAMES and monitor_returncode == 0:
        return 'MEASUREMENT_COMPLETE'
    if not frames:
        return 'MEASUREMENT_INCOMPLETE_NO_DDS_STATE'
    return 'MEASUREMENT_INCOMPLETE_INSUFFICIENT_FRAMES'


def parse_args(argv):
    """采集协议固定且显式；这些参数不是 production gate 阈值。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-root', required=True)
    parser.add_argument('--interface', default='eth0')
    parser.add_argument('--local-ip', default='192.168.123.18')
    parser.add_argument('--robot-ip', default='192.168.123.161')
    parser.add_argument('--runtime-wrapper', required=True)
    parser.add_argument('--monitor', required=True)
    parser.add_argument('--branch', required=True)
    parser.add_argument('--commit', required=True)
    return parser.parse_args(argv)


def main(argv=None):
    """启动一个只读 monitor，持续采样至 200 有效帧或采集硬上限。"""
    args = parse_args(sys.argv[1:] if argv is None else argv)
    os.makedirs(args.run_root, exist_ok=True)
    start_ns = time.monotonic_ns()
    command = [
        args.runtime_wrapper, args.monitor, args.interface,
        '--calibration-stream', '--max-valid-frames', str(TARGET_VALID_FRAMES),
    ]
    environment = dict(os.environ)
    with open(os.path.join(args.run_root, 'environment.txt'), 'w') as output:
        for key in sorted(environment):
            output.write('{}={}\n'.format(key, environment[key]))
        output.write('measurement_hard_timeout_sec={}\n'.format(
            MEASUREMENT_HARD_TIMEOUT_SEC))
        output.write('target_valid_frames={}\n'.format(TARGET_VALID_FRAMES))
        output.write('ping_interval_sec={}\n'.format(PING_INTERVAL_SEC))
        output.write('uses_production_thresholds=false\n')
        output.write('monitor_argv={}\n'.format(json.dumps(command)))
    with open(os.path.join(args.run_root, 'executable_sha256.txt'), 'w') as output:
        output.write('runtime_wrapper {}\n'.format(sha256_file(args.runtime_wrapper)))
        output.write('monitor {}\n'.format(sha256_file(args.monitor)))

    frames = []
    network_failures = []
    first = {}
    monitor_queue = queue.Queue()
    monitor = None
    monitor_returncode = None
    next_ping = start_ns
    ping_sequence = 0
    consecutive_ping_failures = 0
    max_ping_failure_duration_ms = 0.0
    failure_start_ns = None
    classification = 'MEASUREMENT_INCOMPLETE_NO_DDS_STATE'

    with open(os.path.join(args.run_root, 'measurement_events.csv'), 'w', newline='') as event_file, \
            open(os.path.join(args.run_root, 'ping_samples.csv'), 'w', newline='') as ping_file, \
            open(os.path.join(args.run_root, 'state_frames.csv'), 'w', newline='') as state_file, \
            open(os.path.join(args.run_root, 'monitor_stdout.log'), 'w') as stdout_file, \
            open(os.path.join(args.run_root, 'monitor_stderr.log'), 'w') as stderr_file:
        events = csv.writer(event_file)
        pings = csv.writer(ping_file)
        states = csv.writer(state_file)
        events.writerow(['event', 'wall_time', 'monotonic_ns', 'elapsed_ms', 'details_json'])
        pings.writerow(['sequence', 'wall_time', 'monotonic_ns', 'elapsed_ms', 'success', 'rtt_ms', 'consecutive_failures', 'error'])
        states.writerow(['raw_index', 'valid_index', 'valid', 'reason', 'wall_ns', 'monotonic_ns', 'elapsed_ms', 'gap_ms'])
        write_event(events, start_ns, 'MEASUREMENT_START',
                    protocol='read_only_calibration',
                    uses_production_thresholds=False)
        write_event(events, start_ns, 'MONITOR_PROCESS_START', argv=command)
        monitor = subprocess.Popen(
            command, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
            stderr=subprocess.PIPE, text=True, bufsize=1, env=environment,
        )
        stdout_thread = threading.Thread(
            target=reader, args=(monitor.stdout, stdout_file, monitor_queue), daemon=True
        )
        stdout_thread.start()
        stderr_thread = threading.Thread(
            target=reader, args=(monitor.stderr, stderr_file, queue.Queue()), daemon=True
        )
        stderr_thread.start()
        try:
            with open('/proc/{}/maps'.format(monitor.pid), 'r') as maps:
                with open(os.path.join(args.run_root, 'process_maps.txt'), 'w') as output:
                    shutil.copyfileobj(maps, output)
        except OSError as error:
            with open(os.path.join(args.run_root, 'process_maps.txt'), 'w') as maps:
                maps.write('unavailable: {}\n'.format(error))

        def drain_monitor_queue():
            """将已落入 stdout 管道的末尾帧全部记账，避免目标退出时少计帧。"""
            while True:
                try:
                    line = monitor_queue.get_nowait()
                except queue.Empty:
                    break
                if line.startswith('CALIBRATION_EVENT'):
                    fields = key_values(line)
                    event = fields.pop('event', 'MONITOR_EVENT')
                    if event == 'FIRST_DDS_DISCOVERY':
                        first.setdefault('first_dds_discovery_ns', int(fields['monotonic_ns']))
                    if event == 'CHANNEL_FACTORY_INIT_COMPLETE':
                        first.setdefault('channel_factory_complete_ns', int(fields['monotonic_ns']))
                    write_event(events, start_ns, event, monitor_fields=fields)
                elif line.startswith('CALIBRATION_FRAME'):
                    fields = key_values(line)
                    valid = fields.get('valid') == '1'
                    frame_ns = int(fields['monotonic_ns'])
                    gap_ms = ''
                    if frames:
                        gap_ms = (frame_ns - frames[-1]['monotonic_ns']) / 1000000.0
                    frame = {
                        'raw_index': int(fields['raw_index']),
                        'valid_index': int(fields.get('valid_index', '0')),
                        'valid': valid, 'reason': fields.get('reason', ''),
                        'wall_ns': int(fields['wall_ns']), 'monotonic_ns': frame_ns,
                    }
                    frames.append(frame)
                    states.writerow([
                        frame['raw_index'], frame['valid_index'], int(valid), frame['reason'],
                        frame['wall_ns'], frame_ns,
                        '{:.3f}'.format((frame_ns - start_ns) / 1000000.0), gap_ms,
                    ])
                    if 'first_state_frame_ns' not in first:
                        first['first_state_frame_ns'] = frame_ns
                        write_event(events, start_ns, 'FIRST_STATE_FRAME')
                    if valid and 'first_valid_state_ns' not in first:
                        first['first_valid_state_ns'] = frame_ns
                        write_event(events, start_ns, 'FIRST_VALID_STATE')

        while True:
            current_ns = time.monotonic_ns()
            if current_ns - start_ns >= int(MEASUREMENT_HARD_TIMEOUT_SEC * 1e9):
                write_event(events, start_ns, 'MEASUREMENT_HARD_TIMEOUT')
                break
            drain_monitor_queue()
            if current_ns >= next_ping:
                ping_sequence += 1
                link = run_read_only(['ip', '-br', 'link', 'show', args.interface])
                address = run_read_only(['ip', '-br', 'addr', 'show', args.interface])
                route = run_read_only(['ip', 'route', 'get', args.robot_ip])
                if link.returncode == 0 and 'UP' in link.stdout:
                    if 'eth0_link_up_ns' not in first:
                        first['eth0_link_up_ns'] = current_ns
                        write_event(events, start_ns, 'ETH0_LINK_UP', output=link.stdout.strip())
                if address.returncode == 0 and args.local_ip in address.stdout:
                    if 'eth0_address_ready_ns' not in first:
                        first['eth0_address_ready_ns'] = current_ns
                        write_event(events, start_ns, 'ETH0_ADDRESS_READY', output=address.stdout.strip())
                if route.returncode == 0 and ' dev {} '.format(args.interface) in (' ' + route.stdout + ' '):
                    if 'route_ready_ns' not in first:
                        first['route_ready_ns'] = current_ns
                        write_event(events, start_ns, 'ROUTE_READY', output=route.stdout.strip())
                ping = run_read_only(['ping', '-I', args.interface, '-c', '1', '-W', '1', args.robot_ip])
                success = ping.returncode == 0
                if success:
                    if 'first_ping_ns' not in first:
                        first['first_ping_ns'] = time.monotonic_ns()
                        write_event(events, start_ns, 'FIRST_PING_SUCCESS', rtt_ms=parse_rtt_ms(ping.stdout))
                    if failure_start_ns is not None:
                        max_ping_failure_duration_ms = max(
                            max_ping_failure_duration_ms,
                            (time.monotonic_ns() - failure_start_ns) / 1000000.0,
                        )
                        failure_start_ns = None
                    consecutive_ping_failures = 0
                else:
                    network_failures.append(current_ns)
                    consecutive_ping_failures += 1
                    if failure_start_ns is None:
                        failure_start_ns = current_ns
                ping_record = now_record(start_ns)
                pings.writerow([
                    ping_sequence, ping_record['wall_time'], ping_record['monotonic_ns'],
                    '{:.3f}'.format(ping_record['elapsed_ms']), int(success),
                    parse_rtt_ms(ping.stdout) if success else '', consecutive_ping_failures,
                    ping.stderr.strip(),
                ])
                next_ping = current_ns + int(PING_INTERVAL_SEC * 1e9)
            if monitor.poll() is not None:
                monitor_returncode = monitor.returncode
                break
            time.sleep(0.01)

        if monitor.poll() is None:
            monitor.terminate()
        try:
            monitor_returncode = monitor.wait(timeout=10)
        except subprocess.TimeoutExpired:
            # 仅结束本脚本创建的只读订阅子进程；不会触及 SDK Server 或机器人。
            monitor.kill()
            monitor_returncode = monitor.wait(timeout=10)
        # wait() 仅保证子进程退出，不保证后台 reader 已把管道最后几帧放入队列。
        # 先等待 reader EOF 再排空队列，防止完整 200 帧采集被误判为不完整。
        stdout_thread.join(timeout=1)
        stderr_thread.join(timeout=1)
        drain_monitor_queue()
        write_event(events, start_ns, 'MONITOR_PROCESS_EXIT', returncode=monitor_returncode)
        write_event(events, start_ns, 'MEASUREMENT_END')

    valid_frames = sum(1 for frame in frames if frame['valid'])
    classification = classify_measurement(frames, valid_frames, monitor_returncode)
    statistics_data = valid_statistics(frames)
    summary = {
        'schema_version': SCHEMA_VERSION,
        'branch': args.branch, 'commit': args.commit,
        'interface': args.interface, 'robot_ip': args.robot_ip,
        'measurement_protocol': {
            'measurement_hard_timeout_sec': MEASUREMENT_HARD_TIMEOUT_SEC,
            'target_valid_frames': TARGET_VALID_FRAMES,
            'ping_interval_sec': PING_INTERVAL_SEC,
            'uses_production_thresholds': False,
        },
        'sport_actions_called': False,
        'production_readiness_conclusion': False,
        'classification': classification,
        'monitor_returncode': monitor_returncode,
        'total_frames': len(frames), 'valid_frames': valid_frames,
        'invalid_frames': len(frames) - valid_frames,
        'network_interruption_samples': len(network_failures),
        'longest_ping_failure_ms': max_ping_failure_duration_ms,
        'timestamps_monotonic_ns': first,
        'statistics': statistics_data,
    }
    with open(os.path.join(args.run_root, 'measurement_summary.json'), 'w') as output:
        json.dump(summary, output, ensure_ascii=False, indent=2, sort_keys=True)
        output.write('\n')
    with open(os.path.join(args.run_root, 'measurement_report.md'), 'w') as output:
        output.write('# Go2 cold-boot calibration measurement\n\n')
        output.write('- Classification: `{}`\n'.format(classification))
        output.write('- Production thresholds used: `false`\n')
        output.write('- Sport actions called: `false`\n')
        output.write('- Valid frames: `{}` / `{}` target\n'.format(valid_frames, TARGET_VALID_FRAMES))
        output.write('- Frame statistics: `{}`\n'.format(json.dumps(statistics_data)))
    write_post_exit_snapshots(args.run_root)
    print('CALIBRATION_MEASUREMENT classification={}'.format(classification))
    return 0 if classification == 'MEASUREMENT_COMPLETE' else 1


if __name__ == '__main__':
    sys.exit(main())
