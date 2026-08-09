#!/usr/bin/env python3
"""Go2 冷启动控制面只读门禁。

本程序只读取 Linux 链路/路由状态、发送 ICMP echo，并执行只订阅
``rt/sportmodestate`` 的 helper。它从不创建 SportClient，也绝不调用
Move、StopMove、BalanceStand 或任何姿态动作；成功仅代表可以开始启动
SDK UDP server，不能代表机器人已经接收过运动命令。
"""

import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import uuid
import time


def _event(name, **fields):
    """以单行键值记录冷启动时间线，方便跨次冷启动比较。"""
    parts = ['CONTROL_PLANE_DIAG', 'event={}'.format(name),
             'monotonic_sec={:.3f}'.format(time.monotonic())]
    parts.extend('{}={}'.format(key, value) for key, value in fields.items())
    print(' '.join(parts), flush=True)


def _run_read_only(command):
    """执行只读系统查询；调用方只可传入 ip 或 ping 的固定参数。"""
    return subprocess.run(
        command,
        stdin=subprocess.DEVNULL,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        check=False,
    )


def interface_and_route_ready(interface, robot_ip):
    """网络可达的必要条件：接口 UP 且到机器人路由确实走该接口。"""
    link = _run_read_only(['ip', 'link', 'show', 'dev', interface])
    if link.returncode != 0:
        return False
    # 内核输出的 state UP 和 flags 中的 UP 均接受，兼容不同 iproute2 版本。
    if 'state UP' not in link.stdout and ',UP,' not in link.stdout:
        return False
    route = _run_read_only(['ip', 'route', 'get', robot_ip])
    return route.returncode == 0 and ' dev {} '.format(interface) in (
        ' ' + route.stdout + ' '
    )


def stable_ping(interface, robot_ip, required_count, timeout_sec, poll_sec):
    """等待连续 ICMP 成功；失败重置计数而不是把偶发 ping 当成已稳定。"""
    deadline = time.monotonic() + timeout_sec
    consecutive = 0
    first_link_logged = False
    while time.monotonic() < deadline:
        if interface_and_route_ready(interface, robot_ip):
            if not first_link_logged:
                _event('FIRST_LINK_UP', interface=interface)
                first_link_logged = True
            ping = _run_read_only([
                'ping', '-I', interface, '-c', '1', '-W', '1', robot_ip,
            ])
            if ping.returncode == 0:
                if consecutive == 0:
                    _event('FIRST_PING', robot_ip=robot_ip)
                consecutive += 1
                if consecutive >= required_count:
                    _event('NETWORK_STABLE', consecutive=consecutive)
                    return True
            else:
                consecutive = 0
        else:
            consecutive = 0
        time.sleep(poll_sec)
    return False


def make_probe_evidence_path(evidence_dir):
    """为本次进程生成私有且不可复用的 PASS evidence 文件路径。"""
    directory = Path(evidence_dir).expanduser()
    directory.mkdir(mode=0o700, parents=True, exist_ok=True)
    probe_instance_id = str(uuid.uuid4())
    return probe_instance_id, directory / '{}.json'.format(probe_instance_id)


def validate_pass_evidence(path, probe_instance_id, required_frames,
                           max_frame_gap_ms):
    """只接受本次实例、完整帧合同和零 mutation 的 fsync PASS 证据。"""
    try:
        with Path(path).open('r', encoding='utf-8') as stream:
            evidence = json.load(stream)
    except (OSError, ValueError, TypeError) as error:
        return False, 'evidence_unreadable:{}'.format(error)
    if not isinstance(evidence, dict):
        return False, 'evidence_not_object'
    if evidence.get('probe_instance_id') != probe_instance_id:
        return False, 'probe_instance_id_mismatch'
    if evidence.get('classification') != 'CONTROL_PLANE_OBSERVATION_PASS':
        return False, 'classification_mismatch'
    for field in ('total_frames', 'valid_frames', 'invalid_frames',
                  'max_gap_ms', 'first_monotonic_ns', 'last_monotonic_ns',
                  'pass_monotonic_ns', 'motion_calls', 'mutating_calls'):
        value = evidence.get(field)
        if not isinstance(value, int) or isinstance(value, bool):
            return False, 'invalid_{}'.format(field)
    if (evidence['total_frames'] < required_frames or
            evidence['valid_frames'] < required_frames):
        return False, 'frame_count_insufficient'
    if evidence['invalid_frames'] != 0:
        return False, 'invalid_frames_nonzero'
    if evidence['max_gap_ms'] > max_frame_gap_ms:
        return False, 'max_gap_exceeds_contract'
    if (evidence['first_monotonic_ns'] <= 0 or
            evidence['last_monotonic_ns'] < evidence['first_monotonic_ns'] or
            evidence['pass_monotonic_ns'] < evidence['last_monotonic_ns']):
        return False, 'monotonic_timestamp_invalid'
    if evidence['motion_calls'] != 0 or evidence['mutating_calls'] != 0:
        return False, 'motion_or_mutation_nonzero'
    return True, 'valid_pass_evidence'


def run_probe(runtime_wrapper, probe, interface, timeout_sec, required_frames,
              max_frame_gap_ms, probe_instance_id, evidence_path,
              terminal_success_mode):
    """经 SDK 运行时隔离执行唯一 DDS probe，并把诊断原样写入本次日志。"""
    command = [
        runtime_wrapper, probe, interface, '--gate',
        '--timeout-sec', str(timeout_sec),
        '--required-frames', str(required_frames),
        '--max-frame-gap-ms', str(max_frame_gap_ms),
        '--probe-instance-id', probe_instance_id,
        '--pass-evidence', str(evidence_path),
    ]
    if terminal_success_mode == 'controlled':
        command.append('--controlled-terminal-success')
    _event('PROBE_EXEC', executable=probe)
    result = subprocess.run(command, check=False)
    return result.returncode


def parse_arguments(argv):
    """所有阈值必须由冷启动测量给出，避免猜测等待进入生产链。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--interface', required=True)
    parser.add_argument('--robot-ip', required=True)
    parser.add_argument('--runtime-wrapper', required=True)
    parser.add_argument('--probe', required=True)
    parser.add_argument('--network-timeout-sec', required=True, type=float)
    parser.add_argument('--ping-count', required=True, type=int)
    parser.add_argument('--ping-poll-sec', required=True, type=float)
    parser.add_argument('--dds-timeout-sec', required=True, type=float)
    parser.add_argument('--required-frames', required=True, type=int)
    parser.add_argument('--max-frame-gap-ms', required=True, type=int)
    parser.add_argument('--evidence-dir', required=True)
    parser.add_argument(
        '--terminal-success-mode', choices=('normal', 'controlled'),
        default='normal',
    )
    args = parser.parse_args(argv)
    if (args.network_timeout_sec <= 0 or args.ping_count <= 0 or
            args.ping_poll_sec <= 0 or args.dds_timeout_sec <= 0 or
            args.required_frames <= 0 or args.max_frame_gap_ms <= 0):
        parser.error('all timeout/count/frame-gap values must be positive')
    return args


def main(argv=None):
    """依次建立网络与 DDS 证据；失败时非零退出，由编排层拒绝启动。"""
    args = parse_arguments(sys.argv[1:] if argv is None else argv)
    _event('ROBOT_POWER_ON', source='operator_unrecorded')
    if not stable_ping(
            args.interface, args.robot_ip, args.ping_count,
            args.network_timeout_sec, args.ping_poll_sec):
        _event('FAILED', classification='ROBOT_NETWORK_NOT_READY')
        return 1
    probe_instance_id, evidence_path = make_probe_evidence_path(
        args.evidence_dir
    )
    result = run_probe(
        args.runtime_wrapper, args.probe, args.interface,
        args.dds_timeout_sec, args.required_frames, args.max_frame_gap_ms,
        probe_instance_id, evidence_path, args.terminal_success_mode,
    )
    evidence_ok, evidence_detail = validate_pass_evidence(
        evidence_path, probe_instance_id, args.required_frames,
        args.max_frame_gap_ms,
    )
    _event('PROBE_EVIDENCE', probe_instance_id=probe_instance_id,
           path=evidence_path, valid=evidence_ok, detail=evidence_detail)
    if result == 0 and evidence_ok:
        _event('SUCCESS', classification='ROBOT_CONTROL_PLANE_READY')
        return 0
    classification = 'ROBOT_CONTROL_PLANE_NOT_READY'
    if result != 0 and evidence_ok:
        classification = 'CONTROL_PLANE_PROBE_TEARDOWN_FAILURE'
    _event('FAILED', classification=classification,
           probe_returncode=result, evidence=evidence_detail)
    return result if result else 1


if __name__ == '__main__':
    sys.exit(main())
