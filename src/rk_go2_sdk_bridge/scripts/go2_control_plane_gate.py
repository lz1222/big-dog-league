#!/usr/bin/env python3
"""Go2 冷启动控制面只读门禁。

本程序只读取 Linux 链路/路由状态、发送 ICMP echo，并执行只订阅
``rt/sportmodestate`` 的 helper。它从不创建 SportClient，也绝不调用
Move、StopMove、BalanceStand 或任何姿态动作；成功仅代表可以开始启动
SDK UDP server，不能代表机器人已经接收过运动命令。
"""

import argparse
import subprocess
import sys
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


def run_probe(runtime_wrapper, probe, interface, timeout_sec, required_frames,
              max_frame_gap_ms):
    """经 SDK 运行时隔离执行唯一 DDS probe，并把诊断原样写入本次日志。"""
    command = [
        runtime_wrapper, probe, interface, '--gate',
        '--timeout-sec', str(timeout_sec),
        '--required-frames', str(required_frames),
        '--max-frame-gap-ms', str(max_frame_gap_ms),
    ]
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
    result = run_probe(
        args.runtime_wrapper, args.probe, args.interface,
        args.dds_timeout_sec, args.required_frames, args.max_frame_gap_ms,
    )
    if result == 0:
        _event('SUCCESS', classification='ROBOT_CONTROL_PLANE_READY')
        return 0
    _event('FAILED', classification='ROBOT_CONTROL_PLANE_NOT_READY',
           probe_returncode=result)
    return result if result else 1


if __name__ == '__main__':
    sys.exit(main())
