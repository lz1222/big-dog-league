#!/usr/bin/env python3
"""Go2 SDK motion-status UDP 协议的纯 Python 校验器。"""

import json
import math
import time


VALID_EVENTS = frozenset((
    'CLASSIC_VERIFIED', 'STARTUP_STOP', 'MOVE', 'STOP_MOVE', 'SDK_ERROR',
    'GAIT_REQUESTED', 'GAIT_READY', 'GAIT_FAILED',
    'ACTION_REQUESTED', 'ACTION_READY', 'ACTION_FAILED',
))
REQUIRED_FIELDS = frozenset((
    'server_instance_id', 'sequence', 'event', 'ret', 'reason', 'vx', 'vy', 'yaw',
    'server_monotonic_ns',
))


def _valid_int(value, minimum=None):
    """拒绝 bool，避免 JSON true 被误认为 sequence 或 SDK ret。"""
    return isinstance(value, int) and not isinstance(value, bool) and (
        minimum is None or value >= minimum
    )


def _valid_number(value):
    """状态协议不接受 NaN/Inf，避免下游 watchdog 出现不可比较时间。"""
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(float(value))
    )


def decode_status_datagram(payload, receive_monotonic_ns=None):
    """解析 server 在真实 SDK 调用后发送的单个 JSON 状态包。"""
    if isinstance(payload, bytes):
        try:
            payload = payload.decode('utf-8', errors='strict')
        except UnicodeDecodeError:
            return None
    if not isinstance(payload, str):
        return None
    try:
        status = json.loads(payload)
    except (TypeError, ValueError):
        return None
    if not isinstance(status, dict) or not REQUIRED_FIELDS.issubset(status):
        return None
    if (
        not isinstance(status['server_instance_id'], str)
        or not status['server_instance_id']
        or len(status['server_instance_id']) > 128
        or not _valid_int(status['sequence'], 1)
        or status['event'] not in VALID_EVENTS
        or not _valid_int(status['ret'])
        or not isinstance(status['reason'], str)
        or not status['reason']
        or not _valid_int(status['server_monotonic_ns'], 1)
        or not all(_valid_number(status[name]) for name in ('vx', 'vy', 'yaw'))
    ):
        return None
    forwarded_receive_ns = status.get('receive_monotonic_ns')
    if receive_monotonic_ns is None and _valid_int(forwarded_receive_ns, 1):
        receive_monotonic_ns = forwarded_receive_ns
    normalized = {
        'server_instance_id': status['server_instance_id'],
        'sequence': int(status['sequence']),
        'event': status['event'],
        'ret': int(status['ret']),
        'reason': status['reason'],
        'vx': float(status['vx']),
        'vy': float(status['vy']),
        'yaw': float(status['yaw']),
        'server_monotonic_ns': int(status['server_monotonic_ns']),
        'receive_monotonic_ns': int(
            time.monotonic_ns()
            if receive_monotonic_ns is None else receive_monotonic_ns
        ),
        # 刚收到时 age 为零；watchdog 应使用 receive_monotonic_ns 实时重算。
        'status_age_sec': 0.0,
    }
    return normalized


class SdkMotionStatusTracker:
    """过滤重放/乱序 UDP 状态，给 watchdog 提供本机接收时刻。"""

    def __init__(self, expected_server_instance_id=''):
        self.expected_server_instance_id = str(
            expected_server_instance_id
        ).strip()
        self._last_sequence_by_instance = {}

    @property
    def last_sequence(self):
        """保留单实例调用方的观测接口；多实例时返回全局最大序号。"""
        if self.expected_server_instance_id:
            return self._last_sequence_by_instance.get(
                self.expected_server_instance_id, 0
            )
        return max(self._last_sequence_by_instance.values(), default=0)

    def observe(self, payload, receive_monotonic_ns=None):
        """只接受严格递增 sequence；旧 ACK 不能触发下一轮 T0。"""
        status = decode_status_datagram(payload, receive_monotonic_ns)
        if status is None:
            return None
        instance_id = status['server_instance_id']
        if (
            self.expected_server_instance_id
            and instance_id != self.expected_server_instance_id
        ):
            return None
        last_sequence = self._last_sequence_by_instance.get(instance_id, 0)
        if status['sequence'] <= last_sequence:
            return None
        self._last_sequence_by_instance[instance_id] = status['sequence']
        return status

    @staticmethod
    def status_age_sec(status, now_monotonic_ns=None):
        """以本机接收时间计算 freshness，避免跨进程时钟不可直接比较。"""
        now = time.monotonic_ns() if now_monotonic_ns is None else now_monotonic_ns
        return max(0.0, (int(now) - status['receive_monotonic_ns']) / 1e9)


class SdkMotionStatusReplayBuffer:
    """保留当前实例的有界 ACK 历史，供晚发现的 ROS 订阅者原样重放。"""

    def __init__(self, capacity=100):
        if not isinstance(capacity, int) or isinstance(capacity, bool) \
                or capacity <= 0:
            raise ValueError('capacity must be a positive integer')
        self.capacity = capacity
        self._payloads = []

    def remember(self, status):
        """按到达顺序保存规范 JSON；超限只淘汰最旧状态。"""
        payload = json.dumps(status, separators=(',', ':'), allow_nan=False)
        self._payloads.append(payload)
        if len(self._payloads) > self.capacity:
            del self._payloads[:-self.capacity]
        return payload

    def payloads(self):
        """返回不可变快照，避免发布回调修改正在迭代的历史。"""
        return tuple(self._payloads)
