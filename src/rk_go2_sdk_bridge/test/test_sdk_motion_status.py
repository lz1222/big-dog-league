"""独立 status side-channel 的协议和 forwarder 侧回归。"""

import json
import math
import sys
from pathlib import Path


SCRIPTS_DIR = Path(__file__).resolve().parents[1] / 'scripts'
sys.path.insert(0, str(SCRIPTS_DIR))

from sdk_motion_status import (
    SdkMotionStatusReplayBuffer,
    SdkMotionStatusTracker,
    decode_status_datagram,
)


def status_payload(sequence=1, **overrides):
    """构造 server 已完成 SDK 调用后的最小合法状态。"""
    payload = {
        'server_instance_id': 'instance-a',
        'sequence': sequence,
        'event': 'MOVE',
        'ret': 0,
        'reason': 'periodic_move',
        'vx': 0.25,
        'vy': 0.0,
        'yaw': 0.03,
        'server_monotonic_ns': 123456789,
    }
    payload.update(overrides)
    return json.dumps(payload).encode('utf-8')


def test_forwarder_decodes_current_sdk_status_and_preserves_receive_time():
    status = decode_status_datagram(status_payload(), 2000000000)
    assert status['event'] == 'MOVE'
    assert status['ret'] == 0
    assert status['receive_monotonic_ns'] == 2000000000
    assert status['status_age_sec'] == 0.0


def test_forwarder_rejects_malformed_or_nonfinite_status():
    assert decode_status_datagram(b'{bad json}') is None
    assert decode_status_datagram(status_payload(vx=math.nan)) is None
    assert decode_status_datagram(status_payload(sequence=True)) is None
    assert decode_status_datagram(status_payload(event='UNKNOWN')) is None


def test_tracker_rejects_replayed_status_and_computes_freshness():
    tracker = SdkMotionStatusTracker()
    first = tracker.observe(status_payload(10), 1_000_000_000)
    assert first is not None
    assert tracker.observe(status_payload(10), 1_100_000_000) is None
    assert tracker.observe(status_payload(9), 1_100_000_000) is None
    second = tracker.observe(status_payload(11), 1_200_000_000)
    assert second is not None
    assert tracker.status_age_sec(second, 1_700_000_000) == 0.5


def test_tracker_accepts_sequence_restart_only_for_new_instance():
    tracker = SdkMotionStatusTracker()
    assert tracker.observe(status_payload(9), 1_000_000_000) is not None
    assert tracker.observe(status_payload(
        1, server_instance_id='instance-b'
    ), 1_100_000_000) is not None
    assert tracker.observe(status_payload(
        1, server_instance_id='instance-b'
    ), 1_200_000_000) is None


def test_tracker_expected_instance_rejects_latched_previous_run():
    tracker = SdkMotionStatusTracker('instance-current')
    assert tracker.observe(status_payload(
        99, server_instance_id='instance-old'
    ), 1_000_000_000) is None
    current = tracker.observe(status_payload(
        1, server_instance_id='instance-current'
    ), 1_100_000_000)
    assert current is not None
    assert tracker.last_sequence == 1


def test_decoder_preserves_forwarder_receive_monotonic_time():
    payload = json.loads(status_payload().decode('utf-8'))
    payload['receive_monotonic_ns'] = 987654321
    status = decode_status_datagram(json.dumps(payload))
    assert status['receive_monotonic_ns'] == 987654321


def test_replay_buffer_preserves_order_receive_time_and_bound():
    """late joiner 重放不能改序号/接收时刻，历史必须有界。"""
    replay = SdkMotionStatusReplayBuffer(capacity=2)
    for sequence in (1, 2, 3):
        status = decode_status_datagram(
            status_payload(sequence), receive_monotonic_ns=sequence * 100
        )
        replay.remember(status)

    payloads = [json.loads(payload) for payload in replay.payloads()]
    assert [payload['sequence'] for payload in payloads] == [2, 3]
    assert [payload['receive_monotonic_ns'] for payload in payloads] == [200, 300]


def test_replay_buffer_rejects_invalid_capacity():
    for capacity in (0, -1, True, 1.5):
        try:
            SdkMotionStatusReplayBuffer(capacity)
        except ValueError:
            continue
        raise AssertionError('invalid capacity accepted: {}'.format(capacity))
