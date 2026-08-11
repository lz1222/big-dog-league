"""零运动 status audit 判定的纯单元测试。"""

import importlib.util
import sys
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SCRIPTS_DIR = PACKAGE_ROOT / 'scripts'
sys.path.insert(0, str(SCRIPTS_DIR))
AUDIT_PATH = SCRIPTS_DIR / 'sdk_motion_status_audit.py'


def _load_audit_module():
    spec = importlib.util.spec_from_file_location(
        'sdk_motion_status_audit', str(AUDIT_PATH)
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def status(sequence, event, ret=0):
    return {
        'server_instance_id': 'instance-a',
        'sequence': sequence,
        'event': event,
        'ret': ret,
        'reason': (
            'verification_source='
            'validated_sequence_current_cpp_pre_stop_speed_classic_settle_v1'
            if event == 'CLASSIC_VERIFIED' else 'test'
        ),
        'vx': 0.0,
        'vy': 0.0,
        'yaw': 0.0,
        'server_monotonic_ns': sequence * 100,
        'receive_monotonic_ns': sequence * 200,
        'status_age_sec': 0.0,
    }


def test_audit_passes_only_startup_then_verified_classic_then_zero_stop():
    audit = _load_audit_module()
    result = audit.summarize([
        status(1, 'STARTUP_STOP'), status(2, 'CLASSIC_VERIFIED'),
        status(3, 'STOP_MOVE')
    ], 'instance-a')
    assert result['success'] is True
    assert result['startup_sequence'] == 1
    assert result['classic_verified_sequence'] == 2
    assert result['zero_sequence'] == 3
    assert result['move_count'] == 0


def test_audit_rejects_move_error_or_missing_zero_ack():
    audit = _load_audit_module()
    for statuses in (
        [status(1, 'STARTUP_STOP'), status(2, 'CLASSIC_VERIFIED'),
         status(3, 'MOVE')],
        [status(1, 'STARTUP_STOP'), status(2, 'CLASSIC_VERIFIED'),
         status(3, 'SDK_ERROR', -1)],
        [status(1, 'STARTUP_STOP'), status(2, 'STOP_MOVE')],
    ):
        assert audit.summarize(statuses, 'instance-a')['success'] is False
