"""SDK status ROS 门禁的纯解析回归，不创建控制 publisher。"""

import importlib.util
import json
import sys
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SCRIPTS_DIR = PACKAGE_ROOT / 'scripts'
sys.path.insert(0, str(SCRIPTS_DIR))
GATE_PATH = SCRIPTS_DIR / 'sdk_motion_status_gate.py'


def _load_gate_module():
    spec = importlib.util.spec_from_file_location(
        'sdk_motion_status_gate', str(GATE_PATH)
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def ready_payload(**overrides):
    value = {
        'ready': True,
        'receiver_instance_id': 'receiver-a',
        'expected_server_instance_id': 'server-current',
        'status_ip': '127.0.0.1',
        'status_port': 15002,
        'ready_monotonic_ns': 123456789,
    }
    value.update(overrides)
    return json.dumps(value)


def test_receiver_gate_accepts_exact_current_instance_and_port():
    gate = _load_gate_module()
    value = gate.decode_receiver_ready(
        ready_payload(), 'server-current', '127.0.0.1', 15002
    )
    assert value['receiver_instance_id'] == 'receiver-a'


def test_receiver_gate_rejects_previous_instance_or_wrong_bind():
    gate = _load_gate_module()
    assert gate.decode_receiver_ready(
        ready_payload(), 'server-next', '127.0.0.1', 15002
    ) is None
    assert gate.decode_receiver_ready(
        ready_payload(status_port=16002),
        'server-current', '127.0.0.1', 15002,
    ) is None


def test_gate_source_has_no_control_publish_or_sport_action():
    source = GATE_PATH.read_text(encoding='utf-8')
    for forbidden in (
            'create_publisher', 'StopMove', '.Move(', 'BalanceStand',
            'ros2 topic pub', 'ServiceSwitch', 'SelectMode'):
        assert forbidden not in source
