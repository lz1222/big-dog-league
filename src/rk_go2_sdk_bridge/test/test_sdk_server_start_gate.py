"""SDK server 启动门的纯合同回归；不启动 ROS 或 SDK 子进程。"""

import importlib.util
import json
import sys
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SCRIPTS_DIR = PACKAGE_ROOT / 'scripts'
sys.path.insert(0, str(SCRIPTS_DIR))
GATE_PATH = SCRIPTS_DIR / 'sdk_server_start_gate.py'


def _load_gate_module():
    spec = importlib.util.spec_from_file_location(
        'sdk_server_start_gate', str(GATE_PATH)
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _ready_payload(**overrides):
    value = {
        'ready': True,
        'receiver_instance_id': 'receiver-current',
        'expected_server_instance_id': 'server-current',
        'status_ip': '127.0.0.1',
        'status_port': 15002,
        'ready_monotonic_ns': 123456789,
    }
    value.update(overrides)
    return json.dumps(value)


def _contract(module):
    return module.ReceiverStartupContract(
        'server-current', '127.0.0.1', 15002
    )


def test_server_start_refused_until_receiver_ready():
    """CASE A/F：无 receiver-ready（包含 receiver 崩溃）绝不放行 server。"""
    gate = _load_gate_module()
    contract = _contract(gate)
    assert contract.ready is None
    assert contract.observe('not-json') is False
    assert contract.ready is None


def test_current_receiver_ready_allows_exactly_current_server():
    """CASE B：当前 nonce、地址和端口完全匹配才可放行。"""
    gate = _load_gate_module()
    contract = _contract(gate)
    assert contract.observe(_ready_payload()) is True
    assert contract.ready['receiver_instance_id'] == 'receiver-current'
    assert contract.ready['expected_server_instance_id'] == 'server-current'


def test_wrong_or_stale_receiver_ready_fails_closed():
    """CASE C/D：上一轮 nonce、错误 bind 或失效 ready 都不能启动 server。"""
    gate = _load_gate_module()
    for payload in (
            _ready_payload(expected_server_instance_id='server-previous'),
            _ready_payload(status_port=16002),
            _ready_payload(ready=False),
            _ready_payload(ready_monotonic_ns=0)):
        contract = _contract(gate)
        assert contract.observe(payload) is False
        assert contract.ready is None


def test_gate_source_spawns_server_only_after_receiver_contract():
    """CASE A/B：spawn 位于 ready 超时拒绝之后，gate 自身没有控制 publisher。"""
    source = GATE_PATH.read_text(encoding='utf-8')
    assert source.index('if ready is None:') < source.index('subprocess.Popen')
    for forbidden in ('create_publisher', '.Move(', 'ClassicWalk'):
        assert forbidden not in source
