"""动态 validation adapter/recorder 进程隔离的静态合同。"""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
ADAPTER = ROOT / 'rk_bringup' / 'validation_motion_adapter_node.py'
RECORDER = ROOT / 'rk_bringup' / 'validation_dynamic_recorder_node.py'
LEGACY = ROOT / 'scripts' / 'validation_dynamic_run_once.py'


def test_adapter_has_no_file_or_white_bar_path():
    source = ADAPTER.read_text()
    assert 'open(' not in source
    assert 'flush(' not in source
    assert 'fsync(' not in source
    assert 'white_bar' not in source
    assert 'BoundedJsonlWriter' not in source


def test_adapter_uses_latest_state_depth_one():
    source = ADAPTER.read_text()
    assert 'depth=1' in source
    assert '/perception/line_track' in source
    assert '/navigation/line_follow_cmd_suggested' in source


def test_recorder_owns_all_persistence_operations():
    node_source = RECORDER.read_text()
    core_source = (
        ROOT / 'rk_bringup' / 'validation_recorder_core.py'
    ).read_text()
    assert 'BoundedJsonlWriter' in node_source
    assert 'put_nowait' in core_source
    assert 'os.fsync' in core_source
    assert 'RECORDER_BACKPRESSURE' in core_source


def test_legacy_entry_only_delegates_to_lightweight_adapter():
    source = LEGACY.read_text()
    assert 'validation_motion_adapter_node import main' in source
    assert 'SingleDynamicRun' not in source
    assert 'record_file' not in source
