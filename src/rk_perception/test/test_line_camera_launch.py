from pathlib import Path


LAUNCH_DIR = Path(__file__).resolve().parents[1] / 'launch'


def test_camera_only_launch_contains_only_camera_node():
    source = (LAUNCH_DIR / 'line_camera_only.launch.py').read_text(
        encoding='utf-8'
    )

    assert "executable='usb_line_camera_node'" in source
    assert 'real_line_tracker_node' not in source
    assert 'line_follower' not in source
    assert 'command_mux' not in source
    assert 'cmd_vel_bridge' not in source


def test_camera_tracker_debug_launch_excludes_control_chain():
    source = (LAUNCH_DIR / 'line_camera_perception_debug.launch.py').read_text(
        encoding='utf-8'
    )

    assert "executable='usb_line_camera_node'" in source
    assert "executable='real_line_tracker_node'" in source
    assert 'line_follower' not in source
    assert 'command_mux' not in source
    assert 'cmd_vel_bridge' not in source
    assert 'mission' not in source
    assert 'inspection' not in source
