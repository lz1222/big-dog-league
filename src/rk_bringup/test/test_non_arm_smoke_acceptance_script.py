"""Software Smoke cleanup guard 预检的无 ROS 回归测试。"""

import os
from pathlib import Path
import stat
import subprocess

import pytest


WORKSPACE_ROOT = Path(__file__).resolve().parents[3]
SCRIPT_PATH = WORKSPACE_ROOT / 'scripts' / 'accept_non_arm_competition.sh'


def _guard_function_script(tmp_path, check_dir, *, fake_bin=None):
    """从正式脚本提取 guard 函数，在隔离 shell 中测试而不启动 launch。"""
    source = SCRIPT_PATH.read_text(encoding='utf-8')
    start = source.index('prepare_smoke_guard_dir() {')
    end = source.index('\n}\n', start) + 3
    function = source[start:end]
    harness = tmp_path / 'guard_harness.sh'
    harness.write_text(
        '#!/bin/bash\nset -euo pipefail\n'
        'CHECK_DIR="{}"\n'
        'SMOKE_GUARD_DIR="${{CHECK_DIR}}/cleanup_guard_private"\n'
        'SMOKE_GUARD="${{SMOKE_GUARD_DIR}}/front_jump_cleanup_guard.json"\n'
        '{}\n'
        'prepare_smoke_guard_dir\n'.format(check_dir, function),
        encoding='utf-8',
    )
    harness.chmod(0o700)
    env = os.environ.copy()
    if fake_bin is not None:
        env['PATH'] = '{}:{}'.format(fake_bin, env['PATH'])
    return subprocess.run(
        ['bash', str(harness)],
        check=False,
        text=True,
        capture_output=True,
        env=env,
    )


def test_private_guard_is_0700_under_0775_check_dir(tmp_path):
    check_dir = tmp_path / 'check'
    check_dir.mkdir(mode=0o775)
    check_dir.chmod(0o775)

    result = _guard_function_script(tmp_path, check_dir)

    private_dir = check_dir / 'cleanup_guard_private'
    assert result.returncode == 0, result.stderr
    assert private_dir.is_dir() and not private_dir.is_symlink()
    assert stat.S_IMODE(private_dir.stat().st_mode) == 0o700
    assert private_dir.stat().st_uid == os.geteuid()
    guard = private_dir / 'front_jump_cleanup_guard.json'
    assert not guard.exists() and not guard.is_symlink()
    assert (check_dir / 'smoke_cleanup_guard_path.txt').read_text().strip() == str(guard)


def test_symlink_private_guard_directory_fails_closed(tmp_path):
    check_dir = tmp_path / 'check'
    check_dir.mkdir()
    target = tmp_path / 'other'
    target.mkdir()
    (check_dir / 'cleanup_guard_private').symlink_to(target, target_is_directory=True)

    result = _guard_function_script(tmp_path, check_dir)

    assert result.returncode != 0
    assert 'already exists or is a symlink' in result.stderr


def test_chmod_failure_fails_before_launch(tmp_path):
    check_dir = tmp_path / 'check'
    check_dir.mkdir()
    fake_bin = tmp_path / 'fake-bin'
    fake_bin.mkdir()
    fake_chmod = fake_bin / 'chmod'
    fake_chmod.write_text('#!/bin/sh\nexit 1\n', encoding='utf-8')
    fake_chmod.chmod(0o700)

    result = _guard_function_script(tmp_path, check_dir, fake_bin=fake_bin)

    assert result.returncode != 0
    assert 'failed to set private guard directory mode 0700' in result.stderr


def test_owner_mismatch_fails_closed(tmp_path):
    check_dir = tmp_path / 'check'
    check_dir.mkdir()
    fake_bin = tmp_path / 'fake-bin'
    fake_bin.mkdir()
    fake_stat = fake_bin / 'stat'
    fake_stat.write_text(
        '#!/bin/sh\nif [ "$2" = "%u" ]; then echo 0; else echo 700; fi\n',
        encoding='utf-8',
    )
    fake_stat.chmod(0o700)

    result = _guard_function_script(tmp_path, check_dir, fake_bin=fake_bin)

    assert result.returncode != 0
    assert 'private guard directory verification failed' in result.stderr


def test_acceptance_uses_one_preflighted_path_and_stream_final_stop():
    """生产 guard 规则不改；只有 smoke 入参改为私有目录的绝对路径。"""
    source = SCRIPT_PATH.read_text(encoding='utf-8')

    assert 'SMOKE_GUARD_DIR="${CHECK_DIR}/cleanup_guard_private"' in source
    assert 'chmod 0700 "$SMOKE_GUARD_DIR"' in source
    assert "stat -c '%u'" in source
    assert "stat -c '%a'" in source
    assert 'smoke_cleanup_guard_path:="$SMOKE_GUARD"' in source
    assert source.index('prepare_smoke_guard_dir') < source.index(
        'setsid ros2 launch rk_bringup competition_non_arm.launch.py'
    )
    assert 'topic_stream_json_matches "$LINE_COURSE_STREAM" route_phase FINAL_STOP' in source
    assert '"$WHITE_STAGE_PUBLISHER_STREAM" sequence 2' in source


def test_acceptance_uses_native_twist_observer_to_avoid_foxy_discovery_race():
    """Twist 流必须是只读 rclpy 订阅，不能依赖一次性 ros2 CLI 发现。"""
    source = SCRIPT_PATH.read_text(encoding='utf-8')

    assert '"$topic_name" --twist --timeout-sec' in source
    assert 'ros2 topic echo "$topic_name"' not in source
