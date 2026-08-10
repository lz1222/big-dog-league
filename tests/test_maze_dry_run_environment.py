#!/usr/bin/env python3
"""Dry Run 必须与 P0 使用同一 DDS runtime，且不能在脚本内启动运动桥。"""

from pathlib import Path


SOURCE = Path(__file__).resolve().parents[1] / 'scripts' / 'run_maze.sh'


def test_dry_run_reuses_validated_p0_runtime_without_bridge_launch():
    """防止动态库混载导致 DDS 段错误，也防止 Dry Run 扩大为运动测试。"""
    text = SOURCE.read_text(encoding='utf-8')
    assert 'install/rk_go2_sdk_bridge/lib/rk_go2_sdk_bridge/unitree_sdk_runtime' in text
    assert 'go2_sdk_udp_bridge.launch.py' not in text
    assert 'go2_sdk_udp_server' not in text
