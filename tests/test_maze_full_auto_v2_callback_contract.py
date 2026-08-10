#!/usr/bin/env python3
"""验证 v2 点云回调不会在高频 ROS executor 中解码整帧。"""

import ast
from pathlib import Path


SOURCE_PATH = Path(__file__).resolve().parents[1] / 'scripts' / 'maze_full_auto_v2.py'


def _method_source(name):
    """返回指定方法源码，用 AST 避免依赖 ROS 节点实例。"""
    source = SOURCE_PATH.read_text(encoding='utf-8')
    tree = ast.parse(source)
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == name:
            return ast.get_source_segment(source, node)
    raise AssertionError('method not found: {}'.format(name))


def test_cloud_callback_only_snapshots_message_and_timestamp():
    """完整点云遍历必须在规划线程执行，避免抢占 Odom/IMU 回调。"""
    callback = _method_source('_cb_cloud')
    planner_decode = _method_source('_decode_cloud_for_plan')

    assert 'pc2.read_points' not in callback
    assert 'self._cloud_message = deepcopy(msg)' in callback
    assert 'pc2.read_points' in planner_decode


def test_expensive_wall_extraction_is_limited_to_new_cloud_frames():
    """20Hz 安全线程不能对同一帧重复执行二次复杂度的墙线提取。"""
    source = SOURCE_PATH.read_text(encoding='utf-8')

    assert 'wall_ransac_sample_limit: int = 40' in source
    assert 'if snap.cloud_seq == self._last_processed_cloud_seq:' in source
    assert "reason='awaiting_next_cloud'" in source


def test_front_cluster_input_has_deterministic_runtime_bound():
    """点云密度升高不能让贪心聚类饿死安全循环。"""
    source = SOURCE_PATH.read_text(encoding='utf-8')

    assert 'front_cluster_max_points: int = 60' in source
    assert 'fp = fp[::stride]' in source


def test_wall_ransac_is_off_by_default_for_runtime_stability():
    """重几何只能显式启用，不能饿死 STALE 与命令看门狗。"""
    source = SOURCE_PATH.read_text(encoding='utf-8')

    assert 'enable_wall_extraction: bool = False' in source
    assert 'if self.cfg.enable_wall_extraction:' in source


def test_recovery_turn_uses_only_the_route_expected_direction():
    """紧急恢复不得因反侧开口而转入错误路线。"""
    source = SOURCE_PATH.read_text(encoding='utf-8')

    assert 'expected = self._expected_turn_direction()' in source
    assert 'self._prepare_turn(expected)' in source
