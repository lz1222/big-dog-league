#!/usr/bin/env python3
"""迷宫 P0 只读诊断：统一验证 DDS 域、动态库、发现和回调。

该工具绝不创建 Twist Publisher、Timer 或 Unitree SDK Client；它仅订阅三路
传感器并输出证据。六个阶段由轻到重增加依赖，用于定位复杂 maze 节点与最小
订阅节点之间的首个差异。
"""

import argparse
import importlib
import math
import os
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, PointCloud2


TOPICS = {
    'cloud': '/utlidar/cloud_base',
    'odom': '/utlidar/robot_odom',
    'imu': '/utlidar/imu',
}


@dataclass
class Counts:
    """仅记录回调次数，避免诊断代码自身成为点云处理瓶颈。"""

    cloud: int = 0
    odom: int = 0
    imu: int = 0

    def total(self) -> int:
        return self.cloud + self.odom + self.imu


class ReadOnlySubscriptionNode(Node):
    """三路传感器的最小只读订阅拓扑，不含任何运动输出。"""

    def __init__(self):
        super().__init__('maze_p0_diagnose')
        self._lock = threading.Lock()
        self._counts = Counts()
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )
        self.create_subscription(PointCloud2, TOPICS['cloud'], self._on_cloud, sensor_qos)
        self.create_subscription(Odometry, TOPICS['odom'], self._on_odom, sensor_qos)
        self.create_subscription(Imu, TOPICS['imu'], self._on_imu, sensor_qos)

    def _on_cloud(self, _message: PointCloud2):
        with self._lock:
            self._counts.cloud += 1

    def _on_odom(self, _message: Odometry):
        with self._lock:
            self._counts.odom += 1

    def _on_imu(self, _message: Imu):
        with self._lock:
            self._counts.imu += 1

    def counts(self) -> Counts:
        with self._lock:
            return Counts(**vars(self._counts))

    def publisher_summary(self) -> Dict[str, List[str]]:
        """读取发现图中的发布者 QoS，确认匹配不依赖猜测。"""
        result: Dict[str, List[str]] = {}
        for label, topic in TOPICS.items():
            entries = []
            for info in self.get_publishers_info_by_topic(topic):
                entries.append(
                    '{} qos(reliability={}, durability={}, depth={})'.format(
                        info.node_name,
                        info.qos_profile.reliability.name,
                        info.qos_profile.durability.name,
                        info.qos_profile.depth,
                    )
                )
            result[label] = entries
        return result


def _loaded_dds_libraries() -> List[str]:
    """从 Linux loader 映射读取实际库路径，而不是只打印期望环境变量。"""
    maps_path = Path('/proc/self/maps')
    if not maps_path.is_file():
        return []
    libraries = set()
    for line in maps_path.read_text(encoding='utf-8', errors='replace').splitlines():
        path = line.rsplit(maxsplit=1)[-1] if '/' in line else ''
        if '/libddsc.so' in path or '/libddscxx.so' in path:
            libraries.add(path)
    return sorted(libraries)


def _print_environment(expected_domain: int) -> bool:
    """记录影响发现和 ABI 的最小环境契约，域不一致时 fail-closed。"""
    actual_domain = os.environ.get('ROS_DOMAIN_ID', '0')
    print('ENV ROS_DOMAIN_ID={} RMW_IMPLEMENTATION={} CYCLONEDDS_URI={}'.format(
        actual_domain,
        os.environ.get('RMW_IMPLEMENTATION', '<unset>'),
        os.environ.get('CYCLONEDDS_URI', '<unset>'),
    ))
    print('ENV LD_LIBRARY_PATH={}'.format(os.environ.get('LD_LIBRARY_PATH', '<unset>')))
    if actual_domain != str(expected_domain):
        print('FAIL ENV: --domain={} 与进程 ROS_DOMAIN_ID={} 不一致；拒绝诊断。'.format(
            expected_domain, actual_domain))
        return False
    return True


def _run_stage(name: str, node: ReadOnlySubscriptionNode, duration: float) -> Counts:
    """记录阶段增量，并把短暂点云断流与彻底发现失败分开报告。

    Go2 LiDAR 已知可能出现数秒断流，单个 3 秒窗口不能据此断言 DDS 已失效；
    因而阶段内只报告 PASS/PARTIAL/FAIL，最终由全程累计的三路首次回调判定。
    """
    before = node.counts()
    time.sleep(duration)
    after = node.counts()
    delta = Counts(
        cloud=after.cloud - before.cloud,
        odom=after.odom - before.odom,
        imu=after.imu - before.imu,
    )
    if delta.cloud > 0 and delta.odom > 0 and delta.imu > 0:
        status = 'PASS'
    elif delta.total() > 0:
        status = 'PARTIAL'
    else:
        status = 'FAIL'
    print('{} {} cloud={} odom={} imu={}'.format(
        status, name, delta.cloud, delta.odom, delta.imu))
    return delta


def _load_maze_modules() -> None:
    """按 v2 的依赖顺序加载，不创建控制器或任何 ROS publisher。"""
    workspace = Path(__file__).resolve().parents[1]
    maze_python_root = workspace / 'src' / 'rk_maze'
    if str(maze_python_root) not in sys.path:
        sys.path.insert(0, str(maze_python_root))
    importlib.import_module('rk_maze.lidar_distance_core')
    importlib.import_module('rk_maze.lidar_wall_extractor')
    importlib.import_module('rk_maze.local_occupancy_grid')
    importlib.import_module('rk_maze.heading_controller')


def main() -> int:
    parser = argparse.ArgumentParser(description='Maze P0 read-only DDS diagnostic')
    parser.add_argument('--domain', type=int, default=int(os.environ.get('ROS_DOMAIN_ID', '0')))
    parser.add_argument('--stage-seconds', type=float, default=3.0)
    args = parser.parse_args()
    if args.stage_seconds <= 0.0 or not math.isfinite(args.stage_seconds):
        parser.error('--stage-seconds 必须为正有限数')
    if not _print_environment(args.domain):
        return 2

    rclpy.init()
    node = ReadOnlySubscriptionNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    success = True
    observed = Counts()
    try:
        # S1: 纯订阅；S2-S5 逐级增加与 v2 相同的 Python 依赖或后台线程。
        stage = _run_stage('S1_MINIMAL_SUBSCRIPTIONS', node, args.stage_seconds)
        observed.cloud += stage.cloud
        observed.odom += stage.odom
        observed.imu += stage.imu
        _load_maze_modules()
        stage = _run_stage('S2_RK_MAZE_IMPORTS', node, args.stage_seconds)
        observed.cloud += stage.cloud
        observed.odom += stage.odom
        observed.imu += stage.imu

        from rk_maze.heading_controller import HeadingController, HeadingControllerConfig
        from rk_maze.lidar_wall_extractor import LidarWallExtractor
        from rk_maze.local_occupancy_grid import LocalGridConfig, LocalOccupancyGrid

        LidarWallExtractor(LocalGridConfig())
        stage = _run_stage('S3_WALL_EXTRACTOR', node, args.stage_seconds)
        observed.cloud += stage.cloud
        observed.odom += stage.odom
        observed.imu += stage.imu
        LocalOccupancyGrid(LocalGridConfig())
        stage = _run_stage('S4_OCCUPANCY_GRID', node, args.stage_seconds)
        observed.cloud += stage.cloud
        observed.odom += stage.odom
        observed.imu += stage.imu
        HeadingController(HeadingControllerConfig())
        worker_alive = threading.Event()
        worker = threading.Thread(target=lambda: (worker_alive.set(), time.sleep(args.stage_seconds)), daemon=True)
        worker.start()
        success &= worker_alive.wait(timeout=1.0)
        stage = _run_stage('S5_BACKGROUND_THREAD', node, args.stage_seconds)
        observed.cloud += stage.cloud
        observed.odom += stage.odom
        observed.imu += stage.imu
        stage = _run_stage('S6_V2_DEPENDENCY_TOPOLOGY', node, args.stage_seconds)
        observed.cloud += stage.cloud
        observed.odom += stage.odom
        observed.imu += stage.imu

        print('DISCOVERY')
        for label, entries in node.publisher_summary().items():
            print('  {}: {}'.format(label, entries or ['<none>']))
        libraries = _loaded_dds_libraries()
        print('LOADED_DDS {}'.format(libraries or ['<not found in /proc/self/maps>']))
        all_sensors_seen = observed.cloud > 0 and observed.odom > 0 and observed.imu > 0
        print('OBSERVED_TOTAL cloud={} odom={} imu={}'.format(
            observed.cloud, observed.odom, observed.imu))
        if not libraries or not all_sensors_seen:
            success = False
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
    print('RESULT {}'.format('PASS' if success else 'FAIL'))
    return 0 if success else 1


if __name__ == '__main__':
    raise SystemExit(main())
