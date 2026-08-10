#!/usr/bin/env python3
"""MAZE_RUNTIME_STABILITY_V1 — S0-S5: 最小传感器只读探针。

仅订阅 /utlidar/cloud_base, /utlidar/robot_odom, /utlidar/imu，
不发布任何运动命令。用于验证传感器数据链和频率。

用法:
  python3 tools/maze_sensor_probe.py [--duration 60] [--domain 0]
"""

import argparse
import math
import os
import signal
import sys
import time
import threading
from dataclasses import dataclass, field
from typing import List, Optional

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Imu, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from nav_msgs.msg import Odometry


# ---------------------------------------------------------------------------
# 线程安全快照
# ---------------------------------------------------------------------------

@dataclass
class CloudSnapshot:
    rx_monotonic: float = 0.0
    header_stamp_sec: float = 0.0
    frame_id: str = ""
    point_count: int = 0
    x_min: float = float('inf')
    x_max: float = float('-inf')
    y_min: float = float('inf')
    y_max: float = float('-inf')
    z_min: float = float('inf')
    z_max: float = float('-inf')
    max_gap_sec: float = 0.0
    valid: bool = False


@dataclass
class OdomSnapshot:
    rx_monotonic: float = 0.0
    header_stamp_sec: float = 0.0
    yaw: float = 0.0
    max_gap_sec: float = 0.0
    valid: bool = False


@dataclass
class ImuSnapshot:
    rx_monotonic: float = 0.0
    header_stamp_sec: float = 0.0
    wz: float = 0.0
    max_gap_sec: float = 0.0
    valid: bool = False


# ---------------------------------------------------------------------------
# 探针节点
# ---------------------------------------------------------------------------

class SensorProbeNode(Node):
    """只读传感器探针。"""

    def __init__(self, cloud_topic: str, odom_topic: str, imu_topic: str):
        super().__init__('maze_sensor_probe')

        # 传感器快照（锁保护）
        self._lock = threading.Lock()
        self.cloud = CloudSnapshot()
        self.odom = OdomSnapshot()
        self.imu = ImuSnapshot()

        # 统计计数器
        self.cloud_seq = 0
        self.odom_seq = 0
        self.imu_seq = 0

        # QoS: 尝试 BEST_EFFORT 匹配传感器发布者
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )

        self.cloud_sub = self.create_subscription(
            PointCloud2, cloud_topic,
            self._on_cloud, sensor_qos,
        )
        self.odom_sub = self.create_subscription(
            Odometry, odom_topic,
            self._on_odom, sensor_qos,
        )
        self.imu_sub = self.create_subscription(
            Imu, imu_topic,
            self._on_imu, sensor_qos,
        )

        self.get_logger().info(
            f'Probe subscribing: {cloud_topic}, {odom_topic}, {imu_topic}'
        )

    # ---- callbacks (轻量) ----

    def _on_cloud(self, msg: PointCloud2):
        now = time.monotonic()
        pts = list(pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True))
        with self._lock:
            self.cloud_seq += 1
            s = self.cloud
            if s.rx_monotonic > 0.0:
                s.max_gap_sec = max(s.max_gap_sec, now - s.rx_monotonic)
            s.rx_monotonic = now
            s.header_stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            s.frame_id = msg.header.frame_id
            s.point_count = len(pts)
            if pts:
                s.x_min = min(p[0] for p in pts)
                s.x_max = max(p[0] for p in pts)
                s.y_min = min(p[1] for p in pts)
                s.y_max = max(p[1] for p in pts)
                s.z_min = min(p[2] for p in pts)
                s.z_max = max(p[2] for p in pts)
            s.valid = len(pts) > 0

    def _on_odom(self, msg: Odometry):
        now = time.monotonic()
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        with self._lock:
            self.odom_seq += 1
            if self.odom.rx_monotonic > 0.0:
                self.odom.max_gap_sec = max(
                    self.odom.max_gap_sec, now - self.odom.rx_monotonic)
            self.odom.rx_monotonic = now
            self.odom.header_stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.odom.yaw = yaw
            self.odom.valid = True

    def _on_imu(self, msg: Imu):
        now = time.monotonic()
        with self._lock:
            self.imu_seq += 1
            if self.imu.rx_monotonic > 0.0:
                self.imu.max_gap_sec = max(
                    self.imu.max_gap_sec, now - self.imu.rx_monotonic)
            self.imu.rx_monotonic = now
            self.imu.header_stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.imu.wz = msg.angular_velocity.z
            self.imu.valid = True

    # ---- 快照读取 ----

    def snapshot(self):
        with self._lock:
            return (
                CloudSnapshot(
                    rx_monotonic=self.cloud.rx_monotonic,
                    header_stamp_sec=self.cloud.header_stamp_sec,
                    frame_id=self.cloud.frame_id,
                    point_count=self.cloud.point_count,
                    x_min=self.cloud.x_min,
                    x_max=self.cloud.x_max,
                    y_min=self.cloud.y_min,
                    y_max=self.cloud.y_max,
                    z_min=self.cloud.z_min,
                    z_max=self.cloud.z_max,
                    max_gap_sec=self.cloud.max_gap_sec,
                    valid=self.cloud.valid,
                ),
                OdomSnapshot(
                    rx_monotonic=self.odom.rx_monotonic,
                    header_stamp_sec=self.odom.header_stamp_sec,
                    yaw=self.odom.yaw,
                    max_gap_sec=self.odom.max_gap_sec,
                    valid=self.odom.valid,
                ),
                ImuSnapshot(
                    rx_monotonic=self.imu.rx_monotonic,
                    header_stamp_sec=self.imu.header_stamp_sec,
                    wz=self.imu.wz,
                    max_gap_sec=self.imu.max_gap_sec,
                    valid=self.imu.valid,
                ),
            )

    def get_counts(self):
        with self._lock:
            return self.cloud_seq, self.odom_seq, self.imu_seq


# ---------------------------------------------------------------------------
# 主程序
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description='MAZE_RUNTIME_STABILITY_V1 传感器探针')
    parser.add_argument('--duration', type=float, default=60.0, help='运行时长(秒)')
    parser.add_argument('--domain', type=int, default=0, help='ROS_DOMAIN_ID')
    parser.add_argument('--cloud-topic', default='/utlidar/cloud_base')
    parser.add_argument('--odom-topic', default='/utlidar/robot_odom')
    parser.add_argument('--imu-topic', default='/utlidar/imu')
    parser.add_argument(
        '--max-cloud-gap', type=float, default=1.0,
        help='验收允许的最大 cloud 回调间隔(秒)；超过则报告 FAIL',
    )
    args = parser.parse_args()
    if args.max_cloud_gap <= 0.0 or not math.isfinite(args.max_cloud_gap):
        parser.error('--max-cloud-gap 必须为正有限数')

    # 设置环境
    os.environ.setdefault('ROS_DOMAIN_ID', str(args.domain))
    os.environ.setdefault('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')

    rclpy.init()
    node = SensorProbeNode(args.cloud_topic, args.odom_topic, args.imu_topic)
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)

    # 后台 spin
    spin_thread = threading.Thread(target=lambda: executor.spin(), daemon=True)
    spin_thread.start()

    t0 = time.monotonic()
    last_report = t0
    report_interval = 1.0

    # 累计统计
    cloud_last_count = 0
    odom_last_count = 0
    imu_last_count = 0

    header_printed = False

    print(f"\n{'='*70}")
    print(f"MAZE_RUNTIME_STABILITY_V1 — 传感器探针")
    print(f"  Cloud : {args.cloud_topic}")
    print(f"  Odom  : {args.odom_topic}")
    print(f"  IMU   : {args.imu_topic}")
    print(f"  Domain: {args.domain}")
    print(f"  Duration: {args.duration}s")
    print(f"{'='*70}\n")

    try:
        while time.monotonic() - t0 < args.duration:
            time.sleep(0.1)

            now = time.monotonic()
            if now - last_report < report_interval:
                continue
            last_report = now

            cloud, odom, imu = node.snapshot()
            cc, oc, ic = node.get_counts()

            # 频率
            cloud_hz = (cc - cloud_last_count) / report_interval
            odom_hz = (oc - odom_last_count) / report_interval
            imu_hz = (ic - imu_last_count) / report_interval
            cloud_last_count, odom_last_count, imu_last_count = cc, oc, ic

            # 数据年龄
            cloud_age = now - cloud.rx_monotonic if cloud.valid else float('inf')
            odom_age = now - odom.rx_monotonic if odom.valid else float('inf')
            imu_age = now - imu.rx_monotonic if imu.valid else float('inf')

            # 表头
            if not header_printed:
                print(f"{'TIME':>8s} | {'CLOUD_HZ':>8s} {'AGE':>6s} {'PTS':>6s} {'FRAME':>12s} | "
                      f"{'ODOM_HZ':>8s} {'AGE':>6s} {'YAW':>8s} | "
                      f"{'IMU_HZ':>8s} {'AGE':>6s} {'WZ':>8s}")
                header_printed = True

            elapsed = now - t0

            # 状态标记
            cloud_flag = "OK" if cloud.valid and cloud_age < 2.0 else ("STALE" if cloud.valid else "---")
            odom_flag = "OK" if odom.valid and odom_age < 2.0 else ("STALE" if odom.valid else "---")
            imu_flag = "OK" if imu.valid and imu_age < 2.0 else ("STALE" if imu.valid else "---")

            print(f"{elapsed:7.1f}s | {cloud_hz:7.1f}Hz {cloud_age:5.1f}s {cloud.point_count:5d} "
                  f"{cloud_flag:>4s}/{cloud.frame_id:<8s} | "
                  f"{odom_hz:7.1f}Hz {odom_age:5.1f}s {math.degrees(odom.yaw):+7.1f}d "
                  f"{odom_flag:>5s} | "
                  f"{imu_hz:7.1f}Hz {imu_age:5.1f}s {imu.wz:+7.3f} "
                  f"{imu_flag:>5s}")

            # 点云详情（每5秒）
            if cloud.valid and int(elapsed) % 5 == 0 and int(elapsed) != int(elapsed - report_interval):
                print(f"         | Cloud range: x=[{cloud.x_min:.2f}..{cloud.x_max:.2f}] "
                      f"y=[{cloud.y_min:.2f}..{cloud.y_max:.2f}] "
                      f"z=[{cloud.z_min:.2f}..{cloud.z_max:.2f}]")

    except KeyboardInterrupt:
        print("\n中断。")

    finally:
        elapsed = time.monotonic() - t0
        cc, oc, ic = node.get_counts()
        print(f"\n{'='*70}")
        print(f"结果 ({elapsed:.1f}s):")
        print(f"  Cloud:  {cc} msgs,  avg {cc/max(elapsed,0.1):.1f} Hz,  "
              f"last_age={cloud_age:.1f}s  max_gap={cloud.max_gap_sec:.2f}s  "
              f"status={'OK' if cloud.valid else 'NO_DATA'}")
        print(f"  Odom:   {oc} msgs,  avg {oc/max(elapsed,0.1):.1f} Hz,  "
              f"last_age={odom_age:.1f}s  max_gap={odom.max_gap_sec:.2f}s  "
              f"status={'OK' if odom.valid else 'NO_DATA'}")
        print(f"  IMU:    {ic} msgs,  avg {ic/max(elapsed,0.1):.1f} Hz,  "
              f"last_age={imu_age:.1f}s  max_gap={imu.max_gap_sec:.2f}s  "
              f"status={'OK' if imu.valid else 'NO_DATA'}")
        print(f"\n验收: ", end="")
        all_ok = cloud.valid and odom.valid and imu.valid and \
            cloud.max_gap_sec <= args.max_cloud_gap
        if all_ok:
            print("PASS")
        elif not (cloud.valid and odom.valid and imu.valid):
            print("FAIL — 传感器数据链未建立")
        else:
            print(f"FAIL — cloud max_gap={cloud.max_gap_sec:.2f}s 超过 "
                  f"{args.max_cloud_gap:.2f}s 验收门限")
        print(f"{'='*70}\n")

        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
