#!/usr/bin/env python3
"""T0: LiDAR 静态卷尺测距标定工具。

在已知距离处放置挡板，录制 LiDAR 数据，自动计算：
- 距离偏移 (offset)
- 标定系数 (scale)
- 静态标准差
- 有效点数

输出标定参数供 lidar_distance.yaml 使用。

用法:
  python3 tools/lidar_distance_calibration.py --distance 0.50 --label "50cm_front"
  python3 tools/lidar_distance_calibration.py --distance 1.00 --label "100cm_front"
  python3 tools/lidar_distance_calibration.py --analyze  # 分析所有已录数据
"""

import argparse
import json
import math
import sqlite3
import statistics
import sys
import time
from datetime import datetime
from pathlib import Path

# Add source tree
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src' / 'rk_maze'))

from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2

from rk_maze.lidar_distance_core import (
    Point3D, LidarDistanceConfig,
    filter_point_cloud, voxel_downsample,
    compute_hard_distance, find_effective_clusters,
    SECTOR_FRONT,
)


CALIBRATION_DIR = Path.home() / 'rk_inspection_ws' / 'evidence' / 'calibration'


def record_calibration_point(
    distance_m: float,
    label: str,
    duration_sec: float = 15.0,
) -> dict:
    """录制单个标定点并返回 LiDAR 统计。

    需要 Go2 开机、3D SLAM 激活、ROS2 CycloneDDS 可用。
    挡板放在 robot base_link 前方 distance_m 处。
    """
    import rclpy
    from rclpy.node import Node

    rclpy.init()
    node = Node('lidar_calibration')

    # 标定时禁用 body filter — 近距挡板会被 body box 误杀
    config = LidarDistanceConfig(
        body_x_min_m=-0.10, body_x_max_m=0.10,  # 只过滤紧贴传感器的点
        body_y_min_m=-0.08, body_y_max_m=0.08,
        min_cluster_points=3,  # 近距点数可能偏少
    )
    samples = []
    raw_distances = []

    def cloud_callback(msg):
        pts = list(pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True))
        cloud = [Point3D(x=float(p[0]), y=float(p[1]), z=float(p[2]))
                 for p in pts if all(math.isfinite(float(v)) for v in p)]
        # Record raw front-sector distances BEFORE clustering
        for p in cloud:
            if abs(math.degrees(math.atan2(p.y, p.x))) <= 10.0:
                raw_distances.append(p.x)  # x-component for front sector

        filtered = filter_point_cloud(cloud, config)
        filtered = voxel_downsample(filtered, config.voxel_size_m)
        clusters = find_effective_clusters(filtered, config)
        front_pts = [p for p in filtered
                     if abs(math.degrees(math.atan2(p.y, p.x))) <= 10.0]

        if front_pts:
            sd = compute_hard_distance(
                front_pts, SECTOR_FRONT, config,
                msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                time.time(),
            )
            if sd.valid:
                samples.append({
                    'distance': sd.hard_distance,
                    'cluster_pts': sd.cluster_point_count,
                    'confidence': sd.confidence,
                    'spread': sd.spread,
                    'timestamp': time.time(),
                })

    node.create_subscription(
        PointCloud2, '/utlidar/cloud_base', cloud_callback, 10,
    )

    print(f'Recording {label}: target={distance_m:.3f}m, {duration_sec}s...')
    print('  Keep the board STILL during recording.')

    start = time.time()
    while time.time() - start < duration_sec:
        rclpy.spin_once(node, timeout_sec=0.05)

    node.destroy_node()
    rclpy.shutdown()

    if not samples:
        return {'error': 'no LiDAR data received', 'target_m': distance_m}

    dists = [s['distance'] for s in samples]
    median = sorted(dists)[len(dists) // 2]
    mean = sum(dists) / len(dists)
    std = statistics.stdev(dists) if len(dists) > 1 else 0.0
    offset = mean - distance_m

    result = {
        'label': label,
        'timestamp': datetime.now().isoformat(),
        'target_distance_m': distance_m,
        'sample_count': len(samples),
        'liDAR_median_m': round(median, 4),
        'liDAR_mean_m': round(mean, 4),
        'liDAR_std_m': round(std, 4),
        'offset_m': round(offset, 4),
        'offset_pct': round(offset / distance_m * 100, 1) if distance_m > 0 else 0,
        'raw_front_count': len(raw_distances),
        'raw_front_median_m': round(
            sorted(raw_distances)[len(raw_distances) // 2], 4
        ) if raw_distances else None,
        'min_cluster_pts': min(s['cluster_pts'] for s in samples) if samples else 0,
        'mean_confidence': sum(s['confidence'] for s in samples) / len(samples) if samples else 0,
        'accept': abs(offset) < 0.15,
    }

    # Save individual sample
    CALIBRATION_DIR.mkdir(parents=True, exist_ok=True)
    json_path = CALIBRATION_DIR / f'calib_{label}_{datetime.now().strftime("%Y%m%d_%H%M%S")}.json'
    with open(json_path, 'w') as f:
        json.dump(result, f, indent=2)

    return result


def analyze_calibration() -> dict:
    """分析所有已录标定点，计算全局标定参数。"""
    if not CALIBRATION_DIR.exists():
        return {'error': 'no calibration data found'}

    all_points = []
    for f in sorted(CALIBRATION_DIR.glob('calib_*.json')):
        data = json.loads(f.read_text())
        if 'error' not in data and data.get('sample_count', 0) > 5:
            all_points.append(data)

    if len(all_points) < 2:
        return {'error': f'need >=2 calibration points, got {len(all_points)}'}

    print(f'\n{"="*60}')
    print(f'T0 LiDAR 标定分析 ({len(all_points)} 个标定点)')
    print(f'{"="*60}')
    print(f'{"距离":>8s}  {"LiDAR中值":>10s}  {"偏移":>8s}  {"偏移%":>7s}  {"σ":>6s}  {"点数":>5s}  {"判定":>5s}')
    print(f'{"-"*60}')

    targets = []
    lidar_vals = []
    for p in all_points:
        t = p['target_distance_m']
        l = p['liDAR_median_m']
        o = p['offset_m']
        op = p['offset_pct']
        s = p['liDAR_std_m']
        n = p['sample_count']
        a = 'OK' if p['accept'] else 'CHECK'
        print(f'{t:8.3f}  {l:10.4f}  {o:+8.4f}  {op:+6.1f}%  {s:5.3f}  {n:5d}  {a:>5s}')
        targets.append(t)
        lidar_vals.append(l)

    # Linear regression: LiDAR = scale * target + offset
    n = len(targets)
    sum_t = sum(targets)
    sum_l = sum(lidar_vals)
    sum_tl = sum(t * l for t, l in zip(targets, lidar_vals))
    sum_tt = sum(t * t for t in targets)

    slope = (n * sum_tl - sum_t * sum_l) / (n * sum_tt - sum_t * sum_t) if (n * sum_tt - sum_t * sum_t) != 0 else 1.0
    intercept = (sum_l - slope * sum_t) / n

    # Per-point residuals
    residuals = [l - (slope * t + intercept) for t, l in zip(targets, lidar_vals)]
    rmse = math.sqrt(sum(r * r for r in residuals) / n)

    summary = {
        'points': len(all_points),
        'distance_range_m': [min(targets), max(targets)],
        'offset_mean_m': round(statistics.mean([p['offset_m'] for p in all_points]), 4),
        'offset_std_m': round(statistics.stdev([p['offset_m'] for p in all_points]), 4) if n > 1 else 0,
        'linear_scale': round(slope, 4),
        'linear_intercept_m': round(intercept, 4),
        'fit_rmse_m': round(rmse, 4),
    }

    print(f'\n{"="*60}')
    print(f'标定结果:')
    print(f'  线性拟合:  LiDAR = {slope:.4f} × 实际 + {intercept:+.4f}m')
    print(f'  拟合 RMSE: {rmse:.4f}m')
    print(f'  平均偏移:  {summary["offset_mean_m"]:+.4f}m ± {summary["offset_std_m"]:.4f}m')
    if abs(intercept) > 0.30:
        print(f'  ⚠️  截距偏大 ({intercept:+.3f}m) — 检查 base_link 原点与 LiDAR 的物理偏移')
    if abs(slope - 1.0) > 0.10:
        print(f'  ⚠️  斜率偏离1.0 ({slope:.3f}) — 检查 LiDAR 距离标度')
    if rmse < 0.03:
        print(f'  ✅  RMSE < 3cm — 达到验收标准')
    elif rmse < 0.05:
        print(f'  ✅  RMSE < 5cm — 可接受')
    else:
        print(f'  ❌  RMSE > 5cm — 需重新标定')
    print(f'{"="*60}')

    # Save summary
    summary_path = CALIBRATION_DIR / 'calibration_summary.json'
    with open(summary_path, 'w') as f:
        json.dump(summary, f, indent=2)
    print(f'\n已保存: {summary_path}')

    return summary


def main():
    parser = argparse.ArgumentParser(description='T0 LiDAR 静态卷尺测距标定')
    parser.add_argument('--distance', '-d', type=float, help='挡板距离 (m)')
    parser.add_argument('--label', '-l', type=str, help='标定点标签')
    parser.add_argument('--duration', type=float, default=15.0, help='录制时长 (秒)')
    parser.add_argument('--analyze', '-a', action='store_true', help='分析所有已录标定点')
    args = parser.parse_args()

    if args.analyze:
        analyze_calibration()
    elif args.distance is not None and args.label is not None:
        result = record_calibration_point(args.distance, args.label, args.duration)
        if 'error' in result:
            print(f'ERROR: {result["error"]}', file=sys.stderr)
            return 1
        print(json.dumps(result, indent=2))
    else:
        parser.print_help()
        print('\n示例:')
        print('  python3 tools/lidar_distance_calibration.py -d 0.50 -l "50cm_front"')
        print('  python3 tools/lidar_distance_calibration.py -d 1.00 -l "100cm_front"')
        print('  python3 tools/lidar_distance_calibration.py --analyze')
        return 1

    return 0


if __name__ == '__main__':
    sys.exit(main())
