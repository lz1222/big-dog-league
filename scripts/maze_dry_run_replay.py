#!/usr/bin/env python3
"""Dry-run rosbag replay exercising rk_maze LiDAR modules against recorded data.

Reads a rosbag db3 directly via sqlite3, processes each PointCloud2 frame through:
  1. Point cloud filtering (NaN/Inf, range, ground, self, height)
  2. Effective obstacle clustering
  3. Hard distance computation (all 8 sectors)
  4. Odom motion compensation buffer
  5. Compares against B2.1-A known geometry

No motion commands. Dry run only.
"""

import argparse
import json
import math
import sqlite3
import statistics
import sys
from pathlib import Path

# Add source tree
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src' / 'rk_maze'))

from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from nav_msgs.msg import Odometry

from rk_maze.lidar_distance_core import (
    Point3D, LidarDistanceConfig, NavigationDistanceFilter,
    filter_point_cloud, voxel_downsample, find_effective_clusters,
    compute_all_hard_distances,
    SECTOR_FRONT, SECTOR_LEFT, SECTOR_RIGHT, SECTOR_REAR, ALL_SECTORS,
)
from rk_maze.motion_compensated_cloud import (
    MotionCompensatedCloudBuffer, MotionCompensatedCloudConfig, OdomPose,
)


def quat_to_yaw(x, y, z, w):
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny, cosy)


def process_bag(bag_path: str):
    bag_path = Path(bag_path).expanduser().resolve()
    if not bag_path.is_file():
        print(f"ERROR: bag file not found: {bag_path}")
        return None

    # ---- Config ----
    ld_config = LidarDistanceConfig(
        min_range_m=0.08, max_range_m=3.00,
        ground_z_min_m=-0.30, ground_z_max_m=0.03,
        obstacle_z_min_m=0.03, obstacle_z_max_m=0.80,
        body_x_min_m=-0.40, body_x_max_m=0.40,
        body_y_min_m=-0.18, body_y_max_m=0.18,
        voxel_size_m=0.02, cluster_tolerance_m=0.05,
        min_cluster_points=5, cluster_percentile=0.10,
        footprint_front_m=0.35, footprint_rear_m=0.35,
        footprint_left_m=0.18, footprint_right_m=0.18,
        perception_margin_m=0.03,
        hard_max_age_sec=0.15, nav_max_age_sec=0.50,
        nav_max_jump_m=10.0,  # allow large jumps for bag replay
    )
    mcc_config = MotionCompensatedCloudConfig(
        accumulation_window_sec=0.30,
        odom_max_jump_m=0.30, odom_max_jump_yaw_rad=0.30,
        max_accumulated_displacement_m=10.0,  # stationary bag
        max_accumulated_yaw_rad=10.0,
    )

    # ---- Setup ----
    nav_filter = NavigationDistanceFilter(ld_config)
    cloud_buffer = MotionCompensatedCloudBuffer(mcc_config)

    conn = sqlite3.connect(f'file:{bag_path}?mode=ro', uri=True)
    conn.row_factory = sqlite3.Row

    # Map topic_id -> (name, type)
    topic_map = {}
    for row in conn.execute('SELECT id, name, type FROM topics'):
        topic_map[row['id']] = (row['name'], row['type'])

    print(f"Bag: {bag_path}")
    print(f"Topics: {list(t for _, t in topic_map.values())}")

    # ---- Process messages ----
    stats = {
        'cloud_frames': 0,
        'odom_frames': 0,
        'total_points': 0,
        'filtered_points': 0,
        'clusters_found': 0,
        'cluster_point_total': 0,
        'hard_front_samples': [],
        'hard_left_samples': [],
        'hard_right_samples': [],
        'hard_rear_samples': [],
        'recent_filtered_points': [],  # most recent filtered cloud
    }

    _recent_filtered = []
    last_report = 0.0
    _last_cloud_stamp = 0.0

    cursor = conn.execute('SELECT topic_id, timestamp, data FROM messages ORDER BY id')
    for row in cursor:
        topic_id = row['topic_id']
        if topic_id not in topic_map:
            continue
        topic_name, topic_type = topic_map[topic_id]
        stamp_sec = row['timestamp'] * 1e-9
        data = row['data']

        if topic_name == '/utlidar/cloud_base':
            try:
                cloud_msg = deserialize_message(data, PointCloud2)
            except Exception:
                continue

            points = list(pc2.read_points(cloud_msg, field_names=('x', 'y', 'z'), skip_nans=True))
            cloud = [Point3D(x=float(p[0]), y=float(p[1]), z=float(p[2]))
                     for p in points
                     if math.isfinite(p[0]) and math.isfinite(p[1]) and math.isfinite(p[2])]

            stats['cloud_frames'] += 1
            stats['total_points'] += len(cloud)
            _last_cloud_stamp = stamp_sec

            # F1: Filter
            filtered = filter_point_cloud(cloud, ld_config)
            filtered = voxel_downsample(filtered, ld_config.voxel_size_m)
            stats['filtered_points'] += len(filtered)
            _recent_filtered = filtered

            # F2: Hard distances
            hard = compute_all_hard_distances(filtered, ld_config, stamp_sec, stamp_sec + 0.01)

            # F3: Navigation
            nav_filter.update(hard, stamp_sec)

            # Track front sector
            front_hd = hard.get(SECTOR_FRONT)
            if front_hd and front_hd.valid:
                stats['hard_front_samples'].append({
                    'stamp': stamp_sec,
                    'distance': front_hd.hard_distance,
                    'cluster_points': front_hd.cluster_point_count,
                    'confidence': front_hd.confidence,
                    'spread': front_hd.spread,
                })

            # Track side sectors
            for sector, slist in [
                (SECTOR_LEFT, stats['hard_left_samples']),
                (SECTOR_RIGHT, stats['hard_right_samples']),
                (SECTOR_REAR, stats['hard_rear_samples']),
            ]:
                sd = hard.get(sector)
                if sd and sd.valid:
                    slist.append({
                        'stamp': stamp_sec,
                        'distance': sd.hard_distance,
                    })

            # Cluster stats
            clusters = find_effective_clusters(filtered, ld_config)
            stats['clusters_found'] += len(clusters)
            stats['cluster_point_total'] += sum(c.point_count for c in clusters)

            # Periodic report
            if stamp_sec - last_report > 5.0 and stats['hard_front_samples']:
                last_report = stamp_sec
                last = stats['hard_front_samples'][-1]
                print(f"  t={stamp_sec:.1f}s  front={last['distance']:.3f}m  "
                      f"cluster_pts={last['cluster_points']}  conf={last['confidence']:.2f}  "
                      f"n_clusters={len(clusters)}")

        elif topic_name == '/utlidar/robot_odom':
            try:
                odom_msg = deserialize_message(data, Odometry)
            except Exception:
                continue

            stats['odom_frames'] += 1
            q = odom_msg.pose.pose.orientation
            yaw = quat_to_yaw(q.x, q.y, q.z, q.w)

            pose = OdomPose(
                stamp_sec=stamp_sec,
                x=odom_msg.pose.pose.position.x,
                y=odom_msg.pose.pose.position.y,
                yaw=yaw,
            )
            cloud_buffer.update_odom(pose)

            # Add filtered cloud if we have recent data
            if _recent_filtered and abs(stamp_sec - _last_cloud_stamp) < 0.2:
                cloud_buffer.add_cloud(_recent_filtered, stamp_sec, pose)

    conn.close()
    stats['recent_filtered_points'] = len(_recent_filtered)

    # ---- Summary computation ----
    front_dists = [s['distance'] for s in stats['hard_front_samples']]
    front_confs = [s['confidence'] for s in stats['hard_front_samples']]
    front_cluster_pts = [s['cluster_points'] for s in stats['hard_front_samples']]
    n = len(front_dists)

    summary = {
        'bag_path': str(bag_path),
        'cloud_frames': stats['cloud_frames'],
        'odom_frames': stats['odom_frames'],
        'total_raw_points': stats['total_points'],
        'filtered_points': stats['filtered_points'],
        'filter_ratio': stats['filtered_points'] / max(1, stats['total_points']),
        'total_clusters': stats['clusters_found'],
        'avg_cluster_points': stats['cluster_point_total'] / max(1, stats['clusters_found']),
        'front_hard_stats': {
            'samples': n,
            'min_m': min(front_dists) if n else float('inf'),
            'max_m': max(front_dists) if n else 0.0,
            'median_m': sorted(front_dists)[n // 2] if n else float('inf'),
            'mean_m': sum(front_dists) / n if n else 0.0,
            'std_m': statistics.stdev(front_dists) if n > 1 else 0.0,
            'mean_confidence': sum(front_confs) / n if n else 0.0,
            'mean_cluster_points': sum(front_cluster_pts) / n if n else 0.0,
            'min_cluster_points': min(front_cluster_pts) if n else 0,
            'invalid_frames': stats['cloud_frames'] - n,
        },
        'left_samples': len(stats['hard_left_samples']),
        'right_samples': len(stats['hard_right_samples']),
        'rear_samples': len(stats['hard_rear_samples']),
        'cloud_buffer_clears': cloud_buffer.cleared_count,
        'cloud_buffer_clear_reason': cloud_buffer.last_clear_reason,
        'b2_1_a_comparison': {},
    }

    # ---- B2.1-A comparison ----
    b2 = summary['b2_1_a_comparison']
    b2['right_side_median_distance_m'] = (
        sorted([s['distance'] for s in stats['hard_right_samples']])[
            len(stats['hard_right_samples']) // 2
        ] if stats['hard_right_samples'] else float('inf')
    )
    b2['known_wall_000_lateral_approx_m'] = 0.286
    b2['right_sector_match_wall_000'] = b2['right_side_median_distance_m'] < 0.50
    b2['no_single_point_trigger'] = (
        summary['front_hard_stats']['min_cluster_points'] >= 3
        if n > 0 else True
    )
    b2['no_nan_in_output'] = all(math.isfinite(d) for d in front_dists) if n else True
    b2['rear_coverage_sufficient'] = len(stats['hard_rear_samples']) > stats['cloud_frames'] * 0.1
    b2['front_std_acceptable'] = summary['front_hard_stats']['std_m'] < 0.10

    # Check for the known right-side wall_endpoint static collision issue
    if stats['hard_right_samples']:
        right_dists = [s['distance'] for s in stats['hard_right_samples']]
        right_median = sorted(right_dists)[len(right_dists)//2]
        b2['right_wall_endpoint_match'] = (
            'consistent_with_b2_1_a_wall_000'
            if 0.15 < right_median < 0.50
            else 'outside_expected_range'
        )

    return summary


# Track most recent cloud stamp for buffer


def main():

    parser = argparse.ArgumentParser(description='Dry-run rosbag replay for rk_maze')
    parser.add_argument('bag_path', help='Path to rosbag .db3 file')
    parser.add_argument('--output', '-o', help='JSON output file path')
    args = parser.parse_args()

    summary = process_bag(args.bag_path)
    if summary is None:
        return 1

    # ---- Print ----
    print(f"\n{'='*60}")
    print("DRY RUN SUMMARY")
    print(f"{'='*60}")
    print(f"Cloud frames:         {summary['cloud_frames']}")
    print(f"Odom frames:          {summary['odom_frames']}")
    print(f"Raw points:           {summary['total_raw_points']}")
    print(f"Filtered points:      {summary['filtered_points']} "
          f"({summary['filter_ratio']:.1%})")
    print(f"Clusters per frame:   {summary['total_clusters'] / max(1, summary['cloud_frames']):.1f}")
    print(f"Avg cluster pts:      {summary['avg_cluster_points']:.1f}")

    fs = summary['front_hard_stats']
    print(f"\nFront hard distance:")
    print(f"  Valid samples:      {fs['samples']}/{summary['cloud_frames']}")
    print(f"  Min:                {fs['min_m']:.3f}m")
    print(f"  Median:             {fs['median_m']:.3f}m")
    print(f"  Mean:               {fs['mean_m']:.3f}m")
    print(f"  Std dev:            {fs['std_m']:.3f}m")
    print(f"  Mean confidence:    {fs['mean_confidence']:.3f}")
    print(f"  Mean cluster pts:   {fs['mean_cluster_points']:.1f}")
    print(f"  Min cluster pts:    {fs['min_cluster_points']}")

    print(f"\nOther sectors:")
    print(f"  Left samples:       {summary['left_samples']}")
    print(f"  Right samples:      {summary['right_samples']}")
    print(f"  Rear samples:       {summary['rear_samples']}")
    print(f"  Cloud buf clears:   {summary['cloud_buffer_clears']} "
          f"({summary['cloud_buffer_clear_reason']})")

    print(f"\nB2.1-A Comparison:")
    for k, v in summary['b2_1_a_comparison'].items():
        print(f"  {k}: {v}")

    # ---- Acceptance checks ----
    checks = []
    if fs['invalid_frames'] > fs['samples'] * 0.3:
        checks.append(f"WARN: {fs['invalid_frames']} invalid frames ({fs['invalid_frames']/max(1,summary['cloud_frames']):.1%})")
    if fs['std_m'] > 0.10 and fs['samples'] > 10:
        checks.append(f"WARN: front distance std dev {fs['std_m']:.3f}m > 0.10m")
    if not summary['b2_1_a_comparison'].get('no_single_point_trigger', True):
        checks.append("WARN: single-point min trigger detected")
    if not summary['b2_1_a_comparison'].get('right_wall_endpoint_match') == 'consistent_with_b2_1_a_wall_000':
        checks.append("INFO: right wall match differs from B2.1-A wall_000")

    if checks:
        print(f"\nCHECKS:")
        for c in checks:
            print(f"  {c}")
    else:
        print(f"\nAll checks passed.")

    if args.output:
        with open(args.output, 'w') as f:
            json.dump(summary, f, indent=2, default=str)
        print(f"\nSaved to {args.output}")

    return 0


if __name__ == '__main__':
    sys.exit(main())
