#!/usr/bin/env python3

"""直接只读 Round15 SQLite 片段，生成实际轨迹和候选排名证据。"""

import argparse
from dataclasses import fields
import json
import math
from pathlib import Path
import sqlite3
import statistics
import sys
import time

from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import yaml

from maze_first_turn_core import (
    DynamicFootprint,
    FirstTurnTrajectoryPlanner,
    LocalMapBuilder,
    LocalMapConfig,
    MotionPrimitive,
    PlannerConfig,
    PRIMITIVE_FINE_LEFT_ARC,
    PRIMITIVE_FORWARD,
    PRIMITIVE_LEFT_ARC,
    PRIMITIVE_LEFT_ARC_OUTSIDE,
    PRIMITIVE_OUTSIDE_DIAGONAL,
    PRIMITIVE_REVERSE,
    SafetyContext,
    SECTOR_NAMES,
    STATE_SELECT_TRAJECTORY,
)
from maze_round15_replay_core import (
    ReplayMapSnapshot,
    ReplayOdomSample,
    analyze_round15_actual_trajectory,
)


PRIMITIVE_PREFIXES = {
    PRIMITIVE_FORWARD: 'forward_short',
    PRIMITIVE_OUTSIDE_DIAGONAL: 'outside_diagonal_short',
    PRIMITIVE_LEFT_ARC: 'left_arc',
    PRIMITIVE_LEFT_ARC_OUTSIDE: 'left_arc_outside_vy',
    PRIMITIVE_REVERSE: 'reverse_short',
    PRIMITIVE_FINE_LEFT_ARC: 'fine_left_arc',
}


def main(argv=None):
    parser = _argument_parser()
    args = parser.parse_args(argv)
    bag_path = Path(args.bag_db).expanduser().resolve()
    if not bag_path.is_file():
        parser.error(f'bag database does not exist: {bag_path}')

    replay_params = _yaml_parameters(
        args.replay_config,
        'maze_round15_replay_analyzer',
    )
    planner_params = _yaml_parameters(
        args.planner_config,
        'maze_first_turn_dry_run',
    )
    map_config = _dataclass_from_parameters(LocalMapConfig, replay_params)
    footprint = _dataclass_from_parameters(DynamicFootprint, replay_params)
    planner_config = _dataclass_from_parameters(
        PlannerConfig,
        replay_params,
    )
    map_builder = LocalMapBuilder(map_config)

    connection = sqlite3.connect(
        'file:' + str(bag_path) + '?mode=ro',
        uri=True,
    )
    topic_ids = dict(connection.execute(
        'select name,id from topics where name in '
        '("/utlidar/cloud_base","/utlidar/robot_odom")'
    ))
    missing = {
        '/utlidar/cloud_base',
        '/utlidar/robot_odom',
    } - set(topic_ids)
    if missing:
        raise RuntimeError('bag topics missing: ' + ','.join(sorted(missing)))
    bag_start_ns = connection.execute(
        'select min(timestamp) from messages'
    ).fetchone()[0]
    window_start_ns = bag_start_ns + int(args.start_offset_sec * 1.0e9)
    window_end_ns = window_start_ns + int(args.duration_sec * 1.0e9)

    odom_samples = _read_odom(
        connection,
        topic_ids['/utlidar/robot_odom'],
        window_start_ns,
        window_end_ns,
    )
    snapshots, build_times_ms, rejected_cloud_frames = _read_maps(
        connection,
        topic_ids['/utlidar/cloud_base'],
        window_start_ns,
        window_end_ns,
        map_builder,
        args.expected_cloud_frame,
    )
    connection.close()

    result = analyze_round15_actual_trajectory(
        snapshots,
        odom_samples,
        footprint,
        planner_config,
        contact_progress_deg=args.contact_progress_deg,
        turn_direction_sign=1,
    )
    result.update({
        'dry_run': True,
        'motion_output': False,
        'publisher_count': 0,
        'bag_db': str(bag_path),
        'window_start_offset_sec': args.start_offset_sec,
        'window_duration_sec': args.duration_sec,
        'replay_config': str(Path(args.replay_config).resolve()),
        'planner_config': str(Path(args.planner_config).resolve()),
        'rejected_cloud_frame_count': rejected_cloud_frames,
        'map_build_time_median_ms': (
            statistics.median(build_times_ms) if build_times_ms else None
        ),
        'map_build_time_max_ms': max(build_times_ms) if build_times_ms else None,
        'recorded_map_rate_hz': _recorded_rate(snapshots),
        'candidate_snapshot': _candidate_snapshot(
            result,
            snapshots,
            planner_config,
            footprint,
            planner_params,
        ),
    })
    output_path = Path(args.output).expanduser().resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        json.dumps(result, ensure_ascii=False, indent=2) + '\n',
        encoding='utf-8',
    )
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result['gate_status'] == 'DRY_RUN_PASS' else 2


def _read_odom(connection, topic_id, start_ns, end_ns):
    samples = []
    rows = connection.execute(
        'select timestamp,data from messages '
        'where topic_id=? and timestamp between ? and ? order by timestamp',
        (topic_id, start_ns, end_ns),
    )
    for timestamp_ns, serialized in rows:
        msg = deserialize_message(bytes(serialized), Odometry)
        orientation = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (
                orientation.w * orientation.z
                + orientation.x * orientation.y
            ),
            1.0 - 2.0 * (
                orientation.y * orientation.y
                + orientation.z * orientation.z
            ),
        )
        position = msg.pose.pose.position
        samples.append(ReplayOdomSample(
            timestamp_ns * 1.0e-9,
            float(position.x),
            float(position.y),
            float(yaw),
        ))
    return tuple(samples)


def _read_maps(
    connection,
    topic_id,
    start_ns,
    end_ns,
    map_builder,
    expected_frame,
):
    snapshots = []
    build_times_ms = []
    rejected_frames = 0
    rows = connection.execute(
        'select timestamp,data from messages '
        'where topic_id=? and timestamp between ? and ? order by timestamp',
        (topic_id, start_ns, end_ns),
    )
    for timestamp_ns, serialized in rows:
        msg = deserialize_message(bytes(serialized), PointCloud2)
        if str(msg.header.frame_id) != expected_frame:
            rejected_frames += 1
            continue
        started = time.perf_counter()
        grid = map_builder.build(point_cloud2.read_points(
            msg,
            field_names=('x', 'y', 'z'),
            skip_nans=False,
        ))
        build_times_ms.append((time.perf_counter() - started) * 1000.0)
        snapshots.append(ReplayMapSnapshot(
            timestamp_ns * 1.0e-9,
            grid,
        ))
    return tuple(snapshots), build_times_ms, rejected_frames


def _candidate_snapshot(
    replay_result,
    snapshots,
    planner_config,
    footprint,
    parameters,
):
    if not snapshots:
        return None
    alert = replay_result.get('first_geometry_alert') or {}
    target_time = alert.get('snapshot_time_sec', snapshots[-1].timestamp_sec)
    snapshot = min(
        snapshots,
        key=lambda item: abs(item.timestamp_sec - target_time),
    )
    sector_status = {}
    for name in SECTOR_NAMES:
        status = dict(snapshot.grid.sector_stats[name])
        status.update({
            'age_sec': 0.0,
            'stale': False,
            'usable': status.get('valid') is True,
        })
        sector_status[name] = status
    safety = SafetyContext(
        cloud_age_sec=0.0,
        odom_age_sec=0.0,
        sector_status=sector_status,
        watchdog_ok=None,
        watchdog_age_sec=None,
        estop_triggered=None,
        estop_age_sec=None,
    )
    planner = FirstTurnTrajectoryPlanner(
        planner_config,
        footprint,
        _motion_primitives(parameters),
    )
    plan = planner.plan(
        snapshot.grid,
        safety,
        STATE_SELECT_TRAJECTORY,
    )
    return {
        'snapshot_time_sec': snapshot.timestamp_sec,
        'rear_coverage_status': plan['rear_coverage_status'],
        'rear_unavailable_sectors': plan['rear_unavailable_sectors'],
        'has_robust_safe_candidate': plan['has_robust_safe'],
        'wall_segment_count': len(snapshot.grid.wall_segments),
        'top_candidates': [
            _compact_candidate(item)
            for item in plan['top_candidates']
        ],
    }


def _compact_candidate(candidate):
    sweep = candidate['sweep']
    return {
        'rank': candidate['rank'],
        'name': candidate['name'],
        'verdict': candidate['verdict'],
        'minimum_clearance_m': sweep['minimum_clearance_m'],
        'danger_time_sec': sweep['danger_time_sec'],
        'danger_pose': sweep['danger_pose'],
        'danger_part': sweep['collision_part'],
        'danger_wall_segment_id': sweep['danger_wall_segment_id'],
        'robustness_ratio': candidate['robustness_ratio'],
        'legacy_0413_guard_pass': candidate['legacy_0413_guard_pass'],
        'blockers': candidate['blockers'],
        'unknown_reasons': candidate['unknown_reasons'],
    }


def _motion_primitives(parameters):
    primitives = []
    for name, prefix in PRIMITIVE_PREFIXES.items():
        primitives.append(MotionPrimitive(
            name=name,
            vx_mps=float(parameters[prefix + '_vx_mps']),
            vy_mps=float(parameters[prefix + '_vy_mps']),
            wz_radps=float(parameters[prefix + '_wz_radps']),
            duration_sec=float(parameters[prefix + '_duration_sec']),
            calibrated=bool(parameters[prefix + '_calibrated']),
            calibration_id=str(parameters[prefix + '_calibration_id']),
        ))
    return tuple(primitives)


def _yaml_parameters(path, node_name):
    data = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    try:
        return dict(data[node_name]['ros__parameters'])
    except (KeyError, TypeError) as error:
        raise ValueError(f'invalid parameter file for {node_name}') from error


def _dataclass_from_parameters(data_class, parameters):
    defaults = data_class()
    values = {
        field.name: parameters.get(field.name, getattr(defaults, field.name))
        for field in fields(data_class)
    }
    return data_class(**values)


def _recorded_rate(snapshots):
    if len(snapshots) < 2:
        return None
    elapsed = snapshots[-1].timestamp_sec - snapshots[0].timestamp_sec
    return (len(snapshots) - 1) / elapsed if elapsed > 0.0 else None


def _argument_parser():
    parser = argparse.ArgumentParser(
        description='Read-only Round15 SQLite window analyzer',
    )
    parser.add_argument('--bag-db', required=True)
    parser.add_argument('--start-offset-sec', required=True, type=float)
    parser.add_argument('--duration-sec', required=True, type=float)
    parser.add_argument('--contact-progress-deg', type=float, default=44.0)
    parser.add_argument('--expected-cloud-frame', default='base_link')
    parser.add_argument(
        '--replay-config',
        default='config/maze_round15_replay.yaml',
    )
    parser.add_argument(
        '--planner-config',
        default='config/maze_first_turn_dry_run.yaml',
    )
    parser.add_argument('--output', required=True)
    return parser


if __name__ == '__main__':
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(130)
