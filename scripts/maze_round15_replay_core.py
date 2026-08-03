#!/usr/bin/env python3

"""Round15 旧实际轨迹的只读连续足迹回放核心。"""

from bisect import bisect_left
from dataclasses import dataclass
import math

from maze_first_turn_core import (
    Pose2D,
    SweptFootprintChecker,
    normalize_angle,
)


@dataclass(frozen=True)
class ReplayOdomSample:
    """Odom 坐标系中的带时间二维位姿。"""

    timestamp_sec: float
    x_m: float
    y_m: float
    yaw_rad: float


@dataclass(frozen=True)
class ReplayMapSnapshot:
    """某一历史时刻、以当时 base_link 表示的局部地图。"""

    timestamp_sec: float
    grid: object


def analyze_round15_actual_trajectory(
    snapshots,
    odom_samples,
    footprint,
    planner_config,
    contact_progress_deg=44.0,
    turn_direction_sign=1,
):
    """用历史地图检查后续实际轨迹是否在接触前已可预测为不安全。

    `contact_progress_deg` 来自事故记录，只限定分析终点；碰撞结论完全由
    点云几何、动态足迹和实际 Odom 路径计算，不使用该角度伪造危险点。
    """
    ordered_maps = sorted(snapshots, key=lambda item: item.timestamp_sec)
    ordered_odom = sorted(odom_samples, key=lambda item: item.timestamp_sec)
    base_result = {
        'test_id': 'B2_1_A_ROUND15_REPLAY',
        'physical_contact_result': 'FAIL',
        'result': 'UNKNOWN',
        'gate_status': 'BLOCKED',
        'contact_progress_deg': float(contact_progress_deg),
        'map_snapshot_count': len(ordered_maps),
        'odom_sample_count': len(ordered_odom),
        'first_geometry_alert': None,
        'first_round15_matching_raw_geometry_alert': None,
        'first_round15_matching_geometry_alert': None,
        'first_legacy_0413_alert': None,
        'minimum_predicted_clearance_m': None,
        'dangerous_wall_segment': None,
        'dangerous_wall_evidence_kind': None,
        'dangerous_footprint_part': None,
        'global_minimum_danger': None,
        'dangerous_part_matches_round15': False,
        'geometry_alert_before_contact': False,
        'matching_geometry_alert_before_contact': False,
        'legacy_only_explanation': False,
        'failure_reason': '',
    }
    if not ordered_maps:
        base_result['failure_reason'] = 'no_map_snapshots'
        return base_result
    if len(ordered_odom) < 2:
        base_result['failure_reason'] = 'insufficient_odom_samples'
        return base_result
    if turn_direction_sign not in (-1, 1):
        raise ValueError('turn_direction_sign must be -1 or 1')
    if not math.isfinite(contact_progress_deg) or contact_progress_deg <= 0.0:
        raise ValueError('contact_progress_deg must be positive and finite')

    unwrapped_yaw, yaw_jump = _unwrap_yaw(
        ordered_odom,
        planner_config.yaw_jump_limit_deg,
    )
    if yaw_jump:
        base_result['failure_reason'] = 'yaw_jump_in_replay'
        return base_result

    directed_progress = [
        turn_direction_sign * (yaw - unwrapped_yaw[0])
        for yaw in unwrapped_yaw
    ]
    contact_target = math.radians(contact_progress_deg)
    contact_index = next(
        (
            index
            for index, progress in enumerate(directed_progress)
            if progress >= contact_target
        ),
        None,
    )
    if contact_index is None:
        base_result['failure_reason'] = 'contact_progress_not_reached'
        return base_result

    contact_time = ordered_odom[contact_index].timestamp_sec
    odom_times = [item.timestamp_sec for item in ordered_odom]
    checker = SweptFootprintChecker(footprint, planner_config)
    expanded = footprint.expanded_extents()
    corner_radius = math.hypot(
        max(expanded['front'], expanded['rear']),
        max(expanded['left'], expanded['right']),
    )
    first_geometry = None
    first_matching_raw_geometry = None
    first_matching_geometry = None
    first_legacy = None
    global_minimum = None
    global_danger = None

    for snapshot in ordered_maps:
        if snapshot.timestamp_sec >= contact_time:
            break
        start_index = _nearest_index(odom_times, snapshot.timestamp_sec)
        if start_index >= contact_index:
            continue
        actual_poses = _relative_actual_poses(
            ordered_odom,
            unwrapped_yaw,
            start_index,
            contact_index,
            planner_config.trajectory_max_translation_step_m,
            planner_config.trajectory_max_yaw_step_deg,
            corner_radius,
        )
        if len(actual_poses) < 2:
            continue
        sweep = checker.check(snapshot.grid, actual_poses)
        clearance = float(sweep['minimum_clearance_m'])
        if global_minimum is None or clearance < global_minimum:
            global_minimum = clearance
            global_danger = (snapshot, sweep)

        if (
            first_geometry is None
            and sweep['first_unsafe_time_sec'] is not None
        ):
            first_geometry = _geometry_alert_record(
                snapshot,
                sweep,
                ordered_odom,
                unwrapped_yaw,
                directed_progress,
                start_index,
                contact_time,
            )
        matching = _first_matching_part_evidence(sweep)
        if matching is not None:
            raw_record = _part_geometry_alert_record(
                snapshot,
                matching,
                ordered_odom,
                unwrapped_yaw,
                directed_progress,
                start_index,
                contact_time,
            )
            if first_matching_raw_geometry is None:
                # 原始点碰撞永远是首要安全事实，即使该帧暂未提取出有限墙段。
                first_matching_raw_geometry = raw_record
            if (
                first_matching_geometry is None
                and _wall_record_is_association_eligible(
                    raw_record['wall_segment']
                )
            ):
                # 回放门禁需要有限墙证据；短片段必须达到独立的关联置信度。
                # 它仅用于可追溯性，不改变完整墙段的规划放行要求。
                first_matching_geometry = raw_record

        left_distance = snapshot.grid.sector_stats.get(
            'left', {}
        ).get('distance_m')
        if (
            first_legacy is None
            and left_distance is not None
            and float(left_distance)
            < planner_config.legacy_turn_sweep_clearance_m
        ):
            first_legacy = {
                'snapshot_time_sec': snapshot.timestamp_sec,
                'turn_progress_deg': math.degrees(
                    directed_progress[start_index]
                ),
                'left_distance_m': float(left_distance),
                'threshold_m': planner_config.legacy_turn_sweep_clearance_m,
                'lead_to_contact_sec': contact_time - snapshot.timestamp_sec,
            }

    base_result['first_geometry_alert'] = first_geometry
    base_result['first_round15_matching_raw_geometry_alert'] = (
        first_matching_raw_geometry
    )
    base_result['first_round15_matching_geometry_alert'] = (
        first_matching_geometry
    )
    base_result['first_legacy_0413_alert'] = first_legacy
    base_result['minimum_predicted_clearance_m'] = global_minimum
    if global_danger is not None:
        snapshot, sweep = global_danger
        base_result['global_minimum_danger'] = {
            'wall_segment': _wall_segment_for_sweep(snapshot.grid, sweep),
            'footprint_part': sweep['collision_part'],
            'geometry_type': sweep['danger_geometry_type'],
            'pose': sweep['danger_pose'],
        }

    if first_geometry is None:
        base_result['failure_reason'] = 'geometry_did_not_reject_old_path'
        base_result['legacy_only_explanation'] = first_legacy is not None
        return base_result

    danger_part = (
        first_matching_geometry.get('dangerous_footprint_part')
        if first_matching_geometry is not None
        else None
    )
    part_matches = first_matching_geometry is not None
    geometry_before_contact = first_geometry['lead_to_contact_sec'] > 0.0
    matching_before_contact = (
        first_matching_geometry is not None
        and first_matching_geometry['lead_to_contact_sec'] > 0.0
    )
    base_result.update({
        'result': 'FAIL',
        'geometry_alert_before_contact': geometry_before_contact,
        'matching_geometry_alert_before_contact': matching_before_contact,
        'dangerous_part_matches_round15': part_matches,
        'dangerous_wall_segment': (
            first_matching_geometry.get('wall_segment')
            if first_matching_geometry is not None
            else None
        ),
        'dangerous_wall_evidence_kind': (
            first_matching_geometry['wall_segment'].get('evidence_kind')
            if first_matching_geometry is not None
            and first_matching_geometry.get('wall_segment') is not None
            else None
        ),
        'dangerous_footprint_part': danger_part,
        'legacy_only_explanation': False,
        'failure_reason': '',
        # 只有几何提前报警、墙证据存在且危险部位一致才通过回放门禁。
        'gate_status': (
            'DRY_RUN_PASS'
            if geometry_before_contact
            and matching_before_contact
            and first_matching_geometry.get('wall_segment') is not None
            and part_matches
            else 'FAIL'
        ),
    })
    return base_result


def _unwrap_yaw(samples, jump_limit_deg):
    values = [float(samples[0].yaw_rad)]
    jump = False
    for sample in samples[1:]:
        delta = normalize_angle(float(sample.yaw_rad) - values[-1])
        if abs(math.degrees(delta)) > jump_limit_deg:
            jump = True
        values.append(values[-1] + delta)
    return values, jump


def _nearest_index(ordered_values, target):
    index = bisect_left(ordered_values, target)
    if index <= 0:
        return 0
    if index >= len(ordered_values):
        return len(ordered_values) - 1
    before = ordered_values[index - 1]
    after = ordered_values[index]
    return index - 1 if target - before <= after - target else index


def _relative_actual_poses(
    samples,
    yaw_values,
    start_index,
    end_index,
    max_corner_step_m,
    max_yaw_step_deg,
    corner_radius_m,
):
    start = samples[start_index]
    start_yaw = yaw_values[start_index]
    cos_yaw = math.cos(start_yaw)
    sin_yaw = math.sin(start_yaw)
    dense_poses = []
    for index in range(start_index, end_index + 1):
        sample = samples[index]
        dx = sample.x_m - start.x_m
        dy = sample.y_m - start.y_m
        dense_poses.append(Pose2D(
            cos_yaw * dx + sin_yaw * dy,
            -sin_yaw * dx + cos_yaw * dy,
            yaw_values[index] - start_yaw,
            max(0.0, sample.timestamp_sec - start.timestamp_sec),
        ))
    # Odom 约149Hz，逐帧检查会重复大量近似位姿。只有在删除中间样本后
    # 仍保证任一足迹角点不跨过1cm、Yaw不跨过1度时才做下采样。
    if len(dense_poses) <= 2:
        return tuple(dense_poses)
    selected = [dense_poses[0]]
    previous = dense_poses[0]
    max_yaw_step_rad = math.radians(max_yaw_step_deg)
    for index in range(1, len(dense_poses)):
        current = dense_poses[index]
        center_step = math.hypot(
            current.x_m - selected[-1].x_m,
            current.y_m - selected[-1].y_m,
        )
        yaw_step = abs(current.yaw_rad - selected[-1].yaw_rad)
        corner_step_bound = center_step + corner_radius_m * yaw_step
        if (
            corner_step_bound > max_corner_step_m
            or yaw_step > max_yaw_step_rad
        ):
            if previous is not selected[-1]:
                selected.append(previous)
            selected.append(current)
        previous = current
    if selected[-1] is not dense_poses[-1]:
        selected.append(dense_poses[-1])
    return tuple(selected)


def _geometry_alert_record(
    snapshot,
    sweep,
    samples,
    yaw_values,
    directed_progress,
    start_index,
    contact_time,
):
    unsafe_time = float(sweep['first_unsafe_time_sec'])
    unsafe_timestamp = samples[start_index].timestamp_sec + unsafe_time
    sample_times = [item.timestamp_sec for item in samples]
    unsafe_index = _nearest_index(sample_times, unsafe_timestamp)
    wall = _wall_segment_for_id(
        snapshot.grid,
        sweep.get('first_unsafe_wall_segment_id'),
    )
    return {
        'snapshot_time_sec': snapshot.timestamp_sec,
        'alert_turn_progress_deg': math.degrees(
            directed_progress[start_index]
        ),
        'predicted_unsafe_after_sec': unsafe_time,
        'predicted_unsafe_yaw_deg': math.degrees(
            directed_progress[unsafe_index]
        ),
        'predicted_minimum_clearance_m': sweep['minimum_clearance_m'],
        'first_unsafe_pose': sweep['first_unsafe_pose'],
        'dangerous_footprint_part': sweep['first_unsafe_part'],
        'dangerous_geometry_type': sweep['first_unsafe_geometry_type'],
        'dangerous_point_m': sweep['first_unsafe_point_m'],
        'wall_segment': wall,
        'lead_to_contact_sec': contact_time - snapshot.timestamp_sec,
    }


def _first_matching_part_evidence(sweep):
    matching = []
    for part in ('left_side', 'front_left', 'rear_left'):
        evidence = sweep.get('first_unsafe_by_part', {}).get(part)
        if evidence is not None:
            matching.append((float(evidence['time_sec']), part, evidence))
    if not matching:
        return None
    _, part, evidence = min(matching, key=lambda item: item[0])
    result = dict(evidence)
    result['part'] = part
    return result


def _part_geometry_alert_record(
    snapshot,
    evidence,
    samples,
    yaw_values,
    directed_progress,
    start_index,
    contact_time,
):
    unsafe_time = float(evidence['time_sec'])
    unsafe_timestamp = samples[start_index].timestamp_sec + unsafe_time
    sample_times = [item.timestamp_sec for item in samples]
    unsafe_index = _nearest_index(sample_times, unsafe_timestamp)
    wall = _wall_segment_for_id(
        snapshot.grid,
        evidence.get('wall_segment_id'),
    )
    return {
        'snapshot_time_sec': snapshot.timestamp_sec,
        'alert_turn_progress_deg': math.degrees(
            directed_progress[start_index]
        ),
        'predicted_unsafe_after_sec': unsafe_time,
        'predicted_unsafe_yaw_deg': math.degrees(
            directed_progress[unsafe_index]
        ),
        'predicted_clearance_m': evidence.get('clearance_m'),
        'first_unsafe_pose': evidence.get('pose'),
        'dangerous_footprint_part': evidence['part'],
        'dangerous_geometry_type': evidence.get('geometry_type'),
        'dangerous_point_m': evidence.get('point_m'),
        'wall_segment': wall,
        'lead_to_contact_sec': contact_time - snapshot.timestamp_sec,
    }


def _wall_segment_for_sweep(grid, sweep):
    return _wall_segment_for_id(grid, sweep.get('danger_wall_segment_id'))


def _wall_segment_for_id(grid, segment_id):
    if segment_id is None:
        return None
    for segment in grid.wall_segments:
        if segment['id'] == segment_id:
            return dict(segment)
    return None


def _wall_record_is_association_eligible(wall_record):
    """有限墙证据需明确合格，避免弱片段被误当作回放因果关联。"""
    if wall_record is None:
        return False
    return bool(wall_record.get('association_eligible', True))
