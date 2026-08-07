#!/usr/bin/env python3

"""B2.1 第一弯占据图、连续扫掠和安全状态机纯逻辑测试。"""

from dataclasses import replace
import math
from pathlib import Path
import sys
import unittest


SCRIPTS_DIR = Path(__file__).resolve().parents[1] / 'scripts'
sys.path.insert(0, str(SCRIPTS_DIR))

from maze_first_turn_core import (  # noqa: E402
    DynamicFootprint,
    FirstTurnDryRunStateMachine,
    FirstTurnTrajectoryPlanner,
    LocalMapBuilder,
    LocalMapConfig,
    MotionPrimitive,
    PlannerConfig,
    Pose2D,
    PRIMITIVE_FINE_LEFT_ARC,
    PRIMITIVE_FORWARD,
    PRIMITIVE_LEFT_ARC,
    PRIMITIVE_LEFT_ARC_OUTSIDE,
    PRIMITIVE_OUTSIDE_DIAGONAL,
    PRIMITIVE_REVERSE,
    SafetyContext,
    SECTOR_NAMES,
    SectorFreshnessTracker,
    STATE_FAULT_STOP,
    STATE_SELECT_TRAJECTORY,
    STATE_TURN_COMPLETE,
    SweptFootprintChecker,
    TrajectoryGenerator,
    VERDICT_DRY_RUN_SAFE,
    VERDICT_GEOMETRY_SAFE_UNCALIBRATED,
    VERDICT_NOMINAL_SAFE,
    VERDICT_ROBUST_SAFE,
    VERDICT_UNKNOWN,
    VERDICT_UNSAFE,
    default_motion_primitives,
)


class FirstTurnCoreTest(unittest.TestCase):
    """验证第一弯规划不会把未知空间或采样间隙判为安全。"""

    def setUp(self):
        self.map_config = replace(
            LocalMapConfig(),
            body_x_min_m=-0.01,
            body_x_max_m=0.01,
            body_y_min_m=-0.01,
            body_y_max_m=0.01,
            sector_min_coverage_points=1,
            sector_min_coverage_bins=1,
        )
        self.planner_config = replace(
            PlannerConfig(),
            stable_confirm_frames=2,
            stage_timeout_sec=10.0,
        )
        self.footprint = replace(
            DynamicFootprint(),
            gait_sway_margin_m=0.0,
            cloud_uncertainty_margin_m=0.0,
            odom_uncertainty_margin_m=0.0,
            model_uncertainty_margin_m=0.0,
            stop_tail_margin_m=0.0,
        )

    def test_local_map_filters_invalid_ground_and_body_points(self):
        builder = LocalMapBuilder(replace(
            self.map_config,
            body_x_min_m=-0.10,
            body_x_max_m=0.10,
            body_y_min_m=-0.10,
            body_y_max_m=0.10,
        ))
        grid = builder.build((
            (float('nan'), 0.0, 0.2),
            (float('inf'), 0.0, 0.2),
            (0.5, 0.0, -0.3),
            (0.06, 0.00, 0.2),
            (0.8, 0.0, 0.2),
        ))
        self.assertEqual(grid.statistics['total_points'], 5)
        self.assertEqual(grid.statistics['finite_points'], 3)
        self.assertEqual(grid.statistics['occupied_cells'], 1)
        self.assertEqual(grid.statistics['height_filtered_points'], 1)
        self.assertEqual(grid.statistics['body_filtered_points'], 1)

    def test_front_leg_filter_is_local_and_preserves_front_obstacle(self):
        """前腿过滤只能排除已标定角区，不能吞掉机头正前方近障。"""
        builder = LocalMapBuilder(replace(
            self.map_config,
            front_leg_self_filter_enabled=True,
            front_left_leg_x_min_m=0.15,
            front_left_leg_x_max_m=0.45,
            front_left_leg_y_min_m=0.18,
            front_left_leg_y_max_m=0.28,
            front_right_leg_x_min_m=0.15,
            front_right_leg_x_max_m=0.45,
            front_right_leg_y_min_m=-0.28,
            front_right_leg_y_max_m=-0.18,
        ))
        grid = builder.build((
            (0.35, 0.22, 0.20),   # 左前腿静止自回波。
            (0.35, -0.25, 0.20),  # 右前腿静止自回波。
            (0.43, 0.00, 0.20),   # 机头正前方障碍，必须保留。
            (0.50, 0.35, 0.20),   # 侧方墙点，必须保留。
        ))
        self.assertEqual(grid.statistics['leg_self_filtered_points'], 2)
        self.assertEqual(grid.statistics['occupied_cells'], 2)
        self.assertTrue(any(
            math.isclose(x, 0.43, abs_tol=1.0e-6)
            and math.isclose(y, 0.01, abs_tol=1.0e-6)
            for x, y in grid.obstacle_points
        ))
        self.assertTrue(any(
            math.isclose(x, 0.51, abs_tol=1.0e-6)
            and math.isclose(y, 0.35, abs_tol=1.0e-6)
            for x, y in grid.obstacle_points
        ))

    def test_rear_coverage_is_independent(self):
        points = self._ring_points(excluded={'rear'})
        grid = LocalMapBuilder(self.map_config).build(points)
        self.assertTrue(grid.sector_stats['front']['valid'])
        self.assertFalse(grid.sector_stats['rear']['valid'])
        self.assertEqual(
            grid.sector_stats['rear']['coverage_point_count'],
            0,
        )

    def test_finite_wall_model_preserves_endpoints(self):
        points = tuple(
            (0.30 + index * 0.02, 0.55, 0.20)
            for index in range(31)
        )
        grid = LocalMapBuilder(self.map_config).build(points)
        self.assertGreaterEqual(len(grid.wall_segments), 1)
        wall = max(grid.wall_segments, key=lambda item: item['length_m'])
        self.assertGreater(wall['length_m'], 0.50)
        endpoint_x = sorted((wall['start_m'][0], wall['end_m'][0]))
        self.assertAlmostEqual(endpoint_x[0], 0.31, delta=0.04)
        self.assertAlmostEqual(endpoint_x[1], 0.91, delta=0.04)
        self.assertGreater(wall['confidence'], 0.45)

    def test_wall_endpoint_is_reported_as_collision_evidence(self):
        points = tuple(
            (0.30 + index * 0.02, 0.20, 0.20)
            for index in range(11)
        )
        grid = LocalMapBuilder(self.map_config).build(points)
        footprint = replace(
            self.footprint,
            footprint_front_m=0.10,
            footprint_rear_m=0.10,
            footprint_left_m=0.10,
            footprint_right_m=0.10,
            target_physical_clearance_m=0.05,
        )
        poses = (Pose2D(0.20, 0.10, 0.0, 0.0),)
        result = SweptFootprintChecker(
            footprint,
            self.planner_config,
        ).check(grid, poses)
        self.assertFalse(result['wall_endpoint_clearance_censored'])
        self.assertIsNotNone(result['danger_wall_segment_id'])

    def test_each_sector_has_independent_stale_age(self):
        tracker = SectorFreshnessTracker(0.5)
        stats = {
            name: {
                'point_count': 1,
                'coverage_point_count': 1,
                'coverage_bin_count': 1,
                'distance_m': 1.0,
                'valid': name != 'rear',
            }
            for name in SECTOR_NAMES
        }
        tracker.update(stats, 1.0)
        first = tracker.snapshot(1.1)
        self.assertTrue(first['front']['usable'])
        self.assertTrue(first['rear']['stale'])

        stats['rear']['valid'] = True
        stats['front']['valid'] = False
        tracker.update(stats, 1.4)
        second = tracker.snapshot(1.45)
        self.assertTrue(second['rear']['usable'])
        self.assertFalse(second['front']['usable'])
        self.assertFalse(second['front']['stale'])

    def test_continuous_sweep_detects_mid_trajectory_collision(self):
        footprint = DynamicFootprint(
            footprint_front_m=0.10,
            footprint_rear_m=0.10,
            footprint_left_m=0.10,
            footprint_right_m=0.10,
            gait_sway_margin_m=0.0,
            cloud_uncertainty_margin_m=0.0,
            odom_uncertainty_margin_m=0.0,
            model_uncertainty_margin_m=0.0,
            stop_tail_margin_m=0.0,
            margins_calibrated=True,
        )
        primitive = MotionPrimitive(
            'ARC_TEST', 0.40, 0.0, 1.0, 1.0, True, 'test'
        )
        generator = TrajectoryGenerator(self.planner_config, footprint)
        poses = generator.generate(primitive)
        obstacle = (0.19, 0.05, 0.2)
        grid = LocalMapBuilder(self.map_config).build((obstacle,))
        result = SweptFootprintChecker(
            footprint,
            self.planner_config,
        ).check(grid, poses)
        self.assertTrue(result['collision'])
        self.assertGreater(result['danger_time_sec'], 0.0)
        self.assertLess(result['danger_time_sec'], 1.0)

    def test_stop_tail_is_included_in_sweep(self):
        footprint = DynamicFootprint(
            footprint_front_m=0.10,
            footprint_rear_m=0.10,
            footprint_left_m=0.10,
            footprint_right_m=0.10,
            gait_sway_margin_m=0.0,
            cloud_uncertainty_margin_m=0.0,
            odom_uncertainty_margin_m=0.0,
            model_uncertainty_margin_m=0.0,
            stop_tail_margin_m=0.10,
            margins_calibrated=True,
        )
        primitive = MotionPrimitive(
            'FORWARD_TEST', 0.20, 0.0, 0.0, 0.50, True, 'test'
        )
        poses = TrajectoryGenerator(
            self.planner_config,
            footprint,
        ).generate(primitive)
        grid = LocalMapBuilder(self.map_config).build(((0.27, 0.0, 0.2),))
        result = SweptFootprintChecker(
            footprint,
            self.planner_config,
        ).check(grid, poses)
        self.assertTrue(result['collision'])
        self.assertGreater(result['danger_time_sec'], primitive.duration_sec)
        self.assertEqual(result['collision_part'], 'front_edge')

    def test_asymmetric_left_footprint_changes_clearance(self):
        grid = LocalMapBuilder(self.map_config).build(((0.0, 0.22, 0.2),))
        small_left = replace(self.footprint, footprint_left_m=0.15)
        large_left = replace(self.footprint, footprint_left_m=0.24)
        pose = (Pose2D(0.0, 0.0, 0.0, 0.0),)
        safe = SweptFootprintChecker(
            small_left,
            self.planner_config,
        ).check(grid, pose)
        collision = SweptFootprintChecker(
            large_left,
            self.planner_config,
        ).check(grid, pose)
        self.assertFalse(safe['collision'])
        self.assertTrue(collision['collision'])
        self.assertEqual(collision['collision_part'], 'left_side')

    def test_in_place_rotation_is_rejected(self):
        primitive = MotionPrimitive(
            'FORBIDDEN', 0.0, 0.0, 0.3, 0.5, False, 'none'
        )
        with self.assertRaisesRegex(ValueError, 'in-place rotation'):
            TrajectoryGenerator(
                self.planner_config,
                self.footprint,
            ).generate(primitive)

    def test_sampling_limits_every_footprint_corner_motion_and_yaw(self):
        primitive = MotionPrimitive(
            'ARC_SAMPLE_TEST', 0.25, -0.05, 0.60, 0.50, False, 'none'
        )
        poses = TrajectoryGenerator(
            self.planner_config,
            self.footprint,
        ).generate(primitive)
        extents = self.footprint.expanded_extents()
        corners = (
            (extents['front'], extents['left']),
            (extents['front'], -extents['right']),
            (-extents['rear'], extents['left']),
            (-extents['rear'], -extents['right']),
        )
        for previous, current in zip(poses, poses[1:]):
            if current.time_sec > primitive.duration_sec:
                continue
            self.assertLessEqual(
                abs(math.degrees(current.yaw_rad - previous.yaw_rad)),
                self.planner_config.trajectory_max_yaw_step_deg + 1.0e-9,
            )
            for corner_x, corner_y in corners:
                previous_point = self._transform_corner(
                    previous, corner_x, corner_y
                )
                current_point = self._transform_corner(
                    current, corner_x, corner_y
                )
                self.assertLessEqual(
                    math.hypot(
                        current_point[0] - previous_point[0],
                        current_point[1] - previous_point[1],
                    ),
                    self.planner_config.trajectory_max_translation_step_m
                    + 1.0e-6,
                )

    def test_uncalibrated_candidate_is_dry_run_only(self):
        planner = self._planner(
            footprint=replace(self.footprint, margins_calibrated=False)
        )
        evaluation = planner.evaluate_candidate(
            planner.primitives[PRIMITIVE_FORWARD],
            self._safe_grid(),
            self._safety(),
        )
        self.assertEqual(evaluation['verdict'], VERDICT_DRY_RUN_SAFE)
        self.assertIn('motion_model_uncalibrated', evaluation['unverified'])

    def test_calibrated_candidate_can_be_robust_safe(self):
        footprint = replace(self.footprint, margins_calibrated=True)
        primitives = tuple(
            replace(item, calibrated=True, calibration_id='b16-test')
            for item in default_motion_primitives()
        )
        planner = self._planner(footprint=footprint, primitives=primitives)
        evaluation = planner.evaluate_candidate(
            planner.primitives[PRIMITIVE_FORWARD],
            self._safe_grid(),
            self._safety(),
        )
        self.assertEqual(evaluation['verdict'], VERDICT_ROBUST_SAFE)
        self.assertEqual(evaluation['robustness_pass_count'], 7)

    def test_turn_without_reliable_wall_model_is_unknown(self):
        planner = self._planner()
        grid = LocalMapBuilder(self.map_config).build(((1.20, 1.20, 0.20),))
        evaluation = planner.evaluate_candidate(
            planner.primitives[PRIMITIVE_LEFT_ARC],
            grid,
            self._safety(),
        )
        self.assertEqual(evaluation['verdict'], VERDICT_UNKNOWN)
        self.assertIn('wall_model_insufficient', evaluation['unknown_reasons'])

    def test_short_fragment_associates_evidence_but_cannot_authorize_turn(self):
        """稀疏墙片段可审计原始点，但不能降低转弯墙模型准入门槛。"""
        points = (
            (1.20, 0.30, 0.20),
            (1.20, 0.38, 0.20),
            (1.20, 0.46, 0.20),
            (1.20, 0.54, 0.20),
        )
        grid = LocalMapBuilder(self.map_config).build(points)
        fragments = [
            segment
            for segment in grid.wall_segments
            if segment.get('evidence_kind') == 'wall_fragment'
        ]
        self.assertEqual(len(fragments), 1)
        fragment = fragments[0]
        self.assertTrue(fragment['association_eligible'])
        self.assertFalse(fragment['reliable_for_turn_model'])
        self.assertEqual(
            grid.nearest_wall_segment_id((1.21, 0.45)),
            fragment['id'],
        )

        evaluation = self._planner().evaluate_candidate(
            self._planner().primitives[PRIMITIVE_LEFT_ARC],
            grid,
            self._safety(),
        )
        self.assertEqual(evaluation['verdict'], VERDICT_UNKNOWN)
        self.assertIn('wall_model_insufficient', evaluation['unknown_reasons'])

    def test_reliable_finite_wall_can_override_legacy_0413_guard(self):
        planner = self._planner()
        points = tuple(
            (0.25 + index * 0.02, 0.65, 0.20)
            for index in range(41)
        )
        grid = LocalMapBuilder(self.map_config).build(points)
        safety = self._safety()
        safety.sector_status['left']['distance_m'] = 0.35
        evaluation = planner.evaluate_candidate(
            planner.primitives[PRIMITIVE_LEFT_ARC],
            grid,
            safety,
        )
        self.assertFalse(evaluation['legacy_0413_guard_pass'])
        self.assertTrue(evaluation['legacy_0413_geometry_override'])
        self.assertEqual(
            evaluation['verdict'],
            VERDICT_GEOMETRY_SAFE_UNCALIBRATED,
        )

    def test_candidate_ranking_rejects_unknown_then_maximizes_clearance(self):
        def candidate(name, verdict, clearance):
            return {
                'name': name,
                'verdict': verdict,
                'robustness_ratio': 1.0,
                'final_yaw_error_deg': 0.0,
                'final_lateral_error_m': 0.0,
                'duration_sec': 0.5,
                'sweep': {
                    'minimum_clearance_m': clearance,
                    'minimum_stop_tail_clearance_m': clearance,
                    'minimum_wall_endpoint_clearance_m': clearance,
                },
            }

        ranked = FirstTurnTrajectoryPlanner._rank_candidates((
            candidate(PRIMITIVE_FORWARD, VERDICT_NOMINAL_SAFE, 0.19),
            candidate(PRIMITIVE_LEFT_ARC, VERDICT_ROBUST_SAFE, 0.06),
            candidate(PRIMITIVE_REVERSE, VERDICT_UNKNOWN, 0.20),
        ))
        self.assertEqual(ranked[0]['name'], PRIMITIVE_FORWARD)
        self.assertEqual([item['rank'] for item in ranked], [1, 2, 3])

    def test_plan_always_reports_six_candidates_and_top_five(self):
        planner = self._planner()
        plan = planner.plan(
            self._safe_grid(),
            self._safety(),
            STATE_SELECT_TRAJECTORY,
        )
        self.assertEqual(len(plan['candidates']), 6)
        self.assertEqual(len(plan['top_candidates']), 5)
        self.assertEqual(
            sorted(item['rank'] for item in plan['candidates']),
            [1, 2, 3, 4, 5, 6],
        )

    def test_reverse_requires_all_rear_sectors(self):
        planner = self._planner()
        safety = self._safety()
        safety.sector_status['left_rear']['usable'] = False
        evaluation = planner.evaluate_candidate(
            planner.primitives[PRIMITIVE_REVERSE],
            self._safe_grid(),
            safety,
        )
        self.assertEqual(evaluation['verdict'], VERDICT_UNKNOWN)
        self.assertIn(
            'sector_unavailable:left_rear',
            evaluation['unknown_reasons'],
        )

    def test_reverse_requires_live_watchdog_and_clear_estop(self):
        planner = self._planner()
        safety = replace(
            self._safety(),
            watchdog_ok=None,
            estop_triggered=None,
        )
        evaluation = planner.evaluate_candidate(
            planner.primitives[PRIMITIVE_REVERSE],
            self._safe_grid(),
            safety,
        )
        self.assertEqual(evaluation['verdict'], VERDICT_UNKNOWN)
        self.assertIn(
            'reverse_watchdog_not_ok', evaluation['unknown_reasons']
        )
        self.assertIn(
            'reverse_estop_not_clear', evaluation['unknown_reasons']
        )

    def test_reverse_segment_and_distance_limits(self):
        planner = self._planner(
            config=replace(
                self.planner_config,
                max_reverse_segments=1,
                max_reverse_distance_m=0.20,
            )
        )
        planner.record_reverse_execution(PRIMITIVE_REVERSE)
        evaluation = planner.evaluate_candidate(
            planner.primitives[PRIMITIVE_REVERSE],
            self._safe_grid(),
            self._safety(),
        )
        self.assertEqual(evaluation['verdict'], VERDICT_UNSAFE)
        self.assertIn('reverse_segment_limit', evaluation['blockers'])

    def test_initial_sensor_wait_does_not_latch_fault(self):
        planner = self._planner()
        machine = FirstTurnDryRunStateMachine(
            self.planner_config,
            planner,
        )
        safety = replace(
            self._safety(),
            cloud_received=False,
            odom_received=False,
        )
        output = machine.update(
            None,
            safety,
            {},
            1.0,
            0,
        )
        self.assertEqual(output['state'], 'APPROACH_TURN')
        self.assertEqual(output['reason'], 'waiting_initial_sensors')

    def test_stop_scan_selects_dry_run_candidate_without_execution(self):
        planner = self._planner()
        machine = FirstTurnDryRunStateMachine(
            self.planner_config,
            planner,
        )
        output = None
        for sequence in (1, 2):
            output = machine.update(
                self._safe_grid(),
                self._safety(),
                self._observation(front=0.70, left=0.50),
                1.0 + sequence * 0.1,
                sequence,
            )
        self.assertEqual(output['state'], STATE_SELECT_TRAJECTORY)
        self.assertIsNotNone(output['selected_candidate'])
        self.assertEqual(
            output['selected_candidate']['verdict'],
            VERDICT_DRY_RUN_SAFE,
        )
        self.assertFalse(output['execution_allowed'])
        self.assertFalse(output['motion_output'])

    def test_cloud_stale_latches_fault(self):
        planner = self._planner()
        machine = FirstTurnDryRunStateMachine(
            self.planner_config,
            planner,
        )
        stale = replace(self._safety(), cloud_age_sec=1.0)
        output = machine.update(
            self._safe_grid(),
            stale,
            self._observation(),
            1.0,
            1,
        )
        self.assertEqual(output['state'], STATE_FAULT_STOP)
        self.assertEqual(output['reason'], 'cloud_stale')

    def test_live_watchdog_fault_latches_fault(self):
        planner = self._planner()
        machine = FirstTurnDryRunStateMachine(
            self.planner_config,
            planner,
        )
        safety = replace(self._safety(), watchdog_ok=False)
        output = machine.update(
            self._safe_grid(),
            safety,
            self._observation(),
            1.0,
            1,
        )
        self.assertEqual(output['state'], STATE_FAULT_STOP)
        self.assertEqual(output['reason'], 'watchdog_fault')

    def test_live_estop_latches_fault(self):
        planner = self._planner()
        machine = FirstTurnDryRunStateMachine(
            self.planner_config,
            planner,
        )
        safety = replace(self._safety(), estop_triggered=True)
        output = machine.update(
            self._safe_grid(),
            safety,
            self._observation(),
            1.0,
            1,
        )
        self.assertEqual(output['state'], STATE_FAULT_STOP)
        self.assertEqual(output['reason'], 'estop_triggered')

    def test_first_turn_complete_stops_at_route_one_of_five(self):
        planner = self._planner()
        machine = FirstTurnDryRunStateMachine(
            self.planner_config,
            planner,
        )
        machine.state = STATE_SELECT_TRAJECTORY
        machine._state_start_time = 1.0
        output = machine.update(
            self._safe_grid(),
            self._safety(),
            self._observation(front=1.0, left=0.30, right=0.30, turn=math.pi / 2),
            2.0,
            1,
        )
        self.assertEqual(output['state'], STATE_TURN_COMPLETE)
        self.assertEqual(output['route_index'], 1)
        self.assertEqual(output['route_total'], 5)
        self.assertIsNone(output['selected_candidate'])

    def test_tracking_deviation_enters_fault(self):
        footprint = replace(self.footprint, margins_calibrated=True)
        primitives = tuple(
            replace(item, calibrated=True, calibration_id='test')
            for item in default_motion_primitives()
        )
        planner = self._planner(footprint=footprint, primitives=primitives)
        machine = FirstTurnDryRunStateMachine(
            self.planner_config,
            planner,
        )
        candidate = planner.evaluate_candidate(
            planner.primitives[PRIMITIVE_FORWARD],
            self._safe_grid(),
            self._safety(),
        )
        machine.acknowledge_segment_start(
            candidate,
            1.0,
            Pose2D(0.0, 0.0, 0.0, 1.0),
        )
        output = machine.check_tracking(
            Pose2D(0.0, 0.20, 0.0, 1.1),
            1.1,
        )
        self.assertEqual(output['state'], STATE_FAULT_STOP)
        self.assertEqual(output['reason'], 'trajectory_position_deviation')

    def _planner(self, config=None, footprint=None, primitives=None):
        return FirstTurnTrajectoryPlanner(
            config or self.planner_config,
            footprint or self.footprint,
            primitives or default_motion_primitives(),
        )

    def _safe_grid(self):
        return LocalMapBuilder(self.map_config).build(self._ring_points())

    def _safety(self):
        sector_status = {
            name: {
                'valid': True,
                'stale': False,
                'usable': True,
                'age_sec': 0.01,
                'point_count': 3,
                'coverage_point_count': 3,
                'coverage_bin_count': 2,
                'distance_m': 1.2,
            }
            for name in SECTOR_NAMES
        }
        return SafetyContext(
            cloud_age_sec=0.01,
            odom_age_sec=0.01,
            sector_status=sector_status,
            watchdog_ok=True,
            watchdog_age_sec=0.01,
            estop_triggered=False,
            estop_age_sec=0.01,
            roll_rad=0.0,
            pitch_rad=0.0,
            yaw_jump=False,
            linear_speed_mps=0.0,
            angular_speed_radps=0.0,
            stop_stable=True,
        )

    @staticmethod
    def _observation(front=1.2, left=0.25, right=0.25, turn=0.0):
        return {
            'front_distance_m': front,
            'left_open_distance_m': left,
            'left_distance_m': left,
            'right_distance_m': right,
            'turn_progress_rad': turn,
        }

    @staticmethod
    def _transform_corner(pose, corner_x, corner_y):
        cos_yaw = math.cos(pose.yaw_rad)
        sin_yaw = math.sin(pose.yaw_rad)
        return (
            pose.x_m + cos_yaw * corner_x - sin_yaw * corner_y,
            pose.y_m + sin_yaw * corner_x + cos_yaw * corner_y,
        )

    @staticmethod
    def _ring_points(excluded=None):
        excluded = excluded or set()
        points = []
        for angle_deg in range(-175, 180, 5):
            angle = math.radians(angle_deg)
            # 测试只排除指定后区；其他点提供局部地图的全向覆盖。
            if 'rear' in excluded and abs(angle_deg) >= 170:
                continue
            points.append((
                1.2 * math.cos(angle),
                1.2 * math.sin(angle),
                0.20,
            ))
        return tuple(points)


if __name__ == '__main__':
    unittest.main()
