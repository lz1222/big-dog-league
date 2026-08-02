#!/usr/bin/env python3

"""Round15 实际 Odom 轨迹连续足迹回放纯逻辑测试。"""

from dataclasses import replace
import math
from pathlib import Path
import sys
import unittest


SCRIPTS_DIR = Path(__file__).resolve().parents[1] / 'scripts'
sys.path.insert(0, str(SCRIPTS_DIR))

from maze_first_turn_core import (  # noqa: E402
    DynamicFootprint,
    LocalMapBuilder,
    LocalMapConfig,
    PlannerConfig,
)
from maze_round15_replay_core import (  # noqa: E402
    ReplayMapSnapshot,
    ReplayOdomSample,
    analyze_round15_actual_trajectory,
)


class Round15ReplayCoreTest(unittest.TestCase):
    """验证回放结果来自墙几何和实际轨迹，而非固定角度答案。"""

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
        self.footprint = DynamicFootprint(
            footprint_front_m=0.10,
            footprint_rear_m=0.10,
            footprint_left_m=0.10,
            footprint_right_m=0.10,
            gait_sway_margin_m=0.0,
            cloud_uncertainty_margin_m=0.0,
            odom_uncertainty_margin_m=0.0,
            model_uncertainty_margin_m=0.0,
            stop_tail_margin_m=0.0,
            target_physical_clearance_m=0.05,
            margins_calibrated=False,
        )
        self.planner_config = PlannerConfig()

    def test_old_left_arc_is_rejected_before_contact(self):
        # 初始左墙是有限线段；旧实际轨迹沿半径0.5m左转并逐渐逼近它。
        wall_points = tuple(
            (-0.20 + index * 0.02, 0.30, 0.20)
            for index in range(51)
        )
        grid = LocalMapBuilder(self.map_config).build(wall_points)
        snapshots = (ReplayMapSnapshot(0.0, grid),)
        odom = []
        radius = 0.50
        for degree in range(0, 51):
            angle = math.radians(degree)
            odom.append(ReplayOdomSample(
                timestamp_sec=degree * 0.05,
                x_m=radius * math.sin(angle),
                y_m=radius * (1.0 - math.cos(angle)),
                yaw_rad=angle,
            ))

        result = analyze_round15_actual_trajectory(
            snapshots,
            odom,
            self.footprint,
            self.planner_config,
            contact_progress_deg=44.0,
        )
        self.assertEqual(result['result'], 'FAIL')
        self.assertEqual(result['gate_status'], 'DRY_RUN_PASS')
        self.assertTrue(result['geometry_alert_before_contact'])
        self.assertTrue(result['matching_geometry_alert_before_contact'])
        self.assertFalse(result['legacy_only_explanation'])
        alert = result['first_geometry_alert']
        self.assertLessEqual(alert['predicted_unsafe_yaw_deg'], 44.0)
        self.assertIsNotNone(alert['wall_segment'])
        self.assertIn(
            alert['dangerous_footprint_part'],
            ('left_side', 'front_left', 'rear_left'),
        )

    def test_earlier_right_risk_does_not_hide_later_left_contact_risk(self):
        right_wall = tuple(
            (-0.20 + index * 0.02, -0.14, 0.20)
            for index in range(51)
        )
        left_wall = tuple(
            (-0.20 + index * 0.02, 0.30, 0.20)
            for index in range(51)
        )
        grid = LocalMapBuilder(self.map_config).build(right_wall + left_wall)
        odom = []
        for degree in range(0, 51):
            angle = math.radians(degree)
            odom.append(ReplayOdomSample(
                degree * 0.05,
                0.50 * math.sin(angle),
                0.50 * (1.0 - math.cos(angle)),
                angle,
            ))
        result = analyze_round15_actual_trajectory(
            (ReplayMapSnapshot(0.0, grid),),
            odom,
            self.footprint,
            self.planner_config,
            contact_progress_deg=44.0,
        )
        self.assertIn(
            result['first_geometry_alert']['dangerous_footprint_part'],
            ('right_side', 'front_right', 'rear_right'),
        )
        self.assertIn(
            result['first_round15_matching_geometry_alert'][
                'dangerous_footprint_part'
            ],
            ('left_side', 'front_left', 'rear_left'),
        )
        self.assertEqual(result['gate_status'], 'DRY_RUN_PASS')

    def test_missing_map_remains_blocked(self):
        odom = (
            ReplayOdomSample(0.0, 0.0, 0.0, 0.0),
            ReplayOdomSample(1.0, 0.1, 0.0, math.radians(44.0)),
        )
        result = analyze_round15_actual_trajectory(
            (),
            odom,
            self.footprint,
            self.planner_config,
        )
        self.assertEqual(result['result'], 'UNKNOWN')
        self.assertEqual(result['gate_status'], 'BLOCKED')
        self.assertEqual(result['failure_reason'], 'no_map_snapshots')

    def test_yaw_jump_cannot_be_reported_as_valid_replay(self):
        grid = LocalMapBuilder(self.map_config).build(((1.0, 0.5, 0.2),))
        odom = (
            ReplayOdomSample(0.0, 0.0, 0.0, 0.0),
            ReplayOdomSample(0.1, 0.0, 0.0, math.radians(30.0)),
            ReplayOdomSample(0.2, 0.0, 0.0, math.radians(44.0)),
        )
        result = analyze_round15_actual_trajectory(
            (ReplayMapSnapshot(0.0, grid),),
            odom,
            self.footprint,
            self.planner_config,
        )
        self.assertEqual(result['result'], 'UNKNOWN')
        self.assertEqual(result['failure_reason'], 'yaw_jump_in_replay')


if __name__ == '__main__':
    unittest.main()
