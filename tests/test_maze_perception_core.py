#!/usr/bin/env python3

"""B1 五区域提取器的纯逻辑测试，不依赖 ROS 或真机。"""

from pathlib import Path
import sys
import unittest


SCRIPTS_DIR = Path(__file__).resolve().parents[1] / 'scripts'
sys.path.insert(0, str(SCRIPTS_DIR))

from maze_perception_core import (  # noqa: E402
    SectorExtractor,
    SideDistanceStabilizer,
)


class SectorExtractorTest(unittest.TestCase):
    """覆盖真机侧墙投影、点数门槛和原有正侧方输入。"""

    def setUp(self):
        self.extractor = SectorExtractor(
            z_min=-0.15,
            z_max=0.50,
            body_x_min=-0.45,
            body_x_max=0.45,
            body_y_min=-0.25,
            body_y_max=0.25,
            front_angle_deg=45.0,
            min_range=0.05,
            front_max_range=3.0,
            side_max_range=2.0,
            distance_percentile=10.0,
            side_projection_angle_min_deg=15.0,
            side_projection_angle_max_deg=60.0,
            side_projection_x_min=0.45,
            side_projection_x_max=1.50,
            side_min_points=3,
        )

    def test_diagonal_wall_points_project_to_lateral_clearance(self):
        points = [
            (0.50, 0.24, 0.10),
            (0.60, 0.25, 0.10),
            (0.70, 0.26, 0.10),
            (0.50, -0.23, 0.10),
            (0.60, -0.24, 0.10),
            (0.70, -0.25, 0.10),
        ]

        result = self.extractor.extract(points)

        self.assertAlmostEqual(
            result['distances']['left'],
            0.242,
            places=6,
        )
        self.assertAlmostEqual(
            result['distances']['right'],
            0.232,
            places=6,
        )
        self.assertEqual(result['counts']['left'], 3)
        self.assertEqual(result['counts']['right'], 3)
        self.assertEqual(result['sources']['left'], 'projected')
        self.assertEqual(result['sources']['right'], 'projected')
        # 同一点可同时支持斜前障碍和侧墙净距，但有效点只计一次。
        self.assertEqual(result['valid_points'], 6)

    def test_side_projection_requires_minimum_point_count(self):
        result = self.extractor.extract(
            [
                (0.60, 0.24, 0.10),
                (0.70, 0.25, 0.10),
            ]
        )

        self.assertEqual(result['counts']['left'], 0)
        self.assertIsNone(result['distances']['left'])
        self.assertEqual(result['sources']['left'], 'none')

    def test_direct_side_points_remain_supported(self):
        result = self.extractor.extract(
            [
                (0.0, 0.80, 0.10),
                (0.0, 0.81, 0.10),
                (0.0, 0.82, 0.10),
            ]
        )

        self.assertAlmostEqual(
            result['distances']['left'],
            0.802,
            places=6,
        )
        self.assertEqual(result['counts']['left'], 3)
        self.assertEqual(result['sources']['left'], 'direct')

    def test_narrow_front_angle_keeps_true_side_sector(self):
        """缩小正前角宽后，接近90度的真实侧墙仍必须可用。"""
        extractor = self._narrow_front_extractor()
        result = extractor.extract(
            [
                (0.00, 0.28, 0.10),
                (0.03, 0.28, 0.10),
                (0.06, 0.28, 0.10),
                (0.00, -0.27, 0.10),
                (0.03, -0.27, 0.10),
                (0.06, -0.27, 0.10),
            ]
        )

        self.assertAlmostEqual(result['distances']['left'], 0.28)
        self.assertAlmostEqual(result['distances']['right'], 0.27)
        self.assertEqual(result['sources']['left'], 'direct')
        self.assertEqual(result['sources']['right'], 'direct')

    def test_front_wall_edges_are_not_projected_as_side_clearance(self):
        """固定x的近前挡板边缘不能生成约14cm的假左右侧距。"""
        extractor = self._narrow_front_extractor()
        result = extractor.extract(
            [
                (0.46, 0.13, 0.10),
                (0.47, 0.14, 0.10),
                (0.48, 0.15, 0.10),
                (0.46, -0.13, 0.10),
                (0.47, -0.14, 0.10),
                (0.48, -0.15, 0.10),
            ]
        )

        self.assertIsNone(result['distances']['left'])
        self.assertIsNone(result['distances']['right'])
        self.assertEqual(result['sources']['left'], 'none')
        self.assertEqual(result['sources']['right'], 'none')

    def test_front_wall_cluster_does_not_pollute_projected_side_wall(self):
        """前墙和斜视侧墙混合时，只能选择沿x延伸的侧墙簇。"""
        extractor = self._narrow_front_extractor()
        result = extractor.extract(
            [
                (0.46, 0.13, 0.10),
                (0.47, 0.14, 0.10),
                (0.48, 0.15, 0.10),
                (0.50, 0.24, 0.10),
                (0.65, 0.25, 0.10),
                (0.80, 0.26, 0.10),
            ]
        )

        self.assertGreater(result['distances']['left'], 0.23)
        self.assertLess(result['distances']['left'], 0.27)
        self.assertEqual(result['sources']['left'], 'projected')

    def test_direct_side_points_override_front_wall_projection(self):
        """前墙和侧墙同时出现时，安全判断必须采用真实正侧回波。"""
        extractor = self._narrow_front_extractor()
        result = extractor.extract(
            [
                (0.46, 0.13, 0.10),
                (0.47, 0.14, 0.10),
                (0.48, 0.15, 0.10),
                (0.00, 0.28, 0.10),
                (0.03, 0.28, 0.10),
                (0.06, 0.28, 0.10),
            ]
        )

        self.assertAlmostEqual(result['distances']['left'], 0.28)
        self.assertEqual(result['counts']['left'], 3)
        self.assertEqual(result['sources']['left'], 'direct')

    def test_projection_cannot_start_inside_body_filter(self):
        with self.assertRaises(ValueError):
            SectorExtractor(
                z_min=-0.15,
                z_max=0.50,
                body_x_min=-0.45,
                body_x_max=0.45,
                body_y_min=-0.25,
                body_y_max=0.25,
                front_angle_deg=45.0,
                min_range=0.05,
                front_max_range=3.0,
                side_max_range=2.0,
                distance_percentile=10.0,
                side_projection_x_min=0.40,
            )

    def test_narrow_front_sector_rejects_corridor_side_walls(self):
        """20度正前扇区不能把57cm通道的侧墙误判为前挡板。"""
        extractor = self._narrow_front_extractor()
        points = [
            # 正前挡板在base_link前方1.20m。
            (1.20, -0.10, 0.10),
            (1.20, 0.00, 0.10),
            (1.20, 0.10, 0.10),
            # 两侧墙距中线约0.24m，不得进入正负10度的front。
            (0.50, 0.24, 0.10),
            (0.60, 0.24, 0.10),
            (0.70, 0.24, 0.10),
            (0.50, -0.24, 0.10),
            (0.60, -0.24, 0.10),
            (0.70, -0.24, 0.10),
        ]

        result = extractor.extract(points)

        self.assertEqual(result['counts']['front'], 3)
        self.assertGreater(result['distances']['front'], 1.19)
        self.assertLess(result['distances']['front'], 1.21)
        self.assertEqual(result['counts']['left'], 3)
        self.assertEqual(result['counts']['right'], 3)

    @staticmethod
    def _narrow_front_extractor():
        """创建与真机B1一致的20度正前扇区提取器。"""
        return SectorExtractor(
            z_min=-0.15,
            z_max=0.50,
            body_x_min=-0.45,
            body_x_max=0.45,
            body_y_min=-0.25,
            body_y_max=0.25,
            front_angle_deg=20.0,
            min_range=0.05,
            front_max_range=3.0,
            side_max_range=2.0,
            distance_percentile=10.0,
            diagonal_angle_max_deg=30.0,
            side_angle_max_deg=120.0,
            side_projection_angle_min_deg=15.0,
            side_projection_angle_max_deg=60.0,
            side_projection_x_min=0.45,
            side_projection_x_max=1.50,
            side_projection_min_x_span=0.12,
            side_projection_lateral_tolerance=0.04,
            side_min_points=3,
        )


class SideDistanceStabilizerTest(unittest.TestCase):
    """验证缺测和突然变远都需短时确认，近障碍则立即生效。"""

    def setUp(self):
        self.stabilizer = SideDistanceStabilizer(hold_frames=2)

    def test_alternating_side_dropout_uses_explicit_hold_source(self):
        first = self.stabilizer.update(
            self._extraction(0.25, 0.24, 'direct', 'projected')
        )
        second = self.stabilizer.update(
            self._extraction(None, 0.24, 'none', 'direct')
        )
        third = self.stabilizer.update(
            self._extraction(0.26, None, 'direct', 'none')
        )

        self.assertEqual(first['hold_frames']['left'], 0)
        self.assertAlmostEqual(second['distances']['left'], 0.25)
        self.assertEqual(second['sources']['left'], 'held_direct')
        self.assertEqual(second['hold_frames']['left'], 1)
        self.assertAlmostEqual(third['distances']['right'], 0.24)
        self.assertEqual(third['sources']['right'], 'held_direct')

    def test_hold_expires_after_configured_frames(self):
        self.stabilizer.update(
            self._extraction(0.25, 0.24, 'direct', 'direct')
        )
        held_one = self.stabilizer.update(
            self._extraction(None, 0.24, 'none', 'direct')
        )
        held_two = self.stabilizer.update(
            self._extraction(None, 0.24, 'none', 'direct')
        )
        expired = self.stabilizer.update(
            self._extraction(None, 0.24, 'none', 'direct')
        )

        self.assertIsNotNone(held_one['distances']['left'])
        self.assertIsNotNone(held_two['distances']['left'])
        self.assertIsNone(expired['distances']['left'])
        self.assertEqual(expired['sources']['left'], 'none')

    def test_reset_forbids_using_measurement_from_before_stale_gap(self):
        self.stabilizer.update(
            self._extraction(0.25, 0.24, 'direct', 'direct')
        )
        self.stabilizer.reset()
        output = self.stabilizer.update(
            self._extraction(None, 0.24, 'none', 'direct')
        )

        self.assertIsNone(output['distances']['left'])
        self.assertEqual(output['sources']['left'], 'none')

    def test_sudden_farther_measurement_requires_confirmation(self):
        self.stabilizer.update(
            self._extraction(0.25, 0.24, 'direct', 'direct')
        )
        held_one = self.stabilizer.update(
            self._extraction(0.70, 0.24, 'direct', 'direct')
        )
        held_two = self.stabilizer.update(
            self._extraction(0.72, 0.24, 'direct', 'direct')
        )
        accepted = self.stabilizer.update(
            self._extraction(0.71, 0.24, 'direct', 'direct')
        )

        self.assertAlmostEqual(held_one['distances']['left'], 0.25)
        self.assertEqual(held_one['sources']['left'], 'held_rise_direct')
        self.assertEqual(held_one['hold_frames']['left'], 1)
        self.assertAlmostEqual(held_two['distances']['left'], 0.25)
        self.assertEqual(held_two['hold_frames']['left'], 2)
        self.assertAlmostEqual(accepted['distances']['left'], 0.71)
        self.assertEqual(accepted['sources']['left'], 'direct')

    def test_closer_measurement_is_accepted_immediately(self):
        self.stabilizer.update(
            self._extraction(0.60, 0.55, 'direct', 'direct')
        )
        output = self.stabilizer.update(
            self._extraction(0.25, 0.24, 'projected', 'projected')
        )

        self.assertAlmostEqual(output['distances']['left'], 0.25)
        self.assertAlmostEqual(output['distances']['right'], 0.24)
        self.assertEqual(output['sources']['left'], 'projected')
        self.assertEqual(output['hold_frames']['left'], 0)

    @staticmethod
    def _extraction(left, right, left_source, right_source):
        return {
            'distances': {
                'front': 1.0,
                'left_front': 0.6,
                'right_front': 0.6,
                'left': left,
                'right': right,
            },
            'counts': {
                'front': 10,
                'left_front': 5,
                'right_front': 5,
                'left': 3 if left is not None else 0,
                'right': 3 if right is not None else 0,
            },
            'sources': {
                'front': 'angular',
                'left_front': 'angular',
                'right_front': 'angular',
                'left': left_source,
                'right': right_source,
            },
            'valid_points': 20,
            'finite_points': 30,
            'total_points': 30,
        }


if __name__ == '__main__':
    unittest.main()
