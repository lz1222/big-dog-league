#!/usr/bin/env python3

"""B1 五区域提取器的纯逻辑测试，不依赖 ROS 或真机。"""

from pathlib import Path
import sys
import unittest


SCRIPTS_DIR = Path(__file__).resolve().parents[1] / 'scripts'
sys.path.insert(0, str(SCRIPTS_DIR))

from maze_perception_core import SectorExtractor  # noqa: E402


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
        # 同一点可同时支持斜前障碍和侧墙净距，但有效点只计一次。
        self.assertEqual(result['valid_points'], 6)

    def test_side_projection_requires_minimum_point_count(self):
        result = self.extractor.extract(
            [
                (0.60, 0.24, 0.10),
                (0.70, 0.25, 0.10),
            ]
        )

        self.assertEqual(result['counts']['left'], 2)
        self.assertIsNone(result['distances']['left'])

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
        extractor = SectorExtractor(
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
            side_projection_angle_min_deg=15.0,
            side_projection_angle_max_deg=60.0,
            side_projection_x_min=0.45,
            side_projection_x_max=1.50,
            side_min_points=3,
        )
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


if __name__ == '__main__':
    unittest.main()
