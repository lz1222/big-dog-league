#!/usr/bin/env python3

"""B2 纯策略单元测试，不依赖 ROS、雷达或真机。"""

from dataclasses import replace
import math
from pathlib import Path
import sys
import unittest


SCRIPTS_DIR = Path(__file__).resolve().parents[1] / 'scripts'
sys.path.insert(0, str(SCRIPTS_DIR))

from maze_navigation_core import (  # noqa: E402
    DIRECTION_LEFT,
    DIRECTION_RIGHT,
    MazeNavigationPolicy,
    MazeObservation,
    MazePolicyConfig,
    STATE_CORRIDOR_FOLLOW,
    STATE_FAULT_STOP,
    STATE_FINISHED,
    STATE_TURN_LEFT,
    STATE_TURN_RIGHT,
)


ROUTE = (
    DIRECTION_LEFT,
    DIRECTION_LEFT,
    DIRECTION_RIGHT,
    DIRECTION_RIGHT,
    DIRECTION_LEFT,
)


class MazeNavigationPolicyTest(unittest.TestCase):
    """验证状态机、安全包络和固定路线的关键边界。"""

    def setUp(self):
        self.config = replace(
            MazePolicyConfig(),
            sensor_confirm_frames=2,
            corner_confirm_frames=2,
            turn_start_confirm_frames=1,
            turn_confirm_frames=2,
            reacquire_confirm_frames=2,
            recovery_confirm_frames=2,
            exit_confirm_frames=2,
        )
        self.policy = MazeNavigationPolicy(self.config, ROUTE)
        self.now = 0.0

    def test_footprint_cannot_rotate_in_place_in_corridor(self):
        self.assertFalse(
            self.policy.in_place_rotation_fits_corridor
        )
        self.assertGreater(
            self.policy.sweep_diameter_with_margin_m,
            self.config.corridor_width_m,
        )
        _, half_y = self.policy.rectangle_half_extents(
            math.radians(45.0)
        )
        self.assertGreater(2.0 * half_y, 0.57)

    def test_corridor_center_correction_sign(self):
        self._confirm_sensors()
        output = self._step(
            self._observation(
                turn_rad=0.0,
                distances={
                    'front': 2.0,
                    'left_front': 1.0,
                    'right_front': 1.0,
                    'left': 0.34,
                    'right': 0.23,
                },
            )
        )
        self.assertEqual(output['state'], STATE_CORRIDOR_FOLLOW)
        self.assertGreater(output['desired_wz'], 0.0)
        self.assertFalse(output['motion_output'])

    def test_corner_approach_follows_opposite_wall(self):
        """左侧开口波动时只用稳定右墙生成接近阶段修正量。"""
        self._confirm_sensors()
        near_opening = {
            'front': 0.72,
            'left_front': 0.78,
            'right_front': 0.55,
            'left': 0.25,
            'right': 0.245,
        }
        self._step(self._observation(
            turn_rad=0.0,
            distances=near_opening,
        ))
        first = self._step(self._observation(
            turn_rad=0.0,
            distances=near_opening,
        ))

        far_opening = dict(near_opening)
        far_opening['left'] = 0.70
        second = self._step(self._observation(
            turn_rad=0.0,
            distances=far_opening,
        ))

        self.assertEqual(first['state'], 'CORNER_APPROACH')
        self.assertEqual(first['center_reference'], 'right_wall')
        self.assertAlmostEqual(first['center_error_m'], 0.005)
        self.assertAlmostEqual(
            first['desired_wz'],
            second['desired_wz'],
        )

    def test_round10_opening_uses_right_wall_during_confirmation(self):
        """首弯开口出现后，正式状态切换前也不得由左侧远墙反向拉偏。"""
        self.policy = MazeNavigationPolicy(
            replace(
                self.config,
                side_target_m=0.25,
                corner_approach_distance_m=0.98,
                corner_confirm_frames=3,
            ),
            ROUTE,
        )
        self._confirm_sensors()
        opening = {
            'front': 0.92,
            'left_front': 1.00,
            'right_front': 0.65,
            'left': 0.34,
            'right': 0.268,
        }

        first = self._step(self._observation(
            turn_rad=0.0,
            distances=opening,
        ))
        opening['left'] = 0.79
        second = self._step(self._observation(
            turn_rad=0.0,
            distances=opening,
        ))
        third = self._step(self._observation(
            turn_rad=0.0,
            distances=opening,
        ))

        self.assertEqual(first['state'], STATE_CORRIDOR_FOLLOW)
        self.assertEqual(first['center_reference'], 'right_wall')
        self.assertAlmostEqual(first['center_error_m'], -0.018)
        self.assertLess(first['desired_wz'], 0.0)
        self.assertAlmostEqual(first['desired_wz'], second['desired_wz'])
        self.assertEqual(third['state'], 'CORNER_APPROACH')
        self.assertEqual(third['center_reference'], 'right_wall')

    def test_round15_coarse_step_starts_turn_before_overshoot(self):
        """26.5cm点动落入安全开口后应停止前进并连续确认左转。"""
        self.policy = MazeNavigationPolicy(
            replace(
                self.config,
                corner_confirm_frames=1,
                turn_start_confirm_frames=3,
                turn_start_distance_m=0.80,
                recovery_front_clear_m=0.85,
            ),
            ROUTE,
        )
        self._confirm_sensors()

        before_trigger = {
            'front': 0.82,
            'left_front': 0.92,
            'right_front': 0.74,
            'left': 0.55,
            'right': 0.29,
        }
        approaching = self._step(self._observation(
            turn_rad=0.0,
            distances=before_trigger,
        ))
        self.assertEqual(approaching['state'], 'CORNER_APPROACH')
        self.assertEqual(approaching['reason'], 'approaching_turn_start')
        self.assertGreater(approaching['desired_vx'], 0.0)

        # Round15实测落点满足移动扫掠包络；确认期间必须先输出零诊断速度。
        after_step = dict(before_trigger)
        after_step['front'] = 0.75
        outputs = []
        for _ in range(3):
            outputs.append(self._step(self._observation(
                turn_rad=0.0,
                distances=after_step,
            )))

        self.assertEqual(outputs[0]['turn_start_streak'], 1)
        self.assertEqual(outputs[0]['desired_vx'], 0.0)
        self.assertEqual(outputs[0]['desired_wz'], 0.0)
        self.assertEqual(outputs[-1]['state'], STATE_TURN_LEFT)

    def test_stale_after_start_latches_fault_stop(self):
        self._confirm_sensors()
        stale = self._observation(
            turn_rad=0.0,
            sensor_state='STALE',
        )
        output = self._step(stale)
        self.assertEqual(output['state'], STATE_FAULT_STOP)
        self.assertEqual(output['desired_vx'], 0.0)
        self.assertEqual(output['desired_wz'], 0.0)

        output = self._step(self._observation(turn_rad=0.0))
        self.assertEqual(output['state'], STATE_FAULT_STOP)

    def test_moving_turn_requires_open_side(self):
        safe = self._turn_distances(DIRECTION_LEFT)
        self.assertTrue(
            self.policy.moving_turn_sweep_safe(
                DIRECTION_LEFT,
                safe,
            )
        )
        blocked = dict(safe)
        blocked['left'] = 0.24
        self.assertFalse(
            self.policy.moving_turn_sweep_safe(
                DIRECTION_LEFT,
                blocked,
            )
        )

    def test_side_space_insufficient_stops_then_enters_fault(self):
        self._confirm_sensors()
        unsafe = self._observation(
            turn_rad=0.0,
            distances={
                'front': 2.0,
                'left_front': 1.0,
                'right_front': 1.0,
                'left': 0.17,
                'right': 0.40,
            },
        )
        pending = self._step(unsafe)
        self.assertEqual(pending['state'], STATE_CORRIDOR_FOLLOW)
        self.assertEqual(pending['desired_vx'], 0.0)
        self.assertEqual(pending['desired_wz'], 0.0)
        self.assertEqual(pending['side_unsafe_streak'], 1)
        self.assertEqual(
            pending['reason'],
            'corridor_side_clearance_unsafe_confirmation_1/2',
        )

        output = self._step(unsafe)
        self.assertEqual(output['state'], STATE_FAULT_STOP)
        self.assertEqual(
            output['reason'],
            'corridor_side_clearance_unsafe',
        )

    def test_single_unsafe_side_frame_recovers_without_fault(self):
        """步态摆动造成单帧低余量时先停住，下一安全帧恢复走廊状态。"""
        self._confirm_sensors()
        unsafe_distances = dict(
            self._observation(turn_rad=0.0).distances
        )
        unsafe_distances['right'] = 0.184

        pending = self._step(self._observation(
            turn_rad=0.0,
            distances=unsafe_distances,
        ))
        recovered = self._step(self._observation(turn_rad=0.0))

        self.assertEqual(pending['desired_vx'], 0.0)
        self.assertEqual(pending['side_unsafe_streak'], 1)
        self.assertEqual(recovered['state'], STATE_CORRIDOR_FOLLOW)
        self.assertEqual(recovered['reason'], 'corridor_centering')
        self.assertEqual(recovered['side_unsafe_streak'], 0)
        self.assertGreater(recovered['desired_vx'], 0.0)

    def test_missing_side_prevents_initial_sensor_confirmation(self):
        observation = self._observation(
            turn_rad=0.0,
            distances={
                'front': 2.0,
                'left_front': 1.0,
                'right_front': 1.0,
                'left': None,
                'right': None,
            },
        )

        output = self._step(observation)
        output = self._step(observation)

        self.assertEqual(output['state'], 'WAIT_SENSOR')
        self.assertEqual(
            output['reason'],
            'side_distance_confirmation_pending',
        )
        self.assertEqual(output['desired_vx'], 0.0)

    def test_missing_side_after_start_stops_then_latches_fault(self):
        self._confirm_sensors()
        observation = self._observation(
            turn_rad=0.0,
            distances={
                'front': 2.0,
                'left_front': 1.0,
                'right_front': 1.0,
                'left': None,
                'right': 0.285,
            },
        )

        pending = self._step(observation)

        self.assertEqual(pending['state'], STATE_CORRIDOR_FOLLOW)
        self.assertEqual(pending['desired_vx'], 0.0)
        self.assertEqual(pending['desired_wz'], 0.0)
        self.assertEqual(pending['side_missing_streak'], 1)
        self.assertEqual(
            pending['reason'],
            'corridor_side_distance_missing_confirmation_1/2',
        )

        output = self._step(observation)

        self.assertEqual(output['state'], STATE_FAULT_STOP)
        self.assertEqual(
            output['reason'],
            'corridor_side_distance_missing',
        )

    def test_transient_missing_side_recovers_without_fault(self):
        self._confirm_sensors()
        missing = self._observation(
            turn_rad=0.0,
            distances={
                'front': 2.0,
                'left_front': 1.0,
                'right_front': 1.0,
                'left': None,
                'right': 0.285,
            },
        )

        pending = self._step(missing)
        recovered = self._step(self._observation(turn_rad=0.0))

        self.assertEqual(pending['desired_vx'], 0.0)
        self.assertEqual(recovered['state'], STATE_CORRIDOR_FOLLOW)
        self.assertEqual(recovered['reason'], 'corridor_centering')
        self.assertEqual(recovered['side_missing_streak'], 0)
        self.assertGreater(recovered['desired_vx'], 0.0)

    def test_round12_thirteen_missing_side_frames_stop_then_recover(self):
        """真机第一弯13帧缺测期间始终停住，恢复后不得误锁止。"""
        self.policy = MazeNavigationPolicy(
            replace(self.config, side_missing_confirm_frames=15),
            ROUTE,
        )
        self._confirm_sensors()
        missing = self._observation(
            turn_rad=0.0,
            distances={
                'front': 0.60,
                'left_front': 0.80,
                'right_front': 0.80,
                'left': 0.285,
                'right': None,
            },
        )

        for expected_streak in range(1, 14):
            pending = self._step(missing)
            self.assertEqual(pending['state'], STATE_CORRIDOR_FOLLOW)
            self.assertEqual(
                pending['side_missing_streak'],
                expected_streak,
            )
            self.assertEqual(pending['desired_vx'], 0.0)
            self.assertEqual(pending['desired_wz'], 0.0)

        recovered = self._step(self._observation(turn_rad=0.0))
        self.assertEqual(recovered['state'], STATE_CORRIDOR_FOLLOW)
        self.assertEqual(recovered['side_missing_streak'], 0)
        self.assertGreater(recovered['desired_vx'], 0.0)

        for _ in range(15):
            fault = self._step(missing)

        self.assertEqual(fault['state'], STATE_FAULT_STOP)
        self.assertEqual(
            fault['reason'],
            'corridor_side_distance_missing',
        )

    def test_turn_clearance_missing_stops_then_latches_fault(self):
        self._confirm_sensors()
        turn_observation = self._observation(
            turn_rad=0.0,
            sensor_state='BLOCKED',
            distances=self._turn_distances(DIRECTION_LEFT),
        )
        self._step(turn_observation)
        self._step(turn_observation)

        missing_distances = self._turn_distances(DIRECTION_LEFT)
        missing_distances['right'] = None
        missing = self._observation(
            turn_rad=0.3,
            sensor_state='BLOCKED',
            distances=missing_distances,
        )
        pending = self._step(missing)
        fault = self._step(missing)

        self.assertEqual(pending['state'], STATE_TURN_LEFT)
        self.assertEqual(pending['desired_vx'], 0.0)
        self.assertEqual(
            pending['reason'],
            'turn_clearance_missing_confirmation_1/2',
        )
        self.assertEqual(fault['state'], STATE_FAULT_STOP)
        self.assertEqual(fault['reason'], 'turn_clearance_missing')

    def test_missing_opposite_side_never_authorizes_turn(self):
        distances = self._turn_distances(DIRECTION_LEFT)
        distances['left'] = None
        distances['right'] = None

        self.assertFalse(
            self.policy.moving_turn_sweep_safe(
                DIRECTION_LEFT,
                distances,
            )
        )

    def test_expected_opening_side_can_start_turn(self):
        self._confirm_sensors()
        distances = self._turn_distances(DIRECTION_LEFT)
        distances['left'] = None
        observation = self._observation(
            turn_rad=0.0,
            sensor_state='BLOCKED',
            distances=distances,
        )

        self._step(observation)
        output = self._step(observation)

        self.assertEqual(output['state'], STATE_TURN_LEFT)

    def test_turn_start_requires_consecutive_safe_sweep_frames(self):
        """单帧开口噪声不得锁存转向，安全包络中断后必须重新计数。"""
        self.policy = MazeNavigationPolicy(
            replace(
                self.config,
                corner_confirm_frames=1,
                turn_start_confirm_frames=3,
            ),
            ROUTE,
        )
        self._confirm_sensors()
        safe = self._observation(
            turn_rad=0.0,
            sensor_state='BLOCKED',
            distances=self._turn_distances(DIRECTION_LEFT),
        )

        first = self._step(safe)
        self.assertEqual(first['state'], 'CORNER_APPROACH')
        self.assertEqual(first['turn_start_streak'], 1)
        self.assertEqual(first['desired_vx'], 0.0)

        blocked_distances = self._turn_distances(DIRECTION_LEFT)
        blocked_distances['left'] = 0.40
        blocked = self._step(self._observation(
            turn_rad=0.0,
            sensor_state='BLOCKED',
            distances=blocked_distances,
        ))
        self.assertEqual(blocked['state'], 'CORNER_APPROACH')
        self.assertEqual(blocked['reason'], 'waiting_for_turn_opening')
        self.assertEqual(blocked['turn_start_streak'], 0)

        self._step(safe)
        self._step(safe)
        confirmed = self._step(safe)
        self.assertEqual(confirmed['state'], STATE_TURN_LEFT)

    def test_turn_open_hysteresis_bridges_lidar_scan_phase_dip(self):
        """严格开口确认后，数厘米扫描回落不应打断三帧启动确认。"""
        self.policy = MazeNavigationPolicy(
            replace(
                self.config,
                corner_confirm_frames=1,
                turn_start_confirm_frames=3,
                turn_open_distance_m=0.42,
                turn_open_hysteresis_m=0.06,
            ),
            ROUTE,
        )
        self._confirm_sensors()

        distances = self._turn_distances(DIRECTION_LEFT)
        distances['left'] = 0.43
        first = self._step(self._observation(
            turn_rad=0.0,
            sensor_state='BLOCKED',
            distances=distances,
        ))
        self.assertTrue(first['turn_open_latched'])
        self.assertEqual(first['turn_start_streak'], 1)

        for index, clearance in enumerate((0.38, 0.40), start=2):
            distances = dict(distances)
            distances['left'] = clearance
            output = self._step(self._observation(
                turn_rad=0.0,
                sensor_state='BLOCKED',
                distances=distances,
            ))
            if index < 3:
                self.assertTrue(output['turn_start_sweep_safe'])
                self.assertEqual(output['turn_start_streak'], index)

        self.assertEqual(output['state'], STATE_TURN_LEFT)

    def test_turn_open_hysteresis_resets_below_exit_threshold(self):
        """开口跌破退出线时必须清除锁存，不能继续累计启动帧。"""
        self.policy = MazeNavigationPolicy(
            replace(
                self.config,
                corner_confirm_frames=1,
                turn_start_confirm_frames=3,
                turn_open_distance_m=0.42,
                turn_open_hysteresis_m=0.06,
            ),
            ROUTE,
        )
        self._confirm_sensors()
        distances = self._turn_distances(DIRECTION_LEFT)
        distances['left'] = 0.43
        self._step(self._observation(
            turn_rad=0.0,
            sensor_state='BLOCKED',
            distances=distances,
        ))

        distances['left'] = 0.35
        output = self._step(self._observation(
            turn_rad=0.0,
            sensor_state='BLOCKED',
            distances=distances,
        ))

        self.assertFalse(output['turn_open_latched'])
        self.assertEqual(output['turn_start_streak'], 0)
        self.assertEqual(output['reason'], 'waiting_for_turn_opening')

    def test_republished_cloud_does_not_confirm_turn_start(self):
        """同一真机点云的周期重发不能伪装成三帧连续开口。"""
        self.policy = MazeNavigationPolicy(
            replace(
                self.config,
                corner_confirm_frames=1,
                turn_start_confirm_frames=3,
            ),
            ROUTE,
        )
        self._confirm_sensors()
        transient = self._observation(
            turn_rad=0.0,
            sensor_state='CLEAR',
            cloud_sequence=100,
            distances={
                'front': 0.539,
                'left_front': 0.746,
                'right_front': None,
                'left': 0.556,
                'right': 0.252,
            },
        )

        first = self._step(transient)
        duplicate_1 = self._step(transient)
        duplicate_2 = self._step(transient)

        self.assertEqual(first['state'], 'CORNER_APPROACH')
        self.assertEqual(first['turn_start_streak'], 1)
        self.assertEqual(duplicate_1['turn_start_streak'], 1)
        self.assertEqual(duplicate_2['turn_start_streak'], 1)

        recovered = self._step(self._observation(
            turn_rad=0.0,
            cloud_sequence=101,
            distances={
                'front': 0.723,
                'left_front': 0.810,
                'right_front': 0.540,
                'left': 0.472,
                'right': 0.247,
            },
        ))
        self.assertEqual(recovered['state'], 'CORNER_APPROACH')
        self.assertEqual(recovered['turn_start_streak'], 0)
        self.assertEqual(recovered['reason'], 'approaching_turn_start')

    def test_cloud_sequence_regression_latches_fault(self):
        """B1序号倒退表示发布端重启或乱序，运行中必须锁止。"""
        self._confirm_sensors()
        self._step(self._observation(
            turn_rad=0.0,
            cloud_sequence=10,
        ))
        output = self._step(self._observation(
            turn_rad=0.0,
            cloud_sequence=9,
        ))
        self.assertEqual(output['state'], STATE_FAULT_STOP)
        self.assertEqual(output['reason'], 'cloud_sequence_regressed')

    def test_active_turn_rejects_wall_end_inside_full_sweep(self):
        """粗转时墙端虽高于机身半宽，进入整机扫掠区仍必须停止。"""
        self._confirm_sensors()
        start = self._observation(
            turn_rad=0.0,
            sensor_state='BLOCKED',
            distances=self._turn_distances(DIRECTION_LEFT),
        )
        self._step(start)
        started = self._step(start)
        self.assertEqual(started['state'], STATE_TURN_LEFT)

        wall_end = self._turn_distances(DIRECTION_LEFT)
        wall_end.update({
            'front': 0.72,
            'left_front': 0.78,
            'left': 0.25,
            'right': 0.245,
        })
        output = self._step(self._observation(
            turn_rad=0.10,
            sensor_state='BLOCKED',
            distances=wall_end,
        ))

        self.assertEqual(output['state'], STATE_TURN_LEFT)
        self.assertEqual(
            output['reason'],
            'turn_sweep_unsafe_confirmation_1/2',
        )
        self.assertEqual(output['desired_vx'], 0.0)
        self.assertEqual(output['desired_wz'], 0.0)
        self.assertFalse(output['turn_start_sweep_safe'])
        self.assertFalse(output['active_turn_clearance_safe'])
        self.assertFalse(output['moving_turn_sweep_safe'])
        self.assertAlmostEqual(output['turn_progress_rad'], 0.10)
        self.assertAlmostEqual(
            output['turn_progress_deg'],
            math.degrees(0.10),
        )

    def test_round15_mid_turn_inner_wall_latches_fault(self):
        """Round15在45度附近的内侧机身碰板数据必须触发扫掠保护。"""
        self._confirm_sensors()
        start = self._observation(
            turn_rad=0.0,
            sensor_state='BLOCKED',
            distances=self._turn_distances(DIRECTION_LEFT),
        )
        self._step(start)
        self._step(start)

        collision_geometry = {
            'front': 0.487,
            'left_front': 0.647,
            'right_front': 0.480,
            'left': 0.393,
            'right': 0.263,
        }
        observation = self._observation(
            turn_rad=math.radians(44.07),
            sensor_state='BLOCKED',
            distances=collision_geometry,
        )
        pending = self._step(observation)
        fault = self._step(observation)

        self.assertGreater(
            pending['active_turn_required_side_clearance_m'],
            collision_geometry['left'],
        )
        self.assertEqual(pending['desired_vx'], 0.0)
        self.assertEqual(pending['desired_wz'], 0.0)
        self.assertEqual(
            pending['reason'],
            'turn_sweep_unsafe_confirmation_1/2',
        )
        self.assertEqual(fault['state'], STATE_FAULT_STOP)
        self.assertEqual(fault['reason'], 'turn_sweep_unsafe')

    def test_turn_sweep_loss_stops_then_latches_fault(self):
        """转向中矩形扫掠包络丢失时首帧停住，连续第二帧锁止。"""
        self._confirm_sensors()
        turn_observation = self._observation(
            turn_rad=0.0,
            sensor_state='BLOCKED',
            distances=self._turn_distances(DIRECTION_LEFT),
        )
        self._step(turn_observation)
        output = self._step(turn_observation)
        self.assertEqual(output['state'], STATE_TURN_LEFT)

        blocked_distances = self._turn_distances(DIRECTION_LEFT)
        blocked_distances['left'] = 0.17
        blocked = self._observation(
            turn_rad=0.1,
            sensor_state='BLOCKED',
            distances=blocked_distances,
        )
        pending = self._step(blocked)
        fault = self._step(blocked)

        self.assertEqual(pending['state'], STATE_TURN_LEFT)
        self.assertEqual(pending['desired_vx'], 0.0)
        self.assertEqual(pending['desired_wz'], 0.0)
        self.assertEqual(
            pending['reason'],
            'turn_sweep_unsafe_confirmation_1/2',
        )
        self.assertEqual(fault['state'], STATE_FAULT_STOP)
        self.assertEqual(fault['reason'], 'turn_sweep_unsafe')

    def test_turn_timeout_enters_fault_stop(self):
        self.policy = MazeNavigationPolicy(
            replace(self.config, turn_timeout_sec=0.5),
            ROUTE,
        )
        self._confirm_sensors()
        turn_observation = self._observation(
            turn_rad=0.0,
            sensor_state='BLOCKED',
            distances=self._turn_distances(DIRECTION_LEFT),
        )
        self._step(turn_observation)
        output = self._step(turn_observation)
        self.assertEqual(output['state'], STATE_TURN_LEFT)

        self.now += 0.60
        output = self.policy.update(turn_observation, self.now)
        self.assertEqual(output['state'], STATE_FAULT_STOP)
        self.assertEqual(output['reason'], 'turn_timeout')

    def test_short_corridor_can_reacquire_above_emergency_distance(self):
        """紧凑迷宫转后前距不足62cm时，仍可按居中和Yaw进入下一段。"""
        self._confirm_sensors()
        start = self._observation(
            turn_rad=0.0,
            sensor_state='BLOCKED',
            distances=self._turn_distances(DIRECTION_LEFT),
        )
        self._step(start)
        self._step(start)

        target = 0.5 * math.pi
        self._step(self._observation(
            turn_rad=target,
            sensor_state='BLOCKED',
            distances=self._turn_distances(DIRECTION_LEFT),
        ))
        self._step(self._observation(
            turn_rad=target,
            sensor_state='BLOCKED',
            distances=self._turn_distances(DIRECTION_LEFT),
        ))

        short_corridor = self._observation(
            turn_rad=target,
            distances={
                'front': 0.50,
                'left_front': 0.62,
                'right_front': 0.62,
                'left': 0.28,
                'right': 0.28,
            },
        )
        self._step(short_corridor)
        output = self._step(short_corridor)

        self.assertEqual(output['route_index'], 1)
        self.assertEqual(output['state'], STATE_CORRIDOR_FOLLOW)

        output = self._step(short_corridor)
        self.assertEqual(output['state'], 'CORNER_APPROACH')

    def test_reacquire_front_emergency_latches_fault(self):
        """重捕获期间前距跌破紧急线必须锁止，不得输出向前值。"""
        self._confirm_sensors()
        start = self._observation(
            turn_rad=0.0,
            sensor_state='BLOCKED',
            distances=self._turn_distances(DIRECTION_LEFT),
        )
        self._step(start)
        self._step(start)
        target = 0.5 * math.pi
        for _ in range(2):
            self._step(self._observation(
                turn_rad=target,
                sensor_state='BLOCKED',
                distances=self._turn_distances(DIRECTION_LEFT),
            ))

        distances = self._turn_distances(DIRECTION_LEFT)
        distances['front'] = 0.42
        output = self._step(self._observation(
            turn_rad=target,
            sensor_state='BLOCKED',
            distances=distances,
        ))

        self.assertEqual(output['state'], STATE_FAULT_STOP)
        self.assertEqual(
            output['reason'],
            'reacquire_front_clearance_unsafe',
        )

    def test_nominal_five_turn_route_reaches_finished(self):
        self._confirm_sensors()
        current_turn = 0.0

        for index, direction in enumerate(ROUTE):
            turn_observation = self._observation(
                turn_rad=current_turn,
                sensor_state='BLOCKED',
                distances=self._turn_distances(direction),
            )
            self._step(turn_observation)
            output = self._step(turn_observation)
            expected_state = (
                STATE_TURN_LEFT
                if direction == DIRECTION_LEFT
                else STATE_TURN_RIGHT
            )
            self.assertEqual(
                output['state'],
                expected_state,
                msg=f'route turn {index + 1}',
            )

            sign = 1.0 if direction == DIRECTION_LEFT else -1.0
            target = current_turn + sign * 0.5 * math.pi
            for fraction in (0.45, 0.80, 0.94, 1.0, 1.0):
                output = self._step(
                    self._observation(
                        turn_rad=(
                            current_turn
                            + sign * fraction * 0.5 * math.pi
                        ),
                        sensor_state='BLOCKED',
                        distances=self._turn_distances(direction),
                    )
                )
            current_turn = target

            corridor = self._observation(
                turn_rad=current_turn,
            )
            self._step(corridor)
            output = self._step(corridor)
            self.assertEqual(
                output['route_index'],
                index + 1,
            )

        exit_observation = self._observation(
            turn_rad=current_turn,
            distances={
                'front': 2.2,
                'left_front': 1.6,
                'right_front': 1.6,
                'left': 1.2,
                'right': 1.2,
            },
        )
        self._step(exit_observation)
        output = self._step(exit_observation)
        self.assertEqual(output['state'], STATE_FINISHED)
        self.assertEqual(output['desired_vx'], 0.0)
        self.assertEqual(output['desired_wz'], 0.0)

        output = self.policy.mark_input_stale(
            'input_stopped_after_finish',
            self.now + 1.0,
        )
        self.assertEqual(output['state'], STATE_FINISHED)
        output = self.policy.update(
            self._observation(
                turn_rad=current_turn,
                sensor_state='STALE',
            ),
            self.now + 2.0,
        )
        self.assertEqual(output['state'], STATE_FINISHED)

    def _confirm_sensors(self):
        observation = self._observation(turn_rad=0.0)
        self._step(observation)
        output = self._step(observation)
        self.assertEqual(output['state'], STATE_CORRIDOR_FOLLOW)

    def _step(self, observation, delta_sec=0.10):
        self.now += delta_sec
        return self.policy.update(observation, self.now)

    @staticmethod
    def _observation(
        turn_rad,
        sensor_state='CLEAR',
        distances=None,
        cloud_sequence=None,
    ):
        if distances is None:
            distances = {
                'front': 2.20,
                'left_front': 1.10,
                'right_front': 1.10,
                'left': 0.285,
                'right': 0.285,
            }
        return MazeObservation(
            sensor_state=sensor_state,
            cloud_age_sec=0.01,
            odom_age_sec=0.01,
            yaw_rad=math.atan2(
                math.sin(turn_rad),
                math.cos(turn_rad),
            ),
            turn_rad=turn_rad,
            distances=distances,
            cloud_sequence=cloud_sequence,
        )

    @staticmethod
    def _turn_distances(direction):
        distances = {
            'front': 0.60,
            'left_front': 0.80,
            'right_front': 0.80,
            'left': 0.30,
            'right': 0.30,
        }
        if direction == DIRECTION_LEFT:
            distances['left_front'] = 1.00
            distances['left'] = 1.20
        else:
            distances['right_front'] = 1.00
            distances['right'] = 1.20
        return distances


if __name__ == '__main__':
    unittest.main()
