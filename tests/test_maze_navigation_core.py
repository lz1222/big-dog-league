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

    def test_side_space_insufficient_enters_fault_stop(self):
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
        output = self._step(unsafe)
        self.assertEqual(output['state'], STATE_FAULT_STOP)
        self.assertEqual(
            output['reason'],
            'corridor_side_clearance_unsafe',
        )

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
