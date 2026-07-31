#!/usr/bin/env python3

"""终端 D 中文操作提示纯逻辑测试，不依赖 ROS 或真机。"""

from pathlib import Path
import sys
import unittest


SCRIPTS_DIR = Path(__file__).resolve().parents[1] / 'scripts'
sys.path.insert(0, str(SCRIPTS_DIR))

from maze_operator_prompt_core import (  # noqa: E402
    build_operator_view,
    format_dashboard,
)


class MazeOperatorPromptCoreTest(unittest.TestCase):
    """验证每个关键状态只会给出保守且方向正确的人工提示。"""

    def test_missing_status_requires_hold(self):
        view = build_operator_view(None, float('inf'), 1.5)
        self.assertEqual(view.action_code, 'STOP')
        self.assertIn('保持静止', view.instruction)

    def test_stale_status_requires_immediate_stop(self):
        view = build_operator_view(
            self._payload('CORRIDOR_FOLLOW'),
            1.6,
            1.5,
        )
        self.assertEqual(view.action_code, 'STOP')
        self.assertIn('立即松开', view.instruction)
        self.assertIn('走廊跟随', view.state_label)

    def test_stale_status_preserves_last_fault_reason(self):
        payload = self._payload(
            'FAULT_STOP',
            'corner_side_clearance_unsafe',
        )
        view = build_operator_view(payload, 1.6, 1.5)

        self.assertIn('最后：故障锁止', view.state_label)
        self.assertIn('拐角侧向安全余量不足', view.reason_label)

    def test_corridor_follow_prompts_short_forward_step(self):
        view = self._view('CORRIDOR_FOLLOW', 'corridor_centering')
        self.assertEqual(view.action_code, 'FORWARD_STEP')
        self.assertIn('约 8cm', view.instruction)
        self.assertIn('禁止连续点动', view.instruction)

    def test_missing_confirmation_overrides_state_with_stop(self):
        view = self._view(
            'CORRIDOR_FOLLOW',
            'corridor_side_distance_missing_confirmation_1/2',
        )
        self.assertEqual(view.action_code, 'STOP')
        self.assertIn('保持静止', view.instruction)
        self.assertIn('暂停确认 1/2', view.reason_label)

    def test_unsafe_clearance_confirmation_requires_immediate_hold(self):
        view = self._view(
            'CORRIDOR_FOLLOW',
            'corridor_side_clearance_unsafe_confirmation_1/2',
        )
        self.assertEqual(view.action_code, 'STOP')
        self.assertIn('立即松开', view.instruction)
        self.assertIn('危险帧确认 1/2', view.reason_label)

    def test_manual_step_distance_is_configurable(self):
        view = build_operator_view(
            self._payload('CORRIDOR_FOLLOW', 'corridor_centering'),
            0.1,
            1.5,
            manual_step_distance_cm=6.5,
        )
        self.assertIn('约 6.5cm', view.instruction)

    def test_corner_approach_waiting_opening_requires_stop(self):
        view = self._view(
            'CORNER_APPROACH',
            'waiting_for_turn_opening',
        )
        self.assertEqual(view.action_code, 'STOP')
        self.assertIn('不要提前转向', view.instruction)

    def test_turn_direction_prompts_are_explicit(self):
        left = self._view('TURN_LEFT', 'left_turn_started')
        right = self._view('TURN_RIGHT', 'right_turn_started')
        self.assertEqual(left.action_code, 'TURN_LEFT')
        self.assertIn('左转', left.instruction)
        self.assertEqual(right.action_code, 'TURN_RIGHT')
        self.assertIn('右转', right.instruction)
        self.assertIn('禁止纯原地旋转', left.instruction)

    def test_fine_align_uses_turn_error_sign(self):
        payload = self._payload('TURN_FINE_ALIGN')
        payload['turn_error_deg'] = 7.0
        left = build_operator_view(payload, 0.1, 1.5)
        payload['turn_error_deg'] = -7.0
        right = build_operator_view(payload, 0.1, 1.5)
        self.assertIn('向左', left.action_title)
        self.assertIn('向右', right.action_title)

    def test_fine_align_in_tolerance_holds_position(self):
        payload = self._payload('TURN_FINE_ALIGN')
        payload['turn_error_deg'] = 2.0
        view = build_operator_view(payload, 0.1, 1.5)
        self.assertEqual(view.action_code, 'HOLD_ALIGN')
        self.assertIn('不要继续增加转角', view.instruction)

    def test_reverse_recovery_never_tells_operator_to_reverse(self):
        view = self._view(
            'REVERSE_RECOVERY',
            'reverse_recovery_diagnostic_only_no_rear_sector',
        )
        self.assertEqual(view.action_code, 'STOP')
        self.assertIn('禁止', view.instruction)
        self.assertIn('倒退', view.instruction)

    def test_fault_stop_is_latched_stop_instruction(self):
        view = self._view('FAULT_STOP', 'turn_timeout')
        self.assertEqual(view.action_code, 'STOP')
        self.assertIn('必须重启 B2', view.instruction)
        self.assertIn('转向超时', view.reason_label)

    def test_dashboard_marks_diagnostic_velocity_as_not_sent(self):
        payload = self._payload('TURN_LEFT', 'yaw_closed_loop_turn')
        view = build_operator_view(payload, 0.1, 1.5)
        dashboard = format_dashboard(payload, view, 0.1)
        self.assertIn('只读提示，不发送任何运动命令', dashboard)
        self.assertIn('诊断候选（不会发送）', dashboard)
        self.assertIn('已完成 0/5 次转向', dashboard)

    def _view(self, state, reason='test_reason'):
        return build_operator_view(
            self._payload(state, reason),
            0.1,
            1.5,
        )

    @staticmethod
    def _payload(state, reason='test_reason'):
        return {
            'state': state,
            'reason': reason,
            'route_index': 0,
            'route_total': 5,
            'route_complete': False,
            'expected_turn': 'LEFT',
            'state_age_sec': 0.5,
            'distances_m': {
                'front': 1.0,
                'left_front': 0.6,
                'right_front': 0.6,
                'left': 0.28,
                'right': 0.28,
            },
            'yaw_rad': 0.0,
            'turn_rad': 0.0,
            'turn_error_deg': 90.0,
            'center_error_m': 0.0,
            'moving_turn_sweep_safe': True,
            'in_place_rotation_fits_corridor': False,
            'cloud_age_sec': 0.05,
            'odom_age_sec': 0.01,
            'desired_vx': 0.08,
            'desired_wz': 0.30,
        }


if __name__ == '__main__':
    unittest.main()
