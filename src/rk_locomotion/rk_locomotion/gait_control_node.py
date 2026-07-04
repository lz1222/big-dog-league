#!/usr/bin/env python3

import json
import math
import struct
import threading
import time
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import Twist
from rclpy.action import ActionServer
from rclpy.action.server import CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Bool, String

from rk_interfaces.action import ExecuteMotion


STATUS_IDLE = 'IDLE'
STATUS_RUNNING = 'RUNNING'
STATUS_DONE = 'DONE'
STATUS_FAILED = 'FAILED'
STATUS_TIMEOUT = 'TIMEOUT'
STATUS_UNSTABLE = 'UNSTABLE'
STATUS_EMERGENCY_STOP = 'EMERGENCY_STOP'


@dataclass(frozen=True)
class CommandResult:
    success: bool
    status: str
    message: str


@dataclass(frozen=True)
class ObstacleStep:
    name: str
    vx: float
    wz: float
    duration_sec: float
    front_target_m: object = None


@dataclass(frozen=True)
class DistanceSnapshot:
    front_m: object
    left_m: object
    right_m: object
    has_fresh_source: bool


class RobotMotionAdapter:
    """Thin wrapper around the existing cmd_vel to Unitree bridge."""

    def __init__(self, node, cmd_vel_topic, stop_count, stop_period_sec):
        self._node = node
        self._logger = node.get_logger()
        self._publisher = node.create_publisher(Twist, cmd_vel_topic, 10)
        self._cmd_vel_topic = cmd_vel_topic
        self._stop_count = max(1, int(stop_count))
        self._stop_period_sec = max(0.0, float(stop_period_sec))
        self._warned_body_height = False
        self._warned_recovery = False
        self._warned_lateral = False

        self._logger.info(
            'RobotMotionAdapter using cmd_vel topic: '
            f'{self._cmd_vel_topic}'
        )

    def stop(self, reason='stop requested', log_level='info'):
        stop_cmd = Twist()
        for index in range(self._stop_count):
            self._publisher.publish(stop_cmd)
            if index + 1 < self._stop_count and self._stop_period_sec > 0.0:
                time.sleep(self._stop_period_sec)
        self._log_stop(reason, log_level)

    def _log_stop(self, reason, log_level):
        message = f'STOP sent through cmd_vel: {reason}'
        if log_level == 'debug':
            self._logger.debug(message)
        elif log_level == 'warn':
            self._logger.warn(message)
        else:
            self._logger.info(message)

    def recovery_stand(self):
        self.stop('recovery_stand requested', log_level='info')
        if not self._warned_recovery:
            self._logger.warn(
                'recovery_stand is not wired to a Unitree StandUp/'
                'RecoveryStand API yet; keeping TODO adapter placeholder.'
            )
            self._warned_recovery = True
        return False, 'recovery_stand backend is not available yet'

    def hold_stable(self):
        self.stop('hold_stable zero velocity', log_level='debug')

    def move(self, vx, vy, wz):
        if vy != 0.0 and not self._warned_lateral:
            self._logger.warn(
                'nonzero vy requested, but the existing cmd_vel bridge only '
                'maps linear.x and angular.z to Unitree Sport Move. TODO: '
                'wire lateral velocity when the low-level adapter supports it.'
            )
            self._warned_lateral = True

        cmd = Twist()
        cmd.linear.x = float(vx)
        cmd.linear.y = float(vy)
        cmd.angular.z = float(wz)
        self._publisher.publish(cmd)

    def set_body_height(self, height):
        if not self._warned_body_height:
            self._logger.warn(
                'body height adjustment is not wired to a Unitree body-height '
                'API yet; keeping TODO adapter placeholder.'
            )
            self._warned_body_height = True
        return False, f'body_height backend is not available: {height}'


class GaitControlNode(Node):
    """Basic JSON gait command node with lock, timeout, and speed limiting."""

    COMMANDS = {
        'STOP',
        'OBSTACLE_STOP',
        'RECOVERY_STAND',
        'HOLD_STABLE',
        'LOW_SPEED_MOVE',
        'TURN_IN_PLACE',
        'BODY_HEIGHT_ADJUST',
        'SPEED_LIMIT',
        'WAIT_NAVIGATION_SEGMENT',
        'JUMP_START_OBSTACLE',
        'JUMP_END_OBSTACLE',
        'PRACTICAL_OBSTACLE_ZONE',
        'AVOID_ZONE',
        'ENTER_OBSTACLE_ZONE',
        'OBSTACLE_FORWARD_SLOW',
        'OBSTACLE_TURN_LEFT',
        'OBSTACLE_TURN_RIGHT',
        'OBSTACLE_SIDE_ADJUST',
        'EXIT_OBSTACLE_ZONE',
    }

    def __init__(self):
        super().__init__('gait_control_node')
        self.callback_group = ReentrantCallbackGroup()

        self._declare_parameters()
        self.cmd_vel_topic = self._string_parameter('cmd_vel_topic')
        self.command_json_topic = self._string_parameter('command_json_topic')
        self.motion_action_name = self._string_parameter('motion_action_name')

        self.max_vx = self._positive_float_parameter('default_max_vx')
        self.max_vy = self._positive_float_parameter('default_max_vy')
        self.max_wz = self._positive_float_parameter('default_max_wz')
        self.hard_max_vx = self._positive_float_parameter('hard_max_vx')
        self.hard_max_vy = self._positive_float_parameter('hard_max_vy')
        self.hard_max_wz = self._positive_float_parameter('hard_max_wz')
        self.max_action_duration_sec = self._positive_float_parameter(
            'max_action_duration_sec'
        )
        self.default_hold_duration_sec = self._positive_float_parameter(
            'default_hold_duration_sec'
        )
        self.default_move_duration_sec = self._positive_float_parameter(
            'default_move_duration_sec'
        )
        self.default_turn_duration_sec = self._positive_float_parameter(
            'default_turn_duration_sec'
        )
        self.navigation_segment_wait_sec = self._positive_float_parameter(
            'navigation_segment_wait_sec'
        )
        self.publish_rate_hz = self._positive_float_parameter(
            'publish_rate_hz'
        )
        self.roll_pitch_limit_deg = self._positive_float_parameter(
            'roll_pitch_limit_deg'
        )
        self.jump_timeout_sec = self._positive_float_parameter(
            'jump_obstacle.timeout_sec'
        )
        self.jump_pre_stop_sec = self._nonnegative_float_parameter(
            'jump_obstacle.pre_stop_sec'
        )
        self.jump_prepare_sec = self._nonnegative_float_parameter(
            'jump_obstacle.prepare_sec'
        )
        self.jump_phase1_vx = self._nonnegative_float_parameter(
            'jump_obstacle.phase1_vx'
        )
        self.jump_phase1_duration = self._nonnegative_float_parameter(
            'jump_obstacle.phase1_duration'
        )
        self.jump_pause_duration = self._nonnegative_float_parameter(
            'jump_obstacle.pause_duration'
        )
        self.jump_phase2_vx = self._nonnegative_float_parameter(
            'jump_obstacle.phase2_vx'
        )
        self.jump_phase2_duration = self._nonnegative_float_parameter(
            'jump_obstacle.phase2_duration'
        )
        self.jump_recover_sec = self._nonnegative_float_parameter(
            'jump_obstacle.recover_sec'
        )
        self.enable_depth_safety = self._bool_parameter(
            'obstacle_safety.enable_depth'
        )
        self.enable_scan_safety = self._bool_parameter(
            'obstacle_safety.enable_scan'
        )
        self.require_safety_data = self._bool_parameter(
            'obstacle_safety.require_fresh_data'
        )
        self.depth_image_topic = self._string_parameter(
            'obstacle_safety.depth_image_topic'
        )
        self.scan_topic = self._string_parameter(
            'obstacle_safety.scan_topic'
        )
        self.safety_max_age_sec = self._positive_float_parameter(
            'obstacle_safety.max_sensor_age_sec'
        )
        self.front_stop_distance_m = self._positive_float_parameter(
            'obstacle_safety.front_stop_distance_m'
        )
        self.front_slow_distance_m = self._positive_float_parameter(
            'obstacle_safety.front_slow_distance_m'
        )
        self.slow_vx = self._positive_float_parameter(
            'obstacle_safety.slow_vx'
        )
        self.side_stop_distance_m = self._positive_float_parameter(
            'obstacle_safety.side_stop_distance_m'
        )
        self.side_warn_distance_m = self._positive_float_parameter(
            'obstacle_safety.side_warn_distance_m'
        )
        self.side_balance_gain = self._nonnegative_float_parameter(
            'obstacle_safety.side_balance_gain'
        )
        self.max_correction_wz = self._nonnegative_float_parameter(
            'obstacle_safety.max_correction_wz'
        )
        self.depth_min_valid_m = self._positive_float_parameter(
            'obstacle_safety.depth_min_valid_m'
        )
        self.depth_max_valid_m = self._positive_float_parameter(
            'obstacle_safety.depth_max_valid_m'
        )
        self.depth_sample_step_px = self._positive_int_parameter(
            'obstacle_safety.depth_sample_step_px'
        )
        self.depth_roi_top_ratio = self._ratio_parameter(
            'obstacle_safety.depth_roi_top_ratio'
        )
        self.depth_roi_bottom_ratio = self._ratio_parameter(
            'obstacle_safety.depth_roi_bottom_ratio'
        )
        self.depth_roi_percentile = self._ratio_parameter(
            'obstacle_safety.depth_roi_percentile'
        )
        self.scan_front_half_angle_rad = math.radians(
            self._positive_float_parameter(
                'obstacle_safety.scan_front_half_angle_deg'
            )
        )
        self.scan_side_min_angle_rad = math.radians(
            self._positive_float_parameter(
                'obstacle_safety.scan_side_min_angle_deg'
            )
        )
        self.scan_side_max_angle_rad = math.radians(
            self._positive_float_parameter(
                'obstacle_safety.scan_side_max_angle_deg'
            )
        )
        self.scan_percentile = self._ratio_parameter(
            'obstacle_safety.scan_percentile'
        )
        self.practical_timeout_sec = self._positive_float_parameter(
            'practical_obstacle.timeout_sec'
        )
        self.practical_pre_stop_sec = self._nonnegative_float_parameter(
            'practical_obstacle.pre_stop_sec'
        )
        self.practical_recover_sec = self._nonnegative_float_parameter(
            'practical_obstacle.recover_sec'
        )
        self.obstacle_enter_vx = self._nonnegative_float_parameter(
            'practical_obstacle.enter_vx'
        )
        self.obstacle_enter_duration_sec = self._nonnegative_float_parameter(
            'practical_obstacle.enter_duration_sec'
        )
        self.obstacle_forward_vx = self._nonnegative_float_parameter(
            'practical_obstacle.forward_vx'
        )
        self.obstacle_forward_duration_sec = (
            self._nonnegative_float_parameter(
                'practical_obstacle.forward_duration_sec'
            )
        )
        self.obstacle_turn_wz = self._positive_float_parameter(
            'practical_obstacle.turn_wz'
        )
        self.obstacle_turn_duration_sec = self._nonnegative_float_parameter(
            'practical_obstacle.turn_duration_sec'
        )
        self.obstacle_side_adjust_wz = self._positive_float_parameter(
            'practical_obstacle.side_adjust_wz'
        )
        self.obstacle_side_adjust_turn_sec = (
            self._nonnegative_float_parameter(
                'practical_obstacle.side_adjust_turn_sec'
            )
        )
        self.obstacle_side_adjust_forward_sec = (
            self._nonnegative_float_parameter(
                'practical_obstacle.side_adjust_forward_sec'
            )
        )
        self.obstacle_exit_vx = self._nonnegative_float_parameter(
            'practical_obstacle.exit_vx'
        )
        self.obstacle_exit_duration_sec = self._nonnegative_float_parameter(
            'practical_obstacle.exit_duration_sec'
        )
        self.practical_obstacle_steps = self._load_practical_obstacle_steps()
        self._validate_obstacle_parameters()

        stop_count = self._positive_int_parameter('stop_publish_count')
        stop_period_sec = self._nonnegative_float_parameter(
            'stop_publish_period_sec'
        )

        self._state_lock = threading.RLock()
        self._status = STATUS_IDLE
        self._current_mode = 'IDLE'
        self._control_locked = False
        self._action_active = False
        self._emergency_stop = False
        self._active_goal_handle = None
        self._active_feedback_callback = None

        self._sensor_lock = threading.RLock()
        self._depth_front_m = None
        self._depth_left_m = None
        self._depth_right_m = None
        self._last_depth_time = None
        self._scan_front_m = None
        self._scan_left_m = None
        self._scan_right_m = None
        self._last_scan_time = None
        self._last_safety_debug_time = 0.0
        self._warned_depth_encodings = set()

        self.status_pub = self.create_publisher(
            String,
            self._string_parameter('status_topic'),
            10
        )
        self.lock_pub = self.create_publisher(
            Bool,
            self._string_parameter('control_lock_topic'),
            10
        )
        self.debug_pub = self.create_publisher(
            String,
            self._string_parameter('debug_topic'),
            10
        )
        self.mode_pub = self.create_publisher(
            String,
            self._string_parameter('current_mode_topic'),
            10
        )

        self.motion = RobotMotionAdapter(
            self,
            self.cmd_vel_topic,
            stop_count,
            stop_period_sec
        )

        self.command_json_sub = self.create_subscription(
            String,
            self.command_json_topic,
            self._on_command_json,
            10,
            callback_group=self.callback_group
        )
        self.depth_subscription = None
        if self.enable_depth_safety:
            self.depth_subscription = self.create_subscription(
                Image,
                self.depth_image_topic,
                self._on_depth_image,
                10,
                callback_group=self.callback_group
            )

        self.scan_subscription = None
        if self.enable_scan_safety:
            self.scan_subscription = self.create_subscription(
                LaserScan,
                self.scan_topic,
                self._on_scan,
                10,
                callback_group=self.callback_group
            )

        self.action_server = ActionServer(
            self,
            ExecuteMotion,
            self.motion_action_name,
            self._execute_motion_action,
            goal_callback=self._motion_goal_callback,
            cancel_callback=self._motion_cancel_callback,
            callback_group=self.callback_group
        )

        status_period = 1.0 / self._positive_float_parameter(
            'status_publish_rate_hz'
        )
        self.status_timer = self.create_timer(
            status_period,
            self._publish_periodic_state,
            callback_group=self.callback_group
        )

        self.publish_status(STATUS_IDLE, 'gait control ready')
        self.publish_lock(False)
        self.publish_mode('IDLE')
        self.get_logger().info(
            'Gait control node ready: json_topic='
            f'{self.command_json_topic}, action={self.motion_action_name}'
        )
        self.get_logger().info(
            'Obstacle safety: '
            f'depth={self.enable_depth_safety} '
            f'topic={self.depth_image_topic}, '
            f'scan={self.enable_scan_safety} topic={self.scan_topic}, '
            f'require_fresh={self.require_safety_data}'
        )

    def _declare_parameters(self):
        parameters = {
            'cmd_vel_topic': '/navigation/cmd_vel',
            'status_topic': '/gait/status',
            'control_lock_topic': '/gait/control_lock',
            'debug_topic': '/gait/debug',
            'current_mode_topic': '/gait/current_mode',
            'command_json_topic': '/gait/command_json',
            'motion_action_name': '/locomotion/execute_motion',
            'default_max_vx': 0.25,
            'default_max_vy': 0.15,
            'default_max_wz': 0.40,
            'hard_max_vx': 0.25,
            'hard_max_vy': 0.15,
            'hard_max_wz': 0.40,
            'max_action_duration_sec': 10.0,
            'default_hold_duration_sec': 2.0,
            'default_move_duration_sec': 1.0,
            'default_turn_duration_sec': 1.0,
            'navigation_segment_wait_sec': 3.0,
            'publish_rate_hz': 10.0,
            'status_publish_rate_hz': 5.0,
            'stop_publish_count': 3,
            'stop_publish_period_sec': 0.05,
            'roll_pitch_limit_deg': 25.0,
            'jump_obstacle.timeout_sec': 5.0,
            'jump_obstacle.pre_stop_sec': 0.5,
            'jump_obstacle.prepare_sec': 0.3,
            'jump_obstacle.phase1_vx': 0.10,
            'jump_obstacle.phase1_duration': 0.8,
            'jump_obstacle.pause_duration': 0.2,
            'jump_obstacle.phase2_vx': 0.08,
            'jump_obstacle.phase2_duration': 0.5,
            'jump_obstacle.recover_sec': 0.5,
            'obstacle_safety.enable_depth': True,
            'obstacle_safety.enable_scan': False,
            'obstacle_safety.require_fresh_data': False,
            'obstacle_safety.depth_image_topic': (
                '/camera/camera/depth/image_rect_raw'
            ),
            'obstacle_safety.scan_topic': '/scan',
            'obstacle_safety.max_sensor_age_sec': 0.60,
            'obstacle_safety.front_stop_distance_m': 0.35,
            'obstacle_safety.front_slow_distance_m': 0.55,
            'obstacle_safety.slow_vx': 0.03,
            'obstacle_safety.side_stop_distance_m': 0.12,
            'obstacle_safety.side_warn_distance_m': 0.25,
            'obstacle_safety.side_balance_gain': 0.45,
            'obstacle_safety.max_correction_wz': 0.18,
            'obstacle_safety.depth_min_valid_m': 0.08,
            'obstacle_safety.depth_max_valid_m': 2.50,
            'obstacle_safety.depth_sample_step_px': 12,
            'obstacle_safety.depth_roi_top_ratio': 0.30,
            'obstacle_safety.depth_roi_bottom_ratio': 0.72,
            'obstacle_safety.depth_roi_percentile': 0.20,
            'obstacle_safety.scan_front_half_angle_deg': 22.5,
            'obstacle_safety.scan_side_min_angle_deg': 35.0,
            'obstacle_safety.scan_side_max_angle_deg': 110.0,
            'obstacle_safety.scan_percentile': 0.20,
            'practical_obstacle.timeout_sec': 80.0,
            'practical_obstacle.pre_stop_sec': 0.4,
            'practical_obstacle.recover_sec': 0.6,
            'practical_obstacle.enter_vx': 0.08,
            'practical_obstacle.enter_duration_sec': 6.0,
            'practical_obstacle.forward_vx': 0.08,
            'practical_obstacle.forward_duration_sec': 5.0,
            'practical_obstacle.turn_wz': 0.30,
            'practical_obstacle.turn_duration_sec': 4.8,
            'practical_obstacle.side_adjust_wz': 0.18,
            'practical_obstacle.side_adjust_turn_sec': 0.45,
            'practical_obstacle.side_adjust_forward_sec': 0.45,
            'practical_obstacle.exit_vx': 0.08,
            'practical_obstacle.exit_duration_sec': 4.0,
            'practical_obstacle.step_names': [
                'entry_up',
                'turn_left_to_top',
                'top_left',
                'turn_left_to_middle_down',
                'middle_down',
                'turn_right_to_bottom_left',
                'bottom_left',
                'turn_right_to_left_up',
                'left_up',
                'turn_left_to_exit',
                'exit_left',
            ],
            'practical_obstacle.step_types': [
                'forward',
                'turn_left',
                'forward',
                'turn_left',
                'forward',
                'turn_right',
                'forward',
                'turn_right',
                'forward',
                'turn_left',
                'forward',
            ],
            'practical_obstacle.step_distance_m': [
                0.48,
                0.00,
                0.40,
                0.00,
                0.40,
                0.00,
                0.32,
                0.00,
                0.40,
                0.00,
                0.32,
            ],
            'practical_obstacle.step_turn_angle_deg': [
                0.0,
                90.0,
                0.0,
                90.0,
                0.0,
                90.0,
                0.0,
                90.0,
                0.0,
                90.0,
                0.0,
            ],
            'practical_obstacle.step_linear_speed': [
                0.08,
                0.00,
                0.08,
                0.00,
                0.08,
                0.00,
                0.08,
                0.00,
                0.08,
                0.00,
                0.08,
            ],
            'practical_obstacle.step_turn_speed_deg': [
                0.0,
                18.0,
                0.0,
                18.0,
                0.0,
                18.0,
                0.0,
                18.0,
                0.0,
                18.0,
                0.0,
            ],
            'practical_obstacle.step_front_target_m': [
                0.50,
                0.00,
                0.00,
                0.00,
                0.00,
                0.00,
                0.00,
                0.00,
                0.00,
                0.00,
                0.00,
            ],
        }
        for name, value in parameters.items():
            self.declare_parameter(name, value)

    def _on_command_json(self, msg):
        try:
            fields = json.loads(msg.data)
        except json.JSONDecodeError as error:
            self.publish_status(
                STATUS_FAILED,
                f'invalid /gait/command_json: {error}'
            )
            return

        if not isinstance(fields, dict):
            self.publish_status(
                STATUS_FAILED,
                '/gait/command_json must contain a JSON object'
            )
            return

        result = self.handle_command(fields)
        self.publish_debug(
            'JSON command result: '
            f'success={result.success}, status={result.status}'
        )

    def _motion_goal_callback(self, goal_request):
        motion_name = str(goal_request.motion_name or '').strip()
        if not motion_name:
            self.get_logger().warn('Rejected empty motion goal')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _motion_cancel_callback(self, goal_handle):
        del goal_handle
        with self._state_lock:
            self._emergency_stop = True
        self.motion.stop('motion action cancel requested', log_level='warn')
        return CancelResponse.ACCEPT

    def _execute_motion_action(self, goal_handle):
        motion_name = str(goal_handle.request.motion_name or '').strip()
        fields = self.motion_name_to_command_fields(motion_name)
        result_msg = ExecuteMotion.Result()

        if fields is None:
            message = f'unsupported motion_name: {motion_name}'
            self.get_logger().error(message)
            result_msg.success = False
            result_msg.message = message
            goal_handle.abort()
            return result_msg

        def feedback_callback(current_step, progress):
            feedback = ExecuteMotion.Feedback()
            feedback.current_step = str(current_step)
            feedback.progress = float(max(0.0, min(1.0, progress)))
            goal_handle.publish_feedback(feedback)

        with self._state_lock:
            self._active_goal_handle = goal_handle
            self._active_feedback_callback = feedback_callback

        try:
            self.publish_action_feedback(f'{motion_name}: start', 0.0)
            result = self.handle_command(fields)
            self.publish_action_feedback(
                f'{motion_name}: {result.status}',
                1.0
            )
        except Exception as error:
            self.motion.stop('motion action exception', log_level='warn')
            self.get_logger().error(f'motion action exception: {error}')
            result = CommandResult(False, STATUS_FAILED, str(error))
        finally:
            with self._state_lock:
                self._active_goal_handle = None
                self._active_feedback_callback = None

        result_msg.success = bool(result.success)
        result_msg.message = result.message
        if result.success:
            goal_handle.succeed()
        elif goal_handle.is_cancel_requested:
            goal_handle.canceled()
        else:
            goal_handle.abort()
        return result_msg

    def motion_name_to_command_fields(self, motion_name):
        normalized = self._normalize_motion_name(motion_name)
        direct_commands = {
            'stop': {'command': 'STOP'},
            'final_stop': {'command': 'STOP'},
            'obstacle_stop': {'command': 'OBSTACLE_STOP'},
            'hold_stable': {'command': 'HOLD_STABLE'},
            'start_jump': {'command': 'JUMP_START_OBSTACLE'},
            'jump_start_obstacle': {'command': 'JUMP_START_OBSTACLE'},
            'finish_jump': {'command': 'JUMP_END_OBSTACLE'},
            'end_jump': {'command': 'JUMP_END_OBSTACLE'},
            'jump_end_obstacle': {'command': 'JUMP_END_OBSTACLE'},
            'avoid_zone': {'command': 'PRACTICAL_OBSTACLE_ZONE'},
            'practical_obstacle_zone': {
                'command': 'PRACTICAL_OBSTACLE_ZONE'
            },
            'obstacle_zone': {'command': 'PRACTICAL_OBSTACLE_ZONE'},
            'enter_obstacle_zone': {'command': 'ENTER_OBSTACLE_ZONE'},
            'obstacle_forward_slow': {'command': 'OBSTACLE_FORWARD_SLOW'},
            'obstacle_turn_left': {'command': 'OBSTACLE_TURN_LEFT'},
            'obstacle_turn_right': {'command': 'OBSTACLE_TURN_RIGHT'},
            'obstacle_side_adjust': {'command': 'OBSTACLE_SIDE_ADJUST'},
            'exit_obstacle_zone': {'command': 'EXIT_OBSTACLE_ZONE'},
        }
        fields = direct_commands.get(normalized)
        if fields is not None:
            return dict(fields)

        if normalized.startswith('follow_to_') or normalized.startswith(
            'return_to_'
        ):
            return {
                'command': 'WAIT_NAVIGATION_SEGMENT',
                'duration_sec': self.navigation_segment_wait_sec,
            }

        return None

    @staticmethod
    def _normalize_motion_name(motion_name):
        normalized = str(motion_name or '').strip().lower()
        normalized = normalized.replace('-', '_').replace(' ', '_')
        while '__' in normalized:
            normalized = normalized.replace('__', '_')
        return normalized

    def handle_command(self, fields):
        command = str(fields.get('command', '')).strip().upper()
        if command not in self.COMMANDS:
            return CommandResult(
                False,
                STATUS_FAILED,
                f'unknown gait command: {command}'
            )

        if command == 'STOP':
            return self.execute_stop()

        if command == 'OBSTACLE_STOP':
            return self.execute_stop()

        if command == 'SPEED_LIMIT':
            try:
                return self.set_speed_limit(fields)
            except ValueError as error:
                result = CommandResult(False, STATUS_FAILED, str(error))
                self.publish_status(result.status, result.message)
                return result

        if command == 'WAIT_NAVIGATION_SEGMENT':
            try:
                return self.execute_wait_navigation_segment(fields)
            except ValueError as error:
                result = CommandResult(False, STATUS_FAILED, str(error))
                self.publish_status(result.status, result.message)
                return result

        with self._state_lock:
            if self._action_active:
                return CommandResult(
                    False,
                    STATUS_FAILED,
                    'another gait command is running; send STOP first'
                )
            self._action_active = True
            self._emergency_stop = False
            self._status = STATUS_RUNNING
            self._current_mode = command

        self.publish_lock(True)
        self.publish_mode(command)
        self.publish_status(STATUS_RUNNING, f'{command} started')

        try:
            if command == 'RECOVERY_STAND':
                result = self.execute_recovery_stand()
            elif command == 'HOLD_STABLE':
                result = self.execute_hold_stable(fields)
            elif command == 'LOW_SPEED_MOVE':
                result = self.execute_low_speed_move(fields)
            elif command == 'TURN_IN_PLACE':
                result = self.execute_turn_in_place(fields)
            elif command == 'BODY_HEIGHT_ADJUST':
                result = self.execute_body_height_adjust(fields)
            elif command == 'JUMP_START_OBSTACLE':
                result = self.execute_jump_obstacle('start')
            elif command == 'JUMP_END_OBSTACLE':
                result = self.execute_jump_obstacle('end')
            elif command in ('PRACTICAL_OBSTACLE_ZONE', 'AVOID_ZONE'):
                result = self.execute_practical_obstacle_zone()
            elif command == 'ENTER_OBSTACLE_ZONE':
                result = self.execute_obstacle_velocity_command(
                    command,
                    fields,
                    self.obstacle_enter_vx,
                    0.0,
                    self.obstacle_enter_duration_sec
                )
            elif command == 'OBSTACLE_FORWARD_SLOW':
                result = self.execute_obstacle_velocity_command(
                    command,
                    fields,
                    self.obstacle_forward_vx,
                    0.0,
                    self.obstacle_forward_duration_sec
                )
            elif command == 'OBSTACLE_TURN_LEFT':
                result = self.execute_obstacle_velocity_command(
                    command,
                    fields,
                    0.0,
                    self.obstacle_turn_wz,
                    self.obstacle_turn_duration_sec
                )
            elif command == 'OBSTACLE_TURN_RIGHT':
                result = self.execute_obstacle_velocity_command(
                    command,
                    fields,
                    0.0,
                    -self.obstacle_turn_wz,
                    self.obstacle_turn_duration_sec
                )
            elif command == 'OBSTACLE_SIDE_ADJUST':
                result = self.execute_obstacle_side_adjust(fields)
            elif command == 'EXIT_OBSTACLE_ZONE':
                result = self.execute_obstacle_velocity_command(
                    command,
                    fields,
                    self.obstacle_exit_vx,
                    0.0,
                    self.obstacle_exit_duration_sec
                )
            else:
                result = CommandResult(
                    False,
                    STATUS_FAILED,
                    f'unhandled command: {command}'
                )
        except ValueError as error:
            self.zero_velocity(f'invalid {command} command')
            result = CommandResult(False, STATUS_FAILED, str(error))
        finally:
            with self._state_lock:
                self._action_active = False
                self._emergency_stop = False

        self.publish_status(result.status, result.message)
        self.publish_mode('IDLE')
        self.publish_lock(False)
        return result

    def execute_stop(self):
        with self._state_lock:
            interrupted = self._action_active
            self._emergency_stop = True
            self._status = (
                STATUS_EMERGENCY_STOP
                if interrupted
                else STATUS_RUNNING
            )
            self._current_mode = 'STOP'

        self.publish_lock(True)
        self.publish_mode('STOP')
        self.motion.stop('STOP command', log_level='warn')
        status = STATUS_EMERGENCY_STOP if interrupted else STATUS_DONE
        message = (
            'active gait command interrupted by STOP'
            if interrupted
            else 'robot stopped'
        )
        self.publish_status(status, message)

        if not interrupted:
            with self._state_lock:
                self._emergency_stop = False
            self.publish_mode('IDLE')
            self.publish_lock(False)

        return CommandResult(True, status, message)

    def execute_recovery_stand(self):
        if not self.is_robot_stable():
            self.motion.stop(
                'unstable before recovery_stand',
                log_level='warn'
            )

        success, message = self.motion.recovery_stand()
        if success:
            return CommandResult(True, STATUS_DONE, message)
        return CommandResult(False, STATUS_FAILED, message)

    def execute_hold_stable(self, fields):
        duration, capped = self._bounded_duration(
            fields.get('duration_sec', 0.0),
            self.default_hold_duration_sec
        )
        start_time = time.monotonic()
        period = 1.0 / self.publish_rate_hz

        while time.monotonic() - start_time < duration:
            check = self._pre_motion_check(start_time)
            if check is not None:
                return check
            self.motion.hold_stable()
            time.sleep(period)

        self.motion.stop('hold_stable final stop', log_level='info')
        if capped:
            return CommandResult(
                False,
                STATUS_TIMEOUT,
                'HOLD_STABLE exceeded max_action_duration_sec'
            )
        return CommandResult(True, STATUS_DONE, 'HOLD_STABLE completed')

    def execute_low_speed_move(self, fields):
        duration, capped = self._bounded_duration(
            fields.get('duration_sec', 0.0),
            self.default_move_duration_sec
        )
        vx = self._float_field(fields, 'vx', 0.0)
        vy = self._float_field(fields, 'vy', 0.0)
        wz = self._float_field(fields, 'wz', 0.0)
        vx, vy, wz = self.clamp_velocity(vx, vy, wz)

        start_time = time.monotonic()
        period = 1.0 / self.publish_rate_hz
        while time.monotonic() - start_time < duration:
            check = self._pre_motion_check(start_time)
            if check is not None:
                return check
            self.send_velocity(vx, vy, wz)
            time.sleep(period)

        self.zero_velocity('LOW_SPEED_MOVE final stop')
        if capped:
            return CommandResult(
                False,
                STATUS_TIMEOUT,
                'LOW_SPEED_MOVE exceeded max_action_duration_sec'
            )
        return CommandResult(True, STATUS_DONE, 'LOW_SPEED_MOVE completed')

    def execute_turn_in_place(self, fields):
        target_yaw = fields.get('target_yaw')
        wz = self._float_field(fields, 'wz', 0.0)
        if wz == 0.0:
            wz = self.max_wz

        if target_yaw is not None:
            target_yaw = self._as_finite_float(target_yaw, 'target_yaw')
            self.publish_debug(
                'TODO: yaw closed loop is not connected; estimating '
                'TURN_IN_PLACE duration from target_yaw / wz.'
            )
            requested_duration = abs(target_yaw / wz)
        else:
            requested_duration = fields.get('duration_sec', 0.0)

        duration, capped = self._bounded_duration(
            requested_duration,
            self.default_turn_duration_sec
        )
        _, _, wz = self.clamp_velocity(0.0, 0.0, wz)
        start_time = time.monotonic()
        period = 1.0 / self.publish_rate_hz

        while time.monotonic() - start_time < duration:
            check = self._pre_motion_check(start_time)
            if check is not None:
                return check
            self.send_velocity(0.0, 0.0, wz)
            time.sleep(period)

        self.zero_velocity('TURN_IN_PLACE final stop')
        if capped:
            return CommandResult(
                False,
                STATUS_TIMEOUT,
                'TURN_IN_PLACE exceeded max_action_duration_sec'
            )
        return CommandResult(True, STATUS_DONE, 'TURN_IN_PLACE completed')

    def execute_body_height_adjust(self, fields):
        height = self._float_field(fields, 'body_height', 0.0)
        success, message = self.motion.set_body_height(height)
        if success:
            return CommandResult(True, STATUS_DONE, message)
        return CommandResult(False, STATUS_FAILED, message)

    def execute_wait_navigation_segment(self, fields):
        duration, capped = self._bounded_duration(
            fields.get('duration_sec', 0.0),
            self.navigation_segment_wait_sec
        )
        start_time = time.monotonic()
        period = 1.0 / self.publish_rate_hz

        self.publish_action_feedback('navigation segment active', 0.05)
        self.publish_status(
            STATUS_RUNNING,
            'waiting while navigation owns cmd_vel'
        )
        while time.monotonic() - start_time < duration:
            if self._active_goal_cancel_requested():
                self.motion.stop(
                    'navigation wait canceled',
                    log_level='warn'
                )
                return CommandResult(
                    False,
                    STATUS_EMERGENCY_STOP,
                    'navigation segment wait canceled'
                )
            progress = (time.monotonic() - start_time) / max(duration, 0.001)
            self.publish_action_feedback('navigation segment active', progress)
            time.sleep(period)

        if capped:
            return CommandResult(
                False,
                STATUS_TIMEOUT,
                'WAIT_NAVIGATION_SEGMENT exceeded max_action_duration_sec'
            )
        return CommandResult(
            True,
            STATUS_DONE,
            'navigation segment wait completed'
        )

    def execute_practical_obstacle_zone(self):
        start_time = time.monotonic()
        self.publish_action_feedback('practical obstacle pre-stop', 0.02)
        self.zero_velocity('practical obstacle pre-stop')
        check = self._wait_for_practical_obstacle(
            self.practical_pre_stop_sec,
            start_time
        )
        if check is not None:
            return check

        total_steps = max(1, len(self.practical_obstacle_steps))
        for index, step in enumerate(self.practical_obstacle_steps):
            step_progress = index / float(total_steps)
            self.publish_debug(
                'practical obstacle step '
                f'{index + 1}/{total_steps}: {step.name} '
                f'vx={step.vx:.3f}, wz={step.wz:.3f}, '
                f'duration={step.duration_sec:.2f}s'
            )
            self.publish_action_feedback(
                f'obstacle: {step.name}',
                step_progress
            )
            check = self._run_practical_obstacle_step(
                step,
                start_time,
                index,
                total_steps
            )
            if check is not None:
                return check
            self.zero_velocity(f'obstacle step {step.name} stop')

        self.publish_action_feedback('practical obstacle recovery', 0.95)
        self.zero_velocity('practical obstacle final stop')
        check = self._wait_for_practical_obstacle(
            self.practical_recover_sec,
            start_time
        )
        if check is not None:
            return check

        return CommandResult(
            True,
            STATUS_DONE,
            'PRACTICAL_OBSTACLE_ZONE completed'
        )

    def execute_obstacle_velocity_command(
        self,
        command,
        fields,
        default_vx,
        default_wz,
        default_duration
    ):
        duration, capped = self._bounded_duration(
            fields.get('duration_sec', 0.0),
            default_duration
        )
        vx = self._float_field(fields, 'vx', default_vx)
        wz = self._float_field(fields, 'wz', default_wz)
        step = ObstacleStep(command, vx, wz, duration)
        start_time = time.monotonic()
        check = self._run_practical_obstacle_step(step, start_time, 0, 1)
        self.zero_velocity(f'{command} final stop')
        if check is not None:
            return check
        if capped:
            return CommandResult(
                False,
                STATUS_TIMEOUT,
                f'{command} exceeded max_action_duration_sec'
            )
        return CommandResult(True, STATUS_DONE, f'{command} completed')

    def execute_obstacle_side_adjust(self, fields):
        direction = str(fields.get('direction', 'left')).strip().lower()
        if direction not in ('left', 'right'):
            raise ValueError('direction must be left or right')
        sign = 1.0 if direction == 'left' else -1.0
        start_time = time.monotonic()
        steps = [
            ObstacleStep(
                f'OBSTACLE_SIDE_ADJUST_{direction}_turn_out',
                0.0,
                sign * self.obstacle_side_adjust_wz,
                self.obstacle_side_adjust_turn_sec
            ),
            ObstacleStep(
                f'OBSTACLE_SIDE_ADJUST_{direction}_forward',
                self.obstacle_forward_vx,
                0.0,
                self.obstacle_side_adjust_forward_sec
            ),
            ObstacleStep(
                f'OBSTACLE_SIDE_ADJUST_{direction}_turn_back',
                0.0,
                -sign * self.obstacle_side_adjust_wz,
                self.obstacle_side_adjust_turn_sec
            ),
        ]
        for index, step in enumerate(steps):
            check = self._run_practical_obstacle_step(
                step,
                start_time,
                index,
                len(steps)
            )
            self.zero_velocity(f'{step.name} stop')
            if check is not None:
                return check
        return CommandResult(
            True,
            STATUS_DONE,
            'OBSTACLE_SIDE_ADJUST completed'
        )

    def execute_jump_obstacle(self, obstacle_type):
        command_name = self._jump_command_name(obstacle_type)
        start_time = time.monotonic()

        initial_check = self._pre_obstacle_check(start_time)
        if initial_check is not None:
            return initial_check

        self.zero_velocity(f'{command_name} pre-stop')
        check = self._wait_for_obstacle(self.jump_pre_stop_sec, start_time)
        if check is not None:
            return check

        check = self.prepare_obstacle_pose(start_time)
        if check is not None:
            return check

        self.publish_debug('obstacle phase 1')
        check = self._run_obstacle_velocity_phase(
            self.jump_phase1_vx,
            self.jump_phase1_duration,
            start_time
        )
        if check is not None:
            return check

        self.zero_velocity(f'{command_name} pause')
        check = self._wait_for_obstacle(self.jump_pause_duration, start_time)
        if check is not None:
            return check

        self.publish_debug('obstacle phase 2')
        check = self._run_obstacle_velocity_phase(
            self.jump_phase2_vx,
            self.jump_phase2_duration,
            start_time
        )
        if check is not None:
            return check

        self.zero_velocity(f'{command_name} phase sequence stop')
        check = self.recover_after_obstacle(start_time)
        if check is not None:
            return check

        return CommandResult(
            True,
            STATUS_DONE,
            f'{command_name} completed'
        )

    def prepare_obstacle_pose(self, start_time):
        self.publish_debug(
            'prepare_obstacle_pose TODO: body height/gait mode adapter is not '
            'wired yet'
        )
        return self._wait_for_obstacle(self.jump_prepare_sec, start_time)

    def recover_after_obstacle(self, start_time):
        self.publish_debug('obstacle recovery')
        self.zero_velocity('obstacle recovery')
        return self._wait_for_obstacle(self.jump_recover_sec, start_time)

    def _run_obstacle_velocity_phase(self, vx, duration, start_time):
        end_time = time.monotonic() + float(duration)
        period = 1.0 / self.publish_rate_hz

        while time.monotonic() < end_time:
            check = self._pre_obstacle_check(start_time)
            if check is not None:
                return check
            self.send_velocity(vx, 0.0, 0.0)
            time.sleep(period)

        return None

    def _wait_for_obstacle(self, duration, start_time):
        end_time = time.monotonic() + float(duration)
        period = 1.0 / self.publish_rate_hz

        while time.monotonic() < end_time:
            check = self._pre_obstacle_check(start_time)
            if check is not None:
                return check
            time.sleep(period)

        return None

    def _pre_obstacle_check(self, start_time):
        with self._state_lock:
            emergency_stop = self._emergency_stop

        if emergency_stop:
            self.motion.stop('obstacle emergency stop', log_level='warn')
            return CommandResult(
                False,
                STATUS_EMERGENCY_STOP,
                'obstacle interrupted by STOP'
            )

        if time.monotonic() - start_time > self.jump_timeout_sec:
            self.motion.stop('obstacle timeout stop', log_level='warn')
            return CommandResult(False, STATUS_TIMEOUT, 'obstacle timeout')

        if not self.is_robot_stable():
            self.motion.stop('obstacle unstable stop', log_level='warn')
            return CommandResult(False, STATUS_UNSTABLE, 'obstacle unstable')

        return None

    def _run_practical_obstacle_step(
        self,
        step,
        start_time,
        step_index,
        total_steps
    ):
        end_time = time.monotonic() + float(step.duration_sec)
        period = 1.0 / self.publish_rate_hz

        while time.monotonic() < end_time:
            check = self._pre_practical_obstacle_check(start_time)
            if check is not None:
                return check
            if self._step_front_target_reached(step):
                self.publish_debug(
                    f'{step.name}: front target reached at '
                    f'{step.front_target_m:.3f}m'
                )
                return None
            vx, wz = self._adjust_practical_obstacle_velocity(
                step.vx,
                step.wz
            )
            self.send_velocity(vx, 0.0, wz)
            elapsed = max(
                0.0,
                step.duration_sec - (end_time - time.monotonic())
            )
            step_fraction = elapsed / max(step.duration_sec, 0.001)
            progress = (
                step_index + step_fraction
            ) / float(max(1, total_steps))
            self.publish_action_feedback(f'obstacle: {step.name}', progress)
            time.sleep(period)

        return None

    def _step_front_target_reached(self, step):
        if step.front_target_m is None:
            return False
        if step.vx <= 0.0:
            return False
        snapshot = self.distance_snapshot()
        if snapshot.front_m is None:
            return False
        return snapshot.front_m <= step.front_target_m

    def _wait_for_practical_obstacle(self, duration, start_time):
        end_time = time.monotonic() + float(duration)
        period = 1.0 / self.publish_rate_hz

        while time.monotonic() < end_time:
            check = self._pre_practical_obstacle_check(start_time)
            if check is not None:
                return check
            time.sleep(period)
        return None

    def _pre_practical_obstacle_check(self, start_time):
        with self._state_lock:
            emergency_stop = self._emergency_stop

        if emergency_stop or self._active_goal_cancel_requested():
            self.motion.stop(
                'practical obstacle emergency stop',
                log_level='warn'
            )
            return CommandResult(
                False,
                STATUS_EMERGENCY_STOP,
                'practical obstacle interrupted'
            )

        if time.monotonic() - start_time > self.practical_timeout_sec:
            self.motion.stop(
                'practical obstacle timeout stop',
                log_level='warn'
            )
            return CommandResult(
                False,
                STATUS_TIMEOUT,
                'practical obstacle timeout'
            )

        if not self.is_robot_stable():
            self.motion.stop(
                'practical obstacle unstable stop',
                log_level='warn'
            )
            return CommandResult(
                False,
                STATUS_UNSTABLE,
                'practical obstacle unstable'
            )

        snapshot = self.distance_snapshot()
        if self.require_safety_data and not snapshot.has_fresh_source:
            self.motion.stop(
                'no fresh obstacle safety data',
                log_level='warn'
            )
            return CommandResult(
                False,
                STATUS_FAILED,
                'no fresh D435i depth or scan data'
            )

        if (
            snapshot.front_m is not None
            and snapshot.front_m < self.front_stop_distance_m
        ):
            self.motion.stop(
                f'front obstacle too close: {snapshot.front_m:.3f}m',
                log_level='warn'
            )
            return CommandResult(
                False,
                STATUS_EMERGENCY_STOP,
                'front obstacle too close'
            )

        if (
            snapshot.left_m is not None
            and snapshot.left_m < self.side_stop_distance_m
        ):
            self.motion.stop(
                f'left clearance too small: {snapshot.left_m:.3f}m',
                log_level='warn'
            )
            return CommandResult(
                False,
                STATUS_EMERGENCY_STOP,
                'left clearance too small'
            )

        if (
            snapshot.right_m is not None
            and snapshot.right_m < self.side_stop_distance_m
        ):
            self.motion.stop(
                f'right clearance too small: {snapshot.right_m:.3f}m',
                log_level='warn'
            )
            return CommandResult(
                False,
                STATUS_EMERGENCY_STOP,
                'right clearance too small'
            )

        return None

    def _adjust_practical_obstacle_velocity(self, vx, wz):
        snapshot = self.distance_snapshot()
        adjusted_vx = float(vx)
        adjusted_wz = float(wz)

        if (
            snapshot.front_m is not None
            and snapshot.front_m < self.front_slow_distance_m
            and adjusted_vx > self.slow_vx
        ):
            adjusted_vx = self.slow_vx
            self._publish_safety_debug_throttled(
                'front distance '
                f'{snapshot.front_m:.3f}m; slowing vx to {adjusted_vx:.3f}'
            )

        correction = self._side_clearance_correction(snapshot)
        if correction != 0.0:
            adjusted_wz += correction
            self._publish_safety_debug_throttled(
                'side clearance correction '
                f'wz={correction:.3f}, '
                f'left={self._format_distance(snapshot.left_m)}, '
                f'right={self._format_distance(snapshot.right_m)}'
            )

        adjusted_vx, _, adjusted_wz = self.clamp_velocity(
            adjusted_vx,
            0.0,
            adjusted_wz
        )
        return adjusted_vx, adjusted_wz

    def _side_clearance_correction(self, snapshot):
        left = snapshot.left_m
        right = snapshot.right_m
        correction = 0.0

        if left is not None and right is not None:
            if (
                left < self.side_warn_distance_m
                or right < self.side_warn_distance_m
            ):
                correction = -self.side_balance_gain * (right - left)
        elif left is not None and left < self.side_warn_distance_m:
            ratio = (
                self.side_warn_distance_m - left
            ) / self.side_warn_distance_m
            correction = -self.max_correction_wz * ratio
        elif right is not None and right < self.side_warn_distance_m:
            ratio = (
                self.side_warn_distance_m - right
            ) / self.side_warn_distance_m
            correction = self.max_correction_wz * ratio

        if correction > self.max_correction_wz:
            return self.max_correction_wz
        if correction < -self.max_correction_wz:
            return -self.max_correction_wz
        return correction

    @staticmethod
    def _jump_command_name(obstacle_type):
        if obstacle_type == 'end':
            return 'JUMP_END_OBSTACLE'
        return 'JUMP_START_OBSTACLE'

    def set_speed_limit(self, fields):
        updates = {}
        for name, hard_limit in (
            ('max_vx', self.hard_max_vx),
            ('max_vy', self.hard_max_vy),
            ('max_wz', self.hard_max_wz),
        ):
            raw_value = fields.get(name, 0.0)
            if raw_value in (None, ''):
                continue
            value = self._as_finite_float(raw_value, name)
            if value == 0.0:
                continue
            if value < 0.0:
                return CommandResult(
                    False,
                    STATUS_FAILED,
                    f'{name} must be positive'
                )
            if value > hard_limit:
                self.publish_debug(
                    f'{name}={value:.3f} exceeds hard limit '
                    f'{hard_limit:.3f}; clamping'
                )
                value = hard_limit
            updates[name] = value

        if not updates:
            return CommandResult(
                False,
                STATUS_FAILED,
                'SPEED_LIMIT requires at least one positive '
                'max_vx/max_vy/max_wz'
            )

        with self._state_lock:
            self.max_vx = updates.get('max_vx', self.max_vx)
            self.max_vy = updates.get('max_vy', self.max_vy)
            self.max_wz = updates.get('max_wz', self.max_wz)

        message = (
            'speed limits set: '
            f'max_vx={self.max_vx:.3f}, '
            f'max_vy={self.max_vy:.3f}, '
            f'max_wz={self.max_wz:.3f}'
        )
        self.publish_status(STATUS_DONE, message)
        return CommandResult(True, STATUS_DONE, message)

    def publish_status(self, status, debug_message=''):
        with self._state_lock:
            self._status = status
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
        if debug_message:
            self.publish_debug(debug_message)

    def publish_lock(self, locked):
        with self._state_lock:
            self._control_locked = bool(locked)
        msg = Bool()
        msg.data = bool(locked)
        self.lock_pub.publish(msg)

    def publish_mode(self, mode):
        with self._state_lock:
            self._current_mode = str(mode)
        msg = String()
        msg.data = str(mode)
        self.mode_pub.publish(msg)

    def publish_debug(self, text):
        msg = String()
        msg.data = str(text)
        self.debug_pub.publish(msg)
        self.get_logger().info(str(text))

    def publish_action_feedback(self, current_step, progress):
        with self._state_lock:
            callback = self._active_feedback_callback
        if callback is not None:
            callback(current_step, progress)

    def _active_goal_cancel_requested(self):
        with self._state_lock:
            goal_handle = self._active_goal_handle
        return bool(
            goal_handle is not None and goal_handle.is_cancel_requested
        )

    def _on_depth_image(self, msg):
        distances = self._extract_depth_distances(msg)
        if distances is None:
            return
        with self._sensor_lock:
            self._depth_left_m = distances.get('left')
            self._depth_front_m = distances.get('front')
            self._depth_right_m = distances.get('right')
            self._last_depth_time = time.monotonic()

    def _on_scan(self, msg):
        distances = self._extract_scan_distances(msg)
        with self._sensor_lock:
            self._scan_left_m = distances.get('left')
            self._scan_front_m = distances.get('front')
            self._scan_right_m = distances.get('right')
            self._last_scan_time = time.monotonic()

    def _extract_depth_distances(self, msg):
        encoding = str(msg.encoding or '').upper()
        if encoding not in ('16UC1', 'MONO16', '32FC1'):
            if encoding not in self._warned_depth_encodings:
                self.get_logger().warn(
                    'Unsupported depth encoding for obstacle safety: '
                    f'{msg.encoding}'
                )
                self._warned_depth_encodings.add(encoding)
            return None

        if msg.width <= 0 or msg.height <= 0 or msg.step <= 0:
            return None

        row_start = int(msg.height * self.depth_roi_top_ratio)
        row_stop = int(msg.height * self.depth_roi_bottom_ratio)
        row_start = max(0, min(msg.height - 1, row_start))
        row_stop = max(row_start + 1, min(msg.height, row_stop))
        column_ranges = {
            'left': (0.05, 0.35),
            'front': (0.35, 0.65),
            'right': (0.65, 0.95),
        }
        distances = {}
        for name, (start_ratio, stop_ratio) in column_ranges.items():
            col_start = int(msg.width * start_ratio)
            col_stop = int(msg.width * stop_ratio)
            col_start = max(0, min(msg.width - 1, col_start))
            col_stop = max(col_start + 1, min(msg.width, col_stop))
            values = []
            for row in range(
                row_start,
                row_stop,
                self.depth_sample_step_px
            ):
                for col in range(
                    col_start,
                    col_stop,
                    self.depth_sample_step_px
                ):
                    distance = self._read_depth_m(msg, encoding, row, col)
                    if self._is_valid_depth(distance):
                        values.append(distance)
            distances[name] = self._percentile(
                values,
                self.depth_roi_percentile
            )
        return distances

    def _read_depth_m(self, msg, encoding, row, col):
        data = msg.data
        if encoding in ('16UC1', 'MONO16'):
            offset = row * msg.step + col * 2
            if offset + 2 > len(data):
                return None
            fmt = '>H' if msg.is_bigendian else '<H'
            value = struct.unpack_from(fmt, data, offset)[0]
            if value == 0:
                return None
            return value / 1000.0

        offset = row * msg.step + col * 4
        if offset + 4 > len(data):
            return None
        fmt = '>f' if msg.is_bigendian else '<f'
        value = struct.unpack_from(fmt, data, offset)[0]
        return value

    def _is_valid_depth(self, distance):
        return (
            distance is not None
            and math.isfinite(distance)
            and self.depth_min_valid_m <= distance <= self.depth_max_valid_m
        )

    def _extract_scan_distances(self, msg):
        values = {
            'left': [],
            'front': [],
            'right': [],
        }
        range_min = msg.range_min if math.isfinite(msg.range_min) else 0.0
        range_max = (
            msg.range_max
            if math.isfinite(msg.range_max) and msg.range_max > 0.0
            else self.depth_max_valid_m
        )

        for index, raw_range in enumerate(msg.ranges):
            if not math.isfinite(raw_range):
                continue
            if raw_range < range_min or raw_range > range_max:
                continue
            angle = msg.angle_min + index * msg.angle_increment
            angle = math.atan2(math.sin(angle), math.cos(angle))
            if abs(angle) <= self.scan_front_half_angle_rad:
                values['front'].append(float(raw_range))
            elif (
                self.scan_side_min_angle_rad
                <= angle
                <= self.scan_side_max_angle_rad
            ):
                values['left'].append(float(raw_range))
            elif (
                -self.scan_side_max_angle_rad
                <= angle
                <= -self.scan_side_min_angle_rad
            ):
                values['right'].append(float(raw_range))

        return {
            name: self._percentile(samples, self.scan_percentile)
            for name, samples in values.items()
        }

    def distance_snapshot(self):
        now = time.monotonic()
        with self._sensor_lock:
            depth_fresh = (
                self.enable_depth_safety
                and self._last_depth_time is not None
                and now - self._last_depth_time <= self.safety_max_age_sec
            )
            scan_fresh = (
                self.enable_scan_safety
                and self._last_scan_time is not None
                and now - self._last_scan_time <= self.safety_max_age_sec
            )
            front_values = []
            left_values = []
            right_values = []
            if depth_fresh:
                front_values.append(self._depth_front_m)
                left_values.append(self._depth_left_m)
                right_values.append(self._depth_right_m)
            if scan_fresh:
                front_values.append(self._scan_front_m)
                left_values.append(self._scan_left_m)
                right_values.append(self._scan_right_m)

        return DistanceSnapshot(
            self._min_valid_distance(front_values),
            self._min_valid_distance(left_values),
            self._min_valid_distance(right_values),
            bool(depth_fresh or scan_fresh)
        )

    @staticmethod
    def _min_valid_distance(values):
        valid = [
            value
            for value in values
            if value is not None and math.isfinite(value)
        ]
        if not valid:
            return None
        return min(valid)

    @staticmethod
    def _percentile(values, percentile):
        if not values:
            return None
        ordered = sorted(values)
        index = int((len(ordered) - 1) * float(percentile))
        return ordered[max(0, min(len(ordered) - 1, index))]

    def _publish_safety_debug_throttled(self, text):
        now = time.monotonic()
        if now - self._last_safety_debug_time < 0.5:
            return
        self._last_safety_debug_time = now
        self.publish_debug(text)

    @staticmethod
    def _format_distance(distance):
        if distance is None:
            return 'none'
        return f'{distance:.3f}m'

    def send_velocity(self, vx, vy, wz):
        check = self._pre_motion_check(None)
        if check is not None:
            return False
        vx, vy, wz = self.clamp_velocity(vx, vy, wz)
        self.motion.move(vx, vy, wz)
        return True

    def zero_velocity(self, reason='zero velocity requested'):
        self.motion.stop(reason, log_level='info')

    def is_robot_stable(self):
        # TODO: Subscribe to Unitree low_state/high_state IMU roll and pitch
        # when those messages are present in the deployment workspace.
        return True

    def check_timeout(self, start_time):
        if start_time is None:
            return False
        return time.monotonic() - start_time > self.max_action_duration_sec

    def clamp_velocity(self, vx, vy, wz):
        vx = self._as_finite_float(vx, 'vx')
        vy = self._as_finite_float(vy, 'vy')
        wz = self._as_finite_float(wz, 'wz')

        clamped = (
            max(-self.max_vx, min(self.max_vx, vx)),
            max(-self.max_vy, min(self.max_vy, vy)),
            max(-self.max_wz, min(self.max_wz, wz)),
        )
        if clamped != (vx, vy, wz):
            self.publish_debug(
                'velocity clamped: '
                f'({vx:.3f}, {vy:.3f}, {wz:.3f}) -> '
                f'({clamped[0]:.3f}, {clamped[1]:.3f}, {clamped[2]:.3f})'
            )
        return clamped

    def _pre_motion_check(self, start_time):
        with self._state_lock:
            emergency_stop = self._emergency_stop
            action_active = self._action_active

        if emergency_stop:
            self.motion.stop('emergency stop active', log_level='warn')
            return CommandResult(
                False,
                STATUS_EMERGENCY_STOP,
                'motion interrupted by STOP'
            )
        if self._active_goal_cancel_requested():
            self.motion.stop('motion action canceled', log_level='warn')
            return CommandResult(
                False,
                STATUS_EMERGENCY_STOP,
                'motion action canceled'
            )
        if start_time is not None and self.check_timeout(start_time):
            self.motion.stop('gait command timeout', log_level='warn')
            return CommandResult(
                False,
                STATUS_TIMEOUT,
                'gait command timed out'
            )
        if action_active and not self.is_robot_stable():
            self.motion.stop('robot unstable', log_level='warn')
            return CommandResult(
                False,
                STATUS_UNSTABLE,
                'robot roll/pitch exceeded stability limit'
            )
        return None

    def _bounded_duration(self, requested, default_duration):
        duration = self._as_finite_float(requested, 'duration_sec')
        if duration <= 0.0:
            duration = float(default_duration)
        capped = duration > self.max_action_duration_sec
        if capped:
            self.publish_debug(
                f'duration {duration:.3f}s exceeds max '
                f'{self.max_action_duration_sec:.3f}s; command will timeout'
            )
            duration = self.max_action_duration_sec
        return duration, capped

    def _publish_periodic_state(self):
        with self._state_lock:
            status = self._status
            mode = self._current_mode
            locked = self._control_locked

        status_msg = String()
        status_msg.data = status
        self.status_pub.publish(status_msg)

        mode_msg = String()
        mode_msg.data = mode
        self.mode_pub.publish(mode_msg)

        lock_msg = Bool()
        lock_msg.data = locked
        self.lock_pub.publish(lock_msg)

    def _float_field(self, fields, name, default):
        return self._as_finite_float(fields.get(name, default), name)

    def _load_practical_obstacle_steps(self):
        names = self._string_array_parameter('practical_obstacle.step_names')
        step_types = self._string_array_parameter(
            'practical_obstacle.step_types'
        )
        distances = self._float_array_parameter(
            'practical_obstacle.step_distance_m'
        )
        turn_angles = self._float_array_parameter(
            'practical_obstacle.step_turn_angle_deg'
        )
        linear_speeds = self._float_array_parameter(
            'practical_obstacle.step_linear_speed'
        )
        turn_speeds = self._float_array_parameter(
            'practical_obstacle.step_turn_speed_deg'
        )
        front_targets = self._float_array_parameter(
            'practical_obstacle.step_front_target_m'
        )
        lengths = {
            len(names),
            len(step_types),
            len(distances),
            len(turn_angles),
            len(linear_speeds),
            len(turn_speeds),
            len(front_targets),
        }
        if len(lengths) != 1:
            raise ValueError(
                'practical_obstacle step arrays must have the same length: '
                'step_names/step_types/step_distance_m/'
                'step_turn_angle_deg/step_linear_speed/step_turn_speed_deg/'
                'step_front_target_m'
            )
        if not names:
            raise ValueError('practical_obstacle requires at least one step')

        steps = []
        for index, name in enumerate(names):
            step = self._make_obstacle_step(
                str(name),
                step_types[index],
                distances[index],
                turn_angles[index],
                linear_speeds[index],
                turn_speeds[index],
                front_targets[index]
            )
            steps.append(step)
        return steps

    def _make_obstacle_step(
        self,
        name,
        step_type,
        distance_m,
        turn_angle_deg,
        linear_speed,
        turn_speed_deg,
        front_target_m
    ):
        normalized_type = str(step_type).strip().lower()
        normalized_type = normalized_type.replace('-', '_').replace(' ', '_')
        distance_m = self._as_finite_float(distance_m, 'step_distance_m')
        turn_angle_deg = self._as_finite_float(
            turn_angle_deg,
            'step_turn_angle_deg'
        )
        linear_speed = self._as_finite_float(
            linear_speed,
            'step_linear_speed'
        )
        turn_speed_deg = self._as_finite_float(
            turn_speed_deg,
            'step_turn_speed_deg'
        )
        front_target_m = self._as_finite_float(
            front_target_m,
            'step_front_target_m'
        )
        if front_target_m < 0.0:
            raise ValueError(f'{name}: step_front_target_m must be nonnegative')

        if normalized_type in ('forward', 'backward'):
            if linear_speed <= 0.0:
                raise ValueError(
                    f'{name}: step_linear_speed must be positive for forward'
                )
            if (
                front_target_m > 0.0
                and front_target_m <= self.front_stop_distance_m
            ):
                raise ValueError(
                    f'{name}: step_front_target_m must be greater than '
                    'obstacle_safety.front_stop_distance_m'
                )
            direction = -1.0 if normalized_type == 'backward' else 1.0
            distance = abs(distance_m)
            duration = distance / linear_speed
            return ObstacleStep(
                name,
                direction * linear_speed,
                0.0,
                duration,
                front_target_m if front_target_m > 0.0 else None
            )

        if normalized_type in ('turn_left', 'left', 'turn_right', 'right'):
            if turn_speed_deg <= 0.0:
                raise ValueError(
                    f'{name}: step_turn_speed_deg must be positive for turn'
                )
            direction = 1.0
            if normalized_type in ('turn_right', 'right'):
                direction = -1.0
            angle = abs(turn_angle_deg)
            duration = angle / turn_speed_deg
            return ObstacleStep(
                name,
                0.0,
                direction * math.radians(turn_speed_deg),
                duration
            )

        raise ValueError(
            f'{name}: unsupported step_type {step_type}; use forward, '
            'backward, turn_left, or turn_right'
        )

    def _validate_obstacle_parameters(self):
        if self.depth_roi_bottom_ratio <= self.depth_roi_top_ratio:
            raise ValueError(
                'obstacle_safety.depth_roi_bottom_ratio must be greater '
                'than depth_roi_top_ratio'
            )
        if self.depth_max_valid_m <= self.depth_min_valid_m:
            raise ValueError(
                'obstacle_safety.depth_max_valid_m must be greater than '
                'depth_min_valid_m'
            )
        if self.front_slow_distance_m <= self.front_stop_distance_m:
            raise ValueError(
                'obstacle_safety.front_slow_distance_m must be greater than '
                'front_stop_distance_m'
            )
        if self.side_warn_distance_m <= self.side_stop_distance_m:
            raise ValueError(
                'obstacle_safety.side_warn_distance_m must be greater than '
                'side_stop_distance_m'
            )
        if self.scan_side_max_angle_rad <= self.scan_side_min_angle_rad:
            raise ValueError(
                'obstacle_safety.scan_side_max_angle_deg must be greater '
                'than scan_side_min_angle_deg'
            )

    @staticmethod
    def _as_finite_float(value, name):
        try:
            number = float(value)
        except (TypeError, ValueError) as error:
            raise ValueError(f'{name} must be a finite number') from error
        if not math.isfinite(number):
            raise ValueError(f'{name} must be a finite number')
        return number

    def _string_parameter(self, name):
        value = str(self.get_parameter(name).value)
        if not value:
            raise ValueError(f'{name} must not be empty')
        return value

    def _bool_parameter(self, name):
        return bool(self.get_parameter(name).value)

    def _positive_float_parameter(self, name):
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f'{name} must be a finite positive number')
        return value

    def _nonnegative_float_parameter(self, name):
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value < 0.0:
            raise ValueError(f'{name} must be a finite nonnegative number')
        return value

    def _ratio_parameter(self, name):
        value = self._positive_float_parameter(name)
        if value > 1.0:
            raise ValueError(f'{name} must be in (0.0, 1.0]')
        return value

    def _positive_int_parameter(self, name):
        value = int(self.get_parameter(name).value)
        if value <= 0:
            raise ValueError(f'{name} must be positive')
        return value

    def _string_array_parameter(self, name):
        value = self.get_parameter(name).value
        if not isinstance(value, (list, tuple)):
            raise ValueError(f'{name} must be a string array')
        return [str(item) for item in value]

    def _float_array_parameter(self, name):
        value = self.get_parameter(name).value
        if not isinstance(value, (list, tuple)):
            raise ValueError(f'{name} must be a float array')
        return [self._as_finite_float(item, name) for item in value]

    def shutdown(self):
        if not rclpy.ok():
            return
        self.motion.stop('gait_control_node shutdown')
        self.publish_lock(False)
        self.publish_mode('IDLE')
        self.publish_status(STATUS_IDLE, 'gait control shutdown')


def main(args=None):
    rclpy.init(args=args)
    node = None
    executor = MultiThreadedExecutor()
    try:
        node = GaitControlNode()
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        if node is not None and rclpy.ok():
            node.get_logger().warn('Interrupted by user')
    finally:
        if node is not None:
            node.shutdown()
            executor.remove_node(node)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
