#!/usr/bin/env python3

import json
import math
import struct
import threading
import time
import uuid
from dataclasses import dataclass, field

import rclpy
from geometry_msgs.msg import Twist
from rclpy.action import ActionServer
from rclpy.action.server import CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.context import Context
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Bool, String

from rk_locomotion.front_jump_supervisor import FrontJumpConfig
from rk_locomotion.front_jump_supervisor import FrontJumpOutcome
from rk_locomotion.front_jump_supervisor import FrontJumpProfile
from rk_locomotion.front_jump_supervisor import FrontJumpSupervisor
from rk_locomotion.front_jump_supervisor import CleanupGuardError
from rk_locomotion.front_jump_supervisor import PersistentCleanupGuard
from rk_interfaces.action import ExecuteMotion


STATUS_IDLE = 'IDLE'
STATUS_RUNNING = 'RUNNING'
STATUS_DONE = 'DONE'
STATUS_FAILED = 'FAILED'
STATUS_TIMEOUT = 'TIMEOUT'
STATUS_UNSTABLE = 'UNSTABLE'
STATUS_EMERGENCY_STOP = 'EMERGENCY_STOP'

_MOTION_SLOT_TRANSITIONS = {
    'RESERVED': {'ACCEPTED', 'STOPPING', 'FINALIZING', 'FAULTED'},
    'ACCEPTED': {'EXECUTING', 'STOPPING', 'FINALIZING', 'FAULTED'},
    'EXECUTING': {'STOPPING', 'FINALIZING', 'FAULTED'},
    'STOPPING': {'FINALIZING', 'FAULTED'},
    'FINALIZING': {'DONE', 'FAULTED'},
    'DONE': set(),
    'FAULTED': {'FINALIZING'},
}


def _sanitize_fault_text(value, limit=512):
    text = ' '.join(str(value).split())
    if not text:
        text = 'unspecified fault'
    return text[:int(limit)]


FINAL_CMD_QOS = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
)
ESTOP_STATE_QOS = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)


@dataclass(frozen=True)
class CommandResult:
    success: bool
    status: str
    message: str


@dataclass(frozen=True)
class LockPublishResult:
    """One serialized request to publish the desired gait-lock state."""

    requested_state: bool
    previous_state: bool
    generation: int
    publish_succeeded: bool
    error_message: str


class _SerializedControlLockPublisher:
    """Order lock transitions and periodic republishes through one mutex."""

    def __init__(
        self,
        publisher,
        state_lock,
        state_update_callback,
        *,
        initial_state=False,
    ):
        self._publisher = publisher
        self._state_lock = state_lock
        self._state_update_callback = state_update_callback
        self._publish_mutex = threading.Lock()
        self._desired_state = bool(initial_state)
        self._generation = 0
        self._lock_publish_fault = False
        self._last_result = None

    @property
    def desired_state(self):
        with self._state_lock:
            return self._desired_state

    @property
    def generation(self):
        with self._state_lock:
            return self._generation

    @property
    def lock_publish_fault(self):
        with self._state_lock:
            return self._lock_publish_fault

    @property
    def last_result(self):
        with self._state_lock:
            return self._last_result

    def set_locked(self, locked):
        requested = bool(locked)
        with self._publish_mutex:
            with self._state_lock:
                previous = self._desired_state
                self._generation += 1
                generation = self._generation
                self._desired_state = requested
                self._state_update_callback(
                    self._desired_state,
                    self._lock_publish_fault,
                )

            error_message = ''
            try:
                msg = Bool()
                msg.data = requested
                self._publisher.publish(msg)
                publish_succeeded = True
            except Exception as error:
                publish_succeeded = False
                error_message = '{}: {}'.format(
                    type(error).__name__,
                    str(error),
                )
                with self._state_lock:
                    self._lock_publish_fault = True
                    if not requested:
                        self._desired_state = True
                    self._state_update_callback(
                        self._desired_state,
                        self._lock_publish_fault,
                    )

            result = LockPublishResult(
                requested_state=requested,
                previous_state=previous,
                generation=generation,
                publish_succeeded=publish_succeeded,
                error_message=error_message,
            )
            with self._state_lock:
                self._last_result = result
            return result

    def republish(self):
        with self._publish_mutex:
            with self._state_lock:
                requested = self._desired_state
                previous = self._desired_state
                generation = self._generation

            error_message = ''
            try:
                msg = Bool()
                msg.data = requested
                self._publisher.publish(msg)
                publish_succeeded = True
            except Exception as error:
                publish_succeeded = False
                error_message = '{}: {}'.format(
                    type(error).__name__,
                    str(error),
                )
                with self._state_lock:
                    self._lock_publish_fault = True
                    if not requested:
                        self._desired_state = True
                    self._state_update_callback(
                        self._desired_state,
                        self._lock_publish_fault,
                    )

            result = LockPublishResult(
                requested_state=requested,
                previous_state=previous,
                generation=generation,
                publish_succeeded=publish_succeeded,
                error_message=error_message,
            )
            with self._state_lock:
                self._last_result = result
            return result


@dataclass
class _MotionExecutionSlot:
    """One reservation shared by every non-STOP Action or JSON motion."""

    reservation_token: str
    entry_type: str
    motion_name: str
    command: str
    identity: str
    state: str = 'RESERVED'
    goal_handle: object = None
    cancel_event: threading.Event = field(default_factory=threading.Event)
    stop_event: threading.Event = field(default_factory=threading.Event)
    completion_event: threading.Event = field(default_factory=threading.Event)
    worker_started_event: threading.Event = field(
        default_factory=threading.Event
    )
    worker_done_event: threading.Event = field(default_factory=threading.Event)
    cancel_accepted: bool = False
    first_abort_reason: str = ''
    terminal_claimed: bool = False
    expected_terminal: str = ''
    terminal_delivery_succeeded: object = None
    fault_type: str = ''
    fault_reason: str = ''
    transitions: list = field(default_factory=lambda: ['RESERVED'])


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
        'FRONT_JUMP_RECOVER',
        'PRACTICAL_OBSTACLE_ZONE',
        'AVOID_ZONE',
        'OBSTACLE_OPEN_LOOP_TEST',
        'ENTER_OBSTACLE_ZONE',
        'OBSTACLE_FORWARD_SLOW',
        'OBSTACLE_TURN_LEFT',
        'OBSTACLE_TURN_RIGHT',
        'OBSTACLE_SIDE_ADJUST',
        'EXIT_OBSTACLE_ZONE',
    }

    def __init__(
        self,
        *,
        node_name='gait_control_node',
        front_jump_supervisor_factory=FrontJumpSupervisor,
        process_runner=None,
        network_interface_validator=None,
        executable_resolver=None,
        **node_kwargs,
    ):
        super().__init__(node_name, **node_kwargs)
        self.callback_group = ReentrantCallbackGroup()
        self.motion_action_callback_group = ReentrantCallbackGroup()
        self.front_jump_callback_group = ReentrantCallbackGroup()

        self._declare_parameters()
        self.cmd_vel_topic = self._string_parameter('cmd_vel_topic')
        self.command_json_topic = self._string_parameter('command_json_topic')
        self.motion_action_name = self._string_parameter('motion_action_name')
        self.enable_motion_action = self._bool_parameter(
            'enable_motion_action'
        )

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
        self.front_jump_profiles = {
            profile_name: FrontJumpProfile(
                name=profile_name,
                pre_stop_duration=self._front_jump_nonnegative_float_parameter(
                    'front_jump.{}.pre_stop_duration'.format(profile_name)
                ),
                final_zero_epsilon=self._front_jump_positive_float_parameter(
                    'front_jump.{}.final_zero_epsilon'.format(profile_name)
                ),
                final_zero_confirm_samples=(
                    self._front_jump_positive_int_parameter(
                        'front_jump.{}.final_zero_confirm_samples'.format(
                            profile_name
                        )
                    )
                ),
                final_zero_timeout=self._front_jump_positive_float_parameter(
                    'front_jump.{}.final_zero_timeout'.format(profile_name)
                ),
                sdk_timeout=self._front_jump_positive_float_parameter(
                    'front_jump.{}.sdk_timeout'.format(profile_name)
                ),
                post_settle_duration=(
                    self._front_jump_nonnegative_float_parameter(
                        'front_jump.{}.post_settle_duration'.format(
                            profile_name
                        )
                    )
                ),
            )
            for profile_name in ('start', 'finish')
        }
        self.front_jump_config = FrontJumpConfig(
            sdk_action_executable=self._front_jump_string_parameter(
                'front_jump.sdk_action_executable'
            ),
            sdk_network_interface=self._front_jump_string_parameter(
                'front_jump.sdk_network_interface'
            ),
            zero_publish_rate_hz=self._front_jump_positive_float_parameter(
                'front_jump.zero_publish_rate_hz'
            ),
            final_cmd_stale_timeout=(
                self._front_jump_positive_float_parameter(
                    'front_jump.final_cmd_stale_timeout'
                )
            ),
            estop_state_stale_timeout=(
                self._front_jump_positive_float_parameter(
                    'front_jump.estop_state_stale_timeout'
                )
            ),
        )
        self.front_jump_final_cmd_topic = self._front_jump_string_parameter(
            'front_jump.final_cmd_topic'
        )
        self.front_jump_cmd_mux_status_topic = (
            self._front_jump_string_parameter(
                'front_jump.cmd_mux_status_topic'
            )
        )
        self.front_jump_estop_state_topic = (
            self._front_jump_string_parameter(
                'front_jump.estop_state_topic'
            )
        )
        self.front_jump_cleanup_guard_path = (
            self._front_jump_string_parameter(
                'front_jump.cleanup_guard_path'
            )
        )
        self.front_jump_shutdown_drain_timeout_sec = (
            self._front_jump_positive_float_parameter(
                'front_jump.shutdown_drain_timeout_sec'
            )
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
        self._lock_publish_fault = False
        self._accept_new_motion = True
        self._motion_slot = None
        self._json_command_sequence = 0
        self._stop_sequence = 0
        self._safety_faults = {}
        self._terminal_claims_without_slot = {}
        self._active_action_worker_count = 0
        self._action_workers_done_event = threading.Event()
        self._action_workers_done_event.set()
        self._shutdown_requested = threading.Event()
        self._ros_cleanup_allowed = threading.Event()
        self._ros_cleanup_allowed.set()
        self._shutdown_prepared = False
        self._shutdown_prepare_clean = False
        self._shutdown_commit_called = False
        self._fatal_shutdown_fault = False
        self._cleanup_guard = PersistentCleanupGuard(
            self.front_jump_cleanup_guard_path
        )
        self._cleanup_guard_record = None
        self._full_boot_recovery_guard_id = ''
        self._full_boot_recovery_baseline = None
        self._full_boot_recovery_fault_types = set()
        self._full_boot_recovery_fault_ids = set()
        self._full_boot_recovery_stop_sequence = 0
        self._full_boot_recovery_in_progress = False
        self._action_active = False
        self._emergency_stop = False
        self._active_goal_handle = None
        self._active_feedback_callback = None
        self._front_jump_goal_handle = None
        self._front_jump_action_goal_handle = None
        self._front_jump_cancel_event = None
        self._front_jump_stop_event = None
        self._initialize_cleanup_guard_state()

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
        self._control_lock_publisher = _SerializedControlLockPublisher(
            self.lock_pub,
            self._state_lock,
            self._update_control_lock_state_locked,
            initial_state=False,
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
        supervisor_kwargs = {
            'profiles': self.front_jump_profiles,
            'config': self.front_jump_config,
            'publish_lock': self.publish_lock,
            'publish_zero': self._publish_front_jump_zero,
            'event_logger': self._log_front_jump_event,
            'cleanup_guard': self._cleanup_guard,
            'ros_cleanup_allowed': (
                lambda: self._ros_cleanup_allowed.is_set()
            ),
        }
        if process_runner is not None:
            supervisor_kwargs['process_runner'] = process_runner
        if network_interface_validator is not None:
            supervisor_kwargs['interface_index_resolver'] = (
                network_interface_validator
            )
        if executable_resolver is not None:
            supervisor_kwargs['executable_resolver'] = executable_resolver
        self.front_jump_supervisor = front_jump_supervisor_factory(
            **supervisor_kwargs
        )
        if self._full_boot_recovery_guard_id:
            self._full_boot_recovery_baseline = (
                self.front_jump_supervisor.begin_recovery_window()
            )
            self._full_boot_recovery_stop_sequence = (
                self._stop_sequence
            )

        self.command_json_sub = self.create_subscription(
            String,
            self.command_json_topic,
            self._on_command_json,
            10,
            callback_group=self.callback_group
        )
        self.front_jump_final_cmd_sub = self.create_subscription(
            Twist,
            self.front_jump_final_cmd_topic,
            self._on_front_jump_final_cmd,
            FINAL_CMD_QOS,
            callback_group=self.front_jump_callback_group,
        )
        self.front_jump_estop_state_sub = self.create_subscription(
            Bool,
            self.front_jump_estop_state_topic,
            self._on_front_jump_estop_state,
            ESTOP_STATE_QOS,
            callback_group=self.front_jump_callback_group,
        )
        self.front_jump_cmd_mux_status_sub = self.create_subscription(
            String,
            self.front_jump_cmd_mux_status_topic,
            self._on_front_jump_cmd_mux_status,
            10,
            callback_group=self.front_jump_callback_group,
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

        self.action_server = None
        if self.enable_motion_action:
            self.action_server = ActionServer(
                self,
                ExecuteMotion,
                self.motion_action_name,
                self._execute_motion_action,
                goal_callback=self._motion_goal_callback,
                handle_accepted_callback=(
                    self._motion_handle_accepted_callback
                ),
                cancel_callback=self._motion_cancel_callback,
                callback_group=self.motion_action_callback_group
            )
        else:
            self.get_logger().warn(
                'Motion action server disabled; use /gait/command_json.'
            )

        status_period = 1.0 / self._positive_float_parameter(
            'status_publish_rate_hz'
        )
        self.status_timer = self.create_timer(
            status_period,
            self._publish_periodic_state,
            callback_group=self.callback_group
        )

        initial_lock_required = bool(self._safety_faults)
        initial_lock_result = self.publish_lock(initial_lock_required)
        if (
            initial_lock_required
            and initial_lock_result.publish_succeeded
            and self._cleanup_guard.current_record is not None
        ):
            try:
                def mark_initial_lock(guard_record):
                    guard_record['lock'][
                        'lock_acquire_command_published'
                    ] = True
                    guard_record['lock']['generation'] = (
                        initial_lock_result.generation
                    )

                self._cleanup_guard.update(mark_initial_lock)
            except Exception as error:
                self._fault_current_motion_slot(
                    'cleanup_guard_fault',
                    'initial lock evidence update failed: {}: {}'.format(
                        type(error).__name__,
                        str(error),
                    ),
                )
        if self._safety_faults:
            self.publish_status(
                STATUS_FAILED,
                'gait control started with latched safety fault: {}'.format(
                    ','.join(sorted(self._safety_faults))
                ),
            )
        else:
            self.publish_status(STATUS_IDLE, 'gait control ready')
        self.publish_mode('IDLE')
        self.get_logger().info(
            'Gait control node ready: json_topic='
            f'{self.command_json_topic}, action='
            f'{"enabled" if self.enable_motion_action else "disabled"}'
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
            'cmd_vel_topic': '/control/locomotion_cmd',
            'status_topic': '/gait/status',
            'control_lock_topic': '/gait/control_lock',
            'debug_topic': '/gait/debug',
            'current_mode_topic': '/gait/current_mode',
            'command_json_topic': '/gait/command_json',
            'motion_action_name': '/locomotion/execute_motion',
            'enable_motion_action': True,
            'default_max_vx': 0.60,
            'default_max_vy': 0.15,
            'default_max_wz': 1.00,
            'hard_max_vx': 0.60,
            'hard_max_vy': 0.15,
            'hard_max_wz': 1.00,
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
            'front_jump.start.pre_stop_duration': 0.5,
            'front_jump.start.final_zero_epsilon': 0.001,
            'front_jump.start.final_zero_confirm_samples': 3,
            'front_jump.start.final_zero_timeout': 2.0,
            'front_jump.start.sdk_timeout': 12.0,
            'front_jump.start.post_settle_duration': 2.5,
            'front_jump.finish.pre_stop_duration': 0.5,
            'front_jump.finish.final_zero_epsilon': 0.001,
            'front_jump.finish.final_zero_confirm_samples': 3,
            'front_jump.finish.final_zero_timeout': 2.0,
            'front_jump.finish.sdk_timeout': 12.0,
            'front_jump.finish.post_settle_duration': 2.5,
            'front_jump.sdk_action_executable': 'go2_sdk_motion_action',
            'front_jump.sdk_network_interface': 'eth0',
            'front_jump.zero_publish_rate_hz': 10.0,
            'front_jump.final_cmd_topic': '/navigation/cmd_vel',
            'front_jump.final_cmd_stale_timeout': 0.20,
            'front_jump.cmd_mux_status_topic': '/control/cmd_mux_status',
            'front_jump.estop_state_topic': '/safety/estop_state',
            'front_jump.estop_state_stale_timeout': 0.20,
            'front_jump.cleanup_guard_path': (
                '~/rk_line_runtime/front_jump_cleanup_guard.json'
            ),
            'front_jump.shutdown_drain_timeout_sec': 5.0,
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
            'obstacle_safety.slow_vx': 0.25,
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
            'practical_obstacle.enter_vx': 0.30,
            'practical_obstacle.enter_duration_sec': 6.0,
            'practical_obstacle.forward_vx': 0.30,
            'practical_obstacle.forward_duration_sec': 5.0,
            'practical_obstacle.turn_wz': 0.80,
            'practical_obstacle.turn_duration_sec': 4.8,
            'practical_obstacle.side_adjust_wz': 0.18,
            'practical_obstacle.side_adjust_turn_sec': 0.45,
            'practical_obstacle.side_adjust_forward_sec': 0.45,
            'practical_obstacle.exit_vx': 0.30,
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
                0.30,
                0.00,
                0.30,
                0.00,
                0.30,
                0.00,
                0.30,
                0.00,
                0.30,
                0.00,
                0.30,
            ],
            'practical_obstacle.step_turn_speed_deg': [
                0.0,
                50.0,
                0.0,
                50.0,
                0.0,
                50.0,
                0.0,
                50.0,
                0.0,
                50.0,
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

    def _initialize_cleanup_guard_state(self):
        try:
            record = self._cleanup_guard.load()
        except CleanupGuardError as error:
            self._safety_faults['cleanup_guard_fault'] = str(error)
            self._accept_new_motion = False
            return
        if record is None:
            return

        self._cleanup_guard_record = record
        if record['state'] == 'CLEAN':
            try:
                self._cleanup_guard.clear(record['cleanup_fault_id'])
                self._cleanup_guard_record = None
                return
            except CleanupGuardError as error:
                self._safety_faults['cleanup_guard_fault'] = str(error)
                self._accept_new_motion = False
                return
        process_absent, process_reason = (
            FrontJumpSupervisor.guard_process_absent(record)
        )
        if (
            record['boot_id'] != self._cleanup_guard.current_boot_id
            and process_absent
        ):
            self._full_boot_recovery_guard_id = (
                record['cleanup_fault_id']
            )

        fault_records = record.get('faults', [])
        self._full_boot_recovery_fault_types = {
            str(fault.get('fault_type', 'cleanup_fault'))
            for fault in fault_records
        } or {'cleanup_fault'}
        self._full_boot_recovery_fault_ids = {
            str(fault.get('fault_id', ''))
            for fault in fault_records
        }
        if fault_records:
            for fault in fault_records:
                fault_type = str(
                    fault.get('fault_type', 'cleanup_fault')
                )
                self._safety_faults[fault_type] = (
                    str(fault.get('reason', '')).strip()
                    or process_reason
                    or 'dirty cleanup guard requires recovery'
                )
        else:
            self._safety_faults['cleanup_fault'] = (
                process_reason
                or 'dirty cleanup guard requires recovery'
            )
        self._accept_new_motion = False

    def _on_command_json(self, msg):
        try:
            fields = json.loads(msg.data)
        except json.JSONDecodeError as error:
            self.publish_debug(
                f'rejected invalid /gait/command_json: {error}'
            )
            return

        if not isinstance(fields, dict):
            self.publish_debug(
                '/gait/command_json must contain a JSON object'
            )
            return

        result = self.handle_command(fields, entry_type='json')
        self.publish_debug(
            'JSON command result: '
            f'success={result.success}, status={result.status}'
        )

    def _motion_goal_callback(self, goal_request):
        motion_name = str(goal_request.motion_name or '').strip()
        if not motion_name:
            self.get_logger().warn('Rejected empty motion goal')
            return GoalResponse.REJECT

        fields = self.motion_name_to_command_fields(motion_name)
        if fields is None:
            self.get_logger().warn(
                f'Rejected unsupported motion goal: {motion_name}'
            )
            return GoalResponse.REJECT

        command = str(fields.get('command', '')).strip().upper()
        if command in ('STOP', 'OBSTACLE_STOP'):
            return GoalResponse.ACCEPT

        slot, reason = self._try_reserve_motion_slot(
            entry_type='action',
            motion_name=motion_name,
            command=command,
            identity='pending_uuid',
        )
        if slot is None:
            self.get_logger().warn(
                f'Rejected motion goal {motion_name}: {reason}'
            )
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _motion_handle_accepted_callback(self, goal_handle):
        motion_name = str(goal_handle.request.motion_name or '').strip()
        fields = self.motion_name_to_command_fields(motion_name)
        command = (
            ''
            if fields is None
            else str(fields.get('command', '')).strip().upper()
        )

        if command in ('STOP', 'OBSTACLE_STOP'):
            try:
                goal_handle.execute()
            except Exception as error:
                self._latch_action_fault(
                    'action_reservation_fault',
                    'STOP handle execute failed: {}: {}'.format(
                        type(error).__name__,
                        str(error),
                    ),
                )
                self._attempt_action_terminal(
                    goal_handle,
                    None,
                    'abort',
                )
            return

        try:
            goal_uuid = self._front_jump_goal_id(goal_handle)
            if not goal_uuid:
                raise ValueError('accepted goal UUID is unavailable')
            with self._state_lock:
                slot = self._motion_slot
                if (
                    slot is None
                    or slot.entry_type != 'action'
                    or slot.state not in ('RESERVED', 'STOPPING')
                    or slot.motion_name != motion_name
                ):
                    raise RuntimeError(
                        'accepted goal has no matching reservation'
                    )
                slot.goal_handle = goal_handle
                slot.identity = goal_uuid
                if slot.state != 'STOPPING':
                    self._transition_motion_slot_locked(slot, 'ACCEPTED')
        except Exception as error:
            faulted_slot = self._fault_current_motion_slot(
                'action_reservation_fault',
                'handle binding failed: {}: {}'.format(
                    type(error).__name__,
                    str(error),
                ),
            )
            terminal_ok = self._attempt_action_terminal(
                goal_handle,
                faulted_slot,
                'abort',
            )
            self._complete_motion_slot(
                faulted_slot,
                terminal_ok=terminal_ok,
            )
            return

        try:
            goal_handle.execute()
        except Exception as error:
            self._fault_current_motion_slot(
                'action_reservation_fault',
                'goal_handle.execute failed: {}: {}'.format(
                    type(error).__name__,
                    str(error),
                ),
            )
            self._attempt_action_terminal(goal_handle, slot, 'abort')

    def _motion_cancel_callback(self, goal_handle):
        with self._state_lock:
            slot = self._motion_slot
            if (
                slot is None
                or slot.entry_type != 'action'
                or slot.goal_handle is not goal_handle
                or slot.state not in (
                    'ACCEPTED',
                    'EXECUTING',
                    'STOPPING',
                )
                or slot.terminal_claimed
            ):
                return CancelResponse.REJECT
            slot.cancel_accepted = True
            slot.cancel_event.set()
            if not slot.first_abort_reason:
                slot.first_abort_reason = 'cancel_requested'
            self._transition_motion_slot_locked(slot, 'STOPPING')
            is_front_jump = self._is_front_jump_motion_name(
                slot.motion_name
            )
            if not is_front_jump:
                self._emergency_stop = True
        if is_front_jump:
            self.front_jump_supervisor.wake()
        return CancelResponse.ACCEPT

    def _execute_motion_action(self, goal_handle):
        with self._state_lock:
            self._active_action_worker_count += 1
            self._action_workers_done_event.clear()
        try:
            return self._execute_motion_action_impl(goal_handle)
        finally:
            with self._state_lock:
                self._active_action_worker_count -= 1
                if self._active_action_worker_count == 0:
                    self._action_workers_done_event.set()

    def _execute_motion_action_impl(self, goal_handle):
        motion_name = str(goal_handle.request.motion_name or '').strip()
        fields = self.motion_name_to_command_fields(motion_name)
        result_msg = ExecuteMotion.Result()

        if fields is None:
            message = f'unsupported motion_name: {motion_name}'
            result_msg.success = False
            terminal_ok = self._attempt_action_terminal(
                goal_handle,
                None,
                'abort',
            )
            result_msg.message = self._action_result_message(
                message,
                'abort',
                terminal_ok,
            )
            return result_msg

        command = str(fields.get('command', '')).strip().upper()
        if command in ('STOP', 'OBSTACLE_STOP'):
            result = self.execute_stop()
            terminal_state = 'succeed' if result.success else 'abort'
            terminal_ok = self._attempt_action_terminal(
                goal_handle,
                None,
                terminal_state,
            )
            result_msg.success = bool(
                result.success
                and terminal_state == 'succeed'
                and terminal_ok
            )
            result_msg.message = self._action_result_message(
                result.message,
                terminal_state,
                terminal_ok,
            )
            return result_msg

        with self._state_lock:
            current_slot = self._motion_slot
            if (
                current_slot is None
                or current_slot.entry_type != 'action'
                or current_slot.goal_handle is not goal_handle
                or current_slot.state not in ('ACCEPTED', 'STOPPING')
            ):
                slot = None
            else:
                slot = current_slot
                if slot.state == 'ACCEPTED':
                    self._transition_motion_slot_locked(slot, 'EXECUTING')
                slot.worker_started_event.set()
        if slot is None:
            message = 'accepted action has no executable reservation'
            self._latch_action_fault(
                'action_reservation_fault',
                message,
            )
            if self._is_front_jump_motion_name(motion_name):
                message = FrontJumpOutcome(
                    success=False,
                    terminal_state='abort',
                    stage='action_reservation',
                    reason='accepted_action_has_no_executable_reservation',
                    helper_started=False,
                    sdk_request_may_have_been_sent=False,
                    cleanup_completed=False,
                    sdk_command_accepted=False,
                    post_settle_completed=False,
                ).message()
            result_msg.success = False
            terminal_ok = self._attempt_action_terminal(
                goal_handle,
                (
                    current_slot
                    if (
                        current_slot is not None
                        and current_slot.entry_type == 'action'
                        and current_slot.goal_handle is goal_handle
                    )
                    else None
                ),
                'abort',
            )
            result_msg.message = self._action_result_message(
                message,
                'abort',
                terminal_ok,
            )
            return result_msg

        if command in ('JUMP_START_OBSTACLE', 'JUMP_END_OBSTACLE'):
            return self._execute_front_jump_action(
                goal_handle,
                'start_jump'
                if command == 'JUMP_START_OBSTACLE'
                else 'finish_jump',
                slot,
            )

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
            result = self.handle_command(
                fields,
                slot=slot,
                entry_type='action',
            )
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

        terminal_state = self._terminal_state_for_slot(slot, result.success)
        result_msg.success = False
        terminal_ok = self._attempt_action_terminal(
            goal_handle,
            slot,
            terminal_state,
        )
        self._complete_motion_slot(slot, terminal_ok=terminal_ok)
        terminal_fault = bool(slot.fault_type)
        cleanup_completed = bool(
            slot.state == 'DONE'
            and slot.worker_done_event.is_set()
            and slot.completion_event.is_set()
        )
        result_msg.success = bool(
            result.success
            and terminal_state == 'succeed'
            and terminal_ok
            and not terminal_fault
            and cleanup_completed
        )
        result_msg.message = self._action_result_message(
            result.message,
            terminal_state,
            terminal_ok,
        )
        return result_msg

    def _execute_front_jump_action(self, goal_handle, motion_name, slot):
        result_msg = ExecuteMotion.Result()

        def feedback_callback(current_step, progress):
            feedback = ExecuteMotion.Feedback()
            feedback.current_step = str(current_step)
            feedback.progress = float(max(0.0, min(1.0, progress)))
            goal_handle.publish_feedback(feedback)

        try:
            outcome = self._run_front_jump(
                motion_name,
                slot=slot,
                goal_handle=goal_handle,
                feedback_callback=feedback_callback,
            )
        except Exception as error:
            reason = (
                'FrontJump node delivery failed: {}: {}'
                .format(type(error).__name__, str(error))
            )
            self._latch_action_fault(
                'action_terminal_delivery_fault',
                reason,
            )
            outcome = FrontJumpOutcome(
                success=False,
                terminal_state='abort',
                stage='finally_keep_zero_and_release_lock',
                reason='node_delivery_failed',
                helper_started=False,
                sdk_request_may_have_been_sent=False,
                cleanup_completed=False,
                sdk_command_accepted=False,
                post_settle_completed=False,
            )

        with self._state_lock:
            owns_action = self._motion_slot is slot
            cancel_accepted = slot.cancel_accepted
            stop_requested = slot.stop_event.is_set()
        if owns_action and cancel_accepted:
            outcome = self._front_jump_late_stop_outcome(
                outcome,
                terminal_state='canceled',
                reason='cancel_requested',
            )
        elif owns_action and outcome.success and stop_requested:
            outcome = self._front_jump_late_stop_outcome(
                outcome,
                terminal_state='abort',
                reason=slot.first_abort_reason or 'gait_stop_requested',
            )

        result_msg.success = False
        try:
            self.publish_status(
                self._front_jump_outcome_status(outcome),
                outcome.message(),
            )
        except Exception as error:
            self._latch_action_fault(
                'action_terminal_delivery_fault',
                'FrontJump status publish failed: {}: {}'.format(
                    type(error).__name__,
                    str(error),
                ),
            )
        terminal_ok = self._attempt_action_terminal(
            goal_handle,
            slot,
            outcome.terminal_state,
        )
        self._complete_motion_slot(slot, terminal_ok=terminal_ok)
        terminal_fault = bool(slot.fault_type)
        result_msg.success = bool(
            outcome.success
            and outcome.terminal_state == 'succeed'
            and outcome.cleanup_completed
            and terminal_ok
            and not terminal_fault
        )
        result_msg.message = self._action_result_message(
            outcome.message(),
            outcome.terminal_state,
            terminal_ok,
        )
        return result_msg

    def _try_reserve_motion_slot(
        self,
        *,
        entry_type,
        motion_name,
        command,
        identity,
    ):
        with self._state_lock:
            reason = self._motion_admission_reason_locked()
            if reason:
                return None, reason
            if self._motion_slot is not None:
                return None, 'another motion owns the execution slot'
            if entry_type == 'json':
                self._json_command_sequence += 1
                identity = 'json-{}'.format(self._json_command_sequence)
            slot = _MotionExecutionSlot(
                reservation_token=uuid.uuid4().hex,
                entry_type=str(entry_type),
                motion_name=str(motion_name),
                command=str(command),
                identity=str(identity),
            )
            self._motion_slot = slot
            self._action_active = True
            return slot, ''

    def _motion_admission_reason_locked(self):
        if not self._accept_new_motion:
            return 'node is not accepting new motion'
        if self._safety_faults:
            return 'safety fault is latched: {}'.format(
                ','.join(sorted(self._safety_faults))
            )
        if (
            hasattr(self, '_control_lock_publisher')
            and self._control_lock_publisher.lock_publish_fault
        ):
            return 'lock publish fault is latched'
        return ''

    def _fault_current_motion_slot(self, fault_type, reason):
        fault_type = _sanitize_fault_text(fault_type, limit=128)
        reason = _sanitize_fault_text(reason)
        with self._state_lock:
            slot = self._motion_slot
            if slot is not None:
                # A running worker must finish its own cleanup and terminal
                # attempt before completion is observable.  STOPPING blocks
                # new work without claiming that the worker is already done.
                if slot.state not in ('FINALIZING', 'DONE', 'FAULTED'):
                    self._transition_motion_slot_locked(slot, 'STOPPING')
                slot.fault_type = fault_type
                slot.fault_reason = reason
            self._safety_faults[fault_type] = reason
            self._accept_new_motion = False
            operation = {}
            if slot is not None:
                operation = {
                    'reservation_token': slot.reservation_token,
                    'entry_type': slot.entry_type,
                    'motion_name': slot.motion_name,
                    'goal_uuid': (
                        slot.identity
                        if slot.entry_type == 'action'
                        else ''
                    ),
                    'command_identity': slot.identity,
                }
        try:
            record = self._cleanup_guard.record_fault(
                fault_type,
                reason,
                operation=operation,
            )
            self._cleanup_guard_record = record
        except Exception as error:
            with self._state_lock:
                self._safety_faults['cleanup_guard_fault'] = (
                    '{}: {}'.format(type(error).__name__, str(error))
                )
        return slot

    def _transition_motion_slot_locked(self, slot, target_state):
        """Apply one checked transition while ``_state_lock`` is held."""

        target_state = str(target_state)
        current_state = slot.state
        if current_state == target_state:
            return True
        allowed = _MOTION_SLOT_TRANSITIONS.get(current_state, set())
        if target_state in allowed:
            slot.state = target_state
            slot.transitions.append(target_state)
            return True

        reason = 'illegal motion slot transition {}->{}'.format(
            current_state,
            target_state,
        )
        slot.fault_type = 'motion_slot_transition_fault'
        slot.fault_reason = reason
        if current_state != 'FAULTED':
            slot.state = 'FAULTED'
            slot.transitions.append('FAULTED')
        self._safety_faults['motion_slot_transition_fault'] = reason
        self._accept_new_motion = False
        return False

    def _transition_motion_slot(self, slot, target_state):
        """Checked public transition helper used by tests and callbacks."""

        with self._state_lock:
            transitioned = self._transition_motion_slot_locked(
                slot,
                target_state,
            )
            reason = slot.fault_reason if not transitioned else ''
        if not transitioned:
            self._fault_current_motion_slot(
                'motion_slot_transition_fault',
                reason,
            )
        return transitioned

    def _latch_action_fault(self, fault_type, reason):
        reason = _sanitize_fault_text(reason)
        self._fault_current_motion_slot(fault_type, reason)
        try:
            self.get_logger().error(
                '{}: {}'.format(fault_type, reason)
            )
        except Exception:
            pass

    @staticmethod
    def _terminal_state_for_slot(slot, command_succeeded):
        if slot is not None and slot.cancel_accepted:
            return 'canceled'
        if command_succeeded:
            return 'succeed'
        return 'abort'

    @staticmethod
    def _action_result_message(message, desired_terminal, terminal_ok):
        return (
            '{};desired_terminal={};terminal_delivery_succeeded={};'
            'terminal_delivery_fault={}'
        ).format(
            str(message),
            str(desired_terminal),
            str(bool(terminal_ok)).lower(),
            str(not bool(terminal_ok)).lower(),
        )

    def _attempt_action_terminal(self, goal_handle, slot, terminal_state):
        with self._state_lock:
            if slot is not None:
                if slot.terminal_claimed:
                    return False
                slot.terminal_claimed = True
                slot.expected_terminal = str(terminal_state)
                if slot.state != 'FINALIZING':
                    self._transition_motion_slot_locked(slot, 'FINALIZING')
            else:
                goal_uuid = self._front_jump_goal_id(goal_handle)
                terminal_key = (
                    'uuid:{}'.format(goal_uuid)
                    if goal_uuid
                    else 'handle:{}'.format(id(goal_handle))
                )
                if terminal_key in self._terminal_claims_without_slot:
                    return False
                self._terminal_claims_without_slot[terminal_key] = str(
                    terminal_state
                )

        try:
            if terminal_state == 'succeed':
                goal_handle.succeed()
            elif terminal_state == 'canceled':
                goal_handle.canceled()
            elif terminal_state == 'abort':
                goal_handle.abort()
            else:
                raise ValueError(
                    f'unsupported Action terminal state: {terminal_state}'
                )
            with self._state_lock:
                if slot is not None:
                    slot.terminal_delivery_succeeded = True
            return True
        except Exception as error:
            reason = (
                'expected_terminal={};cancel_accepted={};'
                'terminal_delivery_failed=true;exception_type={};error={}'
                .format(
                    terminal_state,
                    str(
                        bool(slot is not None and slot.cancel_accepted)
                    ).lower(),
                    type(error).__name__,
                    str(error),
                )
            )
            self._fault_current_motion_slot(
                'action_terminal_delivery_fault',
                reason,
            )
            with self._state_lock:
                if slot is not None:
                    slot.terminal_delivery_succeeded = False
            try:
                self.get_logger().error(reason)
            except Exception:
                pass
            return False

    def _complete_motion_slot(self, slot, *, terminal_ok=True):
        if slot is None:
            return
        with self._state_lock:
            if slot.state not in ('FINALIZING', 'FAULTED'):
                self._transition_motion_slot_locked(slot, 'FINALIZING')
            # Clean local references before notifying observers. Completion
            # means worker, external terminal attempt, and slot cleanup are
            # all finished, not merely that a fault was noticed.
            if self._motion_slot is slot and not (
                terminal_ok and not slot.fault_type
            ):
                self._action_active = False
                self._front_jump_goal_handle = None
                self._front_jump_action_goal_handle = None
                self._front_jump_cancel_event = None
                self._front_jump_stop_event = None
                self._active_goal_handle = None
                self._active_feedback_callback = None
            slot.worker_done_event.set()
            if not terminal_ok or slot.fault_type:
                self._transition_motion_slot_locked(slot, 'FAULTED')
                slot.completion_event.set()
                return
            self._transition_motion_slot_locked(slot, 'DONE')
            if self._motion_slot is slot:
                self._motion_slot = None
                self._action_active = False
                self._emergency_stop = False
                self._front_jump_goal_handle = None
                self._front_jump_action_goal_handle = None
                self._front_jump_cancel_event = None
                self._front_jump_stop_event = None
            slot.completion_event.set()

    @staticmethod
    def _front_jump_late_stop_outcome(
        outcome,
        *,
        terminal_state,
        reason,
    ):
        return FrontJumpOutcome(
            success=False,
            terminal_state=terminal_state,
            stage='finally_keep_zero_and_release_lock',
            reason=reason,
            helper_started=outcome.helper_started,
            sdk_request_may_have_been_sent=(
                outcome.sdk_request_may_have_been_sent
            ),
            cleanup_completed=outcome.cleanup_completed,
            sdk_command_accepted=outcome.sdk_command_accepted,
            post_settle_completed=outcome.post_settle_completed,
        )

    def _run_front_jump(
        self,
        motion_name,
        *,
        slot,
        goal_handle=None,
        feedback_callback=None,
    ):
        with self._state_lock:
            if self._motion_slot is not slot:
                return FrontJumpOutcome(
                    success=False,
                    terminal_state='abort',
                    stage='acquire_gait_lock',
                    reason='motion_slot_not_owned',
                    helper_started=False,
                    sdk_request_may_have_been_sent=False,
                    cleanup_completed=True,
                    sdk_command_accepted=False,
                    post_settle_completed=False,
                )
            self._status = STATUS_RUNNING
            self._current_mode = motion_name.upper()
            self._front_jump_goal_handle = goal_handle
            self._front_jump_action_goal_handle = goal_handle
            self._front_jump_cancel_event = slot.cancel_event
            self._front_jump_stop_event = slot.stop_event

        self.publish_mode(motion_name.upper())
        self.publish_status(
            STATUS_RUNNING,
            '{} supervised flow started'.format(motion_name),
        )
        goal_id = self._front_jump_goal_id(goal_handle)
        outcome = self.front_jump_supervisor.run(
            motion_name,
            goal_id=goal_id,
            cancel_requested=slot.cancel_event,
            gait_stop_requested=slot.stop_event,
            feedback_callback=feedback_callback,
            reservation_token=slot.reservation_token,
            entry_type=slot.entry_type,
            command_identity=slot.identity,
        )
        if not outcome.cleanup_completed:
            fault_type = (
                'cleanup_guard_fault'
                if outcome.reason
                == (
                    'helper_start_failed_process_identity_'
                    'cleanup_unverified'
                )
                else 'cleanup_fault'
            )
            self._fault_current_motion_slot(
                fault_type,
                outcome.message(),
            )

        with self._state_lock:
            self._front_jump_goal_handle = None
            self._current_mode = 'IDLE'

        self.publish_mode('IDLE')
        return outcome

    @classmethod
    def _is_front_jump_motion_name(cls, motion_name):
        normalized = cls._normalize_motion_name(motion_name)
        return normalized in {
            'start_jump',
            'jump_start_obstacle',
            'finish_jump',
            'end_jump',
            'jump_end_obstacle',
        }

    @staticmethod
    def _front_jump_goal_id(goal_handle):
        if goal_handle is None:
            return ''
        try:
            return bytes(goal_handle.goal_id.uuid).hex()
        except (AttributeError, TypeError, ValueError):
            return ''

    @staticmethod
    def _front_jump_outcome_status(outcome):
        if outcome.success:
            return STATUS_DONE
        if 'timeout' in outcome.reason:
            return STATUS_TIMEOUT
        if (
            outcome.terminal_state == 'canceled'
            or 'estop' in outcome.reason
            or 'gait_stop' in outcome.reason
        ):
            return STATUS_EMERGENCY_STOP
        return STATUS_FAILED

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
            'open_loop_obstacle_test': {
                'command': 'OBSTACLE_OPEN_LOOP_TEST'
            },
            'obstacle_open_loop_test': {
                'command': 'OBSTACLE_OPEN_LOOP_TEST'
            },
            'avoid_zone_open_loop': {
                'command': 'OBSTACLE_OPEN_LOOP_TEST'
            },
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

    def handle_command(
        self,
        fields,
        *,
        slot=None,
        entry_type='json',
    ):
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

        if command == 'FRONT_JUMP_RECOVER':
            return self._execute_front_jump_recovery(fields)

        owns_json_slot = False
        if slot is None:
            slot, reason = self._try_reserve_motion_slot(
                entry_type=entry_type,
                motion_name=command.lower(),
                command=command,
                identity='pending',
            )
            if slot is None:
                return CommandResult(False, STATUS_FAILED, reason)
            owns_json_slot = entry_type == 'json'

        with self._state_lock:
            if self._motion_slot is not slot:
                return CommandResult(
                    False,
                    STATUS_FAILED,
                    'motion execution slot is not owned',
                )
            if slot.state not in ('ACCEPTED', 'RESERVED', 'EXECUTING'):
                return CommandResult(
                    False,
                    STATUS_FAILED,
                    f'motion slot is not executable: {slot.state}',
                )
            # JSON commands share the exact reservation lifecycle with
            # Actions.  They have no ROS goal-handle callback, so bind their
            # local reservation through ACCEPTED before EXECUTING rather than
            # taking an illegal RESERVED->EXECUTING shortcut.
            if slot.state == 'RESERVED':
                self._transition_motion_slot_locked(slot, 'ACCEPTED')
            if slot.state == 'ACCEPTED':
                self._transition_motion_slot_locked(slot, 'EXECUTING')
            slot.worker_started_event.set()

        try:
            result = self._execute_reserved_command(fields, slot)
        except Exception as error:
            self.motion.stop('gait command exception', log_level='warn')
            result = CommandResult(
                False,
                STATUS_FAILED,
                '{}: {}'.format(type(error).__name__, str(error)),
            )
        finally:
            if owns_json_slot:
                self._complete_motion_slot(slot)
        return result

    def _execute_reserved_command(self, fields, slot):
        command = str(fields.get('command', '')).strip().upper()
        if command in ('JUMP_START_OBSTACLE', 'JUMP_END_OBSTACLE'):
            motion_name = (
                'start_jump'
                if command == 'JUMP_START_OBSTACLE'
                else 'finish_jump'
            )
            outcome = self._run_front_jump(
                motion_name,
                slot=slot,
                goal_handle=slot.goal_handle,
            )
            result = CommandResult(
                outcome.success,
                self._front_jump_outcome_status(outcome),
                outcome.message(),
            )
            if slot.entry_type == 'json':
                self.publish_status(result.status, result.message)
            return result

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
            self._emergency_stop = False
            self._status = STATUS_RUNNING
            self._current_mode = command

        lock_result = self.publish_lock(True)
        if not lock_result.publish_succeeded:
            result = CommandResult(
                False,
                STATUS_FAILED,
                'gait lock true publish failed: {}'.format(
                    lock_result.error_message
                ),
            )
            self.publish_status(result.status, result.message)
            return result
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
            elif command in ('PRACTICAL_OBSTACLE_ZONE', 'AVOID_ZONE'):
                result = self.execute_practical_obstacle_zone()
            elif command == 'OBSTACLE_OPEN_LOOP_TEST':
                result = self.execute_obstacle_open_loop_test()
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
                self._emergency_stop = False

        self.publish_status(result.status, result.message)
        self.publish_mode('IDLE')
        unlock_result = self.publish_lock(False)
        if not unlock_result.publish_succeeded:
            return CommandResult(
                False,
                STATUS_FAILED,
                '{}; gait lock false publish failed: {}'.format(
                    result.message,
                    unlock_result.error_message,
                ),
            )
        return result

    def _execute_front_jump_recovery(self, fields):
        requested_fault_id = fields.get('cleanup_fault_id')
        confirmation = fields.get('confirm_no_front_jump_helper')
        if (
            not isinstance(requested_fault_id, str)
            or not requested_fault_id
            or not isinstance(confirmation, bool)
            or confirmation is not True
        ):
            return CommandResult(
                False,
                STATUS_FAILED,
                'recovery requires matching cleanup_fault_id and '
                'confirm_no_front_jump_helper=true',
            )

        with self._state_lock:
            faulted_slot = self._motion_slot
            recovery_stop_sequence = self._stop_sequence
            faulted_cleanup_slot = bool(
                faulted_slot is not None
                and faulted_slot.state == 'FAULTED'
                and faulted_slot.fault_type == 'cleanup_fault'
                and faulted_slot.worker_done_event.is_set()
                and faulted_slot.completion_event.is_set()
            )
            if faulted_slot is not None and not faulted_cleanup_slot:
                return CommandResult(
                    False,
                    STATUS_FAILED,
                    'cannot recover while an active or non-cleanup '
                    'motion slot exists',
                )
            if self._active_action_worker_count:
                return CommandResult(
                    False,
                    STATUS_FAILED,
                    'cannot recover while an Action worker is active',
                )
            blocking_faults = {
                name
                for name in self._safety_faults
                if name
                in {
                    'lock_publish_fault',
                    'action_reservation_fault',
                    'action_terminal_delivery_fault',
                    'fatal_shutdown_fault',
                    'cleanup_guard_fault',
                }
            }
            if blocking_faults:
                return CommandResult(
                    False,
                    STATUS_FAILED,
                    'recovery blocked by {}'.format(
                        ','.join(sorted(blocking_faults))
                    ),
                )

        try:
            record = self._cleanup_guard.load()
        except Exception as error:
            return CommandResult(False, STATUS_FAILED, str(error))
        if record is None:
            return CommandResult(
                False,
                STATUS_FAILED,
                'no cleanup guard is present',
            )
        if record['cleanup_fault_id'] != requested_fault_id:
            return CommandResult(
                False,
                STATUS_FAILED,
                'cleanup fault ID mismatch',
            )
        blocking_guard_faults = {
            str(fault.get('fault_type', ''))
            for fault in record.get('faults', [])
            if str(fault.get('fault_type', ''))
            in {
                'lock_publish_fault',
                'action_reservation_fault',
                'action_terminal_delivery_fault',
                'fatal_shutdown_fault',
                'cleanup_guard_fault',
            }
        }
        if blocking_guard_faults:
            return CommandResult(
                False,
                STATUS_FAILED,
                'recovery blocked by persisted {}'.format(
                    ','.join(sorted(blocking_guard_faults))
                ),
            )
        if self.front_jump_supervisor.active_context is not None:
            return CommandResult(
                False,
                STATUS_FAILED,
                'FrontJump supervisor is active',
            )
        if self.front_jump_supervisor.active_process is not None:
            return CommandResult(
                False,
                STATUS_FAILED,
                'FrontJump process handle is active',
            )
        process_absent, process_reason = (
            FrontJumpSupervisor.guard_process_absent(record)
        )
        if not process_absent:
            return CommandResult(False, STATUS_FAILED, process_reason)

        baseline = self.front_jump_supervisor.begin_recovery_window()
        confirm_samples = max(
            profile.final_zero_confirm_samples
            for profile in self.front_jump_profiles.values()
        )
        epsilon = min(
            profile.final_zero_epsilon
            for profile in self.front_jump_profiles.values()
        )
        timeout = max(
            profile.final_zero_timeout
            for profile in self.front_jump_profiles.values()
        )
        deadline = time.monotonic() + timeout
        evidence_ready = False
        evidence_reason = 'recovery evidence timeout'
        while time.monotonic() < deadline:
            evidence_ready, evidence_reason = (
                self.front_jump_supervisor.recovery_evidence_ready(
                    baseline,
                    confirm_samples=confirm_samples,
                    epsilon=epsilon,
                )
            )
            if evidence_ready:
                break
            self.front_jump_supervisor.wait_for_update(
                min(0.05, max(0.0, deadline - time.monotonic()))
            )
        if not evidence_ready:
            return CommandResult(False, STATUS_FAILED, evidence_reason)

        evidence_ready, evidence_reason = (
            self.front_jump_supervisor.recovery_evidence_ready(
                baseline,
                confirm_samples=confirm_samples,
                epsilon=epsilon,
            )
        )
        if not evidence_ready:
            return CommandResult(False, STATUS_FAILED, evidence_reason)

        def mark_recovered_cleanup(guard_record):
            cleanup = guard_record['cleanup']
            cleanup['leader_reaped'] = True
            cleanup['group_empty'] = True

        try:
            self._cleanup_guard.update(mark_recovered_cleanup)
        except Exception as error:
            return CommandResult(False, STATUS_FAILED, str(error))

        evidence_ready, evidence_reason = (
            self.front_jump_supervisor.recovery_evidence_ready(
                baseline,
                confirm_samples=confirm_samples,
                epsilon=epsilon,
            )
        )
        if not evidence_ready:
            return CommandResult(False, STATUS_FAILED, evidence_reason)

        with self._state_lock:
            if (
                self._motion_slot is not faulted_slot
                or self._active_action_worker_count
                or self._shutdown_requested.is_set()
                or self._stop_sequence != recovery_stop_sequence
            ):
                return CommandResult(
                    False,
                    STATUS_FAILED,
                    'recovery runtime state changed before unlock',
                )
            blocking_faults = {
                name
                for name in self._safety_faults
                if name
                in {
                    'lock_publish_fault',
                    'action_reservation_fault',
                    'action_terminal_delivery_fault',
                    'fatal_shutdown_fault',
                    'cleanup_guard_fault',
                }
            }
            if blocking_faults:
                return CommandResult(
                    False,
                    STATUS_FAILED,
                    'recovery blocked before unlock by {}'.format(
                        ','.join(sorted(blocking_faults))
                    ),
                )

        unlock_result = self.publish_lock(False)
        if not unlock_result.publish_succeeded:
            return CommandResult(
                False,
                STATUS_FAILED,
                'recovery lock false publish failed',
            )

        try:
            def mark_release(guard_record):
                guard_record['lock'][
                    'lock_release_command_published'
                ] = True
                guard_record['lock']['generation'] = (
                    unlock_result.generation
                )
                guard_record['cleanup']['cleanup_completed'] = True

            self._cleanup_guard.update(mark_release)
            self._cleanup_guard.mark_clean_and_clear(requested_fault_id)
        except Exception as error:
            self.publish_lock(True)
            self._fault_current_motion_slot(
                'cleanup_guard_fault',
                str(error),
            )
            return CommandResult(False, STATUS_FAILED, str(error))

        with self._state_lock:
            self._safety_faults.pop('cleanup_fault', None)
            self._accept_new_motion = not self._safety_faults
            self._cleanup_guard_record = None
            if self._motion_slot is faulted_slot:
                faulted_slot.state = 'DONE'
                self._motion_slot = None
                self._action_active = False
                self._emergency_stop = False
        result = CommandResult(
            True,
            STATUS_DONE,
            'FrontJump cleanup fault explicitly recovered',
        )
        self.publish_status(result.status, result.message)
        self.publish_mode('IDLE')
        return result

    def execute_stop(self):
        with self._state_lock:
            self._stop_sequence += 1
            slot = self._motion_slot
            interrupted = slot is not None
            front_jump_interrupted = bool(
                interrupted
                and self._is_front_jump_motion_name(slot.motion_name)
            )
            if interrupted:
                slot.stop_event.set()
                if not slot.first_abort_reason:
                    slot.first_abort_reason = 'gait_stop_requested'
                if slot.state not in ('FINALIZING', 'FAULTED', 'DONE'):
                    self._transition_motion_slot_locked(slot, 'STOPPING')
            if not front_jump_interrupted and interrupted:
                self._emergency_stop = True
            self._status = (
                STATUS_EMERGENCY_STOP
                if interrupted
                else STATUS_RUNNING
            )
            self._current_mode = 'STOP'

        if front_jump_interrupted:
            self.front_jump_supervisor.wake()
            message = 'active FrontJump supervision interrupted by gait STOP'
            self.publish_mode('STOP')
            self.publish_status(STATUS_EMERGENCY_STOP, message)
            return CommandResult(
                True,
                STATUS_EMERGENCY_STOP,
                message,
            )

        lock_result = None
        if not interrupted:
            lock_result = self.publish_lock(True)
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
            if (
                lock_result is not None
                and lock_result.publish_succeeded
                and not self._safety_faults
            ):
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

    def execute_obstacle_open_loop_test(self):
        start_time = time.monotonic()
        steps = [
            ObstacleStep('open_loop_forward_1', 0.30, 0.0, 1.5),
            ObstacleStep('open_loop_turn_left', 0.0, 0.80, 1.5),
            ObstacleStep('open_loop_forward_2', 0.30, 0.0, 1.0),
            ObstacleStep('open_loop_turn_right', 0.0, -0.80, 1.5),
        ]

        self.publish_action_feedback('open loop pre-stop', 0.02)
        self.zero_velocity('open loop obstacle test pre-stop')

        total_steps = len(steps)
        for index, step in enumerate(steps):
            self.publish_debug(
                'open loop obstacle test step '
                f'{index + 1}/{total_steps}: {step.name} '
                f'vx={step.vx:.3f}, wz={step.wz:.3f}, '
                f'duration={step.duration_sec:.2f}s'
            )
            check = self._run_open_loop_obstacle_step(
                step,
                start_time,
                index,
                total_steps
            )
            self.zero_velocity(f'open loop step {step.name} stop')
            if check is not None:
                return check

        return CommandResult(
            True,
            STATUS_DONE,
            'OBSTACLE_OPEN_LOOP_TEST completed'
        )

    def _run_open_loop_obstacle_step(
        self,
        step,
        start_time,
        step_index,
        total_steps
    ):
        end_time = time.monotonic() + float(step.duration_sec)
        period = 1.0 / self.publish_rate_hz

        while time.monotonic() < end_time:
            check = self._pre_motion_check(start_time)
            if check is not None:
                return check
            self.send_velocity(step.vx, 0.0, step.wz)
            elapsed = max(
                0.0,
                step.duration_sec - (end_time - time.monotonic())
            )
            step_fraction = elapsed / max(step.duration_sec, 0.001)
            progress = (
                step_index + step_fraction
            ) / float(max(1, total_steps))
            self.publish_action_feedback(
                f'open_loop: {step.name}',
                progress
            )
            time.sleep(period)

        return None

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
        result = self._control_lock_publisher.set_locked(locked)
        if not result.publish_succeeded:
            self._fault_current_motion_slot(
                'lock_publish_fault',
                'requested_state={} generation={} error={}'.format(
                    str(result.requested_state).lower(),
                    result.generation,
                    result.error_message,
                ),
            )
        return result

    def _update_control_lock_state_locked(self, locked, faulted):
        """Update mirrors while the caller already owns ``_state_lock``."""

        self._control_locked = bool(locked)
        self._lock_publish_fault = bool(faulted)

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
            slot = self._motion_slot
            return bool(slot is not None and slot.cancel_event.is_set())

    def _publish_front_jump_zero(self):
        self.motion.move(0.0, 0.0, 0.0)

    def _log_front_jump_event(self, record):
        self.publish_debug(
            'front_jump_supervision {}'.format(
                json.dumps(record, sort_keys=True, ensure_ascii=True)
            )
        )

    def _on_front_jump_final_cmd(self, msg):
        self.front_jump_supervisor.update_final_command(
            msg.linear.x,
            msg.linear.y,
            msg.angular.z,
            receive_time=time.monotonic(),
        )
        self._try_complete_full_boot_recovery()

    def _on_front_jump_estop_state(self, msg):
        self.front_jump_supervisor.update_estop(
            msg.data,
            receive_time=time.monotonic(),
        )
        self._try_complete_full_boot_recovery()

    def _try_complete_full_boot_recovery(self):
        """Clear a prior-boot guard only after live startup evidence."""

        with self._state_lock:
            guard_id = self._full_boot_recovery_guard_id
            baseline = self._full_boot_recovery_baseline
            if (
                not guard_id
                or baseline is None
                or self._full_boot_recovery_in_progress
                or self._motion_slot is not None
                or self._active_action_worker_count
                or self._shutdown_requested.is_set()
                or self._stop_sequence
                != self._full_boot_recovery_stop_sequence
            ):
                return False
            new_faults = (
                set(self._safety_faults)
                - self._full_boot_recovery_fault_types
            )
            if new_faults:
                return False

        confirm_samples = max(
            profile.final_zero_confirm_samples
            for profile in self.front_jump_profiles.values()
        )
        epsilon = min(
            profile.final_zero_epsilon
            for profile in self.front_jump_profiles.values()
        )
        evidence_ready, _ = (
            self.front_jump_supervisor.recovery_evidence_ready(
                baseline,
                confirm_samples=confirm_samples,
                epsilon=epsilon,
            )
        )
        if not evidence_ready:
            return False

        with self._state_lock:
            if (
                self._full_boot_recovery_in_progress
                or self._motion_slot is not None
                or self._active_action_worker_count
                or guard_id != self._full_boot_recovery_guard_id
            ):
                return False
            self._full_boot_recovery_in_progress = True

        try:
            record = self._cleanup_guard.load()
            if (
                record is None
                or record['cleanup_fault_id'] != guard_id
                or record['boot_id']
                == self._cleanup_guard.current_boot_id
            ):
                raise CleanupGuardError(
                    'full-boot recovery guard identity changed'
                )
            current_fault_ids = {
                str(fault.get('fault_id', ''))
                for fault in record.get('faults', [])
            }
            if current_fault_ids != self._full_boot_recovery_fault_ids:
                raise CleanupGuardError(
                    'new fault appeared during full-boot startup checks'
                )
            process_absent, reason = (
                FrontJumpSupervisor.guard_process_absent(record)
            )
            if not process_absent:
                raise CleanupGuardError(reason)
            if (
                self.front_jump_supervisor.active_context is not None
                or self.front_jump_supervisor.active_process is not None
            ):
                raise CleanupGuardError(
                    'FrontJump supervisor is active during startup check'
                )

            evidence_ready, reason = (
                self.front_jump_supervisor.recovery_evidence_ready(
                    baseline,
                    confirm_samples=confirm_samples,
                    epsilon=epsilon,
                )
            )
            if not evidence_ready:
                raise CleanupGuardError(reason)

            def mark_process_absent(guard_record):
                guard_record['cleanup']['leader_reaped'] = True
                guard_record['cleanup']['group_empty'] = True

            self._cleanup_guard.update(mark_process_absent)
            evidence_ready, reason = (
                self.front_jump_supervisor.recovery_evidence_ready(
                    baseline,
                    confirm_samples=confirm_samples,
                    epsilon=epsilon,
                )
            )
            if not evidence_ready:
                raise CleanupGuardError(reason)
            with self._state_lock:
                if (
                    self._motion_slot is not None
                    or self._active_action_worker_count
                    or self._shutdown_requested.is_set()
                    or self._stop_sequence
                    != self._full_boot_recovery_stop_sequence
                ):
                    raise CleanupGuardError(
                        'startup runtime state changed before unlock'
                    )
                new_faults = (
                    set(self._safety_faults)
                    - self._full_boot_recovery_fault_types
                )
                if new_faults:
                    raise CleanupGuardError(
                        'new safety fault appeared before unlock'
                    )
            unlock_result = self.publish_lock(False)
            if not unlock_result.publish_succeeded:
                raise CleanupGuardError(
                    'full-boot lock false publish failed'
                )

            def mark_released(guard_record):
                guard_record['lock'][
                    'lock_release_command_published'
                ] = True
                guard_record['lock']['generation'] = (
                    unlock_result.generation
                )
                guard_record['cleanup']['cleanup_completed'] = True

            self._cleanup_guard.update(mark_released)
            self._cleanup_guard.mark_clean_and_clear(guard_id)
        except Exception as error:
            if not self._control_lock_publisher.desired_state:
                self.publish_lock(True)
            with self._state_lock:
                self._full_boot_recovery_guard_id = ''
                self._full_boot_recovery_baseline = None
            self._fault_current_motion_slot(
                'cleanup_guard_fault',
                'full-boot startup recovery failed: {}: {}'.format(
                    type(error).__name__,
                    str(error),
                ),
            )
            return False
        finally:
            with self._state_lock:
                self._full_boot_recovery_in_progress = False

        with self._state_lock:
            for fault_type in self._full_boot_recovery_fault_types:
                self._safety_faults.pop(fault_type, None)
            self._full_boot_recovery_guard_id = ''
            self._full_boot_recovery_baseline = None
            self._full_boot_recovery_fault_types = set()
            self._full_boot_recovery_fault_ids = set()
            self._full_boot_recovery_stop_sequence = self._stop_sequence
            self._cleanup_guard_record = None
            self._accept_new_motion = not self._safety_faults
        self.publish_status(
            STATUS_IDLE,
            'prior-boot FrontJump safety guard passed startup checks',
        )
        self.publish_mode('IDLE')
        return True

    def _on_front_jump_cmd_mux_status(self, msg):
        self.front_jump_supervisor.update_mux_status(
            msg.data,
            receive_time=time.monotonic(),
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

        status_msg = String()
        status_msg.data = status
        self.status_pub.publish(status_msg)

        mode_msg = String()
        mode_msg.data = mode
        self.mode_pub.publish(mode_msg)
        result = self._control_lock_publisher.republish()
        if not result.publish_succeeded:
            self._fault_current_motion_slot(
                'lock_publish_fault',
                'periodic requested_state={} generation={} error={}'.format(
                    str(result.requested_state).lower(),
                    result.generation,
                    result.error_message,
                ),
            )

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
            raise ValueError(
                f'{name}: step_front_target_m must be nonnegative'
            )

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

    def _front_jump_string_parameter(self, name):
        value = self.get_parameter(name).value
        if not isinstance(value, str) or not value.strip():
            raise ValueError(f'{name} must be a non-empty string')
        return value.strip()

    def _front_jump_positive_float_parameter(self, name):
        value = self.get_parameter(name).value
        if (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
        ):
            raise ValueError(f'{name} must be a finite positive number')
        number = float(value)
        if not math.isfinite(number) or number <= 0.0:
            raise ValueError(f'{name} must be a finite positive number')
        return number

    def _front_jump_nonnegative_float_parameter(self, name):
        value = self.get_parameter(name).value
        if (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
        ):
            raise ValueError(f'{name} must be a finite nonnegative number')
        number = float(value)
        if not math.isfinite(number) or number < 0.0:
            raise ValueError(f'{name} must be a finite nonnegative number')
        return number

    def _front_jump_positive_int_parameter(self, name):
        value = self.get_parameter(name).value
        if (
            isinstance(value, bool)
            or not isinstance(value, int)
            or value <= 0
        ):
            raise ValueError(f'{name} must be a positive integer')
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

    def request_shutdown(self):
        """Request local shutdown without waiting or using ROS APIs."""

        if self._shutdown_requested.is_set():
            return
        with self._state_lock:
            if self._shutdown_requested.is_set():
                return
            self._accept_new_motion = False
            self._shutdown_requested.set()
            slot = self._motion_slot
            if slot is not None:
                slot.stop_event.set()
                if not slot.first_abort_reason:
                    slot.first_abort_reason = 'node_shutdown'
                if slot.state not in ('FINALIZING', 'FAULTED', 'DONE'):
                    self._transition_motion_slot_locked(slot, 'STOPPING')
                is_front_jump = self._is_front_jump_motion_name(
                    slot.motion_name
                )
                if not is_front_jump:
                    self._emergency_stop = True
            else:
                is_front_jump = False
        if is_front_jump:
            self.front_jump_supervisor.request_stop('node_shutdown')
        self.front_jump_supervisor.wake()

    def request_shutdown_from_context(self):
        """Non-blocking Context callback with no Context/ROS access."""

        self._ros_cleanup_allowed.clear()
        self.request_shutdown()

    def drain_shutdown_step(self, *, allow_ros):
        """Perform a bounded cleanup retry outside the on-shutdown callback."""

        if self.front_jump_supervisor.active_context is not None:
            return
        if self.front_jump_supervisor.cleanup_pending:
            self.front_jump_supervisor.retry_cleanup(
                allow_ros=bool(allow_ros)
            )

    def shutdown_drained(self):
        with self._state_lock:
            slot = self._motion_slot
            slot_drained = bool(
                slot is None
                or (
                    slot.worker_done_event.is_set()
                    and slot.completion_event.is_set()
                )
            )
            action_workers_drained = (
                self._active_action_worker_count == 0
                and self._action_workers_done_event.is_set()
            )
        supervisor_drained = (
            self.front_jump_supervisor.active_context is None
            and self.front_jump_supervisor.active_process is None
            and not self.front_jump_supervisor.cleanup_pending
        )
        return bool(
            slot_drained
            and action_workers_drained
            and supervisor_drained
        )

    def wait_for_shutdown_progress(self, timeout):
        with self._state_lock:
            slot = self._motion_slot
            event = (
                self._action_workers_done_event
                if slot is None
                else slot.worker_done_event
            )
        if event is None:
            self.front_jump_supervisor.completion_event.wait(
                timeout=max(0.0, float(timeout))
            )
        else:
            event.wait(timeout=max(0.0, float(timeout)))

    def prepare_finalize_shutdown(self, *, context_valid):
        """Freeze a drained node fail-closed before executor shutdown.

        This stage never publishes lock=false.  The executor result is not yet
        known, so releasing the gait lock here would create an unsafe transient
        unlock if callbacks later fail to drain.
        """

        if not self.shutdown_drained():
            return False
        with self._state_lock:
            if self._shutdown_prepared:
                return self._shutdown_prepare_clean
            self._shutdown_prepared = True
            faulted = bool(self._safety_faults)
        if not context_valid:
            self._shutdown_prepare_clean = False
            return False
        try:
            lock_result = self.publish_lock(True)
            if not lock_result.publish_succeeded:
                self._shutdown_prepare_clean = False
                return False
            self.motion.stop('gait_control_node shutdown prepare')
            with self._state_lock:
                self._shutdown_prepare_clean = not bool(
                    self._safety_faults
                ) and not faulted
            return self._shutdown_prepare_clean
        except Exception as error:
            self._fault_current_motion_slot(
                'fatal_shutdown_fault',
                '{}: {}'.format(type(error).__name__, str(error)),
            )
            self._shutdown_prepare_clean = False
            return False

    def commit_finalize_shutdown(
        self,
        *,
        executor_shutdown_succeeded,
        context_valid,
        failure_reason='',
    ):
        """Commit normal unlock only after a successful executor shutdown."""

        with self._state_lock:
            if self._shutdown_commit_called:
                return False
            self._shutdown_commit_called = True
            prepared_clean = self._shutdown_prepare_clean

        if (
            not executor_shutdown_succeeded
            or not prepared_clean
            or not context_valid
            or bool(self._safety_faults)
        ):
            reason = str(failure_reason) or (
                'executor shutdown failed or shutdown preparation was not '
                'clean'
            )
            self.record_executor_shutdown_fault(
                reason,
                allow_ros=bool(context_valid),
            )
            return False

        try:
            unlock_result = self.publish_lock(False)
            if not unlock_result.publish_succeeded:
                self._fault_current_motion_slot(
                    'fatal_shutdown_fault',
                    'shutdown lock false publish failed: {}'.format(
                        unlock_result.error_message
                    ),
                )
                return False
            self.publish_mode('IDLE')
            self.publish_status(STATUS_IDLE, 'gait control shutdown')
        except Exception as error:
            self._fault_current_motion_slot(
                'fatal_shutdown_fault',
                '{}: {}'.format(type(error).__name__, str(error)),
            )
            return False
        return not bool(self._safety_faults)

    def record_executor_shutdown_fault(self, reason, *, allow_ros=True):
        self._fatal_shutdown_fault = True
        self._fault_current_motion_slot(
            'fatal_shutdown_fault',
            reason,
        )
        if allow_ros:
            result = self.publish_lock(True)
            if result.publish_succeeded:
                try:
                    def mark_lock(guard_record):
                        guard_record['lock'][
                            'lock_acquire_command_published'
                        ] = True
                        guard_record['lock']['generation'] = (
                            result.generation
                        )

                    self._cleanup_guard.update(mark_lock)
                except Exception as error:
                    self._fault_current_motion_slot(
                        'cleanup_guard_fault',
                        '{}: {}'.format(
                            type(error).__name__,
                            str(error),
                        ),
                    )


def _shutdown_executor_then_commit(
    node,
    executor,
    *,
    prepared,
    context_valid,
):
    """Order executor shutdown before the only possible normal unlock.

    Keeping this narrow coordinator separate makes the fail-closed ordering
    directly testable: ``commit_finalize_shutdown`` never sees a successful
    executor result until ``Executor.shutdown`` has actually returned true.
    """

    executor_shutdown_reason = ''
    try:
        executor_shutdown_ok = bool(executor.shutdown(timeout_sec=2.0))
    except Exception as error:
        executor_shutdown_ok = False
        executor_shutdown_reason = '{}: {}'.format(
            type(error).__name__,
            str(error),
        )
    failure_reason = executor_shutdown_reason
    if not failure_reason:
        failure_reason = (
            'executor shutdown timeout with outstanding callbacks'
            if not executor_shutdown_ok
            else 'shutdown preparation did not establish a clean terminal '
            'state'
        )
    finalized = node.commit_finalize_shutdown(
        executor_shutdown_succeeded=executor_shutdown_ok and bool(prepared),
        context_valid=bool(context_valid),
        failure_reason=failure_reason,
    )
    return executor_shutdown_ok, bool(finalized)


def main(args=None):
    context = Context()
    rclpy.init(args=args, context=context)
    node = None
    executor = MultiThreadedExecutor(num_threads=4, context=context)
    shutdown_ok = True
    try:
        node = GaitControlNode(context=context)
        executor.add_node(node)
        context.on_shutdown(node.request_shutdown_from_context)
        while context.ok() and not node._shutdown_requested.is_set():
            executor.spin_once(timeout_sec=0.05)
    except (KeyboardInterrupt, ExternalShutdownException):
        if node is not None:
            node.request_shutdown()
    finally:
        try:
            if node is not None:
                node.request_shutdown()
                deadline = (
                    time.monotonic()
                    + node.front_jump_shutdown_drain_timeout_sec
                )
                while (
                    not node.shutdown_drained()
                    and time.monotonic() < deadline
                ):
                    if context.ok():
                        executor.spin_once(
                            timeout_sec=min(
                                0.05,
                                max(
                                    0.0,
                                    deadline - time.monotonic(),
                                ),
                            )
                        )
                    else:
                        node.wait_for_shutdown_progress(0.05)
                    node.drain_shutdown_step(allow_ros=context.ok())
                prepared = node.prepare_finalize_shutdown(
                    context_valid=context.ok()
                )
                executor_shutdown_ok, finalized = (
                    _shutdown_executor_then_commit(
                        node,
                        executor,
                        prepared=prepared,
                        context_valid=context.ok(),
                    )
                )
                shutdown_ok = bool(executor_shutdown_ok and finalized)
                try:
                    executor.remove_node(node)
                except Exception as error:
                    shutdown_ok = False
                    node.record_executor_shutdown_fault(
                        'executor remove_node failed: {}: {}'.format(
                            type(error).__name__,
                            str(error),
                        ),
                        allow_ros=context.ok(),
                    )
                finally:
                    node.destroy_node()
        finally:
            context.try_shutdown()
    if not shutdown_ok:
        raise RuntimeError(
            'gait_control_node shutdown did not complete cleanly'
        )


if __name__ == '__main__':
    main()
