#!/usr/bin/env python3

import json
import math
import threading
import time
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import Twist
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, String


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
        'RECOVERY_STAND',
        'HOLD_STABLE',
        'LOW_SPEED_MOVE',
        'TURN_IN_PLACE',
        'BODY_HEIGHT_ADJUST',
        'SPEED_LIMIT',
        'JUMP_START_OBSTACLE',
        'JUMP_END_OBSTACLE',
    }

    def __init__(self):
        super().__init__('gait_control_node')
        self.callback_group = ReentrantCallbackGroup()

        self._declare_parameters()
        self.cmd_vel_topic = self._string_parameter('cmd_vel_topic')
        self.command_json_topic = self._string_parameter('command_json_topic')

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
        self.publish_rate_hz = self._positive_float_parameter('publish_rate_hz')
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
            f'{self.command_json_topic}'
        )

    def _declare_parameters(self):
        parameters = {
            'cmd_vel_topic': '/navigation/cmd_vel',
            'status_topic': '/gait/status',
            'control_lock_topic': '/gait/control_lock',
            'debug_topic': '/gait/debug',
            'current_mode_topic': '/gait/current_mode',
            'command_json_topic': '/gait/command_json',
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

        if command == 'SPEED_LIMIT':
            try:
                return self.set_speed_limit(fields)
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
            self._status = STATUS_EMERGENCY_STOP if interrupted else STATUS_RUNNING
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
            self.motion.stop('unstable before recovery_stand', log_level='warn')

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
                'SPEED_LIMIT requires at least one positive max_vx/max_vy/max_wz'
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

    def _positive_int_parameter(self, name):
        value = int(self.get_parameter(name).value)
        if value <= 0:
            raise ValueError(f'{name} must be positive')
        return value

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
