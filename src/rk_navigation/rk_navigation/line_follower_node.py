#!/usr/bin/env python3

import json
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, String

from rk_interfaces.msg import LineTrack
from rk_navigation.start_ready_core import StartReadyGate


WAIT_START = 'WAIT_START'
START_READY = 'START_READY'
LINE_FOLLOW = 'LINE_FOLLOW'
SHORT_LOST = 'SHORT_LOST'
TURN_LOST_KEEP = 'TURN_LOST_KEEP'
TURN_90 = 'TURN_90'
SEARCH_LINE = 'SEARCH_LINE'
STOP = 'STOP'

RUNNING_STATES = {
    LINE_FOLLOW,
    SHORT_LOST,
    TURN_LOST_KEEP,
    TURN_90,
    SEARCH_LINE,
}
VALID_STATES = {
    WAIT_START,
    START_READY,
    LINE_FOLLOW,
    SHORT_LOST,
    TURN_LOST_KEEP,
    TURN_90,
    SEARCH_LINE,
    STOP,
}


class LineFollowerNode(Node):
    """Competition line navigation state machine."""

    def __init__(self, parameter_overrides=None):
        """创建巡线节点，允许 launch 或测试覆盖参数。"""
        super().__init__(
            'line_follower_node', parameter_overrides=parameter_overrides
        )

        self.declare_parameter(
            'suggested_cmd_topic',
            '/navigation/line_follow_cmd_suggested'
        )
        self.declare_parameter(
            'line_follow_status_topic',
            '/navigation/line_follow_status'
        )
        self.declare_parameter('line_track_topic', '/perception/line_track')
        self.declare_parameter('mission_start_topic', '/mission/start')
        self.declare_parameter('mission_stop_topic', '/mission/stop')
        self.declare_parameter('gait_control_lock_topic', '/gait/control_lock')
        self.declare_parameter('control_rate_hz', 10.0)
        self.declare_parameter('debug_log', True)

        # 巡线速度参数优先改 YAML；丢线找线速度允许为 0。
        self.declare_parameter('min_driving_speed', 0.20)
        self.declare_parameter('base_speed', 0.25)
        self.declare_parameter('mid_speed', 0.22)
        self.declare_parameter('slow_speed', 0.20)

        # 巡线转向参数：摆动大就降低 kp/max_angular_z，转弯慢就适当提高。
        self.declare_parameter('error_slow_threshold', 0.08)
        self.declare_parameter('error_slowest_threshold', 0.20)
        self.declare_parameter('kp_lateral', 2.2)
        self.declare_parameter('kp_heading', 1.8)
        self.declare_parameter('max_angular_z', 1.3)
        self.declare_parameter('angular_deadband', 0.0)
        self.declare_parameter('angular_smoothing_alpha', 1.0)
        self.declare_parameter('line_follow_min_confidence', 0.0)

        # 短暂丢线恢复：用于画面偶发断线或黑线短时间被遮挡。
        self.declare_parameter('short_lost_timeout', 0.6)
        self.declare_parameter('short_lost_linear_speed', 0.0)
        self.declare_parameter('search_angular_speed', 0.35)

        # 转弯/大角度丢线恢复：默认原地转向找线。
        self.declare_parameter('turn_90_duration', 0.8)
        self.declare_parameter('turn_90_angular_speed', 0.50)
        self.declare_parameter('turn_direction_mode', 'last_error')
        self.declare_parameter('default_turn_direction', 1)
        self.declare_parameter('turn_direction_deadband', 0.02)
        self.declare_parameter('turn_lost_keep_time', 0.8)
        self.declare_parameter('lost_turn_linear_speed', 0.0)
        self.declare_parameter('lost_turn_angular_speed', 0.35)
        self.declare_parameter('turn_lost_min_angular_z', 0.08)

        # 持续找线参数：continuous_search_enabled=true 时不会只巡一次就停。
        self.declare_parameter('search_linear_speed', 0.0)
        self.declare_parameter('search_line_angular_speed', 0.30)
        self.declare_parameter('search_sweep_period', 1.0)
        self.declare_parameter('search_timeout', 5.0)
        self.declare_parameter('line_reacquire_count', 5)
        self.declare_parameter('reacquire_confirm_frames', 4)
        self.declare_parameter('reacquire_min_confidence', 0.45)
        self.declare_parameter('reacquire_max_lateral_error', 0.75)
        self.declare_parameter('line_msg_timeout', 0.5)
        self.declare_parameter('continuous_search_enabled', True)

        # 启动稳定门：收到 start 后先确认当前画面可安全巡线，禁止盲走。
        self.declare_parameter('start_ready_confirm_frames', 5)
        self.declare_parameter('start_ready_timeout_sec', 5.0)
        self.declare_parameter('start_ready_min_confidence', 0.55)
        self.declare_parameter('start_ready_max_lateral_error', 0.80)
        self.declare_parameter('start_ready_max_heading_error', 0.80)

        self.suggested_cmd_topic = self.string_parameter(
            'suggested_cmd_topic'
        )
        self.line_follow_status_topic = self.string_parameter(
            'line_follow_status_topic'
        )
        self.line_track_topic = self.string_parameter('line_track_topic')
        self.mission_start_topic = self.string_parameter(
            'mission_start_topic'
        )
        self.mission_stop_topic = self.string_parameter('mission_stop_topic')
        self.gait_control_lock_topic = self.string_parameter(
            'gait_control_lock_topic'
        )

        self.state = WAIT_START
        self.state_enter_time = self.get_clock().now()
        self.last_line_msg = None
        self.last_line_msg_time = None
        self.last_seen_time = None
        self.last_seen_line_time = None
        self.last_lateral_error = 0.0
        self.last_heading_error = 0.0
        self.last_angular_z = 0.0
        self.last_turn_direction = 1
        self.expected_turn_direction = 1
        self.active_turn_direction = 1
        self.stable_seen_count = 0
        self.last_debug_log_ns = 0
        self.debug_log_period_ns = 1_000_000_000
        self.last_loss_reason = 'none'
        self.lost_line_count = 0
        self.gait_control_locked = False
        self.mission_started = False
        self.start_ready_failed = False
        self.last_published_cmd_is_zero = True
        self.last_stop_reason = 'startup'
        self.speed_floor_warning_keys = set()

        self.refresh_parameters()
        self.start_ready_gate = StartReadyGate(
            self.start_ready_confirm_frames,
            self.start_ready_min_confidence,
            self.start_ready_max_lateral_error,
            self.start_ready_max_heading_error,
        )

        self.publisher = self.create_publisher(
            Twist,
            self.suggested_cmd_topic,
            10
        )
        self.status_publisher = self.create_publisher(
            String,
            self.line_follow_status_topic,
            10
        )
        self.line_subscription = self.create_subscription(
            LineTrack,
            self.line_track_topic,
            self.on_line_track,
            10
        )
        self.start_subscription = self.create_subscription(
            Bool,
            self.mission_start_topic,
            self.on_mission_start,
            10
        )
        self.stop_subscription = self.create_subscription(
            Bool,
            self.mission_stop_topic,
            self.on_mission_stop,
            10
        )
        self.lock_subscription = self.create_subscription(
            Bool,
            self.gait_control_lock_topic,
            self.on_gait_control_lock,
            10
        )

        control_period = 1.0 / max(1.0, self.control_rate_hz)
        self.control_timer = self.create_timer(
            control_period,
            self.on_control_timer
        )

        self.get_logger().info(
            'Line follower state machine started: '
            f'line_track_topic={self.line_track_topic}, '
            f'suggested_cmd_topic={self.suggested_cmd_topic}, '
            f'status_topic={self.line_follow_status_topic}, '
            f'start_topic={self.mission_start_topic}, '
            f'stop_topic={self.mission_stop_topic}, '
            f'gait_control_lock_topic={self.gait_control_lock_topic}, '
            f'initial_state={self.state}, '
            f'control_rate_hz={self.control_rate_hz:.1f}'
        )
        self.get_logger().info(
            'Enter state WAIT_START: waiting for mission start'
        )

    def refresh_parameters(self):
        """Refresh runtime-tunable control parameters."""
        self.control_rate_hz = self.positive_float_parameter(
            'control_rate_hz'
        )
        self.debug_log = self.get_parameter(
            'debug_log'
        ).get_parameter_value().bool_value

        self.min_driving_speed = self.nonnegative_float_parameter(
            'min_driving_speed'
        )
        self.base_speed = self.driving_speed_parameter('base_speed')
        self.mid_speed = self.driving_speed_parameter('mid_speed')
        self.slow_speed = self.driving_speed_parameter('slow_speed')
        self.error_slow_threshold = self.nonnegative_float_parameter(
            'error_slow_threshold'
        )
        self.error_slowest_threshold = self.nonnegative_float_parameter(
            'error_slowest_threshold'
        )
        self.kp_lateral = self.finite_float_parameter('kp_lateral')
        self.kp_heading = self.finite_float_parameter('kp_heading')
        self.max_angular_z = self.nonnegative_float_parameter(
            'max_angular_z'
        )
        self.angular_deadband = self.nonnegative_float_parameter(
            'angular_deadband'
        )
        self.angular_smoothing_alpha = self.clamp(
            self.nonnegative_float_parameter('angular_smoothing_alpha'),
            0.0,
            1.0
        )
        self.line_follow_min_confidence = self.clamp(
            self.nonnegative_float_parameter('line_follow_min_confidence'),
            0.0,
            1.0
        )

        self.short_lost_timeout = self.nonnegative_float_parameter(
            'short_lost_timeout'
        )
        self.short_lost_linear_speed = self.optional_driving_speed_parameter(
            'short_lost_linear_speed'
        )
        self.search_angular_speed = self.nonnegative_float_parameter(
            'search_angular_speed'
        )

        self.turn_90_duration = self.nonnegative_float_parameter(
            'turn_90_duration'
        )
        self.turn_90_angular_speed = self.nonnegative_float_parameter(
            'turn_90_angular_speed'
        )
        self.turn_direction_mode = self.string_parameter(
            'turn_direction_mode'
        ).strip().lower()
        self.default_turn_direction = self.direction_from_value(
            self.finite_float_parameter('default_turn_direction')
        )
        self.turn_direction_deadband = self.nonnegative_float_parameter(
            'turn_direction_deadband'
        )
        self.turn_lost_keep_time = self.nonnegative_float_parameter(
            'turn_lost_keep_time'
        )
        self.lost_turn_linear_speed = self.optional_driving_speed_parameter(
            'lost_turn_linear_speed'
        )
        self.lost_turn_angular_speed = self.nonnegative_float_parameter(
            'lost_turn_angular_speed'
        )
        self.turn_lost_min_angular_z = self.nonnegative_float_parameter(
            'turn_lost_min_angular_z'
        )

        self.search_linear_speed = self.optional_driving_speed_parameter(
            'search_linear_speed'
        )
        self.search_line_angular_speed = self.nonnegative_float_parameter(
            'search_line_angular_speed'
        )
        self.search_sweep_period = self.nonnegative_float_parameter(
            'search_sweep_period'
        )
        self.search_timeout = self.nonnegative_float_parameter(
            'search_timeout'
        )
        self.line_reacquire_count = max(
            1,
            int(self.get_parameter(
                'line_reacquire_count'
            ).get_parameter_value().integer_value)
        )
        self.reacquire_confirm_frames = max(
            1,
            int(self.get_parameter(
                'reacquire_confirm_frames'
            ).get_parameter_value().integer_value)
        )
        self.reacquire_min_confidence = self.clamp(
            self.nonnegative_float_parameter('reacquire_min_confidence'),
            0.0,
            1.0
        )
        self.reacquire_max_lateral_error = self.nonnegative_float_parameter(
            'reacquire_max_lateral_error'
        )
        self.line_msg_timeout = self.nonnegative_float_parameter(
            'line_msg_timeout'
        )
        self.continuous_search_enabled = self.get_parameter(
            'continuous_search_enabled'
        ).get_parameter_value().bool_value

        start_ready_confirm_frames = self.get_parameter(
            'start_ready_confirm_frames'
        ).get_parameter_value().integer_value
        self.start_ready_confirm_frames = max(
            1,
            int(start_ready_confirm_frames),
        )
        self.start_ready_timeout_sec = self.positive_float_parameter(
            'start_ready_timeout_sec'
        )
        self.start_ready_min_confidence = self.clamp(
            self.nonnegative_float_parameter('start_ready_min_confidence'),
            0.0,
            1.0,
        )
        self.start_ready_max_lateral_error = self.nonnegative_float_parameter(
            'start_ready_max_lateral_error'
        )
        self.start_ready_max_heading_error = self.nonnegative_float_parameter(
            'start_ready_max_heading_error'
        )
        if hasattr(self, 'start_ready_gate'):
            self.start_ready_gate.configure(
                self.start_ready_confirm_frames,
                self.start_ready_min_confidence,
                self.start_ready_max_lateral_error,
                self.start_ready_max_heading_error,
            )

        if self.error_slowest_threshold < self.error_slow_threshold:
            self.error_slowest_threshold = self.error_slow_threshold

    def on_mission_start(self, msg):
        if not msg.data:
            return

        if self.mission_started:
            self.get_logger().warn(
                'Duplicate mission start ignored: '
                f'state={self.state}, mission remains active'
            )
            return

        now = self.get_clock().now()
        self.get_logger().info(
            'Mission start received: '
            f'state={self.state}, target_state={START_READY}'
        )
        self.stable_seen_count = 0
        self.start_ready_gate.reset()
        self.start_ready_failed = False
        self.last_angular_z = 0.0
        self.last_loss_reason = 'mission_start'
        self.last_stop_reason = 'none'
        self.mission_started = True
        self.set_state(
            START_READY,
            'mission_start_waiting_for_stable_line',
            now,
        )
        self.publish_zero('mission_start_start_ready', force=True)

    def on_mission_stop(self, msg):
        if not msg.data:
            return

        now = self.get_clock().now()
        self.get_logger().warn(
            'Mission stop received: '
            f'state={self.state}, target_state={STOP}'
        )
        self.last_loss_reason = 'mission_stop'
        self.last_angular_z = 0.0
        self.mission_started = False
        self.start_ready_failed = False
        self.start_ready_gate.reset()
        self.set_state(WAIT_START, 'mission_stop', now)
        self.publish_zero('mission_stop', force=True)

    def on_gait_control_lock(self, msg):
        locked = bool(msg.data)
        if locked != self.gait_control_locked:
            message = (
                'gait_control_lock=True; line follower cmd_vel output paused'
                if locked
                else 'gait_control_lock=False; line follower resumed'
            )
            self.get_logger().info(message)
        self.gait_control_locked = locked
        if locked:
            self.publish_zero('gait_control_lock')

    def on_line_track(self, msg):
        self.refresh_parameters()
        now = self.get_clock().now()
        self.last_line_msg = msg
        self.last_line_msg_time = now

        if not self.is_valid_line_msg(msg):
            self.lost_line_count += 1
            self.last_loss_reason = 'invalid_line_track'
            if self.state == START_READY:
                self.start_ready_failed = True
                self.set_state(
                    STOP,
                    'start_ready_invalid_line_track',
                    now,
                )
                self.publish_zero('start_ready_invalid_line_track', force=True)
                return
            if self.state in RUNNING_STATES:
                self.log_invalid_line_track_stop(msg, now)
                self.set_state(STOP, 'invalid_line_track', now)
                self.publish_zero('invalid_line_track')
            return

        if self.state == START_READY:
            self._update_start_ready(msg, now)
            return

        if self.is_trackable_line(msg):
            self.lost_line_count = 0
            self.last_seen_time = now
            self.last_seen_line_time = now
            self.last_lateral_error = float(msg.lateral_error)
            self.last_heading_error = float(msg.heading_error)
            self.last_turn_direction = self.direction_from_errors(msg)

            if self.state == SHORT_LOST:
                self.log_line_recovered('line_recovered', msg)
                self.set_state(LINE_FOLLOW, 'line_recovered', now)
            elif (
                self.state == STOP
                and self.mission_started
                and not self.start_ready_failed
            ):
                self.log_line_recovered('line_recovered_from_stop', msg)
                self.set_state(LINE_FOLLOW, 'line_recovered_from_stop', now)
        else:
            self.lost_line_count += 1
            if self.state == LINE_FOLLOW:
                self.enter_line_lost_state(msg, now)

        if self.state == SEARCH_LINE:
            self.update_reacquire_count(
                msg,
                now,
                self.line_reacquire_count,
                'line_reacquired'
            )
        elif self.state == TURN_LOST_KEEP:
            self.update_reacquire_count(
                msg,
                now,
                self.reacquire_confirm_frames,
                'turn_lost_reacquired'
            )

    def on_control_timer(self):
        self.refresh_parameters()
        now = self.get_clock().now()

        if self.gait_control_locked:
            self.log_control_debug(Twist(), 'gait_control_lock')
            return

        if self.state == WAIT_START:
            self.log_control_debug(Twist(), 'waiting_for_mission_start')
            return

        if self.state == START_READY:
            if self.elapsed_in_state(now) >= self.start_ready_timeout_sec:
                self.start_ready_failed = True
                self.last_loss_reason = 'start_ready_timeout'
                self.set_state(STOP, self.last_loss_reason, now)
                self.publish_zero(self.last_loss_reason, force=True)
            elif self.line_message_timed_out(now):
                self.start_ready_failed = True
                self.last_loss_reason = 'start_ready_line_msg_timeout'
                self.set_state(STOP, self.last_loss_reason, now)
                self.publish_zero(self.last_loss_reason, force=True)
            else:
                # 即使 mux 中留有旧候选，也在确认窗口持续送零候选。
                self.publish_zero(
                    'start_ready_waiting_for_stable_line',
                    force=True,
                )
            return

        if self.state == STOP:
            if (
                self.mission_started
                and not self.start_ready_failed
                and self.is_trackable_line(self.last_line_msg)
                and not self.line_message_timed_out(now)
            ):
                self.log_line_recovered(
                    'line_recovered_from_stop',
                    self.last_line_msg
                )
                self.set_state(LINE_FOLLOW, 'line_recovered_from_stop', now)
            else:
                self.log_control_debug(Twist(), self.last_stop_reason)
                return

        if self.state == STOP:
            return

        if self.state in RUNNING_STATES and self.line_message_timed_out(now):
            self.last_loss_reason = 'line_msg_timeout'
            self.get_logger().warn(
                'Line message timeout; entering STOP: '
                f'state={self.state}, '
                f'last_line_age={self.line_msg_age(now):.3f}s, '
                f'timeout={self.line_msg_timeout:.3f}s'
            )
            self.set_state(STOP, 'line_msg_timeout', now)
            self.publish_zero('line_msg_timeout')
            return

        if self.state == LINE_FOLLOW:
            cmd = self.command_line_follow(now)
        elif self.state == SHORT_LOST:
            cmd = self.command_short_lost(now)
        elif self.state == TURN_LOST_KEEP:
            cmd = self.command_turn_lost_keep(now)
        elif self.state == TURN_90:
            cmd = self.command_turn_90(now)
        elif self.state == SEARCH_LINE:
            cmd = self.command_search_line(now)
        else:
            self.set_state(STOP, f'unhandled_state_{self.state}', now)
            self.publish_zero(f'unhandled_state_{self.state}')
            return

        self.publish_cmd(cmd, self.last_loss_reason)

    def command_line_follow(self, now):
        msg = self.last_line_msg
        if msg is None:
            return Twist()

        if not self.is_valid_line_msg(msg):
            self.last_loss_reason = 'invalid_line_track'
            self.log_invalid_line_track_stop(msg, now)
            self.set_state(STOP, 'invalid_line_track', now)
            return Twist()

        if not self.is_trackable_line(msg):
            return self.enter_line_lost_state(msg, now)

        cmd = Twist()
        raw_angular = -(
            self.kp_lateral * float(msg.lateral_error) +
            self.kp_heading * float(msg.heading_error)
        )
        target_angular = self.clamp(
            raw_angular,
            -self.max_angular_z,
            self.max_angular_z
        )
        if abs(target_angular) < self.angular_deadband:
            target_angular = 0.0

        alpha = self.angular_smoothing_alpha
        cmd.angular.z = (
            alpha * target_angular
            + (1.0 - alpha) * float(self.last_angular_z)
        )
        if abs(cmd.angular.z) < self.angular_deadband:
            cmd.angular.z = 0.0

        error_abs = abs(float(msg.lateral_error))
        if error_abs < self.error_slow_threshold:
            cmd.linear.x = self.base_speed
        elif error_abs < self.error_slowest_threshold:
            cmd.linear.x = self.mid_speed
        else:
            cmd.linear.x = self.slow_speed

        self.last_angular_z = cmd.angular.z
        self.expected_turn_direction = self.direction_from_angular(
            self.last_angular_z
        )
        return cmd

    def enter_line_lost_state(self, msg, now):
        self.last_loss_reason = self.line_loss_reason(msg)
        if self.was_turning_before_loss():
            self.expected_turn_direction = self.direction_from_angular(
                self.last_angular_z
            )
            self.active_turn_direction = self.expected_turn_direction
            self.stable_seen_count = 0
            self.set_state(TURN_LOST_KEEP, self.last_loss_reason, now)
            return self.make_turn_lost_keep_cmd()

        self.set_state(SHORT_LOST, self.last_loss_reason, now)
        return self.make_short_lost_cmd()

    def command_short_lost(self, now):
        if self.is_trackable_line(self.last_line_msg):
            self.log_line_recovered('line_recovered', self.last_line_msg)
            self.set_state(LINE_FOLLOW, 'line_recovered', now)
            return self.command_line_follow(now)

        elapsed = self.elapsed_since_last_seen(now)
        if elapsed <= self.short_lost_timeout:
            return self.make_short_lost_cmd()

        self.active_turn_direction = self.resolve_turn_direction()
        self.get_logger().warn(
            'Short lost timeout: '
            f'elapsed={elapsed:.3f}s, '
            f'timeout={self.short_lost_timeout:.3f}s, '
            f'loss_reason={self.last_loss_reason}, '
            f'turn_direction={self.active_turn_direction}'
        )
        self.set_state(TURN_90, 'short_lost_timeout', now)
        return self.make_turn_90_cmd()

    def command_turn_90(self, now):
        elapsed = self.elapsed_in_state(now)
        if elapsed >= self.turn_90_duration:
            self.stable_seen_count = 0
            self.get_logger().info(
                'Turn 90 complete: '
                f'elapsed={elapsed:.3f}s, '
                f'duration={self.turn_90_duration:.3f}s, '
                f'turn_direction={self.active_turn_direction}'
            )
            self.set_state(SEARCH_LINE, 'turn_90_complete', now)
            return self.make_search_line_cmd()

        return self.make_turn_90_cmd()

    def command_turn_lost_keep(self, now):
        if self.stable_seen_count >= self.reacquire_confirm_frames:
            self.log_line_recovered(
                'turn_lost_reacquired',
                self.last_line_msg
            )
            self.set_state(LINE_FOLLOW, 'turn_lost_reacquired', now)
            return self.command_line_follow(now)

        elapsed = self.elapsed_in_state(now)
        if elapsed >= self.turn_lost_keep_time:
            self.get_logger().error(
                'Turn lost keep timeout: '
                f'elapsed={elapsed:.3f}s, '
                f'timeout={self.turn_lost_keep_time:.3f}s, '
                f'expected_turn_direction={self.expected_turn_direction}, '
                f'stable_seen_count={self.stable_seen_count}'
            )
            if self.continuous_search_enabled:
                self.stable_seen_count = 0
                self.set_state(SEARCH_LINE, 'turn_lost_keep_timeout', now)
                return self.make_search_line_cmd()
            self.set_state(STOP, 'turn_lost_keep_timeout', now)
            return Twist()

        return self.make_turn_lost_keep_cmd()

    def command_search_line(self, now):
        if self.stable_seen_count >= self.line_reacquire_count:
            self.log_line_recovered('line_reacquired', self.last_line_msg)
            self.set_state(LINE_FOLLOW, 'line_reacquired', now)
            return self.command_line_follow(now)

        elapsed = self.elapsed_in_state(now)
        if elapsed >= self.search_timeout:
            log_message = (
                'Search line timeout; continuing search: '
                if self.continuous_search_enabled
                else 'Search line timeout: '
            )
            self.get_logger().error(
                log_message +
                f'elapsed={elapsed:.3f}s, '
                f'timeout={self.search_timeout:.3f}s, '
                f'turn_direction={self.active_turn_direction}, '
                f'stable_seen_count={self.stable_seen_count}'
            )
            if self.continuous_search_enabled:
                self.state_enter_time = now
                self.stable_seen_count = 0
                return self.make_search_line_cmd()
            self.set_state(STOP, 'search_timeout', now)
            return Twist()

        return self.make_search_line_cmd()

    def update_reacquire_count(self, msg, now, required_count, recover_reason):
        if self.is_reacquired_line(msg):
            self.stable_seen_count += 1
            if self.stable_seen_count >= required_count:
                self.log_line_recovered(recover_reason, msg)
                self.set_state(LINE_FOLLOW, recover_reason, now)
        else:
            if self.stable_seen_count > 0:
                self.get_logger().info(
                    'Line reacquire streak reset: '
                    f'count={self.stable_seen_count}, '
                    f'line_visible={msg.line_visible}, '
                    f'confidence={msg.confidence:.3f}, '
                    f'lateral_error={msg.lateral_error:.3f}'
                )
            self.stable_seen_count = 0

    def was_turning_before_loss(self):
        return abs(float(self.last_angular_z)) >= self.turn_lost_min_angular_z

    def make_short_lost_cmd(self):
        cmd = Twist()
        cmd.linear.x = self.short_lost_linear_speed
        cmd.angular.z = self.last_turn_direction * self.search_angular_speed
        return cmd

    def make_turn_lost_keep_cmd(self):
        cmd = Twist()
        cmd.linear.x = self.lost_turn_linear_speed
        cmd.angular.z = (
            self.expected_turn_direction * self.lost_turn_angular_speed
        )
        return cmd

    def make_turn_90_cmd(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = (
            self.active_turn_direction * self.turn_90_angular_speed
        )
        return cmd

    def make_search_line_cmd(self):
        cmd = Twist()
        cmd.linear.x = self.search_linear_speed
        cmd.angular.z = (
            self.search_sweep_direction() * self.search_line_angular_speed
        )
        return cmd

    def search_sweep_direction(self):
        if self.search_sweep_period <= 0.0:
            return self.active_turn_direction or self.default_turn_direction

        elapsed = self.elapsed_in_state(self.get_clock().now())
        phase = int(elapsed / self.search_sweep_period)
        direction = self.active_turn_direction or self.default_turn_direction
        if phase % 2 == 1:
            direction *= -1
        return direction

    def publish_cmd(self, cmd, stop_reason='none'):
        if self.is_zero_cmd(cmd):
            self.publish_zero(stop_reason)
            return

        self.publisher.publish(cmd)
        self.last_published_cmd_is_zero = False
        self.last_stop_reason = 'none'
        self.log_control_debug(cmd, stop_reason)

    def publish_zero(self, reason='stop', force=False):
        cmd = Twist()
        self.last_stop_reason = reason
        self.last_angular_z = 0.0
        if self.last_published_cmd_is_zero and not force:
            self.log_control_debug(cmd, reason)
            return

        self.publisher.publish(cmd)
        self.last_published_cmd_is_zero = True
        self.log_control_debug(cmd, reason)

    def set_state(self, new_state, reason, now):
        requested_state = new_state
        if (
            not isinstance(requested_state, str)
            or requested_state not in VALID_STATES
        ):
            self.get_logger().error(
                'Invalid target state requested; entering STOP: '
                f'current_state={self.state}, '
                f'requested_state={requested_state}, reason={reason}'
            )
            new_state = STOP
            reason = f'invalid_target_state_{requested_state}'

        if new_state == self.state:
            return

        old_state = self.state
        old_elapsed = self.elapsed_in_state(now)
        self.get_logger().info(
            'Exit state '
            f'{old_state}: elapsed={old_elapsed:.3f}s, reason={reason}'
        )

        self.state = new_state
        self.state_enter_time = now

        if new_state in {LINE_FOLLOW, WAIT_START, START_READY, STOP}:
            self.stable_seen_count = 0
        if new_state in {WAIT_START, START_READY, STOP}:
            self.last_angular_z = 0.0
        if new_state == WAIT_START:
            self.get_logger().info(
                'Enter state WAIT_START: '
                f'reason={reason}, waiting for mission start'
            )
            return
        if new_state == START_READY:
            self.get_logger().info(
                'Enter state START_READY: '
                f'reason={reason}, '
                f'confirm_frames={self.start_ready_confirm_frames}, '
                f'min_confidence={self.start_ready_min_confidence:.3f}, '
                f'max_lateral_error={self.start_ready_max_lateral_error:.3f}, '
                f'max_heading_error={self.start_ready_max_heading_error:.3f}'
            )
            return
        if new_state == LINE_FOLLOW:
            self.get_logger().info(
                'Enter state LINE_FOLLOW: '
                f'reason={reason}, '
                f'last_line_age={self.line_msg_age(now):.3f}s, '
                f'{self.line_msg_summary(self.last_line_msg)}'
            )
            return
        if new_state == SHORT_LOST:
            self.get_logger().info(
                'Enter state SHORT_LOST: '
                f'reason={reason}, '
                f'loss_reason={self.last_loss_reason}, '
                f'short_lost_timeout={self.short_lost_timeout:.3f}s, '
                f'last_seen_age={self.last_seen_line_age(now):.3f}s, '
                f'last_line_age={self.line_msg_age(now):.3f}s, '
                f'{self.line_msg_summary(self.last_line_msg)}'
            )
            return
        if new_state == TURN_LOST_KEEP:
            self.get_logger().warn(
                'Enter state TURN_LOST_KEEP: '
                f'reason={reason}, '
                f'loss_reason={self.last_loss_reason}, '
                f'expected_turn_direction={self.expected_turn_direction}, '
                f'last_angular_z={self.last_angular_z:.3f}, '
                f'timeout={self.turn_lost_keep_time:.3f}s, '
                f'lost_turn_linear_speed={self.lost_turn_linear_speed:.3f}, '
                f'lost_turn_angular_speed={self.lost_turn_angular_speed:.3f}, '
                f'reacquire_confirm_frames={self.reacquire_confirm_frames}, '
                f'last_seen_age={self.last_seen_line_age(now):.3f}s, '
                f'{self.line_msg_summary(self.last_line_msg)}'
            )
            return
        if new_state == TURN_90:
            self.get_logger().info(
                'Enter state TURN_90: '
                f'reason={reason}, '
                f'turn_direction={self.active_turn_direction}, '
                f'duration={self.turn_90_duration:.3f}s, '
                f'mode={self.turn_direction_mode}, '
                f'last_lateral_error={self.last_lateral_error:.3f}, '
                f'last_heading_error={self.last_heading_error:.3f}'
            )
            return
        if new_state == SEARCH_LINE:
            self.get_logger().info(
                'Enter state SEARCH_LINE: '
                f'reason={reason}, '
                f'turn_direction={self.active_turn_direction}, '
                f'timeout={self.search_timeout:.3f}s, '
                f'sweep_period={self.search_sweep_period:.3f}s, '
                f'reacquire_count={self.line_reacquire_count}, '
                f'stable_seen_count={self.stable_seen_count}'
            )
            return
        if new_state == STOP:
            self.get_logger().warn(
                'Enter state STOP: '
                f'reason={reason}, '
                f'last_loss_reason={self.last_loss_reason}, '
                f'last_line_age={self.line_msg_age(now):.3f}s, '
                f'{self.line_msg_summary(self.last_line_msg)}'
            )

    def line_message_timed_out(self, now):
        if self.line_msg_timeout <= 0.0:
            return False

        if self.last_line_msg_time is None:
            return self.elapsed_in_state(now) > self.line_msg_timeout

        age = self.elapsed_since(now, self.last_line_msg_time)
        return age > self.line_msg_timeout

    def elapsed_since_last_seen(self, now):
        if self.last_seen_line_time is None:
            return self.elapsed_in_state(now)
        return self.elapsed_since(now, self.last_seen_line_time)

    def last_seen_line_age(self, now):
        if self.last_seen_line_time is None:
            return -1.0
        return self.elapsed_since(now, self.last_seen_line_time)

    def elapsed_in_state(self, now):
        return self.elapsed_since(now, self.state_enter_time)

    @staticmethod
    def elapsed_since(now, start_time):
        return (now.nanoseconds - start_time.nanoseconds) / 1_000_000_000.0

    def is_trackable_line(self, msg):
        if msg is None:
            return False
        return (
            bool(msg.line_visible)
            and float(msg.confidence) >= self.line_follow_min_confidence
        )

    def _update_start_ready(self, msg, now):
        """只在新鲜、有限且连续居中的线迹后解锁普通巡线。"""
        decision = self.start_ready_gate.observe(
            msg.line_visible,
            msg.confidence,
            msg.lateral_error,
            msg.heading_error,
        )
        self.stable_seen_count = decision.confirm_count
        self.last_loss_reason = decision.reason
        if decision.ready:
            self.last_seen_time = now
            self.last_seen_line_time = now
            self.last_lateral_error = float(msg.lateral_error)
            self.last_heading_error = float(msg.heading_error)
            self.last_turn_direction = self.direction_from_errors(msg)
            self.set_state(LINE_FOLLOW, 'start_ready_confirmed', now)
            return
        # 不合格帧会由核心复位计数，期间始终保持零速度。
        self.publish_zero(decision.reason, force=True)

    def is_reacquired_line(self, msg):
        if msg is None:
            return False
        return (
            bool(msg.line_visible)
            and float(msg.confidence) > self.reacquire_min_confidence
            and abs(float(msg.lateral_error))
            < self.reacquire_max_lateral_error
        )

    def line_loss_reason(self, msg):
        if msg is None:
            return 'no_line_track'
        if not self.is_valid_line_msg(msg):
            return 'invalid_line_track'
        if not msg.line_visible:
            return 'line_visible_false'
        if float(msg.confidence) < self.line_follow_min_confidence:
            return 'confidence_low'
        return 'unknown_line_loss'

    def log_invalid_line_track_stop(self, msg, now):
        self.get_logger().warn(
            'Invalid line track received; entering STOP: '
            f'state={self.state}, '
            f'last_line_age={self.line_msg_age(now):.3f}s, '
            f'{self.line_msg_summary(msg)}'
        )

    def log_line_recovered(self, reason, msg):
        required_count = self.line_reacquire_count
        if reason == 'turn_lost_reacquired' or self.state == TURN_LOST_KEEP:
            required_count = self.reacquire_confirm_frames
        self.get_logger().info(
            'Line recovered: '
            f'state={self.state}, '
            f'reason={reason}, '
            f'stable_seen_count={self.stable_seen_count}, '
            f'required_count={required_count}, '
            f'{self.line_msg_summary(msg)}'
        )

    def line_msg_summary(self, msg):
        if msg is None:
            return 'line_msg=None'

        return (
            f'line_visible={bool(msg.line_visible)}, '
            f'confidence={float(msg.confidence):.3f}, '
            f'lateral_error={float(msg.lateral_error):.3f}, '
            f'heading_error={float(msg.heading_error):.3f}'
        )

    def line_msg_age(self, now):
        if self.last_line_msg_time is None:
            return -1.0
        return self.elapsed_since(now, self.last_line_msg_time)

    def direction_from_errors(self, msg):
        correction = -(
            self.kp_lateral * float(msg.lateral_error) +
            self.kp_heading * float(msg.heading_error)
        )
        if correction > self.turn_direction_deadband:
            return 1
        if correction < -self.turn_direction_deadband:
            return -1
        return self.last_turn_direction or self.default_turn_direction

    def direction_from_angular(self, angular_z):
        angular_z = float(angular_z)
        if angular_z > 0.0:
            return 1
        if angular_z < 0.0:
            return -1
        return self.last_turn_direction or self.default_turn_direction

    def resolve_turn_direction(self):
        mode = self.turn_direction_mode
        if mode in {'left', 'fixed_left'}:
            return 1
        if mode in {'right', 'fixed_right'}:
            return -1
        if mode in {'default', 'default_turn_direction'}:
            return self.default_turn_direction
        if mode == 'last_error':
            return self.last_turn_direction or self.default_turn_direction

        self.get_logger().warn(
            'Unknown turn_direction_mode, using last_error: '
            f'{self.turn_direction_mode}'
        )
        return self.last_turn_direction or self.default_turn_direction

    def is_valid_line_msg(self, msg):
        if msg is None:
            return False

        values = [
            msg.lateral_error,
            msg.heading_error,
            msg.confidence,
            self.min_driving_speed,
            self.base_speed,
            self.mid_speed,
            self.slow_speed,
            self.kp_lateral,
            self.kp_heading,
            self.max_angular_z,
            self.angular_deadband,
            self.angular_smoothing_alpha,
            self.short_lost_timeout,
            self.turn_lost_keep_time,
            self.lost_turn_linear_speed,
            self.lost_turn_angular_speed,
            self.turn_lost_min_angular_z,
            self.turn_90_duration,
            self.turn_90_angular_speed,
            self.search_sweep_period,
            self.search_timeout,
            self.line_msg_timeout,
        ]
        return all(math.isfinite(float(value)) for value in values)

    @staticmethod
    def is_zero_cmd(cmd):
        return (
            float(cmd.linear.x) == 0.0
            and float(cmd.linear.y) == 0.0
            and float(cmd.linear.z) == 0.0
            and float(cmd.angular.x) == 0.0
            and float(cmd.angular.y) == 0.0
            and float(cmd.angular.z) == 0.0
        )

    def log_control_debug(self, cmd, stop_reason):
        """发布只读状态；shutdown 中 publisher 失效时不把清理竞态升级为异常。"""
        if not rclpy.ok():
            return
        line_visible = False
        lateral_error = 0.0
        heading_error = 0.0
        if self.last_line_msg is not None:
            line_visible = bool(self.last_line_msg.line_visible)
            lateral_error = float(self.last_line_msg.lateral_error)
            heading_error = float(self.last_line_msg.heading_error)

        status = String()
        status.data = json.dumps({
            'nav_state': self.state,
            'mission_started': bool(self.mission_started),
            'start_ready': self.state == START_READY,
            'ready': bool(
                self.mission_started and self.state == LINE_FOLLOW
            ),
            'start_ready_confirm_count': int(
                self.start_ready_gate.confirm_count
            ),
            'start_ready_confirm_frames': int(
                self.start_ready_confirm_frames
            ),
            'start_ready_failed': bool(self.start_ready_failed),
            'line_visible': line_visible,
            'confidence': (
                float(self.last_line_msg.confidence)
                if self.last_line_msg is not None else 0.0
            ),
            'lateral_error': lateral_error,
            'heading_error': heading_error,
            'lost_line_count': int(self.lost_line_count),
            'suggested_vx': float(cmd.linear.x),
            'suggested_wz': float(cmd.angular.z),
            'reason': str(stop_reason),
        }, separators=(',', ':'))
        try:
            self.status_publisher.publish(status)
        except Exception:
            # 运行态发布错误必须继续暴露，只有 ROS 已关闭才静默结束回调。
            if rclpy.ok():
                raise
            return

        self.log_debug(
            'control: '
            f'mission_started={self.mission_started}, '
            f'line_visible={line_visible}, '
            f'control_lock={self.gait_control_locked}, '
            f'lateral_error={lateral_error:.3f}, '
            f'heading_error={heading_error:.3f}, '
            f'suggested_cmd.linear.x={cmd.linear.x:.3f}, '
            f'suggested_cmd.angular.z={cmd.angular.z:.3f}, '
            f'stop_reason={stop_reason}, '
            f'last_loss_reason={self.last_loss_reason}, '
            f'stable_seen_count={self.stable_seen_count}'
        )

    def log_debug(self, message):
        if not self.debug_log:
            return

        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_debug_log_ns < self.debug_log_period_ns:
            return

        self.last_debug_log_ns = now_ns
        self.get_logger().info(
            'navigation debug: '
            f'state={self.state}, '
            f'last_line_age={self.current_line_age():.3f}, '
            f'last_angular_z={self.last_angular_z:.3f}, '
            f'last_turn_direction={self.last_turn_direction}, '
            f'expected_turn_direction={self.expected_turn_direction}, '
            f'active_turn_direction={self.active_turn_direction}, '
            f'{message}'
        )

    def current_line_age(self):
        return self.line_msg_age(self.get_clock().now())

    def string_parameter(self, name):
        value = str(self.get_parameter(name).value)
        if not value:
            raise ValueError(f'{name} must not be empty')
        return value

    def finite_float_parameter(self, name):
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value):
            raise ValueError(f'{name} must be finite')
        return value

    def nonnegative_float_parameter(self, name):
        value = self.finite_float_parameter(name)
        if value < 0.0:
            raise ValueError(f'{name} must be nonnegative')
        return value

    def driving_speed_parameter(self, name):
        return self.apply_min_driving_speed(
            name,
            self.nonnegative_float_parameter(name),
            allow_zero=False
        )

    def optional_driving_speed_parameter(self, name):
        return self.apply_min_driving_speed(
            name,
            self.nonnegative_float_parameter(name),
            allow_zero=True
        )

    def apply_min_driving_speed(self, name, value, allow_zero):
        if allow_zero and value == 0.0:
            return 0.0

        if value >= self.min_driving_speed:
            return value

        warning_key = (name, value, self.min_driving_speed)
        if warning_key not in self.speed_floor_warning_keys:
            self.speed_floor_warning_keys.add(warning_key)
            self.get_logger().warn(
                'Line speed below Go2 walking threshold; clamping: '
                f'{name}={value:.3f}, '
                f'min_driving_speed={self.min_driving_speed:.3f}'
            )

        return self.min_driving_speed

    def positive_float_parameter(self, name):
        value = self.finite_float_parameter(name)
        if value <= 0.0:
            raise ValueError(f'{name} must be positive')
        return value

    @staticmethod
    def direction_from_value(value):
        return 1 if float(value) >= 0.0 else -1

    @staticmethod
    def clamp(value, minimum, maximum):
        return max(minimum, min(maximum, value))


def main(args=None):
    rclpy.init(args=args)
    node = LineFollowerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
