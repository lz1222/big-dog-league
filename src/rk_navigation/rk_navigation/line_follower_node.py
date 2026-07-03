#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, String

from rk_interfaces.msg import LineTrack


WAIT_START = 'WAIT_START'
LINE_FOLLOW = 'LINE_FOLLOW'
SHORT_LOST = 'SHORT_LOST'
TURN_90 = 'TURN_90'
SEARCH_LINE = 'SEARCH_LINE'
STOP = 'STOP'

RUNNING_STATES = {LINE_FOLLOW, SHORT_LOST, TURN_90, SEARCH_LINE}
VALID_STATES = {
    WAIT_START,
    LINE_FOLLOW,
    SHORT_LOST,
    TURN_90,
    SEARCH_LINE,
    STOP,
}


class LineFollowerNode(Node):
    """Line navigation finite-state machine for competition segments."""

    def __init__(self):
        super().__init__('line_follower_node')

        self.declare_parameter('cmd_vel_topic', '/navigation/cmd_vel')
        self.declare_parameter('line_track_topic', '/perception/line_track')
        self.declare_parameter('mission_start_topic', '/mission/start')
        self.declare_parameter('mission_stop_topic', '/mission/stop')
        self.declare_parameter('control_lock_topic', '/gait/control_lock')
        self.declare_parameter(
            'state_topic',
            '/navigation/line_follower/state'
        )
        self.declare_parameter('control_rate_hz', 10.0)
        self.declare_parameter('debug_log', False)
        self.declare_parameter('auto_start', False)

        self.declare_parameter('base_speed', 0.18)
        self.declare_parameter('mid_speed', 0.12)
        self.declare_parameter('slow_speed', 0.08)
        self.declare_parameter('error_slow_threshold', 0.20)
        self.declare_parameter('error_slowest_threshold', 0.50)
        self.declare_parameter('kp_lateral', 1.2)
        self.declare_parameter('kp_heading', 0.8)
        self.declare_parameter('max_angular_z', 0.8)
        self.declare_parameter('line_follow_min_confidence', 0.0)

        self.declare_parameter('short_lost_timeout', 0.6)
        self.declare_parameter('short_lost_linear_speed', 0.06)
        self.declare_parameter('search_angular_speed', 0.25)

        self.declare_parameter('turn_90_duration', 1.6)
        self.declare_parameter('turn_90_angular_speed', 0.45)
        self.declare_parameter('turn_direction_mode', 'last_error')
        self.declare_parameter('default_turn_direction', 1)
        self.declare_parameter('turn_direction_deadband', 0.02)

        self.declare_parameter('search_linear_speed', 0.05)
        self.declare_parameter('search_line_angular_speed', 0.20)
        self.declare_parameter('search_timeout', 5.0)
        self.declare_parameter('line_reacquire_count', 5)
        self.declare_parameter('reacquire_min_confidence', 0.45)
        self.declare_parameter('reacquire_max_lateral_error', 0.75)
        self.declare_parameter('line_msg_timeout', 0.5)

        self.cmd_vel_topic = self.string_parameter('cmd_vel_topic')
        self.line_track_topic = self.string_parameter('line_track_topic')
        self.mission_start_topic = self.string_parameter(
            'mission_start_topic'
        )
        self.mission_stop_topic = self.string_parameter('mission_stop_topic')
        self.control_lock_topic = self.string_parameter('control_lock_topic')
        self.state_topic = self.string_parameter('state_topic')

        self.state = WAIT_START
        self.state_enter_time = self.get_clock().now()
        self.last_line_msg = None
        self.last_line_msg_time = None
        self.last_seen_line_time = None
        self.last_lateral_error = 0.0
        self.last_heading_error = 0.0
        self.last_turn_direction = 1
        self.active_turn_direction = 1
        self.stable_seen_count = 0
        self.last_debug_log_ns = 0
        self.debug_log_period_ns = 1_000_000_000
        self.last_loss_reason = 'none'
        self.gait_control_locked = False

        self.refresh_parameters()

        self.publisher = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.state_publisher = self.create_publisher(
            String,
            self.state_topic,
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
            self.control_lock_topic,
            self.on_gait_control_lock,
            10
        )

        control_period = 1.0 / max(1.0, self.control_rate_hz)
        self.control_timer = self.create_timer(
            control_period,
            self.on_control_timer
        )

        self.publish_state()
        self.get_logger().info(
            'Line follower state machine started: '
            f'line_track_topic={self.line_track_topic}, '
            f'cmd_vel_topic={self.cmd_vel_topic}, '
            f'start_topic={self.mission_start_topic}, '
            f'stop_topic={self.mission_stop_topic}, '
            f'lock_topic={self.control_lock_topic}, '
            f'state_topic={self.state_topic}, '
            f'initial_state={self.state}, '
            f'control_rate_hz={self.control_rate_hz:.1f}'
        )

        if self.bool_parameter('auto_start'):
            now = self.get_clock().now()
            self.set_state(LINE_FOLLOW, 'auto_start', now)
        else:
            self.get_logger().info(
                'Enter state WAIT_START: waiting for mission start'
            )

    def refresh_parameters(self):
        """Refresh runtime-tunable control parameters."""
        self.control_rate_hz = self.positive_float_parameter(
            'control_rate_hz'
        )
        self.debug_log = self.bool_parameter('debug_log')

        self.base_speed = self.nonnegative_float_parameter('base_speed')
        self.mid_speed = self.nonnegative_float_parameter('mid_speed')
        self.slow_speed = self.nonnegative_float_parameter('slow_speed')
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
        self.line_follow_min_confidence = self.clamp(
            self.nonnegative_float_parameter('line_follow_min_confidence'),
            0.0,
            1.0
        )

        self.short_lost_timeout = self.nonnegative_float_parameter(
            'short_lost_timeout'
        )
        self.short_lost_linear_speed = self.nonnegative_float_parameter(
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

        self.search_linear_speed = self.nonnegative_float_parameter(
            'search_linear_speed'
        )
        self.search_line_angular_speed = self.nonnegative_float_parameter(
            'search_line_angular_speed'
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

        if self.error_slowest_threshold < self.error_slow_threshold:
            self.error_slowest_threshold = self.error_slow_threshold

    def on_mission_start(self, msg):
        if not msg.data:
            return

        now = self.get_clock().now()
        self.get_logger().info(
            'Mission start received: '
            f'state={self.state}, target_state={LINE_FOLLOW}'
        )
        self.stable_seen_count = 0
        self.last_loss_reason = 'mission_start'
        self.set_state(LINE_FOLLOW, 'mission_start', now)

    def on_mission_stop(self, msg):
        if not msg.data:
            return

        now = self.get_clock().now()
        self.get_logger().warn(
            'Mission stop received: '
            f'state={self.state}, target_state={STOP}'
        )
        self.last_loss_reason = 'mission_stop'
        self.set_state(STOP, 'mission_stop', now)
        self.publish_zero()

    def on_gait_control_lock(self, msg):
        locked = bool(msg.data)
        if locked == self.gait_control_locked:
            return

        self.gait_control_locked = locked
        if locked:
            self.get_logger().warn(
                'gait_control_lock=True; line follower cmd_vel output paused'
            )
            self.publish_zero()
        else:
            self.get_logger().info(
                'gait_control_lock=False; line follower output resumed'
            )

    def on_line_track(self, msg):
        self.refresh_parameters()
        now = self.get_clock().now()
        self.last_line_msg = msg
        self.last_line_msg_time = now

        if not self.is_valid_line_msg(msg):
            self.last_loss_reason = 'invalid_line_track'
            if self.state in RUNNING_STATES:
                self.log_invalid_line_track_stop(msg, now)
                self.set_state(STOP, 'invalid_line_track', now)
                self.publish_zero()
            return

        if self.is_trackable_line(msg):
            self.last_seen_line_time = now
            self.last_lateral_error = float(msg.lateral_error)
            self.last_heading_error = float(msg.heading_error)
            self.last_turn_direction = self.direction_from_errors(msg)

            if self.state == SHORT_LOST:
                self.log_line_recovered('line_recovered', msg)
                self.set_state(LINE_FOLLOW, 'line_recovered', now)
        elif self.state == LINE_FOLLOW:
            self.last_loss_reason = self.line_loss_reason(msg)
            self.set_state(SHORT_LOST, self.last_loss_reason, now)

        if self.state == SEARCH_LINE:
            self.update_reacquire_count(msg, now)

    def on_control_timer(self):
        self.refresh_parameters()
        self.publish_state()
        now = self.get_clock().now()

        if self.gait_control_locked:
            self.publish_zero()
            self.log_debug('paused by /gait/control_lock')
            return

        if self.state == WAIT_START:
            self.publish_zero()
            self.log_debug('waiting for /mission/start')
            return

        if self.state == STOP:
            self.publish_zero()
            self.log_debug('stopped')
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
            self.publish_zero()
            return

        if self.state == LINE_FOLLOW:
            cmd = self.command_line_follow(now)
        elif self.state == SHORT_LOST:
            cmd = self.command_short_lost(now)
        elif self.state == TURN_90:
            cmd = self.command_turn_90(now)
        elif self.state == SEARCH_LINE:
            cmd = self.command_search_line(now)
        else:
            self.set_state(STOP, f'unhandled_state_{self.state}', now)
            self.publish_zero()
            return

        self.publisher.publish(cmd)
        self.log_debug(
            'cmd_vel: '
            f'state={self.state}, '
            f'linear.x={cmd.linear.x:.3f}, '
            f'angular.z={cmd.angular.z:.3f}, '
            f'last_loss_reason={self.last_loss_reason}, '
            f'stable_seen_count={self.stable_seen_count}'
        )

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
            self.last_loss_reason = self.line_loss_reason(msg)
            self.set_state(SHORT_LOST, self.last_loss_reason, now)
            return self.make_short_lost_cmd()

        cmd = Twist()
        angular = -(
            self.kp_lateral * float(msg.lateral_error) +
            self.kp_heading * float(msg.heading_error)
        )
        cmd.angular.z = self.clamp(
            angular,
            -self.max_angular_z,
            self.max_angular_z
        )

        error_abs = abs(float(msg.lateral_error))
        if error_abs < self.error_slow_threshold:
            cmd.linear.x = self.base_speed
        elif error_abs < self.error_slowest_threshold:
            cmd.linear.x = self.mid_speed
        else:
            cmd.linear.x = self.slow_speed

        return cmd

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

    def command_search_line(self, now):
        if self.stable_seen_count >= self.line_reacquire_count:
            self.log_line_recovered('line_reacquired', self.last_line_msg)
            self.set_state(LINE_FOLLOW, 'line_reacquired', now)
            return self.command_line_follow(now)

        elapsed = self.elapsed_in_state(now)
        if elapsed >= self.search_timeout:
            self.get_logger().error(
                'Search line timeout: '
                f'elapsed={elapsed:.3f}s, '
                f'timeout={self.search_timeout:.3f}s, '
                f'turn_direction={self.active_turn_direction}, '
                f'stable_seen_count={self.stable_seen_count}'
            )
            self.set_state(STOP, 'search_timeout', now)
            return Twist()

        return self.make_search_line_cmd()

    def update_reacquire_count(self, msg, now):
        if self.is_reacquired_line(msg):
            self.stable_seen_count += 1
            if self.stable_seen_count >= self.line_reacquire_count:
                self.log_line_recovered('line_reacquired', msg)
                self.set_state(LINE_FOLLOW, 'line_reacquired', now)
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

    def make_short_lost_cmd(self):
        cmd = Twist()
        cmd.linear.x = self.short_lost_linear_speed
        cmd.angular.z = self.last_turn_direction * self.search_angular_speed
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
            self.active_turn_direction * self.search_line_angular_speed
        )
        return cmd

    def publish_zero(self):
        self.publisher.publish(Twist())

    def publish_state(self):
        msg = String()
        msg.data = self.state
        self.state_publisher.publish(msg)

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
        self.publish_state()

        if new_state in {LINE_FOLLOW, WAIT_START, STOP}:
            self.stable_seen_count = 0
        if new_state == WAIT_START:
            self.get_logger().info(
                'Enter state WAIT_START: '
                f'reason={reason}, waiting for mission start'
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
        self.get_logger().info(
            'Line recovered: '
            f'state={self.state}, '
            f'reason={reason}, '
            f'stable_seen_count={self.stable_seen_count}, '
            f'required_count={self.line_reacquire_count}, '
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
            self.base_speed,
            self.mid_speed,
            self.slow_speed,
            self.kp_lateral,
            self.kp_heading,
            self.max_angular_z,
            self.short_lost_timeout,
            self.turn_90_duration,
            self.turn_90_angular_speed,
            self.search_timeout,
            self.line_msg_timeout,
        ]
        return all(math.isfinite(float(value)) for value in values)

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
            f'locked={self.gait_control_locked}, '
            f'last_line_age={self.current_line_age():.3f}, '
            f'last_turn_direction={self.last_turn_direction}, '
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

    def bool_parameter(self, name):
        return bool(self.get_parameter(name).value)

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
