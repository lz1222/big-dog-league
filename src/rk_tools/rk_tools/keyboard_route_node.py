#!/usr/bin/env python3

import json
import math
import os
import select
import subprocess
import sys
import termios
import time
import tty
from contextlib import contextmanager
from datetime import datetime, timezone

import rclpy
from geometry_msgs.msg import Twist
from rk_interfaces.msg import LineTrack
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool


SCHEMA = 'rk_keyboard_route.v1'
DEFAULT_ROUTE_FILE = '~/rk_keyboard_routes/latest_route.json'
SDK_LD_LIBRARY_PATH_PREFIX = (
    '/home/unitree/rk_inspection_ws/third_party/unitree_sdk2_official/'
    'thirdparty/lib/aarch64',
    '/home/unitree/rk_inspection_ws/install/rk_go2_sdk_bridge/lib',
    '/home/unitree/rk_inspection_ws/third_party/unitree_sdk2_official/'
    'thirdparty/lib/x86_64',
    '/usr/local/lib',
    '/home/unitree/cyclonedds_ws/install/cyclonedds/lib',
)


@contextmanager
def cbreak_terminal():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        yield
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


class KeyboardRouteNode(Node):
    """Record keyboard routes and replay them with line-follow stages."""

    MOTION_KEYS = {'w', 's', 'a', 'd', ' '}
    VALID_MODES = {'record', 'replay'}

    def __init__(self, default_mode='record'):
        super().__init__('keyboard_route_node')

        self.default_mode = default_mode
        self.mode = self._string_parameter('mode', default_mode)
        self.route_file = os.path.expanduser(
            self._string_parameter('route_file', DEFAULT_ROUTE_FILE)
        )
        self.cmd_vel_topic = self._string_parameter(
            'cmd_vel_topic',
            '/navigation/cmd_vel'
        )
        self.mission_start_topic = self._string_parameter(
            'mission_start_topic',
            '/mission/start'
        )
        self.mission_stop_topic = self._string_parameter(
            'mission_stop_topic',
            '/mission/stop'
        )
        self.line_track_topic = self._string_parameter(
            'line_track_topic',
            '/perception/line_track'
        )
        self.publish_rate_hz = self._positive_float_parameter(
            'publish_rate_hz',
            20.0
        )
        self.forward_speed = self._nonnegative_float_parameter(
            'forward_speed',
            0.30
        )
        self.backward_speed = self._nonnegative_float_parameter(
            'backward_speed',
            0.30
        )
        self.turn_speed = self._nonnegative_float_parameter(
            'turn_speed',
            0.80
        )
        self.key_action_duration_sec = self._positive_float_parameter(
            'key_action_duration_sec',
            1.0
        )
        self.max_linear_x = self._positive_float_parameter(
            'max_linear_x',
            0.60
        )
        self.max_angular_z = self._positive_float_parameter(
            'max_angular_z',
            1.00
        )
        self.speed_scale = self._positive_float_parameter(
            'speed_scale',
            1.0
        )
        self.duration_scale = self._positive_float_parameter(
            'duration_scale',
            1.0
        )
        self.record_min_duration_sec = self._nonnegative_float_parameter(
            'record_min_duration_sec',
            0.05
        )
        self.pre_stop_sec = self._nonnegative_float_parameter(
            'pre_stop_sec',
            0.30
        )
        self.step_stop_sec = self._nonnegative_float_parameter(
            'step_stop_sec',
            0.10
        )
        self.final_stop_sec = self._nonnegative_float_parameter(
            'final_stop_sec',
            0.80
        )
        self.line_insert_duration_sec = self._nonnegative_float_parameter(
            'line_insert_duration_sec',
            3.0
        )
        self.line_until_lost_max_sec = self._nonnegative_float_parameter(
            'line_until_lost_max_sec',
            30.0
        )
        self.line_visible_wait_timeout_sec = (
            self._nonnegative_float_parameter(
                'line_visible_wait_timeout_sec',
                8.0
            )
        )
        self.line_lost_switch_sec = self._nonnegative_float_parameter(
            'line_lost_switch_sec',
            0.60
        )
        self.line_track_stale_sec = self._nonnegative_float_parameter(
            'line_track_stale_sec',
            0.80
        )
        self.execute_line_markers_during_record = self._bool_parameter(
            'execute_line_markers_during_record',
            True
        )
        self.execute_sdk_actions = self._bool_parameter(
            'execute_sdk_actions',
            True
        )
        self.require_sdk_actions = self._bool_parameter(
            'require_sdk_actions',
            False
        )
        self.reapply_gait_before_motion = self._bool_parameter(
            'reapply_gait_before_motion',
            True
        )
        self.sdk_network_interface = self._string_parameter(
            'sdk_network_interface',
            'eth0'
        )
        self.sdk_action_executable = self._string_parameter(
            'sdk_action_executable',
            ''
        )
        self.sdk_action_timeout_padding_sec = (
            self._positive_float_parameter(
                'sdk_action_timeout_padding_sec',
                6.0
            )
        )
        self.economic_gait_action = self._string_parameter(
            'economic_gait_action',
            'economic_gait'
        )
        self.normal_gait_action = self._string_parameter(
            'normal_gait_action',
            'balance_stand'
        )

        self._validate_parameters()

        self.publisher = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.mission_start_publisher = self.create_publisher(
            Bool,
            self.mission_start_topic,
            10
        )
        self.mission_stop_publisher = self.create_publisher(
            Bool,
            self.mission_stop_topic,
            10
        )
        self.line_track_subscription = self.create_subscription(
            LineTrack,
            self.line_track_topic,
            self._on_line_track,
            10
        )

        self._last_line_track_msg = None
        self._last_line_track_time = None
        self._active_sdk_process = None
        self._sdk_action_executable_resolved = None
        self._route = None
        self._active_motion_segment = None
        self._active_motion_start = None
        self._active_cmd = Twist()
        self._last_key_label = 'stop'
        self._current_gait_action = None

    def _string_parameter(self, name, default):
        return str(self.declare_parameter(name, default).value).strip()

    def _bool_parameter(self, name, default):
        value = self.declare_parameter(name, default).value
        if isinstance(value, bool):
            return value
        return str(value).strip().lower() in ('1', 'true', 'yes', 'on')

    def _positive_float_parameter(self, name, default):
        value = float(self.declare_parameter(name, default).value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f'{name} must be a finite positive number')
        return value

    def _nonnegative_float_parameter(self, name, default):
        value = float(self.declare_parameter(name, default).value)
        if not math.isfinite(value) or value < 0.0:
            raise ValueError(f'{name} must be a finite nonnegative number')
        return value

    def _validate_parameters(self):
        if self.mode not in self.VALID_MODES:
            valid_modes = ', '.join(sorted(self.VALID_MODES))
            raise ValueError(f'mode must be one of: {valid_modes}')
        if not self.route_file:
            raise ValueError('route_file must not be empty')
        if not self.cmd_vel_topic:
            raise ValueError('cmd_vel_topic must not be empty')
        if not self.mission_start_topic:
            raise ValueError('mission_start_topic must not be empty')
        if not self.mission_stop_topic:
            raise ValueError('mission_stop_topic must not be empty')
        if not self.line_track_topic:
            raise ValueError('line_track_topic must not be empty')
        if not self.sdk_network_interface:
            raise ValueError('sdk_network_interface must not be empty')
        if not self.economic_gait_action:
            raise ValueError('economic_gait_action must not be empty')
        if not self.normal_gait_action:
            raise ValueError('normal_gait_action must not be empty')

    def run(self):
        self.get_logger().info(
            f'keyboard route node mode={self.mode}, '
            f'route_file={self.route_file}, cmd_vel={self.cmd_vel_topic}'
        )
        if self.mode == 'record':
            self.record_route()
        else:
            self.replay_route()

    def record_route(self):
        if not sys.stdin.isatty():
            raise RuntimeError(
                'keyboard route recording needs an interactive terminal'
            )

        self._route = self._new_route()
        self._print_record_help()
        self._publish_stop('record initial stop', self.pre_stop_sec)

        period_sec = 1.0 / self.publish_rate_hz
        last_status_time = 0.0

        try:
            with cbreak_terminal():
                while rclpy.ok():
                    key = self._read_key()
                    if key:
                        if len(key) == 1:
                            key = key.lower()
                        if key in ('\x03', '\x04', 'q'):
                            break
                        self._handle_record_key(key)

                    self.publisher.publish(Twist())
                    rclpy.spin_once(self, timeout_sec=0.0)

                    now = time.monotonic()
                    if now - last_status_time >= 1.0:
                        self._print_record_status()
                        last_status_time = now

                    time.sleep(period_sec)
        finally:
            self._finalize_active_motion('record finished')
            self._publish_mission_stop('record final mission_stop', 0.20)
            self._publish_stop('record final stop', self.final_stop_sec)
            self._write_route()

    def _print_record_help(self):
        print('', flush=True)
        print('Keyboard route recording controls:', flush=True)
        print(
            '  w forward, s backward, a turn left, d turn right',
            flush=True
        )
        print(
            f'  each motion key runs once for '
            f'{self.key_action_duration_sec:.2f}s',
            flush=True
        )
        print('  x economic_gait, c normal gait, space stop', flush=True)
        print('  l insert timed line-follow stage', flush=True)
        print('  u insert line-follow-until-lost stage', flush=True)
        print('  q finish and write route', flush=True)
        print('', flush=True)

    def _print_record_status(self):
        print(
            f'\ractive={self._last_key_label:<18} '
            f'segments={len(self._route["segments"]):03d}',
            end='',
            flush=True
        )

    def _read_key(self):
        readable, _, _ = select.select([sys.stdin], [], [], 0.0)
        if not readable:
            return ''
        return sys.stdin.read(1)

    def _handle_record_key(self, key):
        if key in self.MOTION_KEYS:
            self._record_motion_once(key)
            return

        if key == 'x':
            self._record_sdk_action(
                'x',
                self.economic_gait_action,
                'economic_gait'
            )
            return

        if key == 'c':
            self._record_sdk_action(
                'c',
                self.normal_gait_action,
                'normal_gait'
            )
            return

        if key == 'l':
            segment = {
                'type': 'line_follow',
                'key': 'l',
                'mode': 'duration',
                'duration_sec': round(self.line_insert_duration_sec, 3),
                'until_lost': False,
            }
            self._record_line_follow_segment(segment)
            return

        if key == 'u':
            segment = {
                'type': 'line_follow',
                'key': 'u',
                'mode': 'until_lost',
                'duration_sec': round(self.line_until_lost_max_sec, 3),
                'until_lost': True,
            }
            self._record_line_follow_segment(segment)
            return

    def _record_motion_once(self, key):
        segment, cmd, label = self._motion_segment_for_key(key)
        duration_sec = self.key_action_duration_sec
        segment['duration_sec'] = round(duration_sec, 3)
        self._route['segments'].append(segment)
        self._last_key_label = label
        self._reapply_current_gait_before_motion(label)
        self.get_logger().info(
            f'key={key!r}: run once {label}, vx={cmd.linear.x:.3f}, '
            f'wz={cmd.angular.z:.3f}, duration={duration_sec:.2f}s'
        )
        self._publish_for_duration(cmd, duration_sec)
        self._publish_stop(f'after key {label}', self.step_stop_sec)

    def _record_sdk_action(self, key, action, label):
        self._finalize_active_motion(f'key {label}')
        self._active_cmd = Twist()
        self._last_key_label = label

        segment = {
            'type': 'sdk_action',
            'key': key,
            'label': label,
            'action': action,
            'wait_sec': 1.0,
        }
        self._route['segments'].append(segment)
        self.get_logger().warn(
            f'recorded sdk_action label={label}, action={action}'
        )
        self._run_sdk_action_segment(segment, fatal=False)
        self._track_gait_action(action, label)
        self.get_logger().info(
            f'sdk_action key {key!r} finished; recorder stays active'
        )

    def _record_line_follow_segment(self, segment):
        self._finalize_active_motion(f'key {segment["mode"]}')
        self._active_cmd = Twist()
        self._last_key_label = segment['mode']
        self._route['segments'].append(segment)
        self.get_logger().warn(
            f'recorded line_follow mode={segment["mode"]}, '
            f'duration={segment["duration_sec"]:.2f}s'
        )
        if self.execute_line_markers_during_record:
            self._run_line_follow_segment(segment)

    def _motion_segment_for_key(self, key):
        cmd = Twist()
        if key == 'w':
            label = 'forward'
            cmd.linear.x = self.forward_speed
        elif key == 's':
            label = 'backward'
            cmd.linear.x = -self.backward_speed
        elif key == 'a':
            label = 'turn_left'
            cmd.angular.z = self.turn_speed
        elif key == 'd':
            label = 'turn_right'
            cmd.angular.z = -self.turn_speed
        else:
            label = 'stop'

        segment = {
            'type': 'cmd_vel',
            'key': key,
            'label': label,
            'linear_x': round(cmd.linear.x, 4),
            'angular_z': round(cmd.angular.z, 4),
        }
        return segment, cmd, label

    def _finalize_active_motion(self, reason):
        if self._active_motion_segment is None:
            return

        now = time.monotonic()
        duration = now - float(self._active_motion_start or now)
        segment = dict(self._active_motion_segment)
        segment['duration_sec'] = round(duration, 3)

        if duration >= self.record_min_duration_sec:
            self._route['segments'].append(segment)
            self.get_logger().info(
                f'recorded {segment["label"]}: '
                f'{duration:.2f}s ({reason})'
            )

        self._active_motion_segment = None
        self._active_motion_start = None

    def _new_route(self):
        return {
            'schema': SCHEMA,
            'created_at': datetime.now(timezone.utc).isoformat(),
            'cmd_vel_topic': self.cmd_vel_topic,
            'mission_start_topic': self.mission_start_topic,
            'mission_stop_topic': self.mission_stop_topic,
            'line_track_topic': self.line_track_topic,
            'controls': {
                'w': 'forward',
                's': 'backward',
                'a': 'turn_left',
                'd': 'turn_right',
                'x': 'economic_gait',
                'c': 'normal_gait',
                'l': 'line_follow_duration',
                'u': 'line_follow_until_lost',
                'space': 'stop',
            },
            'segments': [],
        }

    def _write_route(self):
        directory = os.path.dirname(self.route_file)
        if directory:
            os.makedirs(directory, exist_ok=True)

        with open(self.route_file, 'w', encoding='utf-8') as route_handle:
            json.dump(self._route, route_handle, indent=2, sort_keys=True)
            route_handle.write('\n')

        self.get_logger().warn(
            f'route saved: {self.route_file}, '
            f'segments={len(self._route["segments"])}'
        )

    def replay_route(self):
        route = self._load_route()
        segments = route.get('segments', [])
        if not segments:
            raise RuntimeError(f'route has no segments: {self.route_file}')

        self.get_logger().warn(
            f'replaying route {self.route_file}, segments={len(segments)}, '
            f'duration_scale={self.duration_scale:.3f}, '
            f'speed_scale={self.speed_scale:.3f}'
        )
        self._publish_stop('replay initial stop', self.pre_stop_sec)

        try:
            for index, segment in enumerate(segments, start=1):
                self._run_segment(index, len(segments), segment)
                self._publish_stop('replay step stop', self.step_stop_sec)
        finally:
            self._publish_mission_stop('replay final mission_stop', 0.20)
            self._publish_stop('replay final stop', self.final_stop_sec)

        self.get_logger().info('route replay completed')

    def _load_route(self):
        with open(self.route_file, 'r', encoding='utf-8') as route_handle:
            route = json.load(route_handle)

        schema = route.get('schema')
        if schema != SCHEMA:
            raise RuntimeError(
                f'unsupported route schema: {schema!r}, expected {SCHEMA!r}'
            )
        return route

    def _run_segment(self, index, total, segment):
        segment_type = segment.get('type')
        label = segment.get('label', segment_type)
        self.get_logger().info(
            f'route segment {index}/{total}: type={segment_type}, '
            f'label={label}'
        )

        if segment_type == 'cmd_vel':
            self._run_cmd_vel_segment(segment)
            return
        if segment_type == 'sdk_action':
            self._run_sdk_action_segment(segment)
            self._track_gait_action(
                str(segment.get('action', '')),
                str(segment.get('label', ''))
            )
            return
        if segment_type == 'line_follow':
            self._run_line_follow_segment(segment)
            return

        raise RuntimeError(f'unknown route segment type: {segment_type}')

    def _run_cmd_vel_segment(self, segment):
        duration_sec = self._scaled_duration(segment.get('duration_sec', 0.0))
        if duration_sec <= 0.0:
            return

        cmd = Twist()
        cmd.linear.x = self._clamp(
            float(segment.get('linear_x', 0.0)) * self.speed_scale,
            -self.max_linear_x,
            self.max_linear_x
        )
        cmd.angular.z = self._clamp(
            float(segment.get('angular_z', 0.0)) * self.speed_scale,
            -self.max_angular_z,
            self.max_angular_z
        )
        self.get_logger().info(
            f'cmd_vel vx={cmd.linear.x:.3f}, wz={cmd.angular.z:.3f}, '
            f'duration={duration_sec:.2f}s'
        )
        self._reapply_current_gait_before_motion(
            str(segment.get('label', 'cmd_vel'))
        )
        self._publish_for_duration(cmd, duration_sec)

    def _track_gait_action(self, action, label):
        if label == 'normal_gait':
            self._current_gait_action = None
            self.get_logger().info(
                'current gait action cleared by normal_gait'
            )
            return

        if label != 'economic_gait':
            return

        self._current_gait_action = action
        self.get_logger().info(
            f'current gait action set to {action} by {label}'
        )

    def _reapply_current_gait_before_motion(self, label):
        if (
            not self.reapply_gait_before_motion
            or not self._current_gait_action
        ):
            return

        segment = {
            'type': 'sdk_action',
            'label': 'reapply_gait',
            'action': self._current_gait_action,
            'wait_sec': 0.0,
        }
        self.get_logger().info(
            f'reapply gait before {label}: {self._current_gait_action}'
        )
        self._run_sdk_action_segment(segment, fatal=False)

    def _run_sdk_action_segment(self, segment, fatal=None):
        if fatal is None:
            fatal = self.require_sdk_actions

        if not self.execute_sdk_actions:
            self.get_logger().warn(
                f'skip sdk_action because execute_sdk_actions=false: '
                f'{segment.get("action")}'
            )
            return

        action = str(segment.get('action', '')).strip()
        wait_sec = self._nonnegative_float(segment.get('wait_sec', 1.0))
        if not action:
            raise RuntimeError('sdk_action segment has empty action')

        self._publish_stop(f'before sdk_action {action}', self.pre_stop_sec)
        timeout_sec = wait_sec + self.sdk_action_timeout_padding_sec
        command = self._sdk_action_command(action, wait_sec)

        self.get_logger().warn(
            f'run sdk_action={action}, interface='
            f'{self.sdk_network_interface}, wait={wait_sec:.2f}s'
        )

        try:
            process = subprocess.Popen(command, env=self._sdk_action_env())
            self._active_sdk_process = process
            deadline = time.monotonic() + timeout_sec
            return_code = None

            while rclpy.ok():
                return_code = process.poll()
                if return_code is not None:
                    break
                if time.monotonic() >= deadline:
                    self._terminate_active_sdk_process()
                    self._handle_sdk_action_error(
                        (
                            f'sdk_action {action} timeout after '
                            f'{timeout_sec:.2f}s'
                        ),
                        fatal
                    )
                    return
                rclpy.spin_once(self, timeout_sec=0.0)
                time.sleep(0.05)
        except FileNotFoundError as error:
            message = f'sdk action helper not found: {error}'
            if fatal:
                raise RuntimeError(message) from error
            self.get_logger().error(message)
            return
        except OSError as error:
            message = f'failed to start sdk action helper: {error}'
            if fatal:
                raise RuntimeError(message) from error
            self.get_logger().error(message)
            return
        finally:
            self._active_sdk_process = None

        if return_code != 0:
            message = (
                f'sdk_action {action} failed with exit code {return_code}'
            )
            self._handle_sdk_action_error(message, fatal)

    def _handle_sdk_action_error(self, message, fatal):
        if fatal:
            raise RuntimeError(message)
        self.get_logger().error(message)

    def _run_line_follow_segment(self, segment):
        until_lost = bool(segment.get('until_lost', False))
        duration_sec = self._scaled_duration(
            segment.get('duration_sec', self.line_insert_duration_sec)
        )
        if until_lost and duration_sec <= 0.0:
            duration_sec = self.line_until_lost_max_sec

        self.get_logger().warn(
            f'line_follow until_lost={until_lost}, '
            f'max_or_duration={duration_sec:.2f}s'
        )
        self._wait_for_line_visible('line_follow')
        self._publish_stop('before line_follow', self.pre_stop_sec)
        self._publish_mission_start('line_follow mission_start', 0.30)

        if until_lost:
            self._wait_line_follow_until_lost(duration_sec)
        else:
            self._wait_for_duration(duration_sec)

        self._publish_mission_stop('line_follow mission_stop', 0.30)
        self._publish_stop('after line_follow', self.step_stop_sec)

    def _wait_line_follow_until_lost(self, max_duration_sec):
        period_sec = 1.0 / self.publish_rate_hz
        start_time = time.monotonic()
        lost_since = None
        last_report_time = 0.0

        while rclpy.ok():
            now = time.monotonic()
            if max_duration_sec > 0.0 and now - start_time >= max_duration_sec:
                self.get_logger().warn(
                    'line_follow_until_lost max duration reached'
                )
                return

            visible, reason = self._line_is_visible_now()
            if visible:
                lost_since = None
            else:
                if lost_since is None:
                    lost_since = now
                    self.get_logger().warn(
                        f'line lost candidate during route: {reason}'
                    )
                if now - lost_since >= self.line_lost_switch_sec:
                    self.get_logger().warn(
                        f'line lost for {now - lost_since:.2f}s; '
                        'ending line_follow stage'
                    )
                    return

            if now - last_report_time >= 1.0:
                self._log_line_status('line_follow_until_lost')
                last_report_time = now

            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period_sec)

    def _wait_for_line_visible(self, label):
        timeout_sec = self.line_visible_wait_timeout_sec
        if timeout_sec <= 0.0:
            return

        start_time = time.monotonic()
        last_report_time = 0.0
        self.get_logger().info(
            f'{label}: wait for visible line on {self.line_track_topic}, '
            f'timeout={timeout_sec:.1f}s'
        )

        while rclpy.ok():
            visible, _reason = self._line_is_visible_now()
            if visible:
                self._log_line_status(f'{label}: line visible')
                return

            now = time.monotonic()
            if now - start_time >= timeout_sec:
                self.get_logger().warn(
                    f'{label}: no visible line within {timeout_sec:.1f}s; '
                    'starting line_follow anyway'
                )
                return

            if now - last_report_time >= 1.0:
                self._log_line_status(f'{label}: waiting')
                last_report_time = now

            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.05)

    def _line_is_visible_now(self):
        if self._last_line_track_msg is None:
            return False, 'no_line_track'

        now = time.monotonic()
        age = now - float(self._last_line_track_time or now)
        if age > self.line_track_stale_sec:
            return False, f'line_track_stale_{age:.2f}s'
        if not bool(self._last_line_track_msg.line_visible):
            return False, 'line_visible_false'
        return True, 'line_visible'

    def _log_line_status(self, label):
        msg = self._last_line_track_msg
        if msg is None:
            self.get_logger().info(f'{label}: no line_track received yet')
            return

        age = time.monotonic() - float(self._last_line_track_time or 0.0)
        self.get_logger().info(
            f'{label}: visible={bool(msg.line_visible)}, '
            f'confidence={msg.confidence:.3f}, '
            f'lateral={msg.lateral_error:.3f}, '
            f'heading={msg.heading_error:.3f}, age={age:.2f}s'
        )

    def _on_line_track(self, msg):
        self._last_line_track_msg = msg
        self._last_line_track_time = time.monotonic()

    def _publish_mission_start(self, label, duration_sec):
        self._publish_mission_command(
            self.mission_start_publisher,
            True,
            label,
            duration_sec
        )

    def _publish_mission_stop(self, label, duration_sec):
        self._publish_mission_command(
            self.mission_stop_publisher,
            True,
            label,
            duration_sec
        )

    def _publish_mission_command(self, publisher, value, label, duration_sec):
        msg = Bool()
        msg.data = bool(value)
        period_sec = 1.0 / self.publish_rate_hz
        end_time = time.monotonic() + max(duration_sec, period_sec)

        self.get_logger().info(
            f'{label}: publish {msg.data} for '
            f'{max(duration_sec, period_sec):.2f}s'
        )
        while rclpy.ok() and time.monotonic() < end_time:
            publisher.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period_sec)

    def _publish_stop(self, label, duration_sec):
        if duration_sec <= 0.0:
            return
        self.get_logger().info(
            f'{label}: zero cmd_vel for {duration_sec:.2f}s'
        )
        self._publish_for_duration(Twist(), duration_sec)

    def _publish_for_duration(self, cmd, duration_sec):
        period_sec = 1.0 / self.publish_rate_hz
        end_time = time.monotonic() + duration_sec
        while rclpy.ok() and time.monotonic() < end_time:
            self.publisher.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period_sec)

    def _wait_for_duration(self, duration_sec):
        period_sec = 1.0 / self.publish_rate_hz
        end_time = time.monotonic() + duration_sec
        while rclpy.ok() and time.monotonic() < end_time:
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period_sec)

    def _sdk_action_command(self, action, wait_sec):
        executable = self._resolve_sdk_action_executable()
        if executable:
            return [
                executable,
                self.sdk_network_interface,
                action,
                f'{wait_sec:.3f}',
            ]
        return [
            'ros2',
            'run',
            'rk_go2_sdk_bridge',
            'go2_sdk_motion_action',
            self.sdk_network_interface,
            action,
            f'{wait_sec:.3f}',
        ]

    def _resolve_sdk_action_executable(self):
        if self._sdk_action_executable_resolved is not None:
            return self._sdk_action_executable_resolved

        explicit = self.sdk_action_executable.strip()
        if explicit:
            candidates = [os.path.expanduser(explicit)]
        else:
            candidates = [
                os.environ.get('RK_GO2_SDK_MOTION_ACTION', ''),
                os.path.join(
                    os.getcwd(),
                    'install',
                    'rk_go2_sdk_bridge',
                    'lib',
                    'rk_go2_sdk_bridge',
                    'go2_sdk_motion_action'
                ),
                os.path.expanduser(
                    '~/rk_inspection_ws/install/rk_go2_sdk_bridge/lib/'
                    'rk_go2_sdk_bridge/go2_sdk_motion_action'
                ),
                (
                    '/home/unitree/rk_inspection_ws/install/'
                    'rk_go2_sdk_bridge/lib/rk_go2_sdk_bridge/'
                    'go2_sdk_motion_action'
                ),
            ]

        for candidate in candidates:
            if (
                candidate
                and os.path.isfile(candidate)
                and os.access(candidate, os.X_OK)
            ):
                self._sdk_action_executable_resolved = candidate
                return candidate

        self._sdk_action_executable_resolved = ''
        return ''

    def _sdk_action_env(self):
        env = os.environ.copy()
        paths = list(SDK_LD_LIBRARY_PATH_PREFIX)
        current = env.get('LD_LIBRARY_PATH', '')
        if current:
            paths.extend(current.split(':'))

        merged_paths = []
        seen = set()
        for path in paths:
            if path and path not in seen:
                merged_paths.append(path)
                seen.add(path)

        env['LD_LIBRARY_PATH'] = ':'.join(merged_paths)
        return env

    def _terminate_active_sdk_process(self):
        process = self._active_sdk_process
        if process is None or process.poll() is not None:
            return
        try:
            process.terminate()
            process.wait(timeout=0.30)
        except subprocess.TimeoutExpired:
            try:
                process.kill()
                process.wait(timeout=0.30)
            except (OSError, subprocess.TimeoutExpired):
                pass
        except OSError:
            pass

    def _scaled_duration(self, value):
        duration = self._nonnegative_float(value)
        return duration * self.duration_scale

    @staticmethod
    def _nonnegative_float(value):
        result = float(value)
        if not math.isfinite(result) or result < 0.0:
            raise ValueError('duration must be a finite nonnegative number')
        return result

    @staticmethod
    def _clamp(value, lower, upper):
        return max(lower, min(upper, value))


def _run(default_mode, args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = KeyboardRouteNode(default_mode=default_mode)
        node.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        if node is not None:
            node.get_logger().warn('Interrupted by user')
            node._terminate_active_sdk_process()
            node._publish_mission_stop('interrupt mission_stop', 0.20)
            node._publish_stop('interrupt final stop', node.final_stop_sec)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(args=None):
    _run('record', args=args)


def record_main(args=None):
    _run('record', args=args)


def replay_main(args=None):
    _run('replay', args=args)


if __name__ == '__main__':
    main()
