#!/usr/bin/env python3

import json
import math
import os
import subprocess
import threading
import time
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, String

from rk_interfaces.msg import SignDetectionArray


DEFAULT_ACTION_MAP = {
    'warning:electric_shock': [
        {'sdk_action': 'stretch', 'wait_sec': 3.0},
    ],
    'warning:strong_oxidizer': [
        {'sdk_action': 'hello', 'wait_sec': 3.0},
    ],
    'warning:radiation': [
        {'sdk_action': 'blink_front_light_3', 'wait_sec': 2.5},
    ],
    'place_marker:place_1': [
        {'command': 'HOLD_STABLE', 'duration_sec': 0.8},
    ],
    'place_marker:place_2': [
        {'command': 'HOLD_STABLE', 'duration_sec': 0.8},
    ],
}

DEFAULT_SDK_ACTION_EXECUTABLE = (
    '/home/unitree/rk_inspection_ws/install/'
    'rk_go2_sdk_bridge/lib/rk_go2_sdk_bridge/go2_sdk_motion_action'
)
SDK_LD_LIBRARY_PATH_PREFIX = (
    '/home/unitree/rk_inspection_ws/third_party/unitree_sdk2_official/'
    'thirdparty/lib/aarch64',
    '/home/unitree/rk_inspection_ws/install/rk_go2_sdk_bridge/lib',
    '/home/unitree/rk_inspection_ws/third_party/unitree_sdk2_official/'
    'thirdparty/lib/x86_64',
    '/usr/local/lib',
    '/home/unitree/cyclonedds_ws/install/cyclonedds/lib',
)


def normalize_label(value):
    normalized = str(value or '').strip().lower()
    normalized = normalized.replace('-', '_').replace(' ', '_')
    while '__' in normalized:
        normalized = normalized.replace('__', '_')
    return normalized


def normalize_action_key(value):
    if ':' not in str(value):
        return normalize_label(value)
    sign_type, sign_value = str(value).split(':', 1)
    return f'{normalize_label(sign_type)}:{normalize_label(sign_value)}'


def parse_action_map(raw_json):
    if not raw_json:
        raw = DEFAULT_ACTION_MAP
    else:
        try:
            raw = json.loads(raw_json)
        except (TypeError, ValueError):
            raw = DEFAULT_ACTION_MAP
    if not isinstance(raw, dict):
        raw = DEFAULT_ACTION_MAP

    parsed: Dict[str, List[dict]] = {}
    for key, sequence in raw.items():
        normalized_key = normalize_action_key(key)
        if not isinstance(sequence, list):
            continue
        steps = []
        for step in sequence:
            if isinstance(step, dict):
                steps.append(dict(step))
        if steps:
            parsed[normalized_key] = steps
    return parsed


class SignActionExecutorNode(Node):
    """Trigger conservative Go2 body actions from sign detections."""

    def __init__(self):
        super().__init__('sign_action_executor_node')
        self.callback_group = ReentrantCallbackGroup()
        self._declare_parameters()

        self.sign_detections_topic = self._string_parameter(
            'sign_detections_topic'
        )
        self.gait_command_topic = self._string_parameter('gait_command_topic')
        self.status_topic = self._string_parameter('status_topic')
        self.mission_stop_topic = self._string_parameter('mission_stop_topic')
        self.sdk_network_interface = self._string_parameter(
            'sdk_network_interface'
        )
        self.sdk_action_executable = self._string_parameter(
            'sdk_action_executable'
        )
        self.sdk_action_timeout_padding_sec = self._float_parameter(
            'sdk_action_timeout_padding_sec',
            5.0
        )
        self.min_confidence = self._float_parameter('min_confidence', 0.60)
        self.action_cooldown_sec = self._float_parameter(
            'action_cooldown_sec',
            4.0
        )
        self.command_gap_sec = self._float_parameter('command_gap_sec', 0.15)
        self.duration_padding_sec = self._float_parameter(
            'duration_padding_sec',
            0.25
        )
        self.trigger_once_per_value = self._bool_parameter(
            'trigger_once_per_value'
        )
        self.publish_mission_stop = self._bool_parameter(
            'publish_mission_stop'
        )
        self.dry_run = self._bool_parameter('dry_run')
        self.action_map = parse_action_map(
            self._string_parameter('action_map_json')
        )

        self._state_lock = threading.RLock()
        self._active = False
        self._last_action_time = 0.0
        self._triggered_keys = set()
        self._sdk_action_executable_resolved = None

        self.gait_publisher = self.create_publisher(
            String,
            self.gait_command_topic,
            10
        )
        self.status_publisher = self.create_publisher(
            String,
            self.status_topic,
            10
        )
        self.mission_stop_publisher = self.create_publisher(
            Bool,
            self.mission_stop_topic,
            10
        )
        self.subscription = self.create_subscription(
            SignDetectionArray,
            self.sign_detections_topic,
            self._on_sign_detections,
            10,
            callback_group=self.callback_group
        )

        self._publish_status(
            '',
            'IDLE',
            '',
            True,
            'sign action executor ready'
        )
        self.get_logger().info(
            'Sign action executor ready: '
            f'sign_topic={self.sign_detections_topic}, '
            f'gait_topic={self.gait_command_topic}, '
            f'sdk_interface={self.sdk_network_interface}, '
            f'dry_run={self.dry_run}'
        )

    def _declare_parameters(self):
        self.declare_parameter(
            'sign_detections_topic',
            '/perception/sign_detections'
        )
        self.declare_parameter('gait_command_topic', '/gait/command_json')
        self.declare_parameter('status_topic', '/sign_action/status')
        self.declare_parameter('mission_stop_topic', '/mission/stop')
        self.declare_parameter('sdk_network_interface', 'eth0')
        self.declare_parameter(
            'sdk_action_executable',
            DEFAULT_SDK_ACTION_EXECUTABLE
        )
        self.declare_parameter('sdk_action_timeout_padding_sec', 5.0)
        self.declare_parameter('min_confidence', 0.60)
        self.declare_parameter('action_cooldown_sec', 4.0)
        self.declare_parameter('command_gap_sec', 0.15)
        self.declare_parameter('duration_padding_sec', 0.25)
        self.declare_parameter('trigger_once_per_value', True)
        self.declare_parameter('publish_mission_stop', True)
        self.declare_parameter('dry_run', False)
        self.declare_parameter(
            'action_map_json',
            json.dumps(DEFAULT_ACTION_MAP, separators=(',', ':'))
        )

    def _on_sign_detections(self, msg):
        candidate = self._select_candidate(msg)
        if candidate is None:
            return

        key, confidence = candidate
        sequence = self.action_map.get(key)
        if not sequence:
            return

        now = time.monotonic()
        with self._state_lock:
            if self._active:
                return
            if now - self._last_action_time < self.action_cooldown_sec:
                return
            if self.trigger_once_per_value and key in self._triggered_keys:
                return
            self._active = True
            self._last_action_time = now
            self._triggered_keys.add(key)

        thread = threading.Thread(
            target=self._execute_sequence,
            args=(key, confidence, sequence),
            daemon=True
        )
        thread.start()

    def _select_candidate(self, msg) -> Optional[Tuple[str, float]]:
        best_key = None
        best_confidence = -1.0
        for detection in msg.detections:
            confidence = float(detection.confidence)
            if not math.isfinite(confidence):
                continue
            if confidence < self.min_confidence:
                continue
            key = normalize_action_key(
                f'{detection.sign_type}:{detection.sign_value}'
            )
            if key not in self.action_map:
                continue
            if confidence > best_confidence:
                best_key = key
                best_confidence = confidence
        if best_key is None:
            return None
        return best_key, best_confidence

    def _execute_sequence(self, key, confidence, sequence):
        try:
            self._publish_status(
                key,
                'RUNNING',
                'START',
                True,
                f'triggered by sign confidence={confidence:.2f}'
            )
            if self.publish_mission_stop:
                self._publish_mission_stop()
            for index, step in enumerate(sequence):
                step_name = f'step_{index + 1}'
                wait_sec = self._optional_float(step.get('wait_sec'))
                sdk_action = str(step.get('sdk_action', '')).strip()
                if sdk_action:
                    # 一个 step 同时给出 sdk_action/wait_sec 时，wait 是动作后的
                    # 保持时间；先 wait 会直接 continue 并错误跳过 SDK 动作。
                    self._publish_status(
                        key,
                        'RUNNING',
                        sdk_action,
                        True,
                        'sdk action start'
                    )
                    self._run_sdk_action_step(step)
                    self._publish_status(
                        key,
                        'RUNNING',
                        sdk_action,
                        True,
                        'sdk action finished'
                    )
                    if wait_sec is not None:
                        self._publish_status(
                            key,
                            'RUNNING',
                            step_name,
                            True,
                            f'wait {wait_sec:.2f}s after sdk action'
                        )
                        time.sleep(max(0.0, wait_sec))
                    if self.command_gap_sec > 0.0:
                        time.sleep(self.command_gap_sec)
                    continue

                if wait_sec is not None:
                    self._publish_status(
                        key,
                        'RUNNING',
                        step_name,
                        True,
                        f'wait {wait_sec:.2f}s'
                    )
                    time.sleep(max(0.0, wait_sec))
                    continue

                command = dict(step)
                if not command.get('command'):
                    continue
                self._publish_gait_command(command)
                self._publish_status(
                    key,
                    'RUNNING',
                    str(command.get('command')),
                    True,
                    'gait command published'
                )
                duration = self._optional_float(command.get('duration_sec'))
                if duration is None:
                    duration = 0.0
                time.sleep(max(0.0, duration + self.duration_padding_sec))
                if self.command_gap_sec > 0.0:
                    time.sleep(self.command_gap_sec)

            self._publish_status(
                key,
                'DONE',
                'DONE',
                True,
                'sign action sequence completed'
            )
        except Exception as error:
            self.get_logger().error(f'sign action failed: {error}')
            self._publish_status(
                key,
                'FAILED',
                'EXCEPTION',
                False,
                str(error)
            )
        finally:
            with self._state_lock:
                self._active = False

    def _run_sdk_action_step(self, step):
        action = str(step.get('sdk_action', '')).strip()
        if not action:
            raise RuntimeError('sdk_action step has empty sdk_action')

        # wait_sec 已由调用方作为 SDK 动作后的停留时间处理，helper 必须立刻
        # 返回实际 SDK 动作结果，不能把等待误当作动作本身的一部分。
        timeout_sec = max(1.0, self.sdk_action_timeout_padding_sec)
        command = [
            self._resolve_sdk_action_executable(),
            self.sdk_network_interface,
            action,
            '0.000',
        ]

        if self.dry_run:
            self.get_logger().info(
                '[DRY_RUN] SDK action: ' + ' '.join(command)
            )
            return

        self.get_logger().warn(
            f'Running SDK sign action: action={action}, '
            f'interface={self.sdk_network_interface}, '
            f'helper_wait=0.00s, timeout={timeout_sec:.2f}s'
        )
        try:
            completed = subprocess.run(
                command,
                env=self._sdk_action_env(),
                timeout=timeout_sec,
                check=False,
            )
        except (OSError, subprocess.TimeoutExpired) as error:
            raise RuntimeError(
                f'SDK sign action {action} failed to run: {error}'
            ) from error

        if completed.returncode != 0:
            raise RuntimeError(
                f'SDK sign action {action} failed with exit code '
                f'{completed.returncode}'
            )

    def _resolve_sdk_action_executable(self):
        if self._sdk_action_executable_resolved:
            return self._sdk_action_executable_resolved

        candidates = []
        explicit = str(self.sdk_action_executable).strip()
        if explicit:
            candidates.append(os.path.expanduser(explicit))
        candidates.extend([
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
            DEFAULT_SDK_ACTION_EXECUTABLE,
        ])

        for candidate in candidates:
            if (
                candidate
                and os.path.isfile(candidate)
                and os.access(candidate, os.X_OK)
            ):
                self._sdk_action_executable_resolved = candidate
                return candidate

        checked = ', '.join(candidate for candidate in candidates if candidate)
        raise FileNotFoundError(
            'go2_sdk_motion_action not found or not executable. '
            f'Checked: {checked}'
        )

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

    def _publish_gait_command(self, command):
        payload = json.dumps(command, separators=(',', ':'))
        if self.dry_run:
            self.get_logger().info(f'[DRY_RUN] /gait/command_json {payload}')
            return
        msg = String()
        msg.data = payload
        self.gait_publisher.publish(msg)
        self.get_logger().info(f'Published gait command: {payload}')

    def _publish_mission_stop(self):
        if self.dry_run:
            self.get_logger().info('[DRY_RUN] /mission/stop true')
            return
        msg = Bool()
        msg.data = True
        self.mission_stop_publisher.publish(msg)

    def _publish_status(self, sign_key, state, step, success, message):
        msg = String()
        msg.data = json.dumps({
            'sign': sign_key,
            'state': state,
            'step': step,
            'success': bool(success),
            'message': str(message),
        }, separators=(',', ':'))
        self.status_publisher.publish(msg)

    @staticmethod
    def _optional_float(value):
        if value is None:
            return None
        try:
            result = float(value)
        except (TypeError, ValueError):
            return None
        if not math.isfinite(result):
            return None
        return result

    def _string_parameter(self, name):
        return str(self.get_parameter(name).value)

    def _bool_parameter(self, name):
        value = self.get_parameter(name).value
        if isinstance(value, str):
            return value.strip().lower() in ('1', 'true', 'yes', 'on')
        return bool(value)

    def _float_parameter(self, name, default):
        try:
            value = float(self.get_parameter(name).value)
        except (TypeError, ValueError):
            return float(default)
        if not math.isfinite(value):
            return float(default)
        return value


def main(args=None):
    rclpy.init(args=args)
    node = SignActionExecutorNode()
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
