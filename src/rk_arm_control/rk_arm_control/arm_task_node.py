#!/usr/bin/env python3

import json
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional

import rclpy
import yaml
from ament_index_python.packages import PackageNotFoundError
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionServer
from rclpy.action.server import CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, String

from rk_interfaces.action import ExecuteArmTask


STATUS_IDLE = 'IDLE'
STATUS_RUNNING = 'RUNNING'
STATUS_DONE = 'DONE'
STATUS_FAILED = 'FAILED'
STATUS_ABORTED = 'ABORTED'
STATUS_BUSY = 'BUSY'
STATUS_TIMEOUT = 'TIMEOUT'


TASK_ALIASES = {
    'PICK_START': 'PICK_START',
    'PICK_START_ITEM': 'PICK_START',
    'START_ITEM': 'PICK_START',
    'PLACE_TRANSFER': 'PLACE_TRANSFER',
    'DROP_START_ITEM': 'PLACE_TRANSFER',
    'TRANSFER_PLATFORM': 'PLACE_TRANSFER',
    'PICK_FIELD': 'PICK_FIELD',
    'PICK_FIELD_ITEM': 'PICK_FIELD',
    'FIELD_ITEM': 'PICK_FIELD',
    'PLACE_TARGET': 'PLACE_TARGET',
    'PLACE_FIELD_ITEM': 'PLACE_TARGET',
    'PLACE_PLATFORM': 'PLACE_TARGET',
    'PLACE_PLATFORM_1': 'PLACE_TARGET',
    'PLACE_PLATFORM_2': 'PLACE_TARGET',
    'HOME': 'HOME',
    'OPEN_GRIPPER': 'OPEN_GRIPPER',
    'CLOSE_GRIPPER': 'CLOSE_GRIPPER',
    'ABORT': 'ABORT',
}


TASK_SEQUENCES = {
    'PICK_START': [
        {'type': 'move', 'pose': 'HOME'},
        {'type': 'gripper', 'action': 'open'},
        {'type': 'move', 'pose': 'PRE_PICK_START'},
        {'type': 'move', 'pose': 'PICK_START'},
        {'type': 'gripper', 'action': 'close'},
        {'type': 'wait', 'duration_sec': 0.5},
        {'type': 'move', 'pose': 'LIFT_START'},
        {'type': 'move', 'pose': 'HOME'},
    ],
    'PLACE_TRANSFER': [
        {'type': 'move', 'pose': 'HOME'},
        {'type': 'move', 'pose': 'PRE_PLACE_TRANSFER'},
        {'type': 'move', 'pose': 'PLACE_TRANSFER'},
        {'type': 'gripper', 'action': 'open'},
        {'type': 'wait', 'duration_sec': 0.5},
        {'type': 'move', 'pose': 'LIFT_TRANSFER'},
        {'type': 'move', 'pose': 'HOME'},
    ],
    'PICK_FIELD': [
        {'type': 'move', 'pose': 'HOME'},
        {'type': 'gripper', 'action': 'open'},
        {'type': 'move', 'pose': 'PRE_PICK_FIELD'},
        {'type': 'move', 'pose': 'PICK_FIELD'},
        {'type': 'gripper', 'action': 'close'},
        {'type': 'wait', 'duration_sec': 0.5},
        {'type': 'move', 'pose': 'LIFT_FIELD'},
        {'type': 'move', 'pose': 'HOME'},
    ],
    'PLACE_TARGET': [
        {'type': 'move', 'pose': 'HOME'},
        {'type': 'move', 'pose': 'PRE_PLACE_TARGET'},
        {'type': 'move', 'pose': 'PLACE_TARGET'},
        {'type': 'gripper', 'action': 'open'},
        {'type': 'wait', 'duration_sec': 0.5},
        {'type': 'move', 'pose': 'LIFT_TARGET'},
        {'type': 'move', 'pose': 'HOME'},
    ],
    'HOME': [
        {'type': 'move', 'pose': 'HOME'},
    ],
    'OPEN_GRIPPER': [
        {'type': 'gripper', 'action': 'open'},
    ],
    'CLOSE_GRIPPER': [
        {'type': 'gripper', 'action': 'close'},
    ],
}


@dataclass(frozen=True)
class ExecutionResult:
    success: bool
    state: str
    message: str
    step: str = ''


class ArmHardwareAdapter:
    """Adapter boundary for replacing dry-run with the real arm backend."""

    def move_joints(self, joints: List[float], duration_sec: float) -> bool:
        raise NotImplementedError

    def open_gripper(self, duration_sec: float) -> bool:
        raise NotImplementedError

    def close_gripper(self, duration_sec: float) -> bool:
        raise NotImplementedError

    def stop(self) -> None:
        raise NotImplementedError


class DryRunArmAdapter(ArmHardwareAdapter):
    """Dry-run arm adapter used until the real hardware topics are wired."""

    def __init__(self, node: Node):
        self._node = node
        self._logger = node.get_logger()
        self._stop_event = threading.Event()
        self._logger.warn(
            'No real arm hardware interface was found. '
            'Using DryRunArmAdapter. TODO: replace with publishers/services '
            'for the real arm joint and gripper drivers.'
        )

    def move_joints(self, joints: List[float], duration_sec: float) -> bool:
        # TODO: publish to the real arm joint command topic or call the driver.
        self._stop_event.clear()
        self._logger.info(
            f'[DRY_RUN] move_joints joints={joints}, '
            f'duration_sec={duration_sec:.2f}'
        )
        return not self._sleep(duration_sec)

    def open_gripper(self, duration_sec: float) -> bool:
        # TODO: publish/call the real gripper open command.
        self._stop_event.clear()
        self._logger.info(
            f'[DRY_RUN] open_gripper duration_sec={duration_sec:.2f}'
        )
        return not self._sleep(duration_sec)

    def close_gripper(self, duration_sec: float) -> bool:
        # TODO: publish/call the real gripper close command.
        self._stop_event.clear()
        self._logger.info(
            f'[DRY_RUN] close_gripper duration_sec={duration_sec:.2f}'
        )
        return not self._sleep(duration_sec)

    def stop(self) -> None:
        self._stop_event.set()
        self._logger.warn('[DRY_RUN] arm stop requested')

    def _sleep(self, duration_sec: float) -> bool:
        deadline = time.monotonic() + max(0.0, float(duration_sec))
        while time.monotonic() < deadline:
            if self._stop_event.is_set():
                return True
            time.sleep(min(0.02, deadline - time.monotonic()))
        return self._stop_event.is_set()


class ArmTaskNode(Node):
    """Fixed-position arm task executor with action and JSON topic inputs."""

    def __init__(self):
        super().__init__('arm_task_node')
        self.callback_group = ReentrantCallbackGroup()

        self.declare_parameter('config_file', '')
        self.declare_parameter('status_topic', '/arm/status')
        self.declare_parameter('control_lock_topic', '/arm/control_lock')
        self.declare_parameter('command_json_topic', '/arm/command_json')
        self.declare_parameter('action_name', '/arm/execute_task')

        self._state_lock = threading.RLock()
        self._active_task = ''
        self._current_state = STATUS_IDLE
        self._current_step = ''
        self._abort_event = threading.Event()

        self._config = self._load_config()
        self._arm_config = self._config.get('arm', {})
        self._poses = self._arm_config.get('poses', {})
        self._gripper_config = self._arm_config.get('gripper', {})

        self.default_step_timeout_sec = self._positive_float(
            self._arm_config.get('default_step_timeout_sec'),
            5.0
        )
        self.default_move_duration_sec = self._positive_float(
            self._arm_config.get('default_move_duration_sec'),
            1.0
        )
        self.gripper_wait_sec = self._nonnegative_float(
            self._arm_config.get('gripper_wait_sec'),
            0.5
        )

        self.status_pub = self.create_publisher(
            String,
            self.get_parameter('status_topic').value,
            10
        )
        self.lock_pub = self.create_publisher(
            Bool,
            self.get_parameter('control_lock_topic').value,
            10
        )
        self.command_sub = self.create_subscription(
            String,
            self.get_parameter('command_json_topic').value,
            self._on_command_json,
            10,
            callback_group=self.callback_group
        )

        self.adapter = DryRunArmAdapter(self)

        self.action_server = ActionServer(
            self,
            ExecuteArmTask,
            self.get_parameter('action_name').value,
            self.execute_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group
        )

        self.publish_lock(False)
        self.publish_status('', STATUS_IDLE, '', True, 'arm task node ready')
        self.get_logger().info(
            'Arm task node ready: action='
            f'{self.get_parameter("action_name").value}, '
            'json_topic='
            f'{self.get_parameter("command_json_topic").value}'
        )

    def execute_callback(self, goal_handle):
        raw_task = goal_handle.request.task_name
        target = goal_handle.request.target
        task_name = self.normalize_task_name(raw_task, target)
        result = ExecuteArmTask.Result()

        if task_name == 'ABORT':
            self.request_abort('ABORT action requested')
            goal_handle.succeed()
            result.success = True
            result.message = 'abort requested'
            return result

        execution = self.execute_task_sequence(
            task_name,
            raw_task=raw_task,
            target=target,
            goal_handle=goal_handle
        )

        result.success = execution.success
        result.message = execution.message
        if execution.success:
            goal_handle.succeed()
        elif execution.state == STATUS_ABORTED and goal_handle.is_cancel_requested:
            goal_handle.canceled()
        else:
            goal_handle.abort()
        return result

    def cancel_callback(self, goal_handle):
        del goal_handle
        self.request_abort('action cancel requested')
        return CancelResponse.ACCEPT

    def _on_command_json(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as error:
            self.get_logger().error(f'invalid /arm/command_json: {error}')
            self.publish_status(
                '',
                STATUS_FAILED,
                '',
                False,
                f'invalid JSON: {error}'
            )
            return

        if not isinstance(payload, dict):
            self.publish_status(
                '',
                STATUS_FAILED,
                '',
                False,
                '/arm/command_json must contain a JSON object'
            )
            return

        raw_task = str(payload.get('task', '')).strip()
        target = str(payload.get('target', '')).strip()
        task_name = self.normalize_task_name(raw_task, target)

        if task_name == 'ABORT':
            self.request_abort('ABORT command_json requested')
            return

        thread = threading.Thread(
            target=self._run_json_task,
            args=(task_name, raw_task, target),
            daemon=True
        )
        thread.start()

    def _run_json_task(self, task_name: str, raw_task: str, target: str) -> None:
        self.execute_task_sequence(
            task_name,
            raw_task=raw_task,
            target=target,
            goal_handle=None
        )

    def execute_task_sequence(
        self,
        task_name: str,
        raw_task: str = '',
        target: str = '',
        goal_handle: Optional[Any] = None
    ) -> ExecutionResult:
        if task_name not in TASK_SEQUENCES:
            message = f'unknown arm task: {raw_task or task_name}'
            self.get_logger().error(message)
            self.publish_status(task_name, STATUS_FAILED, '', False, message)
            return ExecutionResult(False, STATUS_FAILED, message)

        if not self._try_begin_task(task_name):
            message = f'arm task busy: active={self._active_task}'
            self.get_logger().warn(message)
            self.publish_status(task_name, STATUS_BUSY, '', False, message)
            return ExecutionResult(False, STATUS_BUSY, message)

        sequence = TASK_SEQUENCES[task_name]
        last_step = ''
        try:
            self.publish_lock(True)
            self.publish_status(
                task_name,
                STATUS_RUNNING,
                '',
                True,
                'executing'
            )
            self.get_logger().info(
                f'Start arm task: task={task_name}, raw_task={raw_task}, '
                f'target={target}, steps={len(sequence)}'
            )

            for index, step in enumerate(sequence):
                last_step = self.step_name(step)
                if self._should_abort(goal_handle):
                    return self._finish_aborted(task_name, last_step)

                self.publish_action_feedback(
                    goal_handle,
                    last_step,
                    index / max(1, len(sequence))
                )
                self.publish_status(
                    task_name,
                    STATUS_RUNNING,
                    last_step,
                    True,
                    'step started'
                )
                self.get_logger().info(
                    f'[{task_name}] step {index + 1}/{len(sequence)} '
                    f'start: {last_step}'
                )

                step_result = self.execute_step(step, task_name, last_step)
                if not step_result.success:
                    return step_result

                self.publish_action_feedback(
                    goal_handle,
                    last_step,
                    (index + 1) / max(1, len(sequence))
                )
                self.publish_status(
                    task_name,
                    STATUS_RUNNING,
                    last_step,
                    True,
                    'step completed'
                )
                self.get_logger().info(
                    f'[{task_name}] step {index + 1}/{len(sequence)} '
                    f'done: {last_step}'
                )

            message = 'task completed'
            self.publish_status(task_name, STATUS_DONE, last_step, True, message)
            self.get_logger().info(f'Arm task done: {task_name}')
            return ExecutionResult(True, STATUS_DONE, message, last_step)
        except Exception as error:  # noqa: BLE001 - keep ROS node alive.
            message = f'arm task exception: {error}'
            self.get_logger().error(message)
            self.publish_status(task_name, STATUS_FAILED, last_step, False, message)
            try:
                self.adapter.stop()
            except Exception as stop_error:  # noqa: BLE001
                self.get_logger().error(f'adapter stop failed: {stop_error}')
            return ExecutionResult(False, STATUS_FAILED, message, last_step)
        finally:
            self.publish_lock(False)
            with self._state_lock:
                self._active_task = ''
                self._current_state = STATUS_IDLE
                self._current_step = ''
                self._abort_event.clear()

    def execute_step(
        self,
        step: Dict[str, Any],
        task_name: str,
        step_name: str
    ) -> ExecutionResult:
        timeout_sec = self._step_timeout(step)
        step_type = step.get('type')

        if step_type == 'move':
            pose_name = str(step.get('pose', '')).upper()
            pose = self._poses.get(pose_name)
            if not isinstance(pose, dict):
                message = f'missing arm pose in config: {pose_name}'
                self.publish_status(
                    task_name,
                    STATUS_FAILED,
                    step_name,
                    False,
                    message
                )
                return ExecutionResult(False, STATUS_FAILED, message, step_name)

            joints = [float(value) for value in pose.get('joints', [])]
            duration_sec = self._positive_float(
                pose.get('duration_sec'),
                self.default_move_duration_sec
            )
            return self._call_adapter(
                lambda: self.adapter.move_joints(joints, duration_sec),
                task_name,
                step_name,
                timeout_sec
            )

        if step_type == 'gripper':
            action = str(step.get('action', '')).lower()
            gripper = self._gripper_config.get(action, {})
            duration_sec = self._positive_float(
                gripper.get('duration_sec'),
                self.gripper_wait_sec
            )
            if action == 'open':
                operation = lambda: self.adapter.open_gripper(duration_sec)
            elif action == 'close':
                operation = lambda: self.adapter.close_gripper(duration_sec)
            else:
                message = f'unknown gripper action: {action}'
                self.publish_status(
                    task_name,
                    STATUS_FAILED,
                    step_name,
                    False,
                    message
                )
                return ExecutionResult(False, STATUS_FAILED, message, step_name)
            return self._call_adapter(operation, task_name, step_name, timeout_sec)

        if step_type == 'wait':
            duration_sec = self._nonnegative_float(
                step.get('duration_sec'),
                self.gripper_wait_sec
            )
            return self._wait_step(task_name, step_name, duration_sec, timeout_sec)

        message = f'unknown step type: {step_type}'
        self.publish_status(task_name, STATUS_FAILED, step_name, False, message)
        return ExecutionResult(False, STATUS_FAILED, message, step_name)

    def _call_adapter(
        self,
        operation,
        task_name: str,
        step_name: str,
        timeout_sec: float
    ) -> ExecutionResult:
        result_holder = {}
        error_holder = {}
        done_event = threading.Event()

        def run_operation():
            try:
                result_holder['ok'] = bool(operation())
            except Exception as error:  # noqa: BLE001
                error_holder['error'] = error
            finally:
                done_event.set()

        thread = threading.Thread(target=run_operation, daemon=True)
        thread.start()
        deadline = time.monotonic() + timeout_sec

        while not done_event.is_set():
            if self._abort_event.is_set():
                self.adapter.stop()
                return self._finish_aborted(task_name, step_name)
            if time.monotonic() >= deadline:
                message = f'step timeout: {step_name}'
                self.adapter.stop()
                self.publish_status(
                    task_name,
                    STATUS_TIMEOUT,
                    step_name,
                    False,
                    message
                )
                self.get_logger().error(message)
                return ExecutionResult(False, STATUS_TIMEOUT, message, step_name)
            time.sleep(0.02)

        if 'error' in error_holder:
            message = f'adapter exception at {step_name}: {error_holder["error"]}'
            self.publish_status(task_name, STATUS_FAILED, step_name, False, message)
            return ExecutionResult(False, STATUS_FAILED, message, step_name)

        if not result_holder.get('ok', False):
            if self._abort_event.is_set():
                return self._finish_aborted(task_name, step_name)
            message = f'adapter failed at {step_name}'
            self.publish_status(task_name, STATUS_FAILED, step_name, False, message)
            return ExecutionResult(False, STATUS_FAILED, message, step_name)

        return ExecutionResult(True, STATUS_RUNNING, 'step completed', step_name)

    def _wait_step(
        self,
        task_name: str,
        step_name: str,
        duration_sec: float,
        timeout_sec: float
    ) -> ExecutionResult:
        if duration_sec > timeout_sec:
            message = f'step timeout: {step_name}'
            self.publish_status(task_name, STATUS_TIMEOUT, step_name, False, message)
            return ExecutionResult(False, STATUS_TIMEOUT, message, step_name)

        deadline = time.monotonic() + duration_sec
        while time.monotonic() < deadline:
            if self._abort_event.is_set():
                return self._finish_aborted(task_name, step_name)
            time.sleep(min(0.02, deadline - time.monotonic()))
        return ExecutionResult(True, STATUS_RUNNING, 'wait completed', step_name)

    def request_abort(self, reason: str) -> None:
        self._abort_event.set()
        self.get_logger().warn(reason)
        try:
            self.adapter.stop()
        except Exception as error:  # noqa: BLE001
            self.get_logger().error(f'adapter stop failed during abort: {error}')
        self.publish_status(
            self._active_task,
            STATUS_ABORTED,
            self._current_step,
            False,
            reason
        )
        self.publish_lock(False)

    def _finish_aborted(self, task_name: str, step_name: str) -> ExecutionResult:
        message = f'arm task aborted at {step_name}'
        self.adapter.stop()
        self.publish_status(task_name, STATUS_ABORTED, step_name, False, message)
        self.get_logger().warn(message)
        return ExecutionResult(False, STATUS_ABORTED, message, step_name)

    def _try_begin_task(self, task_name: str) -> bool:
        with self._state_lock:
            if self._active_task:
                return False
            self._active_task = task_name
            self._current_state = STATUS_RUNNING
            self._current_step = ''
            self._abort_event.clear()
            return True

    def _should_abort(self, goal_handle: Optional[Any]) -> bool:
        if self._abort_event.is_set():
            return True
        if goal_handle is not None and goal_handle.is_cancel_requested:
            self.request_abort('action goal cancel requested')
            return True
        return False

    def publish_action_feedback(
        self,
        goal_handle: Optional[Any],
        step_name: str,
        progress: float
    ) -> None:
        if goal_handle is None:
            return
        feedback = ExecuteArmTask.Feedback()
        feedback.current_step = step_name
        feedback.progress = float(max(0.0, min(1.0, progress)))
        goal_handle.publish_feedback(feedback)

    def publish_status(
        self,
        task_name: str,
        state: str,
        step: str,
        success: bool,
        message: str
    ) -> None:
        with self._state_lock:
            self._current_state = state
            self._current_step = step
        payload = {
            'task': task_name or '',
            'state': state,
            'step': step or '',
            'success': bool(success),
            'message': message,
        }
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        self.status_pub.publish(msg)

    def publish_lock(self, locked: bool) -> None:
        msg = Bool()
        msg.data = bool(locked)
        self.lock_pub.publish(msg)

    def normalize_task_name(self, task_name: str, target: str = '') -> str:
        candidate = self._normalize_key(task_name)
        if candidate in TASK_ALIASES:
            return TASK_ALIASES[candidate]

        target_candidate = self._normalize_key(target)
        if target_candidate in TASK_ALIASES:
            return TASK_ALIASES[target_candidate]
        return candidate

    def step_name(self, step: Dict[str, Any]) -> str:
        step_type = step.get('type')
        if step_type == 'move':
            pose = str(step.get('pose', '')).upper()
            if pose == 'HOME':
                return 'HOME'
            return f'MOVE_{pose}'
        if step_type == 'gripper':
            return f'{str(step.get("action", "")).upper()}_GRIPPER'
        if step_type == 'wait':
            return 'WAIT'
        return str(step_type or 'UNKNOWN').upper()

    def _step_timeout(self, step: Dict[str, Any]) -> float:
        timeout = step.get('timeout_sec', self.default_step_timeout_sec)
        return self._positive_float(timeout, self.default_step_timeout_sec)

    def _load_config(self) -> Dict[str, Any]:
        config_file = str(self.get_parameter('config_file').value or '').strip()
        if not config_file:
            config_file = self._default_config_file()

        path = Path(config_file).expanduser()
        if not path.exists():
            self.get_logger().warn(
                f'arm config file not found: {path}; using built-in defaults'
            )
            return {'arm': {}}

        with path.open('r', encoding='utf-8') as stream:
            data = yaml.safe_load(stream) or {}
        if not isinstance(data, dict):
            raise ValueError(f'arm config must be a YAML mapping: {path}')

        self.get_logger().info(f'Loaded arm config: {path}')
        return data

    def _default_config_file(self) -> str:
        try:
            share = get_package_share_directory('rk_arm_control')
            return str(Path(share) / 'config' / 'arm_poses.yaml')
        except PackageNotFoundError:
            source_path = Path(__file__).resolve().parents[1]
            return str(source_path / 'config' / 'arm_poses.yaml')

    def _positive_float(self, value: Any, default: float) -> float:
        try:
            number = float(value)
        except (TypeError, ValueError):
            return float(default)
        if number <= 0.0:
            return float(default)
        return number

    def _nonnegative_float(self, value: Any, default: float) -> float:
        try:
            number = float(value)
        except (TypeError, ValueError):
            return float(default)
        if number < 0.0:
            return float(default)
        return number

    def _normalize_key(self, value: str) -> str:
        return str(value or '').strip().upper().replace('-', '_').replace(' ', '_')


def main(args=None):
    rclpy.init(args=args)
    node = ArmTaskNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
