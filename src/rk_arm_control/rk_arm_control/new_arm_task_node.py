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

from rk_arm_control.adapters.dry_run_adapter import DryRunArmAdapter
from rk_arm_control.adapters.sdk_bridge_adapter import SdkBridgeArmAdapter
from rk_interfaces.action import ExecuteArmTask
from rk_interfaces.msg import ItemTagArray


STATUS_IDLE = 'IDLE'
STATUS_RUNNING = 'RUNNING'
STATUS_DONE = 'DONE'
STATUS_FAILED = 'FAILED'
STATUS_ABORTED = 'ABORTED'
STATUS_BUSY = 'BUSY'
STATUS_TIMEOUT = 'TIMEOUT'
STATUS_NO_TARGET = 'NO_TARGET'


@dataclass(frozen=True)
class ExecutionResult:
    success: bool
    state: str
    message: str
    step: str = ''


@dataclass(frozen=True)
class VisionTarget:
    """感知节点输出的可选目标。

    方案 A 默认不依赖视觉坐标；这里保留目标缓存，是为了后续从固定点位
    平滑升级到“相机确认物体存在”或“相机 XY 小范围修正”。
    """

    item_type: str
    confidence: float
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    source: str = ''
    received_monotonic: float = 0.0


class NewArmTaskNode(Node):
    """新机械臂固定点位任务节点。

    设计目标：
    - 上层接口保持 `/arm/command_json` 和 `/arm/execute_task`；
    - 抓取流程主要由 YAML 固定点位决定；
    - 识别结果只做可选确认，不让视觉失败卡死比赛主流程；
    - 底层通过 Adapter 替换成新机械臂 SDK Bridge。
    """

    def __init__(self):
        super().__init__('new_arm_task_node')
        self.callback_group = ReentrantCallbackGroup()
        self._declare_parameters()

        self._state_lock = threading.RLock()
        self._target_lock = threading.RLock()
        self._active_task = ''
        self._current_step = ''
        self._abort_event = threading.Event()
        self._latest_object_target: Optional[VisionTarget] = None
        self._latest_item_targets: List[VisionTarget] = []

        self.params_config = self._load_yaml_file(
            self._string_param('params_file'),
            self._default_config_path('new_arm_params.yaml')
        )
        self.poses_config = self._load_yaml_file(
            self._string_param('poses_file'),
            self._default_config_path('new_arm_poses.yaml')
        )

        self.arm_params = self.params_config.get('new_arm', {})
        self.arm_plan = self.poses_config.get('new_arm', {})
        self.task_aliases = self._load_task_aliases()
        self.tasks = self.arm_plan.get('tasks', {})
        self.poses = self.arm_plan.get('poses', {})
        self.gripper = self.arm_plan.get('gripper', {})
        self.timing = self.arm_plan.get('timing', {})
        self.perception = self.arm_params.get('perception', {})

        topics = self.arm_params.get('topics', {})
        self.status_topic = str(topics.get('status', '/arm/status'))
        self.control_lock_topic = str(
            topics.get('control_lock', '/arm/control_lock')
        )
        self.command_json_topic = str(
            topics.get('command_json', '/arm/command_json')
        )
        self.action_name = str(topics.get('action', '/arm/execute_task'))
        self.object_xy_json_topic = str(
            topics.get('object_xy_json', '/perception/object_xy_json')
        )
        self.item_tags_topic = str(
            topics.get('item_tags', '/perception/item_tags')
        )

        self.default_step_timeout_sec = self._positive_float(
            self.timing.get('default_step_timeout_sec'),
            5.0
        )
        self.default_move_duration_sec = self._positive_float(
            self.timing.get('default_move_duration_sec'),
            1.0
        )
        self.default_wait_sec = self._nonnegative_float(
            self.timing.get('default_wait_sec'),
            0.3
        )
        self.target_timeout_sec = self._positive_float(
            self.perception.get('target_timeout_sec'),
            3.0
        )
        self.min_confidence = self._nonnegative_float(
            self.perception.get('min_confidence'),
            0.5
        )

        self.status_pub = self.create_publisher(
            String,
            self.status_topic,
            10
        )
        self.lock_pub = self.create_publisher(
            Bool,
            self.control_lock_topic,
            10
        )
        self.command_sub = self.create_subscription(
            String,
            self.command_json_topic,
            self._on_command_json,
            10,
            callback_group=self.callback_group
        )
        self.object_sub = self.create_subscription(
            String,
            self.object_xy_json_topic,
            self._on_object_xy_json,
            10,
            callback_group=self.callback_group
        )
        self.item_tags_sub = self.create_subscription(
            ItemTagArray,
            self.item_tags_topic,
            self._on_item_tags,
            10,
            callback_group=self.callback_group
        )

        self.adapter = self._create_adapter()
        if not self.adapter.initialize():
            self.get_logger().error(
                'new arm adapter initialize failed; node stays alive.'
            )

        self.action_server = ActionServer(
            self,
            ExecuteArmTask,
            self.action_name,
            self.execute_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group
        )

        self.publish_lock(False)
        self.publish_status(
            '',
            STATUS_IDLE,
            '',
            True,
            'new arm task node ready'
        )
        self.get_logger().info(
            'New arm task node ready: '
            f'action={self.action_name}, command_json={self.command_json_topic}'
        )

    def _declare_parameters(self):
        self.declare_parameter('params_file', '')
        self.declare_parameter('poses_file', '')

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

    def _on_command_json(self, msg: String):
        try:
            payload = json.loads(msg.data)
            if not isinstance(payload, dict):
                raise ValueError('/arm/command_json must be a JSON object')
        except (TypeError, ValueError) as error:
            self.publish_status(
                '',
                STATUS_FAILED,
                '',
                False,
                f'invalid command json: {error}'
            )
            return

        raw_task = str(payload.get('task', '')).strip()
        target = str(payload.get('target', '')).strip()
        task_name = self.normalize_task_name(raw_task, target)

        if task_name == 'ABORT':
            self.request_abort('ABORT command requested')
            return

        worker = threading.Thread(
            target=self.execute_task_sequence,
            args=(task_name, raw_task, target, None),
            daemon=True
        )
        worker.start()

    def _on_object_xy_json(self, msg: String):
        try:
            payload = json.loads(msg.data)
            if not isinstance(payload, dict):
                raise ValueError('object_xy_json must be a JSON object')
            confidence = float(payload.get('confidence', 1.0))
            target = VisionTarget(
                item_type=str(payload.get('item_type', 'object')),
                confidence=confidence,
                x=float(payload.get('x', 0.0)),
                y=float(payload.get('y', 0.0)),
                z=float(payload.get('z', 0.0)),
                source='object_xy_json',
                received_monotonic=time.monotonic(),
            )
        except (TypeError, ValueError) as error:
            self.get_logger().warn(
                f'ignore invalid /perception/object_xy_json: {error}'
            )
            return

        with self._target_lock:
            self._latest_object_target = target

    def _on_item_tags(self, msg: ItemTagArray):
        now = time.monotonic()
        targets = []
        for tag in msg.tags:
            targets.append(VisionTarget(
                item_type=str(tag.item_type),
                confidence=float(tag.confidence),
                x=float(tag.pose.position.x),
                y=float(tag.pose.position.y),
                z=float(tag.pose.position.z),
                source='item_tags',
                received_monotonic=now,
            ))
        with self._target_lock:
            self._latest_item_targets = targets

    def execute_task_sequence(
        self,
        task_name: str,
        raw_task: str = '',
        target: str = '',
        goal_handle: Optional[Any] = None
    ) -> ExecutionResult:
        task_config = self.tasks.get(task_name)
        if not isinstance(task_config, dict):
            message = f'unknown new arm task: {raw_task or task_name}'
            self.publish_status(task_name, STATUS_FAILED, '', False, message)
            return ExecutionResult(False, STATUS_FAILED, message)

        if not self._try_begin_task(task_name):
            message = f'new arm busy: active={self._active_task}'
            self.publish_status(task_name, STATUS_BUSY, '', False, message)
            return ExecutionResult(False, STATUS_BUSY, message)

        steps = list(task_config.get('steps', []))
        last_step = ''
        try:
            self.publish_lock(True)
            self.publish_status(task_name, STATUS_RUNNING, '', True, 'start')
            for index, step in enumerate(steps):
                last_step = self._step_name(step)
                self._current_step = last_step
                if self._should_abort(goal_handle):
                    return self._finish_aborted(task_name, last_step)

                self._publish_action_feedback(
                    goal_handle,
                    last_step,
                    index / float(max(1, len(steps)))
                )
                self.publish_status(
                    task_name,
                    STATUS_RUNNING,
                    last_step,
                    True,
                    'step started'
                )
                result = self._execute_step(step, task_name, last_step)
                if not result.success:
                    return result

                self._publish_action_feedback(
                    goal_handle,
                    last_step,
                    (index + 1) / float(max(1, len(steps)))
                )
                self.publish_status(
                    task_name,
                    STATUS_RUNNING,
                    last_step,
                    True,
                    'step completed'
                )

            message = 'task completed'
            self.publish_status(task_name, STATUS_DONE, last_step, True, message)
            return ExecutionResult(True, STATUS_DONE, message, last_step)
        except Exception as error:  # noqa: BLE001 - 保持 ROS 节点不崩。
            message = f'new arm task exception: {error}'
            self.get_logger().exception(message)
            self.adapter.stop()
            self.publish_status(task_name, STATUS_FAILED, last_step, False, message)
            return ExecutionResult(False, STATUS_FAILED, message, last_step)
        finally:
            self.publish_lock(False)
            with self._state_lock:
                self._active_task = ''
                self._current_step = ''
                self._abort_event.clear()

    def _execute_step(
        self,
        step: Dict[str, Any],
        task_name: str,
        step_name: str,
    ) -> ExecutionResult:
        step_type = str(step.get('type', '')).lower()
        timeout_sec = self._positive_float(
            step.get('timeout_sec'),
            self.default_step_timeout_sec
        )

        if step_type == 'check_target':
            return self._check_target_step(step, task_name, step_name)

        if step_type == 'move':
            pose_name = str(step.get('pose', '')).upper()
            pose = self.poses.get(pose_name)
            if not isinstance(pose, dict):
                message = f'missing pose in new_arm_poses.yaml: {pose_name}'
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
                lambda: self.adapter.move_joints(
                    joints,
                    duration_sec,
                    pose_name
                ),
                task_name,
                step_name,
                timeout_sec
            )

        if step_type == 'gripper':
            action = str(step.get('action', '')).lower()
            duration_sec = self._positive_float(
                self.gripper.get(action, {}).get('duration_sec'),
                self.default_wait_sec
            )
            if action == 'open':
                operation = lambda: self.adapter.open_gripper(duration_sec)
            elif action == 'close':
                operation = lambda: self.adapter.close_gripper(duration_sec)
            else:
                message = f'unknown gripper action: {action}'
                return ExecutionResult(False, STATUS_FAILED, message, step_name)
            return self._call_adapter(operation, task_name, step_name, timeout_sec)

        if step_type == 'wait':
            duration_sec = self._nonnegative_float(
                step.get('duration_sec'),
                self.default_wait_sec
            )
            return self._wait_step(task_name, step_name, duration_sec, timeout_sec)

        message = f'unknown step type: {step_type}'
        self.publish_status(task_name, STATUS_FAILED, step_name, False, message)
        return ExecutionResult(False, STATUS_FAILED, message, step_name)

    def _check_target_step(
        self,
        step: Dict[str, Any],
        task_name: str,
        step_name: str,
    ) -> ExecutionResult:
        # 视觉确认默认是“软确认”：失败只 WARN 后继续，避免比赛流程卡死。
        required = bool(step.get('required', False))
        accepted = [str(value) for value in step.get('accepted_item_types', [])]
        timeout_sec = self._positive_float(
            step.get('timeout_sec'),
            self.target_timeout_sec
        )
        target = self._wait_for_target(accepted, timeout_sec)
        if target is not None:
            message = (
                f'target confirmed: type={target.item_type}, '
                f'confidence={target.confidence:.2f}, source={target.source}'
            )
            self.publish_status(task_name, STATUS_RUNNING, step_name, True, message)
            return ExecutionResult(True, STATUS_RUNNING, message, step_name)

        message = f'no target confirmed for {accepted or ["any"]}'
        if required:
            self.publish_status(task_name, STATUS_NO_TARGET, step_name, False, message)
            return ExecutionResult(False, STATUS_NO_TARGET, message, step_name)

        self.get_logger().warn(message + '; continue fixed-pose fallback')
        self.publish_status(task_name, STATUS_RUNNING, step_name, True, message)
        return ExecutionResult(True, STATUS_RUNNING, message, step_name)

    def _wait_for_target(
        self,
        accepted_item_types: List[str],
        timeout_sec: float,
    ) -> Optional[VisionTarget]:
        deadline = time.monotonic() + timeout_sec
        accepted = {value.strip().lower() for value in accepted_item_types}
        while time.monotonic() < deadline:
            if self._abort_event.is_set():
                return None
            target = self._latest_matching_target(accepted)
            if target is not None:
                return target
            time.sleep(0.05)
        return None

    def _latest_matching_target(self, accepted: set) -> Optional[VisionTarget]:
        now = time.monotonic()
        candidates: List[VisionTarget] = []
        with self._target_lock:
            if self._latest_object_target is not None:
                candidates.append(self._latest_object_target)
            candidates.extend(self._latest_item_targets)

        candidates.sort(key=lambda item: item.received_monotonic, reverse=True)
        for target in candidates:
            if now - target.received_monotonic > self.target_timeout_sec:
                continue
            if target.confidence < self.min_confidence:
                continue
            if accepted and target.item_type.lower() not in accepted:
                continue
            return target
        return None

    def _call_adapter(
        self,
        operation,
        task_name: str,
        step_name: str,
        timeout_sec: float,
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
                return ExecutionResult(False, STATUS_TIMEOUT, message, step_name)
            time.sleep(0.02)

        if 'error' in error_holder:
            message = f'adapter exception: {error_holder["error"]}'
            self.publish_status(task_name, STATUS_FAILED, step_name, False, message)
            return ExecutionResult(False, STATUS_FAILED, message, step_name)

        if not result_holder.get('ok', False):
            message = f'adapter failed at {step_name}'
            self.publish_status(task_name, STATUS_FAILED, step_name, False, message)
            return ExecutionResult(False, STATUS_FAILED, message, step_name)

        return ExecutionResult(True, STATUS_RUNNING, 'step completed', step_name)

    def _wait_step(
        self,
        task_name: str,
        step_name: str,
        duration_sec: float,
        timeout_sec: float,
    ) -> ExecutionResult:
        if duration_sec > timeout_sec:
            message = f'wait step timeout: {step_name}'
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
        self.adapter.stop()
        self.publish_status(
            self._active_task,
            STATUS_ABORTED,
            self._current_step,
            False,
            reason
        )
        self.publish_lock(False)

    def _finish_aborted(self, task_name: str, step_name: str) -> ExecutionResult:
        message = f'new arm task aborted at {step_name}'
        self.adapter.stop()
        self.publish_status(task_name, STATUS_ABORTED, step_name, False, message)
        return ExecutionResult(False, STATUS_ABORTED, message, step_name)

    def _try_begin_task(self, task_name: str) -> bool:
        with self._state_lock:
            if self._active_task:
                return False
            self._active_task = task_name
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

    def normalize_task_name(self, raw_task: str, target: str = '') -> str:
        normalized = str(raw_task or '').strip().upper()
        normalized = normalized.replace('-', '_').replace(' ', '_')
        while '__' in normalized:
            normalized = normalized.replace('__', '_')

        alias_key = normalized
        if normalized in ('PLACE_FIELD_ITEM', 'PLACE_TARGET') and target:
            alias_key = str(target).strip().upper().replace('-', '_')
        return self.task_aliases.get(alias_key, normalized)

    def publish_status(
        self,
        task: str,
        state: str,
        step: str,
        success: bool,
        message: str,
    ) -> None:
        payload = {
            'task': task,
            'state': state,
            'step': step,
            'success': bool(success),
            'message': message,
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False, separators=(',', ':'))
        self.status_pub.publish(msg)

    def publish_lock(self, locked: bool) -> None:
        msg = Bool()
        msg.data = bool(locked)
        self.lock_pub.publish(msg)

    def _publish_action_feedback(self, goal_handle, step: str, progress: float):
        if goal_handle is None:
            return
        feedback = ExecuteArmTask.Feedback()
        feedback.current_step = step
        feedback.progress = float(max(0.0, min(1.0, progress)))
        goal_handle.publish_feedback(feedback)

    def _step_name(self, step: Dict[str, Any]) -> str:
        step_type = str(step.get('type', '')).upper()
        if step_type == 'MOVE':
            return f'MOVE_{str(step.get("pose", "")).upper()}'
        if step_type == 'GRIPPER':
            return f'GRIPPER_{str(step.get("action", "")).upper()}'
        if step_type == 'CHECK_TARGET':
            return 'CHECK_TARGET'
        if step_type == 'WAIT':
            return 'WAIT'
        return step_type or 'UNKNOWN_STEP'

    def _create_adapter(self):
        adapter_config = self.arm_params.get('adapter', {})
        mode = str(adapter_config.get('mode', 'dry_run')).strip().lower()
        if mode == 'sdk_bridge':
            return SdkBridgeArmAdapter(
                self,
                adapter_config.get('sdk_bridge', {})
            )
        if mode != 'dry_run':
            self.get_logger().warn(
                f'unknown new arm adapter mode={mode}; fallback to dry_run'
            )
        return DryRunArmAdapter(self)

    def _load_task_aliases(self) -> Dict[str, str]:
        default_aliases = {
            'PICK_START': 'PICK_START',
            'PICK_START_ITEM': 'PICK_START',
            'START_ITEM': 'PICK_START',
            'PLACE_TRANSFER': 'PLACE_TRANSFER',
            'DROP_START_ITEM': 'PLACE_TRANSFER',
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
        configured = self.arm_plan.get('task_aliases', {})
        for key, value in configured.items():
            default_aliases[str(key).upper()] = str(value).upper()
        return default_aliases

    def _load_yaml_file(self, raw_path: str, fallback_path: Path) -> Dict:
        path = Path(raw_path).expanduser() if raw_path else fallback_path
        with path.open('r', encoding='utf-8') as stream:
            data = yaml.safe_load(stream) or {}
        if not isinstance(data, dict):
            raise ValueError(f'YAML root must be a dict: {path}')
        self.get_logger().info(f'loaded new arm config: {path}')
        return data

    def _default_config_path(self, filename: str) -> Path:
        try:
            share = Path(get_package_share_directory('rk_arm_control'))
            return share / 'config' / filename
        except PackageNotFoundError:
            return Path.cwd() / 'src' / 'rk_arm_control' / 'config' / filename

    def _string_param(self, name: str) -> str:
        return str(self.get_parameter(name).value or '')

    @staticmethod
    def _positive_float(value: Any, fallback: float) -> float:
        try:
            result = float(value)
        except (TypeError, ValueError):
            return float(fallback)
        if result <= 0.0:
            return float(fallback)
        return result

    @staticmethod
    def _nonnegative_float(value: Any, fallback: float) -> float:
        try:
            result = float(value)
        except (TypeError, ValueError):
            return float(fallback)
        if result < 0.0:
            return float(fallback)
        return result

    def destroy_node(self):
        try:
            self.adapter.shutdown()
        except Exception as error:  # noqa: BLE001
            self.get_logger().warn(f'new arm adapter shutdown failed: {error}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = NewArmTaskNode()
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
