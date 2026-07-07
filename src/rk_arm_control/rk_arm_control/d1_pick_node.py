#!/usr/bin/env python3

import json
import math
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import rclpy
from ament_index_python.packages import PackageNotFoundError
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PointStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, String


STATUS_IDLE = 'IDLE'
STATUS_RUNNING = 'RUNNING'
STATUS_DONE = 'DONE'
STATUS_FAILED = 'FAILED'
STATUS_ABORTED = 'ABORTED'
STATUS_BUSY = 'BUSY'
STATUS_TIMEOUT = 'TIMEOUT'
STATUS_NO_TARGET = 'NO_TARGET'
STATUS_OUT_OF_WORKSPACE = 'OUT_OF_WORKSPACE'

PICK_TASKS = {'PICK_BY_CAMERA', 'PICK_START'}
SUPPORTED_TASKS = PICK_TASKS | {
    'PLACE_TRANSFER',
    'HOME',
    'OPEN_GRIPPER',
    'CLOSE_GRIPPER',
    'ABORT',
}


@dataclass
class CameraTarget:
    x: float
    y: float
    confidence: float
    frame_id: str
    source_stamp: float
    received_monotonic: float
    source: str


@dataclass
class TaskResult:
    success: bool
    state: str
    message: str
    step: str = ''
    target_camera_xy: Optional[Tuple[float, float]] = None
    target_arm_xyz: Optional[Tuple[float, float, float]] = None


class ArmHardwareAdapter:
    """D1 arm backend boundary."""

    def initialize(self) -> bool:
        raise NotImplementedError

    def move_cartesian_xyzrpy(
        self,
        x: float,
        y: float,
        z: float,
        roll: float,
        pitch: float,
        yaw: float,
        duration_sec: float,
    ) -> bool:
        raise NotImplementedError

    def move_home(self) -> bool:
        raise NotImplementedError

    def open_gripper(self, duration_sec: float) -> bool:
        raise NotImplementedError

    def close_gripper(self, duration_sec: float) -> bool:
        raise NotImplementedError

    def stop(self) -> None:
        raise NotImplementedError

    def shutdown(self) -> None:
        raise NotImplementedError


class DryRunD1ArmAdapter(ArmHardwareAdapter):
    """Dry-run adapter used for field workflow testing without D1 hardware."""

    def __init__(self, node: Node, home_pose: List[float]):
        self._node = node
        self._logger = node.get_logger()
        self._home_pose = home_pose
        self._stop_event = threading.Event()

    def initialize(self) -> bool:
        self._logger.warn(
            'D1 arm dry_run=true: no hardware command will be sent.'
        )
        return True

    def move_cartesian_xyzrpy(
        self,
        x: float,
        y: float,
        z: float,
        roll: float,
        pitch: float,
        yaw: float,
        duration_sec: float,
    ) -> bool:
        self._stop_event.clear()
        self._logger.info(
            '[DRY_RUN] move_cartesian_xyzrpy '
            f'xyz=({x:.3f}, {y:.3f}, {z:.3f}) '
            f'rpy=({roll:.3f}, {pitch:.3f}, {yaw:.3f}) '
            f'duration_sec={duration_sec:.2f}'
        )
        return not self._sleep(duration_sec)

    def move_home(self) -> bool:
        return self.move_cartesian_xyzrpy(*self._home_pose, duration_sec=1.0)

    def open_gripper(self, duration_sec: float) -> bool:
        self._stop_event.clear()
        self._logger.info(
            f'[DRY_RUN] open_gripper duration_sec={duration_sec:.2f}'
        )
        return not self._sleep(duration_sec)

    def close_gripper(self, duration_sec: float) -> bool:
        self._stop_event.clear()
        self._logger.info(
            f'[DRY_RUN] close_gripper duration_sec={duration_sec:.2f}'
        )
        return not self._sleep(duration_sec)

    def stop(self) -> None:
        self._stop_event.set()
        self._logger.warn('[DRY_RUN] D1 arm stop requested')

    def shutdown(self) -> None:
        self.stop()

    def _sleep(self, duration_sec: float) -> bool:
        deadline = time.monotonic() + max(0.0, float(duration_sec))
        while time.monotonic() < deadline:
            if self._stop_event.is_set():
                return True
            time.sleep(min(0.02, deadline - time.monotonic()))
        return self._stop_event.is_set()


class BasicPoseTableAdapter:
    """Placeholder XYZ-to-joints mapper before IK is wired."""

    def __init__(self, node: Node, calibrated_points_json: str,
                 max_nearest_distance_m: float):
        self._logger = node.get_logger()
        self._max_nearest_distance_m = max(0.0, max_nearest_distance_m)
        self._samples = self._parse_samples(calibrated_points_json)

    def xyzrpy_to_joints(
        self,
        x: float,
        y: float,
        z: float,
        roll: float,
        pitch: float,
        yaw: float,
    ) -> Optional[List[float]]:
        if not self._samples:
            self._logger.error(
                'Unitree D1 pose-table fallback is not calibrated. '
                'Fill pose_table.calibrated_points_json with measured '
                'xyz/joints samples or provide a real IK/Cartesian SDK API.'
            )
            return None

        best_sample = None
        best_dist = float('inf')
        for sample in self._samples:
            sx, sy, sz = sample['xyz']
            dist = math.sqrt((x - sx) ** 2 + (y - sy) ** 2 + (z - sz) ** 2)
            if dist < best_dist:
                best_dist = dist
                best_sample = sample

        if best_sample is None or best_dist > self._max_nearest_distance_m:
            self._logger.error(
                'No calibrated D1 pose-table sample near '
                f'xyz=({x:.3f}, {y:.3f}, {z:.3f}); '
                f'nearest_dist={best_dist:.3f}m'
            )
            return None

        self._logger.warn(
            'Using nearest calibrated D1 pose-table sample. '
            'TODO: replace with real inverse kinematics or interpolation.'
        )
        return list(best_sample['joints'])

    def _parse_samples(self, raw_json: str) -> List[Dict[str, List[float]]]:
        if not raw_json:
            return []
        try:
            data = json.loads(raw_json)
        except (TypeError, ValueError) as error:
            self._logger.error(
                f'invalid pose_table.calibrated_points_json: {error}'
            )
            return []

        if not isinstance(data, list):
            self._logger.error(
                'pose_table.calibrated_points_json must be a list'
            )
            return []

        samples = []
        for item in data:
            if not isinstance(item, dict):
                continue
            xyz = item.get('xyz')
            joints = item.get('joints')
            if (
                isinstance(xyz, list) and len(xyz) >= 3 and
                isinstance(joints, list) and len(joints) >= 6
            ):
                samples.append({
                    'xyz': [float(xyz[0]), float(xyz[1]), float(xyz[2])],
                    'joints': [float(value) for value in joints],
                })
        return samples


class UnitreeD1SdkAdapter(ArmHardwareAdapter):
    """Adapter for the imported Unitree D1 SDK snapshot.

    The local SDK examples publish C++ DDS JSON commands to rt/arm_Command and
    do not expose a Python Cartesian API. This adapter therefore fails
    gracefully until a real callable API or calibrated pose table is provided.
    """

    def __init__(
        self,
        node: Node,
        sdk_root: str,
        home_pose: List[float],
        pose_table: BasicPoseTableAdapter,
    ):
        self._node = node
        self._logger = node.get_logger()
        self._sdk_root = Path(sdk_root).expanduser() if sdk_root else Path()
        self._home_pose = home_pose
        self._pose_table = pose_table
        self._stop_event = threading.Event()
        self._initialized = False

    def initialize(self) -> bool:
        if not self._sdk_root or not self._sdk_root.exists():
            self._logger.error(
                f'Unitree D1 SDK root not found: {self._sdk_root}. '
                'Set d1_sdk_root or use dry_run=true.'
            )
            return False

        examples = [
            self._sdk_root / 'src' / 'joint_angle_control.cpp',
            self._sdk_root / 'src' / 'multiple_joint_angle_control.cpp',
            self._sdk_root / 'src' / 'arm_zero_control.cpp',
            self._sdk_root / 'src' / 'joint_enable_control.cpp',
        ]
        found = [str(path) for path in examples if path.exists()]
        self._logger.warn(
            'Unitree D1 SDK found, but this snapshot exposes C++ DDS examples '
            'instead of a callable Python Cartesian API. Confirmed examples: '
            f'{found}. Hardware commands will fail gracefully until the real '
            'D1 API or a calibrated joint pose table is wired here.'
        )
        self._initialized = True
        return True

    def move_cartesian_xyzrpy(
        self,
        x: float,
        y: float,
        z: float,
        roll: float,
        pitch: float,
        yaw: float,
        duration_sec: float,
    ) -> bool:
        self._stop_event.clear()
        joints = self._pose_table.xyzrpy_to_joints(x, y, z, roll, pitch, yaw)
        if joints is None:
            return False
        return self._send_joint_pose(joints, duration_sec)

    def move_home(self) -> bool:
        return self.move_cartesian_xyzrpy(*self._home_pose, duration_sec=1.0)

    def open_gripper(self, duration_sec: float) -> bool:
        self._logger.error(
            'TODO: wire Unitree D1 gripper open command in '
            'UnitreeD1SdkAdapter. '
            'The local examples do not identify a safe Python gripper API.'
        )
        self._sleep(duration_sec)
        return False

    def close_gripper(self, duration_sec: float) -> bool:
        self._logger.error(
            'TODO: wire Unitree D1 gripper close command in '
            'UnitreeD1SdkAdapter. '
            'The local examples do not identify a safe Python gripper API.'
        )
        self._sleep(duration_sec)
        return False

    def stop(self) -> None:
        self._stop_event.set()
        self._logger.warn(
            'Unitree D1 stop requested. TODO: wire a real emergency/stop '
            'command when the SDK stop API is confirmed.'
        )

    def shutdown(self) -> None:
        self.stop()

    def _send_joint_pose(
        self,
        joints: List[float],
        duration_sec: float,
    ) -> bool:
        self._logger.error(
            'TODO: publish Unitree D1 joint command via rt/arm_Command '
            'or call the confirmed SDK function. Candidate examples: '
            'third_party/unitree_d1_sdk/src/multiple_joint_angle_control.cpp '
            'and joint_angle_control.cpp. '
            f'Requested joints={joints}, duration_sec={duration_sec:.2f}'
        )
        self._sleep(duration_sec)
        return False

    def _sleep(self, duration_sec: float) -> bool:
        deadline = time.monotonic() + max(0.0, float(duration_sec))
        while time.monotonic() < deadline:
            if self._stop_event.is_set():
                return True
            time.sleep(min(0.02, deadline - time.monotonic()))
        return self._stop_event.is_set()


class D1PickNode(Node):
    """Camera-XY based fixed-platform D1 pick/place executor."""

    def __init__(self):
        super().__init__('d1_arm')
        self.callback_group = ReentrantCallbackGroup()

        self._declare_parameters()
        self._state_lock = threading.RLock()
        self._target_lock = threading.RLock()
        self._active_task = ''
        self._abort_event = threading.Event()
        self._latest_json_target = None
        self._latest_point_target = None

        self.command_topic = self._string_param('command_topic')
        self.object_xy_json_topic = self._string_param('object_xy_json_topic')
        self.object_xy_point_topic = self._string_param(
            'object_xy_point_topic'
        )
        self.status_topic = self._string_param('status_topic')
        self.control_lock_topic = self._string_param('control_lock_topic')

        self.target_timeout_sec = self._positive_float_param(
            'target_timeout_sec', 3.0
        )
        self.min_confidence = self._nonnegative_float_param(
            'min_confidence', 0.5
        )
        self.task_timeout_sec = self._positive_float_param(
            'task_timeout_sec', 12.0
        )
        self.dry_run = self._bool_param('dry_run', True)

        self.home_pose = self._pose_param(
            'poses.home_xyzrpy', [0.0, 0.0, 0.22, 0.0, 0.0, 0.0]
        )
        self.transfer_place_pose = self._pose_param(
            'poses.transfer_place_xyzrpy',
            [0.18, 0.10, 0.08, 0.0, 0.0, 0.0],
        )
        self.safe_lift_pose = self._pose_param(
            'poses.safe_lift_xyzrpy', [0.0, 0.0, 0.24, 0.0, 0.0, 0.0]
        )

        self.status_pub = self.create_publisher(
            String, self.status_topic, 10
        )
        self.lock_pub = self.create_publisher(
            Bool, self.control_lock_topic, 10
        )
        self.command_sub = self.create_subscription(
            String,
            self.command_topic,
            self._on_command_json,
            10,
            callback_group=self.callback_group,
        )
        self.object_json_sub = self.create_subscription(
            String,
            self.object_xy_json_topic,
            self._on_object_xy_json,
            10,
            callback_group=self.callback_group,
        )
        self.object_point_sub = self.create_subscription(
            PointStamped,
            self.object_xy_point_topic,
            self._on_object_xy_point,
            10,
            callback_group=self.callback_group,
        )

        self.adapter = self._create_adapter()
        if not self.adapter.initialize():
            self.get_logger().error(
                'D1 arm adapter initialization failed; node remains alive and '
                'will report FAILED for hardware tasks.'
            )

        self._publish_lock(False)
        self._publish_status(
            '',
            STATUS_IDLE,
            '',
            True,
            'd1 pick node ready',
        )
        self.get_logger().info(
            'D1 pick node ready: command_topic='
            f'{self.command_topic}, object_xy_json_topic='
            f'{self.object_xy_json_topic}, object_xy_point_topic='
            f'{self.object_xy_point_topic}, dry_run={self.dry_run}'
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter('command_topic', '/arm/command_json')
        self.declare_parameter(
            'object_xy_json_topic',
            '/perception/object_xy_json',
        )
        self.declare_parameter(
            'object_xy_point_topic',
            '/perception/object_xy',
        )
        self.declare_parameter('status_topic', '/arm/status')
        self.declare_parameter('control_lock_topic', '/arm/control_lock')
        self.declare_parameter('target_timeout_sec', 3.0)
        self.declare_parameter('min_confidence', 0.50)
        self.declare_parameter('task_timeout_sec', 12.0)
        self.declare_parameter('dry_run', True)
        self.declare_parameter('d1_sdk_root', self._default_d1_sdk_root())

        self.declare_parameter('camera_to_arm.a11', 1.0)
        self.declare_parameter('camera_to_arm.a12', 0.0)
        self.declare_parameter('camera_to_arm.a21', 0.0)
        self.declare_parameter('camera_to_arm.a22', 1.0)
        self.declare_parameter('camera_to_arm.tx', 0.0)
        self.declare_parameter('camera_to_arm.ty', 0.0)

        self.declare_parameter('workspace.x_min', -0.30)
        self.declare_parameter('workspace.x_max', 0.30)
        self.declare_parameter('workspace.y_min', -0.25)
        self.declare_parameter('workspace.y_max', 0.25)
        self.declare_parameter('workspace.z_min', 0.02)
        self.declare_parameter('workspace.z_max', 0.35)

        self.declare_parameter('fixed_heights.pre_pick_z', 0.16)
        self.declare_parameter('fixed_heights.pick_z', 0.06)
        self.declare_parameter('fixed_heights.lift_z', 0.22)
        self.declare_parameter('fixed_heights.place_z', 0.08)
        self.declare_parameter('fixed_heights.pre_place_z', 0.18)

        self.declare_parameter('timing.move_duration_sec', 1.0)
        self.declare_parameter('timing.gripper_wait_sec', 0.5)
        self.declare_parameter('timing.settle_wait_sec', 0.3)

        self.declare_parameter('gripper.open_value', 1.0)
        self.declare_parameter('gripper.close_value', 0.0)
        self.declare_parameter('gripper.open_duration_sec', 0.5)
        self.declare_parameter('gripper.close_duration_sec', 0.5)

        self.declare_parameter(
            'poses.home_xyzrpy', [0.0, 0.0, 0.22, 0.0, 0.0, 0.0]
        )
        self.declare_parameter(
            'poses.transfer_place_xyzrpy',
            [0.18, 0.10, 0.08, 0.0, 0.0, 0.0],
        )
        self.declare_parameter(
            'poses.safe_lift_xyzrpy', [0.0, 0.0, 0.24, 0.0, 0.0, 0.0]
        )
        self.declare_parameter('pose_table.calibrated_points_json', '[]')
        self.declare_parameter('pose_table.max_nearest_distance_m', 0.05)

    def _on_command_json(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            if not isinstance(payload, dict):
                raise ValueError('/arm/command_json must be a JSON object')
            task = str(payload.get('task', '')).strip().upper()
        except (TypeError, ValueError) as error:
            self.get_logger().error(f'invalid /arm/command_json: {error}')
            self._publish_status(
                '',
                STATUS_FAILED,
                'PARSE_COMMAND',
                False,
                f'invalid command json: {error}',
            )
            return

        if task == 'ABORT':
            self._handle_abort()
            return

        if task not in SUPPORTED_TASKS:
            self._publish_status(
                task,
                STATUS_FAILED,
                'VALIDATE_COMMAND',
                False,
                f'unknown arm task: {task}',
            )
            return

        with self._state_lock:
            if self._active_task:
                self._publish_status(
                    task,
                    STATUS_BUSY,
                    'BUSY',
                    False,
                    f'arm task busy: active={self._active_task}',
                )
                return
            self._active_task = task
            self._abort_event.clear()

        worker = threading.Thread(
            target=self._run_task_thread,
            args=(task,),
            daemon=True,
        )
        worker.start()

    def _on_object_xy_json(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            if not isinstance(payload, dict):
                raise ValueError('object_xy_json must be a JSON object')
            x = float(payload['x'])
            y = float(payload['y'])
            confidence = float(payload.get('confidence', 1.0))
            frame_id = str(payload.get('frame_id', ''))
            stamp = float(payload.get('stamp', time.time()))
            self._validate_finite([x, y, confidence, stamp], 'object_xy_json')
        except (KeyError, TypeError, ValueError) as error:
            self.get_logger().warn(
                f'ignore invalid /perception/object_xy_json: {error}'
            )
            return

        target = CameraTarget(
            x=x,
            y=y,
            confidence=confidence,
            frame_id=frame_id,
            source_stamp=stamp,
            received_monotonic=time.monotonic(),
            source='json',
        )
        with self._target_lock:
            self._latest_json_target = target

    def _on_object_xy_point(self, msg: PointStamped) -> None:
        try:
            x = float(msg.point.x)
            y = float(msg.point.y)
            self._validate_finite([x, y], 'object_xy_point')
        except ValueError as error:
            self.get_logger().warn(
                f'ignore invalid /perception/object_xy: {error}'
            )
            return

        stamp = 0.0
        if msg.header.stamp.sec or msg.header.stamp.nanosec:
            stamp = (
                float(msg.header.stamp.sec) +
                float(msg.header.stamp.nanosec) / 1000000000.0
            )
        else:
            stamp = time.time()

        target = CameraTarget(
            x=x,
            y=y,
            confidence=1.0,
            frame_id=str(msg.header.frame_id),
            source_stamp=stamp,
            received_monotonic=time.monotonic(),
            source='point',
        )
        with self._target_lock:
            self._latest_point_target = target

    def _run_task_thread(self, task: str) -> None:
        result = TaskResult(False, STATUS_FAILED, 'task did not run')
        self._publish_lock(True)
        start_time = time.monotonic()
        try:
            if task in PICK_TASKS:
                result = self._execute_pick_by_camera(task, start_time)
            elif task == 'PLACE_TRANSFER':
                result = self._execute_place_transfer(task, start_time)
            elif task == 'HOME':
                result = self._execute_home(task, start_time)
            elif task == 'OPEN_GRIPPER':
                result = self._execute_open_gripper(task, start_time)
            elif task == 'CLOSE_GRIPPER':
                result = self._execute_close_gripper(task, start_time)
        except Exception as error:  # noqa: BLE001 - keep ROS node alive.
            self.get_logger().exception(f'D1 arm task exception: {error}')
            self.adapter.stop()
            result = TaskResult(
                False,
                STATUS_FAILED,
                f'arm task exception: {error}',
                step='EXCEPTION',
            )

        if self._abort_event.is_set() and result.state != STATUS_ABORTED:
            result = TaskResult(
                False,
                STATUS_ABORTED,
                'arm task aborted',
                step=result.step or 'ABORT',
                target_camera_xy=result.target_camera_xy,
                target_arm_xyz=result.target_arm_xyz,
            )

        self._publish_status(
            task,
            result.state,
            result.step,
            result.success,
            result.message,
            result.target_camera_xy,
            result.target_arm_xyz,
        )
        self._finish_task()

    def _execute_pick_by_camera(
        self,
        task: str,
        start_time: float,
    ) -> TaskResult:
        self._publish_status(
            task,
            STATUS_RUNNING,
            'WAIT_TARGET',
            True,
            'waiting for recent camera target',
        )
        target = self._wait_for_recent_target(start_time)
        if target is None:
            return TaskResult(
                False,
                STATUS_NO_TARGET,
                'no recent camera target',
                step='WAIT_TARGET',
            )

        camera_xy = (target.x, target.y)
        if target.confidence < self.min_confidence:
            return TaskResult(
                False,
                STATUS_NO_TARGET,
                f'low confidence target: {target.confidence:.3f}',
                step='CHECK_CONFIDENCE',
                target_camera_xy=camera_xy,
            )

        arm_x, arm_y = self._camera_to_arm(target.x, target.y)
        pre_pick = (
            arm_x,
            arm_y,
            self._float_param('fixed_heights.pre_pick_z'),
        )
        pick = (arm_x, arm_y, self._float_param('fixed_heights.pick_z'))
        lift = (arm_x, arm_y, self._float_param('fixed_heights.lift_z'))

        for name, xyz in (
            ('PRE_PICK', pre_pick),
            ('PICK', pick),
            ('LIFT', lift),
        ):
            if not self._is_xyz_in_workspace(xyz):
                return TaskResult(
                    False,
                    STATUS_OUT_OF_WORKSPACE,
                    f'{name} target out of workspace: {xyz}',
                    step='CHECK_WORKSPACE',
                    target_camera_xy=camera_xy,
                    target_arm_xyz=pick,
                )

        sequence = [
            ('HOME', lambda: self.adapter.move_home()),
            ('OPEN_GRIPPER', self._open_gripper_operation),
            ('MOVE_TO_PRE_PICK', lambda: self._move_xyz(pre_pick)),
            ('MOVE_TO_PICK', lambda: self._move_xyz(pick)),
            ('CLOSE_GRIPPER', self._close_gripper_operation),
            ('WAIT', self._settle_wait),
            ('MOVE_TO_LIFT', lambda: self._move_xyz(lift)),
            ('HOME', lambda: self.adapter.move_home()),
        ]
        return self._execute_sequence(
            task, sequence, start_time, camera_xy, pick
        )

    def _execute_place_transfer(
        self,
        task: str,
        start_time: float,
    ) -> TaskResult:
        x, y, _, roll, pitch, yaw = self.transfer_place_pose
        place = (x, y, self._float_param('fixed_heights.place_z'))
        pre_place = (x, y, self._float_param('fixed_heights.pre_place_z'))
        safe_lift = tuple(self.safe_lift_pose[:3])

        for name, xyz in (
            ('PRE_PLACE_TRANSFER', pre_place),
            ('PLACE_TRANSFER', place),
            ('SAFE_LIFT', safe_lift),
        ):
            if not self._is_xyz_in_workspace(xyz):
                return TaskResult(
                    False,
                    STATUS_OUT_OF_WORKSPACE,
                    f'{name} target out of workspace: {xyz}',
                    step='CHECK_WORKSPACE',
                    target_arm_xyz=place,
                )

        sequence = [
            ('HOME', lambda: self.adapter.move_home()),
            ('MOVE_TO_PRE_PLACE_TRANSFER',
             lambda: self._move_pose(
                 pre_place[0], pre_place[1], pre_place[2],
                 roll, pitch, yaw,
             )),
            ('MOVE_TO_PLACE_TRANSFER',
             lambda: self._move_pose(
                 place[0], place[1], place[2],
                 roll, pitch, yaw,
             )),
            ('OPEN_GRIPPER', self._open_gripper_operation),
            ('WAIT', self._settle_wait),
            ('MOVE_TO_SAFE_LIFT',
             lambda: self._move_pose(*self.safe_lift_pose)),
            ('HOME', lambda: self.adapter.move_home()),
        ]
        return self._execute_sequence(
            task, sequence, start_time, None, place
        )

    def _execute_home(self, task: str, start_time: float) -> TaskResult:
        return self._execute_sequence(
            task,
            [('HOME', lambda: self.adapter.move_home())],
            start_time,
            None,
            tuple(self.home_pose[:3]),
        )

    def _execute_open_gripper(
        self,
        task: str,
        start_time: float,
    ) -> TaskResult:
        return self._execute_sequence(
            task,
            [('OPEN_GRIPPER', self._open_gripper_operation)],
            start_time,
            None,
            None,
        )

    def _execute_close_gripper(
        self,
        task: str,
        start_time: float,
    ) -> TaskResult:
        return self._execute_sequence(
            task,
            [('CLOSE_GRIPPER', self._close_gripper_operation)],
            start_time,
            None,
            None,
        )

    def _execute_sequence(
        self,
        task: str,
        sequence: List[Tuple[str, Any]],
        start_time: float,
        camera_xy: Optional[Tuple[float, float]],
        arm_xyz: Optional[Tuple[float, float, float]],
    ) -> TaskResult:
        for step_name, operation in sequence:
            state = self._check_abort_or_timeout(start_time, step_name)
            if state is not None:
                return TaskResult(
                    False,
                    state,
                    'arm task aborted' if state == STATUS_ABORTED
                    else 'arm task timeout',
                    step=step_name,
                    target_camera_xy=camera_xy,
                    target_arm_xyz=arm_xyz,
                )

            self._publish_status(
                task,
                STATUS_RUNNING,
                step_name,
                True,
                'executing',
                camera_xy,
                arm_xyz,
            )
            ok = bool(operation())
            if not ok:
                self.adapter.stop()
                return TaskResult(
                    False,
                    STATUS_FAILED,
                    f'step failed: {step_name}',
                    step=step_name,
                    target_camera_xy=camera_xy,
                    target_arm_xyz=arm_xyz,
                )

        return TaskResult(
            True,
            STATUS_DONE,
            'task completed',
            step='DONE',
            target_camera_xy=camera_xy,
            target_arm_xyz=arm_xyz,
        )

    def _wait_for_recent_target(
        self,
        start_time: float,
    ) -> Optional[CameraTarget]:
        deadline = time.monotonic() + self.target_timeout_sec
        while time.monotonic() < deadline:
            state = self._check_abort_or_timeout(start_time, 'WAIT_TARGET')
            if state is not None:
                return None
            target = self._select_latest_target()
            if target is not None:
                return target
            time.sleep(0.03)
        return None

    def _select_latest_target(self) -> Optional[CameraTarget]:
        with self._target_lock:
            candidates = [
                target for target in (
                    self._latest_json_target,
                    self._latest_point_target,
                )
                if target is not None
            ]

        now = time.monotonic()
        fresh = [
            target for target in candidates
            if now - target.received_monotonic <= self.target_timeout_sec
        ]
        if not fresh:
            return None
        return max(fresh, key=lambda target: target.source_stamp)

    def _handle_abort(self) -> None:
        self._abort_event.set()
        self.adapter.stop()
        self._publish_lock(False)
        with self._state_lock:
            active_task = self._active_task
        self._publish_status(
            active_task or 'ABORT',
            STATUS_ABORTED,
            'ABORT',
            False,
            'abort requested',
        )

    def _finish_task(self) -> None:
        self._publish_lock(False)
        with self._state_lock:
            self._active_task = ''
        self._abort_event.clear()

    def _check_abort_or_timeout(
        self,
        start_time: float,
        step_name: str,
    ) -> Optional[str]:
        if self._abort_event.is_set():
            self.get_logger().warn(f'D1 arm task aborted at {step_name}')
            return STATUS_ABORTED
        if time.monotonic() - start_time > self.task_timeout_sec:
            self.adapter.stop()
            self.get_logger().error(f'D1 arm task timeout at {step_name}')
            return STATUS_TIMEOUT
        return None

    def _camera_to_arm(
        self,
        cam_x: float,
        cam_y: float,
    ) -> Tuple[float, float]:
        a11 = self._float_param('camera_to_arm.a11')
        a12 = self._float_param('camera_to_arm.a12')
        a21 = self._float_param('camera_to_arm.a21')
        a22 = self._float_param('camera_to_arm.a22')
        tx = self._float_param('camera_to_arm.tx')
        ty = self._float_param('camera_to_arm.ty')
        arm_x = a11 * cam_x + a12 * cam_y + tx
        arm_y = a21 * cam_x + a22 * cam_y + ty
        self._validate_finite([arm_x, arm_y], 'camera_to_arm')
        return arm_x, arm_y

    def _move_xyz(self, xyz: Tuple[float, float, float]) -> bool:
        return self._move_pose(xyz[0], xyz[1], xyz[2], 0.0, 0.0, 0.0)

    def _move_pose(
        self,
        x: float,
        y: float,
        z: float,
        roll: float,
        pitch: float,
        yaw: float,
    ) -> bool:
        duration_sec = self._positive_float_param(
            'timing.move_duration_sec', 1.0
        )
        return self.adapter.move_cartesian_xyzrpy(
            x, y, z, roll, pitch, yaw, duration_sec
        )

    def _open_gripper_operation(self) -> bool:
        duration_sec = self._positive_float_param(
            'gripper.open_duration_sec', 0.5
        )
        if not self.adapter.open_gripper(duration_sec):
            return False
        return self._sleep_interruptible(
            self._nonnegative_float_param('timing.gripper_wait_sec', 0.5)
        )

    def _close_gripper_operation(self) -> bool:
        duration_sec = self._positive_float_param(
            'gripper.close_duration_sec', 0.5
        )
        if not self.adapter.close_gripper(duration_sec):
            return False
        return self._sleep_interruptible(
            self._nonnegative_float_param('timing.gripper_wait_sec', 0.5)
        )

    def _settle_wait(self) -> bool:
        return self._sleep_interruptible(
            self._nonnegative_float_param('timing.settle_wait_sec', 0.3)
        )

    def _sleep_interruptible(self, duration_sec: float) -> bool:
        deadline = time.monotonic() + max(0.0, duration_sec)
        while time.monotonic() < deadline:
            if self._abort_event.is_set():
                return False
            time.sleep(min(0.02, deadline - time.monotonic()))
        return True

    def _is_xyz_in_workspace(self, xyz: Tuple[float, float, float]) -> bool:
        try:
            self._validate_finite(list(xyz), 'workspace_xyz')
        except ValueError:
            return False
        x, y, z = xyz
        return (
            self._float_param('workspace.x_min') <= x <=
            self._float_param('workspace.x_max') and
            self._float_param('workspace.y_min') <= y <=
            self._float_param('workspace.y_max') and
            self._float_param('workspace.z_min') <= z <=
            self._float_param('workspace.z_max')
        )

    def _create_adapter(self) -> ArmHardwareAdapter:
        if self.dry_run:
            return DryRunD1ArmAdapter(self, self.home_pose)

        pose_table = BasicPoseTableAdapter(
            self,
            self._string_param('pose_table.calibrated_points_json'),
            self._positive_float_param(
                'pose_table.max_nearest_distance_m', 0.05
            ),
        )
        return UnitreeD1SdkAdapter(
            self,
            self._string_param('d1_sdk_root'),
            self.home_pose,
            pose_table,
        )

    def _publish_status(
        self,
        task: str,
        state: str,
        step: str,
        success: bool,
        message: str,
        target_camera_xy: Optional[Tuple[float, float]] = None,
        target_arm_xyz: Optional[Tuple[float, float, float]] = None,
    ) -> None:
        payload = {
            'task': task,
            'state': state,
            'step': step,
            'success': bool(success),
            'message': message,
        }
        if target_camera_xy is not None:
            payload['target_camera_xy'] = [
                float(target_camera_xy[0]),
                float(target_camera_xy[1]),
            ]
        if target_arm_xyz is not None:
            payload['target_arm_xyz'] = [
                float(target_arm_xyz[0]),
                float(target_arm_xyz[1]),
                float(target_arm_xyz[2]),
            ]
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=True)
        self.status_pub.publish(msg)

    def _publish_lock(self, locked: bool) -> None:
        msg = Bool()
        msg.data = bool(locked)
        self.lock_pub.publish(msg)

    def _validate_finite(self, values: List[float], label: str) -> None:
        for value in values:
            if not math.isfinite(float(value)):
                raise ValueError(f'{label} contains non-finite value: {value}')

    def _string_param(self, name: str) -> str:
        return str(self.get_parameter(name).value)

    def _bool_param(self, name: str, default: bool) -> bool:
        value = self.get_parameter(name).value
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in ('1', 'true', 'yes', 'on')
        return bool(default)

    def _float_param(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        self._validate_finite([value], name)
        return value

    def _positive_float_param(self, name: str, default: float) -> float:
        try:
            value = self._float_param(name)
        except Exception:
            value = default
        return max(0.001, value)

    def _nonnegative_float_param(self, name: str, default: float) -> float:
        try:
            value = self._float_param(name)
        except Exception:
            value = default
        return max(0.0, value)

    def _pose_param(self, name: str, default: List[float]) -> List[float]:
        value = self.get_parameter(name).value
        if not isinstance(value, list) or len(value) < 6:
            self.get_logger().warn(
                f'invalid pose parameter {name}; using default {default}'
            )
            return list(default)
        pose = [float(item) for item in value[:6]]
        self._validate_finite(pose, name)
        return pose

    def _default_d1_sdk_root(self) -> str:
        candidates = []
        try:
            share = Path(get_package_share_directory('rk_arm_control'))
            candidates.append(
                share.parents[3] / 'third_party' / 'unitree_d1_sdk'
            )
        except (PackageNotFoundError, IndexError):
            pass

        here = Path(__file__).resolve()
        for parent in here.parents:
            candidates.append(parent / 'third_party' / 'unitree_d1_sdk')
        candidates.append(Path.cwd() / 'third_party' / 'unitree_d1_sdk')
        candidates.append(Path(
            '/home/lqsdaba/daihao1/big-dog-league/'
            'third_party/unitree_d1_sdk'
        ))

        for candidate in candidates:
            if candidate.exists():
                return str(candidate)
        return str(candidates[-1])

    def destroy_node(self) -> bool:
        try:
            self.adapter.shutdown()
        except Exception as error:  # noqa: BLE001 - shutdown should not crash.
            self.get_logger().warn(f'D1 adapter shutdown error: {error}')
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = D1PickNode()
    executor = MultiThreadedExecutor(num_threads=2)
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
