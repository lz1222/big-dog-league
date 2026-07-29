#!/usr/bin/env python3
"""ROS wiring for the modular national integrated mission FSM.

This node publishes only ``/control/mission_cmd`` candidates.  ``rk_safety``
remains the only publisher of ``/navigation/cmd_vel``.
"""

import json
import math
import threading
import time

import rclpy
from geometry_msgs.msg import PointStamped, Twist
from rclpy.action import ActionClient, ActionServer
from rclpy.action.server import GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String

from rk_interfaces.action import ExecuteArmTask, ExecuteMotion, RunMission
from rk_interfaces.msg import ItemTagArray, LineTrack, SignDetectionArray

from .mission_adapters import MissionAdapter
from .mission_types import MissionState, MotionCommand, TaskResult
from .national_mission_fsm import (
    DEFAULT_PARAMETERS,
    TERMINAL_STATES,
    NationalMissionFSM,
)


class NationalMissionNode(Node, MissionAdapter):
    """Connect national mission policy to existing topics and Action schemas."""

    def __init__(self, parameter_overrides=None):
        super().__init__(
            'national_mission_node', parameter_overrides=parameter_overrides
        )
        self.callback_group = ReentrantCallbackGroup()
        self._declare_parameters()
        self.params = {
            name: self.get_parameter(name).value
            for name in DEFAULT_PARAMETERS
        }
        self.start_from_state = str(
            self.get_parameter('start_from_state').value
        )
        self.simulation_mode = bool(self.params['simulation_mode'])
        self._mission_command_released = True
        self._active_goal_handle = None
        self._action_future_lock = threading.Lock()
        self._pending_action_futures = set()
        self._fsm_lock = threading.RLock()

        self.mission_cmd_pub = self.create_publisher(
            Twist, self._topic('mission_cmd_topic'), 10
        )
        self.line_start_pub = self.create_publisher(
            Bool, self._topic('mission_start_topic'), 10
        )
        self.line_stop_pub = self.create_publisher(
            Bool, self._topic('mission_stop_topic'), 10
        )
        self.state_pub = self.create_publisher(
            String, self._topic('mission_state_topic'), 10
        )
        self.event_pub = self.create_publisher(
            String, self._topic('mission_event_topic'), 10
        )
        self.action_pub = self.create_publisher(
            String, self._topic('mission_action_topic'), 10
        )

        self.locomotion_client = ActionClient(
            self, ExecuteMotion, self._topic('locomotion_action_name'),
            callback_group=self.callback_group,
        )
        self.arm_client = ActionClient(
            self, ExecuteArmTask, self._topic('arm_action_name'),
            callback_group=self.callback_group,
        )
        self.run_server = ActionServer(
            self, RunMission, self._topic('mission_run_action_name'),
            self._execute_run_goal, goal_callback=self._run_goal_callback,
            callback_group=self.callback_group,
        )

        self.create_subscription(
            LineTrack, self._topic('line_track_topic'), self._on_line, 10,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            Bool, self._topic('white_line_topic'), self._on_white_line, 10,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            Float32, self._topic('white_line_confidence_topic'),
            self._on_white_confidence, 10, callback_group=self.callback_group,
        )
        self.create_subscription(
            PointStamped, self._topic('red_circle_topic'), self._on_red_circle,
            10, callback_group=self.callback_group,
        )
        self.create_subscription(
            ItemTagArray, self._topic('item_tags_topic'), self._on_item_tags,
            10, callback_group=self.callback_group,
        )
        self.create_subscription(
            SignDetectionArray, self._topic('sign_detections_topic'),
            self._on_signs, 10, callback_group=self.callback_group,
        )
        self.create_subscription(
            Twist, self._topic('final_cmd_topic'), self._on_final_command, 10,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            String, self._topic('mux_status_topic'), self._on_mux_status, 10,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            Bool, self._topic('gait_lock_topic'), self._on_gait_lock, 10,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            Bool, self._topic('arm_lock_topic'), self._on_arm_lock, 10,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            Bool, self._topic('estop_topic'), self._on_estop, 10,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            Bool, self._topic('legacy_start_topic'), self._on_legacy_start, 10,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            Bool, self._topic('legacy_stop_topic'), self._on_legacy_stop, 10,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            String, self._topic('mission_control_topic'), self._on_control, 10,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            String, self._topic('simulation_action_result_topic'),
            self._on_simulation_action_result, 10,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            Bool, self._topic('simulation_suppress_final_cmd_topic'),
            self._on_simulation_suppress_final_cmd, 10,
            callback_group=self.callback_group,
        )

        self._white_confidence = 0.0
        self._suppress_final_cmd = False
        self._gait_lock = False
        self._arm_lock = False
        self.fsm = NationalMissionFSM(self, self.params)
        period = 1.0 / max(1.0, float(self.params['control_rate_hz']))
        self.timer = self.create_timer(
            period, self._on_timer, callback_group=self.callback_group
        )
        self.emit_event('READY', 'national mission node ready; simulation_mode={}'.format(
            self.simulation_mode
        ))

    def _declare_parameters(self):
        for name, value in DEFAULT_PARAMETERS.items():
            self.declare_parameter(name, value)
        topics = {
            'mission_cmd_topic': '/control/mission_cmd',
            # These are internal line-follower controls.  The public legacy
            # /mission/start and /mission/stop remain separately subscribed
            # below, avoiding a Bool echo that could stop the FSM itself.
            'mission_start_topic': '/mission/national_line/start',
            'mission_stop_topic': '/mission/national_line/stop',
            'legacy_start_topic': '/mission/start',
            'legacy_stop_topic': '/mission/stop',
            'mission_run_action_name': '/mission/run',
            'locomotion_action_name': '/locomotion/execute_motion',
            'arm_action_name': '/arm/execute_task',
            'line_track_topic': '/perception/line_track',
            'white_line_topic': '/perception/route_markers/white_line',
            'white_line_confidence_topic': (
                '/perception/route_markers/white_line_confidence'
            ),
            'red_circle_topic': '/perception/route_markers/red_circle',
            'item_tags_topic': '/perception/item_tags',
            'sign_detections_topic': '/perception/sign_detections',
            'final_cmd_topic': '/navigation/cmd_vel',
            'mux_status_topic': '/control/cmd_mux_status',
            'gait_lock_topic': '/gait/control_lock',
            'arm_lock_topic': '/arm/control_lock',
            'estop_topic': '/safety/estop',
            'mission_state_topic': '/mission/national_state',
            'mission_event_topic': '/mission/national_events',
            'mission_action_topic': '/mission/national_actions',
            'mission_control_topic': '/mission/national_control',
            'simulation_action_result_topic': (
                '/simulation/national/action_result_override'
            ),
            'simulation_suppress_final_cmd_topic': (
                '/simulation/national/suppress_final_cmd'
            ),
        }
        for name, value in topics.items():
            self.declare_parameter(name, value)
        self.declare_parameter('start_from_state', '')

    def _topic(self, parameter_name):
        return str(self.get_parameter(parameter_name).value)

    def _now(self):
        return time.monotonic()

    # MissionAdapter implementation -------------------------------------------------
    def set_line_enabled(self, enabled):
        msg = Bool()
        msg.data = bool(enabled)
        if enabled:
            self.line_start_pub.publish(msg)
            self.emit_event('LINE_CONTROL', 'enabled')
        else:
            self.line_stop_pub.publish(msg)
            self.emit_event('LINE_CONTROL', 'disabled')

    def publish_mission_command(self, command):
        values = (command.vx, command.wz)
        if not all(math.isfinite(float(value)) for value in values):
            self.emit_event('COMMAND_REJECTED', 'non-finite mission candidate')
            command = MotionCommand()
        max_vx = 0.60
        max_wz = 1.30
        msg = Twist()
        msg.linear.x = max(-max_vx, min(max_vx, float(command.vx)))
        msg.angular.z = max(-max_wz, min(max_wz, float(command.wz)))
        self.mission_cmd_pub.publish(msg)
        self._mission_command_released = False

    def release_mission_command(self):
        if not self._mission_command_released:
            self.emit_event('MISSION_CANDIDATE_RELEASED', 'line may own mux')
        self._mission_command_released = True

    def emit_event(self, label, detail=''):
        message = String()
        message.data = json.dumps({
            'time_monotonic': self._now(),
            'state': getattr(self, 'fsm', None).state.value
            if getattr(self, 'fsm', None) is not None else 'INITIALIZING',
            'event': str(label),
            'detail': str(detail),
        }, sort_keys=True)
        self.event_pub.publish(message)
        self.get_logger().info('[MISSION] {} {}'.format(label, detail))

    def execute_motion(self, request):
        self._send_action(self.locomotion_client, request, ExecuteMotion.Goal())

    def execute_arm_task(self, request):
        self._send_action(self.arm_client, request, ExecuteArmTask.Goal())

    def execute_maze_placeholder(self, request):
        self._send_action(self.locomotion_client, request, ExecuteMotion.Goal())

    def _track_action_future(self, future, callback):
        """Keep internal Action Futures alive until callbacks finish."""
        with self._action_future_lock:
            self._pending_action_futures.add(future)

        def completed(done_future):
            try:
                callback(done_future)
            except Exception as exc:
                self.get_logger().error(
                    '[MISSION] ACTION_FUTURE_CALLBACK_ERROR {}'.format(exc)
                )
            finally:
                with self._action_future_lock:
                    self._pending_action_futures.discard(done_future)

        future.add_done_callback(completed)
        return future

    def pending_action_future_count(self):
        with self._action_future_lock:
            return len(self._pending_action_futures)

    def run_goal_active(self):
        return self._active_goal_handle is not None

    def _send_action(self, client, request, goal):
        if request.adapter in ('locomotion', 'maze'):
            goal.motion_name = request.task_name
        else:
            goal.task_name = request.task_name
            goal.target = request.target
        self._publish_action('sent', request)
        if not client.server_is_ready():
            future = client.wait_for_server(timeout_sec=0.20)
            if not future:
                with self._fsm_lock:
                    self.fsm.on_action_result(TaskResult(
                        request.token, False, 'action server unavailable', request.task_name
                    ), self._now())
                return
        future = client.send_goal_async(goal)
        self._track_action_future(
            future,
            lambda done, req=request: self._on_action_goal_response(
                done, req
            ),
        )

    def _on_action_goal_response(self, future, request):
        try:
            handle = future.result()
        except Exception as exc:
            with self._fsm_lock:
                self.fsm.on_action_result(TaskResult(
                    request.token, False, 'goal send error: {}'.format(exc),
                    request.task_name,
                ), self._now())
            return
        if handle is None or not handle.accepted:
            with self._fsm_lock:
                self.fsm.on_action_result(TaskResult(
                    request.token, False, 'action goal rejected', request.task_name,
                ), self._now())
            return
        self._publish_action('accepted', request)
        result_future = handle.get_result_async()
        self._track_action_future(
            result_future,
            lambda done, req=request: self._on_action_result(done, req),
        )

    def _on_action_result(self, future, request):
        try:
            wrapped = future.result()
            result = wrapped.result
            success = bool(result.success)
            message = str(result.message)
        except Exception as exc:
            success = False
            message = 'action result error: {}'.format(exc)
        self._publish_action('result', request, success, message)
        with self._fsm_lock:
            self.fsm.on_action_result(TaskResult(
                request.token, success, message, request.task_name,
                physical_crossing_unverified=request.task_name in (
                    self.params['start_jump_motion'], self.params['finish_jump_motion']
                ),
            ), self._now())

    def _publish_action(self, phase, request, success=None, message=''):
        payload = {
            'time_monotonic': self._now(), 'phase': phase,
            'token': request.token, 'adapter': request.adapter,
            'task_name': request.task_name, 'target': request.target,
        }
        if success is not None:
            payload['success'] = bool(success)
            payload['message'] = str(message)
        out = String()
        out.data = json.dumps(payload, sort_keys=True)
        self.action_pub.publish(out)

    # ROS callbacks -----------------------------------------------------------------
    def _on_timer(self):
        with self._fsm_lock:
            now = self._now()
            self.fsm.tick(now)
            self._publish_state()

    def _on_line(self, msg):
        with self._fsm_lock:
            self.fsm.on_line(msg.line_visible, msg.confidence, msg.lateral_error,
                             msg.heading_error)

    def _on_white_confidence(self, msg):
        self._white_confidence = float(msg.data)

    def _on_white_line(self, msg):
        with self._fsm_lock:
            self.fsm.on_white_line(msg.data, self._white_confidence, self._now())

    def _on_red_circle(self, msg):
        visible = math.isfinite(float(msg.point.z)) and float(msg.point.z) > 0.0
        with self._fsm_lock:
            self.fsm.on_red_circle(visible, msg.point.x, msg.point.y, msg.point.z,
                                   self._now())

    def _on_item_tags(self, msg):
        with self._fsm_lock:
            for tag in msg.tags:
                self.fsm.on_pick_marker(tag.tag_id, tag.confidence, self._now())

    def _on_signs(self, msg):
        with self._fsm_lock:
            for detection in msg.detections:
                value = detection.sign_value or detection.sign_type
                self.fsm.on_inspection_sign(value, detection.confidence, self._now())

    def _on_final_command(self, msg):
        if self.simulation_mode and self._suppress_final_cmd:
            return
        with self._fsm_lock:
            self.fsm.on_final_command(
                MotionCommand(msg.linear.x, msg.angular.z), self._now()
            )

    def _on_mux_status(self, msg):
        try:
            status = json.loads(msg.data)
            invalid_count = int(status.get('invalid_command_count', 0))
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        with self._fsm_lock:
            self.fsm.on_mux_invalid_command(invalid_count, self._now())

    def _on_gait_lock(self, msg):
        self._gait_lock = bool(msg.data)
        with self._fsm_lock:
            self.fsm.on_locks(self._gait_lock, self._arm_lock)

    def _on_arm_lock(self, msg):
        self._arm_lock = bool(msg.data)
        with self._fsm_lock:
            self.fsm.on_locks(self._gait_lock, self._arm_lock)

    def _on_estop(self, msg):
        with self._fsm_lock:
            self.fsm.on_estop(msg.data, self._now())

    def _on_legacy_start(self, msg):
        if msg.data:
            with self._fsm_lock:
                self.fsm.start(self._now(), self.start_from_state)

    def _on_legacy_stop(self, msg):
        if msg.data:
            with self._fsm_lock:
                self.fsm.stop(self._now(), 'legacy /mission/stop')

    def _on_control(self, msg):
        command = str(msg.data).strip().lower()
        now = self._now()
        with self._fsm_lock:
            if command == 'pause':
                self.fsm.pause(now)
            elif command == 'resume':
                self.fsm.resume(now)
            elif command == 'reset':
                self.fsm.reset(now)
            elif command == 'start':
                self.fsm.start(now, self.start_from_state)
            elif command == 'stop':
                self.fsm.stop(now, 'national control stop')

    def _on_simulation_action_result(self, msg):
        if not self.simulation_mode:
            return
        try:
            payload = json.loads(msg.data)
            result = TaskResult(
                str(payload['token']), bool(payload['success']),
                str(payload.get('message', 'simulation override')),
                str(payload.get('task_name', '')),
            )
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            self.emit_event('SIM_RESULT_REJECTED', 'invalid result override')
            return
        with self._fsm_lock:
            self.fsm.on_action_result(result, self._now())

    def _on_simulation_suppress_final_cmd(self, msg):
        if self.simulation_mode:
            self._suppress_final_cmd = bool(msg.data)

    def _run_goal_callback(self, request):
        if not request.start:
            return GoalResponse.REJECT
        with self._fsm_lock:
            if self.fsm.state not in TERMINAL_STATES and self.fsm.state != MissionState.WAIT_START:
                return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _execute_run_goal(self, goal_handle):
        now = self._now()
        with self._fsm_lock:
            started = self.fsm.start(now, self.start_from_state)
            failure_reason = self.fsm.context.failure_reason
        if not started:
            goal_handle.abort()
            result = RunMission.Result()
            result.success = False
            result.message = failure_reason or 'mission start refused'
            return result
        self._active_goal_handle = goal_handle
        try:
            while rclpy.ok():
                with self._fsm_lock:
                    terminal = self.fsm.state in TERMINAL_STATES
                    state_name = self.fsm.state.value
                    progress = self._progress_fraction()
                if terminal:
                    break
                if goal_handle.is_cancel_requested:
                    with self._fsm_lock:
                        self.fsm.stop(self._now(), 'mission action canceled')
                    goal_handle.canceled()
                    break
                feedback = RunMission.Feedback()
                feedback.stage = state_name
                feedback.progress = progress
                goal_handle.publish_feedback(feedback)
                time.sleep(0.03)
            result = RunMission.Result()
            with self._fsm_lock:
                result.success = self.fsm.state == MissionState.MISSION_COMPLETE
                failure_code = self.fsm.context.failure_code.value
                failure_reason = self.fsm.context.failure_reason
            result.message = (
                'national mission completed'
                if result.success
                else '{}: {}'.format(
                    failure_code, failure_reason,
                )
            )
            if result.success:
                goal_handle.succeed()
            elif not goal_handle.is_cancel_requested:
                goal_handle.abort()
            return result
        finally:
            self._active_goal_handle = None

    def _publish_state(self):
        context = self.fsm.context
        msg = String()
        msg.data = json.dumps({
            'mission_id': context.mission_id,
            'attempt_index': context.attempt_index,
            'state': self.fsm.state.value,
            'previous_state': context.previous_state.value
            if context.previous_state else '',
            'failure_code': context.failure_code.value,
            'failure_reason': context.failure_reason,
            'target_place_platform': context.target_place_platform or 0,
            'pick_marker_id': context.pick_marker_id or 0,
            'inspection_type': context.inspection_type.value
            if context.inspection_type else '',
            'inspection_completed': context.inspection_completed,
            'transfer_place_completed': context.transfer_place_completed,
            'transfer_pick_completed': context.transfer_pick_completed,
            'place_completed': context.place_completed,
            'start_jump_completed': context.start_jump_completed,
            'finish_jump_completed': context.finish_jump_completed,
            'physical_crossing_unverified': (
                context.physical_crossing_unverified
            ),
            'anchor': self.fsm.progress.current_segment_anchor or '',
            'commanded_forward_distance_estimate': (
                self.fsm.progress.commanded_forward_distance
            ),
            'absolute_turn_progress': self.fsm.progress.absolute_turn_progress,
        }, sort_keys=True)
        self.state_pub.publish(msg)

    def _progress_fraction(self):
        states = list(MissionState)
        try:
            return min(1.0, states.index(self.fsm.state) / 45.0)
        except ValueError:
            return 0.0


def main(args=None):
    rclpy.init(args=args)
    node = NationalMissionNode()
    executor = MultiThreadedExecutor(num_threads=6)
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
