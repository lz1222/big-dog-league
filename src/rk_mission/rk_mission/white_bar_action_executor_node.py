#!/usr/bin/env python3

"""Execute explicitly configured white-bar jumps through ExecuteMotion."""

import json
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, String

from rk_interfaces.action import ExecuteMotion
from rk_mission.white_bar_action_core import WhiteBarActionExecutorCore


STATUS_HEARTBEAT_SEC = 0.5


class WhiteBarActionExecutorNode(Node):
    """Turn one approved white-bar request into one ExecuteMotion goal."""

    def __init__(self):
        super().__init__('white_bar_action_executor')
        self.callback_group = ReentrantCallbackGroup()
        self._state_lock = threading.RLock()
        self._shutdown_requested = threading.Event()
        self._declare_parameters()
        self._read_parameters()

        self.core = WhiteBarActionExecutorCore()
        self._request_started_time = None
        self._goal_started_time = None
        self._goal_handle = None
        self._last_status_publish_time = 0.0

        self.done_publisher = self.create_publisher(Bool, self.done_topic, 10)
        self.status_publisher = self.create_publisher(
            String,
            self.status_topic,
            10
        )
        self.request_subscription = self.create_subscription(
            String,
            self.request_topic,
            self._on_action_request,
            10,
            callback_group=self.callback_group
        )
        self.stop_subscription = self.create_subscription(
            Bool,
            self.mission_stop_topic,
            self._on_mission_stop,
            10,
            callback_group=self.callback_group
        )
        self.action_client = ActionClient(
            self,
            ExecuteMotion,
            self.action_name,
            callback_group=self.callback_group
        )
        self.state_timer = self.create_timer(
            0.05,
            self._poll_action_state,
            callback_group=self.callback_group
        )

        self._publish_status(
            'IDLE',
            '',
            'white_bar_action_executor_ready',
            0
        )
        self.get_logger().info(
            'White-bar action executor ready: '
            f'request_topic={self.request_topic}, '
            f'action_name={self.action_name}'
        )

    def _declare_parameters(self):
        self.declare_parameter(
            'request_topic',
            '/mission/white_bar_action_request'
        )
        self.declare_parameter('done_topic', '/mission/white_bar_action_done')
        self.declare_parameter(
            'status_topic',
            '/mission/white_bar_action_status'
        )
        self.declare_parameter('mission_stop_topic', '/mission/stop')
        self.declare_parameter('action_name', '/locomotion/execute_motion')
        # FrontJump 最坏时长约 17s；执行器须先超时取消。
        self.declare_parameter('server_wait_timeout_sec', 5.0)
        self.declare_parameter('action_timeout_sec', 22.0)

    def _read_parameters(self):
        for name in (
            'request_topic',
            'done_topic',
            'status_topic',
            'mission_stop_topic',
            'action_name',
        ):
            value = str(self.get_parameter(name).value).strip()
            if not value:
                raise ValueError(f'{name} must not be empty')
            setattr(self, name, value)
        self.server_wait_timeout_sec = self._positive_float_parameter(
            'server_wait_timeout_sec'
        )
        self.action_timeout_sec = self._positive_float_parameter(
            'action_timeout_sec'
        )

    def _positive_float_parameter(self, name):
        value = float(self.get_parameter(name).value)
        if value <= 0.0:
            raise ValueError(f'{name} must be positive')
        return value

    def _on_action_request(self, msg):
        with self._state_lock:
            event = self.core.request(msg.data)
            if event.status == 'WAIT_SERVER' and event.reason == (
                'waiting_for_action_server'
            ):
                self._request_started_time = time.monotonic()
                self._goal_started_time = None
                self._goal_handle = None
        self._handle_event(event)

    def _on_mission_stop(self, msg):
        if not msg.data:
            return
        with self._state_lock:
            event = self.core.mission_stop()
            self._request_started_time = None
            self._goal_started_time = None
        self._handle_event(event)

    def _poll_action_state(self):
        """Action 超时与状态重发，支持 readiness 晚订阅。"""
        event = None
        with self._state_lock:
            if self.core.active:
                request_id = self.core.request_id
                status = self.core.status
                now = time.monotonic()
                if status == 'WAIT_SERVER':
                    if self.action_client.server_is_ready():
                        event = self.core.server_ready(request_id)
                    elif (
                        self._request_started_time is not None
                        and now - self._request_started_time
                        >= self.server_wait_timeout_sec
                    ):
                        event = self.core.timeout(
                            request_id,
                            'execute_motion_server_unavailable_timeout'
                        )
                elif status in ('GOAL_SENT', 'RUNNING'):
                    started_time = (
                        self._goal_started_time or self._request_started_time
                    )
                    if (
                        started_time is not None
                        and now - started_time >= self.action_timeout_sec
                    ):
                        event = self.core.timeout(request_id)
        self._handle_event(event)
        self._publish_status_heartbeat()

    def _handle_event(self, event):
        if event is None:
            return
        if self._shutdown_requested.is_set() or not rclpy.ok():
            # ROS context 失效后只取消 Action，不再发布状态。
            # 否则 launch 退出路径会因 RCLError 变成失败。
            if event.cancel_goal:
                self._cancel_active_goal()
            return
        self._publish_status(
            event.status,
            event.motion_name,
            event.reason,
            event.request_id
        )
        if event.send_goal:
            self._send_goal(event)
        if event.cancel_goal:
            self._cancel_active_goal()
        if event.publish_done:
            done = Bool()
            done.data = True
            self.done_publisher.publish(done)

    def _send_goal(self, event):
        goal = ExecuteMotion.Goal()
        goal.motion_name = event.motion_name
        try:
            goal_future = self.action_client.send_goal_async(goal)
        except Exception as exc:
            # ROS Action 传输异常是运行态故障，不能发布 done。
            with self._state_lock:
                failure = self.core.goal_send_failed(
                    event.request_id,
                    f'execute_motion_goal_send_failed:{exc}'
                )
            self._handle_event(failure)
            return
        goal_future.add_done_callback(
            lambda future: self._on_goal_response(event.request_id, future)
        )

    def _on_goal_response(self, request_id, future):
        try:
            goal_handle = future.result()
        except Exception as exc:
            # DDS 响应丢失按失败处理，避免未知目标成功。
            with self._state_lock:
                event = self.core.goal_send_failed(
                    request_id,
                    f'execute_motion_goal_response_failed:{exc}'
                )
            self._handle_event(event)
            return

        if not goal_handle.accepted:
            with self._state_lock:
                event = self.core.goal_rejected(request_id)
            self._handle_event(event)
            return

        with self._state_lock:
            event = self.core.goal_accepted(request_id)
            still_active = event is not None
            if still_active:
                self._goal_handle = goal_handle
                self._goal_started_time = time.monotonic()
        if not still_active:
            goal_handle.cancel_goal_async()
            return
        self._handle_event(event)
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda future: self._on_result(request_id, future)
        )

    def _on_result(self, request_id, future):
        try:
            wrapped_result = future.result()
            action_completed_normally = (
                wrapped_result.status == GoalStatus.STATUS_SUCCEEDED
            )
            result_success = bool(wrapped_result.result.success)
            result_message = str(wrapped_result.result.message).strip()
            reason = result_message or 'execute_motion_result_received'
        except Exception as exc:  # A result transport error is never success.
            action_completed_normally = False
            result_success = False
            reason = f'execute_motion_result_failed:{exc}'
        with self._state_lock:
            event = self.core.action_result(
                request_id,
                action_completed_normally,
                result_success,
                reason
            )
            if event is not None:
                self._goal_handle = None
                self._goal_started_time = None
                self._request_started_time = None
        self._handle_event(event)

    def _cancel_active_goal(self):
        """请求取消但保留 goal handle，直到 result 证明 Action 已终态。"""
        with self._state_lock:
            goal_handle = self._goal_handle
        if goal_handle is None:
            return
        try:
            goal_handle.cancel_goal_async()
        except Exception as exc:
            # 取消请求异常仍不得绕过核心状态机发布 done。
            self.get_logger().error(
                f'ExecuteMotion cancel request failed: {exc}'
            )

    def _publish_status_heartbeat(self):
        """低频状态心跳消除 readiness 晚订阅的 QoS 竞态。"""
        if self._shutdown_requested.is_set() or not rclpy.ok():
            return
        now = time.monotonic()
        if now - self._last_status_publish_time < STATUS_HEARTBEAT_SEC:
            return
        with self._state_lock:
            status = self.core.status
            motion_name = self.core.motion_name
            request_id = self.core.request_id
        self._publish_status(
            status,
            motion_name,
            'white_bar_action_status_heartbeat',
            request_id,
            log_message=False,
        )

    def _publish_status(
        self,
        status,
        motion_name,
        reason,
        request_id,
        *,
        log_message=True,
    ):
        """发布状态；心跳不刷日志，动作事件完整记录。"""
        message = String()
        message.data = json.dumps({
            'status': status,
            'motion_name': motion_name,
            'reason': reason,
            'request_id': request_id,
        }, separators=(',', ':'))
        self.status_publisher.publish(message)
        self._last_status_publish_time = time.monotonic()
        if not log_message:
            return
        if status in ('FAILED', 'TIMEOUT', 'CANCELED'):
            self.get_logger().warn(
                f'[WHITE_BAR_ACTION] {status}: {motion_name}: {reason}'
            )
        else:
            self.get_logger().info(
                f'[WHITE_BAR_ACTION] {status}: {motion_name}: {reason}'
            )

    def destroy_node(self):
        """关闭时只取消目标，不在失效 context 发布状态。"""
        self._shutdown_requested.set()
        with self._state_lock:
            event = self.core.mission_stop()
        # launch 的 SIGINT 可能先让 rclpy context 失效。
        # 此时 _handle_event 会因 publisher 失效而使清理异常；
        # 仍尽力取消 Action，但不把 shutdown 作为可发布事件。
        self._handle_event(event)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WhiteBarActionExecutorNode()
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
