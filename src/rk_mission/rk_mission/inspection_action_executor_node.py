#!/usr/bin/env python3

"""以 request 驱动方式执行比赛警示牌 SDK 动作。

本节点不产生路线决策、不发布 ``/mission/stop``，也不直接发布最终
``/navigation/cmd_vel``。它只在路线节点显式授权后锁定 gait，确认 command
mux 的最终速度已归零和 estop 心跳安全，再用独立进程组调用 SDK helper。
"""

import json
import math
import os
from pathlib import Path
import socket
import threading
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from std_msgs.msg import Bool, String

from rk_interfaces.msg import SignDetectionArray
from rk_mission.inspection_action_core import CANCELED
from rk_mission.inspection_action_core import FAILED
from rk_mission.inspection_action_core import FAULTED
from rk_mission.inspection_action_core import InspectionActionConfig
from rk_mission.inspection_action_core import InspectionActionCore
from rk_mission.inspection_action_core import InspectionActionEvent
from rk_mission.inspection_action_core import (
    InspectionActionProtocolError,
)
from rk_mission.inspection_action_core import ProcessGroupHelperRunner
from rk_mission.inspection_action_core import SUCCEEDED
from rk_mission.inspection_action_core import TIMEOUT
from rk_mission.inspection_action_core import all_velocity_values_zero
from rk_mission.inspection_action_core import decode_inspection_request
from rk_mission.inspection_action_core import finite_float
from rk_mission.inspection_action_core import normalize_label
from rk_mission.inspection_action_core import warning_action_for

# ---- Foxy/Humble 兼容：Context 关闭时的 shutdown 异常 ----
# Humble+ 中 rclpy._rclpy_pybind11.RCLError 在 Foxy 不存在。
# 构造一个只包含当前平台实际可用异常类型的窄元组，用于 executor.spin() 的
# 安全关闭捕获。
_SHUTDOWN_SIGNALS = ()
try:
    from rclpy._rclpy_pybind11 import RCLError as _RCLError  # noqa: E402
    _SHUTDOWN_SIGNALS = (_RCLError,)
except (ImportError, ModuleNotFoundError):
    pass


FINAL_CMD_QOS = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
)
ESTOP_STATE_QOS = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)
TEST_ONLY_SMOKE_HELPER_MARKER = (
    b'RK_NON_ARM_TEST_ONLY_FAKE_SDK_HELPER_V1'
)
STATUS_HEARTBEAT_SEC = 0.5
# 锁 heartbeat 周期必须明显短于仲裁器 source_timeout_sec(2.0s)，
# 防止空闲时因消息静默触发 fail-closed 安全锁。
LOCK_HEARTBEAT_SEC = 0.8


def is_test_only_smoke_helper(path):
    """验证 software smoke 的 helper 是仓库测试 ELF 而非真实 SDK。

    仅接受 ELF magic 和固定测试标识同时存在的文件。此函数是运行前的
    fail-closed 边界；检查失败时不会启动进程组，也不会触碰网络接口。
    """
    try:
        with Path(str(path)).open('rb') as stream:
            contents = stream.read(65536)
    except (OSError, TypeError, ValueError):
        return False
    return contents.startswith(b'\x7fELF') and (
        TEST_ONLY_SMOKE_HELPER_MARKER in contents
    )


class InspectionActionExecutorNode(Node):
    """把一次已匹配的 inspection request 适配为安全 SDK helper 调用。"""

    def __init__(self):
        super().__init__('inspection_action_executor')
        self.callback_group = ReentrantCallbackGroup()
        self._state_lock = threading.RLock()
        self._shutdown_requested = threading.Event()
        self._helper_thread = None
        self._helper_cancel_event = None
        self._last_cmd_mux_status = ''
        self._last_cmd_mux_status_time = None
        self._last_status_publish_time = 0.0
        # 锁 heartbeat 追踪
        self._last_lock_value = False
        self._last_lock_publish_time = 0.0
        self._declare_parameters()
        self._read_parameters()
        self.core = InspectionActionCore(self.core_config)

        self.status_publisher = self.create_publisher(
            String, self.status_topic, 10
        )
        self.gait_lock_publisher = self.create_publisher(
            Bool, self.gait_control_lock_topic, 10
        )
        self.request_subscription = self.create_subscription(
            String,
            self.request_topic,
            self._on_request,
            10,
            callback_group=self.callback_group,
        )
        self.sign_subscription = self.create_subscription(
            SignDetectionArray,
            self.sign_detections_topic,
            self._on_sign_detections,
            10,
            callback_group=self.callback_group,
        )
        self.stop_subscription = self.create_subscription(
            Bool,
            self.mission_stop_topic,
            self._on_mission_stop,
            10,
            callback_group=self.callback_group,
        )
        self.final_cmd_subscription = self.create_subscription(
            Twist,
            self.final_cmd_topic,
            self._on_final_cmd,
            FINAL_CMD_QOS,
            callback_group=self.callback_group,
        )
        self.estop_subscription = self.create_subscription(
            Bool,
            self.estop_state_topic,
            self._on_estop_state,
            ESTOP_STATE_QOS,
            callback_group=self.callback_group,
        )
        self.cmd_mux_status_subscription = self.create_subscription(
            String,
            self.cmd_mux_status_topic,
            self._on_cmd_mux_status,
            10,
            callback_group=self.callback_group,
        )
        self.state_timer = self.create_timer(
            0.05,
            self._on_state_timer,
            callback_group=self.callback_group,
        )

        self._publish_status(
            InspectionActionEvent(
                state='IDLE',
                run_id='',
                request_id='',
                sign_type='',
                sign_value='',
                sdk_action='',
                confidence=0.0,
                reason='inspection_action_executor_ready',
                success=False,
            )
        )
        mode = (
            'SOFTWARE_SMOKE_MODE'
            if self.software_smoke_mode else 'HARDWARE_MODE'
        )
        self.get_logger().info(
            'Inspection action executor ready: '
            'request_topic={}, sign_topic={}, helper={}, mode={}'.format(
                self.request_topic,
                self.sign_detections_topic,
                self.sdk_action_executable or '<not-configured>',
                mode,
            )
        )

    def _declare_parameters(self):
        """声明正式路径的话题和全部可审计安全参数。"""
        self.declare_parameter(
            'request_topic', '/mission/inspection_action_request'
        )
        self.declare_parameter(
            'status_topic', '/mission/inspection_action_status'
        )
        self.declare_parameter(
            'sign_detections_topic', '/perception/sign_detections'
        )
        self.declare_parameter('mission_stop_topic', '/mission/stop')
        self.declare_parameter('final_cmd_topic', '/navigation/cmd_vel')
        self.declare_parameter('estop_state_topic', '/safety/estop_state')
        self.declare_parameter(
            'cmd_mux_status_topic', '/control/cmd_mux_status'
        )
        self.declare_parameter('gait_control_lock_topic', '/gait/control_lock_req/inspection')

        self.declare_parameter('sign_confirm_frames', 5)
        self.declare_parameter('sign_min_confidence', 0.70)
        self.declare_parameter('sign_wait_timeout_sec', 8.0)
        self.declare_parameter('final_zero_epsilon', 0.001)
        self.declare_parameter('final_zero_confirm_samples', 3)
        self.declare_parameter('final_zero_timeout_sec', 3.0)
        self.declare_parameter('final_cmd_stale_timeout_sec', 0.50)
        self.declare_parameter('estop_state_stale_timeout_sec', 0.50)
        self.declare_parameter('sdk_action_timeout_sec', 15.0)
        self.declare_parameter('post_action_settle_sec', 1.5)
        self.declare_parameter('sdk_network_interface', 'eth0')
        # 正式 launch 必须传入 install 下的可解析绝对路径，不能依赖 cwd 猜测。
        self.declare_parameter('sdk_action_executable', '')
        self.declare_parameter('software_smoke_mode', False)
        self.declare_parameter('helper_poll_interval_sec', 0.05)
        self.declare_parameter('helper_terminate_grace_sec', 0.50)
        self.declare_parameter('helper_kill_grace_sec', 0.50)
        self.declare_parameter('helper_shutdown_wait_sec', 2.0)

    def _read_parameters(self):
        """读取并提前校验节点配置，错误参数不能变成隐式硬件行为。"""
        for name in (
            'request_topic',
            'status_topic',
            'sign_detections_topic',
            'mission_stop_topic',
            'final_cmd_topic',
            'estop_state_topic',
            'cmd_mux_status_topic',
            'gait_control_lock_topic',
        ):
            value = str(self.get_parameter(name).value).strip()
            if not value:
                raise ValueError('{} must not be empty'.format(name))
            setattr(self, name, value)

        self.core_config = InspectionActionConfig(
            sign_confirm_frames=self.get_parameter(
                'sign_confirm_frames'
            ).value,
            sign_min_confidence=self.get_parameter(
                'sign_min_confidence'
            ).value,
            sign_wait_timeout_sec=self.get_parameter(
                'sign_wait_timeout_sec'
            ).value,
            final_zero_epsilon=self.get_parameter(
                'final_zero_epsilon'
            ).value,
            final_zero_confirm_samples=self.get_parameter(
                'final_zero_confirm_samples'
            ).value,
            final_zero_timeout_sec=self.get_parameter(
                'final_zero_timeout_sec'
            ).value,
            final_cmd_stale_timeout_sec=self.get_parameter(
                'final_cmd_stale_timeout_sec'
            ).value,
            estop_state_stale_timeout_sec=self.get_parameter(
                'estop_state_stale_timeout_sec'
            ).value,
        )
        self.sdk_action_timeout_sec = self._positive_parameter(
            'sdk_action_timeout_sec'
        )
        self.post_action_settle_sec = self._nonnegative_parameter(
            'post_action_settle_sec'
        )
        self.helper_poll_interval_sec = self._positive_parameter(
            'helper_poll_interval_sec'
        )
        self.helper_terminate_grace_sec = self._positive_parameter(
            'helper_terminate_grace_sec'
        )
        self.helper_kill_grace_sec = self._positive_parameter(
            'helper_kill_grace_sec'
        )
        self.helper_shutdown_wait_sec = self._positive_parameter(
            'helper_shutdown_wait_sec'
        )
        self.sdk_network_interface = str(
            self.get_parameter('sdk_network_interface').value
        ).strip()
        if not self.sdk_network_interface:
            raise ValueError('sdk_network_interface must not be empty')
        self.sdk_action_executable = str(
            self.get_parameter('sdk_action_executable').value
        ).strip()
        self.software_smoke_mode = self._bool_parameter(
            'software_smoke_mode'
        )

    def _on_request(self, msg):
        """仅接受一次严格 JSON 请求，非法 payload 不会关联到任何当前 run。"""
        now = time.monotonic()
        try:
            request = decode_inspection_request(msg.data)
        except InspectionActionProtocolError as error:
            self._publish_protocol_failure(str(error))
            return
        with self._state_lock:
            event = self.core.request(request, now)
        self._handle_event(event)
        if event.reason != 'inspection_request_accepted':
            return
        with self._state_lock:
            arm_event = self.core.arm(request.run_id, request.request_id, now)
        self._handle_event(arm_event)

    def _on_sign_detections(self, msg):
        """每帧只交给核心一个最佳合格候选，避免同帧多框伪造连续确认。"""
        candidate = self._select_best_warning_detection(msg)
        now = time.monotonic()
        with self._state_lock:
            if candidate is None:
                event = self.core.observe_detection('', '', float('nan'), now)
            else:
                event = self.core.observe_detection(*candidate, now)
        self._handle_event(event)

    def _on_mission_stop(self, msg):
        """mission stop 只取消本 helper；路线级 emergency 仍由路线节点决定。"""
        if not msg.data:
            return
        with self._state_lock:
            event = self.core.mission_stop()
        self._handle_event(event)

    def _on_final_cmd(self, msg):
        """读取 command_mux 的最终 Twist，六个分量均为零才算安全静止。"""
        values = (
            msg.linear.x,
            msg.linear.y,
            msg.linear.z,
            msg.angular.x,
            msg.angular.y,
            msg.angular.z,
        )
        is_zero = all_velocity_values_zero(
            values, self.core_config.final_zero_epsilon
        )
        with self._state_lock:
            event = self.core.observe_final_cmd(is_zero, time.monotonic())
        self._handle_event(event)

    def _on_estop_state(self, msg):
        """typed estop 的 false 心跳必须新鲜，消息缺失不能被当作安全。"""
        with self._state_lock:
            event = self.core.observe_estop(bool(msg.data), time.monotonic())
        self._handle_event(event)

    def _on_cmd_mux_status(self, msg):
        """保留 mux 状态的新鲜观测，便于状态日志和 readiness 排障。"""
        self._last_cmd_mux_status = str(msg.data)
        self._last_cmd_mux_status_time = time.monotonic()

    def _on_state_timer(self):
        """让无检测/无速度消息也能在超时后安全终止。"""
        with self._state_lock:
            event = self.core.tick(time.monotonic())
        self._handle_event(event)
        self._publish_status_heartbeat()
        # 持续重发当前锁状态作为 heartbeat，确保仲裁器不会因静默超时
        # 而进入 fail-closed 安全锁。
        self._republish_gait_lock_heartbeat()

    def _select_best_warning_detection(self, msg):
        """选择本帧最高置信度的允许映射，未知或低置信候选全部忽略。"""
        best = None
        best_confidence = -1.0
        for detection in msg.detections:
            try:
                confidence = float(detection.confidence)
            except (TypeError, ValueError):
                continue
            sign_type = normalize_label(detection.sign_type)
            sign_value = normalize_label(detection.sign_value)
            if (
                not math.isfinite(confidence)
                or confidence < self.core_config.sign_min_confidence
                or warning_action_for(sign_type, sign_value) is None
            ):
                continue
            if confidence > best_confidence:
                best = (sign_type, sign_value, confidence)
                best_confidence = confidence
        return best

    def _handle_event(self, event):
        """按安全顺序执行核心副作用：先状态，再锁/线程，最后在 cleanup 后解锁。"""
        if event is None:
            return
        if self._shutdown_requested.is_set() or not rclpy.ok():
            # ROS context 已关闭后，继续发布状态或 gait lock 会抛 RCLError。
            # 只通知 helper 线程回收其独立进程组；正常 mission stop 仍在
            # context 有效时走完整状态/解锁流程。
            if event.terminate_helper:
                self._request_helper_termination()
            return
        self._publish_status(event)
        if event.terminate_helper:
            self._request_helper_termination()
        if event.acquire_gait_lock:
            self._acquire_gait_lock(event)
        if event.start_helper:
            self._start_helper(event)
        if event.release_gait_lock:
            self._publish_gait_lock(False)

    def _acquire_gait_lock(self, event):
        """成功发布 lock 后才允许核心开始零速度确认，防止提前计数。"""
        try:
            self._publish_gait_lock(True)
        except Exception as error:
            with self._state_lock:
                follow_up = self.core.lock_failed(
                    event.run_id, event.request_id, type(error).__name__
                )
            self._handle_event(follow_up)
            return
        with self._state_lock:
            follow_up = self.core.lock_acquired(
                event.run_id, event.request_id, time.monotonic()
            )
        self._handle_event(follow_up)

    def _start_helper(self, event):
        """在标记 cleanup pending 后启动线程，stop 与启动竞争时也不会提前解锁。"""
        with self._state_lock:
            if not self.core.helper_launching(
                event.run_id, event.request_id
            ):
                return
            cancel_event = threading.Event()
            self._helper_cancel_event = cancel_event
            thread = threading.Thread(
                target=self._run_helper_thread,
                args=(event.run_id, event.request_id, event.sdk_action,
                      cancel_event),
                daemon=True,
            )
            self._helper_thread = thread
            try:
                thread.start()
            except Exception as error:
                self._helper_thread = None
                self._helper_cancel_event = None
                failure = self.core.helper_finished(
                    event.run_id,
                    event.request_id,
                    FAILED,
                    'helper_thread_start_failed:{}'.format(
                        type(error).__name__
                    ),
                    cleanup_completed=True,
                )
        if 'failure' in locals():
            self._handle_event(failure)

    def _run_helper_thread(self, run_id, request_id, sdk_action, cancel_event):
        """解析 helper、校验接口、等待 SDK 与 settle，并把 cleanup 结果送回核心。"""
        result = None
        try:
            executable = self._resolve_sdk_action_executable()
            interface_error = self._validate_network_interface()
            if interface_error:
                result_state = FAILED
                reason = interface_error
                cleanup_completed = True
            else:
                runner = ProcessGroupHelperRunner(
                    poll_interval_sec=self.helper_poll_interval_sec,
                    terminate_grace_sec=self.helper_terminate_grace_sec,
                    kill_grace_sec=self.helper_kill_grace_sec,
                )
                result = runner.run(
                    [
                        executable,
                        self.sdk_network_interface,
                        sdk_action,
                        '0',
                    ],
                    self.sdk_action_timeout_sec,
                    cancel_event,
                    environment=os.environ.copy(),
                )
                result_state = result.terminal_state
                reason = result.reason
                cleanup_completed = result.cleanup_completed
                if result_state == SUCCEEDED:
                    if not self._wait_post_action_settle(cancel_event):
                        result_state = CANCELED
                        reason = 'post_action_settle_canceled'
        except Exception as error:
            result_state = FAILED
            reason = 'helper_execution_exception:{}'.format(
                type(error).__name__
            )
            cleanup_completed = True
            self.get_logger().error(
                'Inspection SDK helper exception: {}'.format(error)
            )

        with self._state_lock:
            event = self.core.helper_finished(
                run_id,
                request_id,
                result_state,
                reason,
                cleanup_completed=cleanup_completed,
            )
            if self._helper_cancel_event is cancel_event:
                self._helper_cancel_event = None
                self._helper_thread = None
        self._handle_event(event)

    def _wait_post_action_settle(self, cancel_event):
        """SDK 返回 0 后仍保持 gait lock 一小段时间，避免立即恢复巡线。"""
        deadline = time.monotonic() + self.post_action_settle_sec
        while time.monotonic() < deadline:
            if cancel_event.wait(min(0.05, deadline - time.monotonic())):
                return False
        return not cancel_event.is_set()

    def _request_helper_termination(self):
        """只通知 helper 线程取消；线程会负责 TERM/KILL/wait 全进程组。"""
        with self._state_lock:
            cancel_event = self._helper_cancel_event
        if cancel_event is not None:
            cancel_event.set()

    def _resolve_sdk_action_executable(self):
        """解析 helper；smoke 额外拒绝任何真实或未标记 SDK 二进制。"""
        configured = self.sdk_action_executable
        if not configured:
            raise RuntimeError('sdk_action_executable_not_configured')
        if not os.path.isabs(configured):
            raise RuntimeError('sdk_action_executable_must_be_absolute')
        resolved = os.path.realpath(configured)
        if resolved != configured:
            raise RuntimeError('sdk_action_executable_must_be_normalized')
        if not os.path.isfile(resolved) or not os.access(resolved, os.X_OK):
            raise RuntimeError('sdk_action_executable_not_executable')
        if (
            self.software_smoke_mode
            and not is_test_only_smoke_helper(resolved)
        ):
            raise RuntimeError('software_smoke_helper_identity_rejected')
        return resolved

    def _validate_network_interface(self):
        """software smoke 的假 helper 不碰 Unitree SDK，其余模式验证真实接口。"""
        if self.software_smoke_mode:
            return ''
        try:
            interface_index = socket.if_nametoindex(self.sdk_network_interface)
        except OSError:
            return 'sdk_network_interface_not_found'
        if interface_index <= 0:
            return 'sdk_network_interface_not_found'
        return ''

    def _publish_gait_lock(self, locked):
        """发布独立 gait lock，永不发布最终 Twist。"""
        message = Bool()
        message.data = bool(locked)
        self.gait_lock_publisher.publish(message)
        self._last_lock_value = bool(locked)
        self._last_lock_publish_time = time.monotonic()

    def _republish_gait_lock_heartbeat(self):
        """持续重发锁状态，确保 fail-closed 仲裁器不会因静默超时。"""
        if not hasattr(self, '_last_lock_value'):
            return
        now = time.monotonic()
        last_time = getattr(self, '_last_lock_publish_time', 0.0)
        if now - last_time < LOCK_HEARTBEAT_SEC:
            return
        message = Bool()
        message.data = self._last_lock_value
        self.gait_lock_publisher.publish(message)
        self._last_lock_publish_time = now

    def _publish_status_heartbeat(self):
        """低频重发当前状态，保证 readiness 晚订阅仍能得到新鲜证据。"""
        if self._shutdown_requested.is_set() or not rclpy.ok():
            return
        now = time.monotonic()
        if now - self._last_status_publish_time < STATUS_HEARTBEAT_SEC:
            return
        with self._state_lock:
            event = InspectionActionEvent(
                state=self.core.state,
                run_id=self.core.run_id,
                request_id=self.core.request_id,
                sign_type=self.core.sign_type,
                sign_value=self.core.sign_value,
                sdk_action=self.core.sdk_action,
                confidence=self.core.confidence,
                reason='inspection_action_status_heartbeat',
                success=self.core.state == SUCCEEDED,
            )
        self._publish_status(event, log_message=False)

    def _publish_status(self, event, *, log_message=True):
        """保持固定 JSON 字段，供路线节点按双 ID 过滤旧结果。"""
        message = String()
        message.data = json.dumps({
            'run_id': event.run_id,
            'request_id': event.request_id,
            'state': event.state,
            'sign_type': event.sign_type,
            'sign_value': event.sign_value,
            'sdk_action': event.sdk_action,
            'confidence': float(event.confidence),
            'reason': event.reason,
            'success': bool(event.success),
        }, separators=(',', ':'))
        self.status_publisher.publish(message)
        self._last_status_publish_time = time.monotonic()
        if not log_message:
            return
        if event.state in (FAILED, TIMEOUT, CANCELED, FAULTED):
            self.get_logger().warn(
                '[INSPECTION_ACTION] {} {} {}: {}'.format(
                    event.state,
                    event.run_id,
                    event.request_id,
                    event.reason,
                )
            )
        else:
            self.get_logger().info(
                '[INSPECTION_ACTION] {} {} {}: {}'.format(
                    event.state,
                    event.run_id,
                    event.request_id,
                    event.reason,
                )
            )

    def _publish_protocol_failure(self, reason):
        """非法请求没有可信 ID，因此只发布不可匹配的 FAILED 诊断状态。"""
        self._publish_status(
            InspectionActionEvent(
                state=FAILED,
                run_id='',
                request_id='',
                sign_type='',
                sign_value='',
                sdk_action='',
                confidence=0.0,
                reason=reason,
                success=False,
            )
        )

    def _bool_parameter(self, name):
        value = self.get_parameter(name).value
        if isinstance(value, str):
            return value.strip().lower() in ('1', 'true', 'yes', 'on')
        return bool(value)

    def _positive_parameter(self, name):
        return finite_float(
            self.get_parameter(name).value, name, positive=True
        )

    def _nonnegative_parameter(self, name):
        return finite_float(
            self.get_parameter(name).value, name, nonnegative=True
        )

    def destroy_node(self):
        """退出时先请求 helper 清理，避免 ROS 已停而 SDK 子进程继续运行。"""
        self._shutdown_requested.set()
        with self._state_lock:
            event = self.core.mission_stop()
            helper_thread = self._helper_thread
        self._handle_event(event)
        if helper_thread is not None and helper_thread.is_alive():
            helper_thread.join(timeout=self.helper_shutdown_wait_sec)
        return super().destroy_node()


def main(args=None):
    """启动多线程 executor，使 timer、stop 和 helper 回调可并发完成。"""
    rclpy.init(args=args)
    node = InspectionActionExecutorNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException) + _SHUTDOWN_SIGNALS:
        # launch 的 SIGINT 与 helper 完成回调可并发到达；Foxy/Humble 在
        # context 失效时从 wait set 抛出的异常类型不同。两者都属于可预期的
        # 关闭竞态而非任务失败，finally 仍会请求 helper 进程组清理。
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
