"""动态验收专用轻量运动 adapter ROS 节点。

节点不写文件、不处理 white-bar、不生成报告；所有 callback 只更新 latest-state，
控制 timer 负责一次性 arm、fail-closed 和 1 秒运动窗口。
"""

import argparse
import json
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from rclpy.qos import ReliabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool

from rk_interfaces.msg import LineTrack
from rk_bringup.validation_motion_core import MOVE
from rk_bringup.validation_motion_core import OPERATOR_STOP
from rk_bringup.validation_motion_core import SILENT
from rk_bringup.validation_motion_core import ValidationMotionCore


class ValidationMotionAdapter(Node):
    """只承担实时安全路径的独立进程；pre-arm 对 line_cmd 保持沉默。"""

    TIMER_PERIOD_SEC = 0.02
    STATUS_PERIOD_TICKS = 5

    def __init__(self, arguments):
        super().__init__('validation_motion_adapter')
        self.arguments = arguments
        self.core = ValidationMotionCore(
            line_timeout_ns=int(arguments.line_timeout_sec * 1e9),
            candidate_timeout_ns=int(arguments.candidate_timeout_sec * 1e9),
            gait_timeout_ns=int(arguments.gait_timeout_sec * 1e9),
            motion_ns=int(arguments.motion_sec * 1e9),
            watchdog_ns=int(arguments.watchdog_sec * 1e9),
            executor_lateness_ns=int(arguments.executor_lateness_sec * 1e9),
            max_vx=arguments.max_vx,
            max_yaw=arguments.max_yaw,
            max_lateral_error=arguments.max_lateral_error,
        )
        self.line_publisher = self.create_publisher(
            Twist, '/control/line_cmd', 1,
        )
        self.status_publisher = self.create_publisher(
            String, '/validation/motion_adapter/status', 10,
        )
        self.estop_client = self.create_client(SetBool, '/safety/estop')
        self.arm_service = self.create_service(
            SetBool, '/validation/motion_adapter/arm', self._on_arm,
        )

        latest_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        status_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            Twist, '/navigation/line_follow_cmd_suggested',
            self._on_candidate, latest_qos,
        )
        self.create_subscription(
            LineTrack, '/perception/line_track', self._on_line, latest_qos,
        )
        self.create_subscription(
            Bool, '/gait/control_lock', self._on_gait, latest_qos,
        )
        self.create_subscription(
            String, '/control/cmd_mux_status', self._on_mux, latest_qos,
        )
        self.create_subscription(
            String, '/go2/sdk_motion_status', self._on_sdk, status_qos,
        )

        self.expected_timer_ns = time.monotonic_ns() + int(
            self.TIMER_PERIOD_SEC * 1e9
        )
        self.estop_future = None
        self.estop_target = None
        self.estop_confirmed = None
        self.zero_published = False
        self.zero_publish_count = 0
        self.tick_count = 0
        self.last_decision_reason = None
        self.timer = self.create_timer(self.TIMER_PERIOD_SEC, self._on_timer)

    @staticmethod
    def _now_ns():
        return time.monotonic_ns()

    def _on_candidate(self, message):
        self.core.observe_candidate(
            self._now_ns(), linear_x=message.linear.x,
            angular_z=message.angular.z,
        )

    def _on_line(self, message):
        self.core.observe_line(
            self._now_ns(),
            visible=message.line_visible,
            lateral=message.lateral_error,
            heading=message.heading_error,
            source_sec=int(message.header.stamp.sec),
            source_nanosec=int(message.header.stamp.nanosec),
        )

    def _on_gait(self, message):
        self.core.observe_gait_lock(self._now_ns(), message.data)

    def _on_mux(self, message):
        try:
            payload = json.loads(message.data)
            source = payload.get('active_source', 'unknown')
        except (TypeError, ValueError, json.JSONDecodeError):
            source = 'invalid_status'
        self.core.observe_mux(self._now_ns(), source)

    def _on_sdk(self, message):
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        if (
            payload.get('server_instance_id')
            != self.arguments.server_instance_id
        ):
            return
        event_ns = payload.get('server_monotonic_ns', self._now_ns())
        self.core.observe_sdk(
            payload.get('event', ''), payload.get('ret', -1), event_ns,
        )

    def _on_arm(self, request, response):
        """operator service 只改变一次性 arm 状态；false 始终请求安全停车。"""
        now_ns = self._now_ns()
        if not request.data:
            self.core.operator_stop()
            self._publish_zero()
            self._request_estop(True)
            response.success = True
            response.message = OPERATOR_STOP
            return response
        ok, reason = self.core.request_arm(now_ns)
        if not ok:
            response.success = False
            response.message = reason
            return response
        if not self._request_estop(False):
            self.core.operator_stop()
            response.success = False
            response.message = 'ESTOP_SERVICE_UNAVAILABLE'
            return response
        response.success = True
        response.message = 'ARMING'
        return response

    def _request_estop(self, enabled):
        if self.estop_future is not None:
            return False
        if self.estop_confirmed is bool(enabled):
            return True
        if not self.estop_client.wait_for_service(timeout_sec=0.0):
            return False
        request = SetBool.Request()
        request.data = bool(enabled)
        self.estop_target = bool(enabled)
        self.estop_future = self.estop_client.call_async(request)
        return True

    def _poll_estop(self):
        if self.estop_future is None or not self.estop_future.done():
            return
        future = self.estop_future
        target = self.estop_target
        self.estop_future = None
        try:
            response = future.result()
        except Exception:
            response = None
        if response is None or not response.success:
            self.core.operator_stop()
            self._publish_zero()
            return
        self.estop_confirmed = target
        if target is False:
            self.core.enable_output()

    def _publish_zero(self):
        self.line_publisher.publish(Twist())
        self.zero_published = True
        self.zero_publish_count += 1

    def _publish_status(self, now_ns, lateness_ns, decision):
        payload = self.core.status(now_ns, lateness_ns)
        payload.update({
            'decision': decision.action,
            'decision_reason': decision.reason,
            'server_instance_id': self.arguments.server_instance_id,
        })
        message = String()
        message.data = json.dumps(payload, separators=(',', ':'))
        self.status_publisher.publish(message)

    def _on_timer(self):
        actual_ns = self._now_ns()
        expected_ns = self.expected_timer_ns
        period_ns = int(self.TIMER_PERIOD_SEC * 1e9)
        while self.expected_timer_ns <= actual_ns:
            self.expected_timer_ns += period_ns
        lateness_ns = max(0, actual_ns - expected_ns)

        self._poll_estop()
        decision = self.core.tick(actual_ns, expected_ns)
        if decision.action == MOVE:
            message = Twist()
            message.linear.x = decision.linear_x
            message.linear.y = 0.0
            message.angular.z = decision.angular_z
            self.line_publisher.publish(message)
        elif decision.action != SILENT:
            if self.zero_publish_count < 3:
                self._publish_zero()
            self._request_estop(True)

        self.tick_count += 1
        decision_changed = decision.reason != self.last_decision_reason
        self.last_decision_reason = decision.reason
        if self.tick_count % self.STATUS_PERIOD_TICKS == 0 or decision_changed:
            self._publish_status(actual_ns, lateness_ns, decision)


def parse_arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--server-instance-id', required=True)
    parser.add_argument('--motion-sec', type=float, default=1.0)
    parser.add_argument('--watchdog-sec', type=float, default=1.1)
    parser.add_argument('--line-timeout-sec', type=float, default=0.35)
    parser.add_argument('--candidate-timeout-sec', type=float, default=0.25)
    parser.add_argument('--gait-timeout-sec', type=float, default=0.35)
    parser.add_argument('--executor-lateness-sec', type=float, default=0.10)
    parser.add_argument('--max-vx', type=float, default=0.25)
    parser.add_argument('--max-yaw', type=float, default=0.8)
    parser.add_argument('--max-lateral-error', type=float, default=0.8)
    arguments = parser.parse_args(argv)
    if not (0.0 < arguments.motion_sec <= 1.0):
        parser.error('--motion-sec must be in (0, 1.0]')
    if not (arguments.motion_sec < arguments.watchdog_sec <= 1.2):
        parser.error('--watchdog-sec must be > motion and <= 1.2')
    return arguments


def main(argv=None):
    arguments = parse_arguments(argv)
    rclpy.init(args=[])
    node = ValidationMotionAdapter(arguments)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Ctrl-C 是受管进程的正常 teardown，不应污染验收日志。
        pass
    finally:
        if node.core.armed:
            node.core.operator_stop()
            node._publish_zero()
            node._request_estop(True)
        node.destroy_node()
        rclpy.shutdown()
