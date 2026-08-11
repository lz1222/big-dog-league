#!/usr/bin/env python3
"""ROS 全局步态 owner：串行确认最终零速后，请求唯一 SDK server 切步态。"""

import json
import socket
import threading
import time
import uuid

import rclpy
from geometry_msgs.msg import Twist
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from rclpy.qos import ReliabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from global_gait_owner_core import (CLASSIC_ESTABLISHED_BY_VALIDATED_SEQUENCE,
                                    CLASSIC_VALIDATED_SEQUENCE_SOURCE, FAILED,
                                    FREE_READY, UNKNOWN)
from global_gait_owner_core import GlobalGaitOwnerCore, SdkStatusSequenceGuard


class GlobalGaitOwner(Node):
    """持有全局步态状态与转换 mutex；实际 SportClient 仅存在于 UDP server。"""

    def __init__(self):
        super().__init__('global_gait_owner')
        self.callback_group = ReentrantCallbackGroup()
        self.udp_host = str(self.declare_parameter(
            'udp_host', '127.0.0.1').value)
        self.udp_port = int(self.declare_parameter('udp_port', 15001).value)
        self.final_cmd_topic = str(self.declare_parameter(
            'final_cmd_topic', '/navigation/cmd_vel').value)
        self.sdk_status_topic = str(self.declare_parameter(
            'sdk_status_topic', '/go2/sdk_motion_status').value)
        self.expected_server_instance_id = str(self.declare_parameter(
            'expected_server_instance_id', '').value).strip()
        self.status_topic = str(self.declare_parameter(
            'status_topic', '/gait/mode_status').value)
        self.lock_topic = str(self.declare_parameter(
            'lock_request_topic',
            '/gait/control_lock_req/global_gait_owner').value)
        self.zero_epsilon = float(self.declare_parameter(
            'zero_epsilon', 0.001).value)
        self.zero_samples_required = int(self.declare_parameter(
            'zero_samples_required', 3).value)
        self.zero_timeout_sec = float(self.declare_parameter(
            'zero_timeout_sec', 2.0).value)
        self.final_cmd_stale_sec = float(self.declare_parameter(
            'final_cmd_stale_sec', 0.30).value)
        self.transition_timeout_sec = float(self.declare_parameter(
            'transition_timeout_sec', 3.0).value)
        self.software_smoke_mode = bool(self.declare_parameter(
            'software_smoke_mode', False).value)
        self._validate_parameters()

        self.core = GlobalGaitOwnerCore()
        # UDP forwarder 会重放启动历史；owner 只能按当前实例的递增序号消费，
        # 否则旧的 GAIT_REQUESTED 会在已验证 Classic 后再次锁车。
        self._status_sequence_guard = SdkStatusSequenceGuard(
            self.expected_server_instance_id)
        self._transition_mutex = threading.Lock()
        self._state_mutex = threading.Lock()
        self._zero_condition = threading.Condition(self._state_mutex)
        self._ack_event = threading.Event()
        self._zero_samples = 0
        self._last_final_cmd_time = None
        # UNKNOWN/FAILED 默认保持 movement lock；FREE_READY 也保持到切回 CLASSIC。
        self._lock_held = not self.software_smoke_mode
        self._last_request_success = False
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.status_publisher = self.create_publisher(
            String, self.status_topic, latched_qos)
        self.lock_publisher = self.create_publisher(
            Bool, self.lock_topic, latched_qos)
        self.create_subscription(
            Twist, self.final_cmd_topic, self._on_final_cmd, 20,
            callback_group=self.callback_group)
        self.create_subscription(
            String, self.sdk_status_topic, self._on_sdk_status, latched_qos,
            callback_group=self.callback_group)
        self.create_service(
            Trigger, '/gait/ensure_classic', self._ensure_classic,
            callback_group=self.callback_group)
        self.create_service(
            Trigger, '/gait/ensure_free', self._ensure_free,
            callback_group=self.callback_group)
        self.create_service(
            Trigger, '/gait/stop_and_hold', self._stop_and_hold,
            callback_group=self.callback_group)
        self.create_timer(0.10, self._publish_state,
                          callback_group=self.callback_group)

        if self.software_smoke_mode:
            self.core.state = CLASSIC_ESTABLISHED_BY_VALIDATED_SEQUENCE
            self.core.target = 'CLASSIC'
            self.core.verification_source = 'software_smoke'
        self._publish_state()

    def _validate_parameters(self):
        """限制时间与零速阈值，避免命令行把有界转换变成无限等待。"""
        if not self.udp_host or not 0 < self.udp_port <= 65535:
            raise ValueError('invalid UDP endpoint')
        # smoke 没有真实 UDP server；正式链必须显式绑定启动 nonce。
        if not self.software_smoke_mode and not self.expected_server_instance_id:
            raise ValueError('expected_server_instance_id is required')
        if not 1 <= self.zero_samples_required <= 20:
            raise ValueError('zero_samples_required must be in range 1..20')
        for name, value in (
            ('zero_epsilon', self.zero_epsilon),
            ('zero_timeout_sec', self.zero_timeout_sec),
            ('final_cmd_stale_sec', self.final_cmd_stale_sec),
            ('transition_timeout_sec', self.transition_timeout_sec),
        ):
            if value <= 0.0:
                raise ValueError('{} must be positive'.format(name))
        if self.zero_timeout_sec > 5.0 or self.transition_timeout_sec > 5.0:
            raise ValueError('gait transition timeouts must not exceed 5s')

    def _on_final_cmd(self, message):
        now = time.monotonic()
        values = (message.linear.x, message.linear.y, message.angular.z)
        with self._zero_condition:
            self._last_final_cmd_time = now
            if all(abs(float(value)) <= self.zero_epsilon for value in values):
                self._zero_samples += 1
            else:
                self._zero_samples = 0
            self._zero_condition.notify_all()

    def _on_sdk_status(self, message):
        """跟随唯一 server 的全部步态/动作事件，旁路请求也必须进入全局锁。"""
        try:
            status = json.loads(message.data)
        except (TypeError, ValueError):
            return
        with self._state_mutex:
            if not self._status_sequence_guard.accept(
                    status.get('server_instance_id'), status.get('sequence')):
                return
        event = status.get('event')
        ret = status.get('ret')
        reason = status.get('reason', '')
        fields = {}
        for item in str(reason).split(';'):
            if '=' in item:
                key, value = item.split('=', 1)
                fields[key] = value
        if event == 'CLASSIC_VERIFIED' and ret == 0:
            with self._state_mutex:
                if not self.core.request_id:
                    self.core.observe_startup_classic(
                        0, fields.get('verification_source', 'none'))
                    self._lock_held = (
                        self.core.state != CLASSIC_ESTABLISHED_BY_VALIDATED_SEQUENCE)
            return
        if event == 'CLASSIC_COMMAND_ACK':
            with self._state_mutex:
                if self.core.observe_classic_command_ack(ret, reason):
                    self._lock_held = True
            self._publish_state()
            return
        if event == 'GAIT_REQUESTED':
            request_id = fields.get('request_id', '')
            target = fields.get('target', '')
            with self._state_mutex:
                if self.core.begin(request_id, target):
                    self._lock_held = True
            self._publish_state()
            return
        if event == 'ACTION_REQUESTED':
            with self._state_mutex:
                self.core.state = UNKNOWN
                self.core.target = fields.get('action', '')
                self.core.verification_source = 'none'
                self._lock_held = True
            self._publish_state()
            return
        if event == 'ACTION_FAILED':
            with self._state_mutex:
                self.core.fail('action_failed ret={}'.format(ret))
                self._lock_held = True
            self._publish_state()
            return
        if event == 'ACTION_READY' and ret == 0:
            with self._state_mutex:
                self.core.observe_startup_classic(
                    0, fields.get('verification_source', 'none'))
                self._lock_held = (
                    self.core.state != CLASSIC_ESTABLISHED_BY_VALIDATED_SEQUENCE)
            self._publish_state()
            return
        if event not in ('GAIT_READY', 'GAIT_FAILED'):
            return
        with self._state_mutex:
            if self.core.observe_ack(event, ret, reason):
                self._last_request_success = event == 'GAIT_READY' and ret == 0
                # 启动 Classic 的 GAIT_READY 不经过 /gait/ensure_classic 服务。
                # 一旦当前 request_id 已由已验收序列确认，必须在此处解除锁；
                # 否则 core 已就绪而 arbiter 永久停车，且不能靠重发 Classic 掩盖。
                if self.core.state == CLASSIC_ESTABLISHED_BY_VALIDATED_SEQUENCE:
                    self._lock_held = False
                self._ack_event.set()
        self._publish_state()

    def _wait_for_final_zero(self):
        """movement lock 生效后要求新鲜、连续最终零速，不能只看候选命令。"""
        deadline = time.monotonic() + self.zero_timeout_sec
        with self._zero_condition:
            self._zero_samples = 0
            while time.monotonic() < deadline:
                age = None if self._last_final_cmd_time is None else (
                    time.monotonic() - self._last_final_cmd_time)
                if (
                    age is not None
                    and age <= self.final_cmd_stale_sec
                    and self._zero_samples >= self.zero_samples_required
                ):
                    return True
                self._zero_condition.wait(timeout=min(
                    0.05, max(0.0, deadline - time.monotonic())))
        return False

    def _request(self, target):
        with self._transition_mutex:
            with self._state_mutex:
                if (target == 'CLASSIC'
                        and self.core.state == CLASSIC_ESTABLISHED_BY_VALIDATED_SEQUENCE):
                    self._lock_held = False
                    return True, 'CLASSIC already established by validated sequence'
                if target == 'FREE' and self.core.state == FREE_READY:
                    self._lock_held = True
                    return True, 'FREE already ready (command_ack)'
                self._lock_held = True
            self._publish_state()
            if not self._wait_for_final_zero():
                with self._state_mutex:
                    self.core.fail('final_cmd_zero_timeout')
                return False, 'final /navigation/cmd_vel zero not confirmed'

            request_id = uuid.uuid4().hex
            with self._state_mutex:
                self.core.begin(request_id, target)
                self._ack_event.clear()
                self._last_request_success = False
            if self.software_smoke_mode:
                fake_reason = (
                    'request_id={};target={};phase=ready;'
                    'verification_source=software_smoke'
                ).format(request_id, target)
                with self._state_mutex:
                    self.core.observe_ack('GAIT_READY', 0, fake_reason)
                    self.core.verification_source = 'software_smoke'
                    self._last_request_success = True
                    self._ack_event.set()
            else:
                payload = 'GAIT {} {}'.format(request_id, target).encode('ascii')
                self.sock.sendto(payload, (self.udp_host, self.udp_port))

            if not self._ack_event.wait(self.transition_timeout_sec):
                with self._state_mutex:
                    self.core.fail('gait_ack_timeout')
                return False, 'SDK gait ACK timeout'
            with self._state_mutex:
                success = self._last_request_success
                state = self.core.state
                if (success and target == 'CLASSIC'
                        and state == CLASSIC_ESTABLISHED_BY_VALIDATED_SEQUENCE):
                    self._lock_held = False
                elif target in ('FREE', 'HOLD') or not success:
                    self._lock_held = True
                reason = self.core.failure_reason
            self._publish_state()
            return success, state if success else reason

    def _ensure_classic(self, request, response):
        response.success, response.message = self._request('CLASSIC')
        return response

    def _ensure_free(self, request, response):
        response.success, response.message = self._request('FREE')
        return response

    def _stop_and_hold(self, request, response):
        response.success, response.message = self._request('HOLD')
        return response

    def _publish_state(self):
        with self._state_mutex:
            payload = {
                'state': self.core.state,
                'target': self.core.target,
                'request_id': self.core.request_id,
                'verification_source': self.core.verification_source,
                'failure_reason': self.core.failure_reason,
                'movement_lock_held': self._lock_held,
                'timestamp_monotonic_ns': time.monotonic_ns(),
            }
            locked = self._lock_held or self.core.state == FAILED
        status = String()
        status.data = json.dumps(payload, separators=(',', ':'), allow_nan=False)
        self.status_publisher.publish(status)
        lock_message = Bool()
        lock_message.data = locked
        self.lock_publisher.publish(lock_message)

    def destroy_node(self):
        try:
            self.sock.close()
        finally:
            return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GlobalGaitOwner()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
