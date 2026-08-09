#!/usr/bin/env python3

"""Gait control lock arbiter: single /gait/control_lock publisher, fail-closed.

Multiple nodes (gait_control, inspection_action_executor) publish their
individual lock requests to separate heartbeat topics.  This arbiter
subscribes to all of them, computes the fail-closed OR, and publishes the
sole authoritative /gait/control_lock.

FAIL-CLOSED SEMANTICS
---------------------
The final /gait/control_lock is **false only when ALL of**:
  - every configured source has been seen at least once,
  - every source is fresh (age <= source_timeout_sec),
  - every source explicitly reports false.

The lock is **true in ALL other cases**:
  - any source has never been seen,
  - any source has timed out (stale),
  - any source reports true,
  - DDS state cannot be confirmed,
  - the arbiter is preparing to shut down.

A late-joining subscriber receives the current lock state via transient-local
QoS.  A companion status topic reports per-source freshness, values, and any
fault reason.
"""

from __future__ import annotations

import json
import threading
import time

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from std_msgs.msg import Bool, String

from rk_safety.gait_lock_profile import evaluate_lock
from rk_safety.gait_lock_profile import required_source_map
from rk_safety.gait_lock_profile import validate_profile


class GaitLockArbiterNode(Node):
    """Fail-closed OR-aggregated gait control lock arbiter."""

    def __init__(self):
        super().__init__('gait_lock_arbiter_node')
        self._declare_parameters()
        self._read_parameters()
        self.add_on_set_parameters_callback(self._reject_startup_parameter_change)

        self._state_lock = threading.RLock()
        # Per-source: {topic: {'seen': bool, 'value': bool, 'last_time': float}}
        self._sources: dict[str, dict] = {}
        self._last_published: bool | None = None
        self._shutting_down = False

        latch_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._lock_pub = self.create_publisher(
            Bool, self.output_topic, latch_qos,
        )
        self._status_pub = self.create_publisher(
            String, self.output_topic + '/status', 10,
        )

        self._subscriptions = []
        for topic in self.input_topics:
            sub = self.create_subscription(
                Bool, topic, self._make_callback(topic), 10,
            )
            self._subscriptions.append(sub)
            with self._state_lock:
                self._sources[topic] = {
                    'seen': False,
                    'value': False,
                    'last_time': 0.0,
                }

        self._timer = self.create_timer(
            1.0 / self.arbiter_rate_hz, self._on_timer_tick,
        )

        # Publish initial safe state (true) immediately.
        self._publish_lock(True)

        self.get_logger().info(
            'Gait lock arbiter ready (fail-closed): profile={}, '
            'start_mission_nodes={}, output={}, inputs={}, required={}, '
            'timeout={:.3f}s, rate={:.1f}Hz'.format(
                self.readiness_profile,
                self.start_mission_nodes,
                self.output_topic,
                list(self.input_topics),
                [
                    topic for topic, required in self.required_sources.items()
                    if required
                ],
                self.source_timeout_sec,
                self.arbiter_rate_hz,
            ),
        )

    def _declare_parameters(self):
        self.declare_parameter(
            'input_topics',
            ['/gait/control_lock_req/gait',
             '/gait/control_lock_req/inspection'],
        )
        self.declare_parameter('output_topic', '/gait/control_lock')
        self.declare_parameter('source_timeout_sec', 2.0)
        self.declare_parameter('arbiter_rate_hz', 10.0)
        self.declare_parameter('readiness_profile', 'production')
        self.declare_parameter('start_mission_nodes', True)

    def _read_parameters(self):
        raw = self.get_parameter('input_topics').value
        if isinstance(raw, list):
            self.input_topics = [str(t).strip().rstrip('/') for t in raw]
        else:
            self.input_topics = [str(raw).strip().rstrip('/')]
        if not self.input_topics or self.input_topics == ['']:
            raise ValueError('input_topics must not be empty')
        self.output_topic = str(
            self.get_parameter('output_topic').value
        ).strip().rstrip('/')
        if not self.output_topic:
            raise ValueError('output_topic must not be empty')
        self.source_timeout_sec = float(
            self.get_parameter('source_timeout_sec').value
        )
        if self.source_timeout_sec <= 0.0:
            raise ValueError('source_timeout_sec must be positive')
        self.arbiter_rate_hz = float(
            self.get_parameter('arbiter_rate_hz').value
        )
        if self.arbiter_rate_hz <= 0.0:
            raise ValueError('arbiter_rate_hz must be positive')
        self.readiness_profile = validate_profile(
            self.get_parameter('readiness_profile').value
        )
        start_mission_nodes = self.get_parameter('start_mission_nodes').value
        if not isinstance(start_mission_nodes, bool):
            raise ValueError('start_mission_nodes must be a boolean')
        self.start_mission_nodes = start_mission_nodes
        self.required_sources = required_source_map(
            self.readiness_profile, self.input_topics,
        )

    def _reject_startup_parameter_change(self, parameters):
        """拒绝运行时切换 profile/图契约，防止 required-set 突变。"""
        startup_only = {
            'input_topics', 'readiness_profile', 'start_mission_nodes',
        }
        changed = sorted(
            parameter.name for parameter in parameters
            if parameter.name in startup_only
        )
        if changed:
            return SetParametersResult(
                successful=False,
                reason='startup-only parameters: {}'.format(','.join(changed)),
            )
        return SetParametersResult(successful=True)

    def _make_callback(self, topic):
        def callback(msg):
            with self._state_lock:
                self._sources[topic] = {
                    'seen': True,
                    'value': bool(msg.data),
                    'last_time': time.monotonic(),
                }
        return callback

    def _compute_lock_state(self):
        """Return (locked, fault_reason, per_source_status).

        Must be called under _state_lock.  Fail-closed: any source that is
        unseen, stale, or reporting true forces the global lock to true.
        """
        locked, reasons, statuses, graph_ok, graph_detail = evaluate_lock(
            profile=self.readiness_profile,
            start_mission_nodes=self.start_mission_nodes,
            input_topics=self.input_topics,
            sources=self._sources,
            now=time.monotonic(),
            source_timeout_sec=self.source_timeout_sec,
            shutting_down=self._shutting_down,
        )
        self._last_graph_ok = graph_ok
        self._last_graph_detail = graph_detail
        return locked, reasons, statuses

    def _publish_lock(self, locked: bool):
        msg = Bool()
        msg.data = locked
        self._lock_pub.publish(msg)
        self._last_published = locked

    def _publish_status(self, locked: bool, fault_reasons, source_statuses):
        payload = {
            'profile': self.readiness_profile,
            'start_mission_nodes': self.start_mission_nodes,
            'profile_graph_consistent': self._last_graph_ok,
            'profile_graph_detail': self._last_graph_detail,
            'global_lock': locked,
            'fault_reason': '; '.join(fault_reasons) if fault_reasons else '',
            'sources': source_statuses,
            'timestamp_monotonic_sec': time.monotonic(),
        }
        msg = String()
        msg.data = json.dumps(payload, allow_nan=False)
        self._status_pub.publish(msg)

    def _on_timer_tick(self):
        with self._state_lock:
            locked, fault_reasons, source_statuses = self._compute_lock_state()

        # Save previous state before publishing so we can detect transitions.
        previous = self._last_published
        # Always publish on every tick so late-joining subscribers and
        # transient-local re-delivery work correctly under fail-closed.
        self._publish_lock(locked)
        if locked != previous:
            self.get_logger().info(
                'gait/control_lock -> {} (reasons: {})'.format(
                    str(locked).lower(),
                    '; '.join(fault_reasons) if fault_reasons else 'none',
                ),
            )

        # Always publish status for observability.
        self._publish_status(locked, fault_reasons, source_statuses)

    def destroy_node(self):
        """Publish safe locked state before shutdown."""
        with self._state_lock:
            self._shutting_down = True
        try:
            # 重复三次仍是幂等布尔锁，不会触发运动；它提高 Foxy 进程销毁前
            # 最终样本进入可靠 writer queue 的确定性。
            for _ in range(3):
                self._publish_lock(True)
            # Foxy publisher 没有 wait_for_all_acked；若立刻销毁 DataWriter，最终
            # fail-closed 样本可能尚未离开进程。节点仍挂在 executor 时执行一个
            # 有界交付窗；_shutting_down 已确保期间定时器也只能发布 true。
            executor = self.executor
            if executor is not None:
                deadline = time.monotonic() + 0.10
                while time.monotonic() < deadline:
                    executor.spin_once(timeout_sec=0.01)
            else:
                time.sleep(0.10)
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = GaitLockArbiterNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
