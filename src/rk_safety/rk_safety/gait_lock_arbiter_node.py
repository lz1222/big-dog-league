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
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from std_msgs.msg import Bool, String


class GaitLockArbiterNode(Node):
    """Fail-closed OR-aggregated gait control lock arbiter."""

    def __init__(self):
        super().__init__('gait_lock_arbiter_node')
        self._declare_parameters()
        self._read_parameters()

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
            'Gait lock arbiter ready (fail-closed): output={}, inputs={}, '
            'timeout={:.3f}s, rate={:.1f}Hz'.format(
                self.output_topic,
                list(self.input_topics),
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
        now = time.monotonic()
        locked = False
        fault_reasons = []
        source_statuses = []

        if self._shutting_down:
            return True, ['arbiter_shutting_down'], []

        for topic in self.input_topics:
            info = self._sources.get(topic, {
                'seen': False, 'value': False, 'last_time': 0.0,
            })
            seen = info['seen']
            value = info['value']
            age = now - info['last_time'] if seen else float('inf')
            fresh = seen and age <= self.source_timeout_sec

            status = {
                'topic': topic,
                'seen': seen,
                'value': value,
                'age_sec': round(age, 6) if seen else None,
                'fresh': fresh,
            }

            if not seen:
                fault_reasons.append('source_unseen:{}'.format(topic))
                locked = True
                status['fault'] = 'unseen'
            elif not fresh:
                fault_reasons.append('source_stale:{} age={:.3f}s'.format(
                    topic, age))
                locked = True
                status['fault'] = 'stale'
            elif value:
                locked = True
                status['fault'] = None  # valid true request
            else:
                status['fault'] = None  # valid false release

            source_statuses.append(status)

        if not locked and not fault_reasons:
            fault_reasons.append('all_sources_fresh_false')

        return locked, fault_reasons, source_statuses

    def _publish_lock(self, locked: bool):
        msg = Bool()
        msg.data = locked
        self._lock_pub.publish(msg)
        self._last_published = locked

    def _publish_status(self, locked: bool, fault_reasons, source_statuses):
        payload = {
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
            self._publish_lock(True)
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
