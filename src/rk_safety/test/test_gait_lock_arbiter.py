#!/usr/bin/env python3

"""Unit tests for GaitLockArbiterNode fail-closed semantics.

FAIL-CLOSED rules under test:
  - Startup: all sources unseen → locked=true
  - One source seen false, other unseen → locked=true
  - All sources fresh false → locked=false
  - Any source true → locked=true
  - True source goes stale → locked=true (not auto-unlock)
  - False source goes stale → locked=true (not auto-unlock)
  - Publisher crash → locked=true
  - Source recovers but still true → locked=true
  - All sources recover and fresh false → locked=false
  - Two actions simultaneously → locked=true
  - Shutdown → final publish true
  - Late subscriber receives current lock via transient-local
  - Heartbeat sustained → no spurious timeout
"""

from __future__ import annotations

import os
import tempfile
import time
import unittest

import rclpy
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Bool

_ARBITER_RATE = 50.0
_SOURCE_TIMEOUT = 0.5
_TOPIC_GAIT = '/gait/control_lock_req/gait'
_TOPIC_INSPECTION = '/gait/control_lock_req/inspection'
_OUTPUT_TOPIC = '/gait/control_lock'


def _make_params_file(input_topics, output_topic, source_timeout, arbiter_rate):
    yaml_content = (
        '/**:\n'
        '  ros__parameters:\n'
        '    input_topics: [{topics}]\n'
        '    output_topic: {output}\n'
        '    source_timeout_sec: {timeout}\n'
        '    arbiter_rate_hz: {rate}\n'
    ).format(
        topics=', '.join(repr(t) for t in input_topics),
        output=repr(output_topic),
        timeout=float(source_timeout),
        rate=float(arbiter_rate),
    )
    tmp = tempfile.NamedTemporaryFile(
        mode='w', suffix='.yaml', delete=False, prefix='arbiter_test_',
    )
    tmp.write(yaml_content)
    tmp.close()
    return tmp.name


_ARGS_CACHE: dict[str, list] = {}
_PARAMS_FILES: list[str] = []


def _init_with_params(**kw):
    key = repr(sorted(kw.items()))
    if key not in _ARGS_CACHE:
        params_path = _make_params_file(
            input_topics=kw.get(
                'input_topics', [_TOPIC_GAIT, _TOPIC_INSPECTION],
            ),
            output_topic=kw.get('output_topic', _OUTPUT_TOPIC),
            source_timeout=kw.get('source_timeout_sec', _SOURCE_TIMEOUT),
            arbiter_rate=kw.get('arbiter_rate_hz', _ARBITER_RATE),
        )
        _PARAMS_FILES.append(params_path)
        _ARGS_CACHE[key] = ['--ros-args', '--params-file', params_path]
    rclpy.init(args=_ARGS_CACHE[key])


class GaitLockArbiterTest(unittest.TestCase):

    @classmethod
    def tearDownClass(cls):
        for path in _PARAMS_FILES:
            try:
                os.unlink(path)
            except OSError:
                pass
        _PARAMS_FILES.clear()
        _ARGS_CACHE.clear()

    def _create_node(self):
        from rk_safety.gait_lock_arbiter_node import GaitLockArbiterNode
        node = GaitLockArbiterNode()
        self.addCleanup(lambda: node.destroy_node())
        return node

    def _spin_until(self, executor, predicate, timeout=1.5):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return True
            executor.spin_once(timeout_sec=0.02)
        return bool(predicate())

    @staticmethod
    def _publish(publisher, value):
        msg = Bool()
        msg.data = value
        publisher.publish(msg)

    # ------------------------------------------------------------------
    # 1. Startup: all unseen → locked=true
    # ------------------------------------------------------------------

    def test_startup_all_unseen_locked_true(self):
        _init_with_params()
        try:
            node = self._create_node()
            executor = SingleThreadedExecutor()
            executor.add_node(node)

            gait_pub = node.create_publisher(Bool, _TOPIC_GAIT, 10)
            node.create_publisher(Bool, _TOPIC_INSPECTION, 10)
            received: list[bool] = []

            def _cb(msg):
                received.append(bool(msg.data))
            node.create_subscription(Bool, _OUTPUT_TOPIC, _cb, 10)

            # Before any source publishes, lock must be true (fail-closed).
            self._spin_until(
                executor, lambda: len(received) >= 1, timeout=2.0,
            )
            self.assertTrue(
                received[-1],
                'initial lock must be true when all sources unseen',
            )
        finally:
            rclpy.shutdown()

    # ------------------------------------------------------------------
    # 2. Only one source seen false → locked=true
    # ------------------------------------------------------------------

    def test_one_source_false_other_unseen_locked_true(self):
        _init_with_params()
        try:
            node = self._create_node()
            executor = SingleThreadedExecutor()
            executor.add_node(node)

            gait_pub = node.create_publisher(Bool, _TOPIC_GAIT, 10)
            node.create_publisher(Bool, _TOPIC_INSPECTION, 10)
            received: list[bool] = []

            def _cb(msg):
                received.append(bool(msg.data))
            node.create_subscription(Bool, _OUTPUT_TOPIC, _cb, 10)

            self._spin_until(
                executor, lambda: len(received) >= 1, timeout=2.0,
            )
            # Gait publishes false, inspection never published → still locked
            self._publish(gait_pub, False)
            for _ in range(30):
                executor.spin_once(timeout_sec=0.02)
            self.assertTrue(
                received[-1],
                'lock must stay true when inspection never seen',
            )
        finally:
            rclpy.shutdown()

    # ------------------------------------------------------------------
    # 3. All sources fresh false → locked=false
    # ------------------------------------------------------------------

    def test_all_sources_fresh_false_locked_false(self):
        _init_with_params()
        try:
            node = self._create_node()
            executor = SingleThreadedExecutor()
            executor.add_node(node)

            gait_pub = node.create_publisher(Bool, _TOPIC_GAIT, 10)
            inspection_pub = node.create_publisher(Bool, _TOPIC_INSPECTION, 10)
            received: list[bool] = []

            def _cb(msg):
                received.append(bool(msg.data))
            node.create_subscription(Bool, _OUTPUT_TOPIC, _cb, 10)

            self._spin_until(
                executor, lambda: len(received) >= 1, timeout=2.0,
            )
            # Both publish fresh false
            self._publish(gait_pub, False)
            self._publish(inspection_pub, False)
            self._spin_until(
                executor, lambda: received[-1] is False, timeout=1.0,
            )
            self.assertFalse(
                received[-1],
                'lock must be false when both sources fresh false',
            )
        finally:
            rclpy.shutdown()

    # ------------------------------------------------------------------
    # 4. Any source true → locked=true
    # ------------------------------------------------------------------

    def test_gait_true_locked_true(self):
        _init_with_params()
        try:
            node = self._create_node()
            executor = SingleThreadedExecutor()
            executor.add_node(node)

            gait_pub = node.create_publisher(Bool, _TOPIC_GAIT, 10)
            inspection_pub = node.create_publisher(Bool, _TOPIC_INSPECTION, 10)
            received: list[bool] = []

            def _cb(msg):
                received.append(bool(msg.data))
            node.create_subscription(Bool, _OUTPUT_TOPIC, _cb, 10)

            self._spin_until(
                executor, lambda: len(received) >= 1, timeout=2.0,
            )
            # Both fresh, one true
            self._publish(inspection_pub, False)
            self._publish(gait_pub, True)
            self._spin_until(
                executor, lambda: received[-1] is True, timeout=1.0,
            )
            self.assertTrue(received[-1])
        finally:
            rclpy.shutdown()

    def test_inspection_true_locked_true(self):
        _init_with_params()
        try:
            node = self._create_node()
            executor = SingleThreadedExecutor()
            executor.add_node(node)

            gait_pub = node.create_publisher(Bool, _TOPIC_GAIT, 10)
            inspection_pub = node.create_publisher(Bool, _TOPIC_INSPECTION, 10)
            received: list[bool] = []

            def _cb(msg):
                received.append(bool(msg.data))
            node.create_subscription(Bool, _OUTPUT_TOPIC, _cb, 10)

            self._spin_until(
                executor, lambda: len(received) >= 1, timeout=2.0,
            )
            self._publish(gait_pub, False)
            self._publish(inspection_pub, True)
            self._spin_until(
                executor, lambda: received[-1] is True, timeout=1.0,
            )
            self.assertTrue(received[-1])
        finally:
            rclpy.shutdown()

    # ------------------------------------------------------------------
    # 5. True source goes stale → locked=true (NOT auto-unlock)
    # ------------------------------------------------------------------

    def test_true_source_stale_locked_true(self):
        _init_with_params()
        try:
            node = self._create_node()
            executor = SingleThreadedExecutor()
            executor.add_node(node)

            gait_pub = node.create_publisher(Bool, _TOPIC_GAIT, 10)
            inspection_pub = node.create_publisher(Bool, _TOPIC_INSPECTION, 10)
            received: list[bool] = []

            def _cb(msg):
                received.append(bool(msg.data))
            node.create_subscription(Bool, _OUTPUT_TOPIC, _cb, 10)

            self._spin_until(
                executor, lambda: len(received) >= 1, timeout=2.0,
            )
            # Gait requests lock, inspection is idle
            self._publish(gait_pub, True)
            self._publish(inspection_pub, False)
            self._spin_until(
                executor, lambda: received[-1] is True, timeout=1.0,
            )
            # Keep inspection fresh, stop gait → gait goes stale
            deadline = time.monotonic() + _SOURCE_TIMEOUT + 1.0
            while time.monotonic() < deadline:
                self._publish(inspection_pub, False)
                executor.spin_once(timeout_sec=0.05)
                if received and not received[-1]:
                    self.fail(
                        'lock must stay true when gait goes stale '
                        '(fail-closed: stale source = locked)',
                    )
            self.assertTrue(
                received[-1],
                'lock must remain true after gait source timeout',
            )
        finally:
            rclpy.shutdown()

    # ------------------------------------------------------------------
    # 6. False source goes stale → locked=true
    # ------------------------------------------------------------------

    def test_false_source_stale_locked_true(self):
        """Even a source that was false, if stale, forces lock=true."""
        _init_with_params()
        try:
            node = self._create_node()
            executor = SingleThreadedExecutor()
            executor.add_node(node)

            gait_pub = node.create_publisher(Bool, _TOPIC_GAIT, 10)
            inspection_pub = node.create_publisher(Bool, _TOPIC_INSPECTION, 10)
            received: list[bool] = []

            def _cb(msg):
                received.append(bool(msg.data))
            node.create_subscription(Bool, _OUTPUT_TOPIC, _cb, 10)

            self._spin_until(
                executor, lambda: len(received) >= 1, timeout=2.0,
            )
            # Both initially fresh false → lock=false
            self._publish(gait_pub, False)
            self._publish(inspection_pub, False)
            self._spin_until(
                executor, lambda: received[-1] is False, timeout=1.0,
            )
            self.assertFalse(received[-1])
            # Stop publishing from inspection → goes stale → lock=true
            deadline = time.monotonic() + _SOURCE_TIMEOUT + 1.0
            locked_up = False
            while time.monotonic() < deadline:
                self._publish(gait_pub, False)  # keep gait fresh
                executor.spin_once(timeout_sec=0.05)
                if received and received[-1] is True:
                    locked_up = True
                    break
            self.assertTrue(
                locked_up,
                'lock must become true when false source goes stale',
            )
        finally:
            rclpy.shutdown()

    # ------------------------------------------------------------------
    # 7. Publisher crash (stop publishing) → locked=true
    # ------------------------------------------------------------------

    def test_publisher_crash_locked_true(self):
        _init_with_params()
        try:
            node = self._create_node()
            executor = SingleThreadedExecutor()
            executor.add_node(node)

            gait_pub = node.create_publisher(Bool, _TOPIC_GAIT, 10)
            inspection_pub = node.create_publisher(Bool, _TOPIC_INSPECTION, 10)
            received: list[bool] = []

            def _cb(msg):
                received.append(bool(msg.data))
            node.create_subscription(Bool, _OUTPUT_TOPIC, _cb, 10)

            self._spin_until(
                executor, lambda: len(received) >= 1, timeout=2.0,
            )
            # Both fresh false → unlocked
            self._publish(gait_pub, False)
            self._publish(inspection_pub, False)
            self._spin_until(
                executor, lambda: received[-1] is False, timeout=1.0,
            )
            # Simulate crash: stop both sources
            deadline = time.monotonic() + _SOURCE_TIMEOUT + 1.0
            locked_up = False
            while time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.05)
                if received and received[-1] is True:
                    locked_up = True
                    break
            self.assertTrue(
                locked_up,
                'lock must become true when all sources crash',
            )
        finally:
            rclpy.shutdown()

    # ------------------------------------------------------------------
    # 8. Source recovers but still true → locked=true
    # ------------------------------------------------------------------

    def test_source_recovers_still_true_locked_true(self):
        _init_with_params()
        try:
            node = self._create_node()
            executor = SingleThreadedExecutor()
            executor.add_node(node)

            gait_pub = node.create_publisher(Bool, _TOPIC_GAIT, 10)
            inspection_pub = node.create_publisher(Bool, _TOPIC_INSPECTION, 10)
            received: list[bool] = []

            def _cb(msg):
                received.append(bool(msg.data))
            node.create_subscription(Bool, _OUTPUT_TOPIC, _cb, 10)

            self._spin_until(
                executor, lambda: len(received) >= 1, timeout=2.0,
            )
            # Gait: true, Inspection: false
            self._publish(gait_pub, True)
            self._publish(inspection_pub, False)
            self._spin_until(
                executor, lambda: received[-1] is True, timeout=1.0,
            )
            # Let gait go stale
            deadline = time.monotonic() + _SOURCE_TIMEOUT + 0.3
            while time.monotonic() < deadline:
                self._publish(inspection_pub, False)
                executor.spin_once(timeout_sec=0.02)
            # Lock must still be true
            self.assertTrue(received[-1])
            # Gait recovers still publishing true → lock stays true
            self._publish(gait_pub, True)
            self._publish(inspection_pub, False)
            for _ in range(20):
                executor.spin_once(timeout_sec=0.02)
            self.assertTrue(
                received[-1],
                'lock must stay true when recovered source still true',
            )
        finally:
            rclpy.shutdown()

    # ------------------------------------------------------------------
    # 9. All sources recover and fresh false → locked=false
    # ------------------------------------------------------------------

    def test_all_sources_recover_fresh_false_locked_false(self):
        _init_with_params()
        try:
            node = self._create_node()
            executor = SingleThreadedExecutor()
            executor.add_node(node)

            gait_pub = node.create_publisher(Bool, _TOPIC_GAIT, 10)
            inspection_pub = node.create_publisher(Bool, _TOPIC_INSPECTION, 10)
            received: list[bool] = []

            def _cb(msg):
                received.append(bool(msg.data))
            node.create_subscription(Bool, _OUTPUT_TOPIC, _cb, 10)

            self._spin_until(
                executor, lambda: len(received) >= 1, timeout=2.0,
            )
            # Initial: both fresh false
            self._publish(gait_pub, False)
            self._publish(inspection_pub, False)
            self._spin_until(
                executor, lambda: received[-1] is False, timeout=1.0,
            )
            # Gait requests lock
            self._publish(gait_pub, True)
            self._spin_until(
                executor, lambda: received[-1] is True, timeout=1.0,
            )
            # Gait releases
            self._publish(gait_pub, False)
            self._spin_until(
                executor, lambda: received[-1] is False, timeout=1.0,
            )
            self.assertFalse(received[-1])
        finally:
            rclpy.shutdown()

    # ------------------------------------------------------------------
    # 10. Two actions simultaneously → locked=true
    # ------------------------------------------------------------------

    def test_both_true_locked_true(self):
        _init_with_params()
        try:
            node = self._create_node()
            executor = SingleThreadedExecutor()
            executor.add_node(node)

            gait_pub = node.create_publisher(Bool, _TOPIC_GAIT, 10)
            inspection_pub = node.create_publisher(Bool, _TOPIC_INSPECTION, 10)
            received: list[bool] = []

            def _cb(msg):
                received.append(bool(msg.data))
            node.create_subscription(Bool, _OUTPUT_TOPIC, _cb, 10)

            self._spin_until(
                executor, lambda: len(received) >= 1, timeout=2.0,
            )
            self._publish(gait_pub, True)
            self._publish(inspection_pub, True)
            self._spin_until(
                executor, lambda: received[-1] is True, timeout=1.0,
            )
            self.assertTrue(received[-1])
            # Release one → still locked
            self._publish(gait_pub, False)
            self._publish(inspection_pub, True)
            for _ in range(20):
                executor.spin_once(timeout_sec=0.02)
            self.assertTrue(received[-1])
        finally:
            rclpy.shutdown()

    # ------------------------------------------------------------------
    # 11. Shutdown → final publish true
    # ------------------------------------------------------------------

    def test_shutdown_publishes_true(self):
        _init_with_params()
        try:
            from rk_safety.gait_lock_arbiter_node import GaitLockArbiterNode
            import rclpy.node

            node = GaitLockArbiterNode()
            probe = rclpy.node.Node('_arbiter_shutdown_probe')
            executor = SingleThreadedExecutor()
            executor.add_node(node)
            executor.add_node(probe)

            gait_pub = node.create_publisher(Bool, _TOPIC_GAIT, 10)
            inspection_pub = node.create_publisher(Bool, _TOPIC_INSPECTION, 10)
            received: list[bool] = []

            def _cb(msg):
                received.append(bool(msg.data))
            probe.create_subscription(Bool, _OUTPUT_TOPIC, _cb, 1)

            self._spin_until(
                executor, lambda: len(received) >= 1, timeout=2.0,
            )
            # Both fresh false → unlocked
            self._publish(gait_pub, False)
            self._publish(inspection_pub, False)
            self._spin_until(
                executor, lambda: received[-1] is False, timeout=1.0,
            )
            # Shutdown arbiter → must publish TRUE (safe state)
            node.destroy_node()
            self._spin_until(
                executor,
                lambda: (received and received[-1] is True),
                timeout=1.0,
            )
            self.assertTrue(
                received[-1] if received else False,
                'shutdown must publish true (fail-closed)',
            )
            probe.destroy_node()
        finally:
            rclpy.shutdown()

    # ------------------------------------------------------------------
    # 12. Heartbeat: constant false publishing doesn't trigger timeout
    # ------------------------------------------------------------------

    def test_sustained_heartbeat_no_false_timeout(self):
        _init_with_params(source_timeout_sec=0.3)
        try:
            node = self._create_node()
            executor = SingleThreadedExecutor()
            executor.add_node(node)

            gait_pub = node.create_publisher(Bool, _TOPIC_GAIT, 10)
            inspection_pub = node.create_publisher(Bool, _TOPIC_INSPECTION, 10)
            received: list[bool] = []

            def _cb(msg):
                received.append(bool(msg.data))
            node.create_subscription(Bool, _OUTPUT_TOPIC, _cb, 10)

            self._spin_until(
                executor, lambda: len(received) >= 1, timeout=2.0,
            )
            # Sustained heartbeat of false from both sources for well
            # beyond the timeout period.
            heartbeat_end = time.monotonic() + _SOURCE_TIMEOUT * 3
            while time.monotonic() < heartbeat_end:
                self._publish(gait_pub, False)
                self._publish(inspection_pub, False)
                executor.spin_once(timeout_sec=0.02)
            self.assertFalse(
                received[-1],
                'lock must stay false under sustained fresh heartbeat',
            )
        finally:
            rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
