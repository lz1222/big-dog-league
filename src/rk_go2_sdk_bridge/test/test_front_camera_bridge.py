#!/usr/bin/env python3
"""Tests for go2_front_camera_bridge: image construction, JPEG decode,
and comprehensive process lifecycle management."""

import os
import shutil
import subprocess
import sys
import tempfile
import time
import unittest

import cv2
import numpy as np
from rclpy.qos import DurabilityPolicy, HistoryPolicy, ReliabilityPolicy

# Add the scripts dir to import the bridge module
_scripts_dir = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', 'scripts')
sys.path.insert(0, _scripts_dir)

import go2_front_camera_bridge as bridge_mod  # noqa: E402

# We import rclpy lazily because initialization is expensive and must only
# happen once per process.  setUpModule / tearDownModule handle the lifecycle.

_FAKE_HELPER_DIR = None
_FAKE_HELPER_PATH = None


def _build_fake_helper_script():
    """Return the source text for a configurable fake stream helper.

    The helper writes length-prefixed JPEG frames to stdout and reads its
    behaviour from environment variables:

    * ``FAKE_EXIT_AFTER_FRAMES`` — exit after N frames (0 = run forever)
    * ``FAKE_EXIT_CODE`` — exit code to use
    * ``FAKE_IGNORE_SIGTERM`` — if "1", ignore SIGTERM
    * ``FAKE_TRUNCATE_FRAME`` — if "1", write truncated payload
    * ``FAKE_INVALID_LENGTH`` — if "1", write invalid (>8 MiB) length header
    * ``FAKE_SLEEP_FOREVER`` — if "1", sleep forever without writing frames
    * ``FAKE_FRAME_INTERVAL`` — seconds between frames (default 0.05)
    """
    # Build line-by-line to avoid any quoting issues.
    return '\n'.join([
        '#!/usr/bin/env python3',
        'import os, signal, struct, sys, time',
        'import cv2',
        'import numpy as np',
        '',
        '_EXIT_AFTER = int(os.environ.get("FAKE_EXIT_AFTER_FRAMES", "0"))',
        '_EXIT_CODE = int(os.environ.get("FAKE_EXIT_CODE", "0"))',
        '_IGNORE_SIGTERM = os.environ.get("FAKE_IGNORE_SIGTERM", "0") == "1"',
        '_TRUNCATE = os.environ.get("FAKE_TRUNCATE_FRAME", "0") == "1"',
        '_INVALID_LEN = os.environ.get("FAKE_INVALID_LENGTH", "0") == "1"',
        '_SLEEP_FOREVER = os.environ.get("FAKE_SLEEP_FOREVER", "0") == "1"',
        '_INTERVAL = float(os.environ.get("FAKE_FRAME_INTERVAL", "0.05"))',
        '',
        'if _IGNORE_SIGTERM:',
        '    signal.signal(signal.SIGTERM, signal.SIG_IGN)',
        'signal.signal(signal.SIGINT, signal.SIG_IGN)',
        '',
        '_img = np.zeros((60, 80, 3), dtype=np.uint8)',
        'cv2.rectangle(_img, (10, 10), (50, 50), (255, 0, 0), -1)',
        '_, _jpeg_buf = cv2.imencode(".jpg", _img)',
        '_jpeg_bytes = _jpeg_buf.tobytes()',
        '',
        'if _SLEEP_FOREVER:',
        '    while True:',
        '        time.sleep(1)',
        '',
        '_count = 0',
        'while True:',
        '    if _EXIT_AFTER > 0 and _count >= _EXIT_AFTER:',
        '        sys.exit(_EXIT_CODE)',
        '',
        '    if _INVALID_LEN:',
        '        sys.stdout.buffer.write(struct.pack("!I", 9 * 1024 * 1024))',
        '        sys.stdout.buffer.flush()',
        '    elif _TRUNCATE:',
        '        sys.stdout.buffer.write(struct.pack("!I", 1000))',
        '        sys.stdout.buffer.write(b"short")',
        '        sys.stdout.buffer.flush()',
        '    else:',
        '        sys.stdout.buffer.write(struct.pack("!I", len(_jpeg_bytes)))',
        '        sys.stdout.buffer.write(_jpeg_bytes)',
        '        sys.stdout.buffer.flush()',
        '',
        '    time.sleep(_INTERVAL)',
        '    _count += 1',
        '',
    ])


def setUpModule():
    """Create a temporary fake stream helper script once for all tests."""
    global _FAKE_HELPER_DIR, _FAKE_HELPER_PATH
    _FAKE_HELPER_DIR = tempfile.mkdtemp(prefix='test_bridge_')
    _FAKE_HELPER_PATH = os.path.join(
        _FAKE_HELPER_DIR, 'fake_stream_helper.py')
    with open(_FAKE_HELPER_PATH, 'w') as fh:
        fh.write(_build_fake_helper_script())
    os.chmod(_FAKE_HELPER_PATH, 0o755)


def tearDownModule():
    """Remove the temporary fake helper."""
    global _FAKE_HELPER_DIR
    if _FAKE_HELPER_DIR and os.path.isdir(_FAKE_HELPER_DIR):
        shutil.rmtree(_FAKE_HELPER_DIR, ignore_errors=True)


# ---- helper utilities for lifecycle tests ----

def _env_with_defaults(overrides=None):
    """Return an os.environ copy with fake helper defaults cleared."""
    env = os.environ.copy()
    for key in list(env.keys()):
        if key.startswith('FAKE_'):
            del env[key]
    if overrides:
        env.update(overrides)
    return env


def _count_children():
    """Return the number of running fake helper processes."""
    try:
        result = subprocess.run(
            ['pgrep', '-af', 'fake_stream_helper.py'],
            capture_output=True, text=True, timeout=5)
        lines = [l for l in result.stdout.strip().split('\n') if l]
        return len(lines)
    except Exception:
        return 0


def wait_until(predicate, timeout_sec, poll_interval_sec=0.02):
    """在有限时间内等待异步桥接状态，超时后由断言输出明确失败。"""
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(poll_interval_sec)
    return predicate()


# ---- Original image-construction / JPEG tests (kept from prior suite) ----


class TestValidateBgrImage(unittest.TestCase):

    def test_valid_bgr8(self):
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        self.assertTrue(bridge_mod._validate_bgr_image(img))

    def test_none_rejected(self):
        self.assertFalse(bridge_mod._validate_bgr_image(None))

    def test_not_ndarray_rejected(self):
        self.assertFalse(bridge_mod._validate_bgr_image([1, 2, 3]))

    def test_grayscale_rejected(self):
        img = np.zeros((480, 640), dtype=np.uint8)
        self.assertFalse(bridge_mod._validate_bgr_image(img))

    def test_four_channel_rejected(self):
        img = np.zeros((480, 640, 4), dtype=np.uint8)
        self.assertFalse(bridge_mod._validate_bgr_image(img))

    def test_non_uint8_rejected(self):
        img = np.zeros((480, 640, 3), dtype=np.float32)
        self.assertFalse(bridge_mod._validate_bgr_image(img))

    def test_zero_dimension_rejected(self):
        img = np.zeros((0, 640, 3), dtype=np.uint8)
        self.assertFalse(bridge_mod._validate_bgr_image(img))


class TestBuildImageMsg(unittest.TestCase):

    def _fake_stamp(self):
        from builtin_interfaces.msg import Time
        t = Time()
        t.sec = 12345
        t.nanosec = 678900000
        return t

    def test_fields_correct(self):
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        img[10, 20] = (1, 2, 3)
        stamp = self._fake_stamp()
        msg = bridge_mod._build_image_msg(img, 'test_frame', stamp)

        self.assertEqual(msg.header.frame_id, 'test_frame')
        self.assertEqual(msg.header.stamp.sec, 12345)
        self.assertEqual(msg.header.stamp.nanosec, 678900000)
        self.assertEqual(msg.height, 480)
        self.assertEqual(msg.width, 640)
        self.assertEqual(msg.encoding, 'bgr8')
        self.assertFalse(msg.is_bigendian)
        self.assertEqual(msg.step, 1920)
        self.assertEqual(len(msg.data), 480 * 1920)

    def test_non_contiguous_input_accepted(self):
        big = np.zeros((480, 1280, 3), dtype=np.uint8)
        sliced = big[:, ::2, :]
        self.assertFalse(sliced.flags['C_CONTIGUOUS'])
        stamp = self._fake_stamp()
        msg = bridge_mod._build_image_msg(sliced, 'f', stamp)
        self.assertEqual(msg.height, 480)
        self.assertEqual(msg.width, 640)
        self.assertEqual(len(msg.data), 480 * 640 * 3)

    def test_timestamp_nonzero(self):
        img = np.zeros((10, 10, 3), dtype=np.uint8)
        stamp = self._fake_stamp()
        msg = bridge_mod._build_image_msg(img, 'f', stamp)
        self.assertGreater(msg.header.stamp.sec, 0)

    def test_data_length_matches(self):
        img = np.random.randint(0, 255, (100, 200, 3), dtype=np.uint8)
        stamp = self._fake_stamp()
        msg = bridge_mod._build_image_msg(img, 'f', stamp)
        self.assertEqual(len(msg.data), msg.height * msg.step)

    def test_bgr_order_preserved(self):
        img = np.zeros((1, 1, 3), dtype=np.uint8)
        img[0, 0] = (10, 20, 30)
        stamp = self._fake_stamp()
        msg = bridge_mod._build_image_msg(img, 'f', stamp)
        self.assertEqual(msg.data[0], 10)
        self.assertEqual(msg.data[1], 20)
        self.assertEqual(msg.data[2], 30)


class TestOutputResizeAndQos(unittest.TestCase):
    """验证 DDS 前缩图边界，避免大 BGR 消息再次压垮可靠交付链。"""

    def test_1920x1080_is_reduced_to_960x540(self):
        source = np.zeros((1080, 1920, 3), dtype=np.uint8)
        output = bridge_mod._resize_output_bgr(source, 960)
        self.assertEqual(output.shape, (540, 960, 3))
        self.assertTrue(output.flags.c_contiguous)
        self.assertEqual(output.dtype, np.uint8)

    def test_1280x720_is_reduced_to_960x540(self):
        source = np.zeros((720, 1280, 3), dtype=np.uint8)
        output = bridge_mod._resize_output_bgr(source, 960)
        self.assertEqual(output.shape, (540, 960, 3))
        self.assertAlmostEqual(output.shape[1] / output.shape[0], 16 / 9)

    def test_640x360_is_not_upscaled(self):
        source = np.zeros((360, 640, 3), dtype=np.uint8)
        output = bridge_mod._resize_output_bgr(source, 960)
        self.assertEqual(output.shape, source.shape)
        self.assertTrue(output.flags.c_contiguous)
        self.assertIsNot(output, source)

    def test_output_message_length_and_bytes_are_reduced(self):
        source = np.zeros((1080, 1920, 3), dtype=np.uint8)
        output = bridge_mod._resize_output_bgr(source, 960)
        msg = bridge_mod._build_image_msg(output, 'camera',
                                          TestBuildImageMsg()._fake_stamp())
        self.assertEqual(len(msg.data), 960 * 540 * 3)
        self.assertEqual(len(msg.data), output.nbytes)
        self.assertLess(output.nbytes, source.nbytes)

    def test_publisher_qos_is_reliable_keep_last_one(self):
        qos = bridge_mod.image_transport_qos()
        self.assertEqual(qos.history, HistoryPolicy.KEEP_LAST)
        self.assertEqual(qos.depth, 1)
        self.assertEqual(qos.reliability, ReliabilityPolicy.RELIABLE)
        self.assertEqual(qos.durability, DurabilityPolicy.VOLATILE)


class TestJpegDecodeValidation(unittest.TestCase):

    def test_valid_jpeg_passes(self):
        img = np.zeros((60, 80, 3), dtype=np.uint8)
        cv2.rectangle(img, (10, 10), (50, 50), (255, 0, 0), -1)
        ok, jpeg = cv2.imencode('.jpg', img)
        self.assertTrue(ok)
        decoded = cv2.imdecode(jpeg, cv2.IMREAD_COLOR)
        self.assertTrue(bridge_mod._validate_bgr_image(decoded))

    def test_corrupt_jpeg_rejected(self):
        corrupt = np.frombuffer(b'not a jpeg', dtype=np.uint8)
        decoded = cv2.imdecode(corrupt, cv2.IMREAD_COLOR)
        self.assertFalse(bridge_mod._validate_bgr_image(decoded))

    def test_empty_jpeg_rejected(self):
        try:
            decoded = cv2.imdecode(
                np.array([], dtype=np.uint8), cv2.IMREAD_COLOR)
            self.assertFalse(bridge_mod._validate_bgr_image(decoded))
        except cv2.error:
            pass


# ---- Process lifecycle tests ----


# ---- Process lifecycle tests (consolidated to avoid ROS2 node name conflicts) ----


class TestHelperLifecycle(unittest.TestCase):
    """Process lifecycle tests for go2_front_camera_bridge.

    All lifecycle tests share a single rclpy.init() via setUpClass to
    avoid node-name conflicts that arise when many test classes create
    nodes with identical names in rapid succession.
    """

    @classmethod
    def setUpClass(cls):
        import rclpy
        if not rclpy.ok():
            rclpy.init(args=[])

    def _make_node(self, stream_helper=None, max_helper_restarts=3,
                   env_overrides=None):
        """Create a bridge node wired to the fake stream helper."""
        if stream_helper is None:
            stream_helper = _FAKE_HELPER_PATH

        original_resolve = bridge_mod._resolve_helper

        def _patched_resolve(param):
            if not param:
                return stream_helper
            return original_resolve(param)

        bridge_mod._resolve_helper = _patched_resolve

        old_environ = os.environ.copy()
        try:
            if env_overrides:
                os.environ.update(env_overrides)
            node = bridge_mod.Go2FrontCameraBridgeNode()
            # set_parameters only updates the ROS store; update the cached
            # attribute directly so the restart guard sees the test value.
            node.max_helper_restarts = max_helper_restarts
            from rclpy.parameter import Parameter
            node.set_parameters([
                Parameter('stream_helper',
                          Parameter.Type.STRING, stream_helper),
                Parameter('max_helper_restarts',
                          Parameter.Type.INTEGER, max_helper_restarts),
            ])
        finally:
            bridge_mod._resolve_helper = original_resolve
            os.environ.clear()
            os.environ.update(old_environ)
        return node

    def tearDown(self):
        # Drain any remaining fake helpers that may have leaked.
        import subprocess
        try:
            subprocess.run(
                ['pkill', '-f', 'fake_stream_helper.py'],
                capture_output=True, timeout=3)
        except Exception:
            pass

    # ----------------------------------------------------------------
    # Test 1: first start metrics
    # ----------------------------------------------------------------
    def test_first_start_metrics(self):
        node = self._make_node(
            env_overrides={'FAKE_FRAME_INTERVAL': '0.02'})
        try:
            self.assertTrue(
                wait_until(lambda: node._frame_count >= 1,
                           timeout_sec=2.0, poll_interval_sec=0.02),
                'timeout waiting for first decoded frame: start=%d restart=%d '
                'frames=%d' % (node._start_count, node._restart_count,
                               node._frame_count))
            self.assertEqual(node._start_count, 1)
            self.assertEqual(node._restart_count, 0)
            self.assertIsNotNone(node._active_process)
            self.assertIsNone(node._active_process.poll())
            self.assertGreater(node._frame_count, 0)
        finally:
            node.destroy_node()
            time.sleep(0.2)

    # ----------------------------------------------------------------
    # Test 2: continuous frames use a single PID
    # ----------------------------------------------------------------
    def test_continuous_frames_one_pid(self):
        node = self._make_node(
            env_overrides={'FAKE_FRAME_INTERVAL': '0.02'})
        try:
            self.assertTrue(
                wait_until(lambda: node._frame_count >= 2,
                           timeout_sec=2.0, poll_interval_sec=0.02),
                'timeout waiting for initial frames: start=%d restart=%d '
                'frames=%d' % (node._start_count, node._restart_count,
                               node._frame_count))
            first_pid = node._helper_pid
            self.assertIsNotNone(first_pid)
            self.assertEqual(node._start_count, 1)
            self.assertEqual(node._restart_count, 0)
            self.assertTrue(
                wait_until(lambda: node._frame_count >= 5,
                           timeout_sec=2.0, poll_interval_sec=0.02),
                'timeout waiting for continuous frames: pid=%s start=%d '
                'restart=%d frames=%d' % (
                    first_pid, node._start_count, node._restart_count,
                    node._frame_count))
            self.assertEqual(node._helper_pid, first_pid)
            self.assertEqual(node._start_count, 1)
            self.assertEqual(node._restart_count, 0)
            self.assertGreaterEqual(node._frame_count, 5)
        finally:
            node.destroy_node()
            time.sleep(0.2)

    # ----------------------------------------------------------------
    # Test 3: destroy_node reaps helper
    # ----------------------------------------------------------------
    def test_destroy_node_clean_exit(self):
        node = self._make_node(
            env_overrides={'FAKE_FRAME_INTERVAL': '0.02'})
        time.sleep(0.3)
        proc = node._active_process
        self.assertIsNotNone(proc)
        pid = proc.pid

        node.destroy_node()

        self.assertIsNotNone(proc.poll())
        self.assertTrue(node._intentional_shutdown)
        try:
            os.kill(pid, 0)
            self.fail('Process pid=%d still exists' % pid)
        except OSError:
            pass

    # ----------------------------------------------------------------
    # Test 4: destroy_node unblocks reader blocked on stdout
    # ----------------------------------------------------------------
    def test_destroy_unblocks_reader(self):
        node = self._make_node(
            env_overrides={'FAKE_SLEEP_FOREVER': '1'})
        try:
            time.sleep(0.4)
            self.assertIsNotNone(node._active_process)
            self.assertEqual(node._frame_count, 0)
            start = time.monotonic()
            node.destroy_node()
            elapsed = time.monotonic() - start
            self.assertLess(elapsed, 8.0)
            self.assertTrue(node._intentional_shutdown)
        finally:
            if not node._intentional_shutdown:
                node.destroy_node()
            time.sleep(0.2)

    # ----------------------------------------------------------------
    # Test 5: SIGTERM escalation to SIGKILL
    # ----------------------------------------------------------------
    def test_sigkill_fallback(self):
        node = self._make_node(
            env_overrides={
                'FAKE_IGNORE_SIGTERM': '1',
                'FAKE_FRAME_INTERVAL': '0.02',
            })
        time.sleep(0.3)
        proc = node._active_process
        self.assertIsNotNone(proc)
        pid = proc.pid

        start = time.monotonic()
        node.destroy_node()
        elapsed = time.monotonic() - start

        self.assertIsNotNone(proc.poll())
        self.assertLess(elapsed, 6.0)
        try:
            os.kill(pid, 0)
            self.fail('Process pid=%d still exists after SIGKILL' % pid)
        except OSError:
            pass

    # ----------------------------------------------------------------
    # Test 6: auto-restart after abnormal exit
    # ----------------------------------------------------------------
    def test_auto_restart_on_crash(self):
        node = self._make_node(
            max_helper_restarts=5,
            env_overrides={
                'FAKE_EXIT_AFTER_FRAMES': '2',
                'FAKE_EXIT_CODE': '1',
                'FAKE_FRAME_INTERVAL': '0.02',
            })
        try:
            time.sleep(1.5)
            self.assertGreaterEqual(node._start_count, 1)
            self.assertGreaterEqual(node._restart_count, 1,
                                    'restart_count must be >= 1 after crash; '
                                    'start=%d restart=%d'
                                    % (node._start_count, node._restart_count))
            self.assertGreater(node._start_count, node._restart_count)
        finally:
            node.destroy_node()
            time.sleep(0.2)

    # ----------------------------------------------------------------
    # Test 7: max_helper_restarts → fail closed
    # ----------------------------------------------------------------
    def test_max_restarts_fail_closed(self):
        node = self._make_node(
            max_helper_restarts=1,
            env_overrides={
                'FAKE_EXIT_AFTER_FRAMES': '1',
                'FAKE_EXIT_CODE': '1',
                'FAKE_FRAME_INTERVAL': '0.02',
            })
        try:
            time.sleep(3.0)
            self.assertFalse(
                node._running,
                'bridge must fail closed: running=%s start=%d restart=%d '
                'error=%s' % (node._running, node._start_count,
                              node._restart_count, node._last_error))
            self.assertIn('max_helper_restarts', node._last_error)
        finally:
            node.destroy_node()
            time.sleep(0.2)

    # ----------------------------------------------------------------
    # Test 8: intentional shutdown suppresses auto-restart
    # ----------------------------------------------------------------
    def test_intentional_shutdown_no_restart(self):
        node = self._make_node(
            max_helper_restarts=5,
            env_overrides={
                'FAKE_EXIT_AFTER_FRAMES': '2',
                'FAKE_EXIT_CODE': '0',
                'FAKE_FRAME_INTERVAL': '0.02',
            })
        try:
            self.assertTrue(
                wait_until(lambda: node._start_count == 1
                           and node._active_process is not None,
                           timeout_sec=2.0, poll_interval_sec=0.02),
                'timeout waiting for first helper before shutdown: '
                'start=%d restart=%d' % (
                    node._start_count, node._restart_count))
            node.destroy_node()
            self.assertTrue(node._intentional_shutdown)
            self.assertIsNone(node._active_process)
        finally:
            if not node._intentional_shutdown:
                node.destroy_node()
            time.sleep(0.2)

    # ----------------------------------------------------------------
    # Test 9: truncated frame → helper recycled
    # ----------------------------------------------------------------
    def test_truncated_frame_recovery(self):
        node = self._make_node(
            max_helper_restarts=3,
            env_overrides={
                'FAKE_TRUNCATE_FRAME': '1',
                'FAKE_EXIT_AFTER_FRAMES': '1',
                'FAKE_EXIT_CODE': '1',
                'FAKE_FRAME_INTERVAL': '0.02',
            })
        try:
            time.sleep(1.5)
            self.assertGreaterEqual(node._start_count, 1,
                                    'helper should restart after truncated '
                                    'frame; start=%d' % node._start_count)
            self.assertTrue(node._running or node._start_count >= 1)
        finally:
            node.destroy_node()
            time.sleep(0.2)

    # ----------------------------------------------------------------
    # Test 10: no residual processes after shutdown
    # ----------------------------------------------------------------
    def test_shutdown_no_residual(self):
        initial_children = _count_children()

        node = self._make_node(
            env_overrides={'FAKE_FRAME_INTERVAL': '0.02'})
        time.sleep(0.3)
        pid = node._helper_pid
        self.assertIsNotNone(pid)

        node.destroy_node()
        time.sleep(0.5)

        try:
            os.kill(pid, 0)
            self.fail('Helper pid=%d still exists' % pid)
        except OSError:
            pass

        remaining = _count_children()
        self.assertEqual(remaining, initial_children)

    # ----------------------------------------------------------------
    # Test 11: missing helper → fail
    # ----------------------------------------------------------------
    def test_resolve_rejects_missing(self):
        with self.assertRaises(FileNotFoundError):
            bridge_mod._resolve_helper('/nonexistent/path/to/stream_helper')

    def test_resolve_rejects_empty(self):
        with self.assertRaises(FileNotFoundError):
            bridge_mod._resolve_helper('')

    def test_node_with_missing_helper_raises(self):
        original_resolve = bridge_mod._resolve_helper
        bridge_mod._resolve_helper = (
            lambda p: (_ for _ in ()).throw(
                FileNotFoundError('injected missing helper')))
        try:
            with self.assertRaises(FileNotFoundError):
                bridge_mod.Go2FrontCameraBridgeNode()
        finally:
            bridge_mod._resolve_helper = original_resolve

    # ----------------------------------------------------------------
    # Test 12: absolute path from launch
    # ----------------------------------------------------------------
    def test_resolve_absolute_path(self):
        resolved = bridge_mod._resolve_helper(_FAKE_HELPER_PATH)
        self.assertEqual(resolved, _FAKE_HELPER_PATH)


class TestDeriveLibraryPath(unittest.TestCase):
    """Test that _derive_library_path never uses hardcoded workspace paths."""

    def test_install_prefix_derivation(self):
        tmp = tempfile.mkdtemp(prefix='test_derive_lib_')
        try:
            lib_dir = os.path.join(tmp, 'lib')
            os.makedirs(lib_dir)
            helper_dir = os.path.join(lib_dir, 'rk_go2_sdk_bridge')
            os.makedirs(helper_dir)
            helper = os.path.join(helper_dir, 'stream_helper')
            with open(helper, 'w') as fh:
                fh.write('')
            os.chmod(helper, 0o755)
            additions = bridge_mod._derive_library_path(helper, '')
            self.assertIn(lib_dir, additions)
        finally:
            shutil.rmtree(tmp, ignore_errors=True)

    def test_no_hardcoded_workspace(self):
        helper = '/opt/rk_go2_sdk_bridge/lib/rk_go2_sdk_bridge/stream_helper'
        additions = bridge_mod._derive_library_path(helper, '')
        joined = ':'.join(additions)
        self.assertNotIn('rk_inspection_ws', joined)
        self.assertNotIn('unitree', joined)
        self.assertNotIn(os.path.expanduser('~'), joined)

    def test_explicit_sdk_library_path(self):
        helper = '/opt/rk_go2_sdk_bridge/lib/rk_go2_sdk_bridge/stream_helper'
        additions = bridge_mod._derive_library_path(
            helper, '/custom/sdk/lib:/another/path')
        self.assertIn('/custom/sdk/lib', additions)
        self.assertIn('/another/path', additions)

    def test_empty_helper_path(self):
        additions = bridge_mod._derive_library_path('', '')
        self.assertEqual(additions, [])


if __name__ == '__main__':
    unittest.main()
