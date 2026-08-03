#!/usr/bin/env python3

"""Go2 front camera bridge node.

Launches a long-running go2_front_camera_stream_helper process and reads
length-prefixed JPEG frames from its stdout pipe.  Constructs
sensor_msgs/Image directly — no cv_bridge dependency.

The stream_helper path must be provided via the ``stream_helper`` ROS
parameter (absolute path resolvable by the OS, or a bare command name
resolvable via ``shutil.which``).  The launch file is responsible for
passing the install-tree path through that parameter.
"""

import json
import math
import os
import shutil
import signal
import struct
import subprocess
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Image
from std_msgs.msg import String


STREAM_HELPER_BASENAME = 'go2_front_camera_stream_helper'

LENGTH_HEADER_FORMAT = '!I'
LENGTH_HEADER_SIZE = struct.calcsize(LENGTH_HEADER_FORMAT)
MAX_JPEG_BYTES = 8 * 1024 * 1024


def image_transport_qos():
    """返回前向相机图像的可靠、单帧 QoS。

    大尺寸图像在真机上使用 BEST_EFFORT 时出现长时间断流。只保留最新帧可
    限制积压，而 RELIABLE 确保 bridge 与 detector 对同一帧交付契约一致。
    """
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


def _validate_bgr_image(array):
    """Return True if *array* is a valid H×W×3 uint8 BGR OpenCV image."""
    if array is None:
        return False
    if not isinstance(array, np.ndarray):
        return False
    if array.ndim != 3:
        return False
    if array.shape[2] != 3:
        return False
    if array.dtype != np.uint8:
        return False
    if array.shape[0] == 0 or array.shape[1] == 0:
        return False
    return True


def _build_image_msg(array, frame_id, stamp):
    """Build a sensor_msgs/Image from a validated H×W×3 uint8 BGR array."""
    contiguous = np.ascontiguousarray(array)
    msg = Image()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = contiguous.shape[0]
    msg.width = contiguous.shape[1]
    msg.encoding = 'bgr8'
    msg.is_bigendian = False
    msg.step = msg.width * 3
    msg.data = contiguous.tobytes()
    assert len(msg.data) == msg.height * msg.step, (
        'data length mismatch: %d != %d' % (
            len(msg.data), msg.height * msg.step))
    return msg


def _resize_output_bgr(array, max_output_width):
    """按宽度上限缩小相机图像，并保证输出为独立连续 BGR 内存。

    helper 始终传输原始 JPEG；缩放只发生在成功解码之后、构造 DDS Image
    之前，避免 1920×1080 原始 BGR 消息占用过大传输缓冲导致下游断流。
    """
    if not _validate_bgr_image(array):
        raise ValueError('invalid_decoded_bgr_image')
    max_output_width = max(1, int(max_output_width))
    height, width = array.shape[:2]
    if width > max_output_width:
        target_height = max(1, int(round(
            height * float(max_output_width) / float(width))))
        array = cv2.resize(
            array,
            (max_output_width, target_height),
            interpolation=cv2.INTER_AREA,
        )
    output = np.ascontiguousarray(array).copy()
    if not _validate_bgr_image(output) or not output.flags.c_contiguous:
        raise RuntimeError('output_bgr_resize_contract_failed')
    return output


def _resolve_helper(stream_helper_param):
    """Resolve the stream helper executable from the ROS parameter.

    * If *stream_helper_param* is an absolute path to an executable file,
      it is returned as-is.
    * If it is a bare command name, ``shutil.which`` resolves it from PATH.
    * Otherwise a ``FileNotFoundError`` is raised — no hard-coded workspace
      paths are ever searched.
    """
    if not stream_helper_param:
        raise FileNotFoundError(
            'stream_helper parameter is empty — must be an absolute path '
            'or a command name resolvable via PATH')

    expanded = os.path.expanduser(stream_helper_param)

    if os.path.isabs(expanded):
        if os.path.isfile(expanded) and os.access(expanded, os.X_OK):
            return expanded
        raise FileNotFoundError(
            'stream_helper absolute path is not an executable: %s'
            % expanded)

    resolved = shutil.which(expanded)
    if resolved:
        return resolved
    raise FileNotFoundError(
        'stream_helper "%s" not found on PATH and is not an absolute '
        'executable path' % stream_helper_param)


def _derive_library_path(helper_path, sdk_library_path_param):
    """Build LD_LIBRARY_PATH additions for the stream helper.

    Sources, in order:
    1. Explicit *sdk_library_path_param* (may be empty).
    2. ``<helper_install_prefix>/lib``, derived from the helper binary's
       location (two directories up).
    """
    additions = []
    seen = set()

    def _add(path):
        if path and path not in seen:
            additions.append(path)
            seen.add(path)

    if sdk_library_path_param:
        for entry in sdk_library_path_param.strip().split(':'):
            entry = entry.strip()
            _add(entry)

    if helper_path:
        try:
            helper_dir = os.path.dirname(os.path.realpath(helper_path))
            install_prefix = os.path.dirname(os.path.dirname(helper_dir))
            lib_dir = os.path.join(install_prefix, 'lib')
            if os.path.isdir(lib_dir):
                _add(lib_dir)
        except (OSError, TypeError):
            pass

    return additions


def _build_helper_env(helper_path, sdk_library_path_param):
    """Return an ``os.environ`` copy with LD_LIBRARY_PATH extended.

    Never uses LD_PRELOAD or overrides system libraries.
    """
    env = os.environ.copy()
    library_additions = _derive_library_path(helper_path, sdk_library_path_param)

    current = env.get('LD_LIBRARY_PATH', '')
    if current:
        for entry in current.split(':'):
            entry = entry.strip()
            if entry:
                library_additions.append(entry)

    # dedup preserving order
    merged = []
    seen = set()
    for p in library_additions:
        if p and p not in seen:
            merged.append(p)
            seen.add(p)

    env['LD_LIBRARY_PATH'] = ':'.join(merged)
    return env


class Go2FrontCameraBridgeNode(Node):
    """Pipe-based bridge: stream helper -> JPEG -> cv2.imdecode -> Image."""

    def __init__(self):
        super().__init__('go2_front_camera_bridge_node')
        self._declare_parameters()
        self._read_parameters()

        self._lock = threading.RLock()
        self._shutdown_requested = threading.Event()
        self._process_lock = threading.Lock()

        self._running = True
        self._active_process = None
        self._intentional_shutdown = False

        self._helper_pid = None
        self._frame_count = 0
        self._consecutive_failures = 0
        self._start_count = 0
        self._restart_count = 0
        self._last_exit_code = None
        self._last_error = ''
        self._last_frame_time = None
        self._last_width = 0
        self._last_height = 0
        self._source_width = 0
        self._source_height = 0
        self._output_bytes = 0
        self._last_encoding = 'bgr8'
        self._fps = 0.0
        self._frame_times = []
        self._decode_failures = 0
        self._protocol_errors = 0
        # 分段计数和时间戳用于只读定位图像断流边界：helper 读取、JPEG
        # 解码和 ROS 发布分别记录，避免把下游订阅问题误判为相机取流失败。
        self._helper_frame_read_count = 0
        self._jpeg_decode_success_count = 0
        self._image_publish_attempt_count = 0
        self._image_publish_return_count = 0
        self._image_publish_exception_count = 0
        self._last_helper_frame_time = None
        self._last_decode_time = None
        self._last_publish_return_time = None
        self._max_publish_duration_ms = 0.0

        self.image_publisher = self.create_publisher(
            Image, self.output_topic, image_transport_qos())
        self.status_publisher = self.create_publisher(
            String, self.status_topic, 10)

        self._reader_thread = threading.Thread(
            target=self._reader_loop, daemon=True)
        self._reader_thread.start()

        self.get_logger().info(
            'Go2 front camera bridge ready: '
            'interface=%s output=%s frame_id=%s helper=%s' % (
                self.network_interface,
                self.output_topic,
                self.frame_id,
                self._resolved_stream_helper,
            )
        )

    # ---- parameters ----

    def _declare_parameters(self):
        self.declare_parameter('network_interface', 'eth0')
        self.declare_parameter('output_topic', '/go2/front_camera/image_raw')
        self.declare_parameter('frame_id', 'go2_front_camera_optical_frame')
        self.declare_parameter('publish_status_topic',
                               '/go2/front_camera/status')
        self.declare_parameter('max_publish_rate_hz', 10.0)
        self.declare_parameter('max_output_width', 960)
        self.declare_parameter('sdk_timeout_sec', 2.0)
        self.declare_parameter('retry_sleep_sec', 0.20)
        self.declare_parameter('max_consecutive_retries', 10)
        self.declare_parameter('max_helper_restarts', 3)
        self.declare_parameter('stream_helper', '')
        self.declare_parameter('sdk_library_path', '')

    def _read_parameters(self):
        self.network_interface = self._str_param('network_interface')
        self.output_topic = self._str_param('output_topic')
        self.frame_id = self._str_param('frame_id')
        self.status_topic = self._str_param('publish_status_topic')
        self.max_publish_rate_hz = self._pos_float('max_publish_rate_hz')
        self.max_output_width = self._pos_int('max_output_width')
        self.sdk_timeout_sec = self._pos_float('sdk_timeout_sec')
        self.retry_sleep_sec = self._pos_float(
            'retry_sleep_sec', positive=False)
        self.max_consecutive_retries = self._pos_int('max_consecutive_retries')
        self.max_helper_restarts = self._pos_int('max_helper_restarts')
        stream_helper = self._str_param('stream_helper', empty_ok=True)
        self._resolved_stream_helper = _resolve_helper(stream_helper)
        self._sdk_library_path = self._str_param(
            'sdk_library_path', empty_ok=True)

    # ---- helper lifecycle ----

    def _start_helper(self):
        """Launch the stream helper subprocess.

        Returns the ``Popen`` instance on success or ``None`` on failure.
        Does not start a new helper when intentional shutdown is in progress.
        """
        if self._intentional_shutdown or self._shutdown_requested.is_set():
            return None

        command = [
            self._resolved_stream_helper,
            self.network_interface,
            str(self.sdk_timeout_sec),
            str(self.retry_sleep_sec),
            str(self.max_consecutive_retries),
        ]
        try:
            process = subprocess.Popen(
                command,
                stdin=subprocess.DEVNULL,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                env=_build_helper_env(
                    self._resolved_stream_helper, self._sdk_library_path),
                start_new_session=True,
            )
        except OSError as error:
            self._last_error = 'helper_start_failed: %s' % error
            self.get_logger().error(self._last_error)
            return None

        with self._process_lock:
            self._active_process = process
            self._helper_pid = process.pid
            self._start_count += 1
            if self._start_count > 1 and not self._intentional_shutdown:
                self._restart_count += 1

        restart_label = ('restart=%d' % self._restart_count
                         if self._restart_count > 0 else 'first')
        self.get_logger().info(
            'Stream helper started: pid=%d start=%d %s (max_restarts=%d)'
            % (self._helper_pid, self._start_count, restart_label,
               self.max_helper_restarts))

        def log_stderr():
            for line in process.stderr:
                if self._shutdown_requested.is_set():
                    break
                text = line.decode('utf-8', errors='replace').rstrip()
                if text:
                    self.get_logger().debug('[helper] %s' % text)

        threading.Thread(target=log_stderr, daemon=True).start()
        return process

    def _cleanup_active_process(self, process):
        """Clear the active process reference after the reader loop finishes
        with *process*.  Records the exit code for status reporting."""
        with self._process_lock:
            if self._active_process is process:
                exit_code = process.poll()
                self._last_exit_code = exit_code
                self._active_process = None

    def _terminate_process_group(self, process):
        """Best-effort termination of *process* and its full process group.

        Escalation: SIGTERM (3 s) → SIGKILL (2 s).
        The caller must still join the reader thread afterwards.
        """
        if process is None:
            return
        if process.poll() is not None:
            return  # already exited

        pid = process.pid
        self.get_logger().info(
            'Terminating helper pid=%d (group=%d)' % (pid, pid))

        # 1) SIGTERM to the process group
        try:
            pgid = os.getpgid(pid)
            os.killpg(pgid, signal.SIGTERM)
        except (OSError, ProcessLookupError):
            pass

        try:
            process.wait(timeout=3.0)
            self.get_logger().info(
                'Helper pid=%d exited after SIGTERM (code=%s)'
                % (pid, process.returncode))
            return
        except subprocess.TimeoutExpired:
            pass

        # 2) SIGKILL fallback
        self.get_logger().warn(
            'Helper pid=%d did not respond to SIGTERM, sending SIGKILL' % pid)
        try:
            pgid = os.getpgid(pid)
            os.killpg(pgid, signal.SIGKILL)
        except (OSError, ProcessLookupError):
            pass

        try:
            process.wait(timeout=2.0)
            self.get_logger().info(
                'Helper pid=%d killed by SIGKILL (code=%s)'
                % (pid, process.returncode))
        except subprocess.TimeoutExpired:
            self.get_logger().error(
                'Helper pid=%d still alive after SIGKILL — may require '
                'manual intervention' % pid)

    # ---- pipe protocol ----

    def _read_exact(self, fp, n):
        data = b''
        while len(data) < n:
            chunk = fp.read(n - len(data))
            if not chunk:
                return None
            data += chunk
        return data

    def _read_frame(self, fp):
        header = self._read_exact(fp, LENGTH_HEADER_SIZE)
        if header is None:
            return None, 'eof_or_header_truncated'
        (length,) = struct.unpack(LENGTH_HEADER_FORMAT, header)
        if length == 0 or length > MAX_JPEG_BYTES:
            self._protocol_errors += 1
            return None, 'invalid_frame_length: %d' % length
        jpeg_data = self._read_exact(fp, length)
        if jpeg_data is None:
            self._protocol_errors += 1
            return None, 'truncated_jpeg_expected_%d' % length
        return jpeg_data, ''

    # ---- main loop ----

    def _reader_loop(self):
        while self._running and not self._shutdown_requested.is_set():
            process = self._start_helper()
            if process is None:
                if self._intentional_shutdown:
                    break
                self._set_fault('helper_failed_to_start')
                break

            # restart_count tracks actual restarts (excludes first start).
            # It was incremented inside _start_helper for non-first starts.
            # max_helper_restarts limits the number of abnormal restarts.
            # Check immediately so we fail closed before entering the
            # inner read loop with an already-exhausted restart budget.
            if self._restart_count >= self.max_helper_restarts:
                self._set_fault('max_helper_restarts_exceeded')
                self._terminate_process_group(process)
                self._cleanup_active_process(process)
                break

            min_frame_interval = (
                1.0 / max(0.1, self.max_publish_rate_hz)
                if self.max_publish_rate_hz > 0.0 else 0.0
            )

            while self._running and not self._shutdown_requested.is_set():
                jpeg_data, error = self._read_frame(process.stdout)
                if jpeg_data is None:
                    if process.poll() is not None:
                        self.get_logger().warn(
                            'Helper exited with code=%d: %s' % (
                                process.returncode, error))
                    else:
                        self._consecutive_failures += 1
                        self._last_error = error
                        self.get_logger().warn(
                            'Pipe read error (helper still running): %s'
                            % error)
                    # Terminate the current (possibly dead) helper and
                    # clean up its process group.  If the helper already
                    # exited this is a no-op; if it is hung it gets killed.
                    self._terminate_process_group(process)
                    self._cleanup_active_process(process)
                    break

                # 成功读到完整长度前缀和 JPEG 后立即记账；该点不包含限速、
                # 解码或 ROS publish，可与后续阶段精确区分断流位置。
                self._helper_frame_read_count += 1
                self._last_helper_frame_time = time.monotonic()

                if min_frame_interval > 0.0:
                    time.sleep(min_frame_interval)

                self._process_jpeg(jpeg_data)

            # Decide whether to restart
            if self._intentional_shutdown or self._shutdown_requested.is_set():
                break

        self._running = False

    def _process_jpeg(self, jpeg_data):
        array = cv2.imdecode(
            np.frombuffer(jpeg_data, dtype=np.uint8), cv2.IMREAD_COLOR)
        if not _validate_bgr_image(array):
            self._decode_failures += 1
            self._consecutive_failures += 1
            self._last_error = 'jpeg_decode_failed_or_invalid'
            return
        self._source_height, self._source_width = array.shape[:2]
        try:
            array = _resize_output_bgr(array, self.max_output_width)
        except (TypeError, ValueError, RuntimeError, cv2.error) as error:
            self._decode_failures += 1
            self._consecutive_failures += 1
            self._last_error = 'output_resize_failed:%s' % type(error).__name__
            return

        self._consecutive_failures = 0
        self._last_error = ''
        now = time.monotonic()
        self._jpeg_decode_success_count += 1
        self._last_decode_time = now
        self._frame_count += 1
        self._last_width = array.shape[1]
        self._last_height = array.shape[0]
        self._output_bytes = int(array.nbytes)
        self._last_encoding = 'bgr8'
        self._last_frame_time = now

        self._frame_times.append(now)
        cutoff = now - 5.0
        self._frame_times = [t for t in self._frame_times if t >= cutoff]
        if len(self._frame_times) >= 2:
            span = self._frame_times[-1] - self._frame_times[0]
            self._fps = (
                (len(self._frame_times) - 1) / span
                if span > 0.0 else 0.0
            )

        publish_started = None
        try:
            stamp = self.get_clock().now().to_msg()
            msg = _build_image_msg(array, self.frame_id, stamp)
            self._image_publish_attempt_count += 1
            publish_started = time.monotonic()
            self.image_publisher.publish(msg)
            publish_duration_ms = (time.monotonic() - publish_started) * 1000.0
            self._max_publish_duration_ms = max(
                self._max_publish_duration_ms, publish_duration_ms)
            self._image_publish_return_count += 1
            self._last_publish_return_time = time.monotonic()
        except Exception as exc:
            if publish_started is not None:
                publish_duration_ms = (
                    time.monotonic() - publish_started) * 1000.0
                self._max_publish_duration_ms = max(
                    self._max_publish_duration_ms, publish_duration_ms)
            self._image_publish_exception_count += 1
            self._consecutive_failures += 1
            self._last_error = 'publish: %s' % type(exc).__name__

        self._publish_status()

    def _set_fault(self, reason):
        self._last_error = reason
        self._running = False
        self._publish_status()
        self.get_logger().error('Bridge fault: %s' % reason)

    def _publish_status(self):
        now_ts = time.monotonic()

        def _age(timestamp):
            return now_ts - timestamp if timestamp is not None else -1.0

        age = _age(self._last_frame_time)
        helper_running = False
        helper_pid = None
        with self._process_lock:
            proc = self._active_process
            if proc is not None and proc.poll() is None:
                helper_running = True
                helper_pid = proc.pid
            elif self._helper_pid is not None:
                helper_pid = self._helper_pid

        payload = {
            'helper_pid': helper_pid,
            'helper_running': helper_running,
            'start_count': self._start_count,
            'restart_count': self._restart_count,
            'last_exit_code': self._last_exit_code,
            'intentional_shutdown': self._intentional_shutdown,
            'running': self._running,
            'frame_count': self._frame_count,
            'fps': round(self._fps, 2),
            'last_frame_age': round(age, 3),
            'helper_frame_read_count': self._helper_frame_read_count,
            'jpeg_decode_success_count': self._jpeg_decode_success_count,
            'image_publish_attempt_count': self._image_publish_attempt_count,
            'image_publish_return_count': self._image_publish_return_count,
            'image_publish_exception_count': self._image_publish_exception_count,
            'last_helper_frame_age': round(
                _age(self._last_helper_frame_time), 3),
            'last_decode_age': round(_age(self._last_decode_time), 3),
            'last_publish_return_age': round(
                _age(self._last_publish_return_time), 3),
            'max_publish_duration_ms': round(
                self._max_publish_duration_ms, 3),
            'width': self._last_width,
            'height': self._last_height,
            'source_width': self._source_width,
            'source_height': self._source_height,
            'output_width': self._last_width,
            'output_height': self._last_height,
            'output_bytes': self._output_bytes,
            'encoding': self._last_encoding,
            'consecutive_failures': self._consecutive_failures,
            'decode_failures': self._decode_failures,
            'protocol_errors': self._protocol_errors,
            'last_error': self._last_error,
        }
        msg = String()
        msg.data = json.dumps(payload, separators=(',', ':'))
        self.status_publisher.publish(msg)

    def destroy_node(self):
        # 1) Mark intentional shutdown so _reader_loop and _start_helper
        #    know not to restart.
        self._intentional_shutdown = True

        # 2) Signal the reader thread.
        self._shutdown_requested.set()

        # 3) Atomically grab and clear the active process reference.
        with self._process_lock:
            process = self._active_process
            self._active_process = None

        # 4) Terminate the helper process group.
        self._terminate_process_group(process)

        # 5) Verify the process is fully reaped.
        if process is not None:
            if process.poll() is None:
                self.get_logger().error(
                    'Helper pid=%d was not reaped after termination '
                    'escalation' % process.pid)

        # 6) Join the reader thread — _terminate_process_group already
        #    closed the pipe fd, so any blocking read will unblock.
        if (self._reader_thread is not None
                and self._reader_thread.is_alive()):
            self._reader_thread.join(timeout=5.0)
            if self._reader_thread.is_alive():
                self.get_logger().error(
                    'Reader thread did not exit within 5 s timeout')

        # 7) Final status report and ROS teardown.
        self._publish_status()
        return super().destroy_node()

    # ---- parameter helpers ----

    def _str_param(self, name, empty_ok=False):
        value = str(self.get_parameter(name).value).strip()
        if not value and not empty_ok:
            raise ValueError('%s must not be empty' % name)
        return value

    def _pos_float(self, name, positive=True):
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value):
            raise ValueError('%s must be finite' % name)
        if positive and value <= 0.0:
            raise ValueError('%s must be positive' % name)
        return value

    def _pos_int(self, name):
        value = self.get_parameter(name).value
        try:
            number = int(value)
        except (TypeError, ValueError):
            raise ValueError('%s must be an integer' % name)
        if number <= 0:
            raise ValueError('%s must be positive' % name)
        return number


def main(args=None):
    rclpy.init(args=args)
    node = Go2FrontCameraBridgeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
