#!/bin/bash
set -e

WORKSPACE_DIR="${RK_INSPECTION_WS:-$HOME/rk_inspection_ws}"
LOG_DIR="${RK_LINE_LOG_DIR:-$HOME/rk_line_logs}"
TOPIC="${1:-${RK_LINE_DEBUG_TOPIC:-/perception/debug/line_overlay}}"
HOST="${RK_IMAGE_WEB_HOST:-0.0.0.0}"
PORT="${RK_IMAGE_WEB_PORT:-8088}"
JPEG_QUALITY="${RK_IMAGE_WEB_JPEG_QUALITY:-80}"

resolve_env_script() {
    local source_script="${WORKSPACE_DIR}/src/rk_bringup/scripts/ros_clean_env.sh"
    local install_script="${WORKSPACE_DIR}/install/rk_bringup/share/rk_bringup/scripts/ros_clean_env.sh"

    if [ -f "$source_script" ]; then
        printf "%s\n" "$source_script"
        return 0
    fi

    if [ -f "$install_script" ]; then
        printf "%s\n" "$install_script"
        return 0
    fi

    echo "ERROR: ros_clean_env.sh not found in source or install tree." >&2
    echo "Checked:" >&2
    echo "  $source_script" >&2
    echo "  $install_script" >&2
    return 1
}

ENV_SCRIPT="$(resolve_env_script)"
source "$ENV_SCRIPT"
mkdir -p "$LOG_DIR"
export ROS_LOG_DIR="${RK_ROS_LOG_DIR:-${LOG_DIR}/ros}"
mkdir -p "$ROS_LOG_DIR"

python3 - "$TOPIC" "$HOST" "$PORT" "$JPEG_QUALITY" <<'PY'
import html
import socket
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class SharedFrame:
    def __init__(self):
        self.condition = threading.Condition()
        self.jpeg = None
        self.seq = 0
        self.info = 'waiting for first frame'
        self.last_update = 0.0

    def update(self, jpeg, info):
        with self.condition:
            self.jpeg = jpeg
            self.seq += 1
            self.info = info
            self.last_update = time.time()
            self.condition.notify_all()

    def wait_for_next(self, previous_seq, timeout_sec=2.0):
        with self.condition:
            deadline = time.time() + timeout_sec
            while self.seq == previous_seq:
                remaining = deadline - time.time()
                if remaining <= 0:
                    break
                self.condition.wait(remaining)
            return self.seq, self.jpeg, self.info, self.last_update


class ImageToJpeg(Node):
    def __init__(self, topic, shared_frame, jpeg_quality):
        super().__init__('stream_line_debug_web')
        self.topic = topic
        self.shared_frame = shared_frame
        self.jpeg_quality = int(jpeg_quality)
        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            Image,
            topic,
            self.callback,
            qos_profile_sensor_data,
        )

    def callback(self, msg):
        try:
            image = self.to_cv_image(msg)
            ok, encoded = cv2.imencode(
                '.jpg',
                image,
                [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality],
            )
            if not ok:
                self.get_logger().warn('cv2.imencode failed')
                return
            info = f'{self.topic} encoding={msg.encoding} size={msg.width}x{msg.height}'
            self.shared_frame.update(encoded.tobytes(), info)
        except Exception as exc:
            self.get_logger().warn(f'failed to convert image: {exc}')

    def to_cv_image(self, msg):
        if msg.encoding in ('mono8', '8UC1'):
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
            return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        if msg.encoding == 'rgb8':
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        if msg.encoding == 'bgr8':
            return self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        return self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')


def best_effort_ip():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.connect(('8.8.8.8', 80))
        return sock.getsockname()[0]
    except OSError:
        return '127.0.0.1'
    finally:
        sock.close()


def make_handler(shared_frame, topic):
    escaped_topic = html.escape(topic)

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            return

        def do_GET(self):
            if self.path in ('/', '/index.html'):
                self.send_response(200)
                self.send_header('Content-Type', 'text/html; charset=utf-8')
                self.end_headers()
                body = f'''<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <title>RK Image Debug</title>
  <style>
    body {{ margin: 0; background: #111; color: #eee; font-family: sans-serif; }}
    header {{ padding: 10px 14px; background: #222; }}
    img {{ display: block; width: 100vw; height: auto; image-rendering: auto; }}
    code {{ color: #8fd; }}
  </style>
</head>
<body>
  <header>Topic: <code>{escaped_topic}</code> | refresh this page if frames stop</header>
  <img src="/stream.mjpg" alt="ROS image stream">
</body>
</html>
'''
                self.wfile.write(body.encode('utf-8'))
                return

            if self.path == '/latest.jpg':
                _, jpeg, info, _ = shared_frame.wait_for_next(-1, timeout_sec=0.1)
                if jpeg is None:
                    self.send_error(503, 'no image received yet')
                    return
                self.send_response(200)
                self.send_header('Content-Type', 'image/jpeg')
                self.send_header('X-Frame-Info', info)
                self.end_headers()
                self.wfile.write(jpeg)
                return

            if self.path == '/stream.mjpg':
                self.send_response(200)
                self.send_header('Age', '0')
                self.send_header('Cache-Control', 'no-cache, private')
                self.send_header('Pragma', 'no-cache')
                self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
                self.end_headers()

                seq = -1
                while True:
                    seq, jpeg, info, _ = shared_frame.wait_for_next(seq)
                    if jpeg is None:
                        continue
                    try:
                        self.wfile.write(b'--frame\r\n')
                        self.wfile.write(b'Content-Type: image/jpeg\r\n')
                        self.wfile.write(f'X-Frame-Info: {info}\r\n'.encode('utf-8'))
                        self.wfile.write(f'Content-Length: {len(jpeg)}\r\n\r\n'.encode('utf-8'))
                        self.wfile.write(jpeg)
                        self.wfile.write(b'\r\n')
                    except (BrokenPipeError, ConnectionResetError):
                        return
                return

            self.send_error(404)

    return Handler


def main():
    topic = sys.argv[1]
    host = sys.argv[2]
    port = int(sys.argv[3])
    jpeg_quality = int(sys.argv[4])
    shared_frame = SharedFrame()

    rclpy.init()
    node = ImageToJpeg(topic, shared_frame, jpeg_quality)
    server = ThreadingHTTPServer((host, port), make_handler(shared_frame, topic))
    server_thread = threading.Thread(target=server.serve_forever, daemon=True)
    server_thread.start()

    robot_ip = best_effort_ip()
    print(f'ROS topic: {topic}', flush=True)
    print(f'Open in robot VNC browser: http://127.0.0.1:{port}/', flush=True)
    print(f'Open from same network if reachable: http://{robot_ip}:{port}/', flush=True)
    print('Press Ctrl-C to stop.', flush=True)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        server.server_close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
PY
