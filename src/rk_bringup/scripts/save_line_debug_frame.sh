#!/bin/bash
set -e

WORKSPACE_DIR="${RK_INSPECTION_WS:-$HOME/rk_inspection_ws}"
LOG_DIR="${RK_LINE_LOG_DIR:-$HOME/rk_line_logs}"
TOPIC="${1:-${RK_LINE_DEBUG_TOPIC:-/perception/debug/line_overlay}}"
TIMEOUT_SEC="${RK_IMAGE_SAVE_TIMEOUT:-8.0}"

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

python3 - "$TOPIC" "$LOG_DIR" "$TIMEOUT_SEC" <<'PY'
import re
import sys
import time
from datetime import datetime
from pathlib import Path

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def safe_name(topic):
    name = re.sub(r'[^A-Za-z0-9_.-]+', '_', topic.strip('/'))
    return name or 'image'


class SingleImageSaver(Node):
    def __init__(self, topic, output_dir):
        super().__init__('save_line_debug_frame')
        self.topic = topic
        self.output_dir = Path(output_dir)
        self.bridge = CvBridge()
        self.saved_path = None
        self.sub = self.create_subscription(
            Image,
            topic,
            self.callback,
            qos_profile_sensor_data,
        )

    def callback(self, msg):
        if self.saved_path is not None:
            return

        image = self.to_cv_image(msg)
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        path = self.output_dir / f'{stamp}_{safe_name(self.topic)}.png'
        if not cv2.imwrite(str(path), image):
            raise RuntimeError(f'cv2.imwrite failed for {path}')
        self.saved_path = path
        self.get_logger().info(
            f'saved {self.topic} frame to {path} '
            f'encoding={msg.encoding} size={msg.width}x{msg.height}'
        )

    def to_cv_image(self, msg):
        if msg.encoding in ('mono8', '8UC1'):
            return self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        if msg.encoding == 'rgb8':
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        if msg.encoding == 'bgr8':
            return self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        return self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')


def main():
    topic = sys.argv[1]
    output_dir = sys.argv[2]
    timeout_sec = float(sys.argv[3])

    rclpy.init()
    node = SingleImageSaver(topic, output_dir)
    deadline = time.monotonic() + timeout_sec
    try:
        while rclpy.ok() and node.saved_path is None:
            if time.monotonic() >= deadline:
                print(
                    f'ERROR: timeout waiting for image on {topic}. '
                    'Check ros2 topic list and ros2 topic info -v.',
                    file=sys.stderr,
                )
                return 1
            rclpy.spin_once(node, timeout_sec=0.2)
        print(node.saved_path)
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
PY
