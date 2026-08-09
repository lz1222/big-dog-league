"""独立动态验收 recorder ROS 节点；控制 callback 永不执行文件 I/O。"""

import argparse
import json
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from rclpy.qos import ReliabilityPolicy
from std_msgs.msg import Bool, String

from rk_interfaces.msg import LineTrack, SpecialTargetDetection
from rk_bringup.validation_recorder_core import BoundedJsonlWriter


class ValidationDynamicRecorder(Node):
    """将八路证据非阻塞入队；writer thread 独立批量持久化。"""

    def __init__(self, arguments):
        super().__init__('validation_dynamic_recorder')
        self.writer = BoundedJsonlWriter(
            arguments.record_path,
            queue_size=arguments.queue_size,
            batch_size=arguments.batch_size,
            flush_interval_sec=arguments.flush_interval_sec,
            writer_delay_sec=arguments.writer_delay_sec,
        )
        self.writer.start()
        self.status_publisher = self.create_publisher(
            String, '/validation/dynamic_recorder/status', 10,
        )
        evidence_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        sdk_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            Twist, '/navigation/line_follow_cmd_suggested',
            lambda msg: self._twist('suggested', msg), evidence_qos,
        )
        self.create_subscription(
            Twist, '/control/line_cmd',
            lambda msg: self._twist('adapter_line_cmd', msg), evidence_qos,
        )
        self.create_subscription(
            String, '/control/cmd_mux_status',
            lambda msg: self._raw('mux_status', msg), evidence_qos,
        )
        self.create_subscription(
            Twist, '/navigation/cmd_vel',
            lambda msg: self._twist('final_cmd', msg), evidence_qos,
        )
        self.create_subscription(
            LineTrack, '/perception/line_track', self._line, evidence_qos,
        )
        self.create_subscription(
            SpecialTargetDetection, '/perception/white_bar_detection',
            self._white, evidence_qos,
        )
        self.create_subscription(
            String, '/go2/sdk_motion_status',
            lambda msg: self._raw('sdk_status', msg), sdk_qos,
        )
        self.create_subscription(
            Bool, '/gait/control_lock', self._gait, evidence_qos,
        )
        self.create_subscription(
            String, '/validation/motion_adapter/status',
            lambda msg: self._raw('adapter_status', msg), evidence_qos,
        )
        self.timer = self.create_timer(0.5, self._publish_status)

    @staticmethod
    def _now_ns():
        return time.monotonic_ns()

    def _enqueue(self, channel, payload):
        self.writer.enqueue(channel, self._now_ns(), payload)

    def _twist(self, channel, message):
        self._enqueue(channel, {
            'linear_x': float(message.linear.x),
            'linear_y': float(message.linear.y),
            'angular_z': float(message.angular.z),
        })

    def _raw(self, channel, message):
        self._enqueue(channel, {'raw': str(message.data)})

    def _line(self, message):
        self._enqueue('line_track', {
            'visible': bool(message.line_visible),
            'confidence': float(message.confidence),
            'lateral': float(message.lateral_error),
            'heading': float(message.heading_error),
            'source_header_sec': int(message.header.stamp.sec),
            'source_header_nanosec': int(message.header.stamp.nanosec),
            'source_sequence': None,
            'frame_id': str(message.header.frame_id),
        })

    def _white(self, message):
        self._enqueue('white_bar', {
            'visible': bool(message.visible),
            'confidence': float(message.confidence),
            'center_x': float(message.center_x),
            'center_y': float(message.center_y),
            'area_ratio': float(message.area_ratio),
            'width_ratio': float(message.width_ratio),
            'height_ratio': float(message.height_ratio),
            'inside_candidate': bool(message.inside_candidate),
            'direction_hint': str(message.direction_hint),
            'reason': str(message.reason),
            'source_header_sec': int(message.header.stamp.sec),
            'source_header_nanosec': int(message.header.stamp.nanosec),
            'source_sequence': None,
            'frame_id': str(message.header.frame_id),
        })

    def _gait(self, message):
        self._enqueue('gait_lock', {'locked': bool(message.data)})

    def _publish_status(self):
        message = String()
        message.data = json.dumps(
            self.writer.snapshot(), separators=(',', ':'),
        )
        self.status_publisher.publish(message)

    def close_writer(self):
        return self.writer.close()


def parse_arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--record-path', required=True)
    parser.add_argument('--queue-size', type=int, default=2048)
    parser.add_argument('--batch-size', type=int, default=64)
    parser.add_argument('--flush-interval-sec', type=float, default=0.1)
    parser.add_argument('--writer-delay-sec', type=float, default=0.0)
    return parser.parse_args(argv)


def main(argv=None):
    arguments = parse_arguments(argv)
    rclpy.init(args=[])
    node = ValidationDynamicRecorder(arguments)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Ctrl-C 后仍必须进入 finally 完整 drain/flush/fsync。
        pass
    finally:
        node.close_writer()
        node.destroy_node()
        rclpy.shutdown()
