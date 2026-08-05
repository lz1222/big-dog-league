#!/usr/bin/env python3

"""将巡线建议速度限制在新鲜 ARM 心跳内，任何上游失联均输出零."""

import math
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool


class LineAcceptanceCmdGateNode(Node):
    """验收专用速度门：仅在 ARM 与巡线候选都新鲜时转发命令."""

    def __init__(self):
        """初始化为无候选、无 ARM 的零速度状态."""
        super().__init__('line_acceptance_cmd_gate_node')
        self.declare_parameter(
            'input_cmd_topic', '/navigation/line_follow_cmd_suggested'
        )
        self.declare_parameter('output_cmd_topic', '/control/line_cmd')
        self.declare_parameter('arm_topic', '/line_acceptance/arm')
        self.declare_parameter('candidate_timeout_sec', 0.25)
        self.declare_parameter('arm_timeout_sec', 0.30)
        self.declare_parameter('publish_rate_hz', 20.0)

        input_topic = self._name_parameter('input_cmd_topic')
        output_topic = self._name_parameter('output_cmd_topic')
        arm_topic = self._name_parameter('arm_topic')
        self.candidate_timeout_sec = self._positive_float_parameter(
            'candidate_timeout_sec'
        )
        self.arm_timeout_sec = self._positive_float_parameter(
            'arm_timeout_sec'
        )
        rate_hz = self._positive_float_parameter('publish_rate_hz')
        self.latest_candidate = Twist()
        self.candidate_received_at = None
        self.arm_received_at = None
        self.armed = False

        self.publisher = self.create_publisher(Twist, output_topic, 10)
        self.command_subscription = self.create_subscription(
            Twist, input_topic, self._on_candidate, 10
        )
        self.arm_subscription = self.create_subscription(
            Bool, arm_topic, self._on_arm, 10
        )
        self.timer = self.create_timer(1.0 / rate_hz, self._on_timer)
        self._publish_zero()
        self.get_logger().info(
            'Line acceptance cmd gate ready: input={}, output={}, arm={}'
            .format(input_topic, output_topic, arm_topic)
        )

    def _on_candidate(self, message):
        """只保存有限速度候选；非有限值立即使该候选失效."""
        if self._finite_twist(message):
            self.latest_candidate = message
            self.candidate_received_at = time.monotonic()
        else:
            self.candidate_received_at = None
            self._publish_zero()

    def _on_arm(self, message):
        """ARM=false 立即归零；ARM=true 仍必须保持心跳新鲜."""
        self.armed = bool(message.data)
        self.arm_received_at = time.monotonic()
        if not self.armed:
            self._publish_zero()

    def _on_timer(self):
        """任一时钟失效都不转发候选，形成守护/巡线节点故障归零链."""
        now = time.monotonic()
        if not self._arm_is_fresh(now) or not self._candidate_is_fresh(now):
            self._publish_zero()
            return
        self.publisher.publish(self.latest_candidate)

    def _arm_is_fresh(self, now):
        return (
            self.armed
            and self.arm_received_at is not None
            and now - self.arm_received_at <= self.arm_timeout_sec
        )

    def _candidate_is_fresh(self, now):
        return (
            self.candidate_received_at is not None
            and now - self.candidate_received_at <= self.candidate_timeout_sec
        )

    def _publish_zero(self):
        self.publisher.publish(Twist())

    @staticmethod
    def _finite_twist(message):
        return all(math.isfinite(value) for value in (
            message.linear.x, message.linear.y, message.angular.z,
        ))

    def _name_parameter(self, name):
        value = self.get_parameter(name).value
        if not isinstance(value, str) or not value.strip():
            raise ValueError(
                "Parameter '{}' must be a non-empty name".format(name)
            )
        return value.strip()

    def _positive_float_parameter(self, name):
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(
                "Parameter '{}' must be a positive finite number".format(
                    name
                )
            )
        return value


def main(args=None):
    """运行验收速度门，退出后 command_mux 输入在超时内归零."""
    rclpy.init(args=args)
    node = LineAcceptanceCmdGateNode()
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
