#!/usr/bin/env python3

"""为全图真机巡线提供双确认 ARM 和急停自动 DISARM."""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, String

from rk_bringup.line_acceptance_core import LineAcceptanceGate


class LineAcceptanceGuardNode(Node):
    """把操作口令转换成短生命周期的 ARM 心跳和巡线启停信号."""

    def __init__(self):
        """初始化时强制 DISARM；节点不会自行产生启动请求."""
        super().__init__('line_acceptance_guard_node')
        self.declare_parameter('command_topic', '/line_acceptance/command')
        self.declare_parameter('arm_topic', '/line_acceptance/arm')
        self.declare_parameter('mission_start_topic', '/line_acceptance/start')
        self.declare_parameter('mission_stop_topic', '/line_acceptance/stop')
        self.declare_parameter('estop_state_topic', '/safety/estop_state')
        self.declare_parameter('allowed_segment_id', 'UNSET')
        self.declare_parameter('arm_heartbeat_hz', 10.0)

        self.command_topic = self._name_parameter('command_topic')
        self.arm_topic = self._name_parameter('arm_topic')
        self.mission_start_topic = self._name_parameter('mission_start_topic')
        self.mission_stop_topic = self._name_parameter('mission_stop_topic')
        self.estop_state_topic = self._name_parameter('estop_state_topic')
        self.gate = LineAcceptanceGate(
            self._name_parameter('allowed_segment_id')
        )
        heartbeat_hz = self._positive_float_parameter('arm_heartbeat_hz')

        self.arm_publisher = self.create_publisher(Bool, self.arm_topic, 10)
        self.start_publisher = self.create_publisher(
            Bool, self.mission_start_topic, 10
        )
        self.stop_publisher = self.create_publisher(
            Bool, self.mission_stop_topic, 10
        )
        self.command_subscription = self.create_subscription(
            String, self.command_topic, self._on_command, 10
        )
        self.estop_subscription = self.create_subscription(
            Bool, self.estop_state_topic, self._on_estop_state, 10
        )
        self.heartbeat_timer = self.create_timer(
            1.0 / heartbeat_hz, self._publish_arm_heartbeat
        )
        self._publish_arm_heartbeat()
        self.get_logger().info(
            'Line acceptance guard ready: segment={}, command={}, arm={}'
            .format(self.gate.allowed_segment_id, self.command_topic,
                    self.arm_topic)
        )

    def _on_command(self, message):
        """仅接受精确口令，避免宽松解析把聊天文本误当作实体启动."""
        decision = self.gate.handle_command(message.data)
        self._apply_decision(decision)

    def _on_estop_state(self, message):
        """急停状态一旦为真就撤销 ARM，防止解除急停后意外恢复运动."""
        if message.data:
            self._apply_decision(self.gate.force_disarm('estop_active'))

    def _apply_decision(self, decision):
        """按决策发布边沿启停事件，并持续公开当前 ARM 状态供下游失效关闭."""
        self._publish_arm_heartbeat()
        if decision.start_requested:
            self._publish_bool(self.start_publisher, True)
        if decision.stop_requested:
            self._publish_bool(self.stop_publisher, True)
        self.get_logger().info(
            'Line acceptance gate: armed={}, ready={}, reason={}'.format(
                decision.armed, decision.ready_confirmed, decision.reason
            )
        )

    def _publish_arm_heartbeat(self):
        """ARM 使用持续心跳；守护节点停止时下游门立即超时归零."""
        self._publish_bool(self.arm_publisher, self.gate.armed)

    @staticmethod
    def _publish_bool(publisher, value):
        message = Bool()
        message.data = bool(value)
        publisher.publish(message)

    def _name_parameter(self, name):
        value = self.get_parameter(name).value
        if not isinstance(value, str) or not value.strip():
            raise ValueError(
                "Parameter '{}' must be a non-empty name".format(name)
            )
        return value.strip()

    def _positive_float_parameter(self, name):
        value = float(self.get_parameter(name).value)
        if value <= 0.0:
            raise ValueError(
                "Parameter '{}' must be greater than zero".format(name)
            )
        return value


def main(args=None):
    """运行实体验收门禁；退出时 ROS 关闭由下游超时链归零."""
    rclpy.init(args=args)
    node = LineAcceptanceGuardNode()
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
