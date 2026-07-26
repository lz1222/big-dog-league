"""Expose the command multiplexer core as a ROS 2 node."""

import json
import math

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_srvs.srv import SetBool

from rk_safety.command_mux_core import CommandMuxCore
from rk_safety.command_mux_core import VelocityCommand


class CommandMuxNode(Node):
    """Publish the only final velocity command for the competition stack."""

    def __init__(self):
        """Initialize parameters, subscriptions, publishers, and timer."""
        super().__init__('command_mux_node')

        self.declare_parameter('line_cmd_topic', '/control/line_cmd')
        self.declare_parameter('mission_cmd_topic', '/control/mission_cmd')
        self.declare_parameter(
            'locomotion_cmd_topic', '/control/locomotion_cmd'
        )
        self.declare_parameter('estop_topic', '/safety/estop')
        self.declare_parameter('gait_lock_topic', '/gait/control_lock')
        self.declare_parameter('arm_lock_topic', '/arm/control_lock')
        self.declare_parameter('output_cmd_topic', '/navigation/cmd_vel')
        self.declare_parameter('status_topic', '/control/cmd_mux_status')
        self.declare_parameter('enable_estop_service', True)
        self.declare_parameter('estop_service_name', '/safety/estop')

        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('line_cmd_timeout_sec', 0.5)
        self.declare_parameter('mission_cmd_timeout_sec', 0.5)
        self.declare_parameter('locomotion_cmd_timeout_sec', 0.3)
        self.declare_parameter('max_linear_x', 0.60)
        self.declare_parameter('max_linear_y', 0.15)
        self.declare_parameter('max_angular_z', 1.30)

        control_rate_hz = self._positive_finite_parameter('control_rate_hz')
        self._core = CommandMuxCore(
            line_cmd_timeout_sec=self._positive_finite_parameter(
                'line_cmd_timeout_sec'
            ),
            mission_cmd_timeout_sec=self._positive_finite_parameter(
                'mission_cmd_timeout_sec'
            ),
            locomotion_cmd_timeout_sec=self._positive_finite_parameter(
                'locomotion_cmd_timeout_sec'
            ),
            max_linear_x=self._positive_finite_parameter('max_linear_x'),
            max_linear_y=self._positive_finite_parameter('max_linear_y'),
            max_angular_z=self._positive_finite_parameter('max_angular_z'),
        )

        line_cmd_topic = self._topic_parameter('line_cmd_topic')
        mission_cmd_topic = self._topic_parameter('mission_cmd_topic')
        locomotion_cmd_topic = self._topic_parameter('locomotion_cmd_topic')
        estop_topic = self._topic_parameter('estop_topic')
        gait_lock_topic = self._topic_parameter('gait_lock_topic')
        arm_lock_topic = self._topic_parameter('arm_lock_topic')
        output_cmd_topic = self._topic_parameter('output_cmd_topic')
        status_topic = self._topic_parameter('status_topic')
        enable_estop_service = self._bool_parameter('enable_estop_service')
        estop_service_name = self._name_parameter('estop_service_name')

        self._command_publisher = self.create_publisher(
            Twist, output_cmd_topic, 10
        )
        self._status_publisher = self.create_publisher(String, status_topic, 10)
        self._subscriptions = [
            self.create_subscription(
                Twist,
                line_cmd_topic,
                self._make_command_callback('line'),
                10,
            ),
            self.create_subscription(
                Twist,
                mission_cmd_topic,
                self._make_command_callback('mission'),
                10,
            ),
            self.create_subscription(
                Twist,
                locomotion_cmd_topic,
                self._make_command_callback('locomotion'),
                10,
            ),
            self.create_subscription(Bool, estop_topic, self._on_estop, 10),
            self.create_subscription(
                Bool, gait_lock_topic, self._on_gait_lock, 10
            ),
            self.create_subscription(
                Bool, arm_lock_topic, self._on_arm_lock, 10
            ),
        ]
        self._estop_service = None
        if enable_estop_service:
            self._estop_service = self.create_service(
                SetBool, estop_service_name, self._on_estop_service
            )
        self._timer = self.create_timer(
            1.0 / control_rate_hz, self._publish_decision
        )

        self.get_logger().info(
            'Command mux ready: output={} status={} rate={:.3f} Hz'.format(
                output_cmd_topic, status_topic, control_rate_hz
            )
        )

    def _positive_finite_parameter(self, name):
        value = self.get_parameter(name).value
        if isinstance(value, bool):
            raise ValueError(
                "Parameter '{}' must be a finite number greater than 0".format(
                    name
                )
            )
        try:
            numeric_value = float(value)
        except (TypeError, ValueError):
            raise ValueError(
                "Parameter '{}' must be a finite number greater than 0".format(
                    name
                )
            )
        if not math.isfinite(numeric_value) or numeric_value <= 0.0:
            raise ValueError(
                "Parameter '{}' must be a finite number greater than 0".format(
                    name
                )
            )
        return numeric_value

    def _topic_parameter(self, name):
        return self._name_parameter(name)

    def _name_parameter(self, name):
        value = self.get_parameter(name).value
        if not isinstance(value, str) or not value.strip():
            raise ValueError(
                "Parameter '{}' must be a non-empty ROS name".format(name)
            )
        return value

    def _bool_parameter(self, name):
        value = self.get_parameter(name).value
        if not isinstance(value, bool):
            raise ValueError(
                "Parameter '{}' must be a boolean".format(name)
            )
        return value

    def _now_sec(self):
        return self.get_clock().now().nanoseconds / 1000000000.0

    def _make_command_callback(self, source):
        def callback(message):
            command = VelocityCommand(
                linear_x=message.linear.x,
                linear_y=message.linear.y,
                angular_z=message.angular.z,
            )
            if not self._core.update_command(source, command, self._now_sec()):
                self.get_logger().warning(
                    'Rejected unsafe or non-finite {} velocity command'.format(
                        source
                    )
                )

        return callback

    def _on_estop(self, message):
        self._transition_estop(message.data)

    def _on_estop_service(self, request, response):
        changed = self._transition_estop(request.data)
        response.success = True
        if changed:
            if request.data:
                response.message = (
                    'Emergency stop enabled; command caches cleared'
                )
            else:
                response.message = (
                    'Emergency stop cleared; waiting for new command'
                )
        else:
            state = 'enabled' if request.data else 'cleared'
            response.message = (
                'Emergency stop already {}; state unchanged'.format(state)
            )
        return response

    def _transition_estop(self, enabled):
        return self._core.set_estop(bool(enabled), self._now_sec())

    def _on_gait_lock(self, message):
        self._core.set_gait_lock(message.data, self._now_sec())

    def _on_arm_lock(self, message):
        self._core.set_arm_lock(message.data, self._now_sec())

    def _publish_decision(self):
        decision = self._core.evaluate(self._now_sec())

        output = Twist()
        output.linear.x = decision.command.linear_x
        output.linear.y = decision.command.linear_y
        output.angular.z = decision.command.angular_z
        self._command_publisher.publish(output)

        status = String()
        status.data = json.dumps(
            decision.status,
            ensure_ascii=True,
            separators=(',', ':'),
            allow_nan=False,
        )
        self._status_publisher.publish(status)


def main(args=None):
    """Run the command multiplexer node."""
    rclpy.init(args=args)
    node = None
    try:
        node = CommandMuxNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
