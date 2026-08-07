#!/usr/bin/env python3

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_srvs.srv import SetBool


class SafetyNode(Node):
    """Expose a simple mock emergency stop service."""

    def __init__(self):
        super().__init__('safety_node')
        self.estop_enabled = False
        self.service = self.create_service(
            SetBool,
            '/safety/estop',
            self.on_estop
        )
        self.get_logger().info('Safety service started at /safety/estop')

    def on_estop(self, request, response):
        self.estop_enabled = request.data
        response.success = True
        response.message = (
            'Emergency stop enabled'
            if self.estop_enabled
            else 'Emergency stop released'
        )
        self.get_logger().warn(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = SafetyNode()
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
