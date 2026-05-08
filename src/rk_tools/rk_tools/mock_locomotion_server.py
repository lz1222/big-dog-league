#!/usr/bin/env python3

import time

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from rk_interfaces.action import ExecuteMotion


class MockLocomotionServer(Node):
    """Mock Go2 locomotion action server."""

    def __init__(self):
        super().__init__('mock_locomotion_server')
        self.action_server = ActionServer(
            self,
            ExecuteMotion,
            '/locomotion/execute_motion',
            self.execute_callback
        )
        self.get_logger().info('Mock locomotion action server started')

    def execute_callback(self, goal_handle):
        motion_name = goal_handle.request.motion_name or 'unnamed_motion'
        self.get_logger().info(f'Executing mock motion: {motion_name}')

        feedback = ExecuteMotion.Feedback()
        for step in range(1, 6):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = ExecuteMotion.Result()
                result.success = False
                result.message = f'Motion canceled: {motion_name}'
                return result

            feedback.current_step = f'{motion_name}: step {step}/5'
            feedback.progress = step / 5.0
            goal_handle.publish_feedback(feedback)
            self.get_logger().info(feedback.current_step)
            time.sleep(0.2)

        goal_handle.succeed()
        result = ExecuteMotion.Result()
        result.success = True
        result.message = f'Mock motion completed: {motion_name}'
        return result


def main(args=None):
    rclpy.init(args=args)
    node = MockLocomotionServer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
