#!/usr/bin/env python3

import time

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from rk_interfaces.action import ExecuteArmTask


class MockArmServer(Node):
    """Mock Unitree D1 arm action server."""

    def __init__(self):
        super().__init__('mock_arm_server')
        self.action_server = ActionServer(
            self,
            ExecuteArmTask,
            '/arm/execute_task',
            self.execute_callback
        )
        self.get_logger().info('Mock arm action server started')

    def execute_callback(self, goal_handle):
        task_name = goal_handle.request.task_name or 'unnamed_task'
        target = goal_handle.request.target or 'default_target'
        self.get_logger().info(f'Executing mock arm task: {task_name} -> {target}')

        feedback = ExecuteArmTask.Feedback()
        for step in range(1, 6):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = ExecuteArmTask.Result()
                result.success = False
                result.message = f'Arm task canceled: {task_name}'
                return result

            feedback.current_step = f'{task_name}: step {step}/5'
            feedback.progress = step / 5.0
            goal_handle.publish_feedback(feedback)
            self.get_logger().info(feedback.current_step)
            time.sleep(0.2)

        goal_handle.succeed()
        result = ExecuteArmTask.Result()
        result.success = True
        result.message = f'Mock arm task completed: {task_name} -> {target}'
        return result


def main(args=None):
    rclpy.init(args=args)
    node = MockArmServer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
