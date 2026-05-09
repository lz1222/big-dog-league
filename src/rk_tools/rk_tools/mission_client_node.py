#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from rk_interfaces.action import RunMission


class MissionClientNode(Node):
    """Manual client for triggering /mission/run."""

    def __init__(self):
        super().__init__('mission_client_node')
        self.client = ActionClient(self, RunMission, '/mission/run')

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Mission feedback: {feedback.stage} {feedback.progress:.2f}'
        )

    def run(self):
        self.get_logger().info('Waiting for /mission/run action server')
        self.client.wait_for_server()

        goal = RunMission.Goal()
        goal.start = True
        goal_future = self.client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback
        )
        rclpy.spin_until_future_complete(self, goal_future)

        goal_handle = goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('Mission goal rejected')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        self.get_logger().info(
            f'Mission result: success={result.success}, message={result.message}'
        )
        return result.success


def main(args=None):
    rclpy.init(args=args)
    node = MissionClientNode()
    try:
        node.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
