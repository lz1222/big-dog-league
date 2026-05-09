#!/usr/bin/env python3

import threading
import time

import rclpy
from rclpy.action import ActionClient, ActionServer
from rclpy.action.server import GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node

from rk_interfaces.action import ExecuteArmTask, ExecuteMotion, RunMission


STAGES = [
    'PRECHECK',
    'START',
    'JUMP_START',
    'AVOID',
    'STAIRS',
    'PICK_START_ITEM',
    'TRANSFER_AND_PICK_FIELD_ITEM',
    'WARNING_DETECT_ACTION',
    'PLACE_ITEM',
    'JUMP_FINISH',
    'FINAL_STOP',
    'DONE',
]

ARM_STAGE_TARGETS = {
    'PICK_START_ITEM': ('pick_start_item', 'start_item'),
    'TRANSFER_AND_PICK_FIELD_ITEM': ('pick_field_item', 'field_item'),
    'PLACE_ITEM': ('place_item', 'finish_platform'),
}

LOCOMOTION_STAGES = {
    'START',
    'JUMP_START',
    'AVOID',
    'STAIRS',
    'WARNING_DETECT_ACTION',
    'JUMP_FINISH',
    'FINAL_STOP',
}


class MissionStateMachineNode(Node):
    """Run the first-stage mock competition mission."""

    def __init__(self):
        super().__init__('mission_state_machine_node')
        self.callback_group = ReentrantCallbackGroup()
        self.mission_running = False
        self.auto_goal_sent = False

        self.declare_parameter('auto_start', True)

        self.server = ActionServer(
            self,
            RunMission,
            '/mission/run',
            self.execute_callback,
            goal_callback=self.goal_callback,
            callback_group=self.callback_group
        )
        self.locomotion_client = ActionClient(
            self,
            ExecuteMotion,
            '/locomotion/execute_motion',
            callback_group=self.callback_group
        )
        self.arm_client = ActionClient(
            self,
            ExecuteArmTask,
            '/arm/execute_task',
            callback_group=self.callback_group
        )
        self.auto_client = ActionClient(
            self,
            RunMission,
            '/mission/run',
            callback_group=self.callback_group
        )

        auto_start = self.get_parameter('auto_start').value
        if auto_start:
            self.auto_timer = self.create_timer(
                1.0,
                self.send_auto_goal,
                callback_group=self.callback_group
            )
        else:
            self.auto_timer = None

        self.get_logger().info(
            f'Mission state machine started, auto_start={auto_start}'
        )

    def goal_callback(self, goal_request):
        if not goal_request.start:
            self.get_logger().warn('Rejected mission goal with start=false')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def send_auto_goal(self):
        if self.auto_goal_sent:
            return

        self.auto_goal_sent = True
        if self.auto_timer is not None:
            self.auto_timer.cancel()

        goal = RunMission.Goal()
        goal.start = True
        self.get_logger().info('Auto-starting mock mission through /mission/run')
        future = self.auto_client.send_goal_async(
            goal,
            feedback_callback=self.auto_feedback_callback
        )
        future.add_done_callback(self.auto_goal_response_callback)

    def auto_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Auto mission feedback: {feedback.stage} {feedback.progress:.2f}'
        )

    def auto_goal_response_callback(self, future):
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('Auto mission goal rejected')
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.auto_result_callback)

    def auto_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(
            f'Auto mission result: success={result.success}, '
            f'message={result.message}'
        )

    def execute_callback(self, goal_handle):
        if self.mission_running:
            goal_handle.abort()
            result = RunMission.Result()
            result.success = False
            result.message = 'Mission is already running'
            return result

        self.mission_running = True
        result = RunMission.Result()
        try:
            success = self.run_stages(goal_handle)
            result.success = success
            result.message = (
                'Mock mission completed'
                if success
                else 'Mock mission failed'
            )
            if success:
                goal_handle.succeed()
            else:
                goal_handle.abort()
        finally:
            self.mission_running = False

        return result

    def run_stages(self, goal_handle):
        for index, stage in enumerate(STAGES):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return False

            progress = index / float(len(STAGES) - 1)
            self.publish_feedback(goal_handle, stage, progress)
            self.get_logger().info(f'Mission stage: {stage}')

            if stage in ARM_STAGE_TARGETS:
                task_name, target = ARM_STAGE_TARGETS[stage]
                if not self.call_arm_task(task_name, target):
                    return False
            elif stage in LOCOMOTION_STAGES:
                if not self.call_motion(stage.lower()):
                    return False
            else:
                time.sleep(0.2)

        self.publish_feedback(goal_handle, 'DONE', 1.0)
        return True

    def publish_feedback(self, goal_handle, stage, progress):
        feedback = RunMission.Feedback()
        feedback.stage = stage
        feedback.progress = float(progress)
        goal_handle.publish_feedback(feedback)

    def call_motion(self, motion_name):
        goal = ExecuteMotion.Goal()
        goal.motion_name = motion_name
        return self.call_action(
            self.locomotion_client,
            goal,
            'locomotion',
            timeout_sec=15.0
        )

    def call_arm_task(self, task_name, target):
        goal = ExecuteArmTask.Goal()
        goal.task_name = task_name
        goal.target = target
        return self.call_action(
            self.arm_client,
            goal,
            'arm',
            timeout_sec=15.0
        )

    def call_action(self, client, goal, name, timeout_sec):
        if not client.wait_for_server(timeout_sec=timeout_sec):
            self.get_logger().error(f'{name} action server unavailable')
            return False

        goal_event = threading.Event()
        goal_result = {}

        def on_goal_response(future):
            goal_result['handle'] = future.result()
            goal_event.set()

        send_future = client.send_goal_async(
            goal,
            feedback_callback=lambda msg: self.child_feedback_callback(name, msg)
        )
        send_future.add_done_callback(on_goal_response)

        if not goal_event.wait(timeout_sec):
            self.get_logger().error(f'{name} goal response timed out')
            return False

        child_goal_handle = goal_result.get('handle')
        if child_goal_handle is None or not child_goal_handle.accepted:
            self.get_logger().error(f'{name} goal rejected')
            return False

        result_event = threading.Event()
        action_result = {}

        def on_result(future):
            action_result['result'] = future.result().result
            result_event.set()

        result_future = child_goal_handle.get_result_async()
        result_future.add_done_callback(on_result)

        if not result_event.wait(timeout_sec):
            self.get_logger().error(f'{name} result timed out')
            return False

        result = action_result['result']
        if not result.success:
            self.get_logger().error(f'{name} failed: {result.message}')
            return False

        self.get_logger().info(f'{name} completed: {result.message}')
        return True

    def child_feedback_callback(self, name, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'{name} feedback: {feedback.current_step} '
            f'{feedback.progress:.2f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = MissionStateMachineNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
