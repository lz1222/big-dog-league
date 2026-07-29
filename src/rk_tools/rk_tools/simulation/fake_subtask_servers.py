"""Simulation-only Action servers; they never import or call a real SDK."""

import json
import time

from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from std_msgs.msg import Bool, String

from rk_interfaces.action import ExecuteArmTask, ExecuteMotion


class FakeSubtaskServers(Node):
    """Provide recorded asynchronous success, failure, and delay outcomes."""

    def __init__(self, scenario):
        super().__init__('national_fake_subtask_servers')
        self.scenario = scenario
        self.callback_group = ReentrantCallbackGroup()
        self.action_pub = self.create_publisher(
            String, '/simulation/national/fake_action_calls', 10
        )
        self.gait_lock_pub = self.create_publisher(Bool, '/gait/control_lock', 10)
        self.arm_lock_pub = self.create_publisher(Bool, '/arm/control_lock', 10)
        self.motion_server = ActionServer(
            self, ExecuteMotion, '/locomotion/execute_motion',
            self._execute_motion, callback_group=self.callback_group,
        )
        self.arm_server = ActionServer(
            self, ExecuteArmTask, '/arm/execute_task', self._execute_arm,
            callback_group=self.callback_group,
        )

    def _execute_motion(self, goal_handle):
        task = str(goal_handle.request.motion_name)
        self._record('locomotion', task, '')
        self._publish_lock(self.gait_lock_pub, True)
        success, message = self._outcome(task)
        self._delay(task)
        self._publish_lock(self.gait_lock_pub, False)
        result = ExecuteMotion.Result()
        result.success = success
        result.message = message
        if success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    def _execute_arm(self, goal_handle):
        task = str(goal_handle.request.task_name)
        target = str(goal_handle.request.target)
        self._record('arm', task, target)
        self._publish_lock(self.arm_lock_pub, True)
        success, message = self._outcome(task)
        self._delay(task)
        self._publish_lock(self.arm_lock_pub, False)
        result = ExecuteArmTask.Result()
        result.success = success
        result.message = message
        if success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    def _outcome(self, task):
        failures = {
            'start_jump_failure': {'start_jump'},
            'maze_failure': {'maze_traverse_fake'},
            'stairs_failure': {'stairs_traverse'},
            'arm_pick_failure': {'pick_item'},
            'transfer_place_failure': {'transfer_place'},
            'transfer_pick_failure': {'transfer_pick'},
            'inspection_action_failure': {'stretch', 'hello', 'headlight_blink_3'},
            'place_action_failure': {'place_platform_1', 'place_platform_2'},
            'finish_jump_failure': {'finish_jump'},
        }
        if task in failures.get(self.scenario.fault, set()):
            return False, 'injected {} failure'.format(self.scenario.fault)
        return True, 'fake success; no real SDK invoked'

    def _delay(self, task):
        delay = 0.025
        if self.scenario.fault == 'maze_timeout' and task == 'maze_traverse_fake':
            delay = 0.80
        elif self.scenario.fault == 'action_duplicate_result':
            delay = 0.15
        time.sleep(delay)

    @staticmethod
    def _publish_lock(publisher, value):
        msg = Bool()
        msg.data = bool(value)
        publisher.publish(msg)

    def _record(self, adapter, task_name, target):
        msg = String()
        msg.data = json.dumps({
            'time_monotonic': time.monotonic(), 'adapter': adapter,
            'task_name': task_name, 'target': target,
            'real_sdk_called': False,
        }, sort_keys=True)
        self.action_pub.publish(msg)
