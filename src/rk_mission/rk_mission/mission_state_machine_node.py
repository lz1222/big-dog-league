#!/usr/bin/env python3

import json
import math
import os
import subprocess
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.action import ActionClient, ActionServer
from rclpy.action.server import GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, String

from rk_interfaces.action import ExecuteArmTask, ExecuteMotion, RunMission
from rk_interfaces.msg import (
    ItemTagArray,
    LineTrack,
    SignDetectionArray,
    SpecialTargetDetection,
)
from rk_mission.sign_action_executor_node import (
    DEFAULT_SDK_ACTION_EXECUTABLE,
    SDK_LD_LIBRARY_PATH_PREFIX,
)
from rk_mission.white_bar_action_core import WhiteBarActionRequestGate


STAGES = [
    'PRECHECK',
    'WAIT_START',
    'START_JUMP',
    'FOLLOW_TO_AVOID_ENTRY',
    'AVOID_ZONE',
    'FOLLOW_TO_STAIRS',
    'STAIRS_UP_DOWN',
    'FOLLOW_TO_PICK_PLATFORM',
    'DETECT_PICK_SIGN',
    'PICK_START_ITEM',
    'FOLLOW_TO_TRANSFER_PLATFORM',
    'DROP_START_ITEM',
    'PICK_FIELD_ITEM',
    'FOLLOW_TO_CHECK_POINT',
    'DETECT_WARNING_SIGN',
    'DO_WARNING_ACTION',
    'FOLLOW_TO_PLACE_PLATFORM',
    'PLACE_FIELD_ITEM',
    'FOLLOW_TO_FINISH_JUMP',
    'FINISH_JUMP',
    'RETURN_TO_START_ZONE',
    'FINAL_STOP',
    'DONE',
]

PICK_SIGN_TARGETS = {
    '1': 'place_platform_1',
    'one': 'place_platform_1',
    'place_1': 'place_platform_1',
    'platform_1': 'place_platform_1',
    'place_platform_1': 'place_platform_1',
    'marker_1': 'place_platform_1',
    '2': 'place_platform_2',
    'two': 'place_platform_2',
    'place_2': 'place_platform_2',
    'platform_2': 'place_platform_2',
    'place_platform_2': 'place_platform_2',
    'marker_2': 'place_platform_2',
}

WARNING_ACTIONS = {
    'electric': 'stretch',
    'electric_shock': 'stretch',
    'electricity': 'stretch',
    'shock': 'stretch',
    'caution_electricity': 'stretch',
    'stretch': 'stretch',
    'oxidizer': 'wave',
    'strong_oxidizer': 'wave',
    'strong_oxidant': 'wave',
    'oxidant': 'wave',
    'wave': 'wave',
    'radiation': 'blink_front_light_3',
    'radioactive': 'blink_front_light_3',
    'blink': 'blink_front_light_3',
    'blink_front_light_3': 'blink_front_light_3',
}

PICK_SIGN_TYPES = {
    'place_marker',
    'placement_marker',
    'pick_platform_marker',
    'pick_sign',
    'marker',
}

WARNING_SIGN_TYPES = {
    'warning',
    'warning_sign',
    'hazard',
}


class MissionStateMachineNode(Node):
    """Run the multimodal inspection competition mission."""

    def __init__(self):
        super().__init__('mission_state_machine_node')
        self.callback_group = ReentrantCallbackGroup()
        self.mission_running = False
        self.auto_goal_sent = False

        self.declare_parameter('auto_start', True)
        self.declare_parameter('start_stage', '')
        self.declare_parameter('end_stage', '')
        self.declare_parameter('action_timeout_sec', 15.0)
        self.declare_parameter('detection_timeout_sec', 3.0)
        self.declare_parameter('max_detection_age_sec', 2.0)
        self.declare_parameter('wait_start_delay_sec', 0.2)
        self.declare_parameter('publish_navigation_start', True)
        self.declare_parameter('stop_navigation_after_segment', True)
        self.declare_parameter('allow_detection_fallback', True)
        self.declare_parameter('default_place_target', 'place_platform_1')
        self.declare_parameter('default_warning_action', 'stretch')
        self.declare_parameter('mission_start_topic', '/mission/start')
        self.declare_parameter('mission_stop_topic', '/mission/stop')
        self.declare_parameter(
            'sign_detections_topic',
            '/perception/sign_detections'
        )
        self.declare_parameter('item_tags_topic', '/perception/item_tags')

        self.refresh_parameters()
        self.reset_mission_context()

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

        self.mission_start_publisher = self.create_publisher(
            Bool,
            self.mission_start_topic,
            10
        )
        self.mission_stop_publisher = self.create_publisher(
            Bool,
            self.mission_stop_topic,
            10
        )
        self.sign_subscription = self.create_subscription(
            SignDetectionArray,
            self.sign_detections_topic,
            self.on_sign_detections,
            10,
            callback_group=self.callback_group
        )
        self.item_tag_subscription = self.create_subscription(
            ItemTagArray,
            self.item_tags_topic,
            self.on_item_tags,
            10,
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
            'Mission state machine started: '
            f'auto_start={auto_start}, '
            f'start_stage={self.start_stage or "FIRST"}, '
            f'end_stage={self.end_stage or "LAST"}'
        )

    def refresh_parameters(self):
        self.start_stage = self.string_parameter('start_stage').upper()
        self.end_stage = self.string_parameter('end_stage').upper()
        self.action_timeout_sec = self.positive_float_parameter(
            'action_timeout_sec'
        )
        self.detection_timeout_sec = self.nonnegative_float_parameter(
            'detection_timeout_sec'
        )
        self.max_detection_age_sec = self.nonnegative_float_parameter(
            'max_detection_age_sec'
        )
        self.wait_start_delay_sec = self.nonnegative_float_parameter(
            'wait_start_delay_sec'
        )
        self.publish_navigation_start = self.bool_parameter(
            'publish_navigation_start'
        )
        self.stop_navigation_after_segment = self.bool_parameter(
            'stop_navigation_after_segment'
        )
        self.allow_detection_fallback = self.bool_parameter(
            'allow_detection_fallback'
        )
        self.default_place_target = self.string_parameter(
            'default_place_target'
        )
        self.default_warning_action = self.string_parameter(
            'default_warning_action'
        )
        self.mission_start_topic = self.string_parameter('mission_start_topic')
        self.mission_stop_topic = self.string_parameter('mission_stop_topic')
        self.sign_detections_topic = self.string_parameter(
            'sign_detections_topic'
        )
        self.item_tags_topic = self.string_parameter('item_tags_topic')

    def reset_mission_context(self):
        self.place_target = None
        self.warning_action = None
        self.latest_sign_detections = []
        self.latest_sign_receive_time = None
        self.latest_item_tags = []
        self.latest_item_receive_time = None

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
        self.get_logger().info('Auto-starting mission through /mission/run')
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

        self.refresh_parameters()
        self.reset_mission_context()
        self.mission_running = True
        result = RunMission.Result()
        try:
            try:
                success = self.run_stages(goal_handle)
            except ValueError as exc:
                self.get_logger().error(str(exc))
                success = False
            result.success = success
            result.message = (
                'Competition mission completed'
                if success
                else 'Competition mission failed'
            )
            if success:
                goal_handle.succeed()
            else:
                goal_handle.abort()
        finally:
            self.publish_mission_stop()
            self.mission_running = False

        return result

    def run_stages(self, goal_handle):
        stages = self.selected_stages()
        for index, stage in enumerate(stages):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return False

            progress = index / float(max(1, len(stages) - 1))
            self.publish_feedback(goal_handle, stage, progress)
            self.get_logger().info(f'Mission stage: {stage}')

            if not self.execute_stage(stage, goal_handle):
                self.get_logger().error(f'Mission stage failed: {stage}')
                return False

            if index + 1 < len(stages):
                self.log_transition(stage, stages[index + 1])

        self.publish_feedback(goal_handle, 'DONE', 1.0)
        return True

    def selected_stages(self):
        start_index = 0
        end_index = len(STAGES)

        if self.start_stage:
            if self.start_stage not in STAGES:
                raise ValueError(f'Unknown start_stage: {self.start_stage}')
            start_index = STAGES.index(self.start_stage)

        if self.end_stage:
            if self.end_stage not in STAGES:
                raise ValueError(f'Unknown end_stage: {self.end_stage}')
            end_index = STAGES.index(self.end_stage) + 1

        if start_index >= end_index:
            raise ValueError(
                'start_stage must appear before or equal to end_stage'
            )

        return STAGES[start_index:end_index]

    def execute_stage(self, stage, goal_handle):
        handlers = {
            'PRECHECK': self.stage_precheck,
            'WAIT_START': self.stage_wait_start,
            'START_JUMP': self.stage_start_jump,
            'FOLLOW_TO_AVOID_ENTRY': self.stage_follow_to_avoid_entry,
            'AVOID_ZONE': self.stage_avoid_zone,
            'FOLLOW_TO_STAIRS': self.stage_follow_to_stairs,
            'STAIRS_UP_DOWN': self.stage_stairs_up_down,
            'FOLLOW_TO_PICK_PLATFORM': self.stage_follow_to_pick_platform,
            'DETECT_PICK_SIGN': self.stage_detect_pick_sign,
            'PICK_START_ITEM': self.stage_pick_start_item,
            'FOLLOW_TO_TRANSFER_PLATFORM': (
                self.stage_follow_to_transfer_platform
            ),
            'DROP_START_ITEM': self.stage_drop_start_item,
            'PICK_FIELD_ITEM': self.stage_pick_field_item,
            'FOLLOW_TO_CHECK_POINT': self.stage_follow_to_check_point,
            'DETECT_WARNING_SIGN': self.stage_detect_warning_sign,
            'DO_WARNING_ACTION': self.stage_do_warning_action,
            'FOLLOW_TO_PLACE_PLATFORM': self.stage_follow_to_place_platform,
            'PLACE_FIELD_ITEM': self.stage_place_field_item,
            'FOLLOW_TO_FINISH_JUMP': self.stage_follow_to_finish_jump,
            'FINISH_JUMP': self.stage_finish_jump,
            'RETURN_TO_START_ZONE': self.stage_return_to_start_zone,
            'FINAL_STOP': self.stage_final_stop,
            'DONE': self.stage_done,
        }
        handler = handlers.get(stage)
        if handler is None:
            self.get_logger().error(f'No handler for stage: {stage}')
            return False
        return handler(goal_handle)

    def stage_precheck(self, goal_handle):
        del goal_handle
        self.get_logger().info(
            'Precheck: '
            f'action_timeout={self.action_timeout_sec:.1f}s, '
            f'detection_timeout={self.detection_timeout_sec:.1f}s'
        )
        return True

    def stage_wait_start(self, goal_handle):
        return self.interruptible_sleep(
            self.wait_start_delay_sec,
            goal_handle,
            'wait_start'
        )

    def stage_start_jump(self, goal_handle):
        del goal_handle
        return self.call_motion('start_jump')

    def stage_follow_to_avoid_entry(self, goal_handle):
        del goal_handle
        return self.call_navigation_segment('follow_to_avoid_entry')

    def stage_avoid_zone(self, goal_handle):
        del goal_handle
        return self.call_navigation_segment('avoid_zone')

    def stage_follow_to_stairs(self, goal_handle):
        del goal_handle
        return self.call_navigation_segment('follow_to_stairs')

    def stage_stairs_up_down(self, goal_handle):
        del goal_handle
        return self.call_motion('stairs_up_down')

    def stage_follow_to_pick_platform(self, goal_handle):
        del goal_handle
        return self.call_navigation_segment('follow_to_pick_platform')

    def stage_detect_pick_sign(self, goal_handle):
        detection = self.wait_for_sign(
            PICK_SIGN_TYPES,
            PICK_SIGN_TARGETS,
            self.detection_timeout_sec,
            goal_handle
        )
        if detection is None:
            return self.use_default_place_target('no pick sign detected')

        normalized_value = self.normalized_text(detection.sign_value)
        self.place_target = PICK_SIGN_TARGETS[normalized_value]
        self.get_logger().info(
            'Pick platform sign detected: '
            f'value={detection.sign_value}, place_target={self.place_target}'
        )
        return True

    def stage_pick_start_item(self, goal_handle):
        self.wait_for_item_tag('start_item', goal_handle)
        return self.call_arm_task('pick_start_item', 'start_item')

    def stage_follow_to_transfer_platform(self, goal_handle):
        del goal_handle
        return self.call_navigation_segment('follow_to_transfer_platform')

    def stage_drop_start_item(self, goal_handle):
        del goal_handle
        return self.call_arm_task('drop_start_item', 'transfer_platform')

    def stage_pick_field_item(self, goal_handle):
        self.wait_for_item_tag('field_item', goal_handle)
        return self.call_arm_task('pick_field_item', 'field_item')

    def stage_follow_to_check_point(self, goal_handle):
        del goal_handle
        return self.call_navigation_segment('follow_to_check_point')

    def stage_detect_warning_sign(self, goal_handle):
        detection = self.wait_for_sign(
            WARNING_SIGN_TYPES,
            WARNING_ACTIONS,
            self.detection_timeout_sec,
            goal_handle
        )
        if detection is None:
            return self.use_default_warning_action('no warning sign detected')

        normalized_value = self.normalized_text(detection.sign_value)
        self.warning_action = WARNING_ACTIONS[normalized_value]
        self.get_logger().info(
            'Warning sign detected: '
            f'value={detection.sign_value}, action={self.warning_action}'
        )
        return True

    def stage_do_warning_action(self, goal_handle):
        del goal_handle
        action = self.warning_action or self.default_warning_action
        if not action:
            self.get_logger().error('No warning action is available')
            return False
        return self.call_motion(action)

    def stage_follow_to_place_platform(self, goal_handle):
        del goal_handle
        target = self.place_target or self.default_place_target
        if not target:
            self.get_logger().error('No place target is available')
            return False
        return self.call_navigation_segment(f'follow_to_{target}')

    def stage_place_field_item(self, goal_handle):
        del goal_handle
        target = self.place_target or self.default_place_target
        if not target:
            self.get_logger().error('No place target is available')
            return False
        return self.call_arm_task('place_field_item', target)

    def stage_follow_to_finish_jump(self, goal_handle):
        del goal_handle
        return self.call_navigation_segment('follow_to_finish_jump')

    def stage_finish_jump(self, goal_handle):
        del goal_handle
        return self.call_motion('finish_jump')

    def stage_return_to_start_zone(self, goal_handle):
        del goal_handle
        return self.call_navigation_segment('return_to_start_zone')

    def stage_final_stop(self, goal_handle):
        del goal_handle
        self.publish_mission_stop()
        return self.call_motion('final_stop')

    def stage_done(self, goal_handle):
        del goal_handle
        self.publish_mission_stop()
        self.get_logger().info(
            'Mission context: '
            f'place_target={self.place_target or self.default_place_target}, '
            'warning_action='
            f'{self.warning_action or self.default_warning_action}'
        )
        return True

    def call_navigation_segment(self, motion_name):
        if self.publish_navigation_start:
            self.publish_mission_start()

        try:
            return self.call_motion(motion_name)
        finally:
            if self.stop_navigation_after_segment:
                self.publish_mission_stop()

    def call_motion(self, motion_name):
        goal = ExecuteMotion.Goal()
        goal.motion_name = motion_name
        return self.call_action(
            self.locomotion_client,
            goal,
            'locomotion',
            timeout_sec=self.action_timeout_sec
        )

    def call_arm_task(self, task_name, target):
        goal = ExecuteArmTask.Goal()
        goal.task_name = task_name
        goal.target = target
        return self.call_action(
            self.arm_client,
            goal,
            'arm',
            timeout_sec=self.action_timeout_sec
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
            feedback_callback=lambda msg: self.child_feedback_callback(
                name,
                msg
            )
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

    def wait_for_sign(self, sign_types, value_map, timeout_sec, goal_handle):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() <= deadline:
            if goal_handle.is_cancel_requested:
                return None

            detection = self.find_sign(sign_types, value_map)
            if detection is not None:
                return detection
            time.sleep(0.05)
        return None

    def find_sign(self, sign_types, value_map):
        if self.latest_sign_receive_time is None:
            return None
        if self.is_stale(self.latest_sign_receive_time):
            return None

        for detection in self.latest_sign_detections:
            sign_type = self.normalized_text(detection.sign_type)
            sign_value = self.normalized_text(detection.sign_value)
            if sign_type in sign_types and sign_value in value_map:
                return detection
        return None

    def wait_for_item_tag(self, item_type, goal_handle):
        deadline = time.monotonic() + self.detection_timeout_sec
        normalized_item_type = self.normalized_text(item_type)
        while time.monotonic() <= deadline:
            if goal_handle.is_cancel_requested:
                return None

            tag = self.find_item_tag(normalized_item_type)
            if tag is not None:
                self.get_logger().info(
                    'Item tag detected: '
                    f'type={tag.item_type}, id={tag.tag_id}, '
                    f'confidence={tag.confidence:.2f}'
                )
                return tag
            time.sleep(0.05)

        self.get_logger().warn(
            f'Item tag not detected before arm task: {item_type}'
        )
        return None

    def find_item_tag(self, item_type):
        if self.latest_item_receive_time is None:
            return None
        if self.is_stale(self.latest_item_receive_time):
            return None

        for tag in self.latest_item_tags:
            if self.normalized_text(tag.item_type) == item_type:
                return tag
        return None

    def use_default_place_target(self, reason):
        if not self.allow_detection_fallback or not self.default_place_target:
            self.get_logger().error(
                f'Cannot choose place target: {reason}'
            )
            return False

        self.place_target = self.default_place_target
        self.get_logger().warn(
            'Using default place target: '
            f'{self.place_target}, reason={reason}'
        )
        return True

    def use_default_warning_action(self, reason):
        if (
            not self.allow_detection_fallback
            or not self.default_warning_action
        ):
            self.get_logger().error(
                f'Cannot choose warning action: {reason}'
            )
            return False

        self.warning_action = self.default_warning_action
        self.get_logger().warn(
            'Using default warning action: '
            f'{self.warning_action}, reason={reason}'
        )
        return True

    def on_sign_detections(self, msg):
        self.latest_sign_detections = list(msg.detections)
        self.latest_sign_receive_time = time.monotonic()

    def on_item_tags(self, msg):
        self.latest_item_tags = list(msg.tags)
        self.latest_item_receive_time = time.monotonic()

    def publish_mission_start(self):
        msg = Bool()
        msg.data = True
        self.mission_start_publisher.publish(msg)
        self.get_logger().info('Published /mission/start true')

    def publish_mission_stop(self):
        msg = Bool()
        msg.data = True
        self.mission_stop_publisher.publish(msg)
        self.get_logger().info('Published /mission/stop true')

    def interruptible_sleep(self, duration_sec, goal_handle, name):
        deadline = time.monotonic() + duration_sec
        while time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                self.get_logger().warn(f'{name} interrupted by cancel')
                return False
            time.sleep(0.05)
        return True

    def is_stale(self, receive_time):
        if self.max_detection_age_sec <= 0.0:
            return False
        return time.monotonic() - receive_time > self.max_detection_age_sec

    def log_transition(self, current_stage, next_stage):
        self.get_logger().info(f'[FSM] {current_stage} -> {next_stage}')

    def publish_feedback(self, goal_handle, stage, progress):
        feedback = RunMission.Feedback()
        feedback.stage = stage
        feedback.progress = float(progress)
        goal_handle.publish_feedback(feedback)

    def child_feedback_callback(self, name, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'{name} feedback: {feedback.current_step} '
            f'{feedback.progress:.2f}'
        )

    def string_parameter(self, name):
        return str(self.get_parameter(name).value).strip()

    def bool_parameter(self, name):
        return bool(self.get_parameter(name).value)

    def nonnegative_float_parameter(self, name):
        value = float(self.get_parameter(name).value)
        if value < 0.0:
            raise ValueError(f'{name} must be nonnegative')
        return value

    def positive_float_parameter(self, name):
        value = self.nonnegative_float_parameter(name)
        if value <= 0.0:
            raise ValueError(f'{name} must be positive')
        return value

    @staticmethod
    def normalized_text(value):
        return str(value).strip().lower().replace('-', '_').replace(' ', '_')


LINE_COURSE_STATES = {
    'WAIT_START',
    'LINE_FOLLOW',
    'CORNER_PRE_TURN',
    'REACQUIRE_LINE',
    'APPROACH_RED_CIRCLE',
    'DO_RED_ACTION',
    'TURN_AFTER_RED',
    'HANDLE_WHITE_BAR',
    'APPROACH_STOP_ZONE',
    'FINAL_STOP',
    'EMERGENCY_STOP',
}


class LineCourseMissionNode(Node):
    """Own final velocity decisions for the visual line course."""

    def __init__(self):
        super().__init__('line_course_mission_node')
        self._declare_line_course_parameters()
        self._read_line_course_parameters()

        self.state = 'WAIT_START'
        self.state_enter_time = time.monotonic()
        self.mission_started = False
        self.latest_suggested_cmd = Twist()
        self.latest_suggested_time = None
        self.latest_line = None
        self.latest_line_time = None
        self.latest_red = None
        self.latest_red_time = None
        self.latest_stop_zone = None
        self.latest_stop_zone_time = None
        self.latest_white_bar = None
        self.latest_white_bar_time = None
        self.latest_corner = None
        self.latest_corner_time = None
        self.red_seen_count = 0
        self.stop_seen_count = 0
        self.stop_inside_count = 0
        self.white_seen_count = 0
        self.corner_seen_count = 0
        self.reacquire_seen_count = 0
        self.red_handled = False
        self.white_bar_handled = False
        self.white_bar_action_gate = WhiteBarActionRequestGate(
            self.white_bar_motion_name
        )
        self.white_bar_action_request_time = None
        self.last_corner_finish_time = -1.0e9
        self.red_action_process = None
        self.red_action_start_time = None
        self.last_published_state = ''

        self.cmd_publisher = self.create_publisher(
            Twist,
            self.cmd_vel_topic,
            10
        )
        self.state_publisher = self.create_publisher(
            String,
            self.mission_state_topic,
            10
        )
        self.white_bar_action_request_publisher = self.create_publisher(
            String,
            self.white_bar_action_request_topic,
            10
        )
        self.create_subscription(
            Twist,
            self.suggested_cmd_topic,
            self._on_suggested_cmd,
            10
        )
        self.create_subscription(
            LineTrack,
            self.line_track_topic,
            self._on_line_track,
            10
        )
        self.create_subscription(
            SpecialTargetDetection,
            self.red_circle_topic,
            self._on_red_circle,
            10
        )
        self.create_subscription(
            SpecialTargetDetection,
            self.stop_zone_topic,
            self._on_stop_zone,
            10
        )
        self.create_subscription(
            SpecialTargetDetection,
            self.white_bar_topic,
            self._on_white_bar,
            10
        )
        self.create_subscription(
            SpecialTargetDetection,
            self.corner_candidate_topic,
            self._on_corner_candidate,
            10
        )
        self.create_subscription(
            Bool,
            self.mission_start_topic,
            self._on_line_course_start,
            10
        )
        self.create_subscription(
            Bool,
            self.mission_stop_topic,
            self._on_line_course_stop,
            10
        )
        self.create_subscription(
            Bool,
            self.white_bar_action_done_topic,
            self._on_white_bar_action_done,
            10
        )
        self.control_timer = self.create_timer(
            1.0 / max(1.0, self.control_rate_hz),
            self._on_line_course_timer
        )
        self.get_logger().info(
            'Line course mission ready: mission_cmd='
            f'{self.cmd_vel_topic}, suggested_cmd={self.suggested_cmd_topic}, '
            f'sdk_action={self.red_circle_sdk_action or "disabled"}'
        )

    def _declare_line_course_parameters(self):
        topics = {
            'cmd_vel_topic': '/control/mission_cmd',
            'suggested_cmd_topic': '/navigation/line_follow_cmd_suggested',
            'mission_state_topic': '/mission/line_course_state',
            'line_track_topic': '/perception/line_track',
            'red_circle_topic': '/perception/red_circle_detection',
            'stop_zone_topic': '/perception/stop_zone_detection',
            'white_bar_topic': '/perception/white_bar_detection',
            'corner_candidate_topic': '/perception/corner_candidate',
            'mission_start_topic': '/mission/start',
            'mission_stop_topic': '/mission/stop',
            'white_bar_action_request_topic': (
                '/mission/white_bar_action_request'
            ),
            'white_bar_action_done_topic': '/mission/white_bar_action_done',
        }
        for name, value in topics.items():
            self.declare_parameter(name, value)

        defaults = {
            'control_rate_hz': 10.0,
            'suggested_cmd_timeout_sec': 0.5,
            'detection_timeout_sec': 0.8,
            'enable_corner_pre_turn': True,
            'corner_turn_direction': 'left',
            'corner_confirm_frames': 3,
            'corner_min_confidence': 0.45,
            'corner_vx': 0.03,
            'corner_angular_z': 0.32,
            'corner_min_time_sec': 0.8,
            'corner_max_time_sec': 2.5,
            'corner_exit_confidence': 0.30,
            'corner_exit_stable_frames': 3,
            'corner_cooldown_sec': 3.0,
            'red_circle_confirm_frames': 3,
            'red_circle_min_confidence': 0.55,
            'red_circle_approach_speed': 0.04,
            'red_circle_stop_area_ratio': 0.020,
            'red_circle_stop_y_ratio': 0.65,
            'red_circle_approach_timeout_sec': 8.0,
            'red_circle_sdk_action': 'stretch',
            'red_circle_sdk_wait_sec': 3.0,
            'red_circle_action_timeout_sec': 8.0,
            'sdk_network_interface': 'eth0',
            'sdk_action_executable': DEFAULT_SDK_ACTION_EXECUTABLE,
            'turn_after_red_direction': 'left',
            'turn_after_red_angle_deg': 90.0,
            'turn_after_red_angular_z': 0.35,
            'turn_after_red_timeout_sec': 5.0,
            'white_bar_confirm_frames': 3,
            'white_bar_min_confidence': 0.55,
            'white_bar_approach_speed': 0.03,
            'white_bar_stop_y_ratio': 0.70,
            'white_bar_action_timeout_sec': 5.0,
            'white_bar_motion_name': '',
            'stop_zone_confirm_frames': 3,
            'stop_zone_min_confidence': 0.55,
            'stop_zone_approach_speed': 0.04,
            'stop_zone_inside_confirm_frames': 3,
            'stop_zone_approach_timeout_sec': 10.0,
            'reacquire_timeout_sec': 8.0,
            'reacquire_min_confidence': 0.30,
            'reacquire_stable_frames': 3,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _read_line_course_parameters(self):
        topic_names = (
            'cmd_vel_topic',
            'suggested_cmd_topic',
            'mission_state_topic',
            'line_track_topic',
            'red_circle_topic',
            'stop_zone_topic',
            'white_bar_topic',
            'corner_candidate_topic',
            'mission_start_topic',
            'mission_stop_topic',
            'white_bar_action_request_topic',
            'white_bar_action_done_topic',
            'white_bar_motion_name',
            'corner_turn_direction',
            'red_circle_sdk_action',
            'sdk_network_interface',
            'sdk_action_executable',
            'turn_after_red_direction',
        )
        for name in topic_names:
            setattr(self, name, str(self.get_parameter(name).value).strip())

        bool_names = ('enable_corner_pre_turn',)
        for name in bool_names:
            setattr(self, name, bool(self.get_parameter(name).value))

        int_names = (
            'corner_confirm_frames',
            'corner_exit_stable_frames',
            'red_circle_confirm_frames',
            'white_bar_confirm_frames',
            'stop_zone_confirm_frames',
            'stop_zone_inside_confirm_frames',
            'reacquire_stable_frames',
        )
        for name in int_names:
            setattr(
                self,
                name,
                max(1, int(self.get_parameter(name).value))
            )

        float_names = (
            'control_rate_hz',
            'suggested_cmd_timeout_sec',
            'detection_timeout_sec',
            'corner_min_confidence',
            'corner_vx',
            'corner_angular_z',
            'corner_min_time_sec',
            'corner_max_time_sec',
            'corner_exit_confidence',
            'corner_cooldown_sec',
            'red_circle_min_confidence',
            'red_circle_approach_speed',
            'red_circle_stop_area_ratio',
            'red_circle_stop_y_ratio',
            'red_circle_approach_timeout_sec',
            'red_circle_sdk_wait_sec',
            'red_circle_action_timeout_sec',
            'turn_after_red_angle_deg',
            'turn_after_red_angular_z',
            'turn_after_red_timeout_sec',
            'white_bar_min_confidence',
            'white_bar_approach_speed',
            'white_bar_stop_y_ratio',
            'white_bar_action_timeout_sec',
            'stop_zone_min_confidence',
            'stop_zone_approach_speed',
            'stop_zone_approach_timeout_sec',
            'reacquire_timeout_sec',
            'reacquire_min_confidence',
        )
        for name in float_names:
            value = float(self.get_parameter(name).value)
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f'{name} must be finite and nonnegative')
            setattr(self, name, value)
        if self.control_rate_hz <= 0.0:
            raise ValueError('control_rate_hz must be positive')
        if self.corner_max_time_sec < self.corner_min_time_sec:
            self.corner_max_time_sec = self.corner_min_time_sec

    def _on_suggested_cmd(self, msg):
        self.latest_suggested_cmd = msg
        self.latest_suggested_time = time.monotonic()

    def _on_line_track(self, msg):
        self.latest_line = msg
        self.latest_line_time = time.monotonic()
        if (
            msg.line_visible
            and float(msg.confidence) >= self.reacquire_min_confidence
        ):
            self.reacquire_seen_count += 1
        else:
            self.reacquire_seen_count = 0

    def _on_red_circle(self, msg):
        self.latest_red = msg
        self.latest_red_time = time.monotonic()
        self.red_seen_count = self._updated_confirm_count(
            self.red_seen_count,
            msg,
            self.red_circle_min_confidence
        )

    def _on_stop_zone(self, msg):
        self.latest_stop_zone = msg
        self.latest_stop_zone_time = time.monotonic()
        self.stop_seen_count = self._updated_confirm_count(
            self.stop_seen_count,
            msg,
            self.stop_zone_min_confidence
        )
        if (
            bool(msg.visible)
            and bool(msg.inside_candidate)
            and float(msg.confidence) >= self.stop_zone_min_confidence
        ):
            self.stop_inside_count += 1
        else:
            self.stop_inside_count = 0

    def _on_white_bar(self, msg):
        self.latest_white_bar = msg
        self.latest_white_bar_time = time.monotonic()
        self.white_seen_count = self._updated_confirm_count(
            self.white_seen_count,
            msg,
            self.white_bar_min_confidence
        )

    def _on_corner_candidate(self, msg):
        self.latest_corner = msg
        self.latest_corner_time = time.monotonic()
        self.corner_seen_count = self._updated_confirm_count(
            self.corner_seen_count,
            msg,
            self.corner_min_confidence
        )

    @staticmethod
    def _updated_confirm_count(current, msg, min_confidence):
        if bool(msg.visible) and float(msg.confidence) >= min_confidence:
            return current + 1
        return 0

    def _on_line_course_start(self, msg):
        if not msg.data:
            return
        self._terminate_red_action()
        self.mission_started = True
        self.red_handled = False
        self.white_bar_handled = False
        self._reset_white_bar_action_state()
        self._reset_detection_counts()
        self._set_line_course_state('LINE_FOLLOW', 'mission_start')

    def _on_line_course_stop(self, msg):
        if not msg.data:
            return
        self.mission_started = False
        self._terminate_red_action()
        self._reset_white_bar_action_state()
        self._set_line_course_state('WAIT_START', 'mission_stop')
        self._publish_final_cmd(Twist())

    def _on_white_bar_action_done(self, msg):
        if not msg.data:
            return
        if self.state != 'HANDLE_WHITE_BAR':
            self.get_logger().debug(
                'Ignored white-bar done outside HANDLE_WHITE_BAR'
            )
            return
        if not self.white_bar_action_gate.accept_done(msg.data):
            self.get_logger().warn(
                'Ignored white-bar done before a current action request'
            )

    def _on_line_course_timer(self):
        now = time.monotonic()
        if self.state == 'WAIT_START':
            self._publish_final_cmd(Twist())
            return
        if self.state in ('FINAL_STOP', 'EMERGENCY_STOP'):
            self._publish_final_cmd(Twist())
            return
        if not self.mission_started:
            self._set_line_course_state('WAIT_START', 'mission_not_started')
            self._publish_final_cmd(Twist())
            return

        if self.state == 'LINE_FOLLOW':
            self._control_line_follow(now)
        elif self.state == 'CORNER_PRE_TURN':
            self._control_corner_pre_turn(now)
        elif self.state == 'REACQUIRE_LINE':
            self._control_reacquire_line(now)
        elif self.state == 'APPROACH_RED_CIRCLE':
            self._control_approach_red(now)
        elif self.state == 'DO_RED_ACTION':
            self._control_red_action(now)
        elif self.state == 'TURN_AFTER_RED':
            self._control_turn_after_red(now)
        elif self.state == 'HANDLE_WHITE_BAR':
            self._control_white_bar(now)
        elif self.state == 'APPROACH_STOP_ZONE':
            self._control_approach_stop_zone(now)
        else:
            self._set_line_course_state(
                'EMERGENCY_STOP',
                f'unhandled_state_{self.state}'
            )
            self._publish_final_cmd(Twist())

    def _control_line_follow(self, now):
        if (
            self._is_fresh(self.latest_stop_zone_time, now)
            and self.stop_seen_count >= self.stop_zone_confirm_frames
        ):
            self._set_line_course_state(
                'APPROACH_STOP_ZONE',
                'stop_zone_confirmed'
            )
            self._publish_final_cmd(Twist())
            return
        if (
            not self.red_handled
            and self._is_fresh(self.latest_red_time, now)
            and self.red_seen_count >= self.red_circle_confirm_frames
        ):
            self._set_line_course_state(
                'APPROACH_RED_CIRCLE',
                'red_circle_confirmed'
            )
            self._publish_final_cmd(Twist())
            return
        if (
            not self.white_bar_handled
            and self._is_fresh(self.latest_white_bar_time, now)
            and self.white_seen_count >= self.white_bar_confirm_frames
        ):
            self._set_line_course_state(
                'HANDLE_WHITE_BAR',
                'white_bar_confirmed'
            )
            self._publish_final_cmd(Twist())
            return
        if (
            self.enable_corner_pre_turn
            and self._is_fresh(self.latest_corner_time, now)
            and self.corner_seen_count >= self.corner_confirm_frames
            and now - self.last_corner_finish_time
            >= self.corner_cooldown_sec
        ):
            self._set_line_course_state(
                'CORNER_PRE_TURN',
                'corner_candidate_confirmed'
            )
            self._publish_final_cmd(Twist())
            return
        self._publish_suggested_or_stop(now)

    def _control_corner_pre_turn(self, now):
        elapsed = now - self.state_enter_time
        line_recovered = (
            elapsed >= self.corner_min_time_sec
            and self.latest_line is not None
            and bool(self.latest_line.line_visible)
            and float(self.latest_line.confidence)
            >= self.corner_exit_confidence
            and self.reacquire_seen_count
            >= self.corner_exit_stable_frames
        )
        if line_recovered or elapsed >= self.corner_max_time_sec:
            self.last_corner_finish_time = now
            self._set_line_course_state(
                'REACQUIRE_LINE',
                'corner_turn_complete'
            )
            self._publish_final_cmd(Twist())
            return
        cmd = Twist()
        cmd.linear.x = self.corner_vx
        direction = self._turn_direction(
            self.corner_turn_direction,
            self.latest_corner
        )
        cmd.angular.z = direction * self.corner_angular_z
        self._publish_final_cmd(cmd)

    def _control_reacquire_line(self, now):
        if self.reacquire_seen_count >= self.reacquire_stable_frames:
            self._set_line_course_state(
                'LINE_FOLLOW',
                'line_reacquired'
            )
            self._publish_suggested_or_stop(now)
            return
        if now - self.state_enter_time >= self.reacquire_timeout_sec:
            self._set_line_course_state(
                'EMERGENCY_STOP',
                'reacquire_timeout'
            )
            self._publish_final_cmd(Twist())
            return
        cmd = self._copy_suggested_cmd(now)
        cmd.linear.x = 0.0
        self._publish_final_cmd(cmd)

    def _control_approach_red(self, now):
        red = self.latest_red
        if not self._is_fresh(self.latest_red_time, now):
            self._set_line_course_state(
                'EMERGENCY_STOP',
                'red_circle_detection_stale'
            )
            self._publish_final_cmd(Twist())
            return
        if (
            red is not None
            and (
                float(red.area_ratio) >= self.red_circle_stop_area_ratio
                or float(red.center_y) >= self.red_circle_stop_y_ratio
            )
        ):
            self._set_line_course_state('DO_RED_ACTION', 'red_circle_reached')
            self._publish_final_cmd(Twist())
            return
        if now - self.state_enter_time >= self.red_circle_approach_timeout_sec:
            self._set_line_course_state(
                'EMERGENCY_STOP',
                'red_circle_approach_timeout'
            )
            self._publish_final_cmd(Twist())
            return
        cmd = self._copy_suggested_cmd(now)
        cmd.linear.x = self.red_circle_approach_speed
        self._publish_final_cmd(cmd)

    def _control_red_action(self, now):
        self._publish_final_cmd(Twist())
        if self.red_action_process is None:
            try:
                self._start_red_action()
            except (OSError, RuntimeError) as exc:
                self.get_logger().error(f'Red SDK action failed: {exc}')
                self._set_line_course_state(
                    'EMERGENCY_STOP',
                    'red_action_start_failed'
                )
            return

        return_code = self.red_action_process.poll()
        if return_code is not None:
            self.red_action_process = None
            self.red_action_start_time = None
            if return_code != 0:
                self._set_line_course_state(
                    'EMERGENCY_STOP',
                    f'red_action_exit_{return_code}'
                )
                return
            self.red_handled = True
            self.red_seen_count = 0
            self._set_line_course_state(
                'TURN_AFTER_RED',
                'red_action_complete'
            )
            return

        if (
            self.red_action_start_time is not None
            and now - self.red_action_start_time
            >= self.red_circle_action_timeout_sec
        ):
            self._terminate_red_action()
            self._set_line_course_state(
                'EMERGENCY_STOP',
                'red_action_timeout'
            )

    def _control_turn_after_red(self, now):
        angular_speed = self.turn_after_red_angular_z
        if angular_speed <= 0.0:
            self._set_line_course_state(
                'EMERGENCY_STOP',
                'turn_after_red_zero_speed'
            )
            self._publish_final_cmd(Twist())
            return
        target_duration = math.radians(
            self.turn_after_red_angle_deg
        ) / angular_speed
        target_duration = min(
            target_duration,
            self.turn_after_red_timeout_sec
        )
        if now - self.state_enter_time >= target_duration:
            self._set_line_course_state(
                'REACQUIRE_LINE',
                'turn_after_red_complete'
            )
            self._publish_final_cmd(Twist())
            return
        cmd = Twist()
        cmd.angular.z = (
            self._turn_direction(self.turn_after_red_direction)
            * angular_speed
        )
        self._publish_final_cmd(cmd)

    def _control_white_bar(self, now):
        if self.white_bar_action_gate.action_done:
            self.white_bar_handled = True
            self.white_seen_count = 0
            self._set_line_course_state(
                'REACQUIRE_LINE',
                'white_bar_action_done'
            )
            return
        if self.white_bar_action_gate.request_sent:
            self._publish_final_cmd(Twist())
            if (
                self.white_bar_action_request_time is not None
                and now - self.white_bar_action_request_time
                >= self.white_bar_action_timeout_sec
            ):
                self._set_line_course_state(
                    'EMERGENCY_STOP',
                    'white_bar_action_timeout'
                )
                self._publish_final_cmd(Twist())
            return
        if not self._is_fresh(self.latest_white_bar_time, now):
            self._set_line_course_state(
                'EMERGENCY_STOP',
                'white_bar_detection_stale'
            )
            self._publish_final_cmd(Twist())
            return
        stop_threshold_reached = (
            self.latest_white_bar is not None
            and float(self.latest_white_bar.center_y)
            >= self.white_bar_stop_y_ratio
        )
        event = self.white_bar_action_gate.evaluate(stop_threshold_reached)
        if event.action == 'APPROACH':
            cmd = self._copy_suggested_cmd(now)
            cmd.linear.x = self.white_bar_approach_speed
            self._publish_final_cmd(cmd)
            return
        self._publish_final_cmd(Twist())
        if event.action == 'CONFIG_ERROR':
            self._set_line_course_state('EMERGENCY_STOP', event.reason)
            return
        if event.action == 'SEND_REQUEST':
            request = String()
            request.data = event.motion_name
            self.white_bar_action_request_publisher.publish(request)
            self.white_bar_action_request_time = now
            self.get_logger().info(
                'Requested white-bar action: '
                f'{event.motion_name}'
            )
        return

    def _control_approach_stop_zone(self, now):
        if not self._is_fresh(self.latest_stop_zone_time, now):
            self._set_line_course_state(
                'EMERGENCY_STOP',
                'stop_zone_detection_stale'
            )
            self._publish_final_cmd(Twist())
            return
        if self.stop_inside_count >= self.stop_zone_inside_confirm_frames:
            self._set_line_course_state('FINAL_STOP', 'inside_stop_zone')
            self._publish_final_cmd(Twist())
            return
        if now - self.state_enter_time >= self.stop_zone_approach_timeout_sec:
            self._set_line_course_state(
                'EMERGENCY_STOP',
                'stop_zone_approach_timeout'
            )
            self._publish_final_cmd(Twist())
            return
        cmd = self._copy_suggested_cmd(now)
        cmd.linear.x = self.stop_zone_approach_speed
        self._publish_final_cmd(cmd)

    def _publish_suggested_or_stop(self, now):
        self._publish_final_cmd(self._copy_suggested_cmd(now))

    def _copy_suggested_cmd(self, now):
        cmd = Twist()
        if (
            self.latest_suggested_time is None
            or now - self.latest_suggested_time
            > self.suggested_cmd_timeout_sec
        ):
            return cmd
        source = self.latest_suggested_cmd
        cmd.linear.x = float(source.linear.x)
        cmd.linear.y = float(source.linear.y)
        cmd.linear.z = float(source.linear.z)
        cmd.angular.x = float(source.angular.x)
        cmd.angular.y = float(source.angular.y)
        cmd.angular.z = float(source.angular.z)
        return cmd

    def _publish_final_cmd(self, cmd):
        values = (
            cmd.linear.x,
            cmd.linear.y,
            cmd.linear.z,
            cmd.angular.x,
            cmd.angular.y,
            cmd.angular.z,
        )
        if not all(math.isfinite(float(value)) for value in values):
            self.get_logger().error('Rejected non-finite final cmd_vel')
            cmd = Twist()
            self._set_line_course_state(
                'EMERGENCY_STOP',
                'non_finite_final_cmd'
            )
        self.cmd_publisher.publish(cmd)
        self._publish_line_course_state(cmd)

    def _publish_line_course_state(self, cmd):
        msg = String()
        msg.data = json.dumps({
            'state': self.state,
            'mission_started': bool(self.mission_started),
            'final_vx': float(cmd.linear.x),
            'final_wz': float(cmd.angular.z),
            'red_confirm_count': self.red_seen_count,
            'white_bar_confirm_count': self.white_seen_count,
            'white_bar_action_request_sent': (
                self.white_bar_action_gate.request_sent
            ),
            'white_bar_motion_name': self.white_bar_motion_name,
            'stop_zone_confirm_count': self.stop_seen_count,
            'stop_zone_inside_count': self.stop_inside_count,
            'corner_confirm_count': self.corner_seen_count,
        }, separators=(',', ':'))
        self.state_publisher.publish(msg)

    def _set_line_course_state(self, new_state, reason):
        if new_state not in LINE_COURSE_STATES:
            new_state = 'EMERGENCY_STOP'
            reason = f'invalid_state:{new_state}'
        if new_state == self.state:
            return
        old_state = self.state
        self.state = new_state
        self.state_enter_time = time.monotonic()
        self.reacquire_seen_count = 0
        if new_state == 'HANDLE_WHITE_BAR':
            self._reset_white_bar_action_state()
        self.get_logger().info(
            f'[LINE_COURSE] {old_state} -> {new_state}: {reason}'
        )

    def _reset_detection_counts(self):
        self.red_seen_count = 0
        self.stop_seen_count = 0
        self.stop_inside_count = 0
        self.white_seen_count = 0
        self.corner_seen_count = 0
        self.reacquire_seen_count = 0

    def _reset_white_bar_action_state(self):
        self.white_bar_action_gate.reset(self.white_bar_motion_name)
        self.white_bar_action_request_time = None

    def _is_fresh(self, receive_time, now):
        return (
            receive_time is not None
            and now - receive_time <= self.detection_timeout_sec
        )

    @staticmethod
    def _turn_direction(mode, detection=None):
        normalized = str(mode).strip().lower()
        if normalized == 'hint' and detection is not None:
            normalized = str(detection.direction_hint).strip().lower()
        return -1.0 if normalized == 'right' else 1.0

    def _start_red_action(self):
        action = self.red_circle_sdk_action.strip()
        if not action:
            raise RuntimeError('red_circle_sdk_action is empty')
        executable = os.path.expanduser(self.sdk_action_executable)
        if not os.path.isfile(executable) or not os.access(
            executable,
            os.X_OK
        ):
            raise RuntimeError(
                f'SDK action executable is unavailable: {executable}'
            )
        command = [
            executable,
            self.sdk_network_interface,
            action,
            f'{self.red_circle_sdk_wait_sec:.3f}',
        ]
        self.get_logger().warn(
            'Starting existing Unitree SDK action: ' + ' '.join(command)
        )
        self.red_action_process = subprocess.Popen(
            command,
            env=self._sdk_action_env()
        )
        self.red_action_start_time = time.monotonic()

    @staticmethod
    def _sdk_action_env():
        env = os.environ.copy()
        paths = list(SDK_LD_LIBRARY_PATH_PREFIX)
        current = env.get('LD_LIBRARY_PATH', '')
        if current:
            paths.extend(current.split(':'))
        unique_paths = []
        for path in paths:
            if path and path not in unique_paths:
                unique_paths.append(path)
        env['LD_LIBRARY_PATH'] = ':'.join(unique_paths)
        return env

    def _terminate_red_action(self):
        process = self.red_action_process
        self.red_action_process = None
        self.red_action_start_time = None
        if process is None or process.poll() is not None:
            return
        process.terminate()
        try:
            process.wait(timeout=1.0)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=1.0)

    def destroy_node(self):
        self._terminate_red_action()
        return super().destroy_node()


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


def line_course_main(args=None):
    rclpy.init(args=args)
    node = LineCourseMissionNode()
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
