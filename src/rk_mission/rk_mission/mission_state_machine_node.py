#!/usr/bin/env python3

from dataclasses import dataclass
import threading
import time

import rclpy
from rclpy.action import ActionClient, ActionServer
from rclpy.action.server import GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool

from rk_interfaces.action import ExecuteArmTask, ExecuteMotion, RunMission
from rk_interfaces.msg import ItemTagArray, SignDetectionArray


@dataclass(frozen=True)
class StageSpec:
    """One mission stage definition used by the mission FSM."""

    name: str
    kind: str
    command: str = ''
    target: str = ''
    item_type: str = ''
    description: str = ''


STAGE_PLAN = (
    StageSpec('PRECHECK', 'precheck', description='Check action timeouts.'),
    StageSpec('WAIT_START', 'wait', description='Short start delay.'),
    StageSpec('START_JUMP', 'motion', 'start_jump'),
    StageSpec(
        'FOLLOW_TO_AVOID_ENTRY',
        'navigation',
        'follow_to_avoid_entry'
    ),
    StageSpec('AVOID_ZONE', 'navigation', 'avoid_zone'),
    StageSpec('FOLLOW_TO_STAIRS', 'navigation', 'follow_to_stairs'),
    StageSpec('STAIRS_UP_DOWN', 'motion', 'stairs_up_down'),
    StageSpec(
        'FOLLOW_TO_PICK_PLATFORM',
        'navigation',
        'follow_to_pick_platform'
    ),
    StageSpec('DETECT_PICK_SIGN', 'detect_pick_sign'),
    StageSpec(
        'PICK_START_ITEM',
        'arm_wait_item',
        'pick_start_item',
        target='start_item',
        item_type='start_item'
    ),
    StageSpec(
        'FOLLOW_TO_TRANSFER_PLATFORM',
        'navigation',
        'follow_to_transfer_platform'
    ),
    StageSpec(
        'DROP_START_ITEM',
        'arm',
        'drop_start_item',
        target='transfer_platform'
    ),
    StageSpec(
        'PICK_FIELD_ITEM',
        'arm_wait_item',
        'pick_field_item',
        target='field_item',
        item_type='field_item'
    ),
    StageSpec(
        'FOLLOW_TO_CHECK_POINT',
        'navigation',
        'follow_to_check_point'
    ),
    StageSpec('DETECT_WARNING_SIGN', 'detect_warning_sign'),
    StageSpec('DO_WARNING_ACTION', 'warning_motion'),
    StageSpec('FOLLOW_TO_PLACE_PLATFORM', 'place_navigation'),
    StageSpec('PLACE_FIELD_ITEM', 'place_arm', 'place_field_item'),
    StageSpec(
        'FOLLOW_TO_FINISH_JUMP',
        'navigation',
        'follow_to_finish_jump'
    ),
    StageSpec('FINISH_JUMP', 'motion', 'finish_jump'),
    StageSpec(
        'RETURN_TO_START_ZONE',
        'navigation',
        'return_to_start_zone'
    ),
    StageSpec('FINAL_STOP', 'final_stop', 'final_stop'),
    StageSpec('DONE', 'done'),
)

STAGES = [stage.name for stage in STAGE_PLAN]
STAGE_BY_NAME = {stage.name: stage for stage in STAGE_PLAN}

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
        spec = STAGE_BY_NAME.get(stage)
        if spec is None:
            self.get_logger().error(f'No handler for stage: {stage}')
            return False
        return self.execute_stage_spec(spec, goal_handle)

    def execute_stage_spec(self, spec, goal_handle):
        self.get_logger().info(
            'Stage detail: '
            f'name={spec.name}, kind={spec.kind}, '
            f'command={spec.command or "-"}, '
            f'target={spec.target or "-"}'
        )

        if spec.kind == 'precheck':
            return self.stage_precheck(goal_handle)
        if spec.kind == 'wait':
            return self.stage_wait_start(goal_handle)
        if spec.kind == 'motion':
            return self.call_motion(spec.command)
        if spec.kind == 'navigation':
            return self.call_navigation_segment(spec.command)
        if spec.kind == 'detect_pick_sign':
            return self.stage_detect_pick_sign(goal_handle)
        if spec.kind == 'detect_warning_sign':
            return self.stage_detect_warning_sign(goal_handle)
        if spec.kind == 'warning_motion':
            return self.call_warning_motion()
        if spec.kind == 'arm':
            return self.call_arm_task(spec.command, spec.target)
        if spec.kind == 'arm_wait_item':
            self.wait_for_item_tag(spec.item_type or spec.target, goal_handle)
            return self.call_arm_task(spec.command, spec.target)
        if spec.kind == 'place_navigation':
            return self.call_place_navigation()
        if spec.kind == 'place_arm':
            return self.call_place_arm(spec.command)
        if spec.kind == 'final_stop':
            self.publish_mission_stop()
            return self.call_motion(spec.command)
        if spec.kind == 'done':
            return self.stage_done(goal_handle)

        self.get_logger().error(
            f'Unknown stage kind: {spec.kind} for {spec.name}'
        )
        return False

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
        return self.call_warning_motion()

    def call_warning_motion(self):
        action = self.warning_action or self.default_warning_action
        if not action:
            self.get_logger().error('No warning action is available')
            return False
        return self.call_motion(action)

    def stage_follow_to_place_platform(self, goal_handle):
        del goal_handle
        return self.call_place_navigation()

    def call_place_navigation(self):
        target = self.place_target or self.default_place_target
        if not target:
            self.get_logger().error('No place target is available')
            return False
        return self.call_navigation_segment(f'follow_to_{target}')

    def stage_place_field_item(self, goal_handle):
        del goal_handle
        return self.call_place_arm('place_field_item')

    def call_place_arm(self, task_name):
        target = self.place_target or self.default_place_target
        if not target:
            self.get_logger().error('No place target is available')
            return False
        return self.call_arm_task(task_name, target)

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
