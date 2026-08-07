#!/usr/bin/env python3

"""非机械臂正式赛道的 ROS 路线适配节点。

节点只向 ``/control/mission_cmd`` 发布任务候选速度；最终
``/navigation/cmd_vel`` 始终由 command_mux_node 仲裁。
视觉事件、白线 Action 状态和检查闭环，
转交给路线阶段核心。
"""

import json
import math
import time
import uuid

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, String

from rk_interfaces.msg import LineTrack, SpecialTargetDetection
from rk_mission.non_arm_route_phase_core import NonArmRoutePhaseCore
from rk_mission.non_arm_route_phase_core import (
    validate_white_bar_timeout_chain,
)
from rk_mission.white_bar_stage_core import WhiteBarStageController


LINE_COURSE_NODE_STATES = frozenset((
    'WAIT_START',
    'START_STAGE',
    'MID_ROUTE',
    'APPROACH_RED_CIRCLE',
    'INSPECTION_WAIT_SIGN',
    'INSPECTION_ACTION',
    'POST_INSPECTION',
    'FINISH_STAGE',
    'HANDLE_WHITE_BAR',
    'TURN_AFTER_RED',
    'CORNER_PRE_TURN',
    'ALIGN_TO_LINE',
    'APPROACH_STOP_ZONE',
    'FINAL_ZONE_ARMED',
    'FINAL_STOP',
    'EMERGENCY_STOP',
))

WHITE_ACTION_FAILURE_STATES = frozenset(('FAILED', 'TIMEOUT', 'CANCELED'))
INSPECTION_FAILURE_STATES = frozenset((
    'FAILED',
    'TIMEOUT',
    'CANCELED',
    'FAULTED',
))


def decode_json_object(raw_message):
    """安全解析 JSON；畸形消息不影响控制循环。"""
    if type(raw_message) is not str:
        return None
    try:
        payload = json.loads(raw_message)
    except (TypeError, ValueError, json.JSONDecodeError):
        return None
    return payload if type(payload) is dict else None


def _finite_float(value):
    """拒绝 NaN/Inf，避免视觉异常被转换为真实运动候选。"""
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


class LineCourseMissionNode(Node):
    """正式赛道阶段门控；异常时只输出零候选速度。"""

    def __init__(self):
        """声明 ROS 输入输出和安全参数，发布初始状态。"""
        super().__init__('line_course_mission_node')
        self._declare_parameters()
        self._read_parameters()

        self.route_core = NonArmRoutePhaseCore()
        self.white_stage_controller = WhiteBarStageController(
            self.allow_finish_only_test
        )
        self.state = 'WAIT_START'
        self.state_enter_time = time.monotonic()
        self.last_reason = 'line_course_ready'
        self.active_action = ''
        self.align_context = ''
        self.align_seen_count = 0
        self._align_last_line_sequence = -1
        self._line_sequence = 0

        self.latest_suggested_cmd = Twist()
        self.latest_suggested_time = None
        self.latest_line = None
        self.latest_line_time = None
        self.latest_line_status = ''
        self.latest_line_follower_status_time = None
        self.line_follower_ready = False
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
        self.white_action_started_time = None
        self.white_action_expected_request_id = None
        self.latest_white_action_request_id = 0
        self.inspection_request_started_time = None

        self.cmd_publisher = self.create_publisher(
            Twist,
            self.cmd_vel_topic,
            10,
        )
        self.state_publisher = self.create_publisher(
            String,
            self.mission_state_topic,
            10,
        )
        self.white_bar_action_request_publisher = self.create_publisher(
            String,
            self.white_bar_action_request_topic,
            10,
        )
        self.white_bar_stage_status_publisher = self.create_publisher(
            String,
            self.white_bar_stage_status_topic,
            10,
        )
        self.inspection_action_request_publisher = self.create_publisher(
            String,
            self.inspection_action_request_topic,
            10,
        )

        self.create_subscription(
            Twist,
            self.suggested_cmd_topic,
            self._on_suggested_cmd,
            10,
        )
        self.create_subscription(
            String,
            self.line_follower_status_topic,
            self._on_line_follower_status,
            10,
        )
        self.create_subscription(
            LineTrack,
            self.line_track_topic,
            self._on_line_track,
            10,
        )
        self.create_subscription(
            SpecialTargetDetection,
            self.red_circle_topic,
            self._on_red_circle,
            10,
        )
        self.create_subscription(
            SpecialTargetDetection,
            self.stop_zone_topic,
            self._on_stop_zone,
            10,
        )
        self.create_subscription(
            SpecialTargetDetection,
            self.white_bar_topic,
            self._on_white_bar,
            10,
        )
        self.create_subscription(
            SpecialTargetDetection,
            self.corner_candidate_topic,
            self._on_corner_candidate,
            10,
        )
        self.create_subscription(
            Bool,
            self.mission_start_topic,
            self._on_mission_start,
            10,
        )
        self.create_subscription(
            Bool,
            self.mission_stop_topic,
            self._on_mission_stop,
            10,
        )
        self.create_subscription(
            String,
            self.white_bar_stage_command_topic,
            self._on_white_bar_stage_command,
            10,
        )
        self.create_subscription(
            String,
            self.white_bar_action_status_topic,
            self._on_white_bar_action_status,
            10,
        )
        self.create_subscription(
            Bool,
            self.white_bar_action_done_topic,
            self._on_white_bar_action_done,
            10,
        )
        self.create_subscription(
            String,
            self.inspection_action_status_topic,
            self._on_inspection_action_status,
            10,
        )
        self.control_timer = self.create_timer(
            1.0 / self.control_rate_hz,
            self._on_control_timer,
        )

        self._publish_white_stage_status(
            self.white_stage_controller.status_event(
                'white_bar_stage_controller_ready'
            )
        )
        self._publish_mission_candidate(Twist())
        self.get_logger().info(
            'Non-arm line course ready: '
            f'candidate_topic={self.cmd_vel_topic}, '
            f'line_follower_status={self.line_follower_status_topic}'
        )

    def _declare_parameters(self):
        """保留兼容参数，并声明正式路径的安全阈值。"""
        topic_defaults = {
            'cmd_vel_topic': '/control/mission_cmd',
            'suggested_cmd_topic': '/navigation/line_follow_cmd_suggested',
            'line_follower_status_topic': '/navigation/line_follow_status',
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
            'white_bar_action_status_topic': (
                '/mission/white_bar_action_status'
            ),
            'white_bar_stage_command_topic': (
                '/mission/white_bar_stage_command'
            ),
            'white_bar_stage_status_topic': (
                '/mission/white_bar_stage_status'
            ),
            'inspection_action_request_topic': (
                '/mission/inspection_action_request'
            ),
            'inspection_action_status_topic': (
                '/mission/inspection_action_status'
            ),
        }
        for name, value in topic_defaults.items():
            self.declare_parameter(name, value)

        defaults = {
            'control_rate_hz': 10.0,
            'suggested_cmd_timeout_sec': 0.5,
            'detection_timeout_sec': 0.8,
            'line_follower_status_timeout_sec': 1.0,
            'enable_corner_pre_turn': True,
            'corner_turn_direction': 'left',
            'corner_confirm_frames': 3,
            'corner_min_confidence': 0.45,
            'corner_vx': 0.03,
            'corner_angular_z': 0.32,
            'corner_min_time_sec': 0.8,
            'corner_max_time_sec': 2.5,
            'corner_cooldown_sec': 3.0,
            'red_circle_confirm_frames': 3,
            'red_circle_min_confidence': 0.55,
            'red_circle_approach_speed': 0.04,
            'red_circle_stop_area_ratio': 0.020,
            'red_circle_stop_y_ratio': 0.65,
            'red_circle_approach_timeout_sec': 8.0,
            'turn_after_red_direction': 'left',
            'turn_after_red_angle_deg': 90.0,
            'turn_after_red_angular_z': 0.35,
            'turn_after_red_timeout_sec': 5.0,
            'white_bar_confirm_frames': 3,
            'white_bar_min_confidence': 0.55,
            'white_bar_approach_speed': 0.03,
            'white_bar_stop_y_ratio': 0.70,
            'white_bar_executor_action_timeout_sec': 22.0,
            'white_bar_action_timeout_sec': 26.0,
            # 与 gait_params 时长同步；测试校验超时余量。
            'front_jump_start_worst_case_duration_sec': 17.0,
            'front_jump_finish_worst_case_duration_sec': 17.0,
            'stop_zone_confirm_frames': 3,
            'stop_zone_min_confidence': 0.55,
            'stop_zone_approach_speed': 0.04,
            'stop_zone_inside_confirm_frames': 3,
            'stop_zone_approach_timeout_sec': 10.0,
            'align_confirm_frames': 5,
            'align_min_confidence': 0.60,
            'align_max_lateral_error': 0.20,
            'align_max_heading_error': 0.25,
            'align_timeout_sec': 8.0,
            'align_max_angular_z': 0.25,
            'align_heading_gain': 0.80,
            'align_lateral_gain': 0.20,
            'inspection_action_timeout_sec': 32.0,
            # 兼容保留；正式检查不读取静态 SDK 动作。
            'red_circle_sdk_action': 'stretch',
            'red_circle_sdk_wait_sec': 3.0,
            'sdk_network_interface': 'eth1',
            'sdk_action_executable': '',
            'white_bar_motion_name': '',
            'allow_finish_only_test': False,
            'reacquire_timeout_sec': 8.0,
            'reacquire_min_confidence': 0.30,
            'reacquire_stable_frames': 3,
            'corner_exit_confidence': 0.30,
            'corner_exit_stable_frames': 3,
            'red_circle_action_timeout_sec': 8.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _read_parameters(self):
        """读取并校验参数；超时关系错误时故障关闭。"""
        topic_names = (
            'cmd_vel_topic',
            'suggested_cmd_topic',
            'line_follower_status_topic',
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
            'white_bar_action_status_topic',
            'white_bar_stage_command_topic',
            'white_bar_stage_status_topic',
            'inspection_action_request_topic',
            'inspection_action_status_topic',
            'corner_turn_direction',
            'turn_after_red_direction',
        )
        for name in topic_names:
            value = str(self.get_parameter(name).value).strip()
            if not value:
                raise ValueError(f'{name} must not be empty')
            setattr(self, name, value)

        self.enable_corner_pre_turn = bool(
            self.get_parameter('enable_corner_pre_turn').value
        )
        self.allow_finish_only_test = bool(
            self.get_parameter('allow_finish_only_test').value
        )
        for name in (
            'corner_confirm_frames',
            'red_circle_confirm_frames',
            'white_bar_confirm_frames',
            'stop_zone_confirm_frames',
            'stop_zone_inside_confirm_frames',
            'align_confirm_frames',
        ):
            value = self.get_parameter(name).value
            if type(value) is not int or value < 1:
                raise ValueError(f'{name} must be an integer >= 1')
            setattr(self, name, value)

        positive_names = (
            'control_rate_hz',
            'suggested_cmd_timeout_sec',
            'detection_timeout_sec',
            'line_follower_status_timeout_sec',
            'corner_min_time_sec',
            'corner_max_time_sec',
            'corner_cooldown_sec',
            'red_circle_approach_timeout_sec',
            'turn_after_red_angle_deg',
            'turn_after_red_timeout_sec',
            'white_bar_executor_action_timeout_sec',
            'white_bar_action_timeout_sec',
            'front_jump_start_worst_case_duration_sec',
            'front_jump_finish_worst_case_duration_sec',
            'stop_zone_approach_timeout_sec',
            'align_timeout_sec',
            'inspection_action_timeout_sec',
        )
        nonnegative_names = (
            'corner_min_confidence',
            'corner_vx',
            'corner_angular_z',
            'red_circle_min_confidence',
            'red_circle_approach_speed',
            'red_circle_stop_area_ratio',
            'red_circle_stop_y_ratio',
            'turn_after_red_angular_z',
            'white_bar_min_confidence',
            'white_bar_approach_speed',
            'white_bar_stop_y_ratio',
            'stop_zone_min_confidence',
            'stop_zone_approach_speed',
            'align_min_confidence',
            'align_max_lateral_error',
            'align_max_heading_error',
            'align_max_angular_z',
            'align_heading_gain',
            'align_lateral_gain',
        )
        for name in positive_names:
            setattr(self, name, self._float_parameter(name, positive=True))
        for name in nonnegative_names:
            setattr(self, name, self._float_parameter(name, positive=False))
        if self.corner_max_time_sec < self.corner_min_time_sec:
            raise ValueError(
                'corner_max_time_sec must be >= corner_min_time_sec'
            )
        validate_white_bar_timeout_chain(
            self.front_jump_start_worst_case_duration_sec,
            self.front_jump_finish_worst_case_duration_sec,
            self.white_bar_executor_action_timeout_sec,
            self.white_bar_action_timeout_sec,
        )

    def _float_parameter(self, name, positive):
        """拒绝非有限参数，避免 YAML 产生不可控速度。"""
        value = _finite_float(self.get_parameter(name).value)
        if value is None or (value <= 0.0 if positive else value < 0.0):
            comparator = 'positive' if positive else 'nonnegative'
            raise ValueError(f'{name} must be finite and {comparator}')
        return value

    def _on_suggested_cmd(self, msg):
        """缓存巡线候选；过期或异常值会在发布前归零。"""
        self.latest_suggested_cmd = msg
        self.latest_suggested_time = time.monotonic()

    def _on_line_follower_status(self, msg):
        """只信任已启动任务的就绪 LINE_FOLLOW 状态。"""
        payload = decode_json_object(msg.data)
        self.latest_line_follower_status_time = time.monotonic()
        if payload is None:
            self.line_follower_ready = False
            self.latest_line_status = 'line_follower_status_invalid_json'
            return
        self.line_follower_ready = (
            payload.get('ready') is True
            and payload.get('nav_state') == 'LINE_FOLLOW'
            and payload.get('mission_started') is True
        )
        self.latest_line_status = str(payload.get('reason', ''))

    def _on_line_track(self, msg):
        """缓存最新线帧与序号；对齐只计算真正的新帧。"""
        self.latest_line = msg
        self.latest_line_time = time.monotonic()
        self._line_sequence += 1

    def _on_red_circle(self, msg):
        """错误阶段红圈清零，防止 START 前跨阶段累计。"""
        self.latest_red = msg
        self.latest_red_time = time.monotonic()
        if (
            self._line_follower_is_ready(self.latest_red_time)
            and self.route_core.red_detection_allowed()
            and self._detection_visible_with_confidence(
                msg,
                self.red_circle_min_confidence,
            )
        ):
            self.red_seen_count += 1
        else:
            self.red_seen_count = 0

    def _on_stop_zone(self, msg):
        """终点蓝区在未 arm 前不保留历史确认次数。"""
        self.latest_stop_zone = msg
        self.latest_stop_zone_time = time.monotonic()
        allowed = (
            self._line_follower_is_ready(self.latest_stop_zone_time)
            and self.route_core.stop_zone_detection_allowed()
        )
        valid = self._detection_visible_with_confidence(
            msg,
            self.stop_zone_min_confidence,
        )
        if allowed and valid:
            self.stop_seen_count += 1
        else:
            self.stop_seen_count = 0
        inside = (
            allowed
            and valid
            and bool(msg.inside_candidate)
        )
        self.stop_inside_count = self.stop_inside_count + 1 if inside else 0

    def _on_white_bar(self, msg):
        """只在当前显式 arm 的 START/FINISH 阶段累计白横线。"""
        self.latest_white_bar = msg
        self.latest_white_bar_time = time.monotonic()
        stage = self.white_stage_controller.active_stage
        allowed = (
            self._line_follower_is_ready(self.latest_white_bar_time)
            and self.white_stage_controller.state == f'{stage}_ARMED'
            and self.route_core.white_bar_detection_allowed(stage)
        )
        if allowed and self._detection_visible_with_confidence(
            msg,
            self.white_bar_min_confidence,
        ):
            self.white_seen_count += 1
        else:
            self.white_seen_count = 0

    def _on_corner_candidate(self, msg):
        """巡线阶段才接收角点，避免检查时转向。"""
        self.latest_corner = msg
        self.latest_corner_time = time.monotonic()
        if (
            self.enable_corner_pre_turn
            and self._line_follower_is_ready(self.latest_corner_time)
            and self.route_core.corner_detection_allowed()
            and self._detection_visible_with_confidence(
                msg,
                self.corner_min_confidence,
            )
        ):
            self.corner_seen_count += 1
        else:
            self.corner_seen_count = 0

    def _on_mission_start(self, msg):
        """新任务生成 run_id；重复 start 不改变当前阶段。"""
        if not msg.data:
            return
        if self.route_core.mission_started:
            self.get_logger().info('Ignored duplicate /mission/start')
            return
        run_id = f'line-course-{uuid.uuid4()}'
        route_event = self.route_core.mission_start(run_id)
        if not route_event.accepted:
            self._enter_emergency_stop(route_event.reason)
            return
        stage_event = self.white_stage_controller.start_run(run_id)
        self._reset_runtime_for_new_run()
        self._set_state('START_STAGE', 'mission_start')
        self._publish_white_stage_status(stage_event)
        self._publish_mission_candidate(Twist())

    def _on_mission_stop(self, msg):
        """停止幂等清除 run、请求与计数，并通知执行器。"""
        if not msg.data:
            return
        route_event = self.route_core.mission_stop()
        stage_event = self.white_stage_controller.mission_stop()
        self._clear_active_action()
        self._reset_detection_counts()
        self._reset_line_follower_readiness()
        self.align_context = ''
        self.align_seen_count = 0
        self._set_state('WAIT_START', route_event.reason)
        self._publish_white_stage_status(stage_event)
        self._publish_mission_candidate(Twist())

    def _on_white_bar_stage_command(self, msg):
        """校验命令后 arm 白横线阶段，错误顺序立即停车。"""
        # stop 清空 run_id；旧 stage 命令不能推进 WAIT_START，
        # 更不能生成新的白横线动作。
        if not self.route_core.mission_started:
            return
        payload = decode_json_object(msg.data)
        if payload is None:
            self._enter_emergency_stop('stage_command_invalid_json')
            return
        stage = payload.get('stage')
        if self._is_duplicate_stage_command(payload):
            self._publish_white_stage_status(
                self.white_stage_controller.status_event(
                    'stage_command_duplicate_ignored'
                )
            )
            return
        if not self.route_core.can_accept_stage_command(stage):
            self._enter_emergency_stop(f'illegal_stage_command_{stage}')
            return
        stage_event = self.white_stage_controller.apply_command(payload)
        if not stage_event.accepted:
            self._publish_white_stage_status(stage_event)
            self._enter_emergency_stop(stage_event.reason)
            return
        route_event = self.route_core.accept_stage_command(stage)
        self._publish_white_stage_status(stage_event)
        if not route_event.accepted:
            self._enter_emergency_stop(route_event.reason)
            return
        self.white_seen_count = 0
        # FINISH 命令只能来自 TURN_AFTER_RED；ACK 不能打断转向，
        # 或其后的 ALIGN_TO_LINE，只更新路线 phase 等待白线。
        if stage == 'START':
            self._set_state('START_STAGE', route_event.reason)
        elif self.state not in ('TURN_AFTER_RED', 'ALIGN_TO_LINE'):
            self._set_state('FINISH_STAGE', route_event.reason)
        else:
            self.last_reason = route_event.reason

    def _on_white_bar_action_status(self, msg):
        """以 executor 的 request_id/motion 状态闭环白横线。"""
        payload = decode_json_object(msg.data)
        if payload is None:
            return
        request_id = payload.get('request_id')
        if type(request_id) is int and request_id >= 0:
            self.latest_white_action_request_id = max(
                self.latest_white_action_request_id,
                request_id,
            )
        if self.white_action_expected_request_id is None:
            return
        expected_request_id = self.white_action_expected_request_id
        if request_id != expected_request_id:
            if (
                type(request_id) is int
                and request_id > expected_request_id
            ):
                self._enter_emergency_stop(
                    'white_bar_action_request_id_mismatch'
                )
            return
        status = payload.get('status')
        motion_name = payload.get('motion_name')
        if (
            type(status) is not str
            or motion_name != self.active_action
        ):
            self._enter_emergency_stop('white_bar_action_status_malformed')
            return
        if status == 'SUCCEEDED':
            self._complete_white_bar_action()
        elif status in WHITE_ACTION_FAILURE_STATES:
            self._fail_white_bar_action(
                str(payload.get('reason', 'white_bar_action_failed'))
            )

    def _on_white_bar_action_done(self, msg):
        """旧 done Topic 只作兼容；成功依赖匹配状态。"""
        if msg.data and self.white_action_expected_request_id is not None:
            self.get_logger().debug(
                'White-bar done observed; matching status is required for '
                'route completion'
            )

    def _on_inspection_action_status(self, msg):
        """只接收当前检查结果，旧 run/request 结果忽略。"""
        payload = decode_json_object(msg.data)
        if payload is None:
            return
        run_id = payload.get('run_id')
        request_id = payload.get('request_id')
        if (
            run_id != self.route_core.run_id
            or request_id != self.route_core.active_request_id
            or not request_id
        ):
            return
        state = payload.get('state')
        success = payload.get('success')
        if type(state) is not str or type(success) is not bool:
            self._enter_emergency_stop('inspection_action_status_malformed')
            return
        if state == 'SUCCEEDED' and success:
            route_event = self.route_core.inspection_action_succeeded(
                request_id
            )
            if not route_event.accepted:
                self._enter_emergency_stop(route_event.reason)
                return
            self._clear_active_action()
            self._reset_detection_counts()
            self._set_state('TURN_AFTER_RED', route_event.reason)
            self._publish_mission_candidate(Twist())
            return
        if state in INSPECTION_FAILURE_STATES or (
            state == 'SUCCEEDED' and not success
        ):
            route_event = self.route_core.inspection_action_failed(
                request_id,
                str(payload.get('reason', 'inspection_action_failed')),
            )
            self._enter_emergency_stop(route_event.reason)
            return
        if state not in (
            'ARMED',
            'WAIT_SIGN',
            'COMMAND_READY',
            'WAIT_ZERO',
            'RUNNING',
            # helper 退出、进程组回收和锁释放尚未全部完成时，执行器必须保持
            # active；路线只能维持零候选等待最终状态，不能把正常停止误判故障。
            'CLEANUP_PENDING',
        ):
            self._enter_emergency_stop(
                'inspection_action_status_unknown_state'
            )
            return
        running = state in (
            'COMMAND_READY',
            'WAIT_ZERO',
            'RUNNING',
            'CLEANUP_PENDING',
        )
        route_event = self.route_core.inspection_action_progress(
            request_id,
            running=running,
        )
        if route_event.accepted and running:
            self._set_state('INSPECTION_ACTION', 'inspection_action_progress')

    def _on_control_timer(self):
        """零速优先的单循环，驱动候选速度和路线状态。"""
        now = time.monotonic()
        if self.state in ('WAIT_START', 'FINAL_STOP', 'EMERGENCY_STOP'):
            self._publish_mission_candidate(Twist())
            return
        if not self.route_core.mission_started:
            self._set_state('WAIT_START', 'mission_not_started')
            self._publish_mission_candidate(Twist())
            return
        if self.state == 'HANDLE_WHITE_BAR':
            self._control_white_bar_wait(now)
        elif self.state == 'APPROACH_RED_CIRCLE':
            self._control_red_approach(now)
        elif self.state in ('INSPECTION_WAIT_SIGN', 'INSPECTION_ACTION'):
            self._control_inspection_wait(now)
        elif self.state == 'TURN_AFTER_RED':
            self._control_turn_after_red(now)
        elif self.state == 'CORNER_PRE_TURN':
            self._control_corner_turn(now)
        elif self.state == 'ALIGN_TO_LINE':
            self._control_align_to_line(now)
        elif self.state == 'APPROACH_STOP_ZONE':
            self._control_stop_zone_approach(now)
        elif self.state in (
            'START_STAGE',
            'MID_ROUTE',
            'POST_INSPECTION',
            'FINISH_STAGE',
            'FINAL_ZONE_ARMED',
        ):
            self._control_route_follow(now)
        else:
            self._enter_emergency_stop(f'unhandled_node_state_{self.state}')

    def _control_route_follow(self, now):
        """正常路线先验证 START_READY，再处理许可事件。"""
        if not self._line_follower_is_ready(now):
            self._reset_detection_counts()
            self._publish_mission_candidate(Twist())
            return
        phase = self.route_core.route_phase
        if phase == 'START_STAGE':
            self._control_armed_white_bar(now)
            return
        if phase == 'MID_ROUTE':
            if self._red_confirmed(now):
                route_event = self.route_core.red_circle_confirmed()
                if route_event.accepted:
                    self._reset_detection_counts()
                    self._set_state('APPROACH_RED_CIRCLE', route_event.reason)
                    self._publish_mission_candidate(Twist())
                return
            if self._corner_confirmed(now):
                self._start_corner_turn()
                return
            self._publish_suggested_or_zero(now)
            return
        if phase == 'FINISH_STAGE':
            if self._white_bar_confirmed(now):
                self._control_armed_white_bar(now)
                return
            if self._corner_confirmed(now):
                self._start_corner_turn()
                return
            self._publish_suggested_or_zero(now)
            return
        if phase == 'FINAL_ZONE_ARMED':
            if self._stop_zone_confirmed(now):
                route_event = self.route_core.stop_zone_confirmed()
                if route_event.accepted:
                    self._set_state('APPROACH_STOP_ZONE', route_event.reason)
                    self._publish_mission_candidate(Twist())
                return
            self._publish_suggested_or_zero(now)
            return
        if phase == 'POST_INSPECTION':
            self._publish_suggested_or_zero(now)
            return
        self._enter_emergency_stop(f'route_phase_state_mismatch_{phase}')

    def _control_armed_white_bar(self, now):
        """白线低速接近阈值，达到后只请求一次 Action。"""
        stage = self.white_stage_controller.active_stage
        if not self.route_core.white_bar_detection_allowed(stage):
            self._publish_mission_candidate(Twist())
            return
        if not self._white_bar_confirmed(now):
            self._publish_suggested_or_zero(now)
            return
        bar = self.latest_white_bar
        center_y = _finite_float(getattr(bar, 'center_y', None))
        if center_y is None:
            self._enter_emergency_stop('white_bar_center_y_non_finite')
            return
        stage_event = self.white_stage_controller.white_bar_event(
            center_y >= self.white_bar_stop_y_ratio
        )
        if stage_event.action == 'APPROACH':
            cmd = self._copy_suggested_cmd(now)
            cmd.linear.x = self.white_bar_approach_speed
            self._publish_mission_candidate(cmd)
            return
        if stage_event.action == 'SEND_REQUEST':
            route_event = self.route_core.white_bar_action_started(stage)
            if not route_event.accepted:
                self._publish_white_stage_status(
                    self.white_stage_controller.action_fault(
                        route_event.reason
                    )
                )
                self._enter_emergency_stop(route_event.reason)
                return
            self.white_seen_count = 0
            self.white_action_started_time = now
            self.white_action_expected_request_id = (
                self.latest_white_action_request_id + 1
            )
            self.active_action = stage_event.motion_name
            self._set_state('HANDLE_WHITE_BAR', stage_event.reason)
            self._publish_white_stage_status(stage_event)
            request = String()
            request.data = stage_event.motion_name
            self.white_bar_action_request_publisher.publish(request)
            self._publish_mission_candidate(Twist())
            return
        if stage_event.action in ('NOT_ARMED', 'FAULTED'):
            self._publish_white_stage_status(stage_event)
            self._enter_emergency_stop(stage_event.reason)
            return
        self._publish_mission_candidate(Twist())

    def _control_white_bar_wait(self, now):
        """Action 期间持续零候选；路线层超时会故障关闭。"""
        self._publish_mission_candidate(Twist())
        if (
            self.white_action_started_time is not None
            and now - self.white_action_started_time
            >= self.white_bar_action_timeout_sec
        ):
            self._fail_white_bar_action('white_bar_action_timeout')

    def _complete_white_bar_action(self):
        """匹配 executor 成功后，零前进重新找线。"""
        stage = self.white_stage_controller.active_stage
        stage_event = self.white_stage_controller.complete_action(True)
        if not stage_event.accepted:
            self._enter_emergency_stop(stage_event.reason)
            return
        route_event = self.route_core.white_bar_action_completed(stage)
        self._publish_white_stage_status(stage_event)
        if not route_event.accepted:
            self._enter_emergency_stop(route_event.reason)
            return
        self._clear_active_action()
        self._reset_detection_counts()
        context = 'start' if stage == 'START' else 'finish'
        self._begin_align(context, route_event.reason)

    def _fail_white_bar_action(self, reason):
        """失败不能发布成功 done，阶段与路线锁定故障。"""
        stage_event = self.white_stage_controller.action_fault(reason)
        self._publish_white_stage_status(stage_event)
        route_event = self.route_core.white_bar_action_failed(reason)
        self._clear_active_action()
        self._enter_emergency_stop(route_event.reason)

    def _control_red_approach(self, now):
        """红圈低速靠近，到位后检查并零速等待。"""
        # 靠近状态仍校验巡线，避免状态失效后继续盲走。
        if not self._line_follower_is_ready(now):
            self._publish_mission_candidate(Twist())
            return
        if not self._is_fresh(self.latest_red_time, now):
            self._enter_emergency_stop('red_circle_detection_stale')
            return
        red = self.latest_red
        area = _finite_float(getattr(red, 'area_ratio', None))
        center_y = _finite_float(getattr(red, 'center_y', None))
        if area is None or center_y is None:
            self._enter_emergency_stop('red_circle_geometry_non_finite')
            return
        if (
            area >= self.red_circle_stop_area_ratio
            or center_y >= self.red_circle_stop_y_ratio
        ):
            request_id = f'inspection-{uuid.uuid4()}'
            route_event = self.route_core.red_circle_reached(request_id)
            if not route_event.accepted:
                self._enter_emergency_stop(route_event.reason)
                return
            request = String()
            request.data = json.dumps({
                'run_id': self.route_core.run_id,
                'request_id': request_id,
                'action': 'detect_and_execute_warning',
            }, separators=(',', ':'))
            self.inspection_action_request_publisher.publish(request)
            self.inspection_request_started_time = now
            self.active_action = 'inspection'
            self._set_state('INSPECTION_WAIT_SIGN', route_event.reason)
            self._publish_mission_candidate(Twist())
            return
        if now - self.state_enter_time >= self.red_circle_approach_timeout_sec:
            self._enter_emergency_stop('red_circle_approach_timeout')
            return
        cmd = self._copy_suggested_cmd(now)
        cmd.linear.x = self.red_circle_approach_speed
        self._publish_mission_candidate(cmd)

    def _control_inspection_wait(self, now):
        """检查识别与 SDK 动作期间候选恒为零，超时急停。"""
        self._publish_mission_candidate(Twist())
        if (
            self.inspection_request_started_time is not None
            and now - self.inspection_request_started_time
            >= self.inspection_action_timeout_sec
        ):
            self._enter_emergency_stop('inspection_action_timeout')

    def _control_turn_after_red(self, now):
        """检查成功后执行 TURN_AFTER_RED，作为 FINISH 里程碑。"""
        if self.turn_after_red_angular_z <= 0.0:
            self._enter_emergency_stop('turn_after_red_zero_speed')
            return
        target_duration = min(
            math.radians(self.turn_after_red_angle_deg)
            / self.turn_after_red_angular_z,
            self.turn_after_red_timeout_sec,
        )
        if now - self.state_enter_time >= target_duration:
            self._begin_align('red', 'turn_after_red_complete')
            return
        cmd = Twist()
        cmd.angular.z = (
            self._turn_direction(self.turn_after_red_direction)
            * self.turn_after_red_angular_z
        )
        self._publish_mission_candidate(cmd)

    def _start_corner_turn(self):
        """确认角点后独立转向，居中后才恢复巡线。"""
        route_event = self.route_core.corner_confirmed()
        if not route_event.accepted:
            return
        self.corner_seen_count = 0
        self._set_state('CORNER_PRE_TURN', route_event.reason)
        self._publish_mission_candidate(Twist())

    def _control_corner_turn(self, now):
        """角点局部转向有时长边界，防止无限转圈。"""
        # 转角不能绕过 START_READY 门，失联时只保留零速。
        if not self._line_follower_is_ready(now):
            self._publish_mission_candidate(Twist())
            return
        elapsed = now - self.state_enter_time
        if elapsed >= self.corner_max_time_sec:
            self._begin_align('corner', 'corner_turn_timeout_boundary')
            return
        if (
            elapsed >= self.corner_min_time_sec
            and self._line_satisfies_align(now)
        ):
            self._begin_align('corner', 'corner_turn_line_visible')
            return
        cmd = Twist()
        cmd.linear.x = self.corner_vx
        cmd.angular.z = (
            self._turn_direction(
                self.corner_turn_direction,
                self.latest_corner,
            ) * self.corner_angular_z
        )
        self._publish_mission_candidate(cmd)

    def _begin_align(self, context, reason):
        """动作后进入 ALIGN_TO_LINE，禁止带线速度恢复巡线。"""
        self.align_context = context
        self.align_seen_count = 0
        self._align_last_line_sequence = -1
        self._set_state('ALIGN_TO_LINE', reason)
        self._publish_mission_candidate(Twist())

    def _control_align_to_line(self, now):
        """只以有限、可见、连续居中的线帧对齐。

        线速度始终为零。
        """
        if now - self.state_enter_time >= self.align_timeout_sec:
            self._enter_emergency_stop('align_to_line_timeout')
            return
        if self._line_sequence != self._align_last_line_sequence:
            self._align_last_line_sequence = self._line_sequence
            if self._line_satisfies_align(now):
                self.align_seen_count += 1
            else:
                self.align_seen_count = 0
        if self.align_seen_count >= self.align_confirm_frames:
            route_event = self.route_core.alignment_completed(
                self.align_context
            )
            if not route_event.accepted:
                self._enter_emergency_stop(route_event.reason)
                return
            self._set_state(route_event.route_phase, route_event.reason)
            self.align_context = ''
            self._publish_mission_candidate(Twist())
            return
        cmd = Twist()
        if self._is_fresh(self.latest_line_time, now):
            line = self.latest_line
            lateral = _finite_float(getattr(line, 'lateral_error', None))
            heading = _finite_float(getattr(line, 'heading_error', None))
            if lateral is not None and heading is not None:
                angular = -(
                    self.align_heading_gain * heading
                    + self.align_lateral_gain * lateral
                )
                cmd.angular.z = max(
                    -self.align_max_angular_z,
                    min(self.align_max_angular_z, angular),
                )
        self._publish_mission_candidate(cmd)

    def _control_stop_zone_approach(self, now):
        """终点区连续 inside 才 FINAL_STOP；失效时不盲走。"""
        # 终点微靠近仍行驶，状态过期时禁止前进候选。
        if not self._line_follower_is_ready(now):
            self._publish_mission_candidate(Twist())
            return
        if not self._is_fresh(self.latest_stop_zone_time, now):
            self._enter_emergency_stop('stop_zone_detection_stale')
            return
        if self.stop_inside_count >= self.stop_zone_inside_confirm_frames:
            route_event = self.route_core.stop_zone_inside_confirmed()
            if not route_event.accepted:
                self._enter_emergency_stop(route_event.reason)
                return
            self._set_state('FINAL_STOP', route_event.reason)
            self._publish_mission_candidate(Twist())
            return
        if now - self.state_enter_time >= self.stop_zone_approach_timeout_sec:
            self._enter_emergency_stop('stop_zone_approach_timeout')
            return
        cmd = self._copy_suggested_cmd(now)
        cmd.linear.x = self.stop_zone_approach_speed
        self._publish_mission_candidate(cmd)

    def _red_confirmed(self, now):
        """红圈确认必须同时满足新鲜度与当前核心许可。"""
        return (
            self._is_fresh(self.latest_red_time, now)
            and self.route_core.red_detection_allowed()
            and self.red_seen_count >= self.red_circle_confirm_frames
        )

    def _white_bar_confirmed(self, now):
        """白线确认只服务当前 arm 阶段，不能按次数推断。"""
        stage = self.white_stage_controller.active_stage
        return (
            self._is_fresh(self.latest_white_bar_time, now)
            and self.route_core.white_bar_detection_allowed(stage)
            and self.white_seen_count >= self.white_bar_confirm_frames
        )

    def _corner_confirmed(self, now):
        """角点确认遵守 phase gate 和本地配置开关。"""
        return (
            self.enable_corner_pre_turn
            and self._is_fresh(self.latest_corner_time, now)
            and self.route_core.corner_detection_allowed()
            and self.corner_seen_count >= self.corner_confirm_frames
        )

    def _stop_zone_confirmed(self, now):
        """终点区只在 FINAL_ZONE_ARMED 处理稳定可见检测。"""
        return (
            self._is_fresh(self.latest_stop_zone_time, now)
            and self.route_core.stop_zone_detection_allowed()
            and self.stop_seen_count >= self.stop_zone_confirm_frames
        )

    def _line_satisfies_align(self, now):
        """集中校验对齐时效、可见性、有限值和误差。"""
        if not self._is_fresh(self.latest_line_time, now):
            return False
        line = self.latest_line
        if line is None or not bool(line.line_visible):
            return False
        confidence = _finite_float(line.confidence)
        lateral = _finite_float(line.lateral_error)
        heading = _finite_float(line.heading_error)
        return (
            confidence is not None
            and lateral is not None
            and heading is not None
            and confidence >= self.align_min_confidence
            and abs(lateral) <= self.align_max_lateral_error
            and abs(heading) <= self.align_max_heading_error
        )

    def _detection_visible_with_confidence(self, msg, min_confidence):
        """特殊目标先过有限置信度检查，低置信不累计。"""
        confidence = _finite_float(getattr(msg, 'confidence', None))
        return (
            bool(getattr(msg, 'visible', False))
            and confidence is not None
            and confidence >= min_confidence
        )

    def _line_follower_is_ready(self, now):
        """状态过期或非 START_READY 时阻止路线事件和候选。"""
        return (
            self.line_follower_ready
            and self._is_fresh(
                self.latest_line_follower_status_time,
                now,
                self.line_follower_status_timeout_sec,
            )
        )

    def _is_fresh(self, receive_time, now, timeout=None):
        """以单调时钟判断新鲜度，未收到输入不新鲜。"""
        allowed_age = (
            self.detection_timeout_sec if timeout is None else timeout
        )
        return (
            receive_time is not None
            and 0.0 <= now - receive_time <= allowed_age
        )

    def _copy_suggested_cmd(self, now):
        """复制新鲜有限的巡线建议；异常字段归零。"""
        cmd = Twist()
        if not self._is_fresh(
            self.latest_suggested_time,
            now,
            self.suggested_cmd_timeout_sec,
        ):
            return cmd
        source = self.latest_suggested_cmd
        source_values = (
            source.linear.x,
            source.linear.y,
            source.linear.z,
            source.angular.x,
            source.angular.y,
            source.angular.z,
        )
        values = tuple(_finite_float(value) for value in source_values)
        if any(value is None for value in values):
            return cmd
        cmd.linear.x, cmd.linear.y, cmd.linear.z = values[:3]
        cmd.angular.x, cmd.angular.y, cmd.angular.z = values[3:]
        return cmd

    def _publish_suggested_or_zero(self, now):
        """统一发布任务候选，不允许绕过有限值检查。"""
        self._publish_mission_candidate(self._copy_suggested_cmd(now))

    def _publish_mission_candidate(self, cmd):
        """发布 command_mux 候选并同步 JSON，不触碰速度。"""
        values = (
            cmd.linear.x,
            cmd.linear.y,
            cmd.linear.z,
            cmd.angular.x,
            cmd.angular.y,
            cmd.angular.z,
        )
        if any(_finite_float(value) is None for value in values):
            cmd = Twist()
            self.route_core.fault('non_finite_mission_candidate')
            self._set_state('EMERGENCY_STOP', 'non_finite_mission_candidate')
        # launch 关闭会先使 ROS context 失效；忽略该窗口内的定时器尾部发布，
        # 但运行态 publisher 错误仍必须抛出，避免吞掉真实控制故障。
        if not rclpy.ok():
            return
        try:
            self.cmd_publisher.publish(cmd)
            self._publish_state(cmd)
        except Exception:
            if rclpy.ok():
                raise

    def _publish_state(self, cmd):
        """公开验收状态；旗标、请求和候选来自真相。"""
        route = self.route_core
        msg = String()
        msg.data = json.dumps({
            'state': self.state,
            'route_phase': route.route_phase,
            'mission_started': bool(route.mission_started),
            'run_id': route.run_id,
            'start_jump_completed': bool(route.start_jump_completed),
            'inspection_completed': bool(route.inspection_completed),
            'finish_jump_completed': bool(route.finish_jump_completed),
            'final_zone_armed': bool(route.final_zone_armed),
            'active_request_id': route.active_request_id,
            'fault_reason': route.fault_reason,
            'reason': self.last_reason,
            'active_action': self.active_action,
            'white_bar_stage_state': self.white_stage_controller.state,
            'white_bar_stage_run_id': self.white_stage_controller.run_id,
            'white_bar_stage': self.white_stage_controller.active_stage,
            'white_bar_action_request_sent': bool(
                self.white_stage_controller.request_sent
            ),
            'white_bar_confirm_count': self.white_seen_count,
            'red_confirm_count': self.red_seen_count,
            'stop_zone_confirm_count': self.stop_seen_count,
            'stop_zone_inside_count': self.stop_inside_count,
            'corner_confirm_count': self.corner_seen_count,
            'line_follower_ready': bool(self.line_follower_ready),
            'line_follower_status_reason': self.latest_line_status,
            'align_context': self.align_context,
            'align_confirm_count': self.align_seen_count,
            'final_vx': float(cmd.linear.x),
            'final_wz': float(cmd.angular.z),
        }, separators=(',', ':'))
        self.state_publisher.publish(msg)

    def _publish_white_stage_status(self, event):
        """保留白线 stage JSON 契约，供发布器确认。"""
        msg = String()
        msg.data = json.dumps({
            'run_id': event.run_id,
            'state': event.state,
            'active_stage': event.active_stage,
            'motion_name': event.motion_name,
            'last_sequence': event.last_sequence,
            'reason': event.reason,
            'request_sent': event.request_sent,
            'action_done': event.action_done,
        }, separators=(',', ':'))
        self.white_bar_stage_status_publisher.publish(msg)

    def _enter_emergency_stop(self, reason):
        """顺序或数据错误锁定路线故障，发布零候选。"""
        route_event = self.route_core.fault(reason)
        if self.white_stage_controller.state in (
            'START_RUNNING',
            'FINISH_RUNNING',
        ):
            self._publish_white_stage_status(
                self.white_stage_controller.action_fault(route_event.reason)
            )
        self._clear_active_action()
        self._reset_detection_counts()
        self._set_state('EMERGENCY_STOP', route_event.reason)
        self._publish_mission_candidate(Twist())

    def _set_state(self, new_state, reason):
        """状态转换更新诊断原因，拒绝未知状态盲走。"""
        if new_state not in LINE_COURSE_NODE_STATES:
            new_state = 'EMERGENCY_STOP'
            reason = f'invalid_node_state_{new_state}'
        if new_state != self.state:
            self.get_logger().info(
                f'[LINE_COURSE] {self.state} -> {new_state}: {reason}'
            )
            self.state = new_state
            self.state_enter_time = time.monotonic()
        self.last_reason = str(reason)

    def _reset_runtime_for_new_run(self):
        """新任务只在显式 start 后建立，不复用旧数据。"""
        self._clear_active_action()
        self._reset_detection_counts()
        self._reset_line_follower_readiness()
        self.align_context = ''
        self.align_seen_count = 0
        self._align_last_line_sequence = -1

    def _reset_detection_counts(self):
        """错误阶段、stop 与新任务都清理跨检测计数。"""
        self.red_seen_count = 0
        self.stop_seen_count = 0
        self.stop_inside_count = 0
        self.white_seen_count = 0
        self.corner_seen_count = 0

    def _reset_line_follower_readiness(self):
        """stop/new-run 丢弃旧 ready/候选，强制新的就绪闭环。"""
        self.line_follower_ready = False
        self.latest_line_follower_status_time = None
        self.latest_line_status = 'mission_start_or_stop_waiting_for_ready'
        self.latest_suggested_cmd = Twist()
        self.latest_suggested_time = None

    def _clear_active_action(self):
        """清除等待时钟和 request 关联，避免旧结果匹配。"""
        self.active_action = ''
        self.white_action_started_time = None
        self.white_action_expected_request_id = None
        self.inspection_request_started_time = None

    def _is_duplicate_stage_command(self, payload):
        """允许 ACK 丢失时重发同一命令，回退序号故障。"""
        if type(payload) is not dict:
            return False
        stage = payload.get('stage')
        sequence = payload.get('sequence')
        return (
            stage in ('START', 'FINISH')
            and payload.get('run_id') == self.white_stage_controller.run_id
            and type(sequence) is int
            and sequence == self.white_stage_controller.last_sequence
            and stage == self.white_stage_controller.active_stage
        )

    @staticmethod
    def _turn_direction(direction, detection=None):
        """保留 left/right/hint 配置，默认左转且不猜测方向。"""
        normalized = str(direction).strip().lower()
        if normalized == 'hint' and detection is not None:
            normalized = str(
                getattr(detection, 'direction_hint', '')
            ).strip().lower()
        return -1.0 if normalized == 'right' else 1.0


def main(args=None):
    """运行路线节点；不创建最终速度发布者。"""
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
