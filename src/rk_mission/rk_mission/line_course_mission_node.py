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
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, String

from rk_interfaces.action import ExecuteArmTask
from rk_interfaces.msg import LineTrack, SpecialTargetDetection
from rk_mission.non_arm_route_phase_core import NonArmRoutePhaseCore
from rk_mission.non_arm_route_phase_core import (
    validate_white_bar_timeout_chain,
)
from rk_mission.transfer_route_core import (
    COMPLETE as TRANSFER_COMPLETE,
    RIGHT_CORNER_SEARCH,
    SAFE_STOP as TRANSFER_SAFE_STOP,
    TransferInputs,
    TransferRouteConfig,
    TransferRouteCore,
)
from rk_mission.white_bar_blind_core import FOLLOW
from rk_mission.white_bar_blind_core import REQUEST_ACTION
from rk_mission.white_bar_blind_core import ZERO
from rk_mission.white_bar_blind_core import WhiteBarBlindApproachCore
from rk_mission.white_bar_stage_core import WhiteBarStageController


LINE_COURSE_NODE_STATES = frozenset((
    'WAIT_START',
    'START_STAGE',
    'MID_ROUTE',
    'TRANSFER_RIGHT_CORNER_SEARCH',
    'TRANSFER_PRE_CORNER_FORWARD',
    'TRANSFER_PRE_CORNER_STOP',
    'TRANSFER_PRE_CORNER_TURN',
    'TRANSFER_PRE_CORNER_REACQUIRE',
    'TRANSFER_ARC_APPROACH',
    'TRANSFER_ARC_TO_TASK',
    'TRANSFER_TASK_STOP',
    'TRANSFER_PLACE_TASK',
    'TRANSFER_PICK_TASK',
    'TRANSFER_ARC_EXIT_STOP',
    'TRANSFER_ARC_TO_EXIT',
    'TRANSFER_LINE_REACQUIRE',
    'TRANSFER_COMPLETE',
    'INSPECTION_APPROACH',
    # 红圆后的状态名称是现场验收接口；不要复用旧的“靠近/动作”泛名。
    'RED_TARGET_SEARCH',
    'RED_APPROACH',
    'RED_TURN_LEFT',
    'RED_REVERSE_APPROACH',
    'RED_REVERSE_STOP_CONFIRM',
    'RED_REVERSE_NOT_CALIBRATED',
    'RED_INSPECTION_PREP',
    'RED_INSPECTION',
    'RED_INSPECTION_TIMEOUT',
    'RED_ACTION_EXECUTION',
    'RED_CLASSIC_RECOVERY',
    'RED_TURN_RIGHT',
    'POST_INSPECTION',
    'FINISH_STAGE',
    'WHITE_BAR_BLIND_FORWARD',
    'WHITE_BAR_BLIND_LINE_RECOVERY',
    'HANDLE_WHITE_BAR',
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


def corner_cooldown_elapsed(now, last_completed_time, cooldown_sec):
    """通用直角只在对齐完成后的 cooldown 结束才能再次消费。"""
    values = tuple(
        _finite_float(value)
        for value in (now, last_completed_time, cooldown_sec)
    )
    return (
        all(value is not None for value in values)
        and values[2] >= 0.0
        and values[0] - values[1] >= values[2]
    )


class LineCourseMissionNode(Node):
    """正式赛道阶段门控；异常时只输出零候选速度。"""

    def __init__(self):
        """声明 ROS 输入输出和安全参数，发布初始状态。"""
        super().__init__('line_course_mission_node')
        self._declare_parameters()
        self._read_parameters()

        self.route_core = NonArmRoutePhaseCore()
        self.transfer_core = TransferRouteCore(self._transfer_config())
        self.white_stage_controller = WhiteBarStageController(
            self.allow_finish_only_test
        )
        self.white_bar_blind_core = WhiteBarBlindApproachCore(
            confirm_frames=self.white_bar_confirm_frames,
            min_confidence=self.white_bar_min_confidence,
            blind_duration_sec=self.white_bar_blind_forward_duration_sec,
            detection_timeout_sec=self.detection_timeout_sec,
            line_recovery_timeout_sec=(
                self.white_bar_line_recovery_timeout_sec
            ),
            line_recovery_confirm_frames=self.white_bar_confirm_frames,
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
        self.latest_gait_lock = True
        self.latest_gait_lock_time = None
        self.latest_mux_status = None
        self.latest_mux_status_time = None
        self.latest_corner = None
        self.latest_corner_time = None
        # 候选可能在转弯期间更新或消失；进入角点后方向必须锁存，
        # 不能由后续帧反向改写实际命令。
        self.active_corner_direction = ''
        self.active_corner_direction_source = ''
        self.active_corner_detected_time = None
        self.active_corner_confidence = 0.0
        self.red_seen_count = 0
        self.stop_seen_count = 0
        self.stop_inside_count = 0
        self.white_seen_count = 0
        self.corner_seen_count = 0
        self.last_corner_completed_time = -1.0e9
        self.white_action_started_time = None
        self.white_action_expected_request_id = None
        self.latest_white_action_request_id = 0
        self.inspection_request_started_time = None
        # 红圆正式链路的时钟只在指定状态使用。后退单独记录有效输出时间，
        # 不能以状态墙钟代替实际倒车时间。
        self.red_first_seen_time = None
        self.red_turn_start_yaw = None
        self.red_reverse_started_time = None
        self.red_reverse_active_elapsed = 0.0
        self.red_reverse_last_tick = None
        self.red_stop_status_sequence = 0
        self.red_classic_status_sequence = 0
        self.latest_odom_yaw = None
        self.latest_odom_time = None
        self.latest_final_cmd = Twist()
        self.latest_final_cmd_time = None
        self.latest_sdk_motion_status = None
        self.latest_sdk_motion_status_time = None
        self.transfer_action_name = ''
        self.transfer_action_goal_handle = None

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
        # 只复用既有 ExecuteArmTask 接口，不修改机械臂轨迹或动作名。
        self.arm_action_client = ActionClient(
            self, ExecuteArmTask, self.transfer_arm_action_name
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
            Bool,
            self.gait_control_lock_topic,
            self._on_gait_control_lock,
            10,
        )
        self.create_subscription(
            String,
            self.cmd_mux_status_topic,
            self._on_cmd_mux_status,
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
        self.create_subscription(
            Odometry,
            self.odom_topic,
            self._on_odom,
            10,
        )
        self.create_subscription(
            Twist,
            self.final_cmd_topic,
            self._on_final_cmd,
            10,
        )
        self.create_subscription(
            String,
            self.sdk_motion_status_topic,
            self._on_sdk_motion_status,
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
            'gait_control_lock_topic': '/gait/control_lock',
            'cmd_mux_status_topic': '/control/cmd_mux_status',
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
            'odom_topic': '/utlidar/robot_odom',
            'final_cmd_topic': '/navigation/cmd_vel',
            # SDK 状态转发器提供真实 Move/StopMove 返回值与经典步态验证事件。
            'sdk_motion_status_topic': '/go2/sdk_motion_status',
            'transfer_arm_action_name': '/arm/execute_task',
        }
        for name, value in topic_defaults.items():
            self.declare_parameter(name, value)

        defaults = {
            'control_rate_hz': 10.0,
            'suggested_cmd_timeout_sec': 0.5,
            'detection_timeout_sec': 0.8,
            'line_follower_status_timeout_sec': 1.0,
            'enable_corner_pre_turn': True,
            # 正式 90° 弯由本节点发布 mission candidate 并独占转向控制权。
            'corner_owner': 'mission',
            'corner_turn_direction': 'left',
            'corner_confirm_frames': 3,
            'corner_min_confidence': 0.45,
            'corner_vx': 0.03,
            'corner_angular_z': 0.32,
            'corner_min_time_sec': 0.8,
            'corner_max_time_sec': 2.5,
            'corner_cooldown_sec': 3.0,
            'red_circle_confirm_frames': 1,
            'red_circle_min_confidence': 0.55,
            # 省赛红圆合同：首次可靠检测后，仍由正式 follower 巡线 3.9 秒。
            'red_approach_duration_sec': 3.9,
            'red_turn_left_angle_deg': 83.0,
            'red_turn_left_angular_z': 1.0,
            # 0.0 是显式“尚未标定”，生产运行时必须 fail-closed。
            'red_reverse_speed_mps': 0.0,
            'red_reverse_duration_sec': 0.0,
            'red_turn_right_angle_deg': 80.0,
            # 右转参数是幅值；固定路线方向由控制代码统一取负，避免 YAML 符号漂移。
            'red_turn_right_angular_z': 1.0,
            'red_yaw_timeout_sec': 8.0,
            'red_stop_confirm_timeout_sec': 3.0,
            'red_classic_recovery_timeout_sec': 8.0,
            'white_bar_confirm_frames': 3,
            'white_bar_min_confidence': 0.55,
            # 0.0 表示尚未标定；stable lost 后必须零速且禁止 action。
            'white_bar_blind_forward_duration_sec': 0.0,
            # 与正式 follower short_lost_timeout 共用同一恢复合同。
            'white_bar_line_recovery_timeout_sec': 0.75,
            # DEPRECATED / rollback-only：正式路径不再读取以下两项作控制。
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
            # TRANSFER 固定路线默认全部未实体标定，禁止非零运动。
            'right_corner_confirm_frames': 3,
            'right_corner_motion_calibrated': False,
            'right_corner_forward_speed': 0.0,
            'right_corner_forward_active_time_sec': 0.0,
            'right_corner_forward_timeout_sec': 20.0,
            'right_corner_turn_wz': 0.0,
            'right_corner_target_yaw_deg': 90.0,
            'right_corner_turn_timeout_sec': 8.0,
            'transfer_arc_entry_calibrated': False,
            'transfer_arc_entry_confirm_frames': 3,
            'transfer_arc_motion_calibrated': False,
            'transfer_arc_vx': 0.0,
            'transfer_arc_wz': 0.0,
            'transfer_arc_task_stop_yaw_deg': 0.0,
            'transfer_arc_exit_delta_yaw_deg': 0.0,
            'transfer_arc_yaw_timeout_sec': 12.0,
            'transfer_yaw_wrong_direction_tolerance_deg': 3.0,
            'transfer_zero_confirm_frames': 3,
            'transfer_zero_epsilon': 0.001,
            'transfer_final_cmd_tolerance': 0.005,
            'transfer_odom_timeout_sec': 0.5,
            'transfer_arm_action_timeout_sec': 90.0,
            'transfer_reacquire_max_lateral_error': 0.75,
            'transfer_place_task': 'transfer_place',
            'transfer_pick_task': 'transfer_pick',
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
            'gait_control_lock_topic',
            'cmd_mux_status_topic',
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
            'odom_topic',
            'final_cmd_topic',
            'sdk_motion_status_topic',
            'transfer_arm_action_name',
            'transfer_place_task',
            'transfer_pick_task',
        )
        for name in topic_names:
            value = str(self.get_parameter(name).value).strip()
            if not value:
                raise ValueError(f'{name} must not be empty')
            setattr(self, name, value)

        self.enable_corner_pre_turn = bool(
            self.get_parameter('enable_corner_pre_turn').value
        )
        self.corner_owner = str(
            self.get_parameter('corner_owner').value
        ).strip().lower()
        if self.corner_owner != 'mission':
            raise ValueError(
                'corner_owner must be mission: mission is the sole formal '
                '90-degree corner controller'
            )
        self.corner_turn_direction = str(
            self.get_parameter('corner_turn_direction').value
        ).strip().lower()
        if self.corner_turn_direction not in ('left', 'right', 'hint'):
            raise ValueError(
                'corner_turn_direction must be left, right, or hint'
            )
        self.allow_finish_only_test = bool(
            self.get_parameter('allow_finish_only_test').value
        )
        self.right_corner_motion_calibrated = bool(
            self.get_parameter('right_corner_motion_calibrated').value
        )
        self.transfer_arc_entry_calibrated = bool(
            self.get_parameter('transfer_arc_entry_calibrated').value
        )
        self.transfer_arc_motion_calibrated = bool(
            self.get_parameter('transfer_arc_motion_calibrated').value
        )
        for name in (
            'corner_confirm_frames',
            'red_circle_confirm_frames',
            'white_bar_confirm_frames',
            'stop_zone_confirm_frames',
            'stop_zone_inside_confirm_frames',
            'align_confirm_frames',
            'right_corner_confirm_frames',
            'transfer_arc_entry_confirm_frames',
            'transfer_zero_confirm_frames',
            'reacquire_stable_frames',
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
            'red_approach_duration_sec',
            'red_turn_left_angle_deg',
            'red_turn_right_angle_deg',
            'red_yaw_timeout_sec',
            'red_stop_confirm_timeout_sec',
            'red_classic_recovery_timeout_sec',
            'white_bar_executor_action_timeout_sec',
            'white_bar_action_timeout_sec',
            'white_bar_line_recovery_timeout_sec',
            'front_jump_start_worst_case_duration_sec',
            'front_jump_finish_worst_case_duration_sec',
            'stop_zone_approach_timeout_sec',
            'align_timeout_sec',
            'inspection_action_timeout_sec',
            'right_corner_forward_timeout_sec',
            'right_corner_target_yaw_deg',
            'right_corner_turn_timeout_sec',
            'transfer_arc_yaw_timeout_sec',
            'transfer_odom_timeout_sec',
            'transfer_arm_action_timeout_sec',
            'transfer_zero_epsilon',
            'transfer_final_cmd_tolerance',
        )
        nonnegative_names = (
            'corner_min_confidence',
            'corner_vx',
            'corner_angular_z',
            'red_circle_min_confidence',
            'red_turn_left_angular_z',
            'red_reverse_speed_mps',
            'red_reverse_duration_sec',
            'red_turn_right_angular_z',
            'white_bar_min_confidence',
            'white_bar_blind_forward_duration_sec',
            'stop_zone_min_confidence',
            'stop_zone_approach_speed',
            'align_min_confidence',
            'align_max_lateral_error',
            'align_max_heading_error',
            'align_max_angular_z',
            'align_heading_gain',
            'align_lateral_gain',
            'right_corner_forward_speed',
            'right_corner_forward_active_time_sec',
            'right_corner_turn_wz',
            'transfer_arc_vx',
            'transfer_arc_wz',
            'transfer_arc_task_stop_yaw_deg',
            'transfer_arc_exit_delta_yaw_deg',
            'transfer_yaw_wrong_direction_tolerance_deg',
            'reacquire_min_confidence',
            'transfer_reacquire_max_lateral_error',
        )
        for name in positive_names:
            setattr(self, name, self._float_parameter(name, positive=True))
        for name in nonnegative_names:
            setattr(self, name, self._float_parameter(name, positive=False))
        # 转向方向是省赛固定合同；两侧参数均为非负幅值，避免角度绝对值掩盖反号配置。
        if self.red_turn_left_angular_z <= 0.0:
            raise ValueError('red_turn_left_angular_z must be positive')
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

    def _transfer_config(self):
        """把 ROS/YAML 参数收敛为可离线测试的 TRANSFER 合同。"""
        return TransferRouteConfig(
            right_corner_confirm_frames=self.right_corner_confirm_frames,
            right_corner_motion_calibrated=(
                self.right_corner_motion_calibrated
            ),
            right_corner_forward_speed=self.right_corner_forward_speed,
            right_corner_forward_active_time_sec=(
                self.right_corner_forward_active_time_sec
            ),
            right_corner_forward_timeout_sec=(
                self.right_corner_forward_timeout_sec
            ),
            right_corner_turn_wz=self.right_corner_turn_wz,
            right_corner_target_yaw_deg=self.right_corner_target_yaw_deg,
            right_corner_turn_timeout_sec=self.right_corner_turn_timeout_sec,
            transfer_arc_entry_calibrated=(
                self.transfer_arc_entry_calibrated
            ),
            transfer_arc_entry_confirm_frames=(
                self.transfer_arc_entry_confirm_frames
            ),
            transfer_arc_motion_calibrated=(
                self.transfer_arc_motion_calibrated
            ),
            transfer_arc_vx=self.transfer_arc_vx,
            transfer_arc_wz=self.transfer_arc_wz,
            transfer_arc_task_stop_yaw_deg=(
                self.transfer_arc_task_stop_yaw_deg
            ),
            transfer_arc_exit_delta_yaw_deg=(
                self.transfer_arc_exit_delta_yaw_deg
            ),
            transfer_arc_yaw_timeout_sec=self.transfer_arc_yaw_timeout_sec,
            reacquire_frames=self.reacquire_stable_frames,
            reacquire_min_confidence=self.reacquire_min_confidence,
            reacquire_max_lateral_error=(
                self.transfer_reacquire_max_lateral_error
            ),
            zero_confirm_frames=self.transfer_zero_confirm_frames,
            zero_epsilon=self.transfer_zero_epsilon,
            final_cmd_tolerance=self.transfer_final_cmd_tolerance,
            yaw_wrong_direction_tolerance_deg=(
                self.transfer_yaw_wrong_direction_tolerance_deg
            ),
            arm_action_timeout_sec=self.transfer_arm_action_timeout_sec,
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
        line_valid = self._white_bar_line_valid(msg)
        self.white_bar_blind_core.observe_line(
            time.monotonic_ns(),
            valid=line_valid,
        )
        if self.route_core.transfer_detection_allowed():
            self.transfer_core.observe_line(
                visible=bool(getattr(msg, 'line_visible', False)),
                confidence=getattr(msg, 'confidence', 0.0),
                lateral_error=getattr(msg, 'lateral_error', float('nan')),
                now=self.latest_line_time,
            )
        if (
            self.white_bar_blind_core.lost_confirmed
            and not self.white_bar_blind_core.fault_reason
            and not line_valid
            and self.state in (
                'START_STAGE',
                'FINISH_STAGE',
                'WHITE_BAR_BLIND_FORWARD',
                'WHITE_BAR_BLIND_LINE_RECOVERY',
            )
        ):
            # 不等待下一个 10 Hz timer；invalid 新帧立即撤销正速候选。
            self._set_state(
                'WHITE_BAR_BLIND_LINE_RECOVERY',
                'WHITE_BAR_BLIND_LINE_RECOVERY',
            )
            self._publish_mission_candidate(Twist())

    def _on_red_circle(self, msg):
        """仅在正式红圆搜索阶段锁存第一次可靠检测。"""
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
            # ``T_RED_FIRST_SEEN`` 只在本次任务的 RED_TARGET_SEARCH 锁存一次；
            # 后续检测消失也不能重置省赛规定的 3.9 秒巡线窗口。
            if (
                self.state in ('RED_TARGET_SEARCH', 'INSPECTION_APPROACH')
                and self.red_first_seen_time is None
            ):
                self.red_first_seen_time = self.latest_red_time
        else:
            self.red_seen_count = 0

    def _on_odom(self, msg):
        """从真实 odometry 四元数提取 yaw；无效姿态绝不用于盲转。"""
        orientation = msg.pose.pose.orientation
        values = (
            _finite_float(orientation.x), _finite_float(orientation.y),
            _finite_float(orientation.z), _finite_float(orientation.w),
        )
        if any(value is None for value in values):
            self.latest_odom_yaw = None
            self.latest_odom_time = time.monotonic()
            return
        x, y, z, w = values
        norm = x * x + y * y + z * z + w * w
        if norm <= 1e-12:
            self.latest_odom_yaw = None
            self.latest_odom_time = time.monotonic()
            return
        self.latest_odom_yaw = math.atan2(
            2.0 * (w * z + x * y),
            1.0 - 2.0 * (y * y + z * z),
        )
        self.latest_odom_time = time.monotonic()

    def _on_final_cmd(self, msg):
        """缓存 mux 最终命令，后退计时和停车确认只信任此真相。"""
        self.latest_final_cmd = msg
        self.latest_final_cmd_time = time.monotonic()

    def _on_sdk_motion_status(self, msg):
        """接收 SDK 的真实 StopMove/CLASSIC_VERIFIED 回执，拒绝畸形数据。"""
        status = decode_json_object(msg.data)
        self.latest_sdk_motion_status = status
        self.latest_sdk_motion_status_time = time.monotonic()

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
        """只用 ARM 后的新帧推进共用 seen/lost streak。"""
        self.latest_white_bar = msg
        self.latest_white_bar_time = time.monotonic()
        stage = self.white_stage_controller.active_stage
        allowed = (
            self.white_stage_controller.state == f'{stage}_ARMED'
            and self.route_core.white_bar_detection_allowed(stage)
        )
        if not allowed:
            return
        self.white_bar_blind_core.observe_detection(
            time.monotonic_ns(),
            visible=bool(msg.visible),
            confidence=msg.confidence,
            center_y=msg.center_y,
        )
        # 兼容旧状态字段；正式决策只读取 shared core。
        self.white_seen_count = self.white_bar_blind_core.seen_count

    def _on_gait_control_lock(self, msg):
        """缓存权威 gait lock；blind 期间 stale/true 都禁止动作。"""
        self.latest_gait_lock = bool(msg.data)
        self.latest_gait_lock_time = time.monotonic()

    def _on_cmd_mux_status(self, msg):
        """缓存 mux 安全状态；畸形 JSON 保持不健康而非猜测。"""
        self.latest_mux_status = decode_json_object(msg.data)
        self.latest_mux_status_time = time.monotonic()

    def _on_corner_candidate(self, msg):
        """巡线阶段才接收角点，避免检查时转向。"""
        self.latest_corner = msg
        self.latest_corner_time = time.monotonic()
        if self.route_core.transfer_detection_allowed():
            self.transfer_core.observe_corner(
                visible=self._detection_visible_with_confidence(
                    msg, self.corner_min_confidence
                ),
                confidence=getattr(msg, 'confidence', 0.0),
                direction_hint=getattr(msg, 'direction_hint', 'unknown'),
                now=self.latest_corner_time,
            )
            if (
                self.transfer_core.transfer_right_corner_detected
                and not self.route_core.transfer_started
            ):
                route_event = self.route_core.transfer_started_event()
                if not route_event.accepted:
                    self._enter_emergency_stop(route_event.reason)
            self.corner_seen_count = 0
            return
        if (
            self.enable_corner_pre_turn
            and self._line_follower_is_ready(self.latest_corner_time)
            and self.route_core.corner_detection_allowed()
            and corner_cooldown_elapsed(
                self.latest_corner_time,
                self.last_corner_completed_time,
                self.corner_cooldown_sec,
            )
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
        self.white_bar_blind_core.reset()
        self.transfer_core.reset()
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
        self.white_bar_blind_core.reset()
        self.transfer_core.reset()
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
        if stage in ('START', 'FINISH'):
            self.white_bar_blind_core.arm(stage, time.monotonic_ns())
        else:
            self.white_bar_blind_core.reset()
        self.white_seen_count = 0
        # FINISH 命令只能在红圆右转并重新找线后生效；不能打断红圆链路。
        if stage == 'START':
            self._set_state('START_STAGE', route_event.reason)
        elif self.state not in (
            'RED_APPROACH', 'RED_TURN_LEFT', 'RED_REVERSE_APPROACH',
            'RED_REVERSE_STOP_CONFIRM', 'RED_INSPECTION_PREP',
            'RED_INSPECTION', 'RED_ACTION_EXECUTION',
            'RED_CLASSIC_RECOVERY', 'RED_TURN_RIGHT', 'ALIGN_TO_LINE',
        ):
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
            # executor 已完成既有“识别→映射→动作”闭环；下一步仍必须经
            # 独立 classic verification gate，不能立即右转。
            self.red_classic_status_sequence = self._sdk_status_sequence()
            self._set_state('RED_CLASSIC_RECOVERY', route_event.reason)
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
            self._set_state(
                'RED_ACTION_EXECUTION', 'inspection_action_progress'
            )

    def _on_control_timer(self):
        """零速优先的单循环，驱动候选速度和路线状态。"""
        now = time.monotonic()
        if self.state in (
            'WAIT_START', 'FINAL_STOP', 'EMERGENCY_STOP',
            'RED_REVERSE_NOT_CALIBRATED', 'RED_INSPECTION_TIMEOUT',
        ):
            self._publish_mission_candidate(Twist())
            return
        if not self.route_core.mission_started:
            self._set_state('WAIT_START', 'mission_not_started')
            self._publish_mission_candidate(Twist())
            return
        if self.route_core.transfer_detection_allowed():
            self._control_transfer(now)
            return
        if self.state == 'HANDLE_WHITE_BAR':
            self._control_white_bar_wait(now)
        elif self.state in (
            'WHITE_BAR_BLIND_FORWARD',
            'WHITE_BAR_BLIND_LINE_RECOVERY',
        ):
            self._control_armed_white_bar(now)
        elif self.state == 'RED_APPROACH':
            self._control_red_approach(now)
        elif self.state == 'RED_TURN_LEFT':
            self._control_red_turn(now, left=True)
        elif self.state == 'RED_REVERSE_APPROACH':
            self._control_red_reverse_approach(now)
        elif self.state == 'RED_REVERSE_STOP_CONFIRM':
            self._control_red_reverse_stop_confirm(now)
        elif self.state == 'RED_INSPECTION_PREP':
            self._control_red_inspection_prep(now)
        elif self.state in ('RED_INSPECTION', 'RED_ACTION_EXECUTION'):
            self._control_inspection_wait(now)
        elif self.state == 'RED_CLASSIC_RECOVERY':
            self._control_red_classic_recovery(now)
        elif self.state == 'RED_TURN_RIGHT':
            self._control_red_turn(now, left=False)
        elif self.state == 'CORNER_PRE_TURN':
            self._control_corner_turn(now)
        elif self.state == 'ALIGN_TO_LINE':
            self._control_align_to_line(now)
        elif self.state == 'APPROACH_STOP_ZONE':
            self._control_stop_zone_approach(now)
        elif self.state in (
            'START_STAGE',
            'MID_ROUTE',
            'INSPECTION_APPROACH',
            'RED_TARGET_SEARCH',
            'POST_INSPECTION',
            'FINISH_STAGE',
            'FINAL_ZONE_ARMED',
        ):
            self._control_route_follow(now)
        else:
            self._enter_emergency_stop(f'unhandled_node_state_{self.state}')

    def _control_transfer(self, now):
        """唯一 production mission owner 在 TRANSFER phase 独占固定动作。"""
        decision = self.transfer_core.tick(now, self._transfer_inputs(now))
        if decision.state == TRANSFER_SAFE_STOP:
            self._enter_emergency_stop(
                self.transfer_core.fault_reason or decision.reason
            )
            return
        self._set_state(decision.state, decision.reason)
        if decision.action:
            self._dispatch_transfer_action(decision.action, now)
        if decision.state == TRANSFER_COMPLETE:
            route_event = self.route_core.transfer_completed_event()
            if not route_event.accepted:
                self._enter_emergency_stop(route_event.reason)
                return
            self._set_state('INSPECTION_APPROACH', route_event.reason)
            self._publish_suggested_or_zero(now)
            return
        if decision.control == 'LINE_FOLLOW':
            if not self._line_follower_is_ready(now):
                self._publish_mission_candidate(Twist())
            else:
                self._publish_suggested_or_zero(now)
            return
        if decision.control == 'REACQUIRE':
            self._publish_transfer_reacquire_command(now)
            return
        cmd = Twist()
        if decision.control == 'FIXED':
            cmd.linear.x = decision.vx
            cmd.linear.y = 0.0
            cmd.angular.z = decision.wz
        self._publish_mission_candidate(cmd)

    def _transfer_inputs(self, now):
        """TRANSFER 只信任 mux 最终速度、权威锁和新鲜 odom。"""
        mux = self.latest_mux_status
        mux_fresh = self._is_fresh(
            self.latest_mux_status_time, now, self.suggested_cmd_timeout_sec
        )
        gait_fresh = self._is_fresh(
            self.latest_gait_lock_time, now, self.suggested_cmd_timeout_sec
        )
        return TransferInputs(
            final_cmd=(
                self.latest_final_cmd.linear.x,
                self.latest_final_cmd.linear.y,
                self.latest_final_cmd.angular.z,
            ),
            final_cmd_fresh=self._is_fresh(
                self.latest_final_cmd_time, now, self.suggested_cmd_timeout_sec
            ),
            mux_healthy=(
                mux_fresh and isinstance(mux, dict)
                and mux.get('active_source') == 'mission'
                and mux.get('estop') is False
                and mux.get('arm_lock') is False
                and mux.get('gait_lock') is False
                and mux.get('invalid_command_count', 0) == 0
            ),
            gait_available=(gait_fresh and not self.latest_gait_lock),
            odom_yaw=self.latest_odom_yaw,
            odom_fresh=self._is_fresh(
                self.latest_odom_time, now, self.transfer_odom_timeout_sec
            ),
        )

    def _publish_transfer_reacquire_command(self, now):
        """复用既有 ALIGN 有限角速度；线丢失时保持 ZERO。"""
        cmd = Twist()
        if self._is_fresh(self.latest_line_time, now):
            line = self.latest_line
            lateral = _finite_float(getattr(line, 'lateral_error', None))
            heading = _finite_float(getattr(line, 'heading_error', None))
            if (
                bool(getattr(line, 'line_visible', False))
                and lateral is not None and heading is not None
            ):
                cmd.angular.z = max(
                    -self.align_max_angular_z,
                    min(
                        self.align_max_angular_z,
                        -(
                            self.align_heading_gain * heading
                            + self.align_lateral_gain * lateral
                        ),
                    ),
                )
        self._publish_mission_candidate(cmd)

    def _dispatch_transfer_action(self, action, now):
        """发送一次既有机械臂 task；拒绝/失败立即安全停车。"""
        if not self.transfer_core.mark_action_dispatched(action, now):
            return
        self.transfer_action_name = action
        if not self.arm_action_client.wait_for_server(timeout_sec=0.0):
            self.transfer_core.action_result(
                action, False, time.monotonic(), 'action_server_unavailable'
            )
            return
        goal = ExecuteArmTask.Goal()
        goal.task_name = (
            self.transfer_place_task
            if action == 'transfer_place' else self.transfer_pick_task
        )
        goal.target = 'transfer_platform'
        future = self.arm_action_client.send_goal_async(goal)
        future.add_done_callback(
            lambda completed, expected=action: self._on_transfer_goal_response(
                completed, expected
            )
        )

    def _on_transfer_goal_response(self, future, action):
        """拒绝的机械臂 goal 不得被当成已完成里程碑。"""
        try:
            goal_handle = future.result()
        except Exception as error:
            self.transfer_core.action_result(
                action, False, time.monotonic(), str(error)
            )
            return
        if goal_handle is None or not goal_handle.accepted:
            self.transfer_core.action_result(
                action, False, time.monotonic(), 'goal_rejected'
            )
            return
        self.transfer_action_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda completed, expected=action: self._on_transfer_action_result(
                completed, expected
            )
        )

    def _on_transfer_action_result(self, future, action):
        """仅当 ExecuteArmTask 显式 success 时推进 place/pick 顺序。"""
        try:
            result = future.result().result
            success = bool(result.success)
            message = str(result.message)
        except Exception as error:
            success = False
            message = str(error)
        self.transfer_action_goal_handle = None
        self.transfer_core.action_result(
            action, success, time.monotonic(), message
        )

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
            if self.red_first_seen_time is not None:
                route_event = self.route_core.red_circle_confirmed()
                if route_event.accepted:
                    first_seen = self.red_first_seen_time
                    self._reset_detection_counts()
                    self.red_first_seen_time = first_seen
                    self._set_state('RED_APPROACH', 'T_RED_FIRST_SEEN')
                    # RED_APPROACH 的首周期也必须走 follower；不额外盲走。
                    self._control_red_approach(now)
                return
            if self._corner_confirmed(now):
                self._start_corner_turn()
                return
            self._publish_suggested_or_zero(now)
            return
        if phase == 'FINISH_STAGE':
            if (
                not self.white_bar_blind_core.seen_latched
                and self._corner_confirmed(now)
            ):
                self._start_corner_turn()
                return
            self._control_armed_white_bar(now)
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
        """共用 seen→lost→blind 状态；center_y 与旧低速参数不参与控制。"""
        stage = self.white_stage_controller.active_stage
        if not self.route_core.white_bar_detection_allowed(stage):
            self._publish_mission_candidate(Twist())
            return
        safety = self._white_bar_safety_inputs(now)
        decision = self.white_bar_blind_core.evaluate(
            time.monotonic_ns(), **safety,
        )
        self.white_seen_count = self.white_bar_blind_core.seen_count
        if decision.action == FOLLOW:
            cmd = Twist()
            cmd.linear.x = decision.linear_x
            cmd.linear.y = 0.0
            cmd.angular.z = decision.angular_z
            if decision.reason == 'WHITE_BAR_BLIND_FORWARD':
                self._set_state(
                    'WHITE_BAR_BLIND_FORWARD', decision.reason
                )
            self._publish_mission_candidate(cmd)
            return
        if (
            decision.action == ZERO
            and decision.reason == 'WHITE_BAR_BLIND_LINE_RECOVERY'
        ):
            # 短时丢线期间 mission 必须保持零候选，不进入不可恢复故障。
            self._set_state(
                'WHITE_BAR_BLIND_LINE_RECOVERY', decision.reason
            )
            self._publish_mission_candidate(Twist())
            return
        if decision.action != REQUEST_ACTION:
            self._enter_emergency_stop(decision.reason)
            return

        # 先撤销 mission candidate；真正动作仍由既有 executor 的最终零速
        # 确认门放行，任务层不会直接调用任何 gait/SDK 动作。
        self._publish_mission_candidate(Twist())
        stage_event = self.white_stage_controller.white_bar_event(True)
        if stage_event.action == 'SEND_REQUEST':
            if stage_event.motion_name != decision.motion_name:
                self._enter_emergency_stop(
                    'white_bar_blind_motion_mapping_mismatch'
                )
                return
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

    def _white_bar_safety_inputs(self, now):
        """汇总 blind 仍必须满足的线、候选、gait 与 mux 安全门。"""
        line = self.latest_line
        line_fresh = self._is_fresh(self.latest_line_time, now)
        line_valid = self._white_bar_line_valid(line)
        suggested_fresh = self._is_fresh(
            self.latest_suggested_time,
            now,
            self.suggested_cmd_timeout_sec,
        )
        suggested = self.latest_suggested_cmd
        gait_fresh = self._is_fresh(
            self.latest_gait_lock_time,
            now,
            self.suggested_cmd_timeout_sec,
        )
        mux_fresh = self._is_fresh(
            self.latest_mux_status_time,
            now,
            self.suggested_cmd_timeout_sec,
        )
        mux = self.latest_mux_status
        mux_healthy = (
            isinstance(mux, dict)
            and mux.get('estop') is False
            and mux.get('arm_lock') is False
            and mux.get('gait_lock') is False
            and mux.get('invalid_command_count', 0) == 0
        )
        return {
            'line_valid': line_valid,
            'line_fresh': line_fresh,
            'suggested_fresh': suggested_fresh,
            'suggested_vx': suggested.linear.x,
            'suggested_wz': suggested.angular.z,
            'follower_ready': self.line_follower_ready,
            'follower_fresh': self._is_fresh(
                self.latest_line_follower_status_time,
                now,
                self.line_follower_status_timeout_sec,
            ),
            'gait_locked': self.latest_gait_lock,
            'gait_fresh': gait_fresh,
            'mux_healthy': mux_healthy,
            'mux_fresh': mux_fresh,
            'mux_source_valid': (
                isinstance(mux, dict)
                and mux.get('active_source') == 'mission'
            ),
        }

    @staticmethod
    def _white_bar_line_valid(line):
        """blind 恢复与原有白线安全门共用同一 LineTrack 有效定义。"""
        return (
            line is not None
            and bool(getattr(line, 'line_visible', False))
            and _finite_float(getattr(line, 'confidence', None)) is not None
            and _finite_float(getattr(line, 'lateral_error', None)) is not None
            and _finite_float(getattr(line, 'heading_error', None)) is not None
        )

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
        """按省赛合同持续 3.9 秒正式巡线，完成后先发布 ZERO。"""
        safety_reason = self._red_follower_safety_reason(now)
        if safety_reason:
            self._enter_emergency_stop(safety_reason)
            return
        if now - self.state_enter_time >= self.red_approach_duration_sec:
            self._publish_mission_candidate(Twist())
            self._start_red_turn(left=True, reason='red_approach_complete')
            return
        # 完整复制 LineTrack → follower → mission_cmd 的候选，不覆盖 vx/yaw。
        self._publish_suggested_or_zero(now)

    def _control_inspection_wait(self, now):
        """检查识别与 SDK 动作期间候选恒为零，超时急停。"""
        self._publish_mission_candidate(Twist())
        if (
            self.inspection_request_started_time is not None
            and now - self.inspection_request_started_time
            >= self.inspection_action_timeout_sec
        ):
            self.route_core.fault('inspection_action_timeout')
            self._set_state(
                'RED_INSPECTION_TIMEOUT', 'inspection_action_timeout'
            )
            self._publish_mission_candidate(Twist())

    def _start_red_turn(self, *, left, reason):
        """以新鲜真实 yaw 建立转角原点；缺 yaw 时禁止用时间替代。"""
        now = time.monotonic()
        if not self._is_fresh(self.latest_odom_time, now):
            self._enter_emergency_stop('red_turn_odom_stale')
            return
        if self.latest_odom_yaw is None:
            self._enter_emergency_stop('red_turn_odom_invalid')
            return
        self.red_turn_start_yaw = self.latest_odom_yaw
        self._set_state('RED_TURN_LEFT' if left else 'RED_TURN_RIGHT', reason)

    def _control_red_turn(self, now, *, left):
        """83°左转和80°右转都以归一化真实 yaw 收敛，绝不时间盲转。"""
        if now - self.state_enter_time >= self.red_yaw_timeout_sec:
            self._enter_emergency_stop('red_turn_yaw_timeout')
            return
        if not self._is_fresh(self.latest_odom_time, now):
            self._enter_emergency_stop('red_turn_odom_stale')
            return
        if self.red_turn_start_yaw is None or self.latest_odom_yaw is None:
            self._enter_emergency_stop('red_turn_odom_invalid')
            return
        safety_reason = self._red_motion_safety_reason(now)
        if safety_reason:
            self._enter_emergency_stop(safety_reason)
            return
        target_deg = (
            self.red_turn_left_angle_deg if left
            else self.red_turn_right_angle_deg
        )
        yaw_delta = self._normalize_angle(
            self.latest_odom_yaw - self.red_turn_start_yaw
        )
        if abs(yaw_delta) >= math.radians(target_deg):
            self._publish_mission_candidate(Twist())
            if left:
                if (
                    self.red_reverse_duration_sec <= 0.0
                    or self.red_reverse_speed_mps <= 0.0
                ):
                    self.route_core.fault('RED_REVERSE_NOT_CALIBRATED')
                    self._set_state(
                        'RED_REVERSE_NOT_CALIBRATED',
                        'RED_REVERSE_NOT_CALIBRATED',
                    )
                    return
                self.red_reverse_started_time = time.monotonic()
                self.red_reverse_active_elapsed = 0.0
                self.red_reverse_last_tick = now
                self._set_state(
                    'RED_REVERSE_APPROACH', 'red_turn_left_complete'
                )
                return
            # 红转后须先安全找线；完成后路线核心进入 POST_INSPECTION 才允许 arm FINISH。
            self._begin_align('red', 'red_turn_right_complete')
            return
        cmd = Twist()
        cmd.angular.z = (
            self.red_turn_left_angular_z if left
            else -abs(self.red_turn_right_angular_z)
        )
        self._publish_mission_candidate(cmd)

    def _control_red_reverse_approach(self, now):
        """按实际最终倒车命令累计有效时间；停车/失锁期间冻结计时。"""
        if self.red_reverse_duration_sec <= 0.0:
            self.route_core.fault('RED_REVERSE_NOT_CALIBRATED')
            self._set_state(
                'RED_REVERSE_NOT_CALIBRATED', 'RED_REVERSE_NOT_CALIBRATED'
            )
            self._publish_mission_candidate(Twist())
            return
        if self.red_reverse_last_tick is None:
            self.red_reverse_last_tick = now
        elapsed_delta = max(0.0, now - self.red_reverse_last_tick)
        self.red_reverse_last_tick = now
        # 先提出普通经典步态 Move 候选；只有 mux 最终输出已证实倒车时才计时。
        cmd = Twist()
        cmd.linear.x = -self.red_reverse_speed_mps
        self._publish_mission_candidate(cmd)
        if self._reverse_command_active(now):
            self.red_reverse_active_elapsed += elapsed_delta
        if self.red_reverse_active_elapsed >= self.red_reverse_duration_sec:
            self._publish_mission_candidate(Twist())
            self.red_stop_status_sequence = self._sdk_status_sequence()
            self._set_state(
                'RED_REVERSE_STOP_CONFIRM', 'red_reverse_active_duration_met'
            )

    def _control_red_reverse_stop_confirm(self, now):
        """倒车结束必须同时看到最终零速和成功 StopMove，才可进入识别准备。"""
        self._publish_mission_candidate(Twist())
        if now - self.state_enter_time >= self.red_stop_confirm_timeout_sec:
            self._enter_emergency_stop('red_reverse_stop_confirm_timeout')
            return
        if not self._final_cmd_is_zero(now):
            return
        status = self.latest_sdk_motion_status
        if (
            isinstance(status, dict)
            and self._sdk_status_sequence() > self.red_stop_status_sequence
            and status.get('event') == 'STOP_MOVE'
            and status.get('ret') == 0
        ):
            self._start_red_inspection_prep(now)

    def _start_red_inspection_prep(self, now):
        """零速确认后才授权既有识别→动作执行器，不改写其映射。"""
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
        self._set_state('RED_INSPECTION_PREP', route_event.reason)
        self._publish_mission_candidate(Twist())

    def _control_red_inspection_prep(self, now):
        """识别前维持最终零速并等待执行器持有 gait lock。"""
        self._publish_mission_candidate(Twist())
        if (
            self._final_cmd_is_zero(now)
            and self._is_fresh(self.latest_gait_lock_time, now)
            and self.latest_gait_lock
        ):
            self._set_state(
                'RED_INSPECTION', 'red_inspection_stationary_locked'
            )
            return
        if (
            self.inspection_request_started_time is not None
            and now - self.inspection_request_started_time
            >= self.inspection_action_timeout_sec
        ):
            self.route_core.fault('red_inspection_prep_timeout')
            self._set_state(
                'RED_INSPECTION_TIMEOUT', 'red_inspection_prep_timeout'
            )

    def _control_red_classic_recovery(self, now):
        """只接受动作成功之后的新 CLASSIC_VERIFIED，禁止再次调用 2049。"""
        self._publish_mission_candidate(Twist())
        if (
            now - self.state_enter_time
            >= self.red_classic_recovery_timeout_sec
        ):
            self._enter_emergency_stop('red_classic_recovery_timeout')
            return
        status = self.latest_sdk_motion_status
        if not isinstance(status, dict):
            return
        if self._sdk_status_sequence() <= self.red_classic_status_sequence:
            return
        if status.get('event') == 'SDK_ERROR' or status.get('ret') not in (0,):
            self._enter_emergency_stop('red_classic_recovery_failed')
            return
        if status.get('event') == 'CLASSIC_VERIFIED':
            self._start_red_turn(left=False, reason='CLASSIC_VERIFIED')

    def _red_follower_safety_reason(self, now):
        """红圈 3.9 秒仍是巡线，线、follower、步态和 mux 任一失效即停。"""
        if not self._line_follower_is_ready(now):
            return 'red_approach_line_follower_not_ready'
        if (
            not self._is_fresh(self.latest_line_time, now)
            or not self._white_bar_line_valid(self.latest_line)
        ):
            return 'red_approach_line_track_invalid'
        if not self._is_fresh(
            self.latest_suggested_time, now, self.suggested_cmd_timeout_sec
        ):
            return 'red_approach_follower_command_stale'
        return self._red_motion_safety_reason(now)

    def _red_motion_safety_reason(self, now):
        """正式红圆运动共享 gait/mux fail-closed 门，避免绕过最终仲裁。"""
        if not self._is_fresh(
            self.latest_gait_lock_time, now, self.suggested_cmd_timeout_sec
        ) or self.latest_gait_lock:
            return 'red_motion_gait_not_available'
        mux = self.latest_mux_status
        if not self._is_fresh(
            self.latest_mux_status_time, now, self.suggested_cmd_timeout_sec
        ) or not isinstance(mux, dict):
            return 'red_motion_mux_status_stale'
        if (
            mux.get('estop') is not False
            or mux.get('arm_lock') is not False
            or mux.get('gait_lock') is not False
            or mux.get('invalid_command_count', 0) != 0
        ):
            return 'red_motion_mux_safety_fault'
        return ''

    def _reverse_command_active(self, now):
        """只有普通 Move 实际输出为预期后退且安全允许时，累计后退有效时间。"""
        if self._red_motion_safety_reason(now):
            return False
        if not self._is_fresh(
            self.latest_final_cmd_time, now, self.suggested_cmd_timeout_sec
        ):
            return False
        final = self.latest_final_cmd
        values = (
            _finite_float(final.linear.x), _finite_float(final.linear.y),
            _finite_float(final.angular.z),
        )
        return (
            all(value is not None for value in values)
            and values[0] < -1e-3
            and abs(values[1]) <= 1e-3
            and abs(values[2]) <= 1e-3
        )

    def _final_cmd_is_zero(self, now):
        """停车确认检查最终 mux 输出的全部平面控制量与新鲜度。"""
        if not self._is_fresh(
            self.latest_final_cmd_time, now, self.suggested_cmd_timeout_sec
        ):
            return False
        final = self.latest_final_cmd
        values = (
            final.linear.x, final.linear.y, final.linear.z,
            final.angular.x, final.angular.y, final.angular.z,
        )
        return all(
            value is not None and abs(value) <= 1e-3
            for value in (_finite_float(value) for value in values)
        )

    def _sdk_status_sequence(self):
        """只接受严格正整数状态序号，旧回执不能满足新的停车/步态门。"""
        status = self.latest_sdk_motion_status
        sequence = status.get('sequence') if isinstance(status, dict) else 0
        return sequence if type(sequence) is int and sequence > 0 else 0

    @staticmethod
    def _normalize_angle(angle):
        """将 yaw 差归一化到 [-pi, pi]，跨 ±pi 时仍能正确判断转角。"""
        return math.atan2(math.sin(angle), math.cos(angle))

    def _start_corner_turn(self):
        """确认角点后独立转向，居中后才恢复巡线。"""
        direction = self._resolved_corner_direction(self.latest_corner)
        if direction is None:
            # direction_hint 未经确认时不能默认左转，避免把右弯变成撞内侧。
            self.last_reason = 'corner_direction_unknown'
            self._publish_mission_candidate(Twist())
            return
        route_event = self.route_core.corner_confirmed()
        if not route_event.accepted:
            return
        self.corner_seen_count = 0
        self.active_corner_direction = direction
        self.active_corner_direction_source = (
            'direction_hint' if self.corner_turn_direction == 'hint'
            else 'configured_fixed_direction'
        )
        self.active_corner_detected_time = self.latest_corner_time
        self.active_corner_confidence = float(
            getattr(self.latest_corner, 'confidence', 0.0)
        )
        self._set_state('CORNER_PRE_TURN', route_event.reason)
        self._publish_mission_candidate(Twist())

    def _control_corner_turn(self, now):
        """角点局部转向有时长边界，防止无限转圈。"""
        # 进入角点前已通过 START_READY；之后 follower 会因丢线进入
        # CORNER_OWNER_WAIT。此时再要求 LINE_FOLLOW 会错误地让 mission 停转。
        if self.active_corner_direction not in ('left', 'right'):
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
        cmd.angular.z = self._turn_sign(self.active_corner_direction) * (
            self.corner_angular_z
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
            next_state = route_event.route_phase
            if route_event.route_phase == 'TRANSFER_ROUTE':
                next_state = RIGHT_CORNER_SEARCH
            elif route_event.route_phase == 'MID_ROUTE':
                # 红圆算法只在正式中段搜索状态接入；路线 phase 保持兼容名称。
                next_state = 'RED_TARGET_SEARCH'
                self.red_first_seen_time = None
            if self.align_context == 'corner':
                self.last_corner_completed_time = now
            self._set_state(next_state, route_event.reason)
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

    def _corner_confirmed(self, now):
        """角点确认遵守 phase gate 和本地配置开关。"""
        return (
            self.enable_corner_pre_turn
            and self._is_fresh(self.latest_corner_time, now)
            and self.route_core.corner_detection_allowed()
            and self.corner_seen_count >= self.corner_confirm_frames
            and self._resolved_corner_direction(self.latest_corner) is not None
            and corner_cooldown_elapsed(
                now, self.last_corner_completed_time,
                self.corner_cooldown_sec,
            )
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
        payload = {
            'state': self.state,
            'route_phase': route.route_phase,
            'mission_started': bool(route.mission_started),
            'run_id': route.run_id,
            'start_jump_completed': bool(route.start_jump_completed),
            'inspection_completed': bool(route.inspection_completed),
            'finish_jump_completed': bool(route.finish_jump_completed),
            'transfer_started': bool(route.transfer_started),
            'transfer_completed': bool(route.transfer_completed),
            'final_zone_armed': bool(route.final_zone_armed),
            'active_request_id': route.active_request_id,
            'fault_reason': route.fault_reason,
            'reason': self.last_reason,
            'active_action': self.active_action,
            'white_bar_stage_state': self.white_stage_controller.state,
            'white_bar_stage_run_id': self.white_stage_controller.run_id,
            'white_bar_action_request_sent': bool(
                self.white_stage_controller.request_sent
            ),
            # 兼容旧字段；新消费者应读取 seen/lost 明确字段。
            'white_bar_confirm_count': self.white_seen_count,
            'red_confirm_count': self.red_seen_count,
            'stop_zone_confirm_count': self.stop_seen_count,
            'stop_zone_inside_count': self.stop_inside_count,
            'corner_confirm_count': self.corner_seen_count,
            'corner_owner': self.corner_owner,
            'corner_direction_config': self.corner_turn_direction,
            'corner_direction_active': self.active_corner_direction,
            'corner_direction_source': self.active_corner_direction_source,
            'corner_first_detected_time': self.active_corner_detected_time,
            'corner_active_confidence': self.active_corner_confidence,
            'corner_candidate_direction_hint': str(getattr(
                self.latest_corner, 'direction_hint', ''
            )),
            'corner_candidate_confidence': float(getattr(
                self.latest_corner, 'confidence', 0.0
            )),
            'line_follower_ready': bool(self.line_follower_ready),
            'line_follower_status_reason': self.latest_line_status,
            'align_context': self.align_context,
            'align_confirm_count': self.align_seen_count,
            'T_RED_FIRST_SEEN': self.red_first_seen_time,
            'red_reverse_configured_duration': self.red_reverse_duration_sec,
            'red_reverse_active_elapsed': self.red_reverse_active_elapsed,
            'red_reverse_wall_elapsed': (
                0.0 if self.red_reverse_started_time is None else max(
                    0.0, time.monotonic() - self.red_reverse_started_time
                )
            ),
            'red_reverse_remaining': max(
                0.0,
                self.red_reverse_duration_sec
                - self.red_reverse_active_elapsed,
            ),
            # 简短字段保留给现场标定脚本，和需求中的术语一一对应。
            'configured_duration': self.red_reverse_duration_sec,
            'active_elapsed': self.red_reverse_active_elapsed,
            'wall_elapsed': (
                0.0 if self.red_reverse_started_time is None else max(
                    0.0, time.monotonic() - self.red_reverse_started_time
                )
            ),
            'remaining': max(
                0.0,
                self.red_reverse_duration_sec
                - self.red_reverse_active_elapsed,
            ),
            'final_vx': float(cmd.linear.x),
            'final_wz': float(cmd.angular.z),
            # final_* 为历史兼容字段且实际是 mission candidate；验收 final
            # 必须读取下列 mux 字段，避免把上游候选误当 SDK 最终命令。
            'mission_candidate_vx': float(cmd.linear.x),
            'mission_candidate_vy': float(cmd.linear.y),
            'mission_candidate_yaw': float(cmd.angular.z),
            'mux_final_cmd_fresh': self._is_fresh(
                self.latest_final_cmd_time,
                time.monotonic(),
                self.suggested_cmd_timeout_sec,
            ),
            'mux_final_vx': float(self.latest_final_cmd.linear.x),
            'mux_final_vy': float(self.latest_final_cmd.linear.y),
            'mux_final_yaw': float(self.latest_final_cmd.angular.z),
        }
        payload.update(
            self.white_bar_blind_core.snapshot(time.monotonic_ns())
        )
        payload.update(self.transfer_core.snapshot())
        msg.data = json.dumps(payload, separators=(',', ':'))
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
        self.red_first_seen_time = None
        self.stop_seen_count = 0
        self.stop_inside_count = 0
        self.white_seen_count = 0
        self.corner_seen_count = 0
        self.active_corner_direction = ''
        self.active_corner_direction_source = ''
        self.active_corner_detected_time = None
        self.active_corner_confidence = 0.0

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
        self.red_turn_start_yaw = None
        self.red_reverse_started_time = None
        self.red_reverse_active_elapsed = 0.0
        self.red_reverse_last_tick = None

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

    def _resolved_corner_direction(self, detection):
        """返回本弯的显式方向；hint 缺失或非法时故障关闭而非默认左转。"""
        direction = self.corner_turn_direction
        if direction == 'hint':
            direction = str(getattr(
                detection, 'direction_hint', ''
            )).strip().lower()
        return direction if direction in ('left', 'right') else None

    @staticmethod
    def _turn_sign(direction):
        """将已验证的左右方向映射到 ROS yaw 符号。"""
        return -1.0 if direction == 'right' else 1.0


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
