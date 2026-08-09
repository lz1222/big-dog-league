#!/usr/bin/env python3
"""任务平台定位 ROS 适配层。

该节点默认不向 cmd mux 发布任何非零速度。路线层须显式在状态 JSON 的
``platform_route_phase`` 写入平台阶段，才会把视觉输入转交给纯状态机。
这样未部署或未标定时，经典巡线控制完全不受影响。
"""

import json
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, String

from rk_interfaces.msg import LineTrack, SpecialTargetDetection

from .platform_positioning_core import (
    BoardObservation, DEFAULT_PLATFORM_PARAMETERS, PlatformCommand,
    TaskPlatformPositioningCore, WhiteBarObservation,
)


class TaskPlatformPositioningNode(Node):
    """连接既有角点、白横线、LineTrack 和前置挡板输入的只读定位框架。"""

    def __init__(self):
        super().__init__('task_platform_positioning_node')
        for name, value in DEFAULT_PLATFORM_PARAMETERS.items():
            self.declare_parameter(name, value)
        topics = {
            'route_state_topic': '/mission/line_course_state',
            'corner_candidate_topic': '/perception/corner_candidate',
            'line_track_topic': '/perception/line_track',
            'white_bar_topic': '/perception/white_bar_detection',
            'pickup_board_topic': '/perception/pickup_board_anchor',
            'final_cmd_topic': '/navigation/cmd_vel',
            'odom_topic': '/utlidar/robot_odom',
            'recognition_result_topic': '/mission/pickup_recognition_result',
            'platform_event_topic': '/mission/task_platform_event',
            'status_topic': '/mission/task_platform_positioning_status',
            'gait_lock_request_topic': (
                '/gait/control_lock_req/platform_positioning'),
            # 独立候选 topic 不接 mux；防止本轮软件框架抢占基础巡线。
            'candidate_topic': '/control/task_platform_positioning_candidate',
        }
        for name, value in topics.items():
            self.declare_parameter(name, value)
        self.declare_parameter('control_rate_hz', 10.0)
        params = {name: self.get_parameter(
            name).value for name in DEFAULT_PLATFORM_PARAMETERS}
        self.core = TaskPlatformPositioningCore(params)
        self._latest_line = LineTrack()
        self.status_pub = self.create_publisher(
            String, self._topic('status_topic'), 10)
        self.command_pub = self.create_publisher(
            Twist, self._topic('candidate_topic'), 10)
        self.gait_lock_pub = self.create_publisher(
            Bool, self._topic('gait_lock_request_topic'), 10)
        self.create_subscription(String, self._topic(
            'route_state_topic'), self._on_route_state, 10)
        self.create_subscription(
            SpecialTargetDetection,
            self._topic('corner_candidate_topic'),
            self._on_corner,
            10)
        self.create_subscription(
            LineTrack,
            self._topic('line_track_topic'),
            self._on_line,
            10)
        self.create_subscription(
            SpecialTargetDetection,
            self._topic('white_bar_topic'),
            self._on_white,
            10)
        self.create_subscription(
            SpecialTargetDetection,
            self._topic('pickup_board_topic'),
            self._on_board,
            10)
        self.create_subscription(Twist, self._topic(
            'final_cmd_topic'), self._on_final_cmd, 10)
        self.create_subscription(
            Odometry,
            self._topic('odom_topic'),
            self._on_odom,
            10)
        self.create_subscription(
            String,
            self._topic('recognition_result_topic'),
            self._on_recognition,
            10)
        self.create_subscription(
            String,
            self._topic('platform_event_topic'),
            self._on_platform_event,
            10)
        rate_hz = float(self.get_parameter('control_rate_hz').value)
        self.create_timer(1.0 / max(1.0, rate_hz), self._on_timer)

    def _topic(self, name):
        return str(self.get_parameter(name).value)

    def _on_route_state(self, message):
        """仅认可显式平台阶段字段，既有 route state 不会被猜测映射。"""
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        if isinstance(
                payload,
                dict) and isinstance(
                payload.get('platform_route_phase'),
                str):
            self.core.set_route_phase(payload['platform_route_phase'])
            if payload.get('finish_rearmed') is True:
                self.core.rearm_finish()

    def _on_line(self, message):
        self._latest_line = message

    def _on_corner(self, message):
        self.core.observe_transfer_anchor(
            detected=bool(message.visible), confidence=message.confidence,
            heading_error=self._latest_line.heading_error,
            lateral_error=self._latest_line.lateral_error,
        )

    def _on_white(self, message):
        # structural detector 已将 center_y/width/height 归一化；适配层只
        # 赋予 PLACE 纵向语义，不创建第二套白线视觉算法。
        self.core.observe_place_white_bar(
            WhiteBarObservation(bool(message.visible), message.confidence,
                                message.center_y, message.width_ratio,
                                message.height_ratio),
            line_lateral_error=self._latest_line.lateral_error,
            line_heading_error=self._latest_line.heading_error,
        )

    def _on_board(self, message):
        self.core.observe_pickup_board(BoardObservation(
            bool(message.visible), message.confidence, message.center_x,
            message.center_y,
            message.center_y + message.height_ratio / 2.0,
            message.width_ratio, message.height_ratio, message.area_ratio))

    def _on_final_cmd(self, message):
        self.core.observe_final_command(PlatformCommand(
            message.linear.x, message.linear.y, message.angular.z))

    def _on_odom(self, message):
        q = message.pose.pose.orientation
        numerator = 2.0 * (q.w * q.z + q.x * q.y)
        denominator = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.core.observe_odom_yaw(math.atan2(numerator, denominator))

    def _on_recognition(self, message):
        try:
            payload = json.loads(message.data)
            if isinstance(
                    payload,
                    dict) and isinstance(
                    payload.get('success'),
                    bool):
                self.core.recognition_result(payload['success'])
        except (TypeError, ValueError, json.JSONDecodeError):
            return

    def _on_platform_event(self, message):
        """接收已有任务执行器的显式结果，不依据传感器猜测任务完成。"""
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        if not isinstance(payload, dict):
            return
        event = payload.get('event')
        if event == 'TRANSFER_TASK_DONE':
            self.core.transfer_task_done()
        elif event == 'PLACE_DONE':
            self.core.place_done()
        elif event == 'REARM_FINISH':
            self.core.rearm_finish()

    def _on_timer(self):
        command = self.core.tick(time.monotonic())
        output = Twist()
        output.linear.x = command.vx
        output.linear.y = command.vy
        output.angular.z = command.wz
        self.command_pub.publish(output)
        lock = Bool()
        lock.data = self.core.gait_lock_requested()
        self.gait_lock_pub.publish(lock)
        status = String()
        status.data = json.dumps(self.core.snapshot(), sort_keys=True)
        self.status_pub.publish(status)


def main(args=None):
    """ROS 入口。"""
    rclpy.init(args=args)
    node = TaskPlatformPositioningNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
