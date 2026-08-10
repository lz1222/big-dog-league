#!/usr/bin/env python3
"""独立验证：巡黑线直到 T 型楼梯入口，再安全交接到 Phase 3。

本节点只用于 ``stair_line_to_t.launch.py``，所有话题均放在 ``/stairs``
命名空间，避免与迷宫或原巡线任务争夺命令。默认两个运动开关均为 false：
只验证图像、巡线链路和 T 型识别，不让机器狗移动。
"""

import os
import subprocess
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String

from rk_bringup.stair_line_handoff_core import detect_t_stair_marker


# ===== 独立楼梯验证参数总表：所有速度、时间和转弯值集中在此处 =====
DEFAULT_LINE_SPEED_MPS = 0.25             # T 标记前持续巡线前进速度（m/s）
DEFAULT_T_CONFIRM_FRAMES = 5              # 连续识别帧数，防止单帧误触发
DEFAULT_ZERO_CONFIRM_MESSAGES = 3         # 交接前连续最终零速度消息数
DEFAULT_ZERO_CONFIRM_TIMEOUT_SEC = 2.0    # 未确认停车时的失败关闭上限（s）
DEFAULT_STAIRS_UP_SPEED_MPS = 0.55        # T 后上台阶前进速度（m/s）
DEFAULT_STAIRS_UP_DURATION_SEC = 3.9      # T 后上台阶动作时间（s）
DEFAULT_TURN_LEFT_ANGLE_DEG = 79.0        # 左转目标角度（度，实时 yaw 闭环）
DEFAULT_TURN_LEFT_WZ_RADPS = 1.0          # 左转角速度（rad/s，正值为左转）
DEFAULT_STAIRS_DOWN_SPEED_MPS = 0.55      # 下台阶前进速度（m/s）
DEFAULT_STAIRS_DOWN_DURATION_SEC = 2.9    # 下台阶动作时间（s）
DEFAULT_INTER_STAGE_STOP_SEC = 0.4        # Phase 3 各阶段停车稳定时间（s）


class StairLineHandoffNode(Node):
    """负责独立巡线流程的启动、T 门禁、安全停车与楼梯动作交接。"""

    WAIT_IMAGE = 'WAIT_IMAGE'
    LINE_FOLLOW = 'LINE_FOLLOW'
    STOPPING_FOR_T = 'STOPPING_FOR_T'
    RUNNING_PHASE3 = 'RUNNING_PHASE3'
    COMPLETE = 'COMPLETE'
    FAILED = 'FAILED'

    def __init__(self):
        super().__init__('stair_line_handoff_node')
        self._declare_parameters()
        self._read_parameters()
        self.state = self.WAIT_IMAGE
        self.marker_count = 0
        self.valid_frame_count = 0
        self.zero_command_count = 0
        self.stop_requested_at = None
        self.phase3_process = None

        self.start_publisher = self.create_publisher(
            Bool, self.line_start_topic, 10)
        self.stop_publisher = self.create_publisher(
            Bool, self.line_stop_topic, 10)
        self.lock_publisher = self.create_publisher(
            Bool, self.gait_lock_topic, 10)
        self.status_publisher = self.create_publisher(
            String, self.status_topic, 10)
        self.create_subscription(
            Image, self.image_topic, self._on_image, qos_profile_sensor_data)
        self.create_subscription(
            Twist, self.final_cmd_topic, self._on_final_command, 10)
        self.timer = self.create_timer(0.10, self._on_timer)

        # 先解锁并保持巡线未启动；防止上一次独立验证残留的 lock 误阻塞新会话。
        self._publish_bool(self.lock_publisher, False)
        self._publish_bool(self.stop_publisher, False)
        self._publish_status('WAIT_IMAGE', '等待有效相机画面；默认不执行运动')
        self.get_logger().info(
            '独立楼梯验证已启动：line_motion={} phase3_motion={} '
            'line_speed={:.3f}m/s t_confirm_frames={}'.format(
                self.execute_line_motion, self.execute_phase3,
                self.line_speed_mps, self.t_confirm_frames))

    def _declare_parameters(self):
        """声明独立话题与运动参数，默认值均可从 launch 命令行覆盖。"""
        self.declare_parameter('image_topic', '/line_camera/image_raw')
        self.declare_parameter('final_cmd_topic', '/stairs/navigation/cmd_vel')
        self.declare_parameter('line_start_topic', '/stairs/line_start')
        self.declare_parameter('line_stop_topic', '/stairs/line_stop')
        self.declare_parameter('gait_lock_topic', '/stairs/line_lock')
        self.declare_parameter('status_topic', '/stairs/handoff_status')
        self.declare_parameter('execute_line_motion', False)
        self.declare_parameter('execute_phase3', False)
        self.declare_parameter('line_speed_mps', DEFAULT_LINE_SPEED_MPS)
        self.declare_parameter('t_confirm_frames', DEFAULT_T_CONFIRM_FRAMES)
        self.declare_parameter('dark_threshold', 70)
        self.declare_parameter('band_min_width_ratio', 0.65)
        self.declare_parameter('lane_min_width_ratio', 0.04)
        self.declare_parameter('lane_max_width_ratio', 0.28)
        self.declare_parameter('zero_confirm_messages', DEFAULT_ZERO_CONFIRM_MESSAGES)
        self.declare_parameter('zero_confirm_timeout_sec', DEFAULT_ZERO_CONFIRM_TIMEOUT_SEC)
        self.declare_parameter('zero_epsilon', 0.01)
        self.declare_parameter('phase3_helper', '')
        self.declare_parameter('sdk_interface', 'eth1')
        self.declare_parameter('stairs_up_speed_mps', DEFAULT_STAIRS_UP_SPEED_MPS)
        self.declare_parameter('stairs_up_duration_sec', DEFAULT_STAIRS_UP_DURATION_SEC)
        self.declare_parameter('stairs_turn_angle_deg', DEFAULT_TURN_LEFT_ANGLE_DEG)
        self.declare_parameter('stairs_turn_wz_radps', DEFAULT_TURN_LEFT_WZ_RADPS)
        self.declare_parameter('stairs_down_speed_mps', DEFAULT_STAIRS_DOWN_SPEED_MPS)
        self.declare_parameter('stairs_down_duration_sec', DEFAULT_STAIRS_DOWN_DURATION_SEC)
        self.declare_parameter('inter_stage_stop_sec', DEFAULT_INTER_STAGE_STOP_SEC)

    def _read_parameters(self):
        """读取并验证边界，非法参数在启动时失败，避免运行中产生盲走。"""
        self.image_topic = self._topic('image_topic')
        self.final_cmd_topic = self._topic('final_cmd_topic')
        self.line_start_topic = self._topic('line_start_topic')
        self.line_stop_topic = self._topic('line_stop_topic')
        self.gait_lock_topic = self._topic('gait_lock_topic')
        self.status_topic = self._topic('status_topic')
        self.execute_line_motion = bool(self.get_parameter('execute_line_motion').value)
        self.execute_phase3 = bool(self.get_parameter('execute_phase3').value)
        # Phase 3 只能在本链已实际控制巡线、并确认停车后运行；否则干跑视觉
        # 验证时会跳过零速度确认，形成错误的“已安全交接”结论。
        if self.execute_phase3 and not self.execute_line_motion:
            raise ValueError('execute_phase3 requires execute_line_motion=true')
        self.line_speed_mps = self._range('line_speed_mps', 0.0, 0.25)
        self.t_confirm_frames = self._positive_int('t_confirm_frames')
        self.dark_threshold = int(self._range('dark_threshold', 0.0, 255.0))
        self.band_min_width_ratio = self._range('band_min_width_ratio', 0.0, 1.0)
        self.lane_min_width_ratio = self._range('lane_min_width_ratio', 0.0, 1.0)
        self.lane_max_width_ratio = self._range('lane_max_width_ratio', 0.0, 1.0)
        if self.lane_min_width_ratio > self.lane_max_width_ratio:
            raise ValueError('lane_min_width_ratio must not exceed lane_max_width_ratio')
        self.zero_confirm_messages = self._positive_int('zero_confirm_messages')
        self.zero_confirm_timeout_sec = self._positive('zero_confirm_timeout_sec')
        self.zero_epsilon = self._range('zero_epsilon', 0.0, 0.10)
        self.phase3_helper = str(self.get_parameter('phase3_helper').value).strip()
        self.sdk_interface = str(self.get_parameter('sdk_interface').value).strip()
        if not self.sdk_interface:
            raise ValueError('sdk_interface must not be empty')
        self.up_speed = self._range('stairs_up_speed_mps', 0.0, 0.60)
        self.up_duration = self._range('stairs_up_duration_sec', 0.01, 5.0)
        self.turn_angle = self._range('stairs_turn_angle_deg', 0.01, 90.0)
        self.turn_wz = self._range('stairs_turn_wz_radps', 0.01, 1.20)
        self.down_speed = self._range('stairs_down_speed_mps', 0.0, 0.60)
        self.down_duration = self._range('stairs_down_duration_sec', 0.01, 5.0)
        self.inter_stage_stop_sec = self._range('inter_stage_stop_sec', 0.0, 2.0)

    def _on_image(self, message):
        """从最新图像刷新 T 门禁；只持续帧确认，绝不使用旧画面。"""
        frame = self._decode_bgr8(message)
        if frame is None or self.state not in (self.WAIT_IMAGE, self.LINE_FOLLOW):
            return
        self.valid_frame_count += 1
        found, metrics = detect_t_stair_marker(
            frame, self.dark_threshold, self.band_min_width_ratio,
            self.lane_min_width_ratio, self.lane_max_width_ratio)
        self.marker_count = self.marker_count + 1 if found else 0
        if self.marker_count >= self.t_confirm_frames:
            self._request_safe_stop(metrics)
            return
        # 先累计完整的 T 确认窗口；若相机一开始就在 T 前，不会抢先发巡线启动。
        if self.state == self.WAIT_IMAGE and self.valid_frame_count >= self.t_confirm_frames:
            self.state = self.LINE_FOLLOW
            if self.execute_line_motion:
                self._publish_bool(self.start_publisher, True)
                detail = '巡线启动，持续跟线直到 T 标记；速度由 line_follower=%.3fm/s' % self.line_speed_mps
            else:
                detail = '巡线链路已就绪；execute_line_motion=false，未发送启动动作'
            self._publish_status('LINE_FOLLOW', detail)

    def _on_final_command(self, message):
        """只在已请求停车后统计最终仲裁速度，确认静止才允许 Phase 3。"""
        if self.state != self.STOPPING_FOR_T:
            return
        is_zero = (
            abs(message.linear.x) <= self.zero_epsilon
            and abs(message.linear.y) <= self.zero_epsilon
            and abs(message.angular.z) <= self.zero_epsilon
        )
        self.zero_command_count = self.zero_command_count + 1 if is_zero else 0

    def _request_safe_stop(self, metrics):
        """T 通过后先锁住巡线和 mux；Phase 3 不可与任何巡线速度并行。"""
        self.state = self.STOPPING_FOR_T
        self.stop_requested_at = time.monotonic()
        self._publish_bool(self.stop_publisher, True)
        self._publish_bool(self.lock_publisher, True)
        self._publish_status('STOPPING_FOR_T', 'T 确认，已停止巡线并等待最终零速度：{}'.format(metrics))
        self.get_logger().warn('T 型楼梯标记确认，巡线已锁定；等待零速度交接。')

    def _on_timer(self):
        """重复发布锁定状态并处理停车确认、子进程结果和失败关闭。"""
        if self.state == self.STOPPING_FOR_T:
            self._publish_bool(self.stop_publisher, True)
            self._publish_bool(self.lock_publisher, True)
            if not self.execute_line_motion:
                # 纯观测验证从未启动速度后端，无需等待不存在的 cmd_vel；仍保留
                # STOPPING 状态和锁定发布，便于确认 T 门禁的真实交接顺序。
                self._start_phase3_or_report_only()
            elif self.zero_command_count >= self.zero_confirm_messages:
                self._start_phase3_or_report_only()
            elif time.monotonic() - self.stop_requested_at > self.zero_confirm_timeout_sec:
                self._fail('等待最终零速度超时，拒绝启动楼梯动作')
        elif self.state == self.RUNNING_PHASE3 and self.phase3_process is not None:
            result = self.phase3_process.poll()
            if result is not None:
                if result == 0:
                    self.state = self.COMPLETE
                    self._publish_status('COMPLETE', 'Phase 3 动作程序已正常退出，保持停车锁定')
                else:
                    self._fail('Phase 3 动作程序退出码={}'.format(result))

    def _start_phase3_or_report_only(self):
        """在停车已确认后启动旧 Phase 3；未授权执行时只报告计划参数。"""
        if not self.execute_phase3:
            self.state = self.COMPLETE
            self._publish_status('COMPLETE', self._phase3_plan('已验证交接；execute_phase3=false，未调用真机动作'))
            return
        if not self.phase3_helper or not os.path.isfile(self.phase3_helper):
            self._fail('Phase 3 工具不存在：{}'.format(self.phase3_helper))
            return
        command = [
            self.phase3_helper, self.sdk_interface,
            '--stairs-up-speed-mps', '{:.3f}'.format(self.up_speed),
            '--stairs-up-duration-sec', '{:.3f}'.format(self.up_duration),
            '--stairs-turn-angle-deg', '{:.3f}'.format(self.turn_angle),
            '--stairs-turn-wz-radps', '{:.3f}'.format(self.turn_wz),
            '--stairs-down-speed-mps', '{:.3f}'.format(self.down_speed),
            '--stairs-down-duration-sec', '{:.3f}'.format(self.down_duration),
            '--inter-stage-stop-sec', '{:.3f}'.format(self.inter_stage_stop_sec),
            '--execute',
        ]
        try:
            self.phase3_process = subprocess.Popen(command)
        except OSError as error:
            self._fail('无法启动 Phase 3：{}'.format(error))
            return
        self.state = self.RUNNING_PHASE3
        self._publish_status('RUNNING_PHASE3', self._phase3_plan('已确认停车，启动'))

    def _phase3_plan(self, prefix):
        """输出中文可审计的 Phase 3 参数，方便现场只改集中参数后复核。"""
        return ('{}：上台阶 {:.2f}m/s×{:.1f}s；左转 {:.1f}°、{:.2f}rad/s；'
                '下台阶 {:.2f}m/s×{:.1f}s；阶段停车 {:.1f}s').format(
                    prefix, self.up_speed, self.up_duration, self.turn_angle,
                    self.turn_wz, self.down_speed, self.down_duration,
                    self.inter_stage_stop_sec)

    def _fail(self, detail):
        """失败时保持锁定，不恢复巡线，要求操作者明确重新启动验证入口。"""
        self.state = self.FAILED
        self._publish_bool(self.stop_publisher, True)
        self._publish_bool(self.lock_publisher, True)
        self._publish_status('FAILED', detail)
        self.get_logger().error(detail)

    def _decode_bgr8(self, message):
        """安全解码常见 ROS 图像；未知编码直接丢弃，禁止猜测颜色顺序。"""
        encoding = message.encoding.lower()
        if encoding not in ('bgr8', 'rgb8') or message.height <= 0 or message.width <= 0:
            self.get_logger().warning('忽略不支持图像编码：{}'.format(message.encoding))
            return None
        expected = int(message.height) * int(message.step)
        if message.step < message.width * 3 or len(message.data) < expected:
            self.get_logger().warning('忽略不完整相机帧')
            return None
        raw = np.frombuffer(message.data, dtype=np.uint8, count=expected)
        image = raw.reshape((message.height, message.step))[:, :message.width * 3]
        image = image.reshape((message.height, message.width, 3))
        return image if encoding == 'bgr8' else cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

    def _publish_bool(self, publisher, value):
        message = Bool()
        message.data = bool(value)
        publisher.publish(message)

    def _publish_status(self, state, detail):
        message = String()
        message.data = '{} {}'.format(state, detail)
        self.status_publisher.publish(message)

    def _topic(self, name):
        value = str(self.get_parameter(name).value).strip()
        if not value.startswith('/'):
            raise ValueError('{} must be an absolute ROS topic'.format(name))
        return value

    def _positive_int(self, name):
        value = int(self.get_parameter(name).value)
        if value <= 0:
            raise ValueError('{} must be a positive integer'.format(name))
        return value

    def _positive(self, name):
        return self._range(name, 0.000001, float('inf'))

    def _range(self, name, lower, upper):
        value = float(self.get_parameter(name).value)
        if not np.isfinite(value) or value < lower or value > upper:
            raise ValueError('{} out of range [{}, {}]'.format(name, lower, upper))
        return value

    def destroy_node(self):
        """退出时仍请求巡线停车和锁定；不终止已发出的 SDK Phase 3 进程。"""
        if hasattr(self, 'stop_publisher'):
            self._publish_bool(self.stop_publisher, True)
            self._publish_bool(self.lock_publisher, True)
        return super().destroy_node()


def main(args=None):
    """运行独立楼梯交接节点。"""
    rclpy.init(args=args)
    node = None
    try:
        node = StairLineHandoffNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
