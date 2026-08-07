"""非机械臂软件验收的确定性合成 ROS 输入发布器。

此节点仅在 ``software_smoke_mode`` 下由正式 launch 启动。它不订阅或调用
任何 SDK/网络接口，不发布最终 ``/navigation/cmd_vel``，而是给真实路线、
白横线 executor、gait Action server 与 inspection executor 提供合成证据。
"""

from __future__ import annotations

import json
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String

from rk_interfaces.msg import LineTrack, SignDetection, SignDetectionArray
from rk_interfaces.msg import SpecialTargetDetection
from rk_bringup.non_arm_competition_contract import (
    DEFAULT_SIGN_CAMERA_FRAME_ID,
)


# 仅 smoke 保持 3 秒低置信新帧：预热订阅已完成 DDS 发现，足够观察真实
# follower 的零速门控，且严格小于正式 5 秒 START_READY 安全超时，不影响实机参数。
START_READY_HOLD_SEC = 3.0
# 合成白线只在对应阶段已 arm 时投递。给出有限的重发窗口以覆盖 DDS 订阅
# 刚连接时丢失首帧的情况；上限避免测试输入在路线故障时无限刷屏。
WHITE_EVIDENCE_MAX_PULSES = 20


class NonArmSmokePublisher(Node):
    """发布可重复的赛道输入，并按真实路线状态逐步推进 happy path。"""

    def __init__(self):
        """建立测试输入 publisher；所有输出均停留在 ROS 软件图内。"""
        super().__init__('competition_smoke_publisher')
        self._declare_parameters()
        self.line_image_topic = self._string_parameter('line_image_topic')
        self.sign_image_topic = self._string_parameter('sign_image_topic')
        self.scenario = self._string_parameter('scenario').lower()
        self.auto_start = self._bool_parameter('auto_start')
        self.publish_rate_hz = self._positive_float_parameter(
            'publish_rate_hz'
        )
        self._started_at = time.monotonic()
        self._started = False
        self._mission_started_at = None
        # 交付器可为同一个逻辑请求进行有限 DDS 重传；输入器只记首条并冻结
        # START_READY 时间窗，后续重复消息绝不能把路线或 follower 重置回起点。
        self._mission_start_messages = 0
        self._prestart_sent = False
        self._white_pulses = {'START': 0, 'FINISH': 0}
        self._red_pulses = 0
        self._valid_sign_pulses = 0
        self._stop_outside_pulses = 0
        self._stop_inside_pulses = 0
        self._last_route = {}
        self._last_route_at = None
        self._line_follower_ready = False
        self._last_status = {}
        self._last_status_at = 0.0
        self._completed = False

        self.line_image_publisher = self.create_publisher(
            Image, self.line_image_topic, 10
        )
        self.sign_image_publisher = self.create_publisher(
            Image, self.sign_image_topic, 10
        )
        self.line_publisher = self.create_publisher(
            LineTrack, '/perception/line_track', 10
        )
        self.red_publisher = self.create_publisher(
            SpecialTargetDetection, '/perception/red_circle_detection', 10
        )
        self.white_publisher = self.create_publisher(
            SpecialTargetDetection, '/perception/white_bar_detection', 10
        )
        self.stop_publisher = self.create_publisher(
            SpecialTargetDetection, '/perception/stop_zone_detection', 10
        )
        self.sign_publisher = self.create_publisher(
            SignDetectionArray, '/perception/sign_detections', 10
        )
        self.start_publisher = self.create_publisher(
            Bool, '/mission/start', 10
        )
        self.status_publisher = self.create_publisher(
            String, '/competition/smoke_status', 10
        )
        self.create_subscription(
            Bool,
            '/mission/start',
            self._on_mission_start,
            10,
        )
        self.create_subscription(
            String,
            '/mission/line_course_state',
            self._on_route_state,
            10,
        )
        self.create_subscription(
            String,
            '/navigation/line_follow_status',
            self._on_line_follower_status,
            10,
        )
        self.timer = self.create_timer(
            1.0 / self.publish_rate_hz, self._on_timer
        )
        self.get_logger().info(
            'SOFTWARE_SMOKE_MODE publisher ready: scenario={}, '
            'auto_start={}, line_topic={}, sign_topic={}'.format(
                self.scenario, self.auto_start,
                self.line_image_topic, self.sign_image_topic,
            )
        )

    def _declare_parameters(self):
        """仅暴露合成输入的控制参数，避免引入任何硬件参数。"""
        self.declare_parameter('line_image_topic', '/line_camera/image_raw')
        self.declare_parameter('sign_image_topic', '/go2/front_camera/image_raw')
        self.declare_parameter('scenario', 'idle')
        self.declare_parameter('auto_start', False)
        self.declare_parameter('publish_rate_hz', 20.0)

    def _string_parameter(self, name):
        value = str(self.get_parameter(name).value).strip()
        if not value:
            raise ValueError('{} must not be empty'.format(name))
        return value

    def _bool_parameter(self, name):
        value = self.get_parameter(name).value
        if not isinstance(value, bool):
            raise ValueError('{} must be boolean'.format(name))
        return value

    def _positive_float_parameter(self, name):
        value = float(self.get_parameter(name).value)
        if value <= 0.0:
            raise ValueError('{} must be positive'.format(name))
        return value

    def _on_route_state(self, msg):
        """缓存真实路线状态，测试输入永远由其当前 phase 决定。"""
        try:
            payload = json.loads(msg.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        if isinstance(payload, dict):
            self._last_route = payload
            self._last_route_at = time.monotonic()
            # mission_start.sh 可在 readiness 通过后外部发布 start；测试输入器
            # 只观察该事实，不重复发布或重置 run_id。
            if payload.get('mission_started') is True:
                if not self._started:
                    self._mission_started_at = time.monotonic()
                self._started = True

    def _on_mission_start(self, msg):
        """观察同一逻辑 start 的重传；只首条影响 smoke 输入状态机。"""
        if bool(msg.data):
            self._mission_start_messages += 1
            if self._mission_started_at is None:
                self._mission_started_at = time.monotonic()

    def _on_line_follower_status(self, msg):
        """只观察真实 follower 的 ready 状态，绝不伪造 START_READY 结果。"""
        try:
            payload = json.loads(msg.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            self._line_follower_ready = False
            return
        self._line_follower_ready = bool(
            isinstance(payload, dict)
            and payload.get('mission_started') is True
            and payload.get('nav_state') == 'LINE_FOLLOW'
            and payload.get('ready') is True
        )

    def _stamp_header(self, message, frame_id='software_smoke'):
        """统一写入测试时间戳；标识相机帧保留正式 Go2 frame 契约。"""
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = frame_id
        return message

    def _publish_image(self):
        """发布极小 RGB 帧到两路独立 Topic，确保 tracker 和 sign detector
        各自获得独立的新鲜度证据，不共享单一相机 Topic。"""
        msg = self._stamp_header(Image())
        msg.height = 2
        msg.width = 2
        msg.encoding = 'rgb8'
        msg.is_bigendian = False
        msg.step = 6
        msg.data = bytes((0, 0, 0) * 4)
        self.line_image_publisher.publish(msg)
        # sign 的合成帧必须满足正式 Go2 相机 frame_id 只读检查，但不会启动桥接。
        sign_msg = self._stamp_header(
            Image(), frame_id=DEFAULT_SIGN_CAMERA_FRAME_ID
        )
        sign_msg.height = 2
        sign_msg.width = 2
        sign_msg.encoding = 'rgb8'
        sign_msg.is_bigendian = False
        sign_msg.step = 6
        sign_msg.data = bytes((128, 128, 128) * 4)
        self.sign_image_publisher.publish(sign_msg)

    def _publish_line(self):
        """发布有限、居中的可见线，完整 smoke 在 start 后先保留零速窗口。"""
        start_ready_hold_active = (
            self.scenario == 'full'
            and self._mission_started_at is not None
            and time.monotonic() - self._mission_started_at
            < START_READY_HOLD_SEC
        )
        msg = self._stamp_header(LineTrack())
        msg.line_visible = True
        # 持续送低置信但有限的新帧，而不是停帧；这样既能验证 START_READY
        # 在不合格证据下保持零速，也不会把正常验收误变成 line 消息陈旧故障。
        msg.confidence = 0.20 if start_ready_hold_active else 0.90
        msg.lateral_error = 0.0
        msg.heading_error = 0.0
        self.line_publisher.publish(msg)

    def _special_target(self, target_type, *, inside_candidate=False):
        """构造有限且高置信的目标消息；阶段门控由真实路线节点处理。"""
        msg = self._stamp_header(SpecialTargetDetection())
        msg.target_type = target_type
        msg.visible = True
        msg.confidence = 0.90
        msg.center_x = 0.50
        msg.center_y = 0.80 if target_type == 'white_bar' else 0.70
        msg.area_ratio = 0.05 if target_type == 'white_bar' else 0.03
        msg.width_ratio = 0.50
        msg.height_ratio = 0.05
        msg.inside_candidate = bool(inside_candidate)
        msg.direction_hint = ''
        msg.reason = 'software_smoke'
        return msg

    def _publish_sign(self, sign_value, confidence):
        """发布一帧 warning 检测；未知/低置信帧专门验证无 fallback。"""
        array = self._stamp_header(SignDetectionArray())
        detection = self._stamp_header(SignDetection())
        detection.sign_type = 'warning'
        detection.sign_value = sign_value
        detection.confidence = float(confidence)
        array.detections = [detection]
        self.sign_publisher.publish(array)

    def _publish_empty_signs(self):
        array = self._stamp_header(SignDetectionArray())
        array.detections = []
        self.sign_publisher.publish(array)

    def _publish_prestart_evidence(self):
        """START 前故意发红圈/蓝区，验证路线核心不会跨阶段累计。"""
        if self._prestart_sent or time.monotonic() - self._started_at < 0.25:
            return
        self._prestart_sent = True
        self.red_publisher.publish(self._special_target('red_circle'))
        self.stop_publisher.publish(self._special_target('stop_zone'))
        self.get_logger().info(
            'SOFTWARE_SMOKE pre-start red/stop evidence published'
        )

    def _maybe_auto_start(self):
        if (
            self.scenario != 'full'
            or not self.auto_start
            or self._started
            or time.monotonic() - self._started_at < 0.75
        ):
            return
        msg = Bool()
        msg.data = True
        self.start_publisher.publish(msg)
        self._started = True
        self.get_logger().info('SOFTWARE_SMOKE published one /mission/start')

    def _route_phase(self):
        return str(self._last_route.get('route_phase', '')).strip()

    def _route_state(self):
        return str(self._last_route.get('state', '')).strip()

    def _white_stage(self):
        return str(self._last_route.get('white_bar_stage', '')).strip()

    def _white_stage_state(self):
        return str(self._last_route.get('white_bar_stage_state', '')).strip()

    def _publish_next_white_bar(self, stage):
        """在已 arm 阶段有限重发白线，避免首批输入早于 DDS 订阅连接。"""
        if (
            self._white_pulses[stage] >= WHITE_EVIDENCE_MAX_PULSES
            or self._white_stage() != stage
            or self._white_stage_state() != '{}_ARMED'.format(stage)
        ):
            return
        self.white_publisher.publish(self._special_target('white_bar'))
        self._white_pulses[stage] += 1
        if (
            self._white_pulses[stage] <= 3
            or self._white_pulses[stage] == WHITE_EVIDENCE_MAX_PULSES
        ):
            self.get_logger().info(
                'SOFTWARE_SMOKE {} white-bar evidence {}/{}'.format(
                    stage,
                    self._white_pulses[stage],
                    WHITE_EVIDENCE_MAX_PULSES,
                )
            )

    def _drive_full_scenario(self):
        """按真实 ``route_phase`` 顺序推进，不伪造 Action 成功状态。"""
        if not self._line_follower_ready:
            return
        phase = self._route_phase()
        state = self._route_state()
        if phase == 'START_STAGE':
            self._publish_next_white_bar('START')
            return
        if phase == 'MID_ROUTE' and self._red_pulses < 3:
            self.red_publisher.publish(self._special_target('red_circle'))
            self._red_pulses += 1
            self.get_logger().info(
                'SOFTWARE_SMOKE red-circle evidence {}/3'.format(
                    self._red_pulses
                )
            )
            return
        if state in ('INSPECTION_WAIT_SIGN', 'INSPECTION_ACTION'):
            # 先证明低置信和未知牌不会执行，再给出唯一允许的正式映射。
            if self._valid_sign_pulses == 0:
                self._publish_sign('electric_shock', 0.20)
                self._valid_sign_pulses = -1
                return
            if self._valid_sign_pulses == -1:
                self._publish_sign('unknown_warning', 0.95)
                self._valid_sign_pulses = 1
                return
            if 1 <= self._valid_sign_pulses <= 5:
                self._publish_sign('electric_shock', 0.95)
                self._valid_sign_pulses += 1
                return
        # FINISH 命令可在 TURN_AFTER_RED 时提前到达；只有路线节点自身已切到
        # FINISH_STAGE 后才投递白线，不能把早到的视觉证据消耗在转向阶段。
        if phase == 'FINISH_STAGE' and state == 'FINISH_STAGE':
            self._publish_next_white_bar('FINISH')
            return
        if phase == 'FINAL_ZONE_ARMED':
            if self._stop_outside_pulses < 3:
                self.stop_publisher.publish(
                    self._special_target('stop_zone', inside_candidate=False)
                )
                self._stop_outside_pulses += 1
                return
            if state == 'APPROACH_STOP_ZONE' and self._stop_inside_pulses < 3:
                self.stop_publisher.publish(
                    self._special_target('stop_zone', inside_candidate=True)
                )
                self._stop_inside_pulses += 1
                return
        if phase == 'FINAL_STOP' and not self._completed:
            self._completed = True
            self.get_logger().info(
                'SOFTWARE_SMOKE full route reached FINAL_STOP'
            )

    def _publish_status(self):
        now = time.monotonic()
        if now - self._last_status_at < 0.25:
            return
        self._last_status_at = now
        payload = {
            'scenario': self.scenario,
            'started': self._started,
            'mission_start_messages_observed': self._mission_start_messages,
            'start_ready_hold_active': bool(
                self._mission_started_at is not None
                and time.monotonic() - self._mission_started_at
                < START_READY_HOLD_SEC
            ),
            'route_phase': self._route_phase(),
            'route_state': self._route_state(),
            'real_line_follower_ready': self._line_follower_ready,
            'completed': self._completed,
            'prestart_evidence_sent': self._prestart_sent,
            'white_pulses': dict(self._white_pulses),
            'red_pulses': self._red_pulses,
            'valid_sign_pulses': self._valid_sign_pulses,
            'stop_outside_pulses': self._stop_outside_pulses,
            'stop_inside_pulses': self._stop_inside_pulses,
            'hardware_access': False,
        }
        self._last_status = payload
        msg = String()
        msg.data = json.dumps(payload, separators=(',', ':'))
        self.status_publisher.publish(msg)

    def _on_timer(self):
        """维持新鲜基线，并在 full 场景下由真实状态机决定下一输入。"""
        if not rclpy.ok():
            return
        try:
            self._publish_image()
            self._publish_line()
            if self._route_state() not in (
                'INSPECTION_WAIT_SIGN', 'INSPECTION_ACTION'
            ):
                self._publish_empty_signs()
            self._publish_prestart_evidence()
            self._maybe_auto_start()
            if self.scenario == 'full' and self._started:
                self._drive_full_scenario()
            self._publish_status()
        except Exception:
            # launch 关闭可在 timer 回调中间失效 publisher；正常运行时仍抛出，
            # 不把真实测试/输入错误伪装成成功。
            if rclpy.ok():
                raise


def main(args=None):
    """运行软件 smoke 输入器；任何退出都不会向机器人发送命令。"""
    rclpy.init(args=args)
    node = None
    try:
        node = NonArmSmokePublisher()
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
