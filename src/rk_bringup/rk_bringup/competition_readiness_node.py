"""非机械臂比赛链的只读 readiness 服务。

该节点只观察 ROS 图、状态 topic 和本地 cleanup guard，绝不发布运动、
绝不调用 SDK；mission_start.sh 通过 Trigger 服务决定是否可以发出一次
``/mission/start``。
"""

from __future__ import annotations

import json
import math
import os
from pathlib import Path
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from rk_interfaces.action import ExecuteMotion
from rk_interfaces.msg import LineTrack

from rk_bringup.non_arm_competition_contract import (
    FINAL_CMD_TOPIC,
    FORBIDDEN_FORMAL_NODE_MARKERS,
    MOTION_ACTION_NAME,
    ReadinessCheck,
    endpoint_is_command_mux,
    is_zero_twist,
    json_object,
    route_is_wait_start,
    status_is_terminal_or_idle,
    smoke_test_helper_status,
)


class CompetitionReadinessNode(Node):
    """汇总正式启动前必须满足的非破坏性安全条件。"""

    def __init__(self):
        """声明话题和超时参数，并建立仅订阅的观测图。"""
        super().__init__('competition_readiness_node')
        self._declare_parameters()
        self._read_parameters()
        self._last_messages = {}
        self._last_payload = {}

        self.status_publisher = self.create_publisher(
            String, '/competition/readiness_status', 10
        )
        self.service = self.create_service(
            Trigger, '/competition/check_readiness', self._on_check_readiness
        )
        self.action_client = ActionClient(
            self, ExecuteMotion, self.motion_action_name
        )

        self.create_subscription(
            Bool, self.estop_state_topic, self._on_estop_state, 10
        )
        self.create_subscription(
            Image, self.image_topic, self._on_image, 10
        )
        self.create_subscription(
            LineTrack, self.line_track_topic, self._on_line_track, 10
        )
        self.create_subscription(
            String,
            self.line_follower_status_topic,
            self._on_line_follower_status,
            10,
        )
        self.create_subscription(
            String,
            self.line_course_state_topic,
            self._on_line_course_state,
            10,
        )
        self.create_subscription(
            String,
            self.white_stage_publisher_status_topic,
            self._on_white_stage_publisher_status,
            10,
        )
        self.create_subscription(
            String,
            self.white_action_status_topic,
            self._on_white_action_status,
            10,
        )
        self.create_subscription(
            String,
            self.inspection_action_status_topic,
            self._on_inspection_action_status,
            10,
        )
        self.create_subscription(
            String, self.gait_status_topic, self._on_gait_status, 10
        )
        self.create_subscription(
            String, self.cmd_mux_status_topic, self._on_mux_status, 10
        )
        self.create_subscription(
            Twist, self.final_cmd_topic, self._on_final_cmd, 10
        )

        self.timer = self.create_timer(
            1.0 / self.status_publish_rate_hz,
            self._publish_periodic_status,
        )
        mode_label = (
            'SOFTWARE_SMOKE_MODE'
            if self.software_smoke_mode else 'HARDWARE_MODE'
        )
        self.get_logger().info(
            'Competition readiness node ready: mode={}, image_topic={}, '
            'action={}'.format(
                mode_label, self.image_topic, self.motion_action_name
            )
        )

    def _declare_parameters(self):
        """声明只读检查使用的稳定接口名称和时间边界。"""
        defaults = {
            'hardware_mode': True,
            'software_smoke_mode': False,
            'image_topic': '/camera/camera/color/image_raw',
            'final_cmd_topic': FINAL_CMD_TOPIC,
            'line_track_topic': '/perception/line_track',
            'line_follower_status_topic': '/navigation/line_follow_status',
            'line_course_state_topic': '/mission/line_course_state',
            'white_stage_publisher_status_topic': (
                '/mission/white_bar_stage_command_publisher_status'
            ),
            'white_action_status_topic': '/mission/white_bar_action_status',
            'inspection_action_status_topic': (
                '/mission/inspection_action_status'
            ),
            'sign_detections_topic': '/perception/sign_detections',
            'gait_status_topic': '/gait/status',
            'estop_state_topic': '/safety/estop_state',
            'estop_service_name': '/safety/estop',
            'motion_action_name': MOTION_ACTION_NAME,
            'cmd_mux_status_topic': '/control/cmd_mux_status',
            'sdk_server': (
                '/home/unitree/unitree_go2_sdk_test/build/'
                'go2_sdk_udp_server'
            ),
            'sdk_action_executable': '',
            'cleanup_guard_path': (
                '~/.rk_non_arm_competition/front_jump_cleanup_guard.json'
            ),
            'freshness_timeout_sec': 2.0,
            'status_publish_rate_hz': 2.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _read_parameters(self):
        """读取参数；空 helper 留给 readiness 明确拒绝而非静默回退。"""
        self.hardware_mode = self._bool_parameter('hardware_mode')
        self.software_smoke_mode = self._bool_parameter(
            'software_smoke_mode'
        )
        for name in (
            'image_topic',
            'final_cmd_topic',
            'line_track_topic',
            'line_follower_status_topic',
            'line_course_state_topic',
            'white_stage_publisher_status_topic',
            'white_action_status_topic',
            'inspection_action_status_topic',
            'sign_detections_topic',
            'gait_status_topic',
            'estop_state_topic',
            'estop_service_name',
            'motion_action_name',
            'cmd_mux_status_topic',
            'sdk_server',
            'sdk_action_executable',
            'cleanup_guard_path',
        ):
            setattr(self, name, str(self.get_parameter(name).value).strip())
        self.freshness_timeout_sec = self._positive_float_parameter(
            'freshness_timeout_sec'
        )
        self.status_publish_rate_hz = self._positive_float_parameter(
            'status_publish_rate_hz'
        )

    def _bool_parameter(self, name):
        value = self.get_parameter(name).value
        if not isinstance(value, bool):
            raise ValueError('{} must be a boolean'.format(name))
        return value

    def _positive_float_parameter(self, name):
        try:
            value = float(self.get_parameter(name).value)
        except (TypeError, ValueError) as error:
            raise ValueError('{} must be numeric'.format(name)) from error
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError('{} must be finite and positive'.format(name))
        return value

    def _remember(self, name, value):
        self._last_messages[name] = (time.monotonic(), value)

    def _on_estop_state(self, msg):
        self._remember('estop_state', bool(msg.data))

    def _on_image(self, _msg):
        self._remember('image', True)

    def _on_line_track(self, _msg):
        self._remember('line_track', True)

    def _on_line_follower_status(self, msg):
        self._remember('line_follower_status', json_object(msg.data))

    def _on_line_course_state(self, msg):
        self._remember('line_course_state', json_object(msg.data))

    def _on_white_stage_publisher_status(self, msg):
        self._remember('white_stage_status', json_object(msg.data))

    def _on_white_action_status(self, msg):
        self._remember('white_action_status', json_object(msg.data))

    def _on_inspection_action_status(self, msg):
        self._remember('inspection_action_status', json_object(msg.data))

    def _on_gait_status(self, msg):
        self._remember('gait_status', str(msg.data).strip())

    def _on_mux_status(self, msg):
        self._remember('mux_status', json_object(msg.data))

    def _on_final_cmd(self, msg):
        self._remember(
            'final_cmd',
            (
                msg.linear.x,
                msg.linear.y,
                msg.linear.z,
                msg.angular.x,
                msg.angular.y,
                msg.angular.z,
            ),
        )

    def _fresh_value(self, name):
        record = self._last_messages.get(name)
        if record is None:
            return None, None
        received_time, value = record
        age = max(0.0, time.monotonic() - received_time)
        if age > self.freshness_timeout_sec:
            return None, age
        return value, age

    def _has_publisher(self, topic):
        return bool(self.get_publishers_info_by_topic(topic))

    def _node_names(self):
        names = set()
        for name, namespace in self.get_node_names_and_namespaces():
            normalized = '{}{}'.format(
                namespace.rstrip('/'), '/' + name.lstrip('/'),
            )
            names.add(normalized.lower())
        return names

    @staticmethod
    def _process_running(token):
        """只读取 /proc cmdline，避免 readiness 产生任何系统副作用。"""
        proc_root = Path('/proc')
        try:
            entries = tuple(proc_root.iterdir())
        except OSError:
            return False
        for entry in entries:
            if not entry.name.isdigit():
                continue
            try:
                raw = (entry / 'cmdline').read_bytes()
            except (OSError, PermissionError):
                continue
            if token in raw.decode('utf-8', errors='ignore'):
                return True
        return False

    def _file_is_executable(self, path):
        return bool(path) and os.path.isfile(path) and os.access(path, os.X_OK)

    def _cleanup_guard_is_clean(self):
        """读取 supervisor journal；不存在表示尚无 jump，允许启动。"""
        if not self.cleanup_guard_path:
            return False, 'cleanup_guard_path_empty'
        path = Path(os.path.expanduser(self.cleanup_guard_path))
        if not path.exists():
            return True, 'cleanup_guard_absent_no_prior_jump'
        try:
            with path.open('r', encoding='utf-8') as stream:
                record = json.load(stream)
        except (OSError, ValueError, TypeError, json.JSONDecodeError) as error:
            return False, 'cleanup_guard_unreadable:{}'.format(error)
        if not isinstance(record, dict):
            return False, 'cleanup_guard_not_object'
        state = str(record.get('state', '')).strip().upper()
        if state == 'DIRTY':
            return False, 'cleanup_guard_dirty'
        if state != 'CLEAN':
            return False, 'cleanup_guard_unknown_state:{}'.format(state)
        return True, 'cleanup_guard_clean'

    def _status_available(self, key, label):
        value, age = self._fresh_value(key)
        if value is None:
            suffix = 'missing' if age is None else 'stale_{:.3f}s'.format(age)
            return ReadinessCheck(label, False, suffix)
        return ReadinessCheck(label, True, 'fresh_{:.3f}s'.format(age))

    def _action_idle_check(self, key, label):
        value, age = self._fresh_value(key)
        if value is None:
            suffix = 'missing' if age is None else 'stale_{:.3f}s'.format(age)
            return ReadinessCheck(label, False, suffix)
        if not status_is_terminal_or_idle(value):
            raw_state = value.get('state', value.get('status', ''))
            return ReadinessCheck(
                label, False, 'active_or_unknown:{}'.format(raw_state)
            )
        return ReadinessCheck(label, True, 'terminal_or_idle')

    def evaluate(self):
        """执行一次完整的只读检查，返回可序列化的检查列表。"""
        checks = []
        publisher_infos = self.get_publishers_info_by_topic(
            self.final_cmd_topic
        )
        mux_publishers = [
            endpoint for endpoint in publisher_infos
            if endpoint_is_command_mux(endpoint)
        ]
        final_owner_ok = (
            len(publisher_infos) == 1 and len(mux_publishers) == 1
        )
        checks.append(ReadinessCheck(
            'final_cmd_single_command_mux_publisher',
            final_owner_ok,
            'publisher_count={}, command_mux_count={}'.format(
                len(publisher_infos), len(mux_publishers)
            ),
        ))
        checks.append(ReadinessCheck(
            'execute_motion_action_server',
            bool(self.action_client.server_is_ready()),
            self.motion_action_name,
        ))

        node_names = self._node_names()
        gait_present = any(
            name.endswith('/gait_control_node') for name in node_names
        )
        gait_status, gait_age = self._fresh_value('gait_status')
        gait_faulted = str(gait_status or '').upper() in {
            'FAILED', 'EMERGENCY_STOP', 'FAULTED'
        }
        checks.append(ReadinessCheck(
            'gait_control_started_without_latched_fault',
            gait_present and gait_status is not None and not gait_faulted,
            'present={}, status={}, age={}'.format(
                gait_present,
                gait_status or 'missing',
                'missing' if gait_age is None else '{:.3f}s'.format(gait_age),
            ),
        ))
        guard_ok, guard_detail = self._cleanup_guard_is_clean()
        checks.append(ReadinessCheck(
            'front_jump_cleanup_guard_clean', guard_ok, guard_detail
        ))
        service_names = {
            name for name, _types in self.get_service_names_and_types()
        }
        checks.append(ReadinessCheck(
            'command_mux_estop_service',
            self.estop_service_name in service_names,
            self.estop_service_name,
        ))

        estop_value, estop_age = self._fresh_value('estop_state')
        checks.append(ReadinessCheck(
            'estop_state_fresh_and_false',
            estop_value is False,
            'value={}, age={}'.format(
                estop_value,
                'missing' if estop_age is None else '{:.3f}s'.format(
                    estop_age
                ),
            ),
        ))
        checks.append(ReadinessCheck(
            'image_topic_has_publisher',
            self._has_publisher(self.image_topic), self.image_topic,
        ))
        image_value, image_age = self._fresh_value('image')
        checks.append(ReadinessCheck(
            'image_topic_fresh',
            image_value is not None,
            'missing' if image_age is None else '{:.3f}s'.format(image_age),
        ))
        line_value, line_age = self._fresh_value('line_track')
        checks.append(ReadinessCheck(
            'line_track_fresh',
            line_value is not None,
            'missing' if line_age is None else '{:.3f}s'.format(line_age),
        ))
        checks.append(self._status_available(
            'line_follower_status', 'line_follower_status_available'
        ))
        checks.append(self._status_available(
            'line_course_state', 'line_course_state_available'
        ))
        checks.append(self._status_available(
            'white_stage_status', 'white_stage_publisher_status_available'
        ))
        checks.append(self._action_idle_check(
            'white_action_status', 'white_bar_action_idle'
        ))
        checks.append(self._action_idle_check(
            'inspection_action_status', 'inspection_action_idle'
        ))
        checks.append(ReadinessCheck(
            'sign_detector_output_topic',
            self._has_publisher(self.sign_detections_topic),
            self.sign_detections_topic,
        ))
        helper_ok = self._file_is_executable(self.sdk_action_executable)
        checks.append(ReadinessCheck(
            'sdk_helper_exists_and_executable',
            helper_ok,
            self.sdk_action_executable or 'empty',
        ))
        if self.software_smoke_mode:
            # 仅存在或可执行不足以证明安全：/usr/bin/true、shell 脚本和
            # 真实 Unitree helper 都不能作为无硬件验收的替身。
            marker_ok, marker_detail = smoke_test_helper_status(
                self.sdk_action_executable
            )
            checks.append(ReadinessCheck(
                'software_smoke_test_only_elf_helper',
                marker_ok,
                marker_detail,
            ))

        final_cmd, final_cmd_age = self._fresh_value('final_cmd')
        checks.append(ReadinessCheck(
            'final_cmd_fresh_and_zero',
            final_cmd is not None and is_zero_twist(final_cmd),
            'missing' if final_cmd_age is None else '{:.3f}s'.format(
                final_cmd_age
            ),
        ))
        course_state, course_age = self._fresh_value('line_course_state')
        checks.append(ReadinessCheck(
            'route_wait_start_without_active_action',
            route_is_wait_start(course_state),
            'missing' if course_age is None else str(course_state),
        ))

        forbidden_nodes = sorted(
            name for name in node_names
            if any(
                marker in name for marker in FORBIDDEN_FORMAL_NODE_MARKERS
            )
        )
        checks.append(ReadinessCheck(
            'no_excluded_or_mock_nodes',
            not forbidden_nodes,
            ','.join(forbidden_nodes) or 'none',
        ))

        if self.software_smoke_mode:
            prohibited_smoke_nodes = sorted(
                name for name in node_names
                if 'cmd_vel_udp_forwarder' in name
                or 'realsense' in name
                or name.endswith('/camera')
            )
            sdk_process = self._process_running('go2_sdk_udp_server')
            checks.append(ReadinessCheck(
                'software_smoke_hardware_suppressed',
                not prohibited_smoke_nodes and not sdk_process,
                'nodes={}, sdk_process={}'.format(
                    prohibited_smoke_nodes, sdk_process
                ),
            ))
        elif self.hardware_mode:
            server_ok = self._file_is_executable(self.sdk_server)
            server_running = self._process_running('go2_sdk_udp_server')
            forwarder_running = any(
                name.endswith('/cmd_vel_udp_forwarder')
                for name in node_names
            )
            checks.append(ReadinessCheck(
                'hardware_sdk_server_ready',
                server_ok and server_running,
                'path_ok={}, process_running={}'.format(
                    server_ok, server_running
                ),
            ))
            checks.append(ReadinessCheck(
                'hardware_udp_forwarder_started',
                forwarder_running,
                'node_present={}'.format(forwarder_running),
            ))
        else:
            checks.append(ReadinessCheck(
                'mode_configuration',
                False,
                'hardware_mode=false requires software_smoke_mode=true',
            ))
        return checks

    def _make_payload(self):
        checks = self.evaluate()
        success = all(check.ok for check in checks if check.critical)
        return {
            'success': success,
            'mode': (
                'SOFTWARE_SMOKE_MODE'
                if self.software_smoke_mode else 'HARDWARE_MODE'
            ),
            'checks': [check.as_dict() for check in checks],
            'timestamp_monotonic_sec': time.monotonic(),
        }

    def _publish_payload(self, payload):
        """发布 readiness 心跳；launch 正在关闭时不再访问失效 publisher。"""
        if not rclpy.ok():
            return
        message = String()
        message.data = json.dumps(
            payload, ensure_ascii=True, separators=(',', ':'), allow_nan=False
        )
        try:
            self.status_publisher.publish(message)
        except Exception:
            if rclpy.ok():
                raise
            return
        self._last_payload = payload

    def _publish_periodic_status(self):
        self._publish_payload(self._make_payload())

    def _on_check_readiness(self, _request, response):
        """Trigger 回调只计算/发布状态，永不改变 estop 或 mission 状态。"""
        payload = self._make_payload()
        self._publish_payload(payload)
        response.success = bool(payload['success'])
        response.message = json.dumps(
            payload, ensure_ascii=True, separators=(',', ':'), allow_nan=False
        )
        return response


def main(args=None):
    """运行 readiness 节点；关闭时不触发任何控制命令。"""
    rclpy.init(args=args)
    node = None
    try:
        node = CompetitionReadinessNode()
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
