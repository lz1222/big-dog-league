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
import shutil
import time
from typing import Set

from geometry_msgs.msg import Twist
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from rclpy.qos import ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from rk_interfaces.action import ExecuteMotion
from rk_interfaces.msg import LineTrack

from rk_bringup.non_arm_competition_contract import (
    DEFAULT_LINE_IMAGE_TOPIC,
    DEFAULT_SIGN_CAMERA_FRAME_ID,
    DEFAULT_SIGN_IMAGE_TOPIC,
    FINAL_CMD_TOPIC,
    FORBIDDEN_FORMAL_NODE_MARKERS,
    MOTION_ACTION_NAME,
    SIGN_CAMERA_BRIDGE_NODE,
    ReadinessCheck,
    is_zero_twist,
    json_object,
    route_is_wait_start,
    status_is_terminal_or_idle,
    smoke_test_helper_status,
)
from rk_bringup.readiness_profile import (
    ISOLATED_LINE_VALIDATION_PROFILE,
    profile_skips_check,
    readiness_profile_graph_check,
    skipped_by_profile_detail,
    validate_readiness_profile,
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
        # SDK status 是调用完成后的事件流而非心跳。当前实例必须先完成
        # STARTUP_STOP，再由唯一 server 完成 CLASSIC_VERIFIED；同实例
        # SDK_ERROR 永久撤销资格。
        self._sdk_startup_status = None
        self._sdk_classic_verified_status = None
        self._sdk_error_status = None

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
        # USB 图像发布者使用 sensor-data/best-effort；readiness 必须使用兼容
        # QoS，否则 tracker 正常工作时本节点仍会误报 LINE_CAMERA_READY=false。
        self.line_image_subscription = self.create_subscription(
            Image, self.line_image_topic, self._on_image,
            qos_profile_sensor_data,
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
        gait_mode_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            String, self.gait_mode_status_topic,
            self._on_gait_mode_status, gait_mode_qos,
        )
        self.create_subscription(
            Bool, self.gait_control_lock_topic,
            self._on_gait_control_lock, 10,
        )
        self.create_subscription(
            String, self.gait_control_lock_status_topic,
            self._on_gait_control_lock_status, 10,
        )
        self.create_subscription(
            String, self.cmd_mux_status_topic, self._on_mux_status, 10
        )
        self.create_subscription(
            Twist, self.final_cmd_topic, self._on_final_cmd, 10
        )
        self.create_subscription(
            Image,
            self.sign_image_topic,
            self._on_sign_camera_image,
            10,
        )
        sdk_status_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        # 显式保留订阅句柄；结合 forwarder 的有界原样重放，晚于 startup ACK
        # 启动的 readiness 也能绑定当前 server_instance_id。
        self.sdk_motion_status_subscription = self.create_subscription(
            String,
            self.sdk_motion_status_topic,
            self._on_sdk_motion_status,
            sdk_status_qos,
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
            'Competition readiness node ready: mode={}, '
            'line_topic={}, sign_topic={}, action={}'.format(
                mode_label, self.line_image_topic,
                self.sign_image_topic, self.motion_action_name
            )
        )

    def _declare_parameters(self):
        """声明只读检查使用的稳定接口名称和时间边界。"""
        defaults = {
            'hardware_mode': True,
            'software_smoke_mode': False,
            # 生产默认永远完整检查；isolated profile 只供明确关闭 mission
            # nodes 的单次巡线 validation，不能用多个模糊 skip 参数替代。
            'readiness_profile': 'production',
            'start_mission_nodes': True,
            'line_image_topic': DEFAULT_LINE_IMAGE_TOPIC,
            'sign_image_topic': DEFAULT_SIGN_IMAGE_TOPIC,
            'sign_camera_frame_id': DEFAULT_SIGN_CAMERA_FRAME_ID,
            'require_arm_camera': False,
            'arm_color_topic': '/arm_camera/color/image_raw',
            'arm_depth_topic': '/arm_camera/aligned_depth_to_color/image_raw',
            'arm_camera_info_topic': '/arm_camera/color/camera_info',
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
            'gait_mode_status_topic': '/gait/mode_status',
            'gait_control_lock_topic': '/gait/control_lock',
            'gait_control_lock_status_topic': '/gait/control_lock/status',
            'estop_state_topic': '/safety/estop_state',
            'estop_service_name': '/safety/estop',
            'motion_action_name': MOTION_ACTION_NAME,
            'cmd_mux_status_topic': '/control/cmd_mux_status',
            # 生产路径由 launch 文件通过 FindPackagePrefix 显式注入；
            # 此默认值仅保证 launch 未覆盖时不无声回退到其他编译目录。
            'sdk_server': 'rk_go2_sdk_bridge',
            'sdk_action_executable': '',
            'sdk_motion_status_topic': '/go2/sdk_motion_status',
            'sdk_server_instance_id': '',
            'sdk_command_port': 15001,
            'sdk_status_port': 15002,
            'cleanup_guard_path': (
                '~/.rk_non_arm_competition/front_jump_cleanup_guard.json'
            ),
            'freshness_timeout_sec': 2.0,
            # 兼容既有参数名；SDK status 已明确为 event stream，readiness
            # 不再按该 elapsed 值把当前实例的 STARTUP_STOP 判 stale。
            'sdk_status_freshness_timeout_sec': 30.0,
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
        self.start_mission_nodes = self._bool_parameter(
            'start_mission_nodes'
        )
        self.readiness_profile = validate_readiness_profile(
            self.get_parameter('readiness_profile').value
        )
        for name in (
            'line_image_topic',
            'sign_image_topic',
            'sign_camera_frame_id',
            'arm_color_topic',
            'arm_depth_topic',
            'arm_camera_info_topic',
            'final_cmd_topic',
            'line_track_topic',
            'line_follower_status_topic',
            'line_course_state_topic',
            'white_stage_publisher_status_topic',
            'white_action_status_topic',
            'inspection_action_status_topic',
            'sign_detections_topic',
            'gait_status_topic',
            'gait_mode_status_topic',
            'gait_control_lock_topic',
            'gait_control_lock_status_topic',
            'estop_state_topic',
            'estop_service_name',
            'motion_action_name',
            'cmd_mux_status_topic',
            'sdk_server',
            'sdk_action_executable',
            'sdk_motion_status_topic',
            'sdk_server_instance_id',
            'cleanup_guard_path',
        ):
            setattr(self, name, str(self.get_parameter(name).value).strip())
        self.require_arm_camera = self._bool_parameter('require_arm_camera')
        self.freshness_timeout_sec = self._positive_float_parameter(
            'freshness_timeout_sec'
        )
        self.sdk_status_freshness_timeout_sec = self._positive_float_parameter(
            'sdk_status_freshness_timeout_sec'
        )
        self.sdk_command_port = self._port_parameter('sdk_command_port')
        self.sdk_status_port = self._port_parameter('sdk_status_port')
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

    def _port_parameter(self, name):
        """读取 UDP 端口；readiness 仅检查端点，不创建或修改 socket。"""
        value = self.get_parameter(name).value
        if isinstance(value, bool) or not isinstance(value, int):
            raise ValueError('{} must be an integer UDP port'.format(name))
        if value < 1 or value > 65535:
            raise ValueError('{} must be within 1..65535'.format(name))
        return int(value)

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

    def _on_gait_mode_status(self, msg):
        """记录全局 owner 的权威状态；JSON 非对象会在 readiness 中拒绝。"""
        self._remember('gait_mode_status', json_object(msg.data))

    def _on_gait_control_lock(self, msg):
        """记录仲裁后的真实锁值，避免只检查 publisher 存在便放行。"""
        self._remember('gait_control_lock', bool(msg.data))

    def _on_gait_control_lock_status(self, msg):
        """记录 profile/required-set 证据，不以虚构心跳代替缺失 provider。"""
        self._remember('gait_control_lock_status', json_object(msg.data))

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

    def _on_sign_camera_image(self, msg):
        self._remember('sign_camera_image', str(msg.header.frame_id))

    def _on_sdk_motion_status(self, msg):
        """绑定当前实例 SDK 事件，禁止用 idle 时间伪造 backend 故障。"""
        status = json_object(msg.data)
        self._remember('sdk_motion_status', status)
        if not isinstance(status, dict):
            return
        if status.get('server_instance_id') != self.sdk_server_instance_id:
            return
        if status.get('event') == 'SDK_ERROR':
            self._sdk_error_status = status
            return
        if (
            status.get('event') == 'STARTUP_STOP'
            and status.get('ret') == 0
            and self._valid_sdk_status_identity(status)
        ):
            self._sdk_startup_status = status
            return
        if (
            status.get('event') == 'CLASSIC_VERIFIED'
            and status.get('ret') == 0
            and self._valid_sdk_status_identity(status)
        ):
            self._sdk_classic_verified_status = status

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

    @staticmethod
    def _normalized_gid(endpoint):
        """将 endpoint_gid 标准化为可哈希的 bytes，失败返回 None。"""
        try:
            raw = endpoint.endpoint_gid
        except Exception:
            return None
        if raw is None:
            return None
        try:
            if isinstance(raw, (bytes, bytearray)):
                return bytes(raw)
            return bytes(list(raw))
        except (TypeError, ValueError):
            return None

    @staticmethod
    def _unique_gid_publishers(topic_infos):
        """按 GID 去重后的唯一发布者列表。

        同一 GID 被 Foxy/CycloneDDS 重复返回时只保留一个。
        无法读取 GID 的端点返回空列表（fail-closed）。
        """
        # Foxy 固定使用 Python 3.8；使用 typing.Set 保持 GID 去重逻辑可导入。
        # 此处仍以不可读取 GID 返回空列表 fail-closed，不能放宽唯一发布者约束。
        seen: Set[bytes] = set()
        unique = []
        for endpoint in topic_infos:
            gid = CompetitionReadinessNode._normalized_gid(endpoint)
            if gid is None:
                return []  # 无法确认 → 拒绝通过
            if gid not in seen:
                seen.add(gid)
                unique.append(endpoint)
        return unique

    @staticmethod
    def _single_gid_publisher_gate(
        publisher_infos, expected_node_name, expected_namespace='',
    ):
        """返回 (ok, detail)。严格单 GID 发布者检查。

        规则：
        - 至少一个发布者端点
        - 按 endpoint_gid 去重
        - 去重后唯一 GID 数量必须严格等于 1
        - 该唯一 GID 的 node_name 和 namespace 必须匹配预期
        - 同一 GID 的重复 endpoint 可通过
        - 两个不同 GID 即使节点名和 namespace 完全相同也必须拒绝
        - GID 不可读时 fail-closed
        """
        if not publisher_infos:
            return False, 'raw_count=0'

        unique = CompetitionReadinessNode._unique_gid_publishers(
            publisher_infos
        )
        if not unique:
            return False, (
                'gid_unreadable raw_count={}'.format(len(publisher_infos))
            )

        if len(unique) != 1:
            node_names = sorted(set(
                getattr(ep, 'node_name', '?') for ep in unique
            ))
            return False, (
                'unique_gid_count={} raw_count={} nodes={}'.format(
                    len(unique), len(publisher_infos),
                    ','.join(node_names),
                )
            )

        sole = unique[0]
        node_name = getattr(sole, 'node_name', '')
        namespace = str(getattr(sole, 'node_namespace', ''))
        # Normalize root namespace
        if namespace in ('', '/'):
            namespace = ''

        if node_name != expected_node_name:
            return False, (
                'unique_gid_count=1 raw_count={} node={} expected_node={}'.format(
                    len(publisher_infos), node_name, expected_node_name,
                )
            )
        if namespace != expected_namespace:
            return False, (
                'unique_gid_count=1 raw_count={} node={} namespace={}'
                ' expected_namespace={}'.format(
                    len(publisher_infos), node_name,
                    repr(namespace), repr(expected_namespace),
                )
            )

        return True, (
            'unique_gid_count=1 raw_count={} node={} namespace={}'.format(
                len(publisher_infos), node_name, repr(namespace),
            )
        )

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

    @staticmethod
    def _udp_listener_count(port):
        """读取 Linux UDP 表确认正式端点仍在，不能把 ROS node 名当 socket 存活。"""
        try:
            with open('/proc/net/udp', 'r', encoding='utf-8') as stream:
                rows = stream.readlines()[1:]
        except OSError:
            return 0
        expected_port = '{:04X}'.format(int(port))
        count = 0
        for row in rows:
            columns = row.split()
            if len(columns) < 2 or ':' not in columns[1]:
                continue
            _address, local_port = columns[1].rsplit(':', 1)
            if local_port.upper() == expected_port:
                count += 1
        return count

    def _file_is_executable(self, path):
        """检查可执行文件：绝对路径直接用 os.access；相对命令名用 shutil.which。"""
        if not path:
            return False
        resolved = shutil.which(path) if not os.path.isabs(path) else path
        if not resolved:
            return False
        return os.path.isfile(resolved) and os.access(resolved, os.X_OK)

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

    def _profile_skipped_check(self, label):
        """显式标注 profile 豁免，绝不把刻意不存在伪装成实际 PASS。"""
        return ReadinessCheck(
            label, True, skipped_by_profile_detail(self.readiness_profile),
            critical=False,
        )

    def _gait_lock_profile_status_ok(self, payload):
        """核对 arbiter profile 和 required-set，拒绝假 seen/value 放行。"""
        if not isinstance(payload, dict):
            return False
        if (
            payload.get('profile') != self.readiness_profile
            or payload.get('start_mission_nodes') != self.start_mission_nodes
            or payload.get('profile_graph_consistent') is not True
            or payload.get('global_lock') is not False
        ):
            return False
        sources = {
            item.get('topic'): item
            for item in payload.get('sources', [])
            if isinstance(item, dict) and item.get('topic')
        }
        gait = sources.get('/gait/control_lock_req/gait', {})
        if not (
            gait.get('required') is True
            and gait.get('seen') is True
            and gait.get('fresh') is True
            and gait.get('value') is False
        ):
            return False
        inspection = sources.get('/gait/control_lock_req/inspection', {})
        global_owner = sources.get(
            '/gait/control_lock_req/global_gait_owner', {}
        )
        if not (
            global_owner.get('required') is True
            and global_owner.get('seen') is True
            and global_owner.get('fresh') is True
            and global_owner.get('value') is False
        ):
            return False
        if self._isolated_line_validation:
            return (
                inspection.get('required') is False
                and inspection.get('reason')
                == skipped_by_profile_detail(self.readiness_profile)
            )
        return (
            inspection.get('required') is True
            and inspection.get('seen') is True
            and inspection.get('fresh') is True
            and inspection.get('value') is False
        )

    @staticmethod
    def _global_gait_mode_ready(payload, software_smoke_mode):
        """只接受经典就绪且已释放 movement lock 的权威 owner 状态。"""
        expected_source = (
            'software_smoke' if software_smoke_mode else 'command_ack'
        )
        return (
            isinstance(payload, dict)
            and payload.get('state') == 'CLASSIC_READY'
            and payload.get('target') == 'CLASSIC'
            and payload.get('verification_source') == expected_source
            and payload.get('movement_lock_held') is False
        )

    @property
    def _isolated_line_validation(self):
        return self.readiness_profile == ISOLATED_LINE_VALIDATION_PROFILE

    def evaluate(self):
        """执行一次完整的只读检查，返回可序列化的检查列表。"""
        checks = []
        # profile 与 launch graph 必须成对出现，阻止比赛现场误用 validation
        # profile 或带着关闭 mission nodes 的 production 启动。
        profile_ok, profile_detail = readiness_profile_graph_check(
            self.readiness_profile, self.start_mission_nodes,
        )
        checks.append(ReadinessCheck(
            'readiness_profile_graph_consistency', profile_ok, profile_detail,
        ))
        # cmd_vel：按 GID 去重，必须恰好一个唯一发布者且为 command_mux_node。
        publisher_infos = self.get_publishers_info_by_topic(
            self.final_cmd_topic
        )
        cmd_ok, cmd_detail = self._single_gid_publisher_gate(
            publisher_infos, 'command_mux_node',
        )
        checks.append(ReadinessCheck(
            'final_cmd_single_command_mux_publisher',
            cmd_ok, cmd_detail,
        ))
        # /gait/control_lock：按 GID 去重，必须恰好一个唯一发布者且为
        # gait_lock_arbiter_node。
        lock_publisher_infos = self.get_publishers_info_by_topic(
            '/gait/control_lock'
        )
        lock_ok, lock_detail = self._single_gid_publisher_gate(
            lock_publisher_infos, 'gait_lock_arbiter_node',
        )
        checks.append(ReadinessCheck(
            'gait_control_lock_single_arbiter_publisher',
            lock_ok, lock_detail,
        ))
        gait_lock, gait_lock_age = self._fresh_value('gait_control_lock')
        checks.append(ReadinessCheck(
            'gait_control_lock_fresh_and_false',
            gait_lock is False,
            'value={}, age={}'.format(
                gait_lock,
                'missing' if gait_lock_age is None else '{:.3f}s'.format(
                    gait_lock_age
                ),
            ),
        ))
        lock_status, lock_status_age = self._fresh_value(
            'gait_control_lock_status'
        )
        lock_status_ok = self._gait_lock_profile_status_ok(lock_status)
        checks.append(ReadinessCheck(
            'gait_control_lock_profile_status',
            lock_status_ok,
            'profile={}, graph={}, lock={}, age={}'.format(
                (
                    lock_status.get('profile', 'missing')
                    if isinstance(lock_status, dict) else 'missing'
                ),
                (
                    lock_status.get('profile_graph_consistent', 'missing')
                    if isinstance(lock_status, dict) else 'missing'
                ),
                (
                    lock_status.get('global_lock', 'missing')
                    if isinstance(lock_status, dict) else 'missing'
                ),
                (
                    'missing' if lock_status_age is None
                    else '{:.3f}s'.format(lock_status_age)
                ),
            ),
        ))
        if profile_skips_check(
            self.readiness_profile, 'execute_motion_action_server'
        ):
            checks.append(self._profile_skipped_check(
                'execute_motion_action_server'
            ))
        else:
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
        gait_owner_present = any(
            name.endswith('/global_gait_owner') for name in node_names
        )
        gait_mode_status, gait_mode_age = self._fresh_value(
            'gait_mode_status'
        )
        gait_mode_ready = (
            gait_owner_present
            and self._global_gait_mode_ready(
                gait_mode_status, self.software_smoke_mode
            )
        )
        checks.append(ReadinessCheck(
            'global_gait_owner_classic_ready',
            gait_mode_ready,
            'present={}, state={}, target={}, verification_source={}, '
            'lock={}, age={}'.format(
                gait_owner_present,
                gait_mode_status.get('state', 'missing')
                if isinstance(gait_mode_status, dict) else 'missing',
                gait_mode_status.get('target', 'missing')
                if isinstance(gait_mode_status, dict) else 'missing',
                gait_mode_status.get('verification_source', 'missing')
                if isinstance(gait_mode_status, dict) else 'missing',
                gait_mode_status.get('movement_lock_held', 'missing')
                if isinstance(gait_mode_status, dict) else 'missing',
                'missing' if gait_mode_age is None else '{:.3f}s'.format(
                    gait_mode_age
                ),
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
        # A. USB 巡线相机：正式非机械臂比赛必须可用。
        line_image_value, line_image_age = self._fresh_value('image')
        line_camera_ready = (
            self._has_publisher(self.line_image_topic)
            and line_image_value is not None
        )
        checks.append(ReadinessCheck(
            'LINE_CAMERA_READY',
            line_camera_ready,
            'topic={}, age={}'.format(
                self.line_image_topic,
                'missing' if line_image_age is None else '{:.3f}s'.format(
                    line_image_age
                ),
            ),
        ))

        # B. Go2 本体前置相机：标识和警示牌识别的唯一来源。
        sign_image_value, sign_image_age = self._fresh_value('sign_camera_image')

        # B2. 生产必须观察真实 Go2 桥接节点；software smoke 使用合成 Image
        # 输入且禁止启动该硬件节点，因此只在 smoke 放行“桥接存在”这一项。
        sign_bridge_nodes = [
            name for name in self._node_names()
            if name.endswith('/' + SIGN_CAMERA_BRIDGE_NODE)
        ]
        sign_bridge_ready = (
            self.software_smoke_mode or bool(sign_bridge_nodes)
        )
        sign_frame_ok = (
            sign_image_value is not None
            and str(sign_image_value) == str(self.sign_camera_frame_id)
        )
        checks.append(ReadinessCheck(
            'GO2_CAMERA_READY',
            self._has_publisher(self.sign_image_topic)
            and sign_image_value is not None
            and sign_bridge_ready
            and sign_frame_ok,
            'topic={} bridge={} expected_frame={} actual_frame={} age={}'.format(
                self.sign_image_topic,
                'software_smoke_synthetic' if self.software_smoke_mode else (
                    'found' if sign_bridge_nodes else 'missing'
                ),
                self.sign_camera_frame_id,
                sign_image_value if sign_image_value is not None else 'missing',
                'missing' if sign_image_age is None else '{:.3f}s'.format(
                    sign_image_age
                ),
            ),
        ))

        # C. 机械臂 D435i：完整国赛才把该项提升为硬性 readiness 条件。
        arm_topics = (
            self.arm_color_topic,
            self.arm_depth_topic,
            self.arm_camera_info_topic,
        )
        arm_camera_ready = all(self._has_publisher(topic) for topic in arm_topics)
        checks.append(ReadinessCheck(
            'ARM_CAMERA_READY',
            arm_camera_ready,
            'required={}, topics={}'.format(
                self.require_arm_camera, ','.join(arm_topics)
            ),
            critical=self.require_arm_camera,
        ))

        # 三路相机命名必须互斥，防止未来启用机械臂 RGB-D 时覆盖现有图像源。
        if self.hardware_mode:
            camera_topics = (
                self.line_image_topic,
                self.sign_image_topic,
                self.arm_color_topic,
                self.arm_depth_topic,
                self.arm_camera_info_topic,
            )
            topics_different = len({str(topic) for topic in camera_topics}) == len(
                camera_topics
            )
            checks.append(ReadinessCheck(
                'camera_topics_distinct',
                topics_different,
                ','.join(str(topic) for topic in camera_topics),
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
        if profile_skips_check(
            self.readiness_profile, 'line_course_state_available'
        ):
            for label in (
                'line_course_state_available',
                'white_stage_publisher_status_available',
                'white_bar_action_idle',
                'inspection_action_idle',
            ):
                checks.append(self._profile_skipped_check(label))
        else:
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
        if profile_skips_check(
            self.readiness_profile, 'route_wait_start_without_active_action'
        ):
            checks.append(self._profile_skipped_check(
                'route_wait_start_without_active_action'
            ))
        else:
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
                or name.endswith('/line_camera_node')
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
            command_listener_count = self._udp_listener_count(
                self.sdk_command_port
            )
            status_listener_count = self._udp_listener_count(
                self.sdk_status_port
            )
            checks.append(ReadinessCheck(
                'hardware_sdk_server_ready',
                server_ok and server_running and command_listener_count == 1,
                'path_ok={}, process_running={}, command_port={} '
                'listener_count={}'.format(
                    server_ok, server_running, self.sdk_command_port,
                    command_listener_count,
                ),
            ))
            checks.append(ReadinessCheck(
                'hardware_udp_forwarder_started',
                forwarder_running and status_listener_count == 1,
                'node_present={}, status_port={} listener_count={}'.format(
                    forwarder_running, self.sdk_status_port,
                    status_listener_count,
                ),
            ))
            sdk_status_ok, sdk_status_detail = self._sdk_status_ready(
                self._sdk_startup_status, self._sdk_classic_verified_status,
            )
            checks.append(ReadinessCheck(
                'SDK_MOTION_BACKEND_READY',
                sdk_status_ok,
                sdk_status_detail,
            ))
        else:
            checks.append(ReadinessCheck(
                'mode_configuration',
                False,
                'hardware_mode=false requires software_smoke_mode=true',
            ))
        return checks

    def _valid_sdk_status_identity(self, status):
        """验证启动资格的不可伪造 identity 字段，不依据接收时间延长寿命。"""
        receive_ns = status.get('receive_monotonic_ns')
        sequence = status.get('sequence')
        return (
            isinstance(sequence, int)
            and not isinstance(sequence, bool)
            and sequence >= 1
            and isinstance(receive_ns, int)
            and not isinstance(receive_ns, bool)
            and receive_ns >= 1
        )

    def _sdk_status_ready(self, startup_status, classic_verified_status):
        """仅当前实例启动停车与随后经典步态 ACK 成功时允许动态控制。"""
        if not self.sdk_server_instance_id:
            return False, 'expected_server_instance_id_empty'
        sdk_error = getattr(self, '_sdk_error_status', None)
        if (
            isinstance(sdk_error, dict)
            and sdk_error.get('server_instance_id')
            == self.sdk_server_instance_id
        ):
            return False, 'current_instance_sdk_error_sequence={}'.format(
                sdk_error.get('sequence', 'unknown')
            )
        if not isinstance(startup_status, dict):
            return False, 'missing_or_invalid_startup_stop_status'
        if (
            startup_status.get('server_instance_id') != self.sdk_server_instance_id
            or startup_status.get('event') != 'STARTUP_STOP'
            or startup_status.get('ret') != 0
            or not self._valid_sdk_status_identity(startup_status)
        ):
            return False, 'startup_stop_instance_event_ret_or_sequence_mismatch'
        if not isinstance(classic_verified_status, dict):
            return False, 'missing_or_invalid_classic_verified_status'
        if (
            classic_verified_status.get('server_instance_id')
            != self.sdk_server_instance_id
            or classic_verified_status.get('event') != 'CLASSIC_VERIFIED'
            or classic_verified_status.get('ret') != 0
            or not self._valid_sdk_status_identity(classic_verified_status)
            or classic_verified_status.get('sequence', 0)
            <= startup_status.get('sequence', 0)
        ):
            return False, 'classic_verified_instance_event_ret_or_sequence_mismatch'
        return True, (
            'instance={} sdk_classic_and_startup_qualified '
            'classic_verified_sequence={} startup_sequence={} '
            'idle_event_age_not_a_liveness_failure'.format(
                self.sdk_server_instance_id,
                classic_verified_status['sequence'],
                startup_status['sequence'],
            )
        )

    def _make_payload(self):
        checks = self.evaluate()
        success = all(check.ok for check in checks if check.critical)
        return {
            'success': success,
            'mode': (
                'SOFTWARE_SMOKE_MODE'
                if self.software_smoke_mode else 'HARDWARE_MODE'
            ),
            'profile': self.readiness_profile,
            'start_mission_nodes': bool(self.start_mission_nodes),
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
