#!/usr/bin/env python3

"""正式非机械臂比赛启动入口。

生产模式只包含 USB 巡线、Go2 本体相机标识识别、白横线、警示牌、步态、mux
与 UDP 后端。机械臂 D435i、避障、楼梯和 mock 均不在本 launch 中。
software_smoke_mode 会强制切断相机/UDP/SDK 硬件后端。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.conditions import IfCondition
from launch.substitution import Substitution
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare

from rk_bringup.non_arm_competition_contract import DEFAULT_IMAGE_TOPIC
from rk_bringup.inspection_helper_path import select_sdk_action_helper


def _truthy_expression(configuration):
    """构造统一的 launch 布尔文本判断，兼容现场命令行 true/false。"""
    return [
        "('", configuration,
        "'.strip().lower() in ('1','true','yes','on'))",
    ]


def _hardware_backend_expression(
    hardware_mode,
    software_smoke_mode,
    optional_start=None,
):
    """硬件进程必须同时满足 hardware_mode 且未处于 smoke 模式。"""
    expression = ['(']
    expression.extend(_truthy_expression(hardware_mode))
    expression.extend([' and not '])
    expression.extend(_truthy_expression(software_smoke_mode))
    if optional_start is not None:
        expression.extend([' and '])
        expression.extend(_truthy_expression(optional_start))
    expression.extend([')'])
    return PythonExpression(expression)


def _not_smoke_expression(software_smoke_mode):
    """软件 smoke 使用合成输入，避免真实感知输出与测试消息竞争。"""
    expression = ['(not ']
    expression.extend(_truthy_expression(software_smoke_mode))
    expression.extend([')'])
    return PythonExpression(expression)


def _smoke_only_timeout_expression(
    software_smoke_mode,
    smoke_timeout,
    production_timeout,
):
    """仅在无硬件 smoke 下选择较宽观察窗，生产安全阈值保持原值。"""
    expression = ['(', str(smoke_timeout), ' if ']
    expression.extend(_truthy_expression(software_smoke_mode))
    expression.extend([' else ', str(production_timeout), ')'])
    return PythonExpression(expression)


class _SelectedSdkActionHelper(Substitution):
    """延迟选择并验证 inspection/gait SDK helper，避免 smoke 解析真实路径。"""

    def __init__(
        self, hardware_mode, software_smoke_mode, fake_helper, package_prefix
    ):
        super().__init__()
        self._hardware_mode = hardware_mode
        self._software_smoke_mode = software_smoke_mode
        self._fake_helper = fake_helper
        self._package_prefix = package_prefix

    def describe(self):
        return 'selected inspection SDK action helper'

    def perform(self, context):
        """仅 production 分支从安装树解析；异常会阻止节点获得不安全参数。"""
        return select_sdk_action_helper(
            self._hardware_mode.perform(context),
            self._software_smoke_mode.perform(context),
            self._fake_helper.perform(context),
            self._package_prefix.perform(context),
        )


def _selected_helper_expression(
    hardware_mode,
    software_smoke_mode,
    fake_helper,
    production_value='',
):
    """选择 helper：非硬件或 smoke 路径永远不用 Unitree SDK helper。

    此函数仅供 cleanup guard 等纯字符串参数使用。SDK helper 使用
    :class:`_SelectedSdkActionHelper`，保证 production 不会退化为 basename。
    """
    expression = [
        "'", fake_helper, "' if not (",
        "('", hardware_mode, "'.strip().lower() in ('1','true','yes','on'))",
        " and not ",
        "('", software_smoke_mode, "'.strip().lower() in ('1','true','yes','on'))",
        ") else '", production_value, "'",
    ]
    return PythonExpression(expression)


def generate_launch_description():
    """组装单一最终 cmd_vel 发布者的正式非机械臂 ROS 图。"""
    hardware_mode = LaunchConfiguration('hardware_mode')
    software_smoke_mode = LaunchConfiguration('software_smoke_mode')
    start_line_camera = LaunchConfiguration('start_line_camera')
    start_sdk_server = LaunchConfiguration('start_sdk_server')
    start_udp_forwarder = LaunchConfiguration('start_udp_forwarder')
    start_go2_front_camera = LaunchConfiguration('start_go2_front_camera')
    enable_debug_image = LaunchConfiguration('enable_debug_image')
    sdk_network_interface = LaunchConfiguration('sdk_network_interface')
    sdk_server_runtime = LaunchConfiguration('sdk_server_runtime')
    stream_helper = LaunchConfiguration('stream_helper')
    line_image_topic = LaunchConfiguration('line_image_topic')
    sign_image_topic = LaunchConfiguration('sign_image_topic')
    go2_front_camera_frame_id = LaunchConfiguration('go2_front_camera_frame_id')
    sdk_server = LaunchConfiguration('sdk_server')
    sdk_udp_host = LaunchConfiguration('sdk_udp_host')
    sdk_udp_port = LaunchConfiguration('sdk_udp_port')
    line_camera_device = LaunchConfiguration('line_camera_device')
    line_camera_width = LaunchConfiguration('line_camera_width')
    line_camera_height = LaunchConfiguration('line_camera_height')
    line_camera_fps = LaunchConfiguration('line_camera_fps')
    fake_sdk_action_executable = LaunchConfiguration(
        'fake_sdk_action_executable'
    )
    cleanup_guard_path = LaunchConfiguration('cleanup_guard_path')
    smoke_cleanup_guard_path = LaunchConfiguration(
        'smoke_cleanup_guard_path'
    )
    smoke_scenario = LaunchConfiguration('smoke_scenario')
    smoke_auto_start = LaunchConfiguration('smoke_auto_start')

    formal_config = PathJoinSubstitution([
        FindPackageShare('rk_bringup'),
        'config',
        'non_arm_competition_params.yaml',
    ])
    gait_config = PathJoinSubstitution([
        FindPackageShare('rk_locomotion'), 'config', 'gait_params.yaml',
    ])
    selected_sdk_helper = _SelectedSdkActionHelper(
        hardware_mode,
        software_smoke_mode,
        fake_sdk_action_executable,
        FindPackagePrefix('rk_go2_sdk_bridge'),
    )
    selected_cleanup_guard = _selected_helper_expression(
        hardware_mode,
        software_smoke_mode,
        smoke_cleanup_guard_path,
        production_value=cleanup_guard_path,
    )
    # smoke helper 不接触 SDK/网络/实体机器人，但高负载 VM 可能让独立的
    # estop 心跳调度抖动超过生产 0.20 秒。仅 smoke 将其观察窗限为 8 秒；
    # 生产模式仍严格使用 0.20 秒，且 smoke 启动前也必须看到新鲜 false。
    smoke_estop_stale_timeout = ParameterValue(
        _smoke_only_timeout_expression(
            software_smoke_mode,
            smoke_timeout=8.0,
            production_timeout=0.20,
        ),
        value_type=float,
    )
    use_hardware_line_camera = IfCondition(_hardware_backend_expression(
        hardware_mode, software_smoke_mode, start_line_camera
    ))
    use_hardware_sdk_server = IfCondition(_hardware_backend_expression(
        hardware_mode, software_smoke_mode, start_sdk_server
    ))
    use_hardware_udp_forwarder = IfCondition(_hardware_backend_expression(
        hardware_mode, software_smoke_mode, start_udp_forwarder
    ))
    use_hardware_front_camera = IfCondition(_hardware_backend_expression(
        hardware_mode, software_smoke_mode, start_go2_front_camera
    ))
    use_real_perception = IfCondition(_not_smoke_expression(
        software_smoke_mode
    ))
    use_smoke_publisher = IfCondition(PythonExpression(
        _truthy_expression(software_smoke_mode)
    ))

    return LaunchDescription([
        DeclareLaunchArgument(
            'hardware_mode', default_value='true',
            description=(
                'Allow production camera/UDP/SDK backend. false requires '
                'software_smoke_mode=true and never starts hardware I/O.'
            ),
        ),
        DeclareLaunchArgument(
            'software_smoke_mode', default_value='false',
            description=(
                'Force SOFTWARE_SMOKE_MODE: no USB/Go2 camera, UDP server, '
                'UDP forwarder or Unitree SDK helper.'
            ),
        ),
        DeclareLaunchArgument(
            'start_line_camera', default_value='true',
            description='Start the only USB line-camera node in hardware mode.',
        ),
        DeclareLaunchArgument(
            'start_sdk_server', default_value='true',
            description='Start go2_sdk_udp_server in hardware mode.',
        ),
        DeclareLaunchArgument(
            'start_udp_forwarder', default_value='true',
            description='Start cmd_vel_udp_forwarder in hardware mode.',
        ),
        DeclareLaunchArgument(
            'enable_debug_image', default_value='false',
            description='Enable debug images only when explicitly requested.',
        ),
        DeclareLaunchArgument(
            'sdk_network_interface', default_value='eth0',
            description='Unitree SDK interface in hardware mode.',
        ),
        DeclareLaunchArgument(
            'line_image_topic', default_value=DEFAULT_IMAGE_TOPIC,
            description='Fixed USB line-camera topic for real_line_tracker_node.',
        ),
        DeclareLaunchArgument(
            'line_camera_device',
            default_value=(
                '/dev/v4l/by-id/'
                'usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera_SN0001-video-index0'
            ),
            description='Stable explicit /dev/v4l/by-id path for USB line camera.',
        ),
        DeclareLaunchArgument(
            'line_camera_width', default_value='640',
            description='Requested USB line-camera width in pixels.',
        ),
        DeclareLaunchArgument(
            'line_camera_height', default_value='480',
            description='Requested USB line-camera height in pixels.',
        ),
        DeclareLaunchArgument(
            'line_camera_fps', default_value='15.0',
            description='Requested USB line-camera frame rate.',
        ),
        DeclareLaunchArgument(
            'sign_image_topic', default_value='/go2/front_camera/image_raw',
            description='Go2 onboard front camera topic for sign detector.',
        ),
        DeclareLaunchArgument(
            'start_go2_front_camera', default_value='true',
            description='Start go2_front_camera_bridge_node in hardware mode.',
        ),
        DeclareLaunchArgument(
            'go2_front_camera_frame_id',
            default_value='go2_front_camera_optical_frame',
            description='frame_id for the Go2 front camera bridge output.',
        ),
        DeclareLaunchArgument(
            'stream_helper',
            default_value=[FindPackagePrefix('rk_go2_sdk_bridge'),
                           '/lib/rk_go2_sdk_bridge/',
                           'go2_front_camera_stream_helper'],
            description='Absolute path to the Go2 front camera stream helper.',
        ),
        DeclareLaunchArgument(
            'sdk_server',
            default_value=[FindPackagePrefix('rk_go2_sdk_bridge'),
                           '/lib/rk_go2_sdk_bridge/go2_sdk_udp_server'],
            description='Production Go2 UDP server executable.',
        ),
        DeclareLaunchArgument(
            'sdk_server_runtime',
            default_value=[FindPackagePrefix('rk_go2_sdk_bridge'),
                           '/lib/rk_go2_sdk_bridge/go2_sdk_server_runtime.py'],
            description=(
                'Installed wrapper that isolates the Unitree SDK DDS '
                'runtime before starting sdk_server.'
            ),
        ),
        DeclareLaunchArgument(
            'sdk_udp_host', default_value='127.0.0.1',
            description='Production SDK UDP host.',
        ),
        DeclareLaunchArgument(
            'sdk_udp_port', default_value='15001',
            description='Production SDK UDP port.',
        ),
        DeclareLaunchArgument(
            'fake_sdk_action_executable',
            default_value=[FindPackagePrefix('rk_go2_sdk_bridge'),
                           '/lib/rk_go2_sdk_bridge/fake_sdk_motion_helper'],
            description=(
                'Fake SDK helper for SOFTWARE_SMOKE_MODE. Contains the '
                'required identity marker so smoke acceptance can verify '
                'the helper without touching real hardware or network.'
            ),
        ),
        DeclareLaunchArgument(
            'cleanup_guard_path',
            default_value=(
                '~/.rk_non_arm_competition/front_jump_cleanup_guard.json'
            ),
            description='Persistent FrontJump cleanup journal path.',
        ),
        DeclareLaunchArgument(
            'smoke_cleanup_guard_path',
            default_value='/tmp/rk_non_arm_competition/smoke_guard.json',
            description=(
                'Smoke-only FrontJump cleanup journal, isolated from the '
                'production guard and normally overridden per acceptance run.'
            ),
        ),
        DeclareLaunchArgument(
            'smoke_scenario', default_value='idle',
            description='Test-only synthetic input scenario in smoke mode.',
        ),
        DeclareLaunchArgument(
            'smoke_auto_start', default_value='false',
            description='Test-only: synthetic publisher may issue one start.',
        ),
        LogInfo(
            msg=(
                'Starting formal non-arm competition chain. '
                '/navigation/cmd_vel is owned only by command_mux_node.'
            ),
        ),
        LogInfo(
            msg=(
                '*** SOFTWARE_SMOKE_MODE: USB/Go2 cameras, UDP server, UDP '
                'forwarder and Unitree SDK are forcibly disabled. ***'
            ),
            condition=use_smoke_publisher,
        ),
        # 正式巡线只使用明确指定的 USB UVC 设备；绝不枚举或回退到 D435i。
        Node(
            package='rk_bringup',
            executable='line_camera_node',
            name='line_camera_node',
            output='log',
            condition=use_hardware_line_camera,
            parameters=[{
                'device': ParameterValue(line_camera_device, value_type=str),
                'width': ParameterValue(line_camera_width, value_type=int),
                'height': ParameterValue(line_camera_height, value_type=int),
                'fps': ParameterValue(line_camera_fps, value_type=float),
            }],
        ),
        # smoke 由测试 publisher 给出确定性证据，防止真实检测输出干扰。
        Node(
            package='rk_perception',
            executable='real_line_tracker_node',
            name='real_line_tracker_node',
            output='log',
            condition=use_real_perception,
            parameters=[formal_config, {
                'image_topic': line_image_topic,
                'enable_debug_image': ParameterValue(
                    enable_debug_image, value_type=bool
                ),
                'debug_log': ParameterValue(
                    enable_debug_image, value_type=bool
                ),
            }],
        ),
        Node(
            package='rk_perception',
            executable='real_sign_detector_node',
            name='real_sign_detector_node',
            output='log',
            condition=use_real_perception,
            parameters=[formal_config, {
                'image_topic': sign_image_topic,
                'frame_id': go2_front_camera_frame_id,
                'enable_debug_image': ParameterValue(
                    enable_debug_image, value_type=bool
                ),
                'debug_log': ParameterValue(
                    enable_debug_image, value_type=bool
                ),
            }],
        ),
        Node(
            package='rk_navigation',
            executable='line_follower_node',
            name='line_follower_node',
            output='log',
            parameters=[formal_config],
        ),
        Node(
            package='rk_mission',
            executable='line_course_mission_node',
            name='line_course_mission_node',
            output='log',
            parameters=[formal_config, {
                'cmd_vel_topic': '/control/mission_cmd',
                'sdk_network_interface': sdk_network_interface,
            }],
        ),
        Node(
            package='rk_mission',
            executable='white_bar_stage_command_publisher',
            name='white_bar_stage_command_publisher',
            output='log',
            parameters=[formal_config],
        ),
        Node(
            package='rk_mission',
            executable='white_bar_action_executor',
            name='white_bar_action_executor',
            output='log',
            parameters=[formal_config],
        ),
        # 真实 Action server 保持唯一；smoke 只替换其 helper，不替换 server。
        Node(
            package='rk_locomotion',
            executable='gait_control_node',
            name='gait_control_node',
            output='log',
            parameters=[gait_config, formal_config, {
                'cmd_vel_topic': '/control/locomotion_cmd',
                'motion_action_name': '/locomotion/execute_motion',
                'enable_motion_action': True,
                'obstacle_safety.enable_depth': False,
                'obstacle_safety.enable_scan': False,
                'front_jump.final_cmd_topic': '/navigation/cmd_vel',
                'front_jump.cmd_mux_status_topic': '/control/cmd_mux_status',
                'front_jump.estop_state_topic': '/safety/estop_state',
                'front_jump.estop_state_stale_timeout': (
                    smoke_estop_stale_timeout
                ),
                'front_jump.sdk_network_interface': ParameterValue(
                    sdk_network_interface, value_type=str
                ),
                'front_jump.sdk_action_executable': ParameterValue(
                    selected_sdk_helper, value_type=str
                ),
                'front_jump.sdk_runtime_wrapper': ParameterValue(
                    sdk_server_runtime, value_type=str
                ),
                'front_jump.cleanup_guard_path': ParameterValue(
                    selected_cleanup_guard, value_type=str
                ),
                'front_jump.software_smoke_mode': ParameterValue(
                    software_smoke_mode, value_type=bool
                ),
            }],
        ),
        Node(
            package='rk_mission',
            executable='inspection_action_executor',
            name='inspection_action_executor',
            output='log',
            parameters=[formal_config, {
                'sdk_network_interface': ParameterValue(
                    sdk_network_interface, value_type=str
                ),
                'sdk_action_executable': ParameterValue(
                    selected_sdk_helper, value_type=str
                ),
                'sdk_runtime_wrapper': ParameterValue(
                    sdk_server_runtime, value_type=str
                ),
                'estop_state_stale_timeout_sec': smoke_estop_stale_timeout,
                'software_smoke_mode': ParameterValue(
                    software_smoke_mode, value_type=bool
                ),
            }],
        ),
        Node(
            package='rk_safety',
            executable='command_mux_node',
            name='command_mux_node',
            output='log',
            parameters=[formal_config, {
                'line_cmd_topic': '/control/line_cmd',
                'mission_cmd_topic': '/control/mission_cmd',
                'locomotion_cmd_topic': '/control/locomotion_cmd',
                'output_cmd_topic': '/navigation/cmd_vel',
                'status_topic': '/control/cmd_mux_status',
                'estop_service_name': '/safety/estop',
            }],
        ),
        # 单一锁仲裁节点将 gait/inspection 的独立锁请求做 OR 聚合，
        # 发布唯一权威的 /gait/control_lock，消除双发布者竞态。
        Node(
            package='rk_safety',
            executable='gait_lock_arbiter_node',
            name='gait_lock_arbiter_node',
            output='log',
            parameters=[{
                'input_topics': [
                    '/gait/control_lock_req/gait',
                    '/gait/control_lock_req/inspection',
                ],
                'output_topic': '/gait/control_lock',
                'source_timeout_sec': 2.0,
                'arbiter_rate_hz': 10.0,
            }],
        ),
        # Go2 本体前向相机 → sensor_msgs/Image 桥接。
        # 长期 stream helper 子进程通过管道写入 JPEG 帧，
        # 桥接节点解码后发布为 ROS Image 消息。
        Node(
            package='rk_go2_sdk_bridge',
            executable='go2_front_camera_bridge.py',
            name='go2_front_camera_bridge_node',
            output='log',
            condition=use_hardware_front_camera,
            parameters=[{
                'network_interface': ParameterValue(
                    sdk_network_interface, value_type=str
                ),
                'output_topic': sign_image_topic,
                'frame_id': go2_front_camera_frame_id,
                'stream_helper': ParameterValue(
                    stream_helper, value_type=str
                ),
                'sdk_runtime_wrapper': ParameterValue(
                    sdk_server_runtime, value_type=str
                ),
            }],
        ),
        Node(
            package='rk_bringup',
            executable='competition_readiness_node',
            name='competition_readiness_node',
            output='log',
            parameters=[formal_config, {
                'hardware_mode': ParameterValue(
                    hardware_mode, value_type=bool
                ),
                'software_smoke_mode': ParameterValue(
                    software_smoke_mode, value_type=bool
                ),
                'line_image_topic': line_image_topic,
                'sign_image_topic': sign_image_topic,
                'sign_camera_frame_id': go2_front_camera_frame_id,
                'sdk_server': PathJoinSubstitution([
                    FindPackagePrefix('rk_go2_sdk_bridge'),
                    'lib',
                    'rk_go2_sdk_bridge',
                    'go2_sdk_udp_server',
                ]),
                # readiness 与实际执行器必须看到同一 helper：smoke 时只能是
                # 带测试标识的 fake helper，生产时才解析安装树绝对路径。
                'sdk_action_executable': ParameterValue(
                    selected_sdk_helper, value_type=str
                ),
                'cleanup_guard_path': ParameterValue(
                    selected_cleanup_guard, value_type=str
                ),
            }],
        ),
        # SDK server/forwarder 仅在真实硬件模式存在，软件测试没有网络出口。
        # ROS Foxy 节点继续继承其自身环境；SDK server 经安装树 wrapper 启动，
        # 仅加载构建时确认的一对 Unitree CycloneDDS 库。
        ExecuteProcess(
            cmd=[sdk_server_runtime, sdk_server], output='log',
            condition=use_hardware_sdk_server,
        ),
        Node(
            package='rk_go2_sdk_bridge',
            executable='cmd_vel_udp_forwarder.py',
            name='cmd_vel_udp_forwarder',
            output='log',
            condition=use_hardware_udp_forwarder,
            parameters=[{
                'cmd_vel_topic': '/navigation/cmd_vel',
                'udp_host': sdk_udp_host,
                'udp_port': ParameterValue(sdk_udp_port, value_type=int),
                'max_vx': 0.30,
                'max_yaw': 0.80,
            }],
        ),
        # 这是测试专用合成输入，不是 mock locomotion Action Server。
        Node(
            package='rk_bringup',
            executable='non_arm_smoke_publisher',
            name='competition_smoke_publisher',
            output='log',
            condition=use_smoke_publisher,
            parameters=[{
                'line_image_topic': line_image_topic,
                'sign_image_topic': sign_image_topic,
                'scenario': smoke_scenario,
                'auto_start': ParameterValue(
                    smoke_auto_start, value_type=bool
                ),
            }],
        ),
    ])
