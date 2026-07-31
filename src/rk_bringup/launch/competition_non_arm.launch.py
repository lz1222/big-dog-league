#!/usr/bin/env python3

"""正式非机械臂比赛启动入口。

生产模式只包含巡线、白横线、警示牌、步态、mux 与 UDP 后端。机械臂、
避障、楼梯和 mock 均不在本 launch 中。software_smoke_mode 会强制切断
相机/UDP/SDK 硬件后端，并以显式测试 ELF helper 走真实 gait Action 流程。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare

from rk_bringup.non_arm_competition_contract import DEFAULT_IMAGE_TOPIC


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


def _selected_helper_expression(
    hardware_mode,
    software_smoke_mode,
    fake_helper,
    production_helper,
):
    """选择 helper：非硬件或 smoke 路径永远不用 Unitree SDK helper。"""
    expression = ['(']
    expression.extend(_truthy_expression(hardware_mode))
    expression.extend([' and not '])
    expression.extend(_truthy_expression(software_smoke_mode))
    expression.extend([') and '])
    expression.extend(["'", production_helper, "' or '", fake_helper, "'"])
    return PythonExpression(expression)


def generate_launch_description():
    """组装单一最终 cmd_vel 发布者的正式非机械臂 ROS 图。"""
    hardware_mode = LaunchConfiguration('hardware_mode')
    software_smoke_mode = LaunchConfiguration('software_smoke_mode')
    start_realsense = LaunchConfiguration('start_realsense')
    start_sdk_server = LaunchConfiguration('start_sdk_server')
    start_udp_forwarder = LaunchConfiguration('start_udp_forwarder')
    enable_debug_image = LaunchConfiguration('enable_debug_image')
    sdk_network_interface = LaunchConfiguration('sdk_network_interface')
    image_topic = LaunchConfiguration('image_topic')
    sdk_server = LaunchConfiguration('sdk_server')
    sdk_udp_host = LaunchConfiguration('sdk_udp_host')
    sdk_udp_port = LaunchConfiguration('sdk_udp_port')
    rgb_camera_profile = LaunchConfiguration('rgb_camera.profile')
    depth_module_profile = LaunchConfiguration('depth_module.profile')
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
    installed_sdk_helper = PathJoinSubstitution([
        FindPackagePrefix('rk_go2_sdk_bridge'),
        'lib', 'rk_go2_sdk_bridge', 'go2_sdk_motion_action',
    ])
    selected_sdk_helper = _selected_helper_expression(
        hardware_mode,
        software_smoke_mode,
        fake_sdk_action_executable,
        installed_sdk_helper,
    )
    selected_cleanup_guard = _selected_helper_expression(
        hardware_mode,
        software_smoke_mode,
        smoke_cleanup_guard_path,
        cleanup_guard_path,
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
    use_hardware_realsense = IfCondition(_hardware_backend_expression(
        hardware_mode, software_smoke_mode, start_realsense
    ))
    use_hardware_sdk_server = IfCondition(_hardware_backend_expression(
        hardware_mode, software_smoke_mode, start_sdk_server
    ))
    use_hardware_udp_forwarder = IfCondition(_hardware_backend_expression(
        hardware_mode, software_smoke_mode, start_udp_forwarder
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
                'Force SOFTWARE_SMOKE_MODE: no RealSense, UDP server, '
                'UDP forwarder or Unitree SDK helper.'
            ),
        ),
        DeclareLaunchArgument(
            'start_realsense', default_value='true',
            description='Start realsense2_camera_node in hardware mode.',
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
            'image_topic', default_value=DEFAULT_IMAGE_TOPIC,
            description=(
                'Shared RGB topic for tracker and sign detector; e.g. '
                '/camera/color/image_raw on an alternate RealSense graph.'
            ),
        ),
        DeclareLaunchArgument(
            'sdk_server',
            default_value=(
                '/home/unitree/unitree_go2_sdk_test/build/'
                'go2_sdk_udp_server'
            ),
            description='Production Go2 UDP server executable.',
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
            'rgb_camera.profile', default_value='424x240x15',
            description='RealSense RGB profile used in hardware mode.',
        ),
        DeclareLaunchArgument(
            'depth_module.profile', default_value='424x240x15',
            description='Kept for camera compatibility; depth stays disabled.',
        ),
        DeclareLaunchArgument(
            'fake_sdk_action_executable', default_value='',
            description=(
                'Required only by SOFTWARE_SMOKE_MODE. Empty is '
                'fail-closed; acceptance supplies a marked test-only ELF.'
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
            default_value='/tmp/rk_non_arm_competition_smoke_guard.json',
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
                '*** SOFTWARE_SMOKE_MODE: RealSense, UDP server, UDP '
                'forwarder and Unitree SDK are forcibly disabled. ***'
            ),
            condition=use_smoke_publisher,
        ),
        # 生产模式才从 D435i 收图；深度/scan 避障均不接入本比赛范围。
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            namespace='camera',
            name='camera',
            output='log',
            condition=use_hardware_realsense,
            parameters=[{
                'enable_color': True,
                'enable_depth': False,
                'enable_gyro': False,
                'enable_accel': False,
                'rgb_camera.profile': rgb_camera_profile,
                'depth_module.profile': depth_module_profile,
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
                'image_topic': image_topic,
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
                'image_topic': image_topic,
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
                'front_jump.sdk_network_interface': sdk_network_interface,
                'front_jump.sdk_action_executable': selected_sdk_helper,
                'front_jump.cleanup_guard_path': selected_cleanup_guard,
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
                'sdk_network_interface': sdk_network_interface,
                'sdk_action_executable': selected_sdk_helper,
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
                'image_topic': image_topic,
                'sdk_server': sdk_server,
                'sdk_action_executable': selected_sdk_helper,
                'cleanup_guard_path': selected_cleanup_guard,
            }],
        ),
        # SDK server/forwarder 仅在真实硬件模式存在，软件测试没有网络出口。
        ExecuteProcess(
            cmd=[sdk_server], output='log', condition=use_hardware_sdk_server,
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
                'image_topic': image_topic,
                'scenario': smoke_scenario,
                'auto_start': ParameterValue(
                    smoke_auto_start, value_type=bool
                ),
            }],
        ),
    ])
