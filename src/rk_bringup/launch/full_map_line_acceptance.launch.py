#!/usr/bin/env python3

"""全图真机巡线专项验收入口：不加载比赛任务和任何实体动作节点."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """构造 line-only 链路；速度必须先经过 ARM 心跳门和 command_mux."""
    allowed_segment_id = LaunchConfiguration('allowed_segment_id')
    start_sdk_server = LaunchConfiguration('start_sdk_server')
    sdk_network_interface = LaunchConfiguration('sdk_network_interface')
    bridge_max_angular_z = LaunchConfiguration('bridge_max_angular_z')
    line_speed = LaunchConfiguration('line_speed')

    bringup_share = FindPackageShare('rk_bringup')
    sdk_bridge_share = FindPackageShare('rk_go2_sdk_bridge')
    line_nav_config = PathJoinSubstitution([
        bringup_share, 'config', 'line_nav_params.yaml',
    ])
    acceptance_config = PathJoinSubstitution([
        bringup_share, 'config', 'full_map_line_acceptance_params.yaml',
    ])
    line_camera_config = PathJoinSubstitution([
        FindPackageShare('rk_perception'), 'config', 'line_camera.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'allowed_segment_id', default_value='UNSET',
            description=(
                'Only this exact SEGMENT_READY <id> command can ARM; '
                'keep UNSET during phase-A manual collection.'
            ),
        ),
        DeclareLaunchArgument(
            'line_speed', default_value='0.05',
            description='Phase-B/C maximum forward speed in m/s.',
        ),
        DeclareLaunchArgument(
            'bridge_max_angular_z', default_value='0.28',
            description=(
                'Do not exceed the current line follower yaw safety cap.'
            ),
        ),
        DeclareLaunchArgument(
            'start_sdk_server', default_value='true',
            description=(
                'Start only the repository-installed SDK velocity server.'
            ),
        ),
        DeclareLaunchArgument(
            'sdk_network_interface', default_value='eth0',
            description='Unitree SDK2 network interface.',
        ),
        LogInfo(
            msg=(
                'Full-map Sonix line acceptance mode: DISARMED by default. No '
                'mission, inspection, arm, gait, blink, stretch, hello, '
                'or jump node is started.'
            ),
        ),
        Node(
            # Sonix UVC 节点只发布巡线图像，不打开 D435i 或任何任务相机。
            package='rk_perception', executable='usb_line_camera_node',
            name='usb_line_camera_node', output='screen',
            parameters=[line_camera_config],
        ),
        Node(
            package='rk_perception', executable='real_line_tracker_node',
            name='real_line_tracker_node', output='screen',
            # 专项覆盖层使用精确节点名，才能压过基础 YAML 的同名节点段。
            parameters=[line_nav_config, acceptance_config],
        ),
        Node(
            package='rk_bringup', executable='line_acceptance_guard_node',
            name='line_acceptance_guard_node', output='screen',
            parameters=[{'allowed_segment_id': allowed_segment_id}],
        ),
        Node(
            package='rk_navigation', executable='line_follower_node',
            name='line_follower_node', output='screen',
            parameters=[line_nav_config, acceptance_config],
        ),
        Node(
            package='rk_bringup', executable='line_acceptance_cmd_gate_node',
            name='line_acceptance_cmd_gate_node', output='screen',
            parameters=[{
                'input_cmd_topic': '/line_acceptance/line_cmd_suggested',
                'output_cmd_topic': '/control/line_cmd',
                'arm_topic': '/line_acceptance/arm',
                'candidate_timeout_sec': 0.25,
                'arm_timeout_sec': 0.30,
            }],
        ),
        Node(
            package='rk_safety', executable='command_mux_node',
            name='command_mux_node', output='screen',
            parameters=[{
                'line_cmd_topic': '/control/line_cmd',
                # 未启动任务节点，两个其他来源保持无发布者且仲裁仍可审计。
                'mission_cmd_topic': '/line_acceptance/disabled_mission_cmd',
                'locomotion_cmd_topic': (
                    '/line_acceptance/disabled_locomotion_cmd'
                ),
                'estop_topic': '/safety/estop',
                'estop_state_topic': '/safety/estop_state',
                'enable_estop_service': True,
                'estop_service_name': '/safety/estop',
                'gait_lock_topic': '/line_acceptance/disabled_gait_lock',
                'arm_lock_topic': '/line_acceptance/disabled_arm_lock',
                'output_cmd_topic': '/navigation/cmd_vel',
                'status_topic': '/control/cmd_mux_status',
                'line_cmd_timeout_sec': 0.20,
                'mission_cmd_timeout_sec': 0.20,
                'locomotion_cmd_timeout_sec': 0.20,
                'max_linear_x': ParameterValue(line_speed, value_type=float),
                'max_linear_y': 0.01,
                'max_angular_z': ParameterValue(
                    bridge_max_angular_z, value_type=float
                ),
            }],
        ),
        # 复用已安装的 Go2 桥接入口，避免误用主工作区或外部旧 build。
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                sdk_bridge_share, 'launch', 'go2_sdk_udp_bridge.launch.py',
            ])),
            launch_arguments={
                'start_sdk_server': start_sdk_server,
                'sdk_network_interface': sdk_network_interface,
                'cmd_vel_topic': '/navigation/cmd_vel',
                'max_vx': line_speed,
                'max_vy': '0.01',
                'max_yaw': bridge_max_angular_z,
                'timeout_sec': '0.20',
            }.items(),
        ),
    ])
