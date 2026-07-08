#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


ROUTE_NODE_ENV = {
    'ROS_DOMAIN_ID': '10',
}


def generate_launch_description():
    workspace_dir = LaunchConfiguration('workspace_dir')
    start_sdk_server = LaunchConfiguration('start_sdk_server')
    sdk_server = LaunchConfiguration('sdk_server')
    bridge_max_linear_x = LaunchConfiguration('bridge_max_linear_x')
    bridge_max_angular_z = LaunchConfiguration('bridge_max_angular_z')
    countdown_sec = LaunchConfiguration('countdown_sec')
    distance_scale = LaunchConfiguration('distance_scale')
    turn_scale = LaunchConfiguration('turn_scale')
    speed_scale = LaunchConfiguration('speed_scale')
    sdk_network_interface = LaunchConfiguration('sdk_network_interface')
    sdk_action_executable = LaunchConfiguration('sdk_action_executable')
    run_without_sdk_actions = LaunchConfiguration('run_without_sdk_actions')
    allow_ros_topic_sdk_actions = LaunchConfiguration(
        'allow_ros_topic_sdk_actions'
    )
    line_lost_switch_sec = LaunchConfiguration('line_lost_switch_sec')
    line_track_stale_sec = LaunchConfiguration('line_track_stale_sec')
    white_line_detection_enabled = LaunchConfiguration(
        'white_line_detection_enabled'
    )
    white_line_min_width_fraction = LaunchConfiguration(
        'white_line_min_width_fraction'
    )
    white_line_min_value = LaunchConfiguration('white_line_min_value')
    white_line_max_saturation = LaunchConfiguration(
        'white_line_max_saturation'
    )
    start_realsense = LaunchConfiguration('start_realsense')
    start_line_nodes = LaunchConfiguration('start_line_nodes')
    image_topic = LaunchConfiguration('image_topic')
    debug = LaunchConfiguration('debug')

    sdk_bridge_launch = PathJoinSubstitution([
        FindPackageShare('rk_go2_sdk_bridge'),
        'launch',
        'go2_sdk_udp_bridge.launch.py',
    ])
    line_nav_config = PathJoinSubstitution([
        FindPackageShare('rk_bringup'),
        'config',
        'line_nav_params.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'workspace_dir',
            default_value='/home/unitree/rk_inspection_ws',
            description='Robot workspace path.'
        ),
        DeclareLaunchArgument(
            'start_sdk_server',
            default_value='true',
            description='Start the existing Go2 SDK UDP server process.'
        ),
        DeclareLaunchArgument(
            'sdk_server',
            default_value=(
                '/home/unitree/unitree_go2_sdk_test/build/'
                'go2_sdk_udp_server'
            ),
            description='Path to the working Go2 SDK UDP server binary.'
        ),
        DeclareLaunchArgument(
            'bridge_max_linear_x',
            default_value='0.60',
            description='Maximum absolute vx passed through the UDP bridge.'
        ),
        DeclareLaunchArgument(
            'bridge_max_angular_z',
            default_value='1.00',
            description='Maximum absolute yaw speed passed through the bridge.'
        ),
        DeclareLaunchArgument(
            'countdown_sec',
            default_value='0.0',
            description='Zero-cmd countdown before the route starts.'
        ),
        DeclareLaunchArgument(
            'distance_scale',
            default_value='1.0',
            description='Scale all hardcoded forward distances.'
        ),
        DeclareLaunchArgument(
            'turn_scale',
            default_value='1.0',
            description='Scale all hardcoded turn angles.'
        ),
        DeclareLaunchArgument(
            'speed_scale',
            default_value='1.0',
            description='Scale all hardcoded linear and yaw speeds.'
        ),
        DeclareLaunchArgument(
            'sdk_network_interface',
            default_value='eth0',
            description='Go2 SDK2 network interface for FrontJump actions.'
        ),
        DeclareLaunchArgument(
            'sdk_action_executable',
            default_value=(
                '/home/unitree/rk_inspection_ws/install/'
                'rk_go2_sdk_bridge/lib/rk_go2_sdk_bridge/'
                'go2_sdk_motion_action'
            ),
            description='Helper executable used for SDK actions.'
        ),
        DeclareLaunchArgument(
            'run_without_sdk_actions',
            default_value='false',
            description=(
                'Skip stand/jump SDK action stages when the SDK action helper '
                'is unavailable. Use only for movement-only tests.'
            )
        ),
        DeclareLaunchArgument(
            'allow_ros_topic_sdk_actions',
            default_value='false',
            description=(
                'Allow fallback to /api/sport/request. Keep false unless '
                'unitree_api typesupport/RMW has been verified.'
            )
        ),
        DeclareLaunchArgument(
            'line_lost_switch_sec',
            default_value='0.6',
            description=(
                'Switch from line following to the hardcoded obstacle route '
                'after line_visible has stayed false for this many seconds.'
            )
        ),
        DeclareLaunchArgument(
            'line_track_stale_sec',
            default_value='0.8',
            description=(
                'Treat /perception/line_track as lost if no fresh message '
                'arrives within this many seconds.'
            )
        ),
        DeclareLaunchArgument(
            'white_line_detection_enabled',
            default_value='true',
            description='Enable image-based long-white-line trigger.'
        ),
        DeclareLaunchArgument(
            'white_line_min_width_fraction',
            default_value='0.22',
            description='Minimum ROI width fraction for the long white line.'
        ),
        DeclareLaunchArgument(
            'white_line_min_value',
            default_value='160',
            description='HSV V lower threshold for white-line detection.'
        ),
        DeclareLaunchArgument(
            'white_line_max_saturation',
            default_value='130',
            description='HSV S upper threshold for white-line detection.'
        ),
        DeclareLaunchArgument(
            'start_realsense',
            default_value='true',
            description='Start D435i camera for line tracking.'
        ),
        DeclareLaunchArgument(
            'start_line_nodes',
            default_value='true',
            description='Start real_line_tracker_node and line_follower_node.'
        ),
        DeclareLaunchArgument(
            'image_topic',
            default_value='/camera/color/image_raw',
            description='RGB image topic consumed by real_line_tracker_node.'
        ),
        DeclareLaunchArgument(
            'debug',
            default_value='false',
            description='Enable line perception debug image/logs.'
        ),
        LogInfo(
            msg='Starting direct hardcoded full route through SDK UDP plus '
            'small SDK action helper. No gait/action/rk_interfaces nodes '
            'are started.'
        ),
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            namespace='camera',
            name='camera',
            output='screen',
            condition=IfCondition(start_realsense),
            parameters=[{
                'enable_color': True,
                'enable_depth': True,
                'enable_gyro': False,
                'enable_accel': False,
                'rgb_camera.profile': '640x480x15',
                'depth_module.profile': '640x480x15',
            }],
        ),
        Node(
            package='rk_perception',
            executable='real_line_tracker_node',
            name='real_line_tracker_node',
            output='screen',
            condition=IfCondition(start_line_nodes),
            parameters=[
                line_nav_config,
                {
                    'image_topic': image_topic,
                    'enable_debug_image': ParameterValue(
                        debug,
                        value_type=bool
                    ),
                    'debug_log': ParameterValue(debug, value_type=bool),
                },
            ],
        ),
        Node(
            package='rk_navigation',
            executable='line_follower_node',
            name='line_follower_node',
            output='screen',
            condition=IfCondition(start_line_nodes),
            parameters=[
                line_nav_config,
                {
                    'debug_log': ParameterValue(debug, value_type=bool),
                },
            ],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sdk_bridge_launch),
            launch_arguments={
                'start_sdk_server': start_sdk_server,
                'sdk_server': sdk_server,
                'max_vx': bridge_max_linear_x,
                'max_yaw': bridge_max_angular_z,
            }.items(),
        ),
        ExecuteProcess(
            cmd=[
                'python3',
                PathJoinSubstitution([
                    workspace_dir,
                    'src',
                    'rk_tools',
                    'rk_tools',
                    'obstacle_direct_route_node.py',
                ]),
                '--ros-args',
                '-r',
                '__node:=obstacle_direct_route_node',
                '-p',
                'cmd_vel_topic:=/navigation/cmd_vel',
                '-p',
                ['countdown_sec:=', countdown_sec],
                '-p',
                ['distance_scale:=', distance_scale],
                '-p',
                ['turn_scale:=', turn_scale],
                '-p',
                ['speed_scale:=', speed_scale],
                '-p',
                ['sdk_network_interface:=', sdk_network_interface],
                '-p',
                ['sdk_action_executable:=', sdk_action_executable],
                '-p',
                'mission_start_topic:=/mission/start',
                '-p',
                'mission_stop_topic:=/mission/stop',
                '-p',
                'line_track_topic:=/perception/line_track',
                '-p',
                ['white_line_image_topic:=', image_topic],
                '-p',
                [
                    'white_line_detection_enabled:=',
                    white_line_detection_enabled,
                ],
                '-p',
                [
                    'white_line_min_width_fraction:=',
                    white_line_min_width_fraction,
                ],
                '-p',
                ['white_line_min_value:=', white_line_min_value],
                '-p',
                ['white_line_max_saturation:=', white_line_max_saturation],
                '-p',
                'line_visible_wait_timeout_sec:=10.0',
                '-p',
                ['line_lost_switch_sec:=', line_lost_switch_sec],
                '-p',
                ['line_track_stale_sec:=', line_track_stale_sec],
                '-p',
                ['run_without_sdk_actions:=', run_without_sdk_actions],
                '-p',
                [
                    'allow_ros_topic_sdk_actions:=',
                    allow_ros_topic_sdk_actions,
                ],
            ],
            output='screen',
            additional_env=ROUTE_NODE_ENV,
        ),
    ])
