#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


SDK_SERVER_ENV = {
    'LD_LIBRARY_PATH': (
        '/usr/local/lib:'
        '/home/unitree/cyclonedds_ws/install/cyclonedds/lib'
    ),
}
FORWARDER_ENV = {}


def generate_launch_description():
    debug = LaunchConfiguration('debug')
    bridge_type = LaunchConfiguration('bridge_type')
    backend = LaunchConfiguration('backend')
    start_sdk_server = LaunchConfiguration('start_sdk_server')
    sdk_server = LaunchConfiguration('sdk_server')
    start_realsense = LaunchConfiguration('start_realsense')
    image_topic = LaunchConfiguration('image_topic')
    bridge_max_linear_x = LaunchConfiguration('bridge_max_linear_x')
    bridge_max_angular_z = LaunchConfiguration('bridge_max_angular_z')
    zero_cmd_debounce_time = LaunchConfiguration('zero_cmd_debounce_time')
    sdk_udp_host = LaunchConfiguration('sdk_udp_host')
    sdk_udp_port = LaunchConfiguration('sdk_udp_port')

    use_sdk_bridge = IfCondition(
        PythonExpression(["'", bridge_type, "' == 'sdk_udp'"])
    )
    use_sdk_server = IfCondition(
        PythonExpression([
            "'", bridge_type, "' == 'sdk_udp' and '",
            start_sdk_server,
            "' == 'true'",
        ])
    )
    use_unitree_driver = IfCondition(
        PythonExpression(["'", bridge_type, "' == 'unitree_driver'"])
    )

    line_nav_config = PathJoinSubstitution([
        FindPackageShare('rk_bringup'),
        'config',
        'line_nav_params.yaml',
    ])
    go2_config = PathJoinSubstitution([
        FindPackageShare('rk_unitree_driver'),
        'config',
        'go2_driver.yaml',
    ])
    return LaunchDescription([
        DeclareLaunchArgument(
            'debug',
            default_value='true',
            description='Enable perception debug images and debug logs.'
        ),
        DeclareLaunchArgument(
            'bridge_type',
            default_value='sdk_udp',
            description=(
                'Low-level bridge: sdk_udp uses the working SDK UDP chain; '
                'unitree_driver uses rk_unitree_driver.'
            )
        ),
        DeclareLaunchArgument(
            'backend',
            default_value='mock',
            description=(
                'Only used when bridge_type:=unitree_driver. Values: mock '
                'or unitree_ros2.'
            )
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
            'start_realsense',
            default_value='true',
            description='Start realsense2_camera from this launch file.'
        ),
        DeclareLaunchArgument(
            'image_topic',
            default_value='/camera/color/image_raw',
            description='RGB image topic consumed by real_line_tracker_node.'
        ),
        DeclareLaunchArgument(
            'bridge_max_linear_x',
            default_value='0.20',
            description='cmd_vel bridge linear.x safety limit.'
        ),
        DeclareLaunchArgument(
            'bridge_max_angular_z',
            default_value='0.80',
            description='cmd_vel bridge angular.z safety limit.'
        ),
        DeclareLaunchArgument(
            'zero_cmd_debounce_time',
            default_value='0.60',
            description=(
                'Seconds that unitree_driver waits before converting a zero '
                'cmd_vel into StopMove.'
            )
        ),
        DeclareLaunchArgument(
            'sdk_udp_host',
            default_value='127.0.0.1',
            description='SDK UDP server host.'
        ),
        DeclareLaunchArgument(
            'sdk_udp_port',
            default_value='15001',
            description='SDK UDP server port.'
        ),
        LogInfo(
            msg='Starting competition line navigation. '
            'Robot waits for /mission/start before moving.'
        ),
        LogInfo(msg=['debug: ', debug]),
        LogInfo(msg=['bridge_type: ', bridge_type]),
        LogInfo(msg=['cmd_vel bridge backend: ', backend]),
        LogInfo(msg=['start_realsense: ', start_realsense]),
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
            parameters=[
                line_nav_config,
                {
                    'image_topic': image_topic,
                    'enable_debug_image': True,
                    'debug_log': True,
                },
            ],
        ),
        Node(
            package='rk_navigation',
            executable='line_follower_node',
            name='line_follower_node',
            output='screen',
            parameters=[
                line_nav_config,
                {
                    'debug_log': ParameterValue(debug, value_type=bool),
                },
            ],
        ),
        ExecuteProcess(
            cmd=[sdk_server],
            output='screen',
            condition=use_sdk_server,
            additional_env=SDK_SERVER_ENV,
        ),
        Node(
            package='rk_go2_sdk_bridge',
            executable='cmd_vel_udp_forwarder.py',
            name='cmd_vel_udp_forwarder',
            output='screen',
            condition=use_sdk_bridge,
            additional_env=FORWARDER_ENV,
            parameters=[{
                'cmd_vel_topic': '/navigation/cmd_vel',
                'udp_host': sdk_udp_host,
                'udp_port': ParameterValue(sdk_udp_port, value_type=int),
                'max_vx': ParameterValue(
                    bridge_max_linear_x,
                    value_type=float
                ),
                'max_yaw': ParameterValue(
                    bridge_max_angular_z,
                    value_type=float
                ),
            }],
        ),
        Node(
            package='rk_unitree_driver',
            executable='cmd_vel_bridge_node',
            name='cmd_vel_bridge_node',
            output='screen',
            condition=use_unitree_driver,
            parameters=[
                go2_config,
                {
                    'backend': backend,
                    'cmd_vel_topic': '/navigation/cmd_vel',
                    'max_linear_x': ParameterValue(
                        bridge_max_linear_x,
                        value_type=float
                    ),
                    'max_angular_z': ParameterValue(
                        bridge_max_angular_z,
                        value_type=float
                    ),
                    'zero_cmd_debounce_time': ParameterValue(
                        zero_cmd_debounce_time,
                        value_type=float
                    ),
                },
            ],
        ),
    ])
