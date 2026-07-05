#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


ROUTE_NODE_ENV = {
    'ROS_DOMAIN_ID': '10',
    'LD_LIBRARY_PATH': (
        '/opt/ros/foxy/lib/aarch64-linux-gnu:'
        '/opt/ros/foxy/lib'
    ),
}


def generate_launch_description():
    start_sdk_server = LaunchConfiguration('start_sdk_server')
    sdk_server = LaunchConfiguration('sdk_server')
    bridge_max_linear_x = LaunchConfiguration('bridge_max_linear_x')
    bridge_max_angular_z = LaunchConfiguration('bridge_max_angular_z')
    countdown_sec = LaunchConfiguration('countdown_sec')
    distance_scale = LaunchConfiguration('distance_scale')
    turn_scale = LaunchConfiguration('turn_scale')
    speed_scale = LaunchConfiguration('speed_scale')

    sdk_bridge_launch = PathJoinSubstitution([
        FindPackageShare('rk_go2_sdk_bridge'),
        'launch',
        'go2_sdk_udp_bridge.launch.py',
    ])

    return LaunchDescription([
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
            default_value='3.0',
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
        LogInfo(
            msg='Starting direct hardcoded obstacle route through SDK UDP. '
            'No gait/action/rk_interfaces nodes are started.'
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
        Node(
            package='rk_tools',
            executable='obstacle_direct_route_node',
            name='obstacle_direct_route_node',
            output='screen',
            additional_env=ROUTE_NODE_ENV,
            parameters=[{
                'cmd_vel_topic': '/navigation/cmd_vel',
                'countdown_sec': ParameterValue(
                    countdown_sec,
                    value_type=float
                ),
                'distance_scale': ParameterValue(
                    distance_scale,
                    value_type=float
                ),
                'turn_scale': ParameterValue(
                    turn_scale,
                    value_type=float
                ),
                'speed_scale': ParameterValue(
                    speed_scale,
                    value_type=float
                ),
            }],
        ),
    ])
