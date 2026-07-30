#!/usr/bin/env python3

"""启动ROS转发器和仓库内确定性Unitree SDK UDP服务。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackagePrefix


SDK_SERVER_ENV = {
    'LD_LIBRARY_PATH': (
        '/usr/local/lib:'
        '/home/unitree/cyclonedds_ws/install/cyclonedds/lib'
    ),
}
FORWARDER_ENV = {
    'ROS_DOMAIN_ID': '10',
    'LD_LIBRARY_PATH': (
        '/opt/ros/foxy/lib/aarch64-linux-gnu:'
        '/opt/ros/foxy/lib'
    ),
}


def generate_launch_description():
    start_sdk_server = LaunchConfiguration('start_sdk_server')
    sdk_server = LaunchConfiguration('sdk_server')
    sdk_network_interface = LaunchConfiguration('sdk_network_interface')
    sdk_rate_hz = LaunchConfiguration('sdk_rate_hz')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    udp_host = LaunchConfiguration('udp_host')
    udp_port = LaunchConfiguration('udp_port')
    max_vx = LaunchConfiguration('max_vx')
    max_vy = LaunchConfiguration('max_vy')
    max_yaw = LaunchConfiguration('max_yaw')
    deadband = LaunchConfiguration('deadband')
    timeout_sec = LaunchConfiguration('timeout_sec')

    return LaunchDescription([
        DeclareLaunchArgument(
            'start_sdk_server',
            default_value='true',
            description='Start the repository-owned Go2 SDK UDP server.'
        ),
        DeclareLaunchArgument(
            'sdk_server',
            default_value=PathJoinSubstitution([
                FindPackagePrefix('rk_go2_sdk_bridge'),
                'lib',
                'rk_go2_sdk_bridge',
                'go2_sdk_udp_server',
            ]),
            description='Path to the repository-owned Go2 SDK UDP server.'
        ),
        DeclareLaunchArgument(
            'sdk_network_interface',
            default_value='eth0',
            description='Network interface used by Unitree SDK2.'
        ),
        DeclareLaunchArgument(
            'sdk_rate_hz',
            default_value='20.0',
            description='Fixed SportClient.Move output rate.'
        ),
        DeclareLaunchArgument(
            'cmd_vel_topic',
            default_value='/navigation/cmd_vel',
            description='Input geometry_msgs/Twist topic.'
        ),
        DeclareLaunchArgument(
            'udp_host',
            default_value='127.0.0.1',
            description='UDP server host.'
        ),
        DeclareLaunchArgument(
            'udp_port',
            default_value='15001',
            description='UDP server port.'
        ),
        DeclareLaunchArgument(
            'max_vx',
            default_value='0.25',
            description='Maximum absolute forward speed sent to SDK server.'
        ),
        DeclareLaunchArgument(
            'max_vy',
            default_value='0.05',
            description='Maximum absolute lateral speed sent to SDK server.'
        ),
        DeclareLaunchArgument(
            'max_yaw',
            default_value='0.60',
            description='Maximum absolute yaw speed sent to SDK server.'
        ),
        DeclareLaunchArgument(
            'deadband',
            default_value='0.01',
            description='Velocity deadband before sending zero.'
        ),
        DeclareLaunchArgument(
            'timeout_sec',
            default_value='0.30',
            description='Send stop after this many seconds without cmd_vel.'
        ),
        LogInfo(
            msg='Starting Go2 SDK UDP bridge: /navigation/cmd_vel -> UDP '
            '-> Unitree SportClient.Move().'
        ),
        ExecuteProcess(
            cmd=[
                sdk_server,
                '--interface', sdk_network_interface,
                '--listen-ip', udp_host,
                '--port', udp_port,
                '--rate-hz', sdk_rate_hz,
                '--watchdog-sec', timeout_sec,
                '--max-vx', max_vx,
                '--max-vy', max_vy,
                '--max-yaw', max_yaw,
                '--deadband', deadband,
            ],
            output='screen',
            condition=IfCondition(start_sdk_server),
            additional_env=SDK_SERVER_ENV,
        ),
        Node(
            package='rk_go2_sdk_bridge',
            executable='cmd_vel_udp_forwarder.py',
            name='cmd_vel_udp_forwarder',
            output='screen',
            additional_env=FORWARDER_ENV,
            parameters=[{
                'cmd_vel_topic': cmd_vel_topic,
                'udp_host': udp_host,
                'udp_port': ParameterValue(udp_port, value_type=int),
                'max_vx': ParameterValue(max_vx, value_type=float),
                'max_vy': ParameterValue(max_vy, value_type=float),
                'max_yaw': ParameterValue(max_yaw, value_type=float),
                'deadband': ParameterValue(deadband, value_type=float),
                'timeout_sec': ParameterValue(timeout_sec, value_type=float),
            }],
        ),
    ])
