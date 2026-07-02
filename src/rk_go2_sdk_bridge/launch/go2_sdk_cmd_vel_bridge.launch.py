#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    network_interface = LaunchConfiguration('network_interface')
    max_vx = LaunchConfiguration('max_vx')
    max_vy = LaunchConfiguration('max_vy')
    max_yaw = LaunchConfiguration('max_yaw')
    timeout_sec = LaunchConfiguration('timeout_sec')
    balance_stand_on_start = LaunchConfiguration('balance_stand_on_start')

    return LaunchDescription([
        DeclareLaunchArgument(
            'cmd_vel_topic',
            default_value='/navigation/cmd_vel',
            description='Input geometry_msgs/msg/Twist topic.'
        ),
        DeclareLaunchArgument(
            'network_interface',
            default_value='eth0',
            description='Network interface used by Unitree SDK DDS.'
        ),
        DeclareLaunchArgument(
            'max_vx',
            default_value='0.20',
            description='Maximum absolute linear.x velocity.'
        ),
        DeclareLaunchArgument(
            'max_vy',
            default_value='0.10',
            description='Maximum absolute linear.y velocity.'
        ),
        DeclareLaunchArgument(
            'max_yaw',
            default_value='0.5',
            description='Maximum absolute angular.z yaw velocity.'
        ),
        DeclareLaunchArgument(
            'timeout_sec',
            default_value='0.5',
            description='StopMove timeout after last cmd_vel message.'
        ),
        DeclareLaunchArgument(
            'balance_stand_on_start',
            default_value='true',
            description='Call BalanceStand after SDK initialization.'
        ),
        Node(
            package='rk_go2_sdk_bridge',
            executable='go2_sdk_cmd_vel_bridge',
            name='go2_sdk_cmd_vel_bridge',
            output='screen',
            parameters=[{
                'cmd_vel_topic': cmd_vel_topic,
                'network_interface': network_interface,
                'max_vx': ParameterValue(max_vx, value_type=float),
                'max_vy': ParameterValue(max_vy, value_type=float),
                'max_yaw': ParameterValue(max_yaw, value_type=float),
                'timeout_sec': ParameterValue(timeout_sec, value_type=float),
                'balance_stand_on_start': ParameterValue(
                    balance_stand_on_start,
                    value_type=bool
                ),
            }],
        ),
    ])
