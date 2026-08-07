#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory('rk_unitree_driver')
    config_file = os.path.join(package_share, 'config', 'go2_driver.yaml')

    backend = LaunchConfiguration('backend')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    sport_request_topic = LaunchConfiguration('sport_request_topic')
    max_linear_x = LaunchConfiguration('max_linear_x')
    max_angular_z = LaunchConfiguration('max_angular_z')

    return LaunchDescription([
        DeclareLaunchArgument(
            'backend',
            default_value='mock',
            description=(
                'Motion backend: mock logs commands only; unitree_ros2 '
                'publishes unitree_api/msg/Request for a real robot.'
            )
        ),
        DeclareLaunchArgument(
            'cmd_vel_topic',
            default_value='/navigation/cmd_vel',
            description='Input geometry_msgs/Twist topic.'
        ),
        DeclareLaunchArgument(
            'sport_request_topic',
            default_value='/api/sport/request',
            description='Output unitree_api/msg/Request topic.'
        ),
        DeclareLaunchArgument(
            'max_linear_x',
            default_value='0.20',
            description='Maximum allowed absolute linear.x speed.'
        ),
        DeclareLaunchArgument(
            'max_angular_z',
            default_value='0.60',
            description='Maximum allowed absolute angular.z speed.'
        ),
        Node(
            package='rk_unitree_driver',
            executable='cmd_vel_bridge_node',
            name='cmd_vel_bridge_node',
            output='screen',
            parameters=[
                config_file,
                {
                    'backend': backend,
                    'cmd_vel_topic': cmd_vel_topic,
                    'sport_request_topic': sport_request_topic,
                    'max_linear_x': ParameterValue(
                        max_linear_x,
                        value_type=float
                    ),
                    'max_angular_z': ParameterValue(
                        max_angular_z,
                        value_type=float
                    ),
                },
            ]
        ),
    ])
