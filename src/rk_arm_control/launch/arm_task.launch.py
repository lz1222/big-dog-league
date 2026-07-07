#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('rk_arm_control')
    default_config_file = os.path.join(
        package_share,
        'config',
        'arm_poses.yaml'
    )

    config_file = LaunchConfiguration('config_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config_file,
            description='Fixed arm pose YAML file.'
        ),
        Node(
            package='rk_arm_control',
            executable='arm_task_node',
            name='arm_task_node',
            output='screen',
            parameters=[{
                'config_file': config_file,
            }],
        ),
    ])
