#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('rk_locomotion')
    config_file = os.path.join(package_share, 'config', 'gait_params.yaml')

    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')

    return LaunchDescription([
        DeclareLaunchArgument(
            'cmd_vel_topic',
            default_value='/control/locomotion_cmd',
            description='Locomotion Twist input consumed by command_mux_node.'
        ),
        Node(
            package='rk_locomotion',
            executable='gait_control_node',
            name='gait_control_node',
            output='screen',
            parameters=[
                config_file,
                {
                    'cmd_vel_topic': cmd_vel_topic,
                },
            ],
        ),
    ])
