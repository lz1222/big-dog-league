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
        'd1_arm_params.yaml'
    )

    config_file = LaunchConfiguration('config_file')
    dry_run = LaunchConfiguration('dry_run')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config_file,
            description='D1 camera-XY pick YAML file.'
        ),
        DeclareLaunchArgument(
            'dry_run',
            default_value='true',
            description='Use dry-run D1 adapter when true.'
        ),
        Node(
            package='rk_arm_control',
            executable='d1_pick_node',
            name='d1_arm',
            output='screen',
            parameters=[
                config_file,
                {
                    'dry_run': dry_run,
                },
            ],
        ),
    ])
