#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('rk_arm_control')
    default_params_file = os.path.join(
        package_share,
        'config',
        'new_arm_params.yaml'
    )
    default_poses_file = os.path.join(
        package_share,
        'config',
        'new_arm_poses.yaml'
    )

    params_file = LaunchConfiguration('params_file')
    poses_file = LaunchConfiguration('poses_file')
    bridge_mode = LaunchConfiguration('bridge_mode')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='New arm topic, perception, adapter, and bridge YAML.'
        ),
        DeclareLaunchArgument(
            'poses_file',
            default_value=default_poses_file,
            description='New arm fixed poses and task sequence YAML file.'
        ),
        DeclareLaunchArgument(
            'bridge_mode',
            default_value='mock',
            description='SDK bridge mode: mock first; real after SDK wiring.'
        ),
        Node(
            package='rk_arm_control',
            executable='new_arm_task_node',
            name='new_arm_task_node',
            output='screen',
            parameters=[{
                'params_file': params_file,
                'poses_file': poses_file,
            }],
        ),
        Node(
            package='rk_arm_control',
            executable='new_arm_sdk_bridge_node',
            name='new_arm_sdk_bridge_node',
            output='screen',
            parameters=[{
                'config_file': params_file,
                'bridge_mode': bridge_mode,
            }],
        ),
    ])
