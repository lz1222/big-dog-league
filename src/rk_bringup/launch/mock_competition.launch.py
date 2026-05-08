#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    auto_start = LaunchConfiguration('auto_start')

    return LaunchDescription([
        DeclareLaunchArgument(
            'auto_start',
            default_value='true',
            description='Automatically start the mock mission through /mission/run.'
        ),
        Node(
            package='rk_perception',
            executable='mock_line_tracker_node',
            name='mock_line_tracker_node',
            output='screen'
        ),
        Node(
            package='rk_perception',
            executable='mock_sign_detector_node',
            name='mock_sign_detector_node',
            output='screen'
        ),
        Node(
            package='rk_perception',
            executable='mock_item_tag_node',
            name='mock_item_tag_node',
            output='screen'
        ),
        Node(
            package='rk_navigation',
            executable='line_follower_node',
            name='line_follower_node',
            output='screen'
        ),
        Node(
            package='rk_tools',
            executable='mock_locomotion_server',
            name='mock_locomotion_server',
            output='screen'
        ),
        Node(
            package='rk_tools',
            executable='mock_arm_server',
            name='mock_arm_server',
            output='screen'
        ),
        Node(
            package='rk_tools',
            executable='safety_node',
            name='safety_node',
            output='screen'
        ),
        Node(
            package='rk_mission',
            executable='mission_state_machine_node',
            name='mission_state_machine_node',
            output='screen',
            parameters=[{'auto_start': auto_start}]
        ),
    ])
