#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    auto_start = LaunchConfiguration('auto_start')
    competition_config = PathJoinSubstitution([
        FindPackageShare('rk_config'),
        'config',
        'mission',
        'competition.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'auto_start',
            default_value='true',
            description=(
                'Automatically start the mock mission through /mission/run.'
            )
        ),
        LogInfo(msg='Starting RK mock competition system.'),
        LogInfo(msg=['mission auto_start: ', auto_start]),
        LogInfo(msg='Starting rk_perception mock nodes.'),
        LogInfo(msg='Starting rk_navigation line follower node.'),
        LogInfo(msg='Starting rk_tools mock hardware and safety nodes.'),
        LogInfo(msg='Starting rk_mission mission state machine node.'),
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
            output='screen',
            parameters=[competition_config]
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
            output='screen',
            parameters=[competition_config]
        ),
        Node(
            package='rk_mission',
            executable='line_course_mission_node',
            name='line_course_mission_node',
            output='screen',
            parameters=[competition_config]
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
            parameters=[
                competition_config,
                {'auto_start': auto_start},
            ]
        ),
    ])
