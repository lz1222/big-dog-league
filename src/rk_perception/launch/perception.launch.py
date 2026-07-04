#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_mock_perception = LaunchConfiguration('use_mock_perception')
    image_topic = LaunchConfiguration('image_topic')
    enable_debug_image = LaunchConfiguration('enable_debug_image')
    debug_log = LaunchConfiguration('debug_log')
    config_file = PathJoinSubstitution([
        FindPackageShare('rk_perception'),
        'config',
        'perception.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_mock_perception',
            default_value='true',
            description=(
                'Use mock line tracker when true, real tracker when false.'
            )
        ),
        DeclareLaunchArgument(
            'image_topic',
            default_value='/camera/color/image_raw',
            description='RGB image topic consumed by real_line_tracker_node.'
        ),
        DeclareLaunchArgument(
            'enable_debug_image',
            default_value='false',
            description='Publish line mask and overlay debug images.'
        ),
        DeclareLaunchArgument(
            'debug_log',
            default_value='false',
            description='Print real tracker contour and error debug values.'
        ),
        LogInfo(msg=['use_mock_perception: ', use_mock_perception]),
        Node(
            package='rk_perception',
            executable='mock_line_tracker_node',
            name='mock_line_tracker_node',
            output='screen',
            condition=IfCondition(use_mock_perception)
        ),
        Node(
            package='rk_perception',
            executable='real_line_tracker_node',
            name='real_line_tracker_node',
            output='screen',
            condition=UnlessCondition(use_mock_perception),
            parameters=[
                config_file,
                {
                    'image_topic': image_topic,
                    'enable_debug_image': ParameterValue(
                        enable_debug_image,
                        value_type=bool
                    ),
                    'debug_log': ParameterValue(
                        debug_log,
                        value_type=bool
                    ),
                },
            ]
        ),
    ])
