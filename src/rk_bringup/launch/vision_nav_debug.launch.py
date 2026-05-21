#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    image_topic = LaunchConfiguration('image_topic')
    enable_debug_image = LaunchConfiguration('enable_debug_image')
    debug_log = LaunchConfiguration('debug_log')

    perception_config = PathJoinSubstitution([
        FindPackageShare('rk_perception'),
        'config',
        'perception.yaml',
    ])
    navigation_config = PathJoinSubstitution([
        FindPackageShare('rk_navigation'),
        'config',
        'navigation.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'image_topic',
            default_value='/camera/camera/color/image_raw',
            description='RGB image topic consumed by real_line_tracker_node.'
        ),
        DeclareLaunchArgument(
            'enable_debug_image',
            default_value='true',
            description='Publish line mask and overlay debug images.'
        ),
        DeclareLaunchArgument(
            'debug_log',
            default_value='true',
            description='Print throttled perception and navigation debug logs.'
        ),
        LogInfo(msg='Starting RK vision + navigation debug nodes.'),
        LogInfo(msg=['image_topic: ', image_topic]),
        LogInfo(msg=['enable_debug_image: ', enable_debug_image]),
        LogInfo(msg=['debug_log: ', debug_log]),
        Node(
            package='rk_perception',
            executable='real_line_tracker_node',
            name='real_line_tracker_node',
            output='screen',
            parameters=[
                perception_config,
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
        Node(
            package='rk_navigation',
            executable='line_follower_node',
            name='line_follower_node',
            output='screen',
            parameters=[
                navigation_config,
                {
                    'debug_log': ParameterValue(
                        debug_log,
                        value_type=bool
                    ),
                },
            ]
        ),
    ])
