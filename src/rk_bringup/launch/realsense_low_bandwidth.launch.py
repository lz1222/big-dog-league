#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    enable_color = LaunchConfiguration('enable_color')
    enable_depth = LaunchConfiguration('enable_depth')
    rgb_camera_profile = LaunchConfiguration('rgb_camera.profile')
    depth_module_profile = LaunchConfiguration('depth_module.profile')

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_color',
            default_value='true',
            description='Enable RealSense color stream.'
        ),
        DeclareLaunchArgument(
            'enable_depth',
            default_value='true',
            description='Enable RealSense depth stream.'
        ),
        DeclareLaunchArgument(
            'rgb_camera.profile',
            default_value='640x480x15',
            description='RealSense color stream profile.'
        ),
        DeclareLaunchArgument(
            'depth_module.profile',
            default_value='640x480x15',
            description='RealSense depth stream profile.'
        ),
        LogInfo(
            msg=[
                'Starting low-bandwidth RealSense: color=',
                rgb_camera_profile,
                ', depth=',
                depth_module_profile,
            ]
        ),
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            namespace='camera',
            name='camera',
            output='screen',
            parameters=[{
                'enable_color': ParameterValue(
                    enable_color,
                    value_type=bool
                ),
                'enable_depth': ParameterValue(
                    enable_depth,
                    value_type=bool
                ),
                'enable_gyro': False,
                'enable_accel': False,
                'rgb_camera.profile': rgb_camera_profile,
                'depth_module.profile': depth_module_profile,
            }],
        ),
    ])
