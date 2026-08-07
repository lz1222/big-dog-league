#!/usr/bin/env python3

"""正式 USB 巡线相机启动入口。

该入口只允许显式指定一个 UVC ``/dev/videoN`` 设备，并固定发布
``/line_camera/image_raw``。它不枚举、不回退设备，因此不会误把 D435i
接入巡线控制链。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """以可审计的设备号和采集参数启动唯一正式巡线相机。"""
    device = LaunchConfiguration('device')
    width = LaunchConfiguration('width')
    height = LaunchConfiguration('height')
    fps = LaunchConfiguration('fps')
    return LaunchDescription([
        DeclareLaunchArgument(
            'device', default_value='0',
            description='Explicit USB line camera /dev/videoN index.',
        ),
        DeclareLaunchArgument(
            'width', default_value='640',
            description='Requested USB line camera width in pixels.',
        ),
        DeclareLaunchArgument(
            'height', default_value='480',
            description='Requested USB line camera height in pixels.',
        ),
        DeclareLaunchArgument(
            'fps', default_value='15.0',
            description='Requested USB line camera frame rate.',
        ),
        LogInfo(msg=(
            'Starting USB line camera only: /line_camera/image_raw '
            '(frame_id=line_camera_optical_frame).'
        )),
        Node(
            package='rk_bringup',
            executable='line_camera_node',
            name='line_camera_node',
            output='screen',
            parameters=[{
                'device': ParameterValue(device, value_type=int),
                'width': ParameterValue(width, value_type=int),
                'height': ParameterValue(height, value_type=int),
                'fps': ParameterValue(fps, value_type=float),
            }],
        ),
    ])
