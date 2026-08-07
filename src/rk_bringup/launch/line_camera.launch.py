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
    performance_stats_enabled = LaunchConfiguration('performance_stats_enabled')
    return LaunchDescription([
        DeclareLaunchArgument(
            'device',
            default_value=(
                '/dev/v4l/by-id/'
                'usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera_SN0001-video-index0'
            ),
            description='Stable explicit USB line-camera /dev/v4l/by-id path.',
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
        DeclareLaunchArgument(
            'performance_stats_enabled', default_value='false',
            description='Emit five-second capture/wrap/publish timing aggregates.',
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
                'device': ParameterValue(device, value_type=str),
                'width': ParameterValue(width, value_type=int),
                'height': ParameterValue(height, value_type=int),
                'fps': ParameterValue(fps, value_type=float),
                'performance_stats_enabled': ParameterValue(
                    performance_stats_enabled, value_type=bool
                ),
            }],
        ),
    ])
