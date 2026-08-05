#!/usr/bin/env python3
"""只启动 Sonix 巡线摄像头，供无控制链的图像流验证使用。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """提供可覆盖的相机参数；本入口不包含巡线控制或运动节点。"""
    config_file = PathJoinSubstitution([
        FindPackageShare('rk_perception'), 'config', 'line_camera.yaml',
    ])
    parameters = {
        'device': LaunchConfiguration('device'),
        'image_topic': LaunchConfiguration('image_topic'),
        'frame_id': LaunchConfiguration('frame_id'),
        'width': ParameterValue(LaunchConfiguration('width'), value_type=int),
        'height': ParameterValue(LaunchConfiguration('height'), value_type=int),
        'fps': ParameterValue(LaunchConfiguration('fps'), value_type=float),
        'fourcc': LaunchConfiguration('fourcc'),
        'buffer_size': ParameterValue(
            LaunchConfiguration('buffer_size'), value_type=int
        ),
        'read_failure_limit': ParameterValue(
            LaunchConfiguration('read_failure_limit'), value_type=int
        ),
        'frame_stall_timeout_sec': ParameterValue(
            LaunchConfiguration('frame_stall_timeout_sec'), value_type=float
        ),
    }
    return LaunchDescription([
        DeclareLaunchArgument('device', default_value=(
            '/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera_'
            'SN0001-video-index0'
        )),
        DeclareLaunchArgument(
            'image_topic', default_value='/camera/color/image_raw'
        ),
        DeclareLaunchArgument(
            'frame_id', default_value='line_camera_optical_frame'
        ),
        DeclareLaunchArgument('width', default_value='640'),
        DeclareLaunchArgument('height', default_value='480'),
        DeclareLaunchArgument('fps', default_value='15.0'),
        DeclareLaunchArgument('fourcc', default_value='MJPG'),
        DeclareLaunchArgument('buffer_size', default_value='1'),
        DeclareLaunchArgument('read_failure_limit', default_value='10'),
        DeclareLaunchArgument(
            'frame_stall_timeout_sec', default_value='1.0'
        ),
        Node(
            package='rk_perception',
            executable='usb_line_camera_node',
            name='usb_line_camera_node',
            output='screen',
            parameters=[config_file, parameters],
        ),
    ])
