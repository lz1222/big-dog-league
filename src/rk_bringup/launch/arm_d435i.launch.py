#!/usr/bin/env python3

"""机械臂 D435i 的唯一正式 RGB-D 启动入口。

本 launch 只发布 ``/arm_camera`` 命名空间下的对齐 RGB-D 数据，不启动
巡线、标识识别或任何机械臂控制节点，避免 D435i 被误用为赛道相机。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """以固定比赛 topic 启动 D435i，并允许显式指定设备序列号。"""
    serial_no = LaunchConfiguration('serial_no')
    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_no', default_value='',
            description='Optional explicit D435i serial number; empty keeps driver default.',
        ),
        LogInfo(msg=(
            'Starting arm D435i only: /arm_camera/color + aligned depth + '
            'camera info. This launch is not a line or sign camera source.'
        )),
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            namespace='arm_camera',
            name='arm_d435i',
            output='screen',
            parameters=[{
                # 机械臂抓取依赖同一彩色坐标系的三路同步数据。
                'serial_no': serial_no,
                'enable_color': True,
                'enable_depth': True,
                'enable_gyro': False,
                'enable_accel': False,
                # 深度生成在设备内部仍会使用双目红外，但抓取链路只消费彩色、
                # 对齐深度和相机内参；关闭 Infra ROS 发布可减少图像拷贝与 DDS 压力。
                'enable_infra1': False,
                'enable_infra2': False,
                'pointcloud.enable': False,
                'enable_sync': False,
                'rgb_camera.profile': '640x480x15',
                'depth_module.profile': '640x480x15',
                # D435i 当前固件不接受 wrapper 默认的自动档值 3；显式关闭
                # 工频补偿以避免启动期反复重配流。现场若出现灯光频闪再按环境设为 1/2。
                'rgb_camera.power_line_frequency': 0,
                'align_depth.enable': True,
            }],
        ),
    ])
