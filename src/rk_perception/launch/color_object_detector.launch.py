#!/usr/bin/env python3

"""启动独立的 D435i 颜色 RGB-D 定位节点，不启动相机或任何执行器。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """提供正式 TF 和默认关闭的未标定静态 TF 接口。"""
    config_file = LaunchConfiguration('config_file')
    publish_static_tf = LaunchConfiguration('publish_static_tf')
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('rk_perception'), 'config',
                'color_object_detector.yaml']),
            description='Color RGB-D detector YAML configuration file.'),
        DeclareLaunchArgument(
            'publish_static_tf', default_value='false',
            description=(
                'NOT CALIBRATED interface. Keep false until surveyed camera '
                'to arm-base extrinsics are available.')),
        DeclareLaunchArgument('static_tf_parent_frame', default_value='arm_base'),
        DeclareLaunchArgument(
            'static_tf_child_frame', default_value='d435i_color_optical_frame'),
        DeclareLaunchArgument('static_tf_x_m', default_value='0.0'),
        DeclareLaunchArgument('static_tf_y_m', default_value='0.0'),
        DeclareLaunchArgument('static_tf_z_m', default_value='0.0'),
        DeclareLaunchArgument('static_tf_roll_rad', default_value='0.0'),
        DeclareLaunchArgument('static_tf_pitch_rad', default_value='0.0'),
        DeclareLaunchArgument('static_tf_yaw_rad', default_value='0.0'),
        LogInfo(msg=['Starting color RGB-D detector with config: ', config_file]),
        # 此节点不启动 RealSense，避免与现有验收链路争夺 D435i。
        Node(
            package='rk_perception',
            executable='color_object_detector_node',
            name='color_object_detector_node',
            output='screen',
            parameters=[{'config_file': config_file}],
        ),
        # 仅为完成标定后的部署预留接口；默认零外参绝不能作为正式结果。
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='color_object_detector_static_tf', output='screen',
            condition=IfCondition(publish_static_tf),
            arguments=[
                LaunchConfiguration('static_tf_x_m'),
                LaunchConfiguration('static_tf_y_m'),
                LaunchConfiguration('static_tf_z_m'),
                LaunchConfiguration('static_tf_yaw_rad'),
                LaunchConfiguration('static_tf_pitch_rad'),
                LaunchConfiguration('static_tf_roll_rad'),
                LaunchConfiguration('static_tf_parent_frame'),
                LaunchConfiguration('static_tf_child_frame'),
            ],
        ),
    ])
