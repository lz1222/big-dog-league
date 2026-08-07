#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    start_realsense = LaunchConfiguration('start_realsense')
    depth_image_topic = LaunchConfiguration('depth_image_topic')
    print_rate_hz = LaunchConfiguration('print_rate_hz')
    roi_top_ratio = LaunchConfiguration('roi_top_ratio')
    roi_bottom_ratio = LaunchConfiguration('roi_bottom_ratio')
    center_left_ratio = LaunchConfiguration('center_left_ratio')
    center_right_ratio = LaunchConfiguration('center_right_ratio')
    percentile = LaunchConfiguration('percentile')
    sample_step_px = LaunchConfiguration('sample_step_px')
    max_valid_m = LaunchConfiguration('max_valid_m')

    return LaunchDescription([
        DeclareLaunchArgument(
            'start_realsense',
            default_value='true',
            description='Start realsense2_camera with depth enabled.'
        ),
        DeclareLaunchArgument(
            'depth_image_topic',
            default_value='/camera/camera/depth/image_rect_raw',
            description='Depth image topic to inspect.'
        ),
        DeclareLaunchArgument(
            'print_rate_hz',
            default_value='2.0',
            description='Distance print rate.'
        ),
        DeclareLaunchArgument(
            'roi_top_ratio',
            default_value='0.30',
            description='Top boundary of the sampled ROI as image ratio.'
        ),
        DeclareLaunchArgument(
            'roi_bottom_ratio',
            default_value='0.72',
            description='Bottom boundary of the sampled ROI as image ratio.'
        ),
        DeclareLaunchArgument(
            'center_left_ratio',
            default_value='0.35',
            description='Left boundary of center distance ROI.'
        ),
        DeclareLaunchArgument(
            'center_right_ratio',
            default_value='0.65',
            description='Right boundary of center distance ROI.'
        ),
        DeclareLaunchArgument(
            'percentile',
            default_value='0.20',
            description='Sorted depth percentile to print.'
        ),
        DeclareLaunchArgument(
            'sample_step_px',
            default_value='8',
            description='Pixel sampling stride inside the ROI.'
        ),
        DeclareLaunchArgument(
            'max_valid_m',
            default_value='3.0',
            description='Ignore depth values farther than this distance.'
        ),
        LogInfo(
            msg='Starting D435i depth wall distance test.'
        ),
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            namespace='camera',
            name='camera',
            output='screen',
            condition=IfCondition(start_realsense),
            parameters=[{
                'enable_color': True,
                'enable_depth': True,
                'enable_gyro': False,
                'enable_accel': False,
                'rgb_camera.color_profile': '640,480,30',
                'depth_module.depth_profile': '640,480,30',
            }],
        ),
        Node(
            package='rk_tools',
            executable='depth_wall_distance_node',
            name='depth_wall_distance_node',
            output='screen',
            parameters=[{
                'depth_image_topic': depth_image_topic,
                'print_rate_hz': ParameterValue(
                    print_rate_hz,
                    value_type=float
                ),
                'roi_top_ratio': ParameterValue(
                    roi_top_ratio,
                    value_type=float
                ),
                'roi_bottom_ratio': ParameterValue(
                    roi_bottom_ratio,
                    value_type=float
                ),
                'center_left_ratio': ParameterValue(
                    center_left_ratio,
                    value_type=float
                ),
                'center_right_ratio': ParameterValue(
                    center_right_ratio,
                    value_type=float
                ),
                'percentile': ParameterValue(
                    percentile,
                    value_type=float
                ),
                'sample_step_px': ParameterValue(
                    sample_step_px,
                    value_type=int
                ),
                'max_valid_m': ParameterValue(
                    max_valid_m,
                    value_type=float
                ),
            }],
        ),
    ])
