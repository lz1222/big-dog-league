#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    start_realsense = LaunchConfiguration('start_realsense')
    start_gait_control = LaunchConfiguration('start_gait_control')
    image_topic = LaunchConfiguration('image_topic')
    enable_debug_image = LaunchConfiguration('enable_debug_image')
    debug_log = LaunchConfiguration('debug_log')
    rgb_camera_profile = LaunchConfiguration('rgb_camera.profile')
    dry_run_action = LaunchConfiguration('dry_run_action')
    sdk_network_interface = LaunchConfiguration('sdk_network_interface')

    perception_config = PathJoinSubstitution([
        FindPackageShare('rk_perception'),
        'config',
        'perception.yaml',
    ])
    gait_config = PathJoinSubstitution([
        FindPackageShare('rk_locomotion'),
        'config',
        'gait_params.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'start_realsense',
            default_value='true',
            description='Start the external RealSense color camera.'
        ),
        DeclareLaunchArgument(
            'start_gait_control',
            default_value='true',
            description='Start rk_locomotion gait_control_node.'
        ),
        DeclareLaunchArgument(
            'image_topic',
            default_value='/camera/color/image_raw',
            description='RGB image topic for sign recognition.'
        ),
        DeclareLaunchArgument(
            'enable_debug_image',
            default_value='true',
            description='Publish /perception/sign_debug_image overlay.'
        ),
        DeclareLaunchArgument(
            'debug_log',
            default_value='true',
            description='Print throttled sign detection logs.'
        ),
        DeclareLaunchArgument(
            'rgb_camera.profile',
            default_value='424x240x15',
            description='Low-bandwidth RGB profile for field testing.'
        ),
        DeclareLaunchArgument(
            'dry_run_action',
            default_value='false',
            description='Print mapped actions without publishing commands.'
        ),
        DeclareLaunchArgument(
            'sdk_network_interface',
            default_value='eth0',
            description='Network interface used by Unitree SDK2.'
        ),
        LogInfo(msg='Starting ROS2 sign recognition debug stack.'),
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            namespace='camera',
            name='camera',
            output='screen',
            condition=IfCondition(start_realsense),
            parameters=[{
                'enable_color': True,
                'enable_depth': False,
                'enable_gyro': False,
                'enable_accel': False,
                'rgb_camera.profile': rgb_camera_profile,
            }],
        ),
        Node(
            package='rk_perception',
            executable='real_sign_detector_node',
            name='real_sign_detector_node',
            output='screen',
            parameters=[
                perception_config,
                {
                    'image_topic': image_topic,
                    'enable_warning_templates': True,
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
            package='rk_locomotion',
            executable='gait_control_node',
            name='gait_control_node',
            output='screen',
            condition=IfCondition(start_gait_control),
            parameters=[
                gait_config,
                {
                    # Standalone action debug keeps the existing direct path.
                    'cmd_vel_topic': '/navigation/cmd_vel',
                    'obstacle_safety.enable_depth': False,
                    'enable_motion_action': True,
                },
            ]
        ),
        Node(
            package='rk_mission',
            executable='sign_action_executor_node',
            name='sign_action_executor_node',
            output='screen',
            parameters=[{
                'dry_run': ParameterValue(dry_run_action, value_type=bool),
                'sdk_network_interface': sdk_network_interface,
            }]
        ),
    ])
