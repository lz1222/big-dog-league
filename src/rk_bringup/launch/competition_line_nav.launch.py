#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    debug = LaunchConfiguration('debug')
    backend = LaunchConfiguration('backend')
    start_realsense = LaunchConfiguration('start_realsense')
    image_topic = LaunchConfiguration('image_topic')
    line_auto_start = LaunchConfiguration('line_auto_start')
    bridge_max_linear_x = LaunchConfiguration('bridge_max_linear_x')
    bridge_max_angular_z = LaunchConfiguration('bridge_max_angular_z')
    start_keyboard_estop = LaunchConfiguration('start_keyboard_estop')
    estop_key = LaunchConfiguration('estop_key')
    send_damp_after_stand_down = LaunchConfiguration(
        'send_damp_after_stand_down'
    )

    line_nav_config = PathJoinSubstitution([
        FindPackageShare('rk_bringup'),
        'config',
        'line_nav_params.yaml',
    ])
    go2_config = PathJoinSubstitution([
        FindPackageShare('rk_unitree_driver'),
        'config',
        'go2_driver.yaml',
    ])
    return LaunchDescription([
        DeclareLaunchArgument(
            'debug',
            default_value='false',
            description='Enable perception debug images and debug logs.'
        ),
        DeclareLaunchArgument(
            'backend',
            default_value='unitree_ros2',
            description='cmd_vel bridge backend: mock or unitree_ros2.'
        ),
        DeclareLaunchArgument(
            'start_realsense',
            default_value='true',
            description='Start realsense2_camera from this launch file.'
        ),
        DeclareLaunchArgument(
            'image_topic',
            default_value='/camera/camera/color/image_raw',
            description='RGB image topic consumed by real_line_tracker_node.'
        ),
        DeclareLaunchArgument(
            'line_auto_start',
            default_value='false',
            description='Start line follower immediately after launch.'
        ),
        DeclareLaunchArgument(
            'bridge_max_linear_x',
            default_value='0.08',
            description='cmd_vel bridge linear.x safety limit.'
        ),
        DeclareLaunchArgument(
            'bridge_max_angular_z',
            default_value='0.35',
            description='cmd_vel bridge angular.z safety limit.'
        ),
        DeclareLaunchArgument(
            'start_keyboard_estop',
            default_value='true',
            description='Start keyboard emergency stop node.'
        ),
        DeclareLaunchArgument(
            'estop_key',
            default_value='g',
            description='Keyboard key that stops Go2 and sends StandDown.'
        ),
        DeclareLaunchArgument(
            'send_damp_after_stand_down',
            default_value='false',
            description='Send Damp after StandDown in keyboard emergency stop.'
        ),
        LogInfo(
            msg='Starting competition line navigation. '
            'Robot waits for /mission/start before moving.'
        ),
        LogInfo(msg=['debug: ', debug]),
        LogInfo(msg=['cmd_vel bridge backend: ', backend]),
        LogInfo(msg=['start_realsense: ', start_realsense]),
        LogInfo(msg=['line_auto_start: ', line_auto_start]),
        LogInfo(msg=['keyboard estop key: ', estop_key]),
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
                'rgb_camera.color_profile': '640,480,30',
            }],
        ),
        Node(
            package='rk_perception',
            executable='real_line_tracker_node',
            name='real_line_tracker_node',
            output='screen',
            parameters=[
                line_nav_config,
                {
                    'image_topic': image_topic,
                    'enable_debug_image': ParameterValue(
                        debug,
                        value_type=bool
                    ),
                    'debug_log': ParameterValue(debug, value_type=bool),
                    'auto_start': ParameterValue(
                        line_auto_start,
                        value_type=bool
                    ),
                },
            ],
        ),
        Node(
            package='rk_navigation',
            executable='line_follower_node',
            name='line_follower_node',
            output='screen',
            parameters=[
                line_nav_config,
                {
                    'debug_log': ParameterValue(debug, value_type=bool),
                },
            ],
        ),
        Node(
            package='rk_unitree_driver',
            executable='cmd_vel_bridge_node',
            name='cmd_vel_bridge_node',
            output='screen',
            parameters=[
                go2_config,
                {
                    'backend': backend,
                    'cmd_vel_topic': '/navigation/cmd_vel',
                    'max_linear_x': ParameterValue(
                        bridge_max_linear_x,
                        value_type=float
                    ),
                    'max_angular_z': ParameterValue(
                        bridge_max_angular_z,
                        value_type=float
                    ),
                },
            ],
        ),
        Node(
            package='rk_unitree_driver',
            executable='keyboard_estop_node',
            name='keyboard_estop_node',
            output='screen',
            condition=IfCondition(start_keyboard_estop),
            parameters=[
                go2_config,
                {
                    'backend': backend,
                    'cmd_vel_topic': '/navigation/cmd_vel',
                    'estop_key': estop_key,
                    'send_damp_after_stand_down': ParameterValue(
                        send_damp_after_stand_down,
                        value_type=bool
                    ),
                },
            ],
        ),
    ])
