#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    backend = LaunchConfiguration('backend')
    start_realsense = LaunchConfiguration('start_realsense')
    depth_image_topic = LaunchConfiguration('depth_image_topic')
    scan_topic = LaunchConfiguration('scan_topic')
    enable_scan = LaunchConfiguration('enable_scan')
    require_safety_data = LaunchConfiguration('require_safety_data')
    bridge_max_linear_x = LaunchConfiguration('bridge_max_linear_x')
    bridge_max_angular_z = LaunchConfiguration('bridge_max_angular_z')

    gait_config = PathJoinSubstitution([
        FindPackageShare('rk_locomotion'),
        'config',
        'gait_params.yaml',
    ])
    go2_config = PathJoinSubstitution([
        FindPackageShare('rk_unitree_driver'),
        'config',
        'go2_driver.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'backend',
            default_value='mock',
            description='cmd_vel bridge backend: mock or unitree_ros2.'
        ),
        DeclareLaunchArgument(
            'start_realsense',
            default_value='true',
            description='Start realsense2_camera with depth enabled.'
        ),
        DeclareLaunchArgument(
            'depth_image_topic',
            default_value='/camera/camera/depth/image_rect_raw',
            description='D435i depth image topic used by obstacle safety.'
        ),
        DeclareLaunchArgument(
            'scan_topic',
            default_value='/scan',
            description='Optional LaserScan topic from Go2 radar/SLAM stack.'
        ),
        DeclareLaunchArgument(
            'enable_scan',
            default_value='false',
            description='Fuse LaserScan with D435i depth safety.'
        ),
        DeclareLaunchArgument(
            'require_safety_data',
            default_value='true',
            description=(
                'Abort obstacle motion when no fresh depth/scan exists.'
            )
        ),
        DeclareLaunchArgument(
            'bridge_max_linear_x',
            default_value='0.12',
            description=(
                'cmd_vel bridge linear.x safety limit for obstacle tests.'
            )
        ),
        DeclareLaunchArgument(
            'bridge_max_angular_z',
            default_value='0.70',
            description=(
                'cmd_vel bridge angular.z safety limit for obstacle tests.'
            )
        ),
        LogInfo(
            msg='Starting practical obstacle stack: D435i depth + gait safety '
            '+ Go2 cmd_vel bridge.'
        ),
        LogInfo(msg=['cmd_vel bridge backend: ', backend]),
        LogInfo(msg=['require_safety_data: ', require_safety_data]),
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
            package='rk_locomotion',
            executable='gait_control_node',
            name='gait_control_node',
            output='screen',
            parameters=[
                gait_config,
                {
                    'obstacle_safety.depth_image_topic': depth_image_topic,
                    'obstacle_safety.scan_topic': scan_topic,
                    'obstacle_safety.enable_scan': ParameterValue(
                        enable_scan,
                        value_type=bool
                    ),
                    'obstacle_safety.require_fresh_data': ParameterValue(
                        require_safety_data,
                        value_type=bool
                    ),
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
    ])
