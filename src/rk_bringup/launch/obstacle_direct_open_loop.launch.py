#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


ROUTE_NODE_ENV = {
    'ROS_DOMAIN_ID': '10',
    'LD_LIBRARY_PATH': (
        '/home/unitree/cyclonedds_ws/install/unitree_api/lib:'
        '/home/unitree/rk_inspection_ws/third_party/unitree_ros2/'
        'cyclonedds_ws/install/unitree_api/lib:'
        '/opt/ros/foxy/lib/aarch64-linux-gnu:'
        '/opt/ros/foxy/lib'
    ),
    'PYTHONPATH': (
        '/home/unitree/cyclonedds_ws/install/unitree_api/lib/'
        'python3.8/site-packages:'
        '/home/unitree/rk_inspection_ws/third_party/unitree_ros2/'
        'cyclonedds_ws/install/unitree_api/lib/python3.8/site-packages'
    ),
}


def generate_launch_description():
    start_sdk_server = LaunchConfiguration('start_sdk_server')
    sdk_server = LaunchConfiguration('sdk_server')
    bridge_max_linear_x = LaunchConfiguration('bridge_max_linear_x')
    bridge_max_angular_z = LaunchConfiguration('bridge_max_angular_z')
    countdown_sec = LaunchConfiguration('countdown_sec')
    distance_scale = LaunchConfiguration('distance_scale')
    turn_scale = LaunchConfiguration('turn_scale')
    speed_scale = LaunchConfiguration('speed_scale')
    sdk_network_interface = LaunchConfiguration('sdk_network_interface')
    sdk_action_executable = LaunchConfiguration('sdk_action_executable')
    start_realsense = LaunchConfiguration('start_realsense')
    start_line_nodes = LaunchConfiguration('start_line_nodes')
    image_topic = LaunchConfiguration('image_topic')
    debug = LaunchConfiguration('debug')

    sdk_bridge_launch = PathJoinSubstitution([
        FindPackageShare('rk_go2_sdk_bridge'),
        'launch',
        'go2_sdk_udp_bridge.launch.py',
    ])
    line_nav_config = PathJoinSubstitution([
        FindPackageShare('rk_bringup'),
        'config',
        'line_nav_params.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'start_sdk_server',
            default_value='true',
            description='Start the existing Go2 SDK UDP server process.'
        ),
        DeclareLaunchArgument(
            'sdk_server',
            default_value=(
                '/home/unitree/unitree_go2_sdk_test/build/'
                'go2_sdk_udp_server'
            ),
            description='Path to the working Go2 SDK UDP server binary.'
        ),
        DeclareLaunchArgument(
            'bridge_max_linear_x',
            default_value='0.60',
            description='Maximum absolute vx passed through the UDP bridge.'
        ),
        DeclareLaunchArgument(
            'bridge_max_angular_z',
            default_value='1.00',
            description='Maximum absolute yaw speed passed through the bridge.'
        ),
        DeclareLaunchArgument(
            'countdown_sec',
            default_value='3.0',
            description='Zero-cmd countdown before the route starts.'
        ),
        DeclareLaunchArgument(
            'distance_scale',
            default_value='1.0',
            description='Scale all hardcoded forward distances.'
        ),
        DeclareLaunchArgument(
            'turn_scale',
            default_value='1.0',
            description='Scale all hardcoded turn angles.'
        ),
        DeclareLaunchArgument(
            'speed_scale',
            default_value='1.0',
            description='Scale all hardcoded linear and yaw speeds.'
        ),
        DeclareLaunchArgument(
            'sdk_network_interface',
            default_value='eth0',
            description='Go2 SDK2 network interface for FrontJump actions.'
        ),
        DeclareLaunchArgument(
            'sdk_action_executable',
            default_value=(
                '/home/unitree/rk_inspection_ws/install/'
                'rk_go2_sdk_bridge/lib/rk_go2_sdk_bridge/'
                'go2_sdk_motion_action'
            ),
            description='Helper executable used for SDK actions.'
        ),
        DeclareLaunchArgument(
            'start_realsense',
            default_value='true',
            description='Start D435i camera for line tracking.'
        ),
        DeclareLaunchArgument(
            'start_line_nodes',
            default_value='true',
            description='Start real_line_tracker_node and line_follower_node.'
        ),
        DeclareLaunchArgument(
            'image_topic',
            default_value='/camera/color/image_raw',
            description='RGB image topic consumed by real_line_tracker_node.'
        ),
        DeclareLaunchArgument(
            'debug',
            default_value='false',
            description='Enable line perception debug image/logs.'
        ),
        LogInfo(
            msg='Starting direct hardcoded full route through SDK UDP plus '
            'small SDK action helper. No gait/action/rk_interfaces nodes '
            'are started.'
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
                'rgb_camera.profile': '640x480x15',
                'depth_module.profile': '640x480x15',
            }],
        ),
        Node(
            package='rk_perception',
            executable='real_line_tracker_node',
            name='real_line_tracker_node',
            output='screen',
            condition=IfCondition(start_line_nodes),
            parameters=[
                line_nav_config,
                {
                    'image_topic': image_topic,
                    'enable_debug_image': ParameterValue(
                        debug,
                        value_type=bool
                    ),
                    'debug_log': ParameterValue(debug, value_type=bool),
                },
            ],
        ),
        Node(
            package='rk_navigation',
            executable='line_follower_node',
            name='line_follower_node',
            output='screen',
            condition=IfCondition(start_line_nodes),
            parameters=[
                line_nav_config,
                {
                    'debug_log': ParameterValue(debug, value_type=bool),
                },
            ],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sdk_bridge_launch),
            launch_arguments={
                'start_sdk_server': start_sdk_server,
                'sdk_server': sdk_server,
                'max_vx': bridge_max_linear_x,
                'max_yaw': bridge_max_angular_z,
            }.items(),
        ),
        Node(
            package='rk_tools',
            executable='obstacle_direct_route_node',
            name='obstacle_direct_route_node',
            output='screen',
            additional_env=ROUTE_NODE_ENV,
            parameters=[{
                'cmd_vel_topic': '/navigation/cmd_vel',
                'countdown_sec': ParameterValue(
                    countdown_sec,
                    value_type=float
                ),
                'distance_scale': ParameterValue(
                    distance_scale,
                    value_type=float
                ),
                'turn_scale': ParameterValue(
                    turn_scale,
                    value_type=float
                ),
                'speed_scale': ParameterValue(
                    speed_scale,
                    value_type=float
                ),
                'sdk_network_interface': sdk_network_interface,
                'sdk_action_executable': sdk_action_executable,
                'mission_start_topic': '/mission/start',
                'mission_stop_topic': '/mission/stop',
            }],
        ),
    ])
