#!/usr/bin/env python3

"""LEGACY / DEBUG / NOT FOR COMPETITION.

该历史避障入口保留独立 RealSense 接线；不属于本轮固定的 USB 巡线、Go2
标识或机械臂 D435i RGB-D 正式运行链。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


SDK_SERVER_ENV = {
    'LD_LIBRARY_PATH': (
        '/usr/local/lib:'
        '/home/unitree/cyclonedds_ws/install/cyclonedds/lib'
    ),
}
FORWARDER_ENV = {
    'ROS_DOMAIN_ID': '10',
    'LD_LIBRARY_PATH': (
        '/opt/ros/foxy/lib/aarch64-linux-gnu:'
        '/opt/ros/foxy/lib'
    ),
}


def generate_launch_description():
    bridge_type = LaunchConfiguration('bridge_type')
    backend = LaunchConfiguration('backend')
    start_realsense = LaunchConfiguration('start_realsense')
    start_sdk_server = LaunchConfiguration('start_sdk_server')
    sdk_server = LaunchConfiguration('sdk_server')
    depth_image_topic = LaunchConfiguration('depth_image_topic')
    scan_topic = LaunchConfiguration('scan_topic')
    enable_scan = LaunchConfiguration('enable_scan')
    require_safety_data = LaunchConfiguration('require_safety_data')
    bridge_max_linear_x = LaunchConfiguration('bridge_max_linear_x')
    bridge_max_angular_z = LaunchConfiguration('bridge_max_angular_z')
    sdk_udp_host = LaunchConfiguration('sdk_udp_host')
    sdk_udp_port = LaunchConfiguration('sdk_udp_port')

    use_sdk_bridge = IfCondition(
        PythonExpression(["'", bridge_type, "' == 'sdk_udp'"])
    )
    use_sdk_server = IfCondition(
        PythonExpression([
            "'", bridge_type, "' == 'sdk_udp' and '",
            start_sdk_server,
            "' == 'true'",
        ])
    )
    use_unitree_driver = IfCondition(
        PythonExpression(["'", bridge_type, "' == 'unitree_driver'"])
    )

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
            'bridge_type',
            default_value='sdk_udp',
            description=(
                'Low-level bridge: sdk_udp uses the same SDK UDP chain as '
                'line following; unitree_driver uses rk_unitree_driver.'
            )
        ),
        DeclareLaunchArgument(
            'backend',
            default_value='mock',
            description=(
                'Only used when bridge_type:=unitree_driver. Values: mock '
                'or unitree_ros2.'
            )
        ),
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
            default_value='0.60',
            description=(
                'cmd_vel bridge linear.x safety limit for obstacle tests.'
            )
        ),
        DeclareLaunchArgument(
            'bridge_max_angular_z',
            default_value='1.00',
            description=(
                'cmd_vel bridge angular.z safety limit for obstacle tests.'
            )
        ),
        DeclareLaunchArgument(
            'sdk_udp_host',
            default_value='127.0.0.1',
            description='SDK UDP server host.'
        ),
        DeclareLaunchArgument(
            'sdk_udp_port',
            default_value='15001',
            description='SDK UDP server port.'
        ),
        LogInfo(
            msg='Starting practical obstacle stack: D435i depth + gait safety '
            '+ Go2 SDK-compatible cmd_vel bridge.'
        ),
        LogInfo(msg=['bridge_type: ', bridge_type]),
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
            additional_env=FORWARDER_ENV,
            parameters=[
                gait_config,
                {
                    # Standalone hardware test keeps the existing direct path.
                    'cmd_vel_topic': '/navigation/cmd_vel',
                    'enable_motion_action': False,
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
        ExecuteProcess(
            cmd=[sdk_server],
            output='screen',
            condition=use_sdk_server,
            additional_env=SDK_SERVER_ENV,
        ),
        Node(
            package='rk_go2_sdk_bridge',
            executable='cmd_vel_udp_forwarder.py',
            name='cmd_vel_udp_forwarder',
            output='screen',
            condition=use_sdk_bridge,
            additional_env=FORWARDER_ENV,
            parameters=[{
                'cmd_vel_topic': '/navigation/cmd_vel',
                'udp_host': sdk_udp_host,
                'udp_port': ParameterValue(sdk_udp_port, value_type=int),
                'max_vx': ParameterValue(
                    bridge_max_linear_x,
                    value_type=float
                ),
                'max_yaw': ParameterValue(
                    bridge_max_angular_z,
                    value_type=float
                ),
            }],
        ),
        Node(
            package='rk_unitree_driver',
            executable='cmd_vel_bridge_node',
            name='cmd_vel_bridge_node',
            output='screen',
            condition=use_unitree_driver,
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
