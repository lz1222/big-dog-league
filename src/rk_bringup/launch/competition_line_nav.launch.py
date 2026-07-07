#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    TimerAction,
)
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
FORWARDER_ENV = {}


def generate_launch_description():
    debug = LaunchConfiguration('debug')
    bridge_type = LaunchConfiguration('bridge_type')
    backend = LaunchConfiguration('backend')
    start_sdk_server = LaunchConfiguration('start_sdk_server')
    sdk_server = LaunchConfiguration('sdk_server')
    start_economic_gait = LaunchConfiguration('start_economic_gait')
    sdk_interface = LaunchConfiguration('sdk_interface')
    gait_action_delay = LaunchConfiguration('gait_action_delay')
    start_realsense = LaunchConfiguration('start_realsense')
    enable_depth = LaunchConfiguration('enable_depth')
    rgb_camera_profile = LaunchConfiguration('rgb_camera.profile')
    depth_module_profile = LaunchConfiguration('depth_module.profile')
    image_topic = LaunchConfiguration('image_topic')
    line_min_speed = LaunchConfiguration('line_min_speed')
    line_base_speed = LaunchConfiguration('line_base_speed')
    line_mid_speed = LaunchConfiguration('line_mid_speed')
    line_slow_speed = LaunchConfiguration('line_slow_speed')
    short_lost_linear_speed = LaunchConfiguration('short_lost_linear_speed')
    search_linear_speed = LaunchConfiguration('search_linear_speed')
    bridge_max_linear_x = LaunchConfiguration('bridge_max_linear_x')
    bridge_max_angular_z = LaunchConfiguration('bridge_max_angular_z')
    zero_cmd_debounce_time = LaunchConfiguration('zero_cmd_debounce_time')
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
    use_economic_gait = IfCondition(start_economic_gait)

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
            'bridge_type',
            default_value='sdk_udp',
            description=(
                'Low-level bridge: sdk_udp uses the working SDK UDP chain; '
                'unitree_driver uses rk_unitree_driver.'
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
            'start_economic_gait',
            default_value='true',
            description='Switch Go2 to Unitree EconomicGait on startup.'
        ),
        DeclareLaunchArgument(
            'sdk_interface',
            default_value='eth0',
            description='Network interface used by direct Unitree SDK actions.'
        ),
        DeclareLaunchArgument(
            'gait_action_delay',
            default_value='2.0',
            description='Delay before running the EconomicGait SDK action.'
        ),
        DeclareLaunchArgument(
            'start_realsense',
            default_value='true',
            description='Start realsense2_camera from this launch file.'
        ),
        DeclareLaunchArgument(
            'enable_depth',
            default_value='false',
            description='Enable RealSense depth stream during line following.'
        ),
        DeclareLaunchArgument(
            'rgb_camera.profile',
            default_value='424x240x15',
            description='RealSense color stream profile for line following.'
        ),
        DeclareLaunchArgument(
            'depth_module.profile',
            default_value='424x240x15',
            description='RealSense depth stream profile when depth is enabled.'
        ),
        DeclareLaunchArgument(
            'image_topic',
            default_value='/camera/color/image_raw',
            description='RGB image topic consumed by real_line_tracker_node.'
        ),
        DeclareLaunchArgument(
            'line_min_speed',
            default_value='0.27',
            description='Minimum nonzero forward speed used by line follower.'
        ),
        DeclareLaunchArgument(
            'line_base_speed',
            default_value='0.30',
            description='Line follower straight-line speed in m/s.'
        ),
        DeclareLaunchArgument(
            'line_mid_speed',
            default_value='0.28',
            description='Line follower medium error speed in m/s.'
        ),
        DeclareLaunchArgument(
            'line_slow_speed',
            default_value='0.27',
            description='Line follower large error speed in m/s.'
        ),
        DeclareLaunchArgument(
            'short_lost_linear_speed',
            default_value='0.27',
            description='Forward speed while briefly losing the line in m/s.'
        ),
        DeclareLaunchArgument(
            'search_linear_speed',
            default_value='0.27',
            description='Forward speed while searching for the line in m/s.'
        ),
        DeclareLaunchArgument(
            'bridge_max_linear_x',
            default_value='0.30',
            description=(
                'cmd_vel bridge linear.x safety limit. Keep this at least '
                'as high as the line follower base_speed.'
            )
        ),
        DeclareLaunchArgument(
            'bridge_max_angular_z',
            default_value='0.80',
            description='cmd_vel bridge angular.z safety limit.'
        ),
        DeclareLaunchArgument(
            'zero_cmd_debounce_time',
            default_value='0.60',
            description=(
                'Seconds that unitree_driver waits before converting a zero '
                'cmd_vel into StopMove.'
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
            msg='Starting competition line navigation. '
            'Robot waits for /mission/start before moving.'
        ),
        LogInfo(msg=['debug: ', debug]),
        LogInfo(msg=['bridge_type: ', bridge_type]),
        LogInfo(msg=['cmd_vel bridge backend: ', backend]),
        LogInfo(msg=['start_realsense: ', start_realsense]),
        LogInfo(msg=['start_economic_gait: ', start_economic_gait]),
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            namespace='camera',
            name='camera',
            output='screen',
            condition=IfCondition(start_realsense),
            parameters=[{
                'enable_color': True,
                'enable_depth': ParameterValue(enable_depth, value_type=bool),
                'enable_gyro': False,
                'enable_accel': False,
                'rgb_camera.profile': rgb_camera_profile,
                'depth_module.profile': depth_module_profile,
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
                    'debug_log': True,
                    'min_driving_speed': ParameterValue(
                        line_min_speed,
                        value_type=float
                    ),
                    'base_speed': ParameterValue(
                        line_base_speed,
                        value_type=float
                    ),
                    'mid_speed': ParameterValue(
                        line_mid_speed,
                        value_type=float
                    ),
                    'slow_speed': ParameterValue(
                        line_slow_speed,
                        value_type=float
                    ),
                    'short_lost_linear_speed': ParameterValue(
                        short_lost_linear_speed,
                        value_type=float
                    ),
                    'search_linear_speed': ParameterValue(
                        search_linear_speed,
                        value_type=float
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
        TimerAction(
            period=gait_action_delay,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ros2',
                        'run',
                        'rk_go2_sdk_bridge',
                        'go2_sdk_motion_action',
                        sdk_interface,
                        'economic_gait',
                        '1.0',
                    ],
                    output='screen',
                    condition=use_economic_gait,
                    additional_env=SDK_SERVER_ENV,
                ),
            ],
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
                    'zero_cmd_debounce_time': ParameterValue(
                        zero_cmd_debounce_time,
                        value_type=float
                    ),
                },
            ],
        ),
    ])
