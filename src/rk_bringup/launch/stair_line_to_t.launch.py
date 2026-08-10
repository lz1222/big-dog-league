#!/usr/bin/env python3
"""独立验证入口：USB 巡线、T 型门禁和楼梯 Phase 3，不接入迷宫流程。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare


def generate_launch_description():
    """创建独占 /stairs 话题的验证链；两个执行开关默认均关闭。"""
    start_camera = LaunchConfiguration('start_camera')
    execute_line_motion = LaunchConfiguration('execute_line_motion')
    execute_phase3 = LaunchConfiguration('execute_phase3')
    camera_device = LaunchConfiguration('camera_device')
    sdk_interface = LaunchConfiguration('sdk_interface')
    line_speed = LaunchConfiguration('line_speed_mps')
    camera_width = LaunchConfiguration('camera_width')
    camera_height = LaunchConfiguration('camera_height')
    camera_fps = LaunchConfiguration('camera_fps')
    line_dark_threshold = LaunchConfiguration('line_dark_threshold')
    line_kp_lateral = LaunchConfiguration('line_kp_lateral')
    line_kp_heading = LaunchConfiguration('line_kp_heading')
    line_max_wz = LaunchConfiguration('line_max_wz_radps')
    t_confirm_frames = LaunchConfiguration('t_confirm_frames')
    t_dark_threshold = LaunchConfiguration('t_dark_threshold')
    t_band_min_width_ratio = LaunchConfiguration('t_band_min_width_ratio')
    t_lane_min_width_ratio = LaunchConfiguration('t_lane_min_width_ratio')
    t_lane_max_width_ratio = LaunchConfiguration('t_lane_max_width_ratio')
    zero_confirm_messages = LaunchConfiguration('zero_confirm_messages')
    zero_confirm_timeout_sec = LaunchConfiguration('zero_confirm_timeout_sec')
    stairs_up_speed = LaunchConfiguration('stairs_up_speed_mps')
    stairs_up_duration = LaunchConfiguration('stairs_up_duration_sec')
    stairs_turn_angle = LaunchConfiguration('stairs_turn_angle_deg')
    stairs_turn_wz = LaunchConfiguration('stairs_turn_wz_radps')
    stairs_down_speed = LaunchConfiguration('stairs_down_speed_mps')
    stairs_down_duration = LaunchConfiguration('stairs_down_duration_sec')
    inter_stage_stop = LaunchConfiguration('inter_stage_stop_sec')
    line_nav_config = PathJoinSubstitution([
        FindPackageShare('rk_bringup'), 'config', 'line_nav_params.yaml'])
    bridge_prefix = FindPackagePrefix('rk_go2_sdk_bridge')
    sdk_server = PathJoinSubstitution([
        bridge_prefix, 'lib', 'rk_go2_sdk_bridge', 'go2_sdk_udp_server'])
    phase3_helper = PathJoinSubstitution([
        bridge_prefix, 'lib', 'rk_go2_sdk_bridge', 'go2_sdk_stair_classic_action'])
    sdk_runtime_dir = PathJoinSubstitution([
        bridge_prefix, 'lib', 'rk_go2_sdk_bridge', 'unitree_sdk_runtime'])

    return LaunchDescription([
        DeclareLaunchArgument('start_camera', default_value='false', description='是否启动独立 USB 相机；已有 line_camera_node 时保持 false。'),
        # /dev/videoN 会动态变化；序列号稳定链接避免重启后误开另一台相机。
        DeclareLaunchArgument(
            'camera_device',
            default_value=(
                '/dev/v4l/by-id/'
                'usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera_SN0001-video-index0'
            ),
            description='楼梯 Sonix 相机稳定设备链接，当前实际映射到 /dev/video6。'
        ),
        DeclareLaunchArgument('camera_width', default_value='320', description='相机宽度 px。'),
        DeclareLaunchArgument('camera_height', default_value='240', description='相机高度 px。'),
        DeclareLaunchArgument('camera_fps', default_value='15.0', description='相机帧率 Hz。'),
        DeclareLaunchArgument('sdk_interface', default_value='eth1', description='Go2 SDK 控制网卡。'),
        DeclareLaunchArgument('line_speed_mps', default_value='0.25', description='T 前持续巡线速度（m/s），范围 0..0.25。'),
        DeclareLaunchArgument('line_dark_threshold', default_value='70', description='巡线黑色二值阈值。'),
        DeclareLaunchArgument('line_kp_lateral', default_value='0.85', description='巡线横向偏差转向增益。'),
        DeclareLaunchArgument('line_kp_heading', default_value='0.35', description='巡线朝向偏差转向增益。'),
        DeclareLaunchArgument('line_max_wz_radps', default_value='0.28', description='巡线最大转向速度 rad/s。'),
        DeclareLaunchArgument('t_confirm_frames', default_value='5', description='T 标记连续确认帧数。'),
        DeclareLaunchArgument('t_dark_threshold', default_value='70', description='T 标记黑色阈值。'),
        DeclareLaunchArgument('t_band_min_width_ratio', default_value='0.65', description='T 顶部横带最小宽度比例。'),
        DeclareLaunchArgument('t_lane_min_width_ratio', default_value='0.04', description='T 下方竖带最小宽度比例。'),
        DeclareLaunchArgument('t_lane_max_width_ratio', default_value='0.28', description='T 下方竖带最大宽度比例。'),
        DeclareLaunchArgument('zero_confirm_messages', default_value='3', description='交接前连续最终零速度消息数。'),
        DeclareLaunchArgument('zero_confirm_timeout_sec', default_value='2.0', description='等待最终零速度超时 s。'),
        # 以下七项就是 T 型标记确认后的完整 Phase 3 标定表，均不会影响 T 前巡线。
        DeclareLaunchArgument('stairs_up_speed_mps', default_value='0.55', description='T 后上台阶前进速度（m/s）。'),
        DeclareLaunchArgument('stairs_up_duration_sec', default_value='3.9', description='T 后上台阶前进时间（s）。'),
        DeclareLaunchArgument('stairs_turn_angle_deg', default_value='79.0', description='T 后左转目标角度（度）。'),
        DeclareLaunchArgument('stairs_turn_wz_radps', default_value='1.0', description='T 后左转角速度（rad/s）。'),
        DeclareLaunchArgument('stairs_down_speed_mps', default_value='0.55', description='T 后下台阶前进速度（m/s）。'),
        DeclareLaunchArgument('stairs_down_duration_sec', default_value='2.9', description='T 后下台阶前进时间（s）。'),
        DeclareLaunchArgument('inter_stage_stop_sec', default_value='0.4', description='Phase 3 段间 StopMove 后稳定时间（s）。'),
        DeclareLaunchArgument('execute_line_motion', default_value='false', description='true 才允许巡线控制真机。'),
        DeclareLaunchArgument('execute_phase3', default_value='false', description='true 才允许 T 后执行 Phase 3。'),
        LogInfo(msg='独立楼梯验证：话题使用 /stairs/*；默认只观测，不会移动机器狗。'),
        Node(
            package='rk_bringup', executable='line_camera_node', name='stair_line_camera_node', output='screen',
            condition=IfCondition(start_camera),
            parameters=[{'device': camera_device,
                         'width': ParameterValue(camera_width, value_type=int),
                         'height': ParameterValue(camera_height, value_type=int),
                         'fps': ParameterValue(camera_fps, value_type=float)}]),
        Node(
            package='rk_perception', executable='real_line_tracker_node', name='stair_real_line_tracker_node', output='screen',
            parameters=[line_nav_config, {
                'image_topic': '/line_camera/image_raw', 'line_track_topic': '/stairs/perception/line_track',
                'threshold_value': ParameterValue(line_dark_threshold, value_type=int),
                'enable_debug_image': False, 'debug_log': False}]),
        Node(
            package='rk_navigation', executable='line_follower_node', name='stair_line_follower_node', output='screen',
            parameters=[line_nav_config, {
                'line_track_topic': '/stairs/perception/line_track', 'suggested_cmd_topic': '/stairs/control/line_cmd',
                'line_follow_status_topic': '/stairs/line_follow_status', 'mission_start_topic': '/stairs/line_start',
                'mission_stop_topic': '/stairs/line_stop', 'gait_control_lock_topic': '/stairs/line_lock',
                # T 前所有巡线速度统一为 0.25；此处不设时长，直到门禁确认才停车。
                'min_driving_speed': ParameterValue(line_speed, value_type=float),
                'base_speed': ParameterValue(line_speed, value_type=float),
                'mid_speed': ParameterValue(line_speed, value_type=float),
                'slow_speed': ParameterValue(line_speed, value_type=float),
                'kp_lateral': ParameterValue(line_kp_lateral, value_type=float),
                'kp_heading': ParameterValue(line_kp_heading, value_type=float),
                'max_angular_z': ParameterValue(line_max_wz, value_type=float),
                'debug_log': False}]),
        Node(
            package='rk_safety', executable='command_mux_node', name='stair_command_mux_node', output='screen',
            parameters=[{
                'line_cmd_topic': '/stairs/control/line_cmd', 'mission_cmd_topic': '/stairs/control/mission_cmd',
                'locomotion_cmd_topic': '/stairs/control/locomotion_cmd', 'output_cmd_topic': '/stairs/navigation/cmd_vel',
                'gait_lock_topic': '/stairs/line_lock', 'arm_lock_topic': '/stairs/arm_lock',
                'estop_topic': '/stairs/estop', 'estop_state_topic': '/stairs/estop_state',
                'estop_service_name': '/stairs/estop', 'max_linear_x': 0.25, 'max_linear_y': 0.05,
                'max_angular_z': 0.60}]),
        ExecuteProcess(
            cmd=[sdk_server, '--interface', sdk_interface, '--listen-ip', '127.0.0.1', '--port', '15101',
                 '--rate-hz', '20.0', '--watchdog-sec', '0.30', '--max-vx', '0.25', '--max-vy', '0.05',
                 '--max-yaw', '0.60', '--deadband', '0.01'], output='screen',
            condition=IfCondition(execute_line_motion),
            additional_env={'LD_LIBRARY_PATH': PathJoinSubstitution([bridge_prefix, 'lib'])}),
        Node(
            package='rk_go2_sdk_bridge', executable='cmd_vel_udp_forwarder.py', name='stair_cmd_vel_udp_forwarder', output='screen',
            condition=IfCondition(execute_line_motion), additional_env={'LD_LIBRARY_PATH': sdk_runtime_dir},
            parameters=[{'cmd_vel_topic': '/stairs/navigation/cmd_vel', 'udp_port': 15101, 'status_port': 15102,
                         'max_vx': 0.25, 'max_vy': 0.05, 'max_yaw': 0.60, 'timeout_sec': 0.30,
                         'status_topic': '/stairs/go2/sdk_motion_status',
                         'status_ready_topic': '/stairs/go2/sdk_motion_status_receiver_ready'}]),
        Node(
            package='rk_bringup', executable='stair_line_handoff_node', name='stair_line_handoff_node', output='screen',
            parameters=[{
                'image_topic': '/line_camera/image_raw', 'final_cmd_topic': '/stairs/navigation/cmd_vel',
                'line_start_topic': '/stairs/line_start', 'line_stop_topic': '/stairs/line_stop',
                'gait_lock_topic': '/stairs/line_lock', 'status_topic': '/stairs/handoff_status',
                'execute_line_motion': ParameterValue(execute_line_motion, value_type=bool),
                'execute_phase3': ParameterValue(execute_phase3, value_type=bool),
                'line_speed_mps': ParameterValue(line_speed, value_type=float), 'phase3_helper': phase3_helper,
                'sdk_interface': sdk_interface,
                't_confirm_frames': ParameterValue(t_confirm_frames, value_type=int),
                'dark_threshold': ParameterValue(t_dark_threshold, value_type=int),
                'band_min_width_ratio': ParameterValue(t_band_min_width_ratio, value_type=float),
                'lane_min_width_ratio': ParameterValue(t_lane_min_width_ratio, value_type=float),
                'lane_max_width_ratio': ParameterValue(t_lane_max_width_ratio, value_type=float),
                'zero_confirm_messages': ParameterValue(zero_confirm_messages, value_type=int),
                'zero_confirm_timeout_sec': ParameterValue(zero_confirm_timeout_sec, value_type=float),
                'stairs_up_speed_mps': ParameterValue(stairs_up_speed, value_type=float),
                'stairs_up_duration_sec': ParameterValue(stairs_up_duration, value_type=float),
                'stairs_turn_angle_deg': ParameterValue(stairs_turn_angle, value_type=float),
                'stairs_turn_wz_radps': ParameterValue(stairs_turn_wz, value_type=float),
                'stairs_down_speed_mps': ParameterValue(stairs_down_speed, value_type=float),
                'stairs_down_duration_sec': ParameterValue(stairs_down_duration, value_type=float),
                'inter_stage_stop_sec': ParameterValue(inter_stage_stop, value_type=float)}]),
    ])
