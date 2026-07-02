#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


SDK_SERVER = '/home/unitree/unitree_go2_sdk_test/build/go2_sdk_udp_server'
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
    return LaunchDescription([
        ExecuteProcess(
            cmd=[SDK_SERVER],
            output='screen',
            additional_env=SDK_SERVER_ENV,
        ),
        Node(
            package='rk_go2_sdk_bridge',
            executable='cmd_vel_udp_forwarder.py',
            name='cmd_vel_udp_forwarder',
            output='screen',
            additional_env=FORWARDER_ENV,
        ),
    ])
