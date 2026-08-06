from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """仅启动服务客户端入口；它没有 DDS command writer。"""
    return LaunchDescription([Node(package='rk_arm', executable='arm_manual_control_node', name='arm_manual_control_node')])
