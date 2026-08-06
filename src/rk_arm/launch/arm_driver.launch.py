from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """分离 Unitree DDS reader 与 ROS 状态节点，避免在同一进程混载两套 DDS。"""
    config = os.path.join(get_package_share_directory('rk_arm'), 'config', 'arm_driver.yaml')
    return LaunchDescription([
        Node(package='rk_arm', executable='d1_dds_driver_node', name='d1_dds_driver_node',
             arguments=['--network-interface', 'eth0', '--state-socket', '/tmp/rk_d1_arm_feedback.sock']),
        Node(package='rk_arm', executable='arm_manual_control_node', name='arm_manual_control_node',
             parameters=[config], additional_env={'ROS_DOMAIN_ID': '42'}),
    ])
