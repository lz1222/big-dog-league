from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """默认只启动反馈订阅与 ROS 服务；配置显式关闭所有运动。"""
    config = os.path.join(get_package_share_directory('rk_arm'), 'config', 'arm_driver.yaml')
    return LaunchDescription([Node(package='rk_arm', executable='d1_dds_driver_node', name='d1_dds_driver_node', parameters=[config])])
