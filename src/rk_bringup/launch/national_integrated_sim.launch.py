"""One-command no-hardware national mission simulation launch entrypoint."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    scenario = LaunchConfiguration('scenario')
    output_dir = LaunchConfiguration('output_dir')
    return LaunchDescription([
        DeclareLaunchArgument(
            'scenario', default_value='marker1_radiation',
            description='Nominal or injected-fault national simulation scenario.',
        ),
        DeclareLaunchArgument(
            'output_dir', default_value='',
            description='Optional /tmp/national_integrated_sim_* output path.',
        ),
        Node(
            package='rk_tools',
            executable='national_route_simulator',
            name='national_route_simulator',
            output='screen',
            arguments=['--scenario', scenario, '--output-dir', output_dir],
        ),
    ])
