from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    return LaunchDescription([Node(package='rk_maze', executable='realtime_maze_controller',
        name='realtime_maze_controller', output='screen',
        parameters=[{'dry_run': True, 'enable_motion': False, 'armed': False}])])
