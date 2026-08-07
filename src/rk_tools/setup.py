from setuptools import find_packages, setup


package_name = 'rk_tools'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ZhenLi',
    maintainer_email='2605128876@qq.com',
    description='Mock hardware tools for the RK inspection robot.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'mock_locomotion_server = rk_tools.mock_locomotion_server:main',
            'mock_arm_server = rk_tools.mock_arm_server:main',
            'safety_node = rk_tools.safety_node:main',
            'mission_client_node = rk_tools.mission_client_node:main',
            'two_step_walk_test_node = rk_tools.two_step_walk_test_node:main',
            'cmd_vel_speed_sweep_node = '
            'rk_tools.cmd_vel_speed_sweep_node:main',
            'obstacle_direct_route_node = '
            'rk_tools.obstacle_direct_route_node:main',
            'depth_wall_distance_node = '
            'rk_tools.depth_wall_distance_node:main',
            'keyboard_route_node = rk_tools.keyboard_route_node:main',
            'keyboard_route_recorder = '
            'rk_tools.keyboard_route_node:record_main',
            'keyboard_route_replay = rk_tools.keyboard_route_node:replay_main',
            'national_route_simulator = '
            'rk_tools.national_route_simulator:main',
            'run_national_sim_suite = '
            'rk_tools.run_national_sim_suite:main',
        ],
    },
)
