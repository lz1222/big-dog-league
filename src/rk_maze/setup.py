from setuptools import find_packages, setup
setup(name='rk_maze', version='0.1.0', packages=find_packages(exclude=['test']),
      data_files=[('share/rk_maze', ['package.xml']),
                  ('share/rk_maze/config', ['config/lidar_distance.yaml', 'config/joint_health_guard.yaml', 'config/maze_realtime_fusion.yaml']),
                  ('share/rk_maze/launch', ['launch/maze_dry_run.launch.py'])],
      install_requires=['setuptools'], zip_safe=True,
      entry_points={'console_scripts': ['realtime_maze_controller = rk_maze.realtime_maze_controller:main']})
