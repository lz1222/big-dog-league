import os
from glob import glob

from setuptools import setup


package_name = 'rk_arm_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ZhenLi',
    maintainer_email='2605128876@qq.com',
    description='Fixed-position arm task control for the RK inspection robot.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'arm_task_node = rk_arm_control.arm_task_node:main',
            'd1_pick_node = rk_arm_control.d1_pick_node:main',
        ],
    },
)
