import os
from glob import glob

from setuptools import setup


package_name = 'rk_locomotion'

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
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='ZhenLi',
    maintainer_email='2605128876@qq.com',
    description='Basic gait control layer for the RK inspection robot.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'gait_control_node = rk_locomotion.gait_control_node:main',
            'gait_basic_test_node = rk_locomotion.gait_basic_test_node:main',
        ],
    },
)
