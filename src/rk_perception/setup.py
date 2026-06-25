import os
from glob import glob

from setuptools import setup


package_name = 'rk_perception'

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
    tests_require=['pytest'],
    maintainer='ZhenLi',
    maintainer_email='2605128876@qq.com',
    description='Mock perception publishers for the RK inspection robot.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'mock_line_tracker_node = '
            'rk_perception.mock_line_tracker_node:main',
            'real_line_tracker_node = '
            'rk_perception.real_line_tracker_node:main',
            'mock_sign_detector_node = '
            'rk_perception.mock_sign_detector_node:main',
            'mock_item_tag_node = rk_perception.mock_item_tag_node:main',
        ],
    },
)
