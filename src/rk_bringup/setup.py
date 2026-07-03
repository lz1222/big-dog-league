import os
from glob import glob

from setuptools import setup


package_name = 'rk_bringup'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        ('share/' + package_name, ['package.xml']),
        (
            'share/' + package_name,
            ['README_line_system.md']
        ),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')
        ),
        (
            os.path.join('share', package_name, 'scripts'),
            glob('scripts/*.sh')
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ZhenLi',
    maintainer_email='2605128876@qq.com',
    description='Launch files for the RK inspection robot mock system.',
    license='MIT',
    entry_points={
        'console_scripts': [],
    },
)
