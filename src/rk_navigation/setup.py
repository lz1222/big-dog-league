from setuptools import setup


package_name = 'rk_navigation'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ZhenLi',
    maintainer_email='2605128876@qq.com',
    description='Mock line following navigation for the RK inspection robot.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'line_follower_node = rk_navigation.line_follower_node:main',
        ],
    },
)
