from setuptools import setup


package_name = 'rk_mission'

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
    description='Mock competition mission state machine.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'mission_state_machine_node = rk_mission.mission_state_machine_node:main',
            'sign_action_executor_node = rk_mission.sign_action_executor_node:main',
        ],
    },
)
