from setuptools import setup


package_name = 'rk_safety'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ZhenLi',
    maintainer_email='2605128876@qq.com',
    description='Safety command arbitration for the RK inspection robot.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'command_mux_node = rk_safety.command_mux_node:main',
        ],
    },
)
