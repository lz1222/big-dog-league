from setuptools import setup

package_name = 'rk_perception'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ZhenLi',
    maintainer_email='2605128876@qq.com',
    description='rk_perception package',
    license='MIT',
    entry_points={
    'console_scripts': [
        'mock_line_tracker_node = rk_perception.mock_line_tracker_node:main',
    ],
},
)
