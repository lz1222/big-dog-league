from setuptools import setup

package_name = 'rk_navigation'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ZhenLi',
    maintainer_email='2605128876@qq.com',
    description='rk_navigation package',
    license='MIT',
    entry_points={
    'console_scripts': [
        'line_follower_node = rk_navigation.line_follower_node:main',
    ],
},
)
