from setuptools import find_packages, setup

package_name = 'xbox_wsl'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='stiank',
    maintainer_email='stiank@todo.todo',
    description='Xbox controller driver for WSL2 + ROS2',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'xbox_joy_node = xbox_wsl.xbox_joy_node:main',
        ],
    },
)
