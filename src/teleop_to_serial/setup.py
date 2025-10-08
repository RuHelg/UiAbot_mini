from setuptools import find_packages, setup

package_name = 'teleop_to_serial'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gruppe-6',
    maintainer_email='gruppe-6@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_to_serial = teleop_to_serial.teleop_to_serial:main',
            'wheel_state_publisher = teleop_to_serial.wheel_state_publisher:main',
            'differential_drive_odometry = teleop_to_serial.differential_drive_odometry:main',
        ],
    },
)
