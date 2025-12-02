from setuptools import find_packages, setup

package_name = 'uiabot_mini_core'

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
    maintainer='gruppe-6',
    maintainer_email='gruppe-6@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wheel_joint_state_publisher = uiabot_mini_core.wheel_joint_state_publisher:main',
            'teleop_to_serial = uiabot_mini_core.teleop_to_serial:main',
            'wheel_encoder_odometry = uiabot_mini_core.wheel_encoder_odometry:main',
        ],
    },
)
