from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('my_robot_bringup'),
        'config',
        'ekf.yaml'
    )

    return LaunchDescription([
        # Robot Localization (EKF)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[config]
        ),
        
        # Teleop to Serial Communication
        Node(
            package='teleop_to_serial',
            executable='teleop_to_serial',
            name='teleop_to_serial_node',
            output='screen'
        ),
        
        # RPLidar A1 - Using proper launch file on ttyUSB0
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('rplidar_ros'),
                '/launch/rplidar_a1_launch.py'
            ]),
            launch_arguments={
                'serial_port': '/dev/ttyUSB0'
            }.items()
        ),
        
        # BNO055 IMU - Using proper launch file with I2C configuration
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('bno055'),
                '/launch/bno055.launch.py'
            ])
        ),
        
        # Static Transform Publishers for TF tree
        # Transform from base_link to laser frame (RPLidar)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser_tf',
            arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser'],
            output='screen'
        ),
        
        # Transform from base_link to IMU frame (BNO055)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_imu_tf', 
            arguments=['0', '0', '0.05', '0', '0', '0', 'base_link', 'bno055'],
            output='screen'
        )
    ])
