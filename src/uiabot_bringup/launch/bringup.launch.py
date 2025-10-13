from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directories using ROS2 package system
    bringup_dir = get_package_share_directory('uiabot_bringup')
    urdf_file = os.path.join(bringup_dir, 'urdf', 'uiabot_mini.urdf')
    ekf_config = os.path.join(bringup_dir, 'config', 'ekf.yaml')
    
    with open(urdf_file, 'r', encoding='utf-8') as f:
        robot_description = f.read()

    return LaunchDescription([
        # Robot State Publisher - Publishes robot model from URDF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': False
            }]
        ),
        
        # Robot Localization (EKF) - Fuses IMU data for orientation
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config]
        ),
        
        # Teleop to Serial Communication
        Node(
            package='serial_communication',
            executable='teleop_to_serial',
            name='teleop_to_serial_node',
            output='screen'
        ),
        
        # Wheel State Publisher - Publishes joint_states for URDF
        Node(
            package='serial_communication',
            executable='wheel_state_publisher',
            name='wheel_state_publisher_node',
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
        
        # Static world frame - Fixed reference point for mapping
        # world is the absolute fixed frame, map will be published by SLAM relative to world
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_map_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'map'],
            output='screen'
        ),

    ])
