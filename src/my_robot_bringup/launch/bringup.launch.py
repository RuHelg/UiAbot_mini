from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot_bringup')
    config = os.path.join(pkg_share, 'config', 'ekf.yaml')
    urdf_file = os.path.join(pkg_share, 'urdf', 'uiabot_mini.urdf')
    robot_description = open(urdf_file, 'r', encoding='utf-8').read()

    # config = os.path.join(
    #     get_package_share_directory('my_robot_bringup'),
    #     'config',
    #     'ekf.yaml'
    # )

    # pkg_share = get_package_share_directory('my_robot_bringup')
    # urdf_file = os.path.join(pkg_share, 'urdf', 'uiabot_mini.urdf')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()
    
    # ========================================
    # Robot Frame Positions (in meters)
    # ========================================
    # Laser (RPLidar A1)
    laser_x = 0.0
    laser_y = 0.0
    laser_z = 0.1
    
    # IMU (BNO055)
    imu_x = 71.75e-3
    imu_y = 58.75e-3
    imu_z = 110.35e-3
    
    # Wheels
    wheel_right_x = -0.6e-3
    wheel_right_y = 98.75e-3
    wheel_right_z = 34.0e-3
    wheel_left_x = -0.6e-3
    wheel_left_y = -98.75e-3
    wheel_left_z = 34.0e-3

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
            output='screen',
            parameters=[{
                'wheel_right_x': wheel_right_x,
                'wheel_right_y': wheel_right_y,
                'wheel_right_z': wheel_right_z,
                'wheel_left_x': wheel_left_x,
                'wheel_left_y': wheel_left_y,
                'wheel_left_z': wheel_left_z
            }]
        ),
        
        # Wheel State Publisher - Publishes joint_states for URDF
        Node(
            package='teleop_to_serial',
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
        
        # Static Transform Publishers for TF tree
        # Transform from base_link to laser frame (RPLidar)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser_tf',
            arguments=[str(laser_x), str(laser_y), str(laser_z), '1.5708', '0', '0', 'base_link', 'laser'],
            output='screen'
        ),
        
        # Transform from base_link to IMU frame (BNO055)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_imu_tf', 
            arguments=[str(imu_x), str(imu_y), str(imu_z), '1.5708', '0', '0', 'base_link', 'bno055'],
            output='screen'
        ),

        # Node(
        # package='robot_state_publisher',
        # executable='robot_state_publisher',
        # name='robot_state_publisher',
        # output='screen',
        # parameters=[{'robot_description': robot_description}],
        # ),

        # Node(
        # package='joint_state_publisher_gui',
        # executable='joint_state_publisher_gui',
        # name='joint_state_publisher_gui',
        # output='screen',
        # )
    ])
