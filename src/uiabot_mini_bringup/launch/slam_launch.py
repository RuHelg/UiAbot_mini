from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    # File paths to uiabot_mini_bringup package
    bringup_dir   = get_package_share_directory('uiabot_mini_bringup') # Path to bringup package
    config_dir    = os.path.join(bringup_dir, 'config')                # Path to config directory

    # File paths, internal to uiabot_mini_bringup package
    bno055_params = os.path.join(config_dir, 'bno055_params_i2c.yaml') # Path to BNO055 params file
    ekf_config    = os.path.join(config_dir, 'ekf.yaml')               # Path to ekf config file
    slam_params   = os.path.join(config_dir, 'slam_params.yaml')       # Path to SLAM params file
    

    # File paths, external packages
    # URDF
    urdf_path = os.path.join(
        get_package_share_directory('uiabot_mini_description'),
        'urdf',
        'uiabot_mini.urdf')


    # SLLIDAR (from sllidar_ros2 package)
    sllidar_launch_file = os.path.join(
        get_package_share_directory('sllidar_ros2'),
        'launch',
        'sllidar_a1_launch.py'
    )

    # Serial Communication Node (custom)
    sc = Node(
        package='uiabot_mini',
        executable='serial_communication',
        name='serial_communication',
        parameters=[{'serial_timeout': 0.2,
                     'read_feedback_hz': 100.0,
                     'cmd_vel_send_delay': 0.0}])

    # Robot State Publisher Node (built-in)
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': open(urdf_path).read()}])
    
    # Sllidar launch, forwarding launch args (git-cloned)
    sllidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sllidar_launch_file),
        launch_arguments={
            'serial_port': '/dev/ttyUSB0',
            'frame_id': 'lidar_link',
            'angle_compensate': 'true'
        }.items(),
    )

    # BNO055 IMU Node (built-in)
    bno055_node = Node(
        package='bno055',
        executable='bno055',
        name='bno055',
        output='screen',
        parameters=[bno055_params],
    )

    # EKF Node from robot_localization (built-in)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[ekf_config],
        output='screen',
    )

    # SLAM Toolbox Node (built-in)
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch',
                'online_async_launch.py'
            )
        ),
        launch_arguments={
            'slam_params_file': slam_params,
        }.items(),
    )

    return LaunchDescription([
        # Core robot nodes
        sc,
        rsp,
        ekf_node,

        # Sensors
        sllidar_launch,
        bno055_node,

        # SLAM
        slam_launch,
    ])