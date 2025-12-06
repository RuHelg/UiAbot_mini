from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

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
        name='serial_communication'
    )

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

    return LaunchDescription([
        # Core robot nodes
        sc,
        rsp,

        # Sensors
        sllidar_launch,
    ])