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
    

    # File paths, external packages
    # URDF
    urdf_path = os.path.join(
        get_package_share_directory('uiabot_mini_description'),
        'urdf',
        'uiabot_mini.urdf')

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

    # BNO055 IMU Node (built-in)
    bno055_node = Node(
        package='bno055',
        executable='bno055',
        name='bno055',
        output='screen',
        parameters=[bno055_params],
    )

    return LaunchDescription([
        # Core robot nodes
        sc,
        rsp,

        # Sensors
        bno055_node,
    ])