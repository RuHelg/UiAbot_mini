from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import subprocess
import tempfile

def generate_launch_description():

    # Directory paths internal to uiabot_mini package
    bringup_dir     = get_package_share_directory('uiabot_mini')              # Path to uiabot_mini package
    config_dir      = os.path.join(bringup_dir, 'config')                     # Path to config directory
    description_pkg = get_package_share_directory('uiabot_mini_description')  # Path to description package

    # File paths internal to uiabot_mini package
    bno055_params = os.path.join(config_dir, 'bno055_params_i2c.yaml') # Path to BNO055 params file
    xacro_file    = os.path.join(description_pkg, 'urdf', 'uiabot_differential.xacro')
    urdf_output   = os.path.join(tempfile.gettempdir(), 'uiabot_differential_merged.urdf')
    
    # Run xacro to generate URDF
    try:
        result = subprocess.run(
            ['xacro', xacro_file, '-o', urdf_output],
            check=True,
            capture_output=True,
            text=True
        )
    except subprocess.CalledProcessError as e:
        raise RuntimeError(
            f"xacro generation failed for {xacro_file}\n"
            f"Error: {e.stderr}"
        ) from e

    # Read the generated URDF
    with open(urdf_output, 'r') as urdf_file_handle:
        urdf_content = urdf_file_handle.read()

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
        parameters=[{'robot_description': urdf_content}],)

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