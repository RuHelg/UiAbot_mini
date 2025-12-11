from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
import subprocess
import tempfile


def generate_launch_description():
    # Package paths
    ros_gz_sim_pkg = get_package_share_directory('ros_gz_sim')
    gazebo_pkg = get_package_share_directory('uiabot_mini_gazebo')
    bringup_pkg   = get_package_share_directory('uiabot_mini') # Path to bringup package
    description_pkg = get_package_share_directory('uiabot_mini_description')
    
    # Directory paths
    config_dir    = os.path.join(bringup_pkg, 'config')                # Path to config directory
    gazebo_config_dir = os.path.join(gazebo_pkg, 'config')          # Path to gazebo config directory

    # File paths, internal to uiabot_mini_bringup package
    bno055_params = os.path.join(config_dir, 'bno055_params_i2c.yaml') # Path to BNO055 params file
    ekf_config    = os.path.join(config_dir, 'ekf.yaml')               # Path to ekf config file
    slam_params   = os.path.join(config_dir, 'slam_params.yaml')       # Path to SLAM params file
    nav2_params   = os.path.join(config_dir, 'nav2_params_differential.yaml')       # Path to Nav2 params file

    gz_launch_path = os.path.join(ros_gz_sim_pkg, 'launch', 'gz_sim.launch.py')
    world_file = os.path.join(gazebo_pkg, 'worlds', 'simple_world.sdf')
    xacro_file = os.path.join(gazebo_pkg, 'urdf', 'uiabot_gazebo_differential.urdf')
    
    urdf_output = os.path.join(tempfile.gettempdir(), 'uiabot_gazebo_merged.urdf')

    # Launch arguments
    use_rviz = LaunchConfiguration('rviz')
    run_slam = LaunchConfiguration('run_slam')
    run_nav = LaunchConfiguration('run_nav')
    
    # Set GZ_SIM_RESOURCE_PATH for model://uiabot_mini_description/meshes
    description_share_parent = os.path.dirname(description_pkg)
    
    # Build resource path once
    gz_resource_path = os.pathsep.join(filter(None, [
        os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
        description_share_parent,
    ]))
    
    # Set up environment for xacro to find includes from other packages
    env = os.environ.copy()
    env['ROS_PACKAGE_PATH'] = os.pathsep.join(filter(None, [
        os.path.dirname(gazebo_pkg),
        os.path.dirname(description_pkg),
        env.get('ROS_PACKAGE_PATH', ''),
    ]))
    env['GZ_SIM_RESOURCE_PATH'] = gz_resource_path

    xacro_cmd = ['xacro', xacro_file, '-o', urdf_output]

    try:
        subprocess.run(xacro_cmd, check=True, capture_output=True, text=True, env=env)
    except subprocess.CalledProcessError as e:
        raise RuntimeError(
            f"xacro generation failed for {xacro_file}\n"
            f"Command: {' '.join(xacro_cmd)}\n"
            f"Error: {e.stderr}"
        ) from e

    # Read the generated URDF
    with open(urdf_output, 'r') as urdf_file_handle:
        urdf_content = urdf_file_handle.read()

    # Nodes
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': urdf_content, 'use_sim_time': True}]
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', urdf_output,
            '-name', 'uiabot_mini',
            '-z', '0.1'
        ],
        output='screen'
    )

    gazebo_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch_path),
        launch_arguments={
            'gz_args': world_file,
            'on_exit_shutdown': 'True'
        }.items(),
    )
    
    gazebo = TimerAction(
        period=0.5,
        actions=[gazebo_include]
    )
    
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/wheel_encoder_odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/bno055/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    
    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    spawn_robot_delayed = TimerAction(
        period=2.0,
        actions=[spawn_robot]
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'slam_params_file': slam_params,
            'use_sim_time': 'true'
        }.items(),
        condition=IfCondition(run_slam)
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'params_file': nav2_params,
            'use_sim_time': 'true'
        }.items(),
        condition=IfCondition(run_nav)
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_config,
            {'use_sim_time': True}
        ],
    )

    return LaunchDescription([
        SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=gz_resource_path),
        DeclareLaunchArgument('rviz', default_value='false', description='Start RViz'),
        DeclareLaunchArgument('run_slam', default_value='false', description='Start SLAM Toolbox'),
        DeclareLaunchArgument('run_nav', default_value='false', description='Start Nav2'),
        gazebo,
        robot_state_publisher,
        spawn_robot_delayed,
        bridge,
        ekf_node,
        slam_launch,
        nav2_launch,
        rviz_node,
    ])