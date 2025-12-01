from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription, DeclareLaunchArgument, TimerAction, RegisterEventHandler, ExecuteProcess
from launch.event_handlers import OnProcessExit, OnShutdown, OnProcessStart
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, AndSubstitution, NotSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
import subprocess
import tempfile
import time


def generate_launch_description():
    # Launch arguments
    use_rviz = LaunchConfiguration('rviz')
    run_slam = LaunchConfiguration('run_slam')
    run_nav = LaunchConfiguration('run_nav')
    
    # Package paths
    ros_gz_sim_pkg = get_package_share_directory('ros_gz_sim')
    uiabot_mini_gazebo_pkg = get_package_share_directory('uiabot_mini_gazebo')
    uiabot_mini_description_pkg = get_package_share_directory('uiabot_mini_description')
    uiabot_mini_bringup_pkg = get_package_share_directory('uiabot_mini_bringup')
    
    # Set GZ_SIM_RESOURCE_PATH to include BOTH:
    # 1. The install share directory (for model://uiabot_mini_description/meshes)
    # 2. The models directory (for custom Gazebo models)
    models_dir = os.path.join(uiabot_mini_gazebo_pkg, 'models')
    description_share_parent = os.path.dirname(uiabot_mini_description_pkg)
    
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.pathsep.join(filter(None, [
            os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
            description_share_parent,  # For model://uiabot_mini_description/meshes
            models_dir,                # For custom Gazebo models
        ]))
    )

    # File paths
    gz_launch_path = os.path.join(ros_gz_sim_pkg, 'launch', 'gz_sim.launch.py')
    world_file = os.path.join(uiabot_mini_gazebo_pkg, 'worlds', 'simple_world.sdf')
    # Path to the XACRO file
    xacro_file = 'src/uiabot_mini_gazebo/urdf/uiabot_mini.xacro'
    # Path for the generated URDF file
    urdf_output = 'src/uiabot_mini_gazebo/urdf/merged.urdf'

    # Run xacro now (synchronously) so we have the URDF contents available for nodes
    try:
        subprocess.run(['xacro', xacro_file, '-o', urdf_output], check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    except subprocess.CalledProcessError as e:
        raise RuntimeError(f"xacro generation failed:\n{e.stderr}") from e

    # Read the generated URDF and provide its contents to robot_state_publisher
    with open(urdf_output, 'r') as urdf_file_handle:
        urdf_content = urdf_file_handle.read()

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

    # Nodes
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
            'slam_params_file': os.path.join(uiabot_mini_bringup_pkg, 'config', 'slam_params.yaml'),
            'use_sim_time': 'true'
        }.items(),
        condition=IfCondition(run_slam)
    )

    nav2_params = os.path.join(uiabot_mini_bringup_pkg, 'config', 'nav2_params.yaml')
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
        parameters=[os.path.join(uiabot_mini_bringup_pkg, 'config', 'ekf.yaml'), {'use_sim_time': True}],
    )

    return LaunchDescription([
        # Set Gazebo resource path (combines description meshes + models dir)
        gz_resource_path,
        # Declare launch arguments
        DeclareLaunchArgument('rviz', default_value='false', description='Start RViz'),
        DeclareLaunchArgument('run_slam', default_value='false'),
        DeclareLaunchArgument('run_nav', default_value='false'),
        DeclareLaunchArgument('map', default_value=os.path.expanduser('~/ros2_ws/maps/my_map.yaml')),
        
        gazebo,
        robot_state_publisher,
        spawn_robot_delayed,
        bridge,
        ekf_node,
        slam_launch,
        nav2_launch,
        rviz_node,
    ])