from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():
    # Package paths
    ros_gz_sim_pkg = get_package_share_directory('ros_gz_sim')
    uiabot_gazebo_pkg = get_package_share_directory('uiabot_mini_gazebo')
    pkg_share = get_package_share_directory('xbox_wsl')
    
    # File paths
    gz_launch_path = os.path.join(ros_gz_sim_pkg, 'launch', 'gz_sim.launch.py')
    world_file = os.path.join(uiabot_gazebo_pkg, 'worlds', 'simple_world.sdf')
    urdf_file = os.path.join(pkg_share, 'urdf', 'test_robot.urdf.xacro')
    
    # Process xacro to urdf
    robot_description = xacro.process_file(urdf_file).toxml()

    # Start Gazebo with world file
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
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}]
    )
    
    # Spawn Robot in Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-string', robot_description, '-name', 'test_robot', '-z', '0.2'],
        output='screen'
    )
    
    spawn_robot_delayed = TimerAction(
        period=2.0,
        actions=[spawn_robot]
    )
    
    # Bridge for cmd_vel
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot_delayed,
        bridge,
    ])