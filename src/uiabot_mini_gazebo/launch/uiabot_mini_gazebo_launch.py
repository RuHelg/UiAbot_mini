from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription, DeclareLaunchArgument, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Launch arguments
    use_rviz = LaunchConfiguration('rviz')
    run_slam = LaunchConfiguration('run_slam')
    
    # Package paths
    ros_gz_sim_pkg = get_package_share_directory('ros_gz_sim')
    uiabot_mini_gazebo_pkg = get_package_share_directory('uiabot_mini_gazebo')
    uiabot_mini_description_pkg = get_package_share_directory('uiabot_mini_description')
    uiabot_mini_bringup_pkg = get_package_share_directory('uiabot_mini_bringup')
    
    # File paths
    gz_launch_path = os.path.join(ros_gz_sim_pkg, 'launch', 'gz_sim.launch.py')
    world_file = os.path.join(uiabot_mini_gazebo_pkg, 'worlds', 'simple_world.sdf')
    urdf_file = os.path.join(uiabot_mini_description_pkg, 'urdf', 'uiabot_mini.urdf')
    gazebo_file = os.path.join(uiabot_mini_description_pkg, 'urdf', 'uiabot_mini.gazebo')
    controller_config = os.path.join(uiabot_mini_gazebo_pkg, 'config', 'diff_drive_controller.yaml')
    
    # Read URDF and merge with Gazebo elements
    with open(urdf_file, 'r') as f:
        urdf_content = f.read()
    with open(gazebo_file, 'r') as f:
        gazebo_content = f.read()
    
    # Merge: remove closing </robot> from URDF, add Gazebo content, then close
    robot_desc = urdf_content.replace('</robot>', gazebo_content.replace('<?xml version="1.0"?>', '').replace('<robot name="uiabot_mini_gazebo">', '').replace('</robot>', '') + '</robot>')

    # Nodes
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch_path),
        launch_arguments={
            'gz_args': world_file,
            'on_exit_shutdown': 'True'
        }.items(),
    )
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', '/robot_description',
            '-name', 'uiabot_mini',
            '-z', '0.1'
        ],
        output='screen'
    )

    bridge_lidar = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'],
        remappings=[('/lidar', '/scan')],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    bridge_imu = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/imu@sensor_msgs/msg/Imu[gz.msgs.IMU'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    bridge_cmd_vel = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    bridge_joint_states = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    bridge_odom = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    bridge_tf = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'],
        parameters=[{'use_sim_time': True}],
        output='screen'
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
    
    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument('rviz', default_value='false', description='Start RViz'),
        DeclareLaunchArgument('run_slam', default_value='false'),
        
        # Set Gazebo resource path to find meshes and models
        SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            ':'.join([
                os.path.join(os.path.expanduser('~'), 'ros2_ws', 'src'),
                os.path.join(os.path.expanduser('~'), 'ros2_ws', 'src', 'uiabot_mini_gazebo')
            ])
        ),
        
        gazebo,
        robot_state_publisher,
        spawn_robot,
        bridge_lidar,
        bridge_imu,
        bridge_cmd_vel,
        bridge_joint_states,
        bridge_odom,
        bridge_tf,
        slam_launch,
        rviz_node,
    ])