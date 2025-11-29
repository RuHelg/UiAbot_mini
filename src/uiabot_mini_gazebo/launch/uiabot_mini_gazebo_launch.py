from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription, DeclareLaunchArgument, TimerAction, RegisterEventHandler, ExecuteProcess
from launch.event_handlers import OnProcessExit, OnShutdown, OnProcessStart
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, AndSubstitution, NotSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
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
    
    models_dir = os.path.join(uiabot_mini_gazebo_pkg, 'models')
    gz_resource_path = ':'.join(filter(None, [
        models_dir,
        os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
    ]))
    set_gz_resource_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        gz_resource_path,
    )

    # File paths
    gz_launch_path = os.path.join(ros_gz_sim_pkg, 'launch', 'gz_sim.launch.py')
    world_file = os.path.join(uiabot_mini_gazebo_pkg, 'worlds', 'simple_world.sdf')
    urdf_file = os.path.join(uiabot_mini_description_pkg, 'urdf', 'uiabot_mini.urdf')

    # Try installed package share paths first, then workspace `src` copies as a fallback
    gazebo_candidates = [
        os.path.join(uiabot_mini_gazebo_pkg, 'urdf', 'uiabot_mini_gazebo.urdf'),
        os.path.join(os.path.expanduser('~'), 'ros2_ws', 'src', 'uiabot_mini_gazebo', 'urdf', 'uiabot_mini_gazebo.urdf'),
    ]
    gazebo_file = next((p for p in gazebo_candidates if os.path.exists(p)), None)
    if gazebo_file is None:
        raise FileNotFoundError(
            f"uiabot_mini_gazebo.urdf not found in any of: {gazebo_candidates}"
        )

    controller_config = os.path.join(uiabot_mini_gazebo_pkg, 'config', 'diff_drive_controller.yaml')
    
    # Read URDF and merge with Gazebo elements
    with open(urdf_file, 'r') as f:
        urdf_content = f.read()
    with open(gazebo_file, 'r') as f:
        gazebo_content = f.read()
    
    # Extract gazebo content (remove XML declaration and robot wrapper if present)
    gazebo_inner = gazebo_content.replace('<?xml version="1.0"?>', '', 1).strip()
    if gazebo_inner.startswith('<robot'):
        # Extract content between <robot ...> and </robot>
        start = gazebo_inner.find('>') + 1
        end = gazebo_inner.rfind('</robot>')
        gazebo_inner = gazebo_inner[start:end].strip()
    
    # Merge: insert gazebo content before closing </robot>
    robot_desc = urdf_content.replace('</robot>', gazebo_inner)

    # Save the merged URDF to a temporary file in a user-writable directory
    fd, merged_urdf_path = tempfile.mkstemp(suffix='.urdf', prefix='uiabot_mini_merged_')
    # Close the low-level fd returned by mkstemp before using Python's file API.
    # Ensure data is flushed & synced and give a tiny delay so other processes can open it reliably.
    os.close(fd)
    try:
        with open(merged_urdf_path, 'w') as mf:
            mf.write(robot_desc)
            mf.flush()
            os.fsync(mf.fileno())
        # Small delay to avoid race where Gazebo/spawn tries to open file immediately
        time.sleep(0.05)
    except OSError as e:
        raise RuntimeError(
            f"Failed to write merged URDF to '{merged_urdf_path}'. "
            "This may be due to insufficient permissions, missing directory, or lack of disk space. "
            f"Original error: {e}"
        )
    
    # Nodes
    # create the Gazebo IncludeLaunchDescription and start it with a small delay
    gazebo_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch_path),
        launch_arguments={
            'gz_args': world_file,
            'on_exit_shutdown': 'True'
        }.items(),
    )
    gazebo = TimerAction(
        period=0.5,  # increase to 1.0 or 2.0 if race persists
        actions=[gazebo_include]
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
            '-file', merged_urdf_path,
            '-name', 'uiabot_mini',
            '-z', '0.1'
        ],
        output='screen'
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',            # ROS -> Gazebo
            '/wheel_encoder_odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',              # Gazebo -> ROS
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',              # Gazebo -> ROS
            '/bno055/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',                      # Gazebo -> ROS
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',         # Gazebo -> ROS
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',    # Gazebo -> ROS
            # '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',                 # Gazebo -> ROS
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

    # spawn the robot after a short delay to avoid race opening the merged URDF
    spawn_robot_delayed = TimerAction(
        period=2.0,
        actions=[spawn_robot]
    )
    # cleanup temporary merged URDF on shutdown (safer: works for IncludeLaunchDescription)
    cleanup_handler = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[ExecuteProcess(cmd=['/bin/rm', '-f', merged_urdf_path])]
        )
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
        # remappings=[
        #     ('/wheel_encoder_odometry', '/odom'),
        #     ('/bno055/imu', '/imu'),
        # ]
    )

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument('rviz', default_value='false', description='Start RViz'),
        DeclareLaunchArgument('run_slam', default_value='false'),
        DeclareLaunchArgument('run_nav', default_value='false'),
        DeclareLaunchArgument('map', default_value=os.path.expanduser('~/ros2_ws/maps/my_map.yaml')),
        
        # Set Gazebo resource path
        set_gz_resource_path,

        # # Set Gazebo resource path
        # SetEnvironmentVariable(
        #     'GZ_SIM_RESOURCE_PATH',
        #     ':'.join([
        #         os.path.join(os.path.expanduser('~'), 'ros2_ws', 'src'),
        #         os.path.join(os.path.expanduser('~'), 'ros2_ws', 'src', 'uiabot_mini_gazebo')
        #     ])
        # ),
        
        gazebo,
        robot_state_publisher,
        spawn_robot_delayed,
        bridge,
        ekf_node,
        cleanup_handler,
        slam_launch,
        nav2_launch,
        rviz_node,
    ])