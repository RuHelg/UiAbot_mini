# Assignment 1
from launch import LaunchDescription
from launch_ros.actions import Node
# Assignment 2, tts, wjsp
from ament_index_python.packages import get_package_share_directory
import os
# Assignment 2, rplidar, bno055
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
# Assignment 3, slam
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
# Assignment 4, nav2
from launch.substitutions import AndSubstitution, NotSubstitution


def generate_launch_description():
    # URDF path
    desc_pkg = get_package_share_directory('uiabot_mini_description')
    urdf_path = os.path.join(desc_pkg, 'urdf', 'uiabot_mini.urdf')

    # Declare launch arguments (i.e. "run_slam true" at command line launch)
    run_slam = LaunchConfiguration('run_slam')
    run_slam_arg = DeclareLaunchArgument('run_slam', default_value='false')

    run_nav = LaunchConfiguration('run_nav')
    map_file = LaunchConfiguration('map')
    run_nav_arg = DeclareLaunchArgument('run_nav', default_value='false')
    map_file_arg = DeclareLaunchArgument('map', default_value='/home/gruppe-6/ros2/ws2/maps/my_map.yaml')


    # Teleop to Serial Node (custom)
    tts = Node(
        package='uiabot_mini_core',
        executable='teleop_to_serial',
        name='teleop_to_serial',
        parameters=[{'serial_timeout': 0.1,
                     'read_feedback_hz': 30.0,
                     'cmd_vel_send_delay': 0.0}])
    
    # Wheel Joint State Publisher Node (custom)
    wjsp = Node(
        package='uiabot_mini_core',
        executable='wheel_joint_state_publisher',
        name='wheel_joint_state_publisher',
        parameters=[{'joint_state_update_hz': 30.0}])

    # Robot State Publisher Node (built-in)
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': open(urdf_path).read()}])
    
    
    # Include rplidar launch, forwarding launch args (git-cloned)
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('rplidar_ros'), 'launch', 'rplidar_a1_launch.py')
        ),
        launch_arguments={
            'serial_port': '/dev/ttyUSB0',
            'frame_id': 'lidar_link'
        }.items()
    )

    # Include bno055 (no args required)  (git-cloned)
    bno055_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('bno055'), 'launch', 'bno055.launch.py')
        )
    )

    # Include odometry launch (custom)
    we_odom = Node(
        package='uiabot_mini_core',
        executable='wheel_encoder_odometry',
        name='wheel_encoder_odometry',
        output='screen',
        parameters=[{'odom_update_hz': 30.0}]
    )

    # EKF Node from robot_localization (built-in)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[os.path.join(get_package_share_directory('uiabot_mini_bringup'),'config','ekf.yaml')],
        output='screen',
    )

    # SLAM Toolbox Node (built-in)
    slam_params = os.path.join(get_package_share_directory('uiabot_mini_bringup'), 'config', 'slam_params.yaml')
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
        ),
        launch_arguments={'slam_params_file': slam_params,
        }.items(),
        condition=IfCondition(run_slam)
    )

    # Nav2 - Two modes:
    # 1. With localization (map_server + AMCL): use when run_slam=false
    # 2. Without localization: use when run_slam=true (SLAM provides map->odom)
    nav2_params = os.path.join(get_package_share_directory('uiabot_mini_bringup'), 'config', 'nav2_params.yaml')
    
    # Mode 1: Full Nav2 with map_server and AMCL (for pre-mapped environments)
    nav2_with_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_file,
            'params_file': nav2_params,
            'use_sim_time': 'false',
            'autostart': 'true'
        }.items(),
        condition=IfCondition(AndSubstitution(run_nav, NotSubstitution(run_slam)))
    )
    
    # Mode 2: Nav2 without localization (for SLAM mode - only navigation stack)
    nav2_without_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'params_file': nav2_params,
            'use_sim_time': 'false',
            'autostart': 'true'
        }.items(),
        condition=IfCondition(AndSubstitution(run_nav, run_slam))
    )

    return LaunchDescription([
        tts,
        wjsp,
        rsp,
        rplidar_launch,
        bno055_launch,
        we_odom, ekf_node,
        slam_launch,
        run_slam_arg,
        run_nav_arg,
        map_file_arg,
        nav2_with_localization,
        nav2_without_localization
    ])