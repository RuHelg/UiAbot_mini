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
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
# Assignment 4, nav2
from launch.substitutions import AndSubstitution, NotSubstitution

def generate_launch_description():

    # Package share directories
    pkg_share = get_package_share_directory('uiabot_mini_description')

    # File paths, internal to uiabot_mini package
    config_dir     = os.path.join(pkg_share, 'config')                       # Path to config directory
    bno055_params  = os.path.join(config_dir, 'bno055_params_i2c.yaml')      # Path to BNO055 params file
    ekf_config     = os.path.join(config_dir, 'ekf.yaml')                    # Path to ekf config file
    slam_params    = os.path.join(config_dir, 'slam_params.yaml')            # Path to SLAM params file
    nav2_params    = os.path.join(config_dir, 'nav2_params.yaml')            # Path to Nav2 params file
    default_map    = os.path.join(pkg_share, 'maps', 'my_map.yaml')          # Path to default map file

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

    # Launch configurations
    run_slam   = LaunchConfiguration('run_slam')
    run_nav    = LaunchConfiguration('run_nav')
    map_file   = LaunchConfiguration('map')

    # Launch arguments (i.e. "run_slam true" at command line launch)
    run_slam_arg = DeclareLaunchArgument(
        'run_slam',
        default_value='false',
        description='Run SLAM Toolbox'
    )

    run_nav_arg = DeclareLaunchArgument(
        'run_nav',
        default_value='false',
        description='Run Nav2'
    )

    map_file_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Full path to the map yaml file (used when run_slam=false)'
    )

    # Teleop to Serial Node (custom)
    tts = Node(
        package='uiabot_mini',
        executable='teleop_to_serial',
        name='teleop_to_serial',
        parameters=[{'serial_timeout': 0.2,
                     'read_feedback_hz': 100.0,
                     'cmd_vel_send_delay': 0.0}])
    
    # Wheel Joint State Publisher Node (custom)
    wjsp = Node(
        package='uiabot_mini',
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

    # Include odometry launch (custom)
    we_odom = Node(
        package='uiabot_mini',
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
        condition=IfCondition(run_slam),
    )

    # Nav2 - Two modes:
    # 1. With localization (map_server + AMCL): use when run_slam=false
    # 2. Without localization: use when run_slam=true (SLAM provides map->odom)
    
    # Nav2 – Mode 1: With localization (map_server + AMCL), used when run_slam=false
    nav2_with_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch',
                'bringup_launch.py'
            )
        ),
        launch_arguments={
            'map': map_file,
            'params_file': nav2_params,
            'use_sim_time': 'false',
            'autostart': 'true',
        }.items(),
        condition=IfCondition(AndSubstitution(run_nav, NotSubstitution(run_slam))),
    )
    
    # Nav2 – Mode 2: Without localization (for SLAM mode), used when run_slam=true
    nav2_without_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            )
        ),
        launch_arguments={
            'params_file': nav2_params,
            'use_sim_time': 'false',
            'autostart': 'true',
        }.items(),
        condition=IfCondition(AndSubstitution(run_nav, run_slam)),
    )

    return LaunchDescription([
        # Launch arguments
        run_slam_arg,
        run_nav_arg,
        map_file_arg,

        # Core robot nodes
        tts,
        wjsp,
        rsp,
        # we_odom,
        ekf_node,

        # Sensors
        sllidar_launch,
        bno055_node,

        # SLAM and Nav2
        slam_launch,
        nav2_with_localization,
        nav2_without_localization,
    ])