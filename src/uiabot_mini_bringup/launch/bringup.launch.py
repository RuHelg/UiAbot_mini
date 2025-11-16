from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, AndSubstitution, NotSubstitution
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Launch arguments
    run_slam = LaunchConfiguration('run_slam')
    run_nav = LaunchConfiguration('run_nav')
    map_file = LaunchConfiguration('map')  # Only used when run_slam=false and run_nav=true
    
    run_slam_arg = DeclareLaunchArgument('run_slam', default_value='false')
    run_nav_arg = DeclareLaunchArgument('run_nav', default_value='false')
    map_file_arg = DeclareLaunchArgument(
        'map', 
        default_value='/home/gruppe-6/ros2/ws/src/maps/Datalab.yaml',
        description='Path to map file (only used when run_slam=false and run_nav=true)'
    )

    # URDF
    desc_pkg = get_package_share_directory('uiabot_mini_description')
    urdf_path = os.path.join(desc_pkg, 'urdf', 'uiabot_mini.urdf')
    
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': open(urdf_path).read()}]
    )

    # Your hardware nodes
    tts = Node(
        package='uiabot_mini_core',
        executable='teleop_to_serial',
        name='teleop_to_serial',
        parameters=[{'serial_port':'/dev/ttyUSB1','baudrate':115200,'rx_hz':50.0}]
    )
    
    wjsp = Node(
        package='uiabot_mini_core',
        executable='wheel_joint_state_publisher',
        name='wheel_joint_state_publisher',
        parameters=[{'left_joint':'wheel_l_joint','right_joint':'wheel_r_joint'}]
    )
    
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('rplidar_ros'), 'launch', 'rplidar_a1_launch.py')
        ),
        launch_arguments={
            'serial_port': '/dev/ttyUSB0', 
            'frame_id': 'lidar_link',
            'angle_compensate': 'true',
            'scan_mode': 'Standard'
        }.items()
    )

    bno055_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('bno055'), 'launch', 'bno055.launch.py')
        )
    )

    we_odom = Node(
        package='uiabot_mini_core',
        executable='wheel_encoder_odometry',
        name='wheel_encoder_odometry',
        output='screen'
    )

    # EKF
    ekf_params = os.path.join(get_package_share_directory('uiabot_mini_bringup'), 'config', 'ekf.yaml')
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[ekf_params],
        output='screen'
    )

    # SLAM Toolbox (use standard launch)
    slam_params = os.path.join(get_package_share_directory('uiabot_mini_bringup'), 'config', 'slam_params.yaml')
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'slam_params_file': slam_params,
            'use_sim_time': 'false'
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


    # # Static TF: map -> odom (use when not running AMCL)
    # map_to_odom_tf = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='map_to_odom_publisher',
    #     arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
    #     output='screen',
    #     condition=IfCondition(LaunchConfiguration('use_static_map_tf', default='false'))
    # )

    return LaunchDescription([
        run_slam_arg,
        run_nav_arg,
        map_file_arg,
        rsp,
        tts,
        wjsp,
        rplidar_launch,
        bno055_launch,
        we_odom,
        ekf_node,
        #map_to_odom_tf,  # <-- add this
        slam_launch,
        nav2_with_localization,
        nav2_without_localization
    ])
