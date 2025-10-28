from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    desc_pkg = get_package_share_directory('uiabot_mini_description')
    urdf_path = os.path.join(desc_pkg, 'urdf', 'uiabot_mini.urdf')

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': open(urdf_path).read()}])

    tts = Node(
        package='uiabot_mini_core',
        executable='teleop_to_serial',
        name='teleop_to_serial',
        parameters=[{'serial_port':'/dev/ttyUSB1','baudrate':115200,'rx_hz':50.0}])
    
    wjsp = Node(
        package='uiabot_mini_core',
        executable='wheel_joint_state_publisher',
        name='wheel_joint_state_publisher',
        parameters=[{'left_joint':'wheel_l_joint','right_joint':'wheel_r_joint'}])
    
    # Include rplidar launch, forwarding launch args
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('rplidar_ros'), 'launch', 'rplidar_a1_launch.py')
        ),
        launch_arguments={
            'serial_port': '/dev/ttyUSB0',
            'frame_id': 'lidar_link'
        }.items()
    )

    # Include bno055 (no args required)
    bno055_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('bno055'), 'launch', 'bno055.launch.py')
        )
    )

    # Include odometry launch
    we_odom = Node(
        package='uiabot_mini_core',
        executable='wheel_encoder_odometry',
        name='wheel_encoder_odometry',
        output='screen'
    )

    # slam_toolbox params file (create this at the path below)
    slam_params = os.path.join(
        get_package_share_directory('uiabot_mini_bringup'),
        'config', 'slam_params.yaml'
    )

    # Declare a launch argument to enable/disable SLAM include
    run_slam_arg = DeclareLaunchArgument(
        'run_slam',
        default_value='false',
        description='If true, include SLAM Toolbox launch'
    )
    
    run_nav_arg = DeclareLaunchArgument(
        'run_nav',
        default_value='false',
        description='If true, include Nav2 navigation stack'
    )
    
    map_file_arg = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Full path to map yaml file for Nav2 localization'
    )

    run_slam = LaunchConfiguration('run_slam')
    run_nav = LaunchConfiguration('run_nav')
    map_file = LaunchConfiguration('map')

    # Use SLAM Toolbox's built-in launch file with autostart
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'slam_params_file': slam_params,
            'use_sim_time': 'false'
        }.items()
        ,
        # Explicitly check for the literal string 'true' so run_slam:=false is not truthy
        # Wrap the substitution in quotes so unquoted tokens (e.g. true) don't become
        # bare Python names (which would raise NameError). This produces e.g. "'true' == 'true'".
        condition=IfCondition(PythonExpression(["'", run_slam, "' == 'true'"]))
    )

    # Nav2 navigation stack (using bringup which includes map server + localization + navigation)
    nav2_params = os.path.join(
        get_package_share_directory('uiabot_mini_bringup'),
        'config', 'nav2_params.yaml'
    )
    
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'params_file': nav2_params,
            'map': map_file,
            'use_sim_time': 'false',
            'autostart': 'true'
        }.items(),
        condition=IfCondition(PythonExpression(["'", run_nav, "' == 'true'"]))
    )

    ekf_params = os.path.join(
        get_package_share_directory('uiabot_mini_bringup'),
        'config', 'ekf.yaml'
    )
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[ekf_params],
        output='screen',
        remappings=[('odometry/filtered', '/odometry/filtered')]
    )

    return LaunchDescription([
        run_slam_arg,
        run_nav_arg,
        map_file_arg,
        rsp, tts, wjsp, rplidar_launch, bno055_launch, we_odom, ekf_node, slam_launch, nav2_launch
    ])


