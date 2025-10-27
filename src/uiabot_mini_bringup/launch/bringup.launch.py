from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
        # parameters=[{
        #     'wheel_radius': 0.033,
        #     'wheel_separation': 0.1975,
        #     'odom_frame': 'odom',       # must match RViz Fixed Frame parent for robot model
        #     'base_frame': 'base_link',  # must match URDF robot base link name
        #     'left_vel_topic': 'wheel_l/velocity',
        #     'right_vel_topic': 'wheel_r/velocity',
        #     'odom_topic': '/wheel_encoder_odometry',
        #     'publish_tf': True,
        #     'rate_hz': 50.0
        # }],
        output='screen'
    )

    # slam_toolbox params file (create this at the path below)
    slam_params = os.path.join(
        get_package_share_directory('uiabot_mini_bringup'),
        'config', 'slam_params.yaml'
    )

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params]
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=['/home/gruppe-6/ros2/ws2/src/uiabot_mini_bringup/config/ekf.yaml'],
        # parameters=[os.path.join(get_package_share_directory('uiabot_mini_bringup'),'config','ekf.yaml')],
        output='screen'
    )

    return LaunchDescription([rsp, tts, wjsp, rplidar_launch, bno055_launch, we_odom, slam_node, ekf_node])
