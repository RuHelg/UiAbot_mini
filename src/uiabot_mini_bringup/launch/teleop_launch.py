from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Serial Communication Node (custom)
    sc = Node(
        package='uiabot_mini',
        executable='serial_communication',
        name='serial_communication',
        parameters=[{'serial_timeout': 0.2,
                     'read_feedback_hz': 100.0,
                     'cmd_vel_send_delay': 0.0}])

    return LaunchDescription([
        sc,
    ])