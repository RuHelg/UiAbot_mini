#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState


class WheelStatePublisher(Node):
    def __init__(self):
        super().__init__('wheel_state_publisher')
        
        # Declare parameters
        self.declare_parameter('wheel_radius', 0.034)  # 34mm = 0.034m wheel radius
        self.wheel_radius = self.get_parameter('wheel_radius').value
        
        self.get_logger().info(f'Wheel radius: {self.wheel_radius} m')
        
        # Subscribe to individual wheel velocity topics
        self.wheel_r_sub = self.create_subscription(
            Float32,
            'wheel_r',
            self.wheel_r_callback,
            10
        )
        self.wheel_l_sub = self.create_subscription(
            Float32,
            'wheel_l',
            self.wheel_l_callback,
            10
        )
        
        # Publisher for joint states
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
        
        # Store wheel velocities (angular velocity in rad/s from robot)
        self.wheel_r_angular_velocity = 0.0
        self.wheel_l_angular_velocity = 0.0
        
        # Track wheel joint angles in radians (angular position)
        self.wheel_r_angle = 0.0
        self.wheel_l_angle = 0.0
        self.last_time = self.get_clock().now()
        
        self.get_logger().info('Wheel State Publisher node started')
    
    def wheel_r_callback(self, msg):
        """Callback for right wheel velocity (rad/s from robot)."""
        self.wheel_r_angular_velocity = msg.data
        self.publish_joint_state()
    
    def wheel_l_callback(self, msg):
        """Callback for left wheel velocity (rad/s from robot)."""
        self.wheel_l_angular_velocity = msg.data
        self.publish_joint_state()
    
    def publish_joint_state(self):
        """Publish joint state message with wheel angles (rad) and angular velocities (rad/s)."""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        # Integrate angular velocities to get angular positions (radians)
        self.wheel_r_angle += self.wheel_r_angular_velocity * dt
        self.wheel_l_angle += self.wheel_l_angular_velocity * dt
        
        # Create and publish JointState message
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = current_time.to_msg()
        joint_state_msg.name = ['joint_wheel_right', 'joint_wheel_left']
        joint_state_msg.position = [self.wheel_r_angle, self.wheel_l_angle]  # radians
        joint_state_msg.velocity = [self.wheel_r_angular_velocity, self.wheel_l_angular_velocity]  # rad/s
        
        self.joint_state_pub.publish(joint_state_msg)


def main(args=None):
    rclpy.init(args=args)
    node = WheelStatePublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutdown requested by user...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
