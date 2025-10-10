#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster
import math


class DifferentialDriveOdometry(Node):
    def __init__(self):
        super().__init__('differential_drive_odometry')
        
        # Declare parameters
        self.declare_parameter('wheel_radius', 0.034)  # meters
        self.declare_parameter('wheel_base', 0.1975)  # distance between wheels in meters
        
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_base = self.get_parameter('wheel_base').value
        
        self.get_logger().info(f'Wheel radius: {self.wheel_radius} m')
        self.get_logger().info(f'Wheel base: {self.wheel_base} m')
        
        # Subscribe to wheel velocities (angular velocity in rad/s)
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
        
        # Publisher for odometry
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        
        # TF broadcaster for odom->base_link transform
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Robot pose (x, y, theta)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Wheel velocities (angular in rad/s)
        self.wheel_r_velocity = 0.0
        self.wheel_l_velocity = 0.0
        
        self.last_time = self.get_clock().now()
        
        self.get_logger().info('Differential Drive Odometry node started')
    
    def wheel_r_callback(self, msg):
        """Callback for right wheel velocity."""
        self.wheel_r_velocity = msg.data
        self.update_odometry()
    
    def wheel_l_callback(self, msg):
        """Callback for left wheel velocity."""
        self.wheel_l_velocity = msg.data
        self.update_odometry()
    
    def update_odometry(self):
        """Calculate and publish odometry based on differential drive kinematics."""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        if dt == 0:
            return
        
        # Convert angular velocities to linear velocities
        v_r = self.wheel_r_velocity * self.wheel_radius  # right wheel linear velocity (m/s)
        v_l = self.wheel_l_velocity * self.wheel_radius  # left wheel linear velocity (m/s)
        
        # Differential drive kinematics
        v = (v_r + v_l) / 2.0  # linear velocity of robot (m/s)
        omega = (v_r - v_l) / self.wheel_base  # angular velocity of robot (rad/s)
        
        # Update pose using odometry integration
        delta_theta = omega * dt
        delta_x = v * math.cos(self.theta + delta_theta / 2.0) * dt
        delta_y = v * math.sin(self.theta + delta_theta / 2.0) * dt
        
        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta
        
        # Normalize theta to [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # Create quaternion from yaw
        q = self.yaw_to_quaternion(self.theta)
        
        # Publish TF transform (odom -> base_link)
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = q
        self.tf_broadcaster.sendTransform(t)
        
        # Publish Odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        
        # Set position
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation = q
        
        # Set velocity
        odom_msg.twist.twist.linear.x = v
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.angular.z = omega
        
        self.odom_pub.publish(odom_msg)
    
    def yaw_to_quaternion(self, yaw):
        """Convert yaw angle to quaternion."""
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q


def main(args=None):
    rclpy.init(args=args)
    node = DifferentialDriveOdometry()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutdown requested by user...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
