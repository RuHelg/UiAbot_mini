import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math



class JointStateListener(Node):
    def __init__(self):
        super().__init__('odometry_publisher')
        
        # Declare parameters
        self.declare_parameter('wheel_radius', 0.034)  # 34mm wheel radius in meters
        self.declare_parameter('wheel_base', 0.1975)   # Distance between wheels in meters
        
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_base = self.get_parameter('wheel_base').value
        
        # Subscribe to joint states
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Publisher for odometry
        self.odom_pub = self.create_publisher(Odometry, 'wheel_encoder_odometry', 10)
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Robot pose
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        self.last_time = self.get_clock().now()
        
        self.get_logger().info(f'Odometry Publisher started')
        self.get_logger().info(f'Wheel radius: {self.wheel_radius} m')
        self.get_logger().info(f'Wheel base: {self.wheel_base} m')

    def joint_state_callback(self, msg: JointState):
        # Find right and left wheel indices
        try:
            right_idx = msg.name.index('joint_wheel_right')
            left_idx = msg.name.index('joint_wheel_left')
        except ValueError:
            self.get_logger().warn('Could not find wheel joints in joint_states')
            return
        
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        # Get wheel angular velocities directly from joint_states (rad/s)
        right_angular_vel = msg.velocity[right_idx]
        left_angular_vel = msg.velocity[left_idx]
        
        # Convert angular velocities to linear velocities
        right_linear_vel = right_angular_vel * self.wheel_radius
        left_linear_vel = left_angular_vel * self.wheel_radius
        
        # Calculate robot velocities using differential drive kinematics
        v_x = (right_linear_vel + left_linear_vel) / 2.0  # Linear velocity
        omega = (right_linear_vel - left_linear_vel) / self.wheel_base  # Angular velocity
        
        # Update pose by integrating velocities
        if dt > 0:
            delta_theta = omega * dt
            self.theta += delta_theta
            # Normalize theta to [-pi, pi]
            self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
            
            delta_s = v_x * dt
            delta_x = delta_s * math.cos(self.theta)
            delta_y = delta_s * math.sin(self.theta)
            
            self.x += delta_x
            self.y += delta_y
        
        # Create quaternion from yaw
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)
        
        # Publish TF transform (odom -> base_link)
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        
        self.tf_broadcaster.sendTransform(t)
        
        # Create and publish Odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        # Set pose
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        
        # Set velocity
        odom.twist.twist.linear.x = v_x
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = omega
        
        # Set covariance (simple diagonal values)
        odom.pose.covariance[0] = 0.001   # x
        odom.pose.covariance[7] = 0.001   # y
        odom.pose.covariance[35] = 0.001  # yaw
        
        odom.twist.covariance[0] = 0.001   # vx
        odom.twist.covariance[7] = 0.001   # vy
        odom.twist.covariance[35] = 0.001  # vyaw
        
        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = JointStateListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()