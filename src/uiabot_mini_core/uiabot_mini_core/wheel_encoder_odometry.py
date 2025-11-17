#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster

def yaw_to_quaternion(yaw: float) -> Quaternion:
    """Convert a yaw angle (in radians) into a Quaternion message."""

    q = Quaternion()
    h = 0.5 * yaw
    q.w = math.cos(h)
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(h)
    return q

class WheelEncoderOdometry(Node):
    def __init__(self):
        """Node that computes odometry from wheel encoder velocities."""

        super().__init__('wheel_encoder_odometry')

        # Declare parameters
        self.declare_parameter('wheel_radius', 0.034)        # m
        self.declare_parameter('wheel_separation', 0.1975)   # m
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', False)
        self.declare_parameter('left_vel_topic',  'wheel_l/velocity')
        self.declare_parameter('right_vel_topic', 'wheel_r/velocity')
        self.declare_parameter('odom_topic', '/wheel_encoder_odometry')
        self.declare_parameter('odom_update_hz', 50.0)

        # Configure from parameters
        self.R = float(self.get_parameter('wheel_radius').value)
        self.B = float(self.get_parameter('wheel_separation').value)
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        left_topic  = self.get_parameter('left_vel_topic').value
        right_topic = self.get_parameter('right_vel_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.odom_update_hz = float(self.get_parameter('odom_update_hz').value)

        # Internal state
        self.omega_l = 0.0
        self.omega_r = 0.0
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.t_last = self.get_clock().now()

        # Subscriptions, publications and timers
        self.sub_l = self.create_subscription(Float32, left_topic,  self.cb_l, qos_profile_sensor_data)
        self.sub_r = self.create_subscription(Float32, right_topic, self.cb_r, qos_profile_sensor_data)
        self.pub_odom = self.create_publisher(Odometry, self.odom_topic, 10)
        self.tf = TransformBroadcaster(self)

        self.timer = self.create_timer(1.0 / self.odom_update_hz, self.tick)

    # Callbacks for wheel velocity subscriptions
    def cb_l(self, msg: Float32): self.omega_l = float(msg.data)
    def cb_r(self, msg: Float32): self.omega_r = float(msg.data)

    def tick(self):
        """Periodic update to compute and publish odometry."""

        # Time delta
        now = self.get_clock().now()
        dt = (now - self.t_last).nanoseconds * 1e-9
        if dt <= 0.0:
            return
        self.t_last = now

        # Compute wheel linear velocities
        v_l = self.R * self.omega_l
        v_r = self.R * self.omega_r

        # Compute robot linear and angular velocities
        v = 0.5 * (v_r + v_l)
        w = (v_r - v_l) / self.B

        # Integrate SE(2) with constant twist over dt
        if abs(w) < 1e-9:
            dx = v * dt * math.cos(self.yaw)
            dy = v * dt * math.sin(self.yaw)
            dth = 0.0
        else:
            dth = w * dt
            R_icc = v / w # Radius to Instantaneous Center of Curvature
            dx = R_icc * (math.sin(self.yaw + dth) - math.sin(self.yaw))
            dy = -R_icc * (math.cos(self.yaw + dth) - math.cos(self.yaw))

        # Update pose
        self.x += dx
        self.y += dy
        self.yaw = math.atan2(math.sin(self.yaw + dth), math.cos(self.yaw + dth))

        # Publish odometry (no covariances filled)
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = yaw_to_quaternion(self.yaw)
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        self.pub_odom.publish(odom)

        # Publish TF transform
        if self.publish_tf:
            tfm = TransformStamped()
            tfm.header.stamp = now.to_msg()
            tfm.header.frame_id = self.odom_frame
            tfm.child_frame_id = self.base_frame
            tfm.transform.translation.x = self.x
            tfm.transform.translation.y = self.y
            tfm.transform.rotation = odom.pose.pose.orientation
            self.tf.sendTransform(tfm)

def main():
    rclpy.init()
    node = WheelEncoderOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
