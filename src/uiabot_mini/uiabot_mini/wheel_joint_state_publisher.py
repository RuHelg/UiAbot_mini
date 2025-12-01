import rclpy, math
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState

# from rclpy.exceptions import RCLError

class WheelJointStatePublisher(Node):
    def __init__(self):
        """ROS2 node to publish wheel joint states based on velocity inputs"""

        # Initialize the ROS2 node
        super().__init__('wheel_joint_state_publisher')

        # Declare parameters, so they can be changed via command line or launch file
        self.declare_parameter('left_joint', 'wheel_l_joint')
        self.declare_parameter('right_joint', 'wheel_r_joint')
        self.declare_parameter('joint_state_update_hz', 50.0)

        # Get parameters
        self.left_name  = self.get_parameter('left_joint').value
        self.right_name = self.get_parameter('right_joint').value
        self.joint_state_update_hz = float(self.get_parameter('joint_state_update_hz').value)

        # Initialize position and velocity variables
        self.pos_l = 0.0
        self.pos_r = 0.0
        self.vel_l = 0.0
        self.vel_r = 0.0
        self.t_last = self.get_clock().now()

        # Create subscriptions and publishers
        self.sub_l = self.create_subscription(Float32, 'wheel_l/velocity', self.cb_l, qos_profile_sensor_data)
        self.sub_r = self.create_subscription(Float32, 'wheel_r/velocity', self.cb_r, qos_profile_sensor_data)
        self.pub_js = self.create_publisher(JointState, 'joint_states', 20)

        # Create a timer to periodically publish joint states
        self.timer = self.create_timer(1.0 / self.joint_state_update_hz, self.tick)  # 50 Hz

    # Callback functions for velocity subscriptions
    def cb_l(self, msg): self.vel_l = float(msg.data)
    def cb_r(self, msg): self.vel_r = float(msg.data)

    def tick(self):
        """Update positions based on velocities and publish JointState"""

        # Calculate time elapsed since last update
        now = self.get_clock().now()
        dt = (now - self.t_last).nanoseconds * 1e-9
        self.t_last = now
        if dt <= 0: return

        # Update positions based on velocities and elapsed time
        self.pos_l += self.vel_l * dt
        self.pos_r += self.vel_r * dt

        # Create and publish JointState message
        js = JointState()
        js.header.stamp = now.to_msg()
        js.name = [self.left_name, self.right_name]
        js.position = [self.pos_l, self.pos_r]
        js.velocity = [self.vel_l, self.vel_r]
        self.pub_js.publish(js)
        
        # Uncomment for debugging
        # self.get_logger().info(
        #     f"Publishing JointState: position=({self.pos_l:.3f}, {self.pos_r:.3f}), "
        #     f"velocity=({self.vel_l:.3f}, {self.vel_r:.3f})"
        # )

def main(args=None):
    rclpy.init(args=args)           # Initialize ROS2
    node = WheelJointStatePublisher()   # Create the node

    try:
        rclpy.spin(node)          # Keep it running until interrupted
    except KeyboardInterrupt:   # Handle Ctrl+C gracefully
        node.get_logger().info('Keyboard interrupt, shutting down WheelJointStatePublisher')
    finally:
        node.destroy_node()               # Clean up the node explicitly
        if rclpy.ok():             # Check if ROS2 is still running
            rclpy.shutdown()        # Shutdown ROS2

        # if rclpy.ok():             # Check if ROS2 is still running
        #     try:
        #         rclpy.shutdown()        # Shutdown ROS2
        #     except RCLError:
        #         # Context already shut down; ignore
        #         pass

if __name__ == '__main__':
    main()
