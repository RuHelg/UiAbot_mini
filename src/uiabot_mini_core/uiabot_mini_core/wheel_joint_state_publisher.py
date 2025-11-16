import rclpy, math
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState

class WheelJointStatePublisher(Node):
    def __init__(self):
        super().__init__('wheel_joint_state_publisher')
        self.declare_parameter('left_joint', 'wheel_l_joint')
        self.declare_parameter('right_joint', 'wheel_r_joint')

        self.left_name  = self.get_parameter('left_joint').value
        self.right_name = self.get_parameter('right_joint').value

        self.pos_l = 0.0
        self.pos_r = 0.0
        self.vel_l = 0.0
        self.vel_r = 0.0
        self.t_last = self.get_clock().now()

        self.sub_l = self.create_subscription(Float32, 'wheel_l/velocity', self.cb_l, qos_profile_sensor_data)
        self.sub_r = self.create_subscription(Float32, 'wheel_r/velocity', self.cb_r, qos_profile_sensor_data)
        self.pub_js = self.create_publisher(JointState, 'joint_states', 20)

        self.timer = self.create_timer(0.02, self.tick)  # 50 Hz

    def cb_l(self, msg): self.vel_l = float(msg.data)
    def cb_r(self, msg): self.vel_r = float(msg.data)

    def tick(self):
        now = self.get_clock().now()
        dt = (now - self.t_last).nanoseconds * 1e-9
        self.t_last = now
        if dt <= 0: return
        self.pos_l += self.vel_l * dt
        self.pos_r += self.vel_r * dt

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

def main():
    rclpy.init()
    node = WheelJointStatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
