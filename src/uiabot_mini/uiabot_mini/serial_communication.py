# Assignment 1
import serial, struct, rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from time import sleep
# Assignment 2
# from std_msgs.msg import Float32
from sensor_msgs.msg import JointState

# Odometry
import math
from nav_msgs.msg import Odometry


class SerialCommunication(Node):
    def __init__(self):
        """ROS2 node to handle serial communication with the robot's microcontroller"""

        # Initialize the ROS2 node
        super().__init__('serial_communication')
        
        # =========== Declare parameters =========== (so they can be changed via command line or launch file)
        # Serial communication parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB1') # Linux, run "dmesg | grep -i usb"
        self.declare_parameter('baudrate', 115200)            # Common baudrate for serial communication
        self.declare_parameter('serial_timeout', 0.02)        # Timeout for serial read operations in seconds
        self.declare_parameter('read_feedback_hz', 10.0)      # read_wheel_feedback rate
        self.declare_parameter('cmd_vel_send_delay', 0.0)     # Delay after sending cmd_vel data in seconds

        # Joint state publisher parameters
        self.declare_parameter('left_wheel_joint', 'wheel_l_joint')  # Name of the left wheel joint
        self.declare_parameter('right_wheel_joint', 'wheel_r_joint') # Name of the right wheel joint

        # Odometry parameters
        self.declare_parameter('odom_topic', '/wheel_encoder_odometry')  # Topic name for odometry data
        self.declare_parameter('odom_frame', 'odom')                     # Frame ID for odometry
        self.declare_parameter('base_frame', 'base_link')                # Frame ID for the robot base

        # =========== Retrieve parameter values ===========
        # Serial communication parameters
        serial_port             = self.get_parameter('serial_port').value
        baudrate                = self.get_parameter('baudrate').value
        serial_timeout          = self.get_parameter('serial_timeout').value
        read_feedback_hz        = self.get_parameter('read_feedback_hz').value
        self.cmd_vel_send_delay = self.get_parameter('cmd_vel_send_delay').value

        # Joint state publisher parameters
        self.left_joint_name  = self.get_parameter('left_wheel_joint').value
        self.right_joint_name = self.get_parameter('right_wheel_joint').value

        # Odometry parameters
        self.odom_topic   = self.get_parameter('odom_topic').value
        self.odom_frame   = self.get_parameter('odom_frame').value
        self.base_frame   = self.get_parameter('base_frame').value
    
        # =========== Create subscriptions and publishers ===========
        self.sub_cmd_vel = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 5) # Queue size 5
        self.pub_js = self.create_publisher(JointState, 'joint_states', 10)                     # Queue size 10
        self.pub_odom = self.create_publisher(Odometry, self.odom_topic, 20)                    # Queue size 20

        # =========== Initialize serial communication ===========
        try:
            self.ser = serial.Serial(port=serial_port,
                                     baudrate=baudrate,
                                     timeout=serial_timeout)
            
            # Log successful connection
            self.get_logger().info(f"Serial port '{self.ser.port}' opened successfully.")

            # RX buffer for robust packet parsing
            self.rx_buffer = bytearray()

            # Send reset on startup to zero odometry on ESP32 side
            self.send_reset_pulse()
        
        # Handle serial port opening errors
        except serial.SerialException as e:
            self.get_logger().error(f"Error opening serial port: {e}")
            self.ser = None
            self.rx_buffer = bytearray()
            exit(1)

        # Timer to read feedback from serial port
        self.timer = self.create_timer(1.0 / read_feedback_hz, self.read_from_serial)

    def cmd_vel_callback(self, msg):
        """Callback function for cmd_vel topic"""

        # Extract linear and angular velocities from the Twist message (controller input)
        linear_velocity_x = msg.linear.x
        angular_velocity_z = msg.angular.z

        # Uncomment for debugging
        #self.get_logger().info(
        #    f"Received: lin.x={linear_velocity_x}, lin.y={msg.linear.y}, lin.z={msg.linear.z}, "
        #    f"ang.x={msg.angular.x}, ang.y={msg.angular.y}, ang.z={angular_velocity_z}"
        #)
        
        # Send the velocities to the serial port
        self.send_to_serial(linear_velocity_x, angular_velocity_z)

    def send_to_serial(self, linear_velocity_x, angular_velocity_z):
        """Send cmd_vel + reset flag to serial port.

        Packet format:
            uint8  0x24  (header 1)
            uint8  0x24  (header 2)
            uint8  reset_flag (0 or 1)
            float  linear_velocity_x
            float  angular_velocity_z
        """

        # Check if serial port is initialized
        if self.ser is None:
            self.get_logger().error("Serial port not initialized.")
            return
        
        # Pack the data into binary format and send it via serial
        try:
            values_buff = struct.pack('=BBBff', 36, 36, 0, linear_velocity_x, angular_velocity_z)
            self.ser.write(values_buff)
            
            # Uncomment for debugging
            # self.get_logger().info(f"Sent values: lin.x={linear_velocity_x}, ang.z={angular_velocity_z}")

            # Short delay to ensure data is sent
            sleep(self.cmd_vel_send_delay)

        # Handle serial communication errors
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to send values: {e}")

    def send_zero_values(self):
        """Send zero values for velocity and rotation before shutdown."""
        try:
            zero_buff = struct.pack('=BBBff', 36, 36, 0, 0.0, 0.0)
            self.ser.write(zero_buff)
            sleep(0.01)
        
        # Handle serial communication errors
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to send zero values: {e}")

    def read_from_serial(self):
        """Read odometry and joint states from serial and publish to ROS2 topics.

        Expected packet format from ESP32 (little-endian):
            uint8  0x24        header 1
            uint8  0x24        header 2
            float  theta_l     [rad]
            float  theta_r     [rad]
            float  x           [m]
            float  y           [m]
            float  yaw         [rad]
            float  v           [m/s]    (linear x in base_link)
            float  w           [rad/s]  (angular z)
        Total: 2 bytes header + 7*4 bytes floats = 30 bytes
        """

        if self.ser is None:
            self.get_logger().error("Serial port not initialized.")
            return

        HEADER = b'\x24\x24'
        FRAME_LEN = 30
        PAYLOAD_LEN = FRAME_LEN - len(HEADER)

        try:
            # Read all currently available bytes (non-blocking due to timeout)
            available = self.ser.in_waiting
            if available == 0:
                return

            chunk = self.ser.read(available)
            if not chunk:
                return

            # Append to rolling buffer
            self.rx_buffer.extend(chunk)

            # Try to extract as many complete frames as possible
            while True:
                # Look for header in buffer
                idx = self.rx_buffer.find(HEADER)

                if idx == -1:
                    # No header at all: keep only last byte as potential start of header
                    if len(self.rx_buffer) > 1:
                        self.rx_buffer = self.rx_buffer[-1:]
                    break

                # Drop any noise before the header
                if idx > 0:
                    del self.rx_buffer[:idx]

                # Now buffer starts with header at index 0
                if len(self.rx_buffer) < FRAME_LEN:
                    # Not enough bytes yet for a full frame
                    break

                # Extract one frame
                frame = self.rx_buffer[:FRAME_LEN]
                del self.rx_buffer[:FRAME_LEN]

                # Safety: verify header again
                if frame[0:2] != HEADER:
                    self.get_logger().warning("Header mismatch after alignment, discarding frame.")
                    continue

                payload = frame[2:]
                if len(payload) != PAYLOAD_LEN:
                    self.get_logger().warning("Incorrect payload length, discarding frame.")
                    continue

                try:
                    theta_l, theta_r, x, y, yaw, v, w = struct.unpack('<fffffff', payload)
                except struct.error as e:
                    self.get_logger().warning(f"Error unpacking frame: {e}")
                    continue

                # --- Publish JointState ---
                js = JointState()
                now = self.get_clock().now()
                js.header.stamp = now.to_msg()
                js.name = [self.left_joint_name, self.right_joint_name]
                js.position = [theta_l, theta_r]
                self.pub_js.publish(js)

                # --- Publish Odometry ---
                self.publish_odom(x, y, yaw, v, w)

        except serial.SerialException as e:
            self.get_logger().error(f"Serial communication error: {e}")
        except Exception as e:
            # Catch-all so one bad frame doesn't kill the node
            self.get_logger().error(f"Unexpected error while reading serial: {e}")

    def send_reset_pulse(self):
        """Send a single reset packet (zero velocities, reset flag = 1)"""
        if self.ser is None:
            self.get_logger().error("Serial port not initialized, cannot send reset.")
            return
        try:
            reset = 1
            buf = struct.pack('=BBBff', 36, 36, reset, 0.0, 0.0)
            self.ser.write(buf)
            self.get_logger().info("Sent odometry reset pulse to ESP32.")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to send reset pulse: {e}")

    def publish_odom(self, x: float, y: float, yaw: float, v: float, w: float):
        """Publish odometry message"""
        odom_msg = Odometry()
        now = self.get_clock().now().to_msg()

        odom_msg.header.stamp = now
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame

        # Pose
        odom_msg.pose.pose.position.x = float(x)
        odom_msg.pose.pose.position.y = float(y)
        odom_msg.pose.pose.position.z = 0.0

        # Yaw -> quaternion (Z-only rotation)
        half_yaw = 0.5 * yaw
        qz = math.sin(half_yaw)
        qw = math.cos(half_yaw)

        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = float(qz)
        odom_msg.pose.pose.orientation.w = float(qw)

        # Twist (in base_link frame)
        odom_msg.twist.twist.linear.x = float(v)
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.linear.z = 0.0

        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = float(w)

        # Covariances can be filled here if desired; is currently zeros

        self.pub_odom.publish(odom_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SerialCommunication()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down SerialCommunication')
    finally:
        if node.ser is not None:
            node.ser.flush()
            for i in range(10):
                node.send_zero_values()
                sleep(0.05)

            sleep(0.2)

            if node.ser.is_open:
                node.ser.close()
                node.get_logger().info('Serial port closed.')

        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()