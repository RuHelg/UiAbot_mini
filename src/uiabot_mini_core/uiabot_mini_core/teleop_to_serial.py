# Assignment 1
import serial, struct, rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from time import sleep
# Assignment 2
from std_msgs.msg import Float32
from rclpy.qos import qos_profile_sensor_data


class TeleopToSerial(Node):
    def __init__(self):
        """ROS2 node to convert teleoperation commands to serial communication"""

        # Initialize the ROS2 node
        super().__init__('teleop_to_serial')
        
        # Declare parameters, so they can be changed via command line or launch file
        self.declare_parameter('serial_port', '/dev/ttyUSB1') # Linux, run "dmesg | grep -i usb"
        self.declare_parameter('baudrate', 115200)            # Common baudrate for serial communication
        self.declare_parameter('serial_timeout', 0.02)        # Timeout for serial read operations in seconds
        self.declare_parameter('read_feedback_hz', 10.0)      # read_wheel_feedback rate
        self.declare_parameter('cmd_vel_send_delay', 0.0)     # Delay after sending cmd_vel data in seconds

        # Retrieve parameter values
        serial_port = self.get_parameter('serial_port').value
        baudrate = self.get_parameter('baudrate').value
        serial_timeout = self.get_parameter('serial_timeout').value
        read_feedback_hz = self.get_parameter('read_feedback_hz').value
        self.cmd_vel_send_delay = self.get_parameter('cmd_vel_send_delay').value
    
        # Create subscriptions and publishers
        self.sub_cmd_vel = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10) # Queue size 10
        
        self.pub_wheel_l = self.create_publisher(Float32, 'wheel_l/velocity', qos_profile_sensor_data)
        self.pub_wheel_r = self.create_publisher(Float32, 'wheel_r/velocity', qos_profile_sensor_data)

        # Initialize serial communication
        try:
            self.ser = serial.Serial(port=serial_port,
                                     baudrate=baudrate,
                                     timeout=serial_timeout)
            
            # Log successful connection
            self.get_logger().info(f"Serial port '{self.ser.port}' opened successfully.")
        
        # Handle serial port opening errors
        except serial.SerialException as e:
            self.get_logger().error(f"Error opening serial port: {e}")
            self.ser = None
            exit(1)

        self.timer = self.create_timer(1.0 / read_feedback_hz, self.read_wheel_feedback)

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
        self.send_cmd_vel_to_serial(linear_velocity_x, angular_velocity_z)

    def send_cmd_vel_to_serial(self, linear_velocity_x, angular_velocity_z):
        """Send cmd_vel data to serial port"""

        # Check if serial port is initialized
        if self.ser is None:
            self.get_logger().error("Serial port not initialized.")
            return
        
        # Pack the data into binary format and send it via serial
        try:
            values_buff = struct.pack('=BBff', 36, 36, linear_velocity_x, angular_velocity_z)
            self.ser.write(values_buff)
            
            # Uncomment for debugging
            #self.get_logger().info(
            #    f"Sent values: lin.x={linear_velocity_x}, ang.z={angular_velocity_z}"
            #)

            # Short delay to ensure data is sent
            sleep(self.cmd_vel_send_delay)

        # Handle serial communication errors
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to send values: {e}")

    def send_zero_values(self):
        """Send zero values for velocity and rotation before shutdown."""
        try:
            zero_buff = struct.pack('=BBff', 36, 36, 0.0, 0.0)
            self.ser.write(zero_buff)
            sleep(0.01)
        
        # Handle serial communication errors
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to send zero values: {e}")

    def read_wheel_feedback(self):
        """Read wheel feedback from serial and publish to ROS2 topics"""
        
        # Check if serial port is initialized 
        if self.ser is None:
            self.get_logger().error("Serial port not initialized.")
            return

        # Read 40 bytes from serial
        try:
            input_data = self.ser.read(40)

            # Check if enough data was received
            if len(input_data) < 40:
                self.get_logger().warning("Incomplete data received")
                return
            
            # Clear buffer to avoid overflow and look for header
            self.ser.reset_input_buffer()             
            header_pos = input_data.find(b'\x24\x24') 

            # Check if header was found
            if header_pos == -1:
                self.get_logger().warning("Header not found in received data")
                return

            # Extract 8 bytes after the header for two floats
            in_data = input_data[(header_pos+2):(header_pos+10)]

            # Check if we have exactly 8 bytes for two floats
            if len(in_data) != 8:
                self.get_logger().warning("Incomplete float data received")
                return
            
            # Unpack the two floats (wheel velocities)
            wheel_velocity_r, wheel_velocity_l = struct.unpack('<ff', in_data)

            # Uncomment for debugging
            # self.get_logger().info(f"Received values: {wheel_velocity_l}, {wheel_velocity_r}")
            
            # Publish the wheel velocities to ROS2 topics
            self.pub_wheel_l.publish(Float32(data=wheel_velocity_l))
            self.pub_wheel_r.publish(Float32(data=wheel_velocity_r))
        
        # Handle unpacking and serial communication errors
        except struct.error as e:
            self.get_logger().error(f"Error unpacking data: {e}")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial communication error: {e}")


def main(args=None):
    rclpy.init(args=args)        # Initialize ROS2
    node = TeleopToSerial()      # Create node instance

    try:                         # Spin the node to process callbacks
        rclpy.spin(node)     
    except KeyboardInterrupt:    # Handle Ctrl+C interruption
        print("Shutdown requested by user...")
    finally:
        
        if node.ser is not None: # Check if serial port is initialized
            node.ser.flush()     # Flush serial buffers
            for i in range(10):  # Send zero values multiple times to ensure reception
                node.send_zero_values()
                sleep(0.05)     # Short delay between sends

            sleep(0.2)          # Give receiver time to process

            # Close the serial port
            if node.ser.is_open:
                node.ser.close()
                print("Serial port closed.")

        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()