import serial, struct, rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from time import sleep
from std_msgs.msg import Float32
from rclpy.qos import QoSProfile, qos_profile_sensor_data


class TeleopToSerial(Node):
    def __init__(self):
        super().__init__('teleop_to_serial')
        self.declare_parameter('serial_port', '/dev/ttyUSB1') # Linux, run "dmesg | grep -i usb"
        self.declare_parameter('baudrate', 115200)

        self.sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.pub_l = self.create_publisher(Float32, 'wheel_l/velocity', qos_profile_sensor_data)
        self.pub_r = self.create_publisher(Float32, 'wheel_r/velocity', qos_profile_sensor_data)


        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baudrate').value
        try:
            self.ser = serial.Serial(port=port, baudrate=baud, timeout=0.1)
            self.get_logger().info(f"Serial port '{self.ser.port}' opened successfully.")
        except serial.SerialException as e:
            self.get_logger().error(f"Error opening serial port: {e}")
            self.ser = None
            exit(1)

        self.timer = self.create_timer(0.1, self.read_data)

    def cmd_vel_callback(self, msg):
        linear_velocity_x = msg.linear.x
        angular_velocity_z = msg.angular.z

        # Uncomment for debugging
        #self.get_logger().info(
        #    f"Received: lin.x={linear_velocity_x}, lin.y={msg.linear.y}, lin.z={msg.linear.z}, "
        #    f"ang.x={msg.angular.x}, ang.y={msg.angular.y}, ang.z={angular_velocity_z}"
        #)
        self.send_values(linear_velocity_x, angular_velocity_z)


    def send_values(self, linear_velocity_x, angular_velocity_z):
        if self.ser is None:
            self.get_logger().error("Serial port not initialized.")
            return
        
        try:
            values_buff = struct.pack('=BBff', 36, 36, linear_velocity_x, angular_velocity_z)
            self.ser.write(values_buff)
            
            # Uncomment for debugging
            #self.get_logger().info(
            #    f"Sent values: lin.x={linear_velocity_x}, ang.z={angular_velocity_z}"
            #)
            sleep(0.01)

        except serial.SerialException as e:
            self.get_logger().error(f"Failed to send values: {e}")


    def send_zero_values(self, linear_velocity_x, angular_velocity_z):
        """Send zero values for velocity and rotation before shutdown."""
        try:
            zero_buff = struct.pack('=BBff', 36, 36, 0.0, 0.0)
            self.ser.write(zero_buff)
            sleep(0.01)
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to send zero values: {e}")


    def read_data(self):
        if self.ser is None:
            self.get_logger().error("Serial port not initialized.")
            return

        try:
            input_data = self.ser.read(40)

            if len(input_data) < 40:
                self.get_logger().warning("Incomplete data received")
                return

            self.ser.reset_input_buffer()

            header_pos = input_data.find(b'\x24\x24')

            if header_pos == -1:
                self.get_logger().warning("Header not found in received data")
                return


            # Extract 8 bytes after the header for two floats
            in_data = input_data[(header_pos+2):(header_pos+10)]

            if len(in_data) != 8:
                self.get_logger().warning("Incomplete float data received")
                return
            
            wr, wl = struct.unpack('<ff', in_data)
            # Uncomment for debugging
            # self.get_logger().info(f"Received values: {wl}, {wr}")
            self.pub_l.publish(Float32(data=wl))
            self.pub_r.publish(Float32(data=wr))

        except struct.error as e:
            self.get_logger().error(f"Error unpacking data: {e}")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial communication error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = TeleopToSerial()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutdown requested by user...")
    finally:
        
        # Send zero values before shutdown
        if node.ser is not None:
            node.ser.flush()
            for i in range(100):
                node.send_zero_values(0.0, 0.0)
            sleep(1)

            # Close the serial port
            if node.ser.is_open:
                node.ser.close()
                print("Serial port closed.")

        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()