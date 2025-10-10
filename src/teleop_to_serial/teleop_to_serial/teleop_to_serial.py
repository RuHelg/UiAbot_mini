import serial
import struct
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from std_msgs.msg import Float32
from tf2_ros import TransformBroadcaster
from time import sleep
import math


class TeleopToSerial(Node):
    def __init__(self):
        super().__init__('teleop_to_serial')
        
        # Declare parameters with default values, then get them from launch file
        self.declare_parameter('wheel_right_x', 34e-3)
        self.declare_parameter('wheel_right_y', 98.75e-3)
        self.declare_parameter('wheel_right_z', -0.6e-3)
        self.declare_parameter('wheel_left_x', 34e-3)
        self.declare_parameter('wheel_left_y', -98.75e-3)
        self.declare_parameter('wheel_left_z', -0.6e-3)
        
        # Get wheel position parameters from launch file
        self.wheel_right_pos = [
            self.get_parameter('wheel_right_x').value,
            self.get_parameter('wheel_right_y').value,
            self.get_parameter('wheel_right_z').value
        ]
        self.wheel_left_pos = [
            self.get_parameter('wheel_left_x').value,
            self.get_parameter('wheel_left_y').value,
            self.get_parameter('wheel_left_z').value
        ]
        
        self.get_logger().info(f'Wheel right position: {self.wheel_right_pos}')
        self.get_logger().info(f'Wheel left position: {self.wheel_left_pos}')

        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Create publishers for received wheel velocities
        self.wheel_r_pub = self.create_publisher(Float32, 'wheel_r', 10)
        self.wheel_l_pub = self.create_publisher(Float32, 'wheel_l', 10)
        
        # TF broadcaster for dynamic wheel transforms
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Track wheel rotations (in radians)
        self.wheel_r_rotation = 0.0
        self.wheel_l_rotation = 0.0
        self.last_time = self.get_clock().now()

        try:
            self.ser = serial.Serial(
                port='/dev/ttyUSB1', # Back to ttyUSB1 - this port shows active communication
                baudrate=115200,
                timeout=0.1
            )
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

            received_values = struct.unpack('=ff', in_data)

            # Extract wheel_R and wheel_L from received data
            wheel_R = received_values[0]
            wheel_L = received_values[1]
            
            # Publish wheel velocities
            wheel_r_msg = Float32()
            wheel_r_msg.data = wheel_R
            self.wheel_r_pub.publish(wheel_r_msg)
            
            wheel_l_msg = Float32()
            wheel_l_msg.data = wheel_L
            self.wheel_l_pub.publish(wheel_l_msg)
            
            # Update wheel rotations based on received velocities
            current_time = self.get_clock().now()
            dt = (current_time - self.last_time).nanoseconds / 1e9
            self.last_time = current_time
            
            # wheel_R and wheel_L are already angular velocities in rad/s
            # Negate to correct rotation direction
            self.wheel_r_rotation -= wheel_R * dt
            self.wheel_l_rotation -= wheel_L * dt
            
            # Broadcast dynamic TF for wheels with position from parameters
            self.broadcast_wheel_tf('base_link', 'wheel_right', 
                                   self.wheel_right_pos[0], self.wheel_right_pos[1], self.wheel_right_pos[2],
                                   self.wheel_r_rotation)
            self.broadcast_wheel_tf('base_link', 'wheel_left',
                                   self.wheel_left_pos[0], self.wheel_left_pos[1], self.wheel_left_pos[2],
                                   self.wheel_l_rotation)

            # Uncomment for debugging
            self.get_logger().info(f"Received values: wheel_R={wheel_R}, wheel_L={wheel_L}")

        except struct.error as e:
            self.get_logger().error(f"Error unpacking data: {e}")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial communication error: {e}")

    
    def broadcast_wheel_tf(self, parent_frame, child_frame, x, y, z, rotation):
        """Broadcast TF transform for wheel with position and rotation around Y-axis."""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame
        
        # Position of wheel relative to base_link (from parameters)
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        
        # Rotation around Y-axis (wheel spinning)
        # Convert rotation angle to quaternion
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = math.sin(rotation / 2.0)
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = math.cos(rotation / 2.0)
        
        self.tf_broadcaster.sendTransform(t)


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