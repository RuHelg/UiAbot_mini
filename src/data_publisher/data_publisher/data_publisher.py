#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import serial, struct, time

PORT   = '/dev/ttyUSB0'   #
BAUD   = 115200
TOPIC  = 'data_vel'       
HEADER = b'\x36\x36'      # 0x36 = '6'
BLOCK  = 40               # leseblokk som i eksempelet
PAYLEN = 8                # 2 x float32 rett etter header

class DataPublish(Node):
    def __init__(self):
        super().__init__('data_publish')
        self.pub = self.create_publisher(TwistStamped, TOPIC, 10)

        try:
            self.ser = serial.Serial(port=PORT, baudrate=BAUD, timeout=0.1)
            self.get_logger().info(f"Serial port '{self.ser.port}' åpnet.")
            time.sleep(2.0)  # gi MCU tid til evt. reset
        except serial.SerialException as e:
            self.ser = None
            self.get_logger().error(f"Feil ved åpning av seriell port: {e}")
            raise SystemExit(1)

        # 10 Hz som i eksempelet (0.1 s)
        self.timer = self.create_timer(0.1, self.read_and_publish)

    def read_and_publish(self):
        if self.ser is None:
            return
        try:
            buf = self.ser.read(BLOCK)
            if len(buf) < BLOCK:
                self.get_logger().debug("Ufullstendig blokk mottatt")
                return

            # flush input så vi alltid jobber på ferske blokker (samme som i eksempelet)
            self.ser.reset_input_buffer()

            pos = buf.find(HEADER)
            if pos == -1:
                self.get_logger().debug("Fant ikke header i blokk")
                return

            start = pos + len(HEADER)
            end   = start + PAYLEN
            if end > len(buf):
                self.get_logger().debug("Ufullstendig float-data etter header")
                return

            v, w = struct.unpack('=ff', buf[start:end])  # native std alignment, little-endian ofte OK; bruk '<ff' om du vil låse LE
            msg = TwistStamped()
            # header.stamp lar vi stå default (0), frame_id tomt
            msg.twist.linear.x  = float(v)
            msg.twist.angular.z = float(w)
            self.pub.publish(msg)

            # Debug (valgfritt)
            # self.get_logger().info(f"Publiserte v={v:.3f}, w={w:.3f}")

        except struct.error as e:
            self.get_logger().error(f"Pakkefeil: {e}")
        except serial.SerialException as e:
            self.get_logger().error(f"Seriellfeil: {e}")

def main():
    rclpy.init()
    node = DataPublish()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser and node.ser.is_open:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
