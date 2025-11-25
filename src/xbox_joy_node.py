#!/usr/bin/env python3
"""
Xbox Controller Joy Node for WSL2
Reads from /dev/input/event0 and publishes sensor_msgs/Joy messages
Works around WSL2's lack of joydev kernel module
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import evdev
import sys

class XboxJoyNode(Node):
    def __init__(self):
        super().__init__('xbox_joy_node')
        
        # Parameters
        self.declare_parameter('device', '/dev/input/event0')
        self.declare_parameter('autorepeat_rate', 20.0)  # Hz
        
        device_path = self.get_parameter('device').value
        rate = self.get_parameter('autorepeat_rate').value
        
        # Open the input device
        try:
            self.device = evdev.InputDevice(device_path)
            self.get_logger().info(f'Opened device: {self.device.name} at {device_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to open {device_path}: {e}')
            sys.exit(1)
        
        # Publisher
        self.pub = self.create_publisher(Joy, 'joy', 10)
        
        # Timer for publishing at fixed rate
        self.timer = self.create_timer(1.0/rate, self.publish_joy)
        
        # Xbox 360 controller state
        self.axes = [0.0] * 8  # 8 axes
        self.buttons = [0] * 11  # 11 buttons
        
        # Axis mappings for Xbox 360 controller
        self.axis_map = {
            0: 0,   # Left stick X
            1: 1,   # Left stick Y
            3: 2,   # Right stick X
            4: 3,   # Right stick Y
            2: 4,   # Left trigger (RT)
            5: 5,   # Right trigger (LT)
            16: 6,  # D-pad X
            17: 7,  # D-pad Y
        }
        
        # Button mappings for Xbox 360 controller
        self.button_map = {
            304: 0,  # A
            305: 1,  # B
            307: 2,  # X
            308: 3,  # Y
            310: 4,  # LB
            311: 5,  # RB
            314: 6,  # Back
            315: 7,  # Start
            316: 8,  # Xbox button
            317: 9,  # Left stick press
            318: 10, # Right stick press
        }
        
        self.get_logger().info('Xbox Joy Node started. Publishing to /joy topic')
    
    def publish_joy(self):
        """Read events and publish Joy message"""
        try:
            # Read all pending events
            for event in self.device.read():
                if event.type == evdev.ecodes.EV_ABS:
                    # Absolute axis event
                    if event.code in self.axis_map:
                        idx = self.axis_map[event.code]
                        # Normalize to -1.0 to 1.0
                        if event.code in [2, 5]:  # Triggers
                            self.axes[idx] = event.value / 255.0
                        elif event.code in [16, 17]:  # D-pad
                            self.axes[idx] = float(event.value)
                        else:  # Sticks
                            self.axes[idx] = event.value / 32768.0
                
                elif event.type == evdev.ecodes.EV_KEY:
                    # Button event
                    if event.code in self.button_map:
                        idx = self.button_map[event.code]
                        self.buttons[idx] = event.value
            
            # Publish Joy message
            msg = Joy()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'joy'
            msg.axes = self.axes.copy()
            msg.buttons = self.buttons.copy()
            self.pub.publish(msg)
            
        except BlockingIOError:
            # No events available, just publish current state
            msg = Joy()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'joy'
            msg.axes = self.axes.copy()
            msg.buttons = self.buttons.copy()
            self.pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Error reading device: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = XboxJoyNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
