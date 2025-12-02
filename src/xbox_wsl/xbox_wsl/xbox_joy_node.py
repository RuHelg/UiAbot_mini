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
import time

class XboxJoyNode(Node):
    def __init__(self):
        super().__init__('xbox_joy_node')
        
        # Parameters
        self.declare_parameter('device', '/dev/input/event0')
        self.declare_parameter('autorepeat_rate', 20.0)  # Hz
        # List of axis indices to invert (e.g. [0] to invert left-stick X)
        # Default: invert left-stick Y (axis index 1) so forward is positive
        self.declare_parameter('axis_invert', [1])
        
        device_path = self.get_parameter('device').value
        rate = self.get_parameter('autorepeat_rate').value
        # remember path for reconnects
        self.device_path = device_path
        # Read invert list (ensure ints)
        inv = self.get_parameter('axis_invert').value
        try:
            self.invert_axes = set(int(x) for x in inv)
        except Exception:
            self.invert_axes = set()
        
        # Open the input device
        try:
            self.device = evdev.InputDevice(device_path)
            self.get_logger().info(f'Opened device: {self.device.name} at {device_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to open {device_path}: {e}')
            # don't exit here; try reconnecting in publish loop
            self.device = None
        
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
            # Check if device needs to be reopened
            if self.device is None:
                try:
                    self.device = evdev.InputDevice(self.device_path)
                    self.get_logger().info(f'Reopened device: {self.device.name}')
                except Exception:
                    pass
            
            # Read all pending events
            if self.device is not None:
                for event in self.device.read():
                    if event.type == evdev.ecodes.EV_ABS:
                        # Absolute axis event
                        if event.code in self.axis_map:
                            idx = self.axis_map[event.code]
                            # Normalize to -1.0 to 1.0
                            if event.code in [2, 5]:  # Triggers
                                val = event.value / 255.0
                            elif event.code in [16, 17]:  # D-pad
                                val = float(event.value)
                            else:  # Sticks
                                val = event.value / 32768.0
                            # Apply inversion if requested for this axis index
                            if hasattr(self, 'invert_axes') and idx in self.invert_axes:
                                val = -val
                            self.axes[idx] = val
                    
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
            # Commonly this is a libusb transfer error when the USB device briefly disconnects
            self.get_logger().error(f'Error reading device: {e}')
            # Attempt to close and reopen the device (transient USB faults)
            try:
                if self.device is not None:
                    try:
                        self.device.close()
                    except Exception:
                        pass
                    self.device = None
            except Exception:
                pass

            # Try to reconnect a few times, then keep trying in background
            for attempt in range(1, 6):
                try:
                    self.get_logger().info(f'Attempting to reopen device (attempt {attempt})...')
                    self.device = evdev.InputDevice(self.device_path)
                    self.get_logger().info(f'Reopened device: {self.device.name} at {self.device_path}')
                    break
                except Exception as e2:
                    self.get_logger().warning(f'Reopen attempt {attempt} failed: {e2}')
                    time.sleep(1)
            else:
                self.get_logger().error('Failed to reopen input device after multiple attempts; will keep trying on next publish')

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
