#!/usr/bin/env python3

import serial
import struct
import time
import matplotlib.pyplot as plt

# Serial communication parameters
SERIAL_PORT   = "/dev/ttyUSB1"
BAUDRATE      = 115200
TIMEOUT       = 0.1         # seconds for serial read timeout

# Robot parameters
WHEEL_RADIUS  = 0.034       # meters (for converting rad/s to m/s)
SEND_RATE_HZ  = 20.0        # Send commands and read feedback rate

# Command parameters
LINEAR_VEL_X  = 0.2         # m/s
ANGULAR_VEL_Z =  0          # rad/s
DURATION      = 10.0        # seconds to hold this command

# PID parameters - Right Wheel
P_R = 250.0
I_R = 50.0
D_R = 2.0
N_R = 40 # N filter

# PID parameters - Left Wheel
P_L = 272.0
I_L = 30.0
D_L = 2.0
N_L = 40 # N filter

# ===============================================================================

def open_serial(port, baud, timeout):
    try:
        ser = serial.Serial(port=port, baudrate=baud, timeout=timeout)
        print(f"Opened serial port: {ser.port}")
        return ser
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
        return None


def build_command_packet(linear_x, angular_z, P_R, I_R, D_R, N_R, P_L, I_L, D_L, N_L):
    """ 
        Packet format with D and N parameters:
        struct.pack('=BBffffffffff', 36, 36, linear_x, angular_z, P_R, I_R, D_R, N_R, P_L, I_L, D_L, N_L)
        Header: 2 bytes (BB)
        Velocities: 8 bytes (ff - linear_x, angular_z)
        PID wheel 1: 16 bytes (ffff - P_R, I_R, D_R, N_R)
        PID wheel 2: 16 bytes (ffff - P_L, I_L, D_L, N_L)
        Total: 42 bytes
    """
    return struct.pack('=BBffffffffff', 36, 36, linear_x, angular_z, P_R, I_R, D_R, N_R, P_L, I_L, D_L, N_L)


def send_zero_command(ser, repeats=10, delay=0.05, P_R=P_R, I_R=I_R, D_R=D_R, N_R=N_R, P_L=P_L, I_L=I_L, D_L=D_L, N_L=N_L):
    """Send zero velocities a few times to stop the robot cleanly."""
    zero_packet = build_command_packet(0.0, 0.0, P_R, I_R, D_R, N_R, P_L, I_L, D_L, N_L)
    for _ in range(repeats):
        ser.write(zero_packet)
        time.sleep(delay)


def parse_feedback(data):
    """
    Look for header 0x24 0x24 and unpack two floats after it.
    """
    header_pos = data.find(b'\x24\x24')
    if header_pos == -1:
        return None

    start = header_pos + 2
    end   = start + 8
    if end > len(data):
        return None

    try:
        wr, wl = struct.unpack('<ff', data[start:end])
        return wr, wl
    except struct.error:
        return None


def main():
    ser = open_serial(SERIAL_PORT, BAUDRATE, TIMEOUT)
    if ser is None:
        return

    period = 1.0 / SEND_RATE_HZ
    start_time = time.time()

    # Data logging lists
    timestamps = []
    left_velocities = []
    right_velocities = []

    print(f"Sending lin.x={LINEAR_VEL_X}, ang.z={ANGULAR_VEL_Z} "
          f"for {DURATION} seconds...")

    try:
        while (time.time() - start_time) < DURATION:

            # Send command packet
            packet = build_command_packet(LINEAR_VEL_X, ANGULAR_VEL_Z, P_R, I_R, D_R, N_R, P_L, I_L, D_L, N_L)
            ser.write(packet)

            # Read feedback
            data = ser.read(40)
            if data:
                header_pos = data.find(b'\x24\x24')
                if header_pos != -1:
                    start = header_pos + 2
                    end = start + 8  # two float32 (wr, wl)
                    if end <= len(data):
                        try:
                            wr_rads, wl_rads = struct.unpack('<ff', data[start:end])
                            
                            # Convert from rad/s to m/s
                            wr = wr_rads * WHEEL_RADIUS
                            wl = wl_rads * WHEEL_RADIUS
                            
                            current_time = time.time() - start_time
                            
                            # Log data
                            timestamps.append(current_time)
                            left_velocities.append(wl)
                            right_velocities.append(wr)
                            
                            print(f"Feedback -> left: {wl:.3f} m/s ({wl_rads:.2f} rad/s), right: {wr:.3f} m/s ({wr_rads:.2f} rad/s)")
                        except struct.error:
                            pass

            # Rate control
            time.sleep(period)

    except KeyboardInterrupt:
        print("\nInterrupted by user.")

    finally:
        # Stop the robot
        print("Sending zero velocity commands...")
        send_zero_command(ser, repeats=20, delay=period, P_R=P_R, I_R=I_R, D_R=D_R, N_R=N_R, P_L=P_L, I_L=I_L, D_L=D_L, N_L=N_L)

        if ser.is_open:
            ser.close()
            print("Serial port closed.")
        
        # Plot the data
        if timestamps:
            plt.figure(figsize=(10, 6))
            plt.plot(timestamps, left_velocities, label='Left Wheel', marker='o', markersize=3)
            plt.plot(timestamps, right_velocities, label='Right Wheel', marker='x', markersize=3)
            plt.axhline(y=LINEAR_VEL_X, color='r', linestyle='--', label=f'Target: {LINEAR_VEL_X}')
            plt.xlabel('Time (s)')
            plt.ylabel('Velocity (m/s)')
            plt.title(f'PID values\nP_R={P_R}, I_R={I_R}, D_R={D_R}, N_R={N_R} | P_L={P_L}, I_L={I_L}, D_L={D_L}, N_L={N_L}')
            plt.legend()
            plt.grid(True)
            plt.tight_layout()
            
            # Save plot to file with full path
            import os
            filename = f'velocity_plot.png'
            full_path = os.path.abspath(filename)
            plt.savefig(full_path, dpi=150)
            print(f"Plot saved to: {full_path}")

        else:
            print("No data to plot.")


if __name__ == "__main__":
    main()
