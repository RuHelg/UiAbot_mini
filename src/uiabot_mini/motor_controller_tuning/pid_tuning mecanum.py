import serial
import struct
import time
import matplotlib.pyplot as plt
import os

# Serial communication parameters
SERIAL_PORT   = "/dev/ttyUSB1"
BAUDRATE      = 115200
TIMEOUT       = 0.1
WRITE_TIMEOUT = 0.2

# Robot parameters
WHEEL_RADIUS  = 0.03       # meters (rad/s -> m/s)
SEND_RATE_HZ  = 20.0

# Command parameters
LINEAR_VEL_X  = 0.2        # m/s
ANGULAR_VEL_Z = 0.0        # rad/s
DURATION      = 10.0

# PIDN parameters per motor
P_FL, I_FL, D_FL, N_FL = 250.0, 50.0, 2.0, 40.0
P_FR, I_FR, D_FR, N_FR = 272.0, 30.0, 2.0, 40.0
P_BL, I_BL, D_BL, N_BL = 250.0, 50.0, 2.0, 40.0
P_BR, I_BR, D_BR, N_BR = 272.0, 30.0, 2.0, 40.0

# Feedback framing
HEADER = b"\x24\x24"                 # 0x24 0x24
FEEDBACK_FRAME_LEN = 2 + 4 * 4       # header + 4 float32 (w1..w4) = 18 bytes

# =============================================================================

def open_serial(port, baud, timeout, write_timeout):
    try:
        ser = serial.Serial(
            port=port,
            baudrate=baud,
            timeout=timeout,
            write_timeout=write_timeout
        )
        try:
            ser.reset_input_buffer()
            ser.reset_output_buffer()
        except Exception:
            pass
        print(f"Opened serial port: {ser.port}")
        return ser
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
        return None


def build_command_packet(linear_x, angular_z,
                         P1, I1, D1, N1,
                         P2, I2, D2, N2,
                         P3, I3, D3, N3,
                         P4, I4, D4, N4):
    """
      uint8  0x24
      uint8  0x24
      float  linear_x
      float  angular_z
      float  P_FL I_FL D_FL N_FL
      float  P_FR I_FR D_FR N_FR
      float  P_BL I_BL D_BL N_BL
      float  P_BR I_BR D_BR N_BR

    Total: 74 bytes
    """
    return struct.pack(
        '<BBff' + 'f' * 16,
        36, 36,
        float(linear_x), float(angular_z),
        float(P_FL), float(I_FL), float(D_FL), float(N_FL),
        float(P_FR), float(I_FR), float(D_FR), float(N_FR),
        float(P_BL), float(I_BL), float(D_BL), float(N_BL),
        float(P_BR), float(I_BR), float(D_BR), float(N_BR),
    )


def send_zero_command(ser, repeats=10, delay=0.05):
    zero_packet = build_command_packet(
        0.0, 0.0,
        P_FL, I_FL, D_FL, N_FL,
        P_FR, I_FR, D_FR, N_FR,
        P_BL, I_BL, D_BL, N_BL,
        P_BR, I_BR, D_BR, N_BR
    )
    for _ in range(repeats):
        try:
            ser.write(zero_packet)
        except serial.SerialException:
            break
        time.sleep(delay)


def extract_feedback_frames(rx_buffer: bytearray):

    results = []

    while True:
        idx = rx_buffer.find(HEADER)

        if idx == -1:
            # No header present, keep last byte
            if len(rx_buffer) > 1:
                del rx_buffer[:-1]
            break

        if idx > 0:
            del rx_buffer[:idx]  # drop noise before header

        if len(rx_buffer) < FEEDBACK_FRAME_LEN:
            break  # wait for more bytes

        frame = bytes(rx_buffer[:FEEDBACK_FRAME_LEN])
        del rx_buffer[:FEEDBACK_FRAME_LEN]

        if frame[0:2] != HEADER:
            continue

        payload = frame[2:]
        if len(payload) != 16:
            continue

        try:
            w1, w2, w3, w4 = struct.unpack('<ffff', payload)
            results.append((w1, w2, w3, w4))
        except struct.error:
            continue

    return results


def read_feedback_nonblocking(ser, rx_buffer: bytearray, max_bytes=1024):
    try:
        available = ser.in_waiting
    except serial.SerialException:
        return []

    if available <= 0:
        return []

    to_read = min(available, max_bytes)

    try:
        chunk = ser.read(to_read)
    except serial.SerialException:
        return []

    if not chunk:
        return []

    rx_buffer.extend(chunk)
    return extract_feedback_frames(rx_buffer)


def main():
    ser = open_serial(SERIAL_PORT, BAUDRATE, TIMEOUT, WRITE_TIMEOUT)
    if ser is None:
        return

    period = 1.0 / SEND_RATE_HZ
    start_time = time.time()

    rx_buffer = bytearray()

    # Data logging
    timestamps = []
    m1_vel = []
    m2_vel = []
    m3_vel = []
    m4_vel = []

    print(f"Sending lin.x={LINEAR_VEL_X}, ang.z={ANGULAR_VEL_Z} for {DURATION} seconds...")

    try:
        while (time.time() - start_time) < DURATION:
            loop_t0 = time.time()

            packet = build_command_packet(
                LINEAR_VEL_X, ANGULAR_VEL_Z,
                P_FL, I_FL, D_FL, N_FL,
                P_FR, I_FR, D_FR, N_FR,
                P_BL, I_BL, D_BL, N_BL,
                P_BR, I_BR, D_BR, N_BR
            )

            try:
                ser.write(packet)
            except serial.SerialException as e:
                print(f"Serial write error: {e}")
                break

            frames = read_feedback_nonblocking(ser, rx_buffer)

            for w1_rads, w2_rads, w3_rads, w4_rads in frames:
                v1 = w1_rads * WHEEL_RADIUS
                v2 = w2_rads * WHEEL_RADIUS
                v3 = w3_rads * WHEEL_RADIUS
                v4 = w4_rads * WHEEL_RADIUS

                t = time.time() - start_time
                timestamps.append(t)
                m1_vel.append(v1)
                m2_vel.append(v2)
                m3_vel.append(v3)
                m4_vel.append(v4)

                print(
                    f"Feedback (m/s) -> "
                    f"M1:{v1:.3f} ({w1_rads:.2f} rad/s), "
                    f"M2:{v2:.3f} ({w2_rads:.2f} rad/s), "
                    f"M3:{v3:.3f} ({w3_rads:.2f} rad/s), "
                    f"M4:{v4:.3f} ({w4_rads:.2f} rad/s)"
                )

            elapsed = time.time() - loop_t0
            sleep_time = period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\nInterrupted by user.")

    finally:
        print("Sending zero velocity commands...")
        send_zero_command(ser, repeats=20, delay=period)

        if ser.is_open:
            try:
                ser.flush()
            except Exception:
                pass
            ser.close()
            print("Serial port closed.")

        # Plot
        if timestamps:
            fig, axs = plt.subplots(2, 2, figsize=(12, 8), sharex=True, sharey=True)

            axs[0, 0].plot(timestamps, m1_vel, marker='o', markersize=3)
            axs[0, 0].set_title('Motor 1')
            axs[0, 0].grid(True)

            axs[0, 1].plot(timestamps, m2_vel, marker='x', markersize=3)
            axs[0, 1].set_title('Motor 2')
            axs[0, 1].grid(True)

            axs[1, 0].plot(timestamps, m3_vel, marker='s', markersize=3)
            axs[1, 0].set_title('Motor 3')
            axs[1, 0].grid(True)

            axs[1, 1].plot(timestamps, m4_vel, marker='^', markersize=3)
            axs[1, 1].set_title('Motor 4')
            axs[1, 1].grid(True)

            # Target line on each subplot
            for ax in axs.flat:
                ax.axhline(y=LINEAR_VEL_X, linestyle='--')
                ax.set_xlabel('Time (s)')
                ax.set_ylabel('Wheel velocity (m/s)')

            fig.suptitle(
                "4-Motor Feedback\n"
                f"M1(PIDN)=({P_FL},{I_FL},{D_FL},{N_FL})  M2=({P_FR},{I_FR},{D_FR},{N_FR})\n"
                f"M3(PIDN)=({P_BL},{I_BL},{D_BL},{N_BL})  M4=({P_BR},{I_BR},{D_BR},{N_BR})",
                y=0.98
            )

            plt.tight_layout()

            filename = "velocity_plot_4motors_2x2.png"
            full_path = os.path.abspath(filename)
            plt.savefig(full_path, dpi=150)
            print(f"Plot saved to: {full_path}")
        else:
            print("No data to plot.")

if __name__ == "__main__":
    main()
