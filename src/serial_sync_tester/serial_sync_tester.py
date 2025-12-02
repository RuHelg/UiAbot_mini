import serial
import struct
import time
import math
import matplotlib.pyplot as plt
import os

# ===================== Serial communication parameters =========================

SERIAL_PORT   = "/dev/ttyUSB1"
BAUDRATE      = 115200
TIMEOUT       = 0.1        # seconds for serial read timeout
READ_TIMEOUT  = 0.25       # allow more time than one period to gather 2 bytes
STARTUP_DELAY = 0.2        # time to let the device settle after open

# ======================== Test signal parameters ==============================

DURATION          = 1.0     # seconds to run the test
SAMPLE_RATE_HZ    = 10.0   # how often we send samples
SQUARE_FREQ_HZ    = 2.0     # square wave frequency (how many up/down cycles per second)

# Values sent in the square wave (16-bit unsigned)
LOW_VALUE         = 0
HIGH_VALUE        = 100     # adjust as needed to match firmware expectations

# =========================== Helper functions =================================

def open_serial(port, baud, timeout):
    try:
        ser = serial.Serial(port=port, baudrate=baud, timeout=timeout, write_timeout=timeout)
        print(f"Opened serial port: {ser.port}")
        return ser
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
        return None


def generate_square_value(sample_idx, samples_per_half_period, low=LOW_VALUE, high=HIGH_VALUE):
    """
    Returns LOW or HIGH depending on which half-period we are in.
    """
    if samples_per_half_period <= 0:
        # Degenerate case: just alternate every sample
        return high if (sample_idx % 2 == 0) else low

    half_period_index = sample_idx // samples_per_half_period
    is_high = (half_period_index % 2 == 0)
    return high if is_high else low


def square_value_at_time(t, freq=SQUARE_FREQ_HZ, low=LOW_VALUE, high=HIGH_VALUE):
    """
    Time-driven square wave to avoid integer rounding drift.
    """
    if freq <= 0:
        return low
    half = 0.5 / freq
    # Which half-period are we in?
    half_index = int(t // half)
    is_high = (half_index % 2 == 0)
    return high if is_high else low


def main():
    ser = open_serial(SERIAL_PORT, BAUDRATE, TIMEOUT)
    if ser is None:
        return

    # Clean start: flush buffers and let device settle
    try:
        ser.reset_input_buffer()
        ser.reset_output_buffer()
    except Exception:
        pass
    time.sleep(STARTUP_DELAY)

    period = 1.0 / SAMPLE_RATE_HZ

    print("=== Data Integrity Test ===")
    print(f"Duration:       {DURATION} s")
    print(f"Sample rate:    {SAMPLE_RATE_HZ} Hz")
    print(f"Square freq:    {SQUARE_FREQ_HZ} Hz")
    print(f"Values:         LOW={LOW_VALUE}, HIGH={HIGH_VALUE}")
    print("===========================")

    timestamps   = []
    tx_values    = []
    rx_values    = []

    start_time = time.time()
    rx_buf = bytearray()

    try:
        while (time.time() - start_time) < DURATION:
            loop_start = time.time()
            t = loop_start - start_time

            # --- Build square wave sample to send (time-based) ---
            tx_value = square_value_at_time(t)
            tx_bytes = struct.pack('<H', tx_value)  # 16-bit unsigned, little endian

            # --- Send sample ---
            try:
                ser.write(tx_bytes)
                ser.flush()  # ensure bytes go out promptly
            except serial.SerialTimeoutException:
                print("Write timeout")
            except serial.SerialException as e:
                print(f"Serial write error: {e}")

            # --- Read echoed sample (accumulate until we have 2 bytes or timeout) ---
            rx_value = math.nan
            deadline = time.time() + READ_TIMEOUT
            try:
                while len(rx_buf) < 2 and time.time() < deadline:
                    chunk = ser.read(2 - len(rx_buf))
                    if chunk:
                        rx_buf.extend(chunk)
                    else:
                        # small sleep to avoid tight loop if port returns immediately on timeout
                        time.sleep(0.001)

                if len(rx_buf) >= 2:
                    rx_bytes = bytes(rx_buf[:2])
                    del rx_buf[:2]
                    rx_value = struct.unpack('<H', rx_bytes)[0]
            except serial.SerialException as e:
                print(f"Serial read error: {e}")
                rx_value = math.nan

            # --- Log data ---
            timestamps.append(t)
            tx_values.append(tx_value)
            rx_values.append(rx_value)

            # --- Rate control ---
            elapsed = time.time() - loop_start
            sleep_time = period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\nInterrupted by user.")

    finally:
        if ser.is_open:
            ser.close()
            print("Serial port closed.")

        # ================== Analysis and plotting ==========================
        if not timestamps:
            print("No samples collected.")
            return

        # Basic integrity stats
        total_samples   = len(tx_values)
        missing_samples = sum(1 for v in rx_values if math.isnan(v))
        mismatches      = 0

        for tx, rx in zip(tx_values, rx_values):
            if math.isnan(rx):
                continue
            if tx != rx:
                mismatches += 1

        print("\n=== Integrity Report ===")
        print(f"Total samples sent:       {total_samples}")
        print(f"Samples received (valid): {total_samples - missing_samples}")
        print(f"Missing samples:          {missing_samples}")
        print(f"Mismatched samples:       {mismatches}")
        if total_samples > 0:
            print(f"Mismatch ratio:           {mismatches / total_samples:.6f}")
        print("====================================================")

        # Plot TX and RX over time
        plt.figure(figsize=(10, 6))
        plt.step(timestamps, tx_values, where='post', label='TX (sent)', linewidth=1.5)
        plt.step(timestamps, rx_values, where='post', label='RX (received)', linewidth=1.0, alpha=0.8)

        plt.xlabel("Time (s)")
        plt.ylabel("Sample value (uint16)")
        plt.title("Serial Data Integrity Test – Square Wave")
        plt.legend()
        plt.grid(True)
        plt.tight_layout()

        # Highlight mismatches as red dots
        mismatch_times = []
        mismatch_vals  = []
        for tt, tx, rx in zip(timestamps, tx_values, rx_values):
            if math.isnan(rx):
                continue
            if tx != rx:
                mismatch_times.append(tt)
                mismatch_vals.append(rx)

        if mismatch_times:
            plt.scatter(mismatch_times, mismatch_vals, marker='x', s=30, label='Mismatches')

        # Save before showing (safer with various backends)
        filename = f'serial_sync_test_plot.png'
        full_path = os.path.abspath(filename)
        plt.savefig(full_path, dpi=150)
        print(f"Plot saved to: {full_path}")

        plt.show()

if __name__ == "__main__":
    main()
