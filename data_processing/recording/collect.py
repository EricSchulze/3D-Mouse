import os
import csv
import time
import serial

# UART serial port settings
SERIAL_PORT = 'COM36'   # Change to your UART port (e.g., 'COM3' on Windows, '/dev/ttyUSB0' on Linux)
BAUD_RATE = 115200

# Session settings
SESSION_SECONDS = 3.0
SESSION_LABELS = ["+x", "-x", "+y", "-y", "+z", "-z", "+rx", "-rx", "+ry", "-ry", "+rz", "-rz"]

# Output directory
OUTPUT_DIR = "3d_data"

def parse_uart_line(line: str):
    """
    Parse a UART text line into 9 float values.
    Expected format: 'v1x,v1y,v1z,v2x,v2y,v2z,v3x,v3y,v3z'
    Returns list[float] of length 9 or None if not valid.
    """
    try:
        parts = [p.strip() for p in line.strip().split(',')]
        values = [float(x) for x in parts if x != ""]
        if len(values) == 9:
            return values
    except Exception:
        pass
    return None

def run_sessions(port: str, baud: int, out_dir: str, labels, session_seconds: float):
    # Ensure output directory exists
    os.makedirs(out_dir, exist_ok=True)

    print("This script will record 12 sessions, each 3 seconds long, into separate CSV files.")
    print("Each CSV row format: timestamp_s, s1_x, s1_y, s1_z, s2_x, s2_y, s2_z, s3_x, s3_y, s3_z")
    print(f"Output directory: {out_dir}")
    print("You will be prompted to press Enter before each session begins.")
    print("Make sure the device orientation matches the session label before starting each recording.\n")

    # Open serial port
    try:
        ser = serial.Serial(port, baud, timeout=0.1)
    except Exception as e:
        print(f"ERROR: Unable to open serial port {port} at {baud} baud. Details: {e}")
        return

    # Clear any stale buffered data
    try:
        ser.reset_input_buffer()
    except Exception:
        pass

    for idx, label in enumerate(labels, start=1):
        filename = os.path.join(out_dir, f"{label}.csv")
        prompt = f"[{idx}/{len(labels)}] Ready to record session '{label}' ({session_seconds:.1f}s). Press Enter to START..."
        input(prompt)

        # Clear buffered data again right before recording to avoid stale samples
        try:
            ser.reset_input_buffer()
        except Exception:
            pass

        print(f"Recording session '{label}' for {session_seconds:.1f}s...")

        row_count = 0
        timestamps = []  # for estimating sample rate

        try:
            with open(filename, mode='w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                # Header
                writer.writerow(["timestamp_s", "s1_x", "s1_y", "s1_z", "s2_x", "s2_y", "s2_z", "s3_x", "s3_y", "s3_z"])

                # Session timing
                t0 = time.perf_counter()
                end_time = t0 + session_seconds

                while True:
                    now = time.perf_counter()
                    if now >= end_time:
                        break

                    try:
                        if ser.in_waiting:
                            line_bytes = ser.readline()
                            try:
                                line = line_bytes.decode('utf-8', errors='ignore')
                            except Exception:
                                continue

                            vals = parse_uart_line(line)
                            if vals is not None:
                                # Use the time at receipt to create a real-time relative timestamp
                                t_rel = now - t0
                                writer.writerow([f"{t_rel:.6f}"] + [f"{v:.6f}" for v in vals])
                                row_count += 1
                                timestamps.append(now)
                        else:
                            # Sleep briefly to avoid busy-waiting when no data is available
                            time.sleep(0.001)
                    except Exception:
                        # Ignore transient read/parse errors and continue
                        continue

                # Ensure data is flushed to disk
                f.flush()
        except Exception as e:
            print(f"ERROR: Could not write to file {filename}. Details: {e}")
            continue

        # Compute approximate average frequency for this session
        avg_freq = None
        if len(timestamps) >= 2:
            dts = [timestamps[i] - timestamps[i - 1] for i in range(1, len(timestamps))]
            mean_dt = sum(dts) / len(dts) if dts else None
            if mean_dt and mean_dt > 0:
                avg_freq = 1.0 / mean_dt

        if row_count == 0:
            print(f"Session '{label}' completed. No valid data captured. Check wiring/port/format.\n")
        else:
            if avg_freq is not None:
                print(f"Session '{label}' completed. Rows: {row_count}. Approx. average sample rate: {avg_freq:.2f} Hz.\n")
            else:
                print(f"Session '{label}' completed. Rows: {row_count}. Not enough samples to estimate rate.\n")

    # Close serial port
    try:
        ser.close()
    except Exception:
        pass

    print("All sessions recorded successfully.")
    print(f"Data saved in directory: {out_dir}")
    print("Done.")

if __name__ == "__main__":
    run_sessions(SERIAL_PORT, BAUD_RATE, OUTPUT_DIR, SESSION_LABELS, SESSION_SECONDS)
