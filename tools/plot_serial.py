"""
Simple serial plotter for telemetry emitted by the ESP32 firmware.
Expects CSV lines: timestamp_ms,setpoint,adc_reading,control_signal

Usage (Windows PowerShell):
  python tools/plot_serial.py COM3 --baud 115200


The script reads serial lines and updates a live matplotlib plot.
"""
import argparse
import threading
import time
from collections import deque
import csv
from datetime import datetime
import os
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import serial
import serial.tools.list_ports  # For COM port listing


def reader_thread(ser, data_queue, stop_event, saved_rows):
    print("Reader thread started, waiting for data...")
    line_count = 0
    while not stop_event.is_set():
        try:
            line = ser.readline().decode('utf-8').strip()
            if not line:
                continue
            # Debug: show all received lines
            if line_count < 5:  # Show first 5 lines
                print(f"Received: {line}")
                line_count += 1
            # Only accept telemetry lines prefixed with 'TEL,'
            if not line.startswith('TEL,'):
                continue
            parts = line.split(',')
            # Expect: TEL,timestamp_ms,setpoint,adc_reading,control_signal,solenoid_state
            if len(parts) != 6:
                print(f"Wrong number of parts ({len(parts)}): {line}")
                continue
            t_ms = int(parts[1])
            setpoint = int(parts[2])
            adc = int(parts[3])
            control = int(parts[4])
            solenoid = int(parts[5])
            data_queue.append((t_ms, setpoint, adc, control, solenoid))
            # Append saved row with local receive timestamp
            try:
                saved_rows.append((t_ms, setpoint, adc, control, solenoid, datetime.now().isoformat()))
            except Exception:
                pass
        except Exception:
            continue


def list_ports():
    """List all available COM ports"""
    print("\nAvailable COM ports:")
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("  No COM ports found!")
    for p in ports:
        print(f"  {p.device}: {p.description}")

def plot_from_csv(csv_file):
    """Plot data from a previously saved CSV file"""
    print(f"Loading data from {csv_file}...")
    
    if not os.path.exists(csv_file):
        print(f"Error: File not found: {csv_file}")
        return
    
    timestamps = []
    setpoints = []
    adcs = []
    controls = []
    solenoids = []
    
    try:
        with open(csv_file, 'r') as f:
            reader = csv.reader(f)
            header = next(reader)  # Skip header
            for row in reader:
                if len(row) >= 5:
                    timestamps.append(int(row[0]))
                    setpoints.append(int(row[1]))
                    adcs.append(int(row[2]))
                    controls.append(int(row[3]))
                    solenoids.append(int(row[4]))
    except Exception as e:
        print(f"Error reading CSV: {e}")
        return
    
    if not timestamps:
        print("No data found in CSV file")
        return
    
    print(f"Loaded {len(timestamps)} data points")
    
    # Create plot
    fig, ax = plt.subplots(2, 1, sharex=True, figsize=(12, 8))
    for a in ax:
        a.grid(True, linestyle='--', alpha=0.7)
        a.set_facecolor('#f0f0f0')
    fig.patch.set_facecolor('white')
    
    (ax1, ax2) = ax
    
    # Convert timestamps to relative time (seconds)
    t0 = timestamps[0]
    times = [(t - t0) / 1000.0 for t in timestamps]  # Convert to seconds
    
    # ADC conversion constants
    ADC_MAX = 4095
    ADC_VREF = 3.3
    PWM_MAX = 1023
    
    # Convert values
    volts_set = [(s / ADC_MAX) * ADC_VREF for s in setpoints]
    volts_adc = [(a / ADC_MAX) * ADC_VREF for a in adcs]
    pwm_pct = [(c / PWM_MAX) * 100.0 for c in controls]
    sol_pct = [s * 100.0 for s in solenoids]
    
    # Plot voltage
    ax1.plot(times, volts_set, label='Setpoint', alpha=0.7)
    ax1.plot(times, volts_adc, label='ADC Reading', linewidth=2)
    ax1.set_ylabel('Voltage (V)')
    ax1.set_ylim(0, ADC_VREF)
    ax1.legend()
    ax1.set_title(f'Telemetry Data from {os.path.basename(csv_file)}')
    
    # Plot PWM and solenoid
    ax2.plot(times, pwm_pct, label='PWM duty %', linewidth=2)
    ax2.plot(times, sol_pct, label='Solenoid', linewidth=2, linestyle='--')
    ax2.set_ylabel('PWM / Solenoid (%)')
    ax2.set_xlabel('Time (seconds)')
    ax2.set_ylim(0, 100)
    ax2.legend()
    
    plt.tight_layout()
    plt.show()

def main():
    parser = argparse.ArgumentParser(description='Plot real-time telemetry from ESP32')
    parser.add_argument('port', nargs='?', help='Serial port (e.g., COM3)')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate (default: 115200)')
    parser.add_argument('--window', type=int, default=20000, help='Window size in ms (default: 20000)')
    parser.add_argument('--list', '-l', action='store_true', help='List available COM ports and exit')
    parser.add_argument('--csv', type=str, help='Plot data from a previously saved CSV file instead of live data')
    args = parser.parse_args()

    if args.list or (not args.port and not args.csv):
        list_ports()
        if not args.port and not args.csv:
            print("\nUsage examples:")
            print("  List ports:  python tools/plot_serial.py --list")
            print("  Plot live:   python tools/plot_serial.py COM3")
            print("  Plot CSV:    python tools/plot_serial.py --csv telemetry/20251022_1430.csv")
            print("  With options: python tools/plot_serial.py COM3 --baud 115200 --window 30000")
        return

    # If plotting from CSV file
    if args.csv:
        plot_from_csv(args.csv)
        return

    # Try to open the serial port
    try:
        print(f"Opening {args.port} at {args.baud} baud...")
        ser = serial.Serial(args.port, args.baud, timeout=1)
    except Exception as e:
        print(f"ERROR: Cannot open {args.port}. The port might be in use.")
        print(f"Error details: {str(e)}")
        print("\nTips:")
        print("1. Close any open serial monitors (ESP-IDF Monitor, Arduino, PuTTY)")
        print("2. Check if the correct COM port is selected")
        print("3. Try unplugging and re-plugging the device")
        list_ports()
        return
    
    stop_event = threading.Event()
    data_queue = deque(maxlen=10000)
    saved_rows = []

    # Create telemetry directory if it doesn't exist
    telemetry_dir = "telemetry"
    if not os.path.exists(telemetry_dir):
        os.makedirs(telemetry_dir)
        print(f"Created directory: {telemetry_dir}")

    t = threading.Thread(target=reader_thread, args=(ser, data_queue, stop_event, saved_rows), daemon=True)
    t.start()

    # Use default style with customizations
    fig, ax = plt.subplots(2, 1, sharex=True, figsize=(10, 6))
    
    # Configure subplot appearance
    for a in ax:
        a.grid(True, linestyle='--', alpha=0.7)
        a.set_facecolor('#f0f0f0')
    
    fig.patch.set_facecolor('white')
    (ax1, ax2) = ax
    line_setpoint, = ax1.plot([], [], label='setpoint')
    line_adc, = ax1.plot([], [], label='adc_reading')
    line_control, = ax2.plot([], [], label='PWM duty %', linewidth=2)
    line_solenoid, = ax2.plot([], [], label='Solenoid', linewidth=2, linestyle='--')

    ax1.set_ylabel('Voltage ADC (V)')
    ax2.set_ylabel('PWM / Solenoid (%)')
    ax2.set_xlabel('Time (ms)')
    ax1.legend()
    ax2.legend()

    window = args.window
    
    # ADC conversion constants (ESP32-C3: 12-bit ADC, 3.3V reference)
    ADC_MAX = 4095
    ADC_VREF = 3.3
    PWM_MAX = 1023  # 10-bit PWM

    def init():
        ax1.set_xlim(0, window)
        ax1.set_ylim(0, ADC_VREF)  # 0-3.3V
        ax2.set_xlim(0, window)
        ax2.set_ylim(0, 100)  # 0-100%
        return line_setpoint, line_adc, line_control, line_solenoid

    def update(frame):
        if not data_queue:
            return line_setpoint, line_adc, line_control, line_solenoid
        data = list(data_queue)
        t0 = data[-1][0]
        xs = [d[0] - t0 + window for d in data]
        # Convert ADC values to volts
        ys_set = [(d[1] / ADC_MAX) * ADC_VREF for d in data]
        ys_adc = [(d[2] / ADC_MAX) * ADC_VREF for d in data]
        # Convert PWM to percentage
        ys_ctrl = [(d[3] / PWM_MAX) * 100.0 for d in data]
        # Solenoid state as percentage (0% or 100%)
        ys_sol = [d[4] * 100.0 for d in data]

        # saved_rows are appended in the reader thread; nothing to do here

        line_setpoint.set_data(xs, ys_set)
        line_adc.set_data(xs, ys_adc)
        line_control.set_data(xs, ys_ctrl)
        line_solenoid.set_data(xs, ys_sol)

        # Fixed y-limits for voltage (0-3.3V)
        ax1.set_ylim(0, ADC_VREF)
        
        # Fixed y-limits for PWM/Solenoid (0-100%)
        ax2.set_ylim(0, 100)

        ax1.set_xlim(0, window)
        ax2.set_xlim(0, window)
        return line_setpoint, line_adc, line_control, line_solenoid

    ani = animation.FuncAnimation(
        fig,
        update,
        init_func=init,
        interval=100,
        blit=False,
        cache_frame_data=False,  # Don't cache frame data
        save_count=100  # Keep only last 100 frames
    )

    def on_key(event):
        # Press 's' to save current CSV, 'q' to save and quit
        if event.key == 's':
            save_filename = os.path.join(telemetry_dir, f"{datetime.now().strftime('%Y%m%d_%H%M')}.csv")
            with open(save_filename, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['device_timestamp_ms','setpoint','adc_reading','control_signal','solenoid_state','received_iso'])
                writer.writerows(saved_rows)
            print(f"Saved telemetry to {save_filename}")
        elif event.key == 'q':
            save_filename = os.path.join(telemetry_dir, f"{datetime.now().strftime('%Y%m%d_%H%M')}.csv")
            with open(save_filename, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['device_timestamp_ms','setpoint','adc_reading','control_signal','solenoid_state','received_iso'])
                writer.writerows(saved_rows)
            print(f"Saved telemetry to {save_filename}")
            plt.close('all')

    fig.canvas.mpl_connect('key_press_event', on_key)

    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        ser.close()


if __name__ == '__main__':
    main()
