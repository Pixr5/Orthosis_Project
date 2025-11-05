"""
Interactive Control GUI for ESP32 Motor Control
Features:
- Real-time telemetry graphs (voltage and PWM)
- FFT analysis of ADC readings
- Setpoint adjustment with up/down arrows
- Current value display
- Serial communication for setpoint control
- Auto-save telemetry to CSV

Usage (Windows PowerShell):
  python tools/control_gui.py COM3
"""

import argparse
import threading
import time
from collections import deque
import csv
from datetime import datetime
import os
import serial
import serial.tools.list_ports
import tkinter as tk
from tkinter import ttk
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import matplotlib.animation as animation
import numpy as np


class MotorControlGUI:
    def __init__(self, root, serial_port, baud_rate=115200):
        self.root = root
        self.root.title("ESP32 Motor Control")
        self.root.geometry("1000x700")
        
        # Start in fullscreen mode
        self.root.state('zoomed')  # Windows fullscreen
        # Alternative for cross-platform: self.root.attributes('-fullscreen', True)
        
        # Allow ESC to exit fullscreen
        self.root.bind('<Escape>', self.toggle_fullscreen)
        
        # Serial communication
        self.serial_port = serial_port
        self.baud_rate = baud_rate
        self.ser = None
        self.stop_event = threading.Event()
        
        # Data storage
        self.data_queue = deque(maxlen=3000)  # Store last 3000 points for better curves
        self.saved_rows = []
        
        # Sliding window parameters
        self.time_window_seconds = 10.0  # Show last 10 seconds of data
        
        # Current setpoint (percentage 0.0 - 100.0)
        self.setpoint_pct = 0.0
        
        # Constants
        self.ADC_MAX = 4095
        self.ADC_VREF = 3.3
        self.PWM_MAX = 1023
        
        # Create telemetry directory
        self.telemetry_dir = "telemetry"
        if not os.path.exists(self.telemetry_dir):
            os.makedirs(self.telemetry_dir)
        
        # Build UI
        self.build_ui()
        # Initialize display values
        self.update_setpoint_display()
        
        # Connect to serial
        self.connect_serial()
        
        # Start animation with faster refresh rate
        self.ani = animation.FuncAnimation(
            self.fig, self.update_plot, interval=50, blit=True, cache_frame_data=False
        )
        
        # Handle window close
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
    
    def build_ui(self):
        """Build the GUI interface"""
        # Main container
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Configure grid weights
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(0, weight=1)
        main_frame.rowconfigure(1, weight=1)
        
        # ===== Control Panel (Top) =====
        control_frame = ttk.LabelFrame(main_frame, text="Control Panel", padding="10")
        control_frame.grid(row=0, column=0, sticky=(tk.W, tk.E), pady=(0, 10))
        
        # Mode selector
        ttk.Label(control_frame, text="Mode:", font=('Arial', 12)).grid(row=0, column=0, padx=5)
        self.mode_var = tk.StringVar(value="Manual %")  # Default to Manual control
        mode_combo = ttk.Combobox(control_frame, textvariable=self.mode_var, width=14, values=["ADC", "Manual %"], state='readonly')
        mode_combo.grid(row=0, column=1, padx=5)
        mode_combo.bind('<<ComboboxSelected>>', self.on_mode_change)

        # Setpoint display and controls
        ttk.Label(control_frame, text="Setpoint:", font=('Arial', 12)).grid(row=0, column=2, padx=5)

        # Big label shows percentage; small label shows volts and ADC
        self.setpoint_voltage_label = ttk.Label(control_frame, text="0.0%", font=('Arial', 20, 'bold'), foreground='blue')
        self.setpoint_voltage_label.grid(row=0, column=3, padx=10)

        self.setpoint_adc_label = ttk.Label(control_frame, text="(0.00 V, 0 ADC)", font=('Arial', 10), foreground='gray')
        self.setpoint_adc_label.grid(row=0, column=4, padx=5)
        
        # Up/Down buttons
        button_frame = ttk.Frame(control_frame)
        button_frame.grid(row=0, column=5, padx=20)
        
        self.up_button = ttk.Button(button_frame, text="▲", width=5, command=self.increase_setpoint)
        self.up_button.grid(row=0, column=0, padx=2)
        
        self.down_button = ttk.Button(button_frame, text="▼", width=5, command=self.decrease_setpoint)
        self.down_button.grid(row=0, column=1, padx=2)
        
        # Step size (percentage)
        ttk.Label(control_frame, text="Step (%):").grid(row=0, column=6, padx=(20, 5))
        self.step_var = tk.StringVar(value="5")
        step_combo = ttk.Combobox(control_frame, textvariable=self.step_var, width=8, values=["1", "2", "5", "10", "20"], state='readonly')
        step_combo.grid(row=0, column=7, padx=5)
        
        # Preset buttons row
        preset_frame = ttk.Frame(control_frame)
        preset_frame.grid(row=1, column=0, columnspan=8, pady=(10, 0))
        
        ttk.Label(preset_frame, text="Quick Set:", font=('Arial', 10)).grid(row=0, column=0, padx=(0, 10))
        
        # Preset percentage buttons
        preset_values = [10, 20, 50, 75, 100]
        for i, pct in enumerate(preset_values):
            btn = ttk.Button(
                preset_frame, 
                text=f"{pct}%", 
                width=6,
                command=lambda p=pct: self.set_preset_percentage(p)
            )
            btn.grid(row=0, column=i+1, padx=2)
        
        # Add time window control
        ttk.Label(preset_frame, text="Time Window:", font=('Arial', 10)).grid(row=0, column=7, padx=(20, 5))
        self.time_window_var = tk.StringVar(value=str(int(self.time_window_seconds)))
        time_combo = ttk.Combobox(
            preset_frame, 
            textvariable=self.time_window_var, 
            width=8, 
            values=["5", "10", "15", "20", "30"], 
            state='readonly'
        )
        time_combo.grid(row=0, column=8, padx=5)
        time_combo.bind('<<ComboboxSelected>>', self.on_time_window_change)
        ttk.Label(preset_frame, text="sec", font=('Arial', 10)).grid(row=0, column=9, padx=(2, 0))
        
        # Current readings display
        reading_frame = ttk.Frame(control_frame)
        reading_frame.grid(row=2, column=0, columnspan=8, pady=(10, 0))
        
        ttk.Label(reading_frame, text="Current ADC read value :", font=('Arial', 10)).grid(
            row=0, column=0, padx=5
        )
        self.current_adc_label = ttk.Label(
            reading_frame, text="0 (0.00 V)", font=('Arial', 10, 'bold')
        )
        self.current_adc_label.grid(row=0, column=1, padx=10)
        
        ttk.Label(reading_frame, text="PWM duty:", font=('Arial', 10)).grid(
            row=0, column=2, padx=(20, 5)
        )
        self.current_pwm_label = ttk.Label(
            reading_frame, text="0 (0.0%)", font=('Arial', 10, 'bold')
        )
        self.current_pwm_label.grid(row=0, column=3, padx=10)
        
        ttk.Label(reading_frame, text="Solenoid state :", font=('Arial', 10)).grid(
            row=0, column=4, padx=(20, 5)
        )
        self.solenoid_label = ttk.Label(
            reading_frame, text="OFF", font=('Arial', 10, 'bold'), foreground='gray'
        )
        self.solenoid_label.grid(row=0, column=5, padx=10)
        
        # Status bar
        self.status_label = ttk.Label(
            control_frame, text="Connecting...", foreground='orange', wraplength=800
        )
        self.status_label.grid(row=2, column=0, columnspan=8, pady=(10, 0), sticky=(tk.W, tk.E))
        
        # ===== Graph Panel (Bottom) =====
        graph_frame = ttk.LabelFrame(main_frame, text="Live Telemetry", padding="10")
        graph_frame.grid(row=1, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        graph_frame.columnconfigure(0, weight=1)
        graph_frame.rowconfigure(0, weight=1)
        
        # Create matplotlib figure with 3 subplots (time-domain + FFT)
        self.fig = Figure(figsize=(10, 8), dpi=100)
        self.ax1 = self.fig.add_subplot(3, 1, 1)
        self.ax2 = self.fig.add_subplot(3, 1, 2, sharex=self.ax1)
        self.ax3 = self.fig.add_subplot(3, 1, 3)  # FFT plot
        
        # Configure axes with clean appearance
        self.ax1.set_ylabel('Voltage (V)', fontsize=10)
        self.ax1.set_ylim(0, self.ADC_VREF)
        self.ax1.grid(True, linestyle='--', alpha=0.5)
        self.ax1.set_facecolor("#ffffff")
        # Remove y-axis tick labels but keep grid
        self.ax1.tick_params(axis='y', which='both', labelleft=False)
        self.ax1.tick_params(axis='x', which='both', labelbottom=False)  # Hide x labels on top plot
        
        self.ax2.set_ylabel('PWM (%)', fontsize=10)
        self.ax2.set_xlabel('Time (seconds)', fontsize=10)
        self.ax2.set_ylim(0, 100)
        self.ax2.grid(True, linestyle='--', alpha=0.5)
        self.ax2.set_facecolor("#ffffff")
        # Remove y-axis tick labels but keep Time (x-axis) labels
        self.ax2.tick_params(axis='y', which='both', labelleft=False)
        # Keep x-axis labels and make them stick to the graph
        self.ax2.tick_params(axis='x', which='both', labelbottom=True, pad=2)
        
        # FFT plot configuration
        self.ax3.set_ylabel('Magnitude', fontsize=10)
        self.ax3.set_xlabel('Frequency (Hz)', fontsize=10)
        self.ax3.grid(True, linestyle='--', alpha=0.5)
        self.ax3.set_facecolor("#ffffff")
        self.ax3.set_xlim(0, 50)  # Show 0-50 Hz by default
        # Remove y-axis tick labels but keep Frequency (x-axis) labels
        self.ax3.tick_params(axis='y', which='both', labelleft=False)
        # Keep x-axis labels and make them stick to the graph
        self.ax3.tick_params(axis='x', which='both', labelbottom=True, pad=2)
        
        # Initialize empty line objects for efficient updating
        self.line_setpoint, = self.ax1.plot([], [], label='Setpoint', linewidth=2, color='blue', alpha=0.7)
        self.line_adc, = self.ax1.plot([], [], label='ADC Reading', linewidth=2, color='red')
        self.line_pwm, = self.ax2.plot([], [], label='PWM duty %', linewidth=2, color='green')
        self.line_fft, = self.ax3.plot([], [], linewidth=1, color='purple')
        
        # Add legends
        self.ax1.legend(loc='upper right')
        # No legend needed for PWM plot since it's just one line
        
        # FFT update counter for performance
        self.fft_update_counter = 0
        
        self.fig.tight_layout()
        
        # Embed in tkinter
        self.canvas = FigureCanvasTkAgg(self.fig, master=graph_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Save button
        save_frame = ttk.Frame(main_frame)
        save_frame.grid(row=2, column=0, pady=(10, 0))
        
        ttk.Button(save_frame, text="Save CSV", command=self.save_csv).pack(side=tk.LEFT, padx=5)
        self.save_status_label = ttk.Label(save_frame, text="", foreground='green')
        self.save_status_label.pack(side=tk.LEFT, padx=5)
        
    def connect_serial(self):
        """Connect to the serial port and start thread"""
        try:
            print(f"Connecting to {self.serial_port} at {self.baud_rate} baud...")
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            self.status_label.config(text=f"Connected to {self.serial_port}", foreground='green')
            
            # Start thread
            self.reader = threading.Thread(
                target=self.reader_thread, daemon=True
            )
            self.reader.start()
            
            # Set initial mode to Manual and send initial setpoint
            self.root.after(1000, self.on_mode_change)  # Delay to ensure connection is stable
            
        except Exception as e:
            error_msg = f"Connection failed: {self.serial_port}"
            self.status_label.config(text=error_msg, foreground='red')
            print(f"Serial connection error: {e}")
            print(f"Status label updated to: '{error_msg}'")
    
    def reader_thread(self):
        """Background thread to read serial data"""
        print("Thread started")
        while not self.stop_event.is_set():
            try:
                if self.ser and self.ser.in_waiting:
                    # Read raw bytes and handle encoding errors gracefully
                    try:
                        raw_data = self.ser.readline()
                        line = raw_data.decode('utf-8', errors='ignore').strip()
                    except UnicodeDecodeError:
                        # Skip corrupted data and continue
                        print("Skipping corrupted serial data")
                        continue
                    
                    # Skip empty lines or corrupted data
                    if not line or len(line) < 5:
                        continue
                    
                    # Only accept telemetry lines prefixed with 'TEL,'
                    if not line.startswith('TEL,'):
                        continue
                    
                    parts = line.split(',')
                    # Expect: TEL,timestamp_ms,setpoint,adc_reading,control_signal,solenoid_state
                    if len(parts) != 6:
                        continue
                    
                    try:
                        t_ms = int(parts[1])
                        setpoint = int(parts[2])
                        adc = int(parts[3])
                        control = int(parts[4])
                        solenoid = int(parts[5])
                    except (ValueError, IndexError):
                        # Skip malformed data
                        continue
                    
                    # Store actual data from ESP32 without modification
                    self.data_queue.append((t_ms, setpoint, adc, control, solenoid))
                    
                    # Save for CSV export
                    self.saved_rows.append((
                        t_ms, setpoint, adc, control, solenoid, 
                        datetime.now().isoformat()
                    ))
                    
                    # Update current readings in UI (thread-safe via after)
                    self.root.after(0, self.update_current_readings, adc, control, solenoid)
                    
            except serial.SerialException as e:
                print(f"Serial connection lost: {e}")
                # Update status in UI thread
                self.root.after(0, lambda: self.status_label.config(
                    text=f"Connection lost: {self.serial_port}", foreground='red'
                ))
                break
            except Exception as e:
                print(f"Reader error: {e}")
                time.sleep(0.001)  # Reduced sleep for faster response
    
    def update_current_readings(self, adc, control, solenoid):
        """Update the current readings display"""
        voltage = (adc / self.ADC_MAX) * self.ADC_VREF
        pwm_pct = (control / self.PWM_MAX) * 100.0
        
        self.current_adc_label.config(text=f"{adc} ({voltage:.2f} V)")
        self.current_pwm_label.config(text=f"{control} ({pwm_pct:.1f}%)")
        
        if solenoid > 0:
            self.solenoid_label.config(text="ON", foreground='green')
        else:
            self.solenoid_label.config(text="OFF", foreground='orange')
    
    def update_plot(self, frame):
        """Optimized animation update function using line objects"""
        if len(self.data_queue) < 2:
            return []
        
        # Extract data (more efficient list comprehension)
        data = list(self.data_queue)
        timestamps = [d[0] for d in data]
        setpoints = [d[1] for d in data]
        adcs = [d[2] for d in data]
        controls = [d[3] for d in data]
        solenoids = [d[4] for d in data]
        
        # Convert to relative time and filter for sliding window
        current_time = timestamps[-1] / 1000.0  # Current time in seconds
        
        # Filter data to only show the last time_window_seconds
        filtered_data = []
        for i, timestamp in enumerate(timestamps):
            time_sec = timestamp / 1000.0
            if (current_time - time_sec) <= self.time_window_seconds:
                filtered_data.append((time_sec, setpoints[i], adcs[i], controls[i], solenoids[i]))
        
        if not filtered_data:
            return []
        
        # Extract filtered data
        times = [d[0] - current_time + self.time_window_seconds for d in filtered_data]  # Relative to window
        setpoints_filt = [d[1] for d in filtered_data]
        adcs_filt = [d[2] for d in filtered_data]
        controls_filt = [d[3] for d in filtered_data]
        solenoids_filt = [d[4] for d in filtered_data]
        
        # Convert values
        volts_set = [(s / self.ADC_MAX) * self.ADC_VREF for s in setpoints_filt]
        volts_adc = [(a / self.ADC_MAX) * self.ADC_VREF for a in adcs_filt]
        pwm_pct = [(c / self.PWM_MAX) * 100.0 for c in controls_filt]
        
        # Update line data with filtered data
        self.line_setpoint.set_data(times, volts_set)
        self.line_adc.set_data(times, volts_adc)
        self.line_pwm.set_data(times, pwm_pct)
        
        # Set fixed sliding window for x-axis (always shows last 10 seconds)
        self.ax1.set_xlim(0, self.time_window_seconds)
        self.ax2.set_xlim(0, self.time_window_seconds)
        
        # Update FFT less frequently for better performance (every 5th frame = 250ms)
        self.fft_update_counter += 1
        if self.fft_update_counter >= 5 and len(adcs) >= 32:
            self.fft_update_counter = 0
            
            # Calculate sample rate from timestamps
            if len(times) > 1:
                dt = np.mean(np.diff(times))
                sample_rate = 1.0 / dt if dt > 0 else 10.0
            else:
                sample_rate = 10.0
            
            # Remove DC component (mean) and compute FFT efficiently
            adc_signal = np.array(adcs) - np.mean(adcs)
            window = np.hanning(len(adc_signal))
            adc_windowed = adc_signal * window
            
            # Compute FFT
            fft_vals = np.fft.rfft(adc_windowed)
            fft_freq = np.fft.rfftfreq(len(adc_windowed), 1.0 / sample_rate)
            fft_mag = np.abs(fft_vals)
            
            # Update FFT line (skip DC component at index 0)
            self.line_fft.set_data(fft_freq[1:], fft_mag[1:])
            self.ax3.set_xlim(0, min(50, sample_rate / 2))
            if len(fft_mag) > 1:
                self.ax3.set_ylim(0, np.max(fft_mag[1:]) * 1.1)
            self.ax3.set_title(f'ADC FFT (Sample Rate: {sample_rate:.1f} Hz)', fontsize=9)
        
        # Return all artist objects for blitting (massive performance boost)
        return [self.line_setpoint, self.line_adc, self.line_pwm, self.line_fft]
    
    def increase_setpoint(self):
        """Increase setpoint by step percentage value"""
        step_pct = float(self.step_var.get())
        self.setpoint_pct = min(100.0, self.setpoint_pct + step_pct)
        self.update_setpoint_display()
        self.send_setpoint()
    
    def decrease_setpoint(self):
        """Decrease setpoint by step percentage value"""
        step_pct = float(self.step_var.get())
        self.setpoint_pct = max(0.0, self.setpoint_pct - step_pct)
        self.update_setpoint_display()
        self.send_setpoint()
    
    def set_preset_percentage(self, percentage):
        """Set setpoint to a specific percentage preset"""
        self.setpoint_pct = float(percentage)
        self.update_setpoint_display()
        self.send_setpoint()
    
    def update_setpoint_display(self):
        """Update the setpoint display labels"""
        adc_value = int(round((self.setpoint_pct / 100.0) * self.ADC_MAX))
        voltage = (adc_value / self.ADC_MAX) * self.ADC_VREF
        self.setpoint_voltage_label.config(text=f"{self.setpoint_pct:.1f}%")
        self.setpoint_adc_label.config(text=f"({voltage:.2f} V, {adc_value} ADC)")
        # Disable/enable controls based on mode
        is_manual = (self.mode_var.get() == "Manual %")
        state = (tk.NORMAL if is_manual else tk.DISABLED)
        self.up_button.configure(state=state)
        self.down_button.configure(state=state)
        # Dim setpoint labels in ADC mode
        fg = ('blue' if is_manual else 'gray')
        self.setpoint_voltage_label.configure(foreground=fg)
        self.setpoint_adc_label.configure(foreground=('gray' if is_manual else '#aaaaaa'))
    
    def send_setpoint(self):
        """Send setpoint command to ESP32 via serial (as ADC value)"""
        if self.ser and self.ser.is_open:
            try:
                adc_value = int(round((self.setpoint_pct / 100.0) * self.ADC_MAX))
                command = f"SET,{adc_value}\n"
                self.ser.write(command.encode('utf-8'))
                print(f"Sent: {command.strip()} (from {self.setpoint_pct:.1f}%)")
            except Exception as e:
                print(f"Error sending setpoint: {e}")

    def on_mode_change(self, event=None):
        """Send mode change to ESP32 and refresh UI state"""
        mode = self.mode_var.get()
        if self.ser and self.ser.is_open:
            try:
                if mode == "ADC":
                    self.ser.write(b"MODE,ADC\n")
                    print("Sent: MODE,ADC")
                else:
                    self.ser.write(b"MODE,MAN\n")
                    print("Sent: MODE,MAN")
            except Exception as e:
                print(f"Error sending mode: {e}")
        # Update controls enabled/disabled
        self.update_setpoint_display()
    
    def on_time_window_change(self, event=None):
        """Update time window when combobox selection changes"""
        try:
            new_window = float(self.time_window_var.get())
            self.time_window_seconds = new_window
            print(f"Time window changed to {new_window} seconds")
        except ValueError:
            print("Invalid time window value")
    
    def save_csv(self):
        """Save telemetry data to CSV file"""
        if not self.saved_rows:
            self.save_status_label.config(text="No data to save", foreground='orange')
            return
        
        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M")
        filename = os.path.join(self.telemetry_dir, f"{timestamp_str}.csv")
        
        try:
            with open(filename, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    'timestamp_ms', 'setpoint', 'adc_reading', 
                    'control_signal', 'solenoid_state', 'local_timestamp'
                ])
                writer.writerows(self.saved_rows)
            
            self.save_status_label.config(
                text=f"Saved: {filename}", foreground='green'
            )
            print(f"Saved {len(self.saved_rows)} rows to {filename}")
            
            # Clear save status after 3 seconds
            self.root.after(3000, lambda: self.save_status_label.config(text=""))
            
        except Exception as e:
            self.save_status_label.config(
                text=f"Save error: {e}", foreground='red'
            )
            print(f"CSV save error: {e}")
    
    def on_closing(self):
        """Handle window close event"""
        print("Closing application...")
        self.stop_event.set()
        
        if self.ser and self.ser.is_open:
            self.ser.close()
        
        self.root.quit()
        self.root.destroy()
    
    def toggle_fullscreen(self, event=None):
        """Toggle between fullscreen and windowed mode (ESC key)"""
        current_state = self.root.state()
        if current_state == 'zoomed':
            self.root.state('normal')
            self.root.geometry("1000x700")
        else:
            self.root.state('zoomed')


def list_ports():
    """List all available COM ports"""
    print("\nAvailable COM ports:")
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("  No COM ports found!")
    for p in ports:
        print(f"  {p.device}: {p.description}")


def main():
    parser = argparse.ArgumentParser(description='Interactive Motor Control GUI')
    parser.add_argument('port', nargs='?', help='Serial port (e.g., COM3)')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate (default: 115200)')
    parser.add_argument('--list', '-l', action='store_true', help='List available COM ports and exit')
    args = parser.parse_args()
    
    if args.list or not args.port:
        list_ports()
        if not args.port:
            print("\nUsage:")
            print("  List ports:  python tools/control_gui.py --list")
            print("  Run GUI:     python tools/control_gui.py COM3")
        return
    
    # Create and run GUI
    root = tk.Tk()
    app = MotorControlGUI(root, args.port, args.baud)
    root.mainloop()


if __name__ == '__main__':
    main()
