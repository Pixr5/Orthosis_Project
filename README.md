| Supported Targets | ESP32 | ESP32-C2 | ESP32-C3 | ESP32-C5 | ESP32-C6 | ESP32-C61 | ESP32-H2 | ESP32-H21 | ESP32-P4 | ESP32-S2 | ESP32-S3 |
| ----------------- | ----- | -------- | -------- | -------- | -------- | --------- | -------- | --------- | -------- | -------- | -------- |


## Interactive Control GUI (Recommended)

The project includes an interactive control GUI with real-time graphs and setpoint adjustment.

### Features:
- **Real-time telemetry graphs** showing voltage and PWM/solenoid status
- **Interactive setpoint control in percentage** with up/down arrows
- **Current readings display** (ADC, PWM, Solenoid state)
- **CSV data logging** with organized storage in `telemetry/` folder
- **Adjustable step sizes (percent)**: 1, 2, 5, 10, 20%

### Installation:

```powershell
python -m pip install pyserial matplotlib
```

### Usage:

1. Build and flash the firmware to your ESP32:
```esp-idf powershell
idf.py -p COM3 flash
```

2. Launch the control GUI (replace COM3 with your port and esp-idf powershell must be closed):
```powershell
python tools\control_gui.py COM3
```

The GUI will open showing:
- **Control Panel** (top): Setpoint display with up/down buttons, current readings
- **Live Graphs** (bottom): Voltage and PWM/Solenoid percentage over time

### Controls:
- **▲/▼ buttons**: Increase/decrease setpoint percentage
- **Step dropdown**: Choose adjustment step size (1–20%)
- **Save CSV button**: Export telemetry data with timestamp
- The ESP32 will continuously read the analog input and respond to setpoint changes


## Basic Telemetry Plotter (Alternative)

If you prefer a simple plotter without interactive controls:

```powershell
python tools\plot_serial.py COM3 --baud 115200
```

The plotter will open a matplotlib window showing setpoint/ADC and the control (PWM duty).

Keyboard shortcuts in the plot window:
- Press `s` to save the collected telemetry to a timestamped CSV file.
- Press `q` to save the CSV and quit the plotter.

### Replay saved CSV data:
```powershell
python tools\plot_serial.py --csv telemetry\20251022_1430.csv
```

CSV format: device_timestamp_ms,setpoint,adc_reading,control_signal,solenoid_state,received_iso

