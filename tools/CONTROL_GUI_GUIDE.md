# Control GUI User Guide

## Overview
The interactive control GUI provides a complete interface for controlling your ESP32 motor system with real-time visualization and setpoint adjustment.

## Prerequisites

### Python Dependencies
The GUI requires Python 3 and the following packages:
- **pyserial**: For serial communication with the ESP32
- **matplotlib**: For real-time graphing
- **tkinter**: For the GUI (usually included with Python)

### Installation

```powershell
# Install required packages
python -m pip install pyserial matplotlib
```

**Important**: Make sure you install `pyserial` (NOT `serial`). If you accidentally installed the wrong package, fix it with:
```powershell
# Remove incorrect packages
python -m pip uninstall serial pyserial -y

# Install correct package
python -m pip install pyserial
```

**Common Issue**: If you get the error `module 'serial' has no attribute 'Serial'`, you have the wrong `serial` package installed. Follow the commands above to fix it.

## Quick Start

1. **Build and flash firmware:**
   ```powershell
   cd C:\Users\rosse\Pwmtest
   idf.py -p COM3 flash
   ```

2. **Launch GUI:**
   ```powershell
   python tools\control_gui.py COM3
   ```

## GUI Layout

### Control Panel (Top Section)
```
┌─────────────────────────────────────────────────────┐
│  Setpoint: 50.0% (1.65 V, 2048 ADC)  [▲] [▼]  Step: 5% │
│  Current ADC: 2045 (1.64 V)   PWM: 512 (50.0%)       │
│  Solenoid: ON                                        │
│  Status: Connected to COM3                           │
└─────────────────────────────────────────────────────┘
```

- **Setpoint Display**: Shows target percentage, voltage, and ADC value
- **▲/▼ Buttons**: Increase/decrease setpoint percentage
- **Step Dropdown**: Choose adjustment increment (1, 2, 5, 10, 20%)
- **Current Readings**: Real-time ADC, PWM, and solenoid status
- **Status Bar**: Connection status

### Graph Panel (Bottom Section)
Two synchronized graphs:
1. **Voltage Graph** (top):
   - Blue line: Setpoint voltage (0-3.3V)
   - Red line: Actual ADC reading (0-3.3V)

2. **PWM/Solenoid Graph** (bottom):
   - Green line: PWM duty cycle percentage (0-100%)
   - Orange dashed line: Solenoid state (0% OFF, 100% ON)

## How It Works

### Communication Protocol
The GUI sends commands to the ESP32 via UART (percentage converted to ADC):
```
SET,<adc_value>\n
```

Example: `SET,2048\n` sets the setpoint to 2048 (≈50% → 1.65V)

The ESP32 responds with telemetry:
```
TEL,timestamp_ms,setpoint,adc_reading,control_signal,solenoid_state
```

### Control Flow
1. User clicks ▲ or ▼ button
2. GUI calculates new setpoint (current ± step)
3. GUI sends `SET,<value>` command to ESP32
4. ESP32 receives command in UART task
5. ESP32 updates PWM output based on new setpoint
6. ESP32 sends telemetry back to GUI
7. GUI updates graphs and current readings

## Features

### Setpoint Control
- **Range**: 0 to 100% (mapped to 0–4095 ADC; 0–3.3V)
- **Step sizes (percent)**: 1, 2, 5 (default), 10, 20

### Data Logging
Click **Save CSV** button to export telemetry data:
- Saved to `telemetry/YYYYMMDD_HHMM.csv`
- Includes all received data points
- Contains: timestamp, setpoint, ADC, PWM, solenoid, local timestamp

### Graph Features
- **Auto-scaling X-axis**: Shows last ~20 seconds of data
- **Fixed Y-axis**: 
  - Voltage: 0-3.3V
  - PWM/Solenoid: 0-100%
- **Live updates**: 10Hz refresh rate (100ms intervals)
- **Color-coded**: Easy visual distinction between signals

## Troubleshooting

### "Cannot open COM3"
- Close ESP-IDF monitor (`Ctrl+]`)
- Check COM port number in Device Manager
- Unplug and replug the ESP32

### "No data received"
- Verify firmware is running (check with `idf.py monitor`)
- Ensure correct baud rate (115200)
- Check USB cable and connection

### Graphs not updating
- Check serial connection status in status bar
- Verify ESP32 is sending telemetry (TEL, prefix)
- Restart GUI application

### Setpoint not responding
- Verify UART driver is installed on ESP32
- Check that firmware was rebuilt after code changes
- Monitor ESP32 logs for "Setpoint updated" messages

## Command Line Options

```powershell
# List available COM ports
python tools\control_gui.py --list

# Connect to specific port
python tools\control_gui.py COM3

# Use custom baud rate
python tools\control_gui.py COM3 --baud 115200
```

## Tips

1. **Start with low setpoint**: Use small steps to find the right operating range
2. **Monitor solenoid**: Solenoid activates when PWM > 0
3. **Save data regularly**: Click "Save CSV" to preserve telemetry for analysis
4. **Use appropriate steps**: 
   - Fine tuning: 10-50 steps
   - Normal operation: 100-200 steps
   - Quick changes: 500 steps

## System Architecture

```
┌──────────────┐     UART (115200)      ┌──────────────┐
│              │◄──── SET,<value> ───────┤              │
│   ESP32-C3   │                         │  Control GUI │
│              ├───── TEL,... ──────────►│   (Python)   │
└──────────────┘                         └──────────────┘
      │
      ├─► GPIO6: Motor PWM (20kHz)
      ├─► GPIO18: Solenoid (Digital)
      └─► GPIO4: ADC Input (Analog)
```

## Next Steps

- **Analyze saved data**: Use `plot_serial.py --csv <file>` to review
- **Adjust control parameters**: Modify step sizes for your application
- **Monitor performance**: Watch voltage tracking and PWM response
- **Export data**: Use CSV files for further analysis in Excel/MATLAB

## Technical Details

- **Update Rate**: 10Hz (100ms loop)
- **PWM Frequency**: 20kHz (silent operation)
- **PWM Resolution**: 10-bit (0-1023)
- **ADC Resolution**: 12-bit (0-4095)
- **Voltage Range**: 0-3.3V
- **Protocol**: CSV over UART with "TEL," prefix
