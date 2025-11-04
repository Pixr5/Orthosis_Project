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

## System Architecture & Advanced Features

### Multi-Rate Control System
The firmware implements a sophisticated multi-rate control architecture:
- **Motor Control**: 1000 Hz PI controller with conditional integration anti-windup
- **ADC Sampling**: 500 Hz with continuous DMA mode (100 Hz background sampling)  
- **Telemetry**: 100 Hz data transmission via UART
- **BLE Updates**: 50 Hz wireless telemetry notifications

### PI Controller Configuration
- **Optimized for motor control**: No derivative term to reduce noise sensitivity
- **Adaptive sample time**: Automatically adjusts for timing variations
- **Anti-windup protection**: Conditional integration prevents control instability
- **Safety limits**: Hard-clamped output range (0-1023 PWM)

### Hardware Safety Systems
- **Solenoid safety margin**: 20-count threshold prevents noise activation
- **Input validation**: All UART commands range-checked and clamped
- **Thread safety**: Mutex-protected shared data with non-blocking access
- **Timing protection**: Adaptive dt clamped to 0.1ms-100ms range

## Development & Configuration

### UART Command Interface
The system supports runtime configuration via UART commands:

```
# Setpoint Control
SET,2048          # Set raw ADC setpoint (0-4095)
SETP,50           # Set percentage setpoint (0-100%)
MODE,MAN          # Manual setpoint mode
MODE,ADC          # ADC passthrough mode

# PI Controller Tuning  
PI,1.2,0.05       # Set Kp and Ki gains
PIGET             # Get current PI parameters

# Multi-Rate Frequency Control
FREQ,MOTOR,1000   # Set motor control frequency (10-10000 Hz)
FREQ,ADC,500      # Set ADC sampling frequency  
FREQ,TEL,100      # Set telemetry frequency
FREQ,BLE,50       # Set BLE update frequency
FREQGET           # Get all current frequencies
```

### Hardware Configuration
```c
// Pin assignments (main/The_Holy_Code.c)
#define MOTOR_PIN       6    // PWM output to motor driver
#define ADC_INPUT_PIN   4    // Analog feedback input (0-3.1V)
#define SOLENOID_PIN    18   // Digital solenoid control

// Control parameters
#define LEDC_FREQUENCY  200  // PWM frequency (Hz)
#define SECURITY_MARGIN 20   // Solenoid activation threshold
```

### BLE Telemetry
- **Service UUID**: 0x00FF
- **Characteristic UUID**: 0xFF01  
- **Device Name**: "ESP32_Motor"
- **Data Format**: Same as UART telemetry
- **Connection**: Automatic advertising, supports notifications

## Future Development Guidelines

### Performance Optimizations
The system includes several performance optimizations:
1. **Hardware read elimination**: Uses calculated PWM values instead of reading back
2. **Continuous ADC with DMA**: Background sampling reduces CPU overhead
3. **Optimized task stack sizes**: ~1.3KB RAM savings on ESP32-C3
4. **Pre-calculated timing**: Avoid runtime division in control loops



### Code Architecture
```
main/The_Holy_Code.c          # Main firmware with multi-rate control
├── PI controller functions   # Conditional integration anti-windup
├── Multi-rate task system   # 1000/500/100/50 Hz architecture  
├── UART command parser      # Runtime configuration interface
├── BLE GATT server         # Wireless telemetry service
└── Safety & validation     # Input clamping, mutex protection

tools/control_gui.py         # Interactive control interface
tools/plot_serial.py         # Basic telemetry visualization
```

### Testing & Validation
- **Step response testing**: Verify control stability and settling time
- **Frequency response**: Check system bandwidth and phase margins  
- **Safety validation**: Test all input ranges and fault conditions
- **Timing analysis**: Verify real-time performance under load
- **EMI testing**: Validate operation with electrical noise present

### Hardware Considerations
- **5Hz analog filter**: System designed for pre-filtered ADC input
- **Motor driver compatibility**: 200Hz PWM, 0-3.3V logic levels
- **Power supply**: Stable 3.3V required for ADC accuracy
- **Grounding**: Proper analog/digital ground separation recommended

