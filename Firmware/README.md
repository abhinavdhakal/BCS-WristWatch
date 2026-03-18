# Multi-Sensor Health Monitor

### Circuit Design Tools

EasyEDA.

### Number of Layers

2-layer PCB.

### Sensor Modules and Connections

- **MAX30102**: For heart rate and SpO₂ measurement, connected via I²C.
- **MLX90614**: For temperature sensing, connected via I²C.
- **IMU (ST LSM6DS3TR‑C)**: For motion (accelerometer and gyroscope) data, connected via I²C.

### Power Source

- **Battery Type**: 3.7V Li-ion battery.
- **Capacity**: 200mAh.

### Voltage Regulation

No voltage regulator was needed as everything was working in 3.3V and the charging circuit was already in the microcontroller itself.

### Board Size

1.19 x 1.50 inches (30.1 x 38.1 mm).

### Component Placement Logic

- **Sensors**: MAX30102 is in the backside of the PCB facing down, MLX90614 is IR based, so it is in the front side and facing down, and a cutout is made for the IR to see the skin.
- **IMU**: Integrated inside the microcontroller.

### Enclosure Design

Currently in progress, It would expose the sensors (MAX30102, MLX90614) and protect the PCB while having a strap that will put the device in constant pressure to the skin.

### Programming Environment

- **Environment**: Arduino IDE.
- **Language/Libraries**:
  - C++.
  - Libraries:
    - SparkFun MAX30105 (MAX30102) for heart rate and SpO₂ (with heartRate.h).
    - Adafruit MLX90614 for temperature.
    - SparkFun LSM6DS3 for IMU data.
    - ArduinoBLE for Bluetooth Low Energy.

### Sampling and Notification Rates

- IR & Red (MAX30102): 50 Hz (every 20 ms).
- Temperature (MLX90614): 5 Hz (every 200 ms), value sent as °C × 100.
- IMU (LSM6DS3): 5 Hz (every 200 ms), 6 floats (ax, ay, az, gx, gy, gz).
- Battery Voltage: 5 Hz (every 200 ms), value sent as millivolts.

### Data Packet Structure

**Structure:**

- IR LED: 4 bytes uint32 (raw IR count).
- Red LED: 4 bytes uint32 (raw Red count).
- Temperature: 4 bytes uint32 (object temperature °C × 100).
- IMU: 24 bytes (6 × float32: ax, ay, az, gx, gy, gz).
- Battery: 4 bytes uint32 (voltage mV).

### Transmission Protocol

- Protocol: Bluetooth Low Energy (BLE).
- Device Name: `BCS-Watch`.
- Notifications: Enabled for all characteristics listed above.
- MTU Consideration: IMU characteristic requires MTU ≥ 27 bytes for single-frame delivery (24-byte payload + overhead).

### Synchronization Logic

Sensors are read sequentially (temperature → MAX30102 → IMU) in a single pass per send interval.

### Buffering

No transmission buffering. An internal 50-sample circular buffer is used only for SpO₂ computation (not for BLE queuing).

### Receiver Setup

There is a python script to receive the data from the custom service/characteristics.

Host processing:

- HR & SpO₂ derived every ~0.5 s from rolling IR/Red buffers (`compute_hr_spo2`).
- 120-sample circular buffers (~2.5 s at 50 Hz) drive real-time plots of IR and Red.
- Battery percentage computed from linear mapping of voltage (3.0–4.2 V).

Dependencies: `bleak`, `numpy`, `matplotlib`, `scipy`.

### Error Handling

- **Mechanisms**:
  - Sensor initialization errors are logged ("Warning: Some sensors failed to initialize").
  - No explicit retry mechanism for lost BLE packets or invalid data is mentioned.

### Power Management

- **Implementation**:
  - Power state is monitored via `POWER_SENSE_PIN`.
  - Sleep modes are planned but not implemented (TODO comments for deep sleep and wake-up logic).
  - Sensors are powered down when the system is off (`sensors.shutdown()`).
