# WirstWatch_BCS

WirstWatch_BCS is a wearable vital-signs prototype focused on heart rate, SpO2, temperature, IMU data, and battery telemetry over BLE.

This project is still in prototype stage. The goal right now is to make the full pipeline stable end-to-end:
firmware -> BLE -> desktop receiver -> session logging -> signal processing.

## Project Structure (high level)

- `Firmware/`
  - Arduino firmware for nRF52840-based hardware
  - BLE service/characteristics, sensor sampling, battery telemetry, power modes
- `ReceiverSoftware/`
  - PyQt GUI receiver (`wirstwatch_gui.py`)
  - HR/SpO2 processing (`hr_spo2.py`)
  - BLE utility (`ble_connector.py`)
  - session CSV + log files
- `everything else/`
  - older experiments and archive material

## Current Data Pipeline

1. Firmware samples sensors and sends data over BLE.
2. Receiver GUI subscribes to BLE notifications.
3. GUI logs every session to CSV.
4. HR/SpO2 is computed from IR/Red stream in software.
5. Live values and plots are shown in the GUI.

## Firmware Data Output (data path only)

BLE Service UUID:

- `12345678-1234-1234-1234-123456789abc`

Characteristics:

- IR (`...ab1`): uint32 (4 bytes)
- Red (`...ab2`): uint32 (4 bytes)
- Temperature (`...ab3`): uint32 (4 bytes), value is `tempC * 100`
- IMU (`...ab4`): 6 x float32 (24 bytes): `ax, ay, az, gx, gy, gz`
- Battery (`...ab5`): uint32 (4 bytes), millivolts

Nominal send rates from firmware:

- IR + Red: every 20 ms (~50 Hz)
- Temperature + IMU + Battery: every 200 ms (~5 Hz)

## Receiver Software Setup

Main software folder: `ReceiverSoftware/`

### Windows (recommended for users)

- One click:
  - double-click `ReceiverSoftware/run_wirstwatch_gui.bat`

- Manual:
  - `py -3.11 -m venv .venv`
  - `.venv\Scripts\activate`
  - `pip install -r ReceiverSoftware/requirements_gui.txt`
  - `python ReceiverSoftware/wirstwatch_gui.py`

### macOS/Linux (dev)

- `python3 -m venv .venv`
- `source .venv/bin/activate`
- `pip install -r ReceiverSoftware/requirements_gui.txt`
- `python ReceiverSoftware/wirstwatch_gui.py`

## Hardware Design

### Current Design

- nRF52840-based main board with BLE
- MAX30102 (PPG) for HR/SpO2 signal source
- MLX90614 for temperature
- 6-axis IMU for motion context
- Battery measurement through onboard divider/ADC
- Wrist-worn form factor prototype

### Limitations

- This prototype uses readily available sensors, not medical-grade modules
- Wrist PPG quality is more variable than finger-based setups
- Motion artifacts are still a major challenge for stable HR/SpO2
- Current HR/SpO2 output is best-effort and for prototyping, not clinical use
- Mechanical/contact consistency is limited by prototype enclosure/fit

### Future Improvements

- Move to higher-grade optical sensor for more stable wrist PPG
- Add improved optical/mechanical design for skin contact consistency
- Upgrade HR/SpO2 algorithm (stronger motion artifact handling + confidence model)
- Add algorithm versioning/benchmarking with reference datasets
- Add calibration and validation workflow against reference device

## Notes

- Logs are written by the GUI to `ReceiverSoftware/logs/`
- Session CSV files are written to `ReceiverSoftware/sessions/`
- For software-only details, see `ReceiverSoftware/README.md`
# BCS-WristWatch
