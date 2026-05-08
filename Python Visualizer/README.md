# BLE Wearable Visualizer

Python desktop visualizer for the BLE streams produced by the wearable firmware projects in this repository. It scans for BLE peripherals, subscribes to supported notify characteristics, plots live physiological and motion data, shows BLE link diagnostics, and can log current sessions to CSV.

## Features

- BLE device scan and connect workflow using `bleak`.
- Dynamic characteristic discovery for supported UUID families.
- Real-time pyqtgraph plots for ADS1299 ECG/EMG channels.
- Optional ADS1299 high-pass, low-pass, notch, smoothing, and spike filtering controls.
- Dedicated IMU windows for accelerometer, gyroscope, and temperature streams.
- Dedicated PPG window for MAX30102 red/IR data with BPM and SpO2 estimates.
- Dedicated AS7341 spectral status windows with red/NIR ratio readouts.
- AS7341 LED intensity/PWM controls when the firmware exposes the config UUID.
- BLE RSSI, throughput, packet-rate, and connection-state diagnostics.
- CSV logging for EEG/ECG/EMG, PPG, and IMU streams while plotting is enabled.
- Last-device persistence and automatic reconnect support.

## Files

| File | Purpose |
| --- | --- |
| `wearable_visualizer.py` | Main Qt application and BLE stream decoder. |
| `bluetooth_dialog.py` | BLE scan, filter, selection, and connect dialog. |
| `ppg_window.py` | Standalone MAX30102 PPG plotting window. |
| `protocol_display.py` | Protocol/countdown panel widget retained for gesture workflows. |
| `run_visualizer.sh` | Convenience launcher for the main app. |
| `BLE_UUID_Visualizer.spec` | PyInstaller build spec for a standalone app bundle/executable. |
| `data/` | Runtime state, handshake logs, and generated CSV outputs. |

## Supported BLE UUIDs

The visualizer accepts standard 16-bit UUIDs expanded into the Bluetooth base UUID and vendor-specific 128-bit UUIDs whose first field ends with the same 16-bit short value.

| UUID short value | Data type |
| --- | --- |
| `EEF1`-`EEF8` / `EFF1`-`EFF8` | ADS1299 biopotential channels, decoded as signed 24-bit samples and plotted in microvolts. |
| `EEFF` / `EFFF` | ADS1299 or stream configuration characteristic. |
| `1CF1`-`1CF4` | IMU stream characteristics. |
| `1CFF` | IMU configuration characteristic. |
| `AEC1` | MAX30102 red PPG channel. |
| `AEC2` | MAX30102 IR PPG channel. |
| `1525` | AS7341 configuration characteristic for LED intensity/PWM. |
| `1526` / `1527` | AS7341 spectral data streams. |
| `BFEE` | BLE link parameters such as RSSI and configured Tx power. |

## Setup

Use Python 3.10 or newer. On macOS, run the app from a terminal that has Bluetooth permission.

```bash
cd "Python Visualizer"
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install --upgrade pip
python3 -m pip install PyQt5 pyqtgraph bleak qasync numpy
```

## Run

```bash
cd "Python Visualizer"
source .venv/bin/activate
python3 wearable_visualizer.py
```

Or use the launcher:

```bash
cd "Python Visualizer"
chmod +x run_visualizer.sh
./run_visualizer.sh
```

## Basic Workflow

1. Start the visualizer.
2. Select `Connect Bluetooth` or `Connect > Bluetooth`.
3. Scan for the target wearable and connect.
4. Confirm the UUID list in the main window.
5. Select `Start Plotting` to begin live plotting and CSV logging.
6. Use the sensor selector to open dedicated IMU, PPG, or AS7341 windows when those streams are available.
7. Select `Stop Plotting` or `Disconnect` to end the session cleanly.

## CSV Outputs

When plotting is enabled, the app writes latest-session CSV files under `data/`:

| File | Contents |
| --- | --- |
| `data/ble_uuid_data_latest.csv` | ADS1299 ECG/EMG channel values by UUID column. |
| `data/ppg_data_latest.csv` | PPG time, red, and IR rows. |
| `data/imu_data_latest.csv` | IMU time, UUID, accel, gyro, and temperature rows. |

These files are overwritten on each new plotting session.

## Build Standalone App

Install PyInstaller inside the virtual environment, then build from this directory:

```bash
source .venv/bin/activate
python3 -m pip install pyinstaller
pyinstaller BLE_UUID_Visualizer.spec
```

The spec includes the `data/` directory so saved device state and runtime output paths are available to the packaged app.

## Troubleshooting

- If no devices appear, confirm the wearable is advertising and the terminal/app has Bluetooth permission.
- If connect fails immediately after scanning, stop the scan and retry; active scans can interfere with BLE connection attempts.
- If no notify UUIDs are discovered, confirm the firmware exposes notify-enabled characteristics for the supported UUIDs above.
- If plots are flat or saturated, disable filtering, reset Y zoom, or enable `Auto Y`.
- If AS7341 LED controls are unavailable, confirm the firmware exposes UUID `1525`.
