# Dysphagia Wearable Sensor Firmware

BLE firmware for a dysphagia research prototype combining ADS1291/2 biopotential sensing with dual ICM20948 motion capture.

## Snapshot

| Category | Details |
| --- | --- |
| Signal focus | Swallowing-related sensing for dysphagia research |
| MCU platform | Nordic nRF52, nRF5 SDK 17.1.0 |
| Sensors | ADS1291/2, two ICM20948 devices |
| BLE name | `IMU_MIC_Sensor` |
| Main entry | `main.c` |

## What This Project Shows

- Developed a custom application for dysphagia sensing with biopotential and motion data streams.
- Integrated ADS1291/2 acquisition for a compact analog front-end path.
- Supported two ICM20948 devices with different service intervals and FIFO-oriented burst handling.
- Added RAM buffering for IMU samples to absorb BLE scheduling delays.
- Included battery measurement, BLE services, and board-specific pin configuration.

## Project Media

<div align="center">
  <img src="image%20%2813%29.png" alt="Dysphagia wearable prototype board" width="42%">
  <br>
  <sub><b>Prototype hardware.</b> Dysphagia wearable PCB with connected sensing leads for firmware and signal-acquisition testing.</sub>
</div>

- [BLE UUID visualizer demo](BLE%20UUID%20Visualizer%20%28Dynamic%20per%20UUID%29%202026-04-23%2016-44-32.mp4) - BLE GUI demo showing dynamic UUID-based visualization for streamed project data.
- [ICM live graph demo](ICM%20Device%201%20Live%20Graph%20%281CF1%29%20_%20Freq_%20177.3%20Hz%202026-04-23%2016-45-22.mp4) - IMU live graph demo for ICM device streaming and sample-rate visualization.

## Firmware Architecture

```text
ADS1291/2 DRDY -> SPI read -> BLE biopotential stream
ICM20948 device 1 -> FIFO burst drain -> RAM queue -> BLE IMU stream
ICM20948 device 2 -> lower-rate drain -> RAM queue -> BLE IMU stream
SAADC battery path -> BLE battery service
```

## Key Modules

| Module | Role |
| --- | --- |
| `src/ads1291-2.c` | ADS1291/2 driver and acquisition path |
| `src/icm20948.c` | ICM20948 driver and motion sample handling |
| `src/ble_eeg.c` | Biopotential BLE service |
| `src/ble_icm.c` | IMU BLE service and buffering support |
| `src/ble_usr_defined.c` | Custom BLE application service support |
| `pca10040/s132/config/usr_defined.h` | Device name, sensor enable flags, and pin mapping |

## Engineering Depth

This project demonstrates embedded work beyond a single-sensor demo. It includes multi-device IMU scheduling, burst sample handling, RAM queueing with dropped-sample accounting, connection-aware transmission, and a compact ADS1291/2 signal path suitable for wearable research hardware.

## Power And Reliability Considerations

- IMU devices are serviced at different timer intervals to match expected sample behavior.
- FIFO bursts reduce transaction overhead compared with single-sample reads.
- RAM queueing helps tolerate temporary BLE backpressure.
- Battery measurement is periodic and separated from high-rate sensing.
- Board-level configuration keeps pin mapping explicit for hardware bring-up.
