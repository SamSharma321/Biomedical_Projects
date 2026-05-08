# Biomedical Wearable Systems Portfolio

This repository contains research firmware, drivers, BLE applications, PCB-oriented project assets, and visualization demos for biomedical wearable prototypes developed around Nordic nRF52 devices.

The work spans ECG, EMG, IMU, fNIRS, optical pulse sensing, sleep monitoring, dysphagia sensing, and robotic-hand control. The projects are organized as embedded firmware snapshots for real lab prototypes, with each project folder containing its own README, source modules, configuration files, build assets, and media where available.

## What This Repository Contains

| Category | Contents |
| --- | --- |
| Embedded firmware | nRF5 SDK BLE peripheral applications written in C |
| Sensor drivers | Register-level drivers for biomedical AFEs, IMUs, optical sensors, and battery measurement paths |
| BLE applications | Custom GATT services, notification packetizers, runtime configuration characteristics, battery/device services |
| Hardware integration | Board pin maps, SPI/TWI configuration, PCB design images, assembled board photos, deployable hex artifacts |
| Visualization | GUI screenshots and demo videos for live ECG, IMU, fNIRS, BLE UUID, and PPG data views |
| Optimization work | BLE packet batching, connection-aware streaming, sample queues, timer scheduling, FIFO draining, and low-power measurement paths |

## Supported Driver Part Numbers

| Part number | Device class | Used for |
| --- | --- | --- |
| `ADS1299` | 8-channel biopotential analog front end | ECG and EMG acquisition, multi-channel biomedical signal streaming |
| `ADS1291` / `ADS1292` | Low-power biopotential analog front end | Dysphagia and compact biopotential sensing |
| `ICM20649` | 6-axis IMU | Motion context for ECG, EMG, and sleep wearable projects |
| `ICM20948` | 9-axis IMU family | Updated motion sensing, dual-IMU dysphagia capture, register-verified bring-up |
| `ICM40609` | 6-axis IMU | Newer robotic-hand control motion subsystem |
| `AS7341` | Multi-spectral optical sensor | fNIRS red/NIR optical sensing |
| `MAX30102` | Optical pulse oximetry / PPG sensor | Sleep prediction and PPG-style optical streaming |
| `nRF52` / `nRF52832` | BLE microcontroller platform | BLE peripheral firmware, custom services, timers, GPIO, SPI, TWI, SAADC |

## Firmware And BLE Capabilities

- Interrupt-driven ADS acquisition using DRDY lines for deterministic sampling.
- SPI and TWI/I2C sensor bring-up with register configuration and validation.
- Custom BLE services for ECG/EMG, IMU, fNIRS, and PPG/SpO2-oriented payloads.
- Packed BLE notifications to reduce radio overhead and support higher-rate streams.
- Connection-aware acquisition paths that avoid unnecessary work when no BLE central is connected.
- Timer-driven sensor scheduling for IMU and optical data paths.
- FIFO and RAM queue handling for bursty sensor streams and BLE backpressure.
- Battery measurement support using SAADC and BLE Battery Service plumbing.
- Device Information Service support and deployable hex/build artifacts for embedded testing.

## Project Index

| Project | Main hardware / drivers | What it demonstrates |
| --- | --- | --- |
| [12-Lead 10-Electrode ECG v1](nRF_Driver_Projects/12_Lead_10_ELectrode_ECG_v1/README.md) | `ADS1299`, `ICM20649`, `nRF52` | Research-grade ECG acquisition with synchronized IMU telemetry and BLE streaming |
| [12-Lead ECG v2](nRF_Driver_Projects/12_Lead_ECG_v2/ADS1299_ICM20948/README.md) | `ADS1299`, `ICM20948`, `nRF52` | Updated ECG + IMU firmware with banked IMU register handling and readback validation |
| [Dysphagia Wearable Sensor Firmware](nRF_Driver_Projects/Dysphagia/README.md) | `ADS1291/2`, dual `ICM20948`, `nRF52` | Swallowing research prototype with biopotential sensing, dual IMU streams, RAM buffering, and BLE visualization |
| [EMG + ICM20649 Robotic Hand Control](nRF_Driver_Projects/EMG_ICM20649_Robotic_Hand_Control/ADS1299_ICM20649_Prj/README.md) | `ADS1299`, `ICM20649`, `nRF52` | EMG and motion streaming firmware for gesture/control pipelines |
| [EMG + ICM40609 Robotic Hand Control](nRF_Driver_Projects/EMG_ICM40609_Robotic_Hand_Control/HODAM_EMG_ICM40609_Prj/README.md) | `ADS1299`, `ICM40609`, `nRF52` | Sensor migration from ICM20649-class motion capture to ICM40609-based acquisition |
| [fNIRS Sleep Project](nRF_Driver_Projects/NIRS_Sleep_Project/fNIRS/README.md) | `AS7341`, LED control, optional `ADS1299`, `nRF52` | Red/NIR optical sensing, runtime BLE configuration, LED control, and queued spectral telemetry |
| [fNIRS Sleep Project v2](nRF_Driver_Projects/NIRS_Sleep_Project_v2/fNIRS/README.md) | `AS7341`, LED control, `nRF52` | Cleaner fNIRS-only firmware with round-robin sensor scheduling and BLE queueing |
| [Sleep Prediction Multimodal Wearable](nRF_Driver_Projects/Sleep_Prediction_Prj/Sleep_Prediction_Prj/README.md) | `ADS1299`, `ICM20649`, `MAX30102`, `nRF52` | Multimodal sleep firmware combining biopotential, motion, and optical streams |

## Repository Structure

```text
Biomedical_Projects/
  README.md
  nRF_Driver_Projects/
    12_Lead_10_ELectrode_ECG_v1/
    12_Lead_ECG_v2/
    Dysphagia/
    EMG_ICM20649_Robotic_Hand_Control/
    EMG_ICM40609_Robotic_Hand_Control/
    NIRS_Sleep_Project/
    NIRS_Sleep_Project_v2/
    Sleep_Prediction_Prj/
```

## Platform And Tooling

| Area | Details |
| --- | --- |
| MCU platform | Nordic nRF52 family, including nRF52832-oriented BLE designs |
| SDK / stack | nRF5 SDK 17.1.0 with S132 SoftDevice-style BLE peripheral applications |
| Language | Embedded C |
| Sensor buses | SPI for ADS biopotential AFEs; TWI/I2C for IMU and optical sensors |
| Build assets | SEGGER Embedded Studio, GCC, IAR, and Keil project files are retained where available |
| Output artifacts | Source code, configuration headers, project files, hex builds, PCB images, board photos, and GUI/demo media |

## Engineering Areas Demonstrated

- Biomedical wearable firmware architecture.
- Custom low-level sensor driver development.
- BLE throughput optimization for continuous physiological data.
- Power-aware sampling and packet scheduling.
- Multi-sensor synchronization and buffering.
- Hardware/software bring-up for custom PCBs.
- Live visualization pipelines for lab validation.
- Iterative prototype development across multiple sensor revisions.
