# EMG + ICM20649 Robotic Hand Control

BLE firmware for a wearable EMG and motion-sensing prototype intended to support robotic-hand control and gesture-driven applications.

## Snapshot

| Category | Details |
| --- | --- |
| Signal focus | EMG-style biopotential acquisition with inertial motion context |
| MCU platform | Nordic nRF52, nRF5 SDK 17.1.0 |
| Sensors | ADS1299, ICM20649 |
| Interfaces | SPI, TWI/I2C, BLE notifications |
| Main entry | `main.c` |

## What This Project Shows

- Adapted ADS1299 biopotential acquisition for EMG/control-oriented streaming.
- Integrated ICM20649 motion data alongside muscle activity data.
- Designed custom BLE services for high-rate biomedical and motion payloads.
- Used packet buffering to make BLE notifications practical for real-time control data.
- Kept diagnostics to measure effective sample rate and timing stability.

## Project Media

| Asset | Caption |
| --- | --- |
| <img src="../image%20%285%29.png" alt="EMG ICM20649 PCB design" width="420"> | PCB design for the EMG + ICM20649 robotic-hand control board, showing front-side placement, reverse-side routing, and routed layout. |

## Control Pipeline Role

This firmware is the wearable data-acquisition side of a robotic-hand control system. It focuses on collecting EMG and IMU streams reliably enough for downstream gesture classification, visualization, or actuation logic.

```text
EMG electrodes -> ADS1299 -> nRF52 packetizer -> BLE central / GUI / controller
Motion sensor -> ICM20649 -> nRF52 packetizer -> BLE central / GUI / controller
```

## Key Modules

| Module | Role |
| --- | --- |
| `src/ads1299-x.c` | ADS1299 driver and EMG/biopotential sampling path |
| `src/icm20649.c` | ICM20649 motion driver |
| `src/ble_eeg.c` | Biopotential BLE service reused for EMG-style samples |
| `src/ble_icm.c` | IMU BLE service |
| `src/ble_usr_defined.c` | Custom BLE service plumbing |
| `src/timing_utils.c` | Timing support and diagnostics |

## Engineering Depth

The project demonstrates wearable firmware design for a closed-loop control workload: low-latency data movement, sensor timing, BLE throughput management, and robust packet boundaries. It also shows reuse of a proven ADS1299 data path while changing the application target from clinical ECG to EMG-driven control.

## Power And Latency Considerations

- BLE packets are batched to reduce radio wakeups while preserving useful update cadence.
- The IMU path is timer-driven and connection-aware.
- High-rate signal reads are separated from lower-priority telemetry such as battery reporting.
- The architecture leaves room for tuning sample rate, BLE connection interval, and payload size for either lower latency or longer battery life.
