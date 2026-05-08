# 12-Lead ECG v2: ADS1299 + ICM20948

Second-generation BLE firmware for a portable ECG platform, updating the motion subsystem from ICM20649 to ICM20948 while keeping a high-throughput ADS1299 ECG path.

## Snapshot

| Category | Details |
| --- | --- |
| Signal focus | Multi-channel ECG with synchronized IMU telemetry |
| MCU platform | Nordic nRF52, nRF5 SDK 17.1.0, S132 BLE stack |
| Sensors | ADS1299 ECG AFE, ICM20948 9-axis IMU family |
| Interfaces | SPI, TWI/I2C, custom BLE services |
| Main entry | `main.c` |

## What This Project Shows

- Ported the IMU path to an ICM20948 driver with banked register access.
- Added register readback checks during initialization to catch sensor bring-up failures early.
- Maintained ADS1299 DRDY-driven ECG acquisition and BLE streaming.
- Used timer-driven IMU sampling with BLE packet buffering.
- Kept sample-rate diagnostics for acquisition validation and throughput tuning.

## Project Media

<div align="center">
  <img src="image%20%281%29.png" alt="Version 2 ECG PCB render and layout" width="70%">
  <br>
  <sub><b>PCB design.</b> Version 2 nRF52 + dual ADS1299 + ICM20948 ECG wearable showing updated motion-sensor integration.</sub>
  <br><br>
  <img src="image%20%2814%29.png" alt="ECG v2 assembled PCB comparison" width="70%">
  <br>
  <sub><b>Hardware iteration.</b> Assembled ECG v2 board shown alongside an earlier board for form-factor comparison.</sub>
  <br><br>
  <img src="image%20%283%29.png" alt="ECG and IMU GUI template" width="70%">
  <br>
  <sub><b>Visualization.</b> Real-time Python GUI template for ECG and IMU data streamed over BLE.</sub>
</div>

[Project poster](../Poster_ECG_Wearable.pdf) - Poster summary for the wearable ECG system and design rationale.

## Firmware Architecture

```text
Application boot
  -> Nordic BLE stack and GATT services
  -> ADS1299 SPI configuration
  -> ICM20948 TWI configuration with WHO_AM_I/register validation
  -> app_timer acquisition loops
  -> custom BLE notifications for ECG and IMU payloads
```

The ECG path remains interrupt-driven. The IMU path uses periodic reads and a packet buffer so BLE notifications are amortized across multiple samples.

## Key Modules

| Module | Role |
| --- | --- |
| `src/ads1299-x.c` | ECG front-end configuration and SPI acquisition |
| `src/icm20948.c` | ICM20948 driver, bank selection, register read/write, sample reads |
| `src/ble_eeg.c` | ECG BLE transport |
| `src/ble_icm.c` | IMU BLE transport |
| `pca10040/s132/config/icm20948_config.h` | Motion sensor sampling and register profile |
| `pca10040/s132/config/usr_config.h` | Project-level device and feature configuration |

## Engineering Depth

The ICM20948 driver demonstrates hardware-level care: bank-aware register addressing, bounded multi-byte transfers, device discovery, WHO_AM_I checks, configuration writeback validation, endian-aware sample conversion, and clear error propagation through Nordic `ret_code_t` returns.

## Power And BLE Optimization

- Acquisition work is gated by BLE connection state where appropriate.
- IMU samples are accumulated before notification to reduce radio overhead.
- Timed diagnostics expose actual ECG sample rate and DRDY interval stability.
- Battery measurement support is isolated from high-rate sensing paths.
