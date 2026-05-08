# fNIRS Sleep Project

BLE firmware for a sleep-oriented fNIRS wearable prototype using red/NIR optical sensing, programmable LED control, and optional ADS1299 acquisition support.

## Snapshot

| Category | Details |
| --- | --- |
| Signal focus | Red/NIR optical measurements for fNIRS-style sleep research |
| MCU platform | Nordic nRF52, nRF5 SDK 17.1.0 |
| Sensors | AS7341 spectral sensor, ADS1299 support retained |
| Actuation | LED intensity and location/interval control |
| Main entry | `main.c` |

## What This Project Shows

- Integrated AS7341 spectral sensing with a custom BLE streaming service.
- Added firmware-controlled LED intensity and location timing.
- Implemented per-device sample queues for optical data.
- Used BLE configuration writes to update optical integration and LED behavior at runtime.
- Preserved ADS1299 support for multimodal physiological experiments.

## Project Media

<div align="center">
  <img src="../image%20%2811%29.png" alt="fNIRS system block diagram" width="70%">
  <br>
  <sub><b>System block diagram.</b> AS7341 optical sensors, nRF52832 BLE controller, ADS1299 support, LED drive, and BLE communication.</sub>
  <br><br>
  <img src="../image%20%2819%29.png" alt="fNIRS firmware and sensor flow" width="70%">
  <br>
  <sub><b>Firmware flow.</b> fNIRS sensing, BLE configuration, and sensor data movement through the application.</sub>
  <br><br>
  <img src="../image%20%2817%29.png" alt="fNIRS PCB top render" width="70%">
  <br>
  <sub><b>PCB top render.</b> Top-side board view for the fNIRS wearable.</sub>
  <br><br>
  <img src="../image%20%2818%29.png" alt="fNIRS PCB routed layout" width="70%">
  <br>
  <sub><b>PCB routed layout.</b> Routed fNIRS board layout including sensor and antenna placement constraints.</sub>
  <br><br>
  <img src="../image%20%282%29.png" alt="fNIRS GUI live view" width="70%">
  <br>
  <sub><b>GUI live view.</b> Live fNIRS streaming interface with optical channel plots and timing/status traces.</sub>
  <br><br>
  <img src="../image%20%2820%29.png" alt="fNIRS waveform plots" width="70%">
  <br>
  <sub><b>Optical waveforms.</b> Red/NIR sensor data captured from streamed fNIRS measurements.</sub>
  <br><br>
  <img src="../image%20%2821%29.png" alt="Assembled fNIRS boards" width="70%">
  <br>
  <sub><b>Assembled prototypes.</b> fNIRS PCB hardware used for board bring-up and firmware validation.</sub>
</div>

- [BLE UUID visualizer demo](../BLE%20UUID%20Visualizer%20%28Dynamic%20per%20UUID%29%202026-05-04%2015-26-50.mp4) - BLE GUI demo showing dynamic UUID-based visualization for fNIRS data streams.
- [ICM live graph demo](../ICM%20Device%201%20Live%20Graph%20%281CF1%29%20_%20Freq_%20177.3%20Hz%202026-04-23%2016-45-22.mp4) - IMU live graph demo used to validate live streaming and GUI plotting behavior.

## Firmware Architecture

```text
BLE central config write
  -> update AS7341 integration settings
  -> update LED intensity/location interval
  -> restart optical sampling timer

AS7341 timer
  -> select initialized device round-robin
  -> read red 630 / red 680 / NIR channels
  -> queue samples
  -> flush packed BLE notifications
```

## Key Modules

| Module | Role |
| --- | --- |
| `src/as7341.c` | AS7341 driver and red/NIR readout |
| `src/ble_as7341.c` | Custom BLE service for optical samples and configuration |
| `src/led.c` | LED PWM/intensity and location scheduling |
| `src/ads1299-x.c` | Retained ADS1299 acquisition support |
| `src/ble_eeg.c` | Biopotential BLE service |
| `pca10040/s132/config/as7341.h` | Optical sensor configuration |

## Engineering Depth

This project shows firmware for an optical biomedical wearable rather than a purely electrical signal chain. The implementation manages sensor integration timing, LED drive behavior, BLE runtime configuration, queueing, and multi-sensor scheduling while staying within the power and throughput constraints of a small BLE device.

## Power And BLE Optimization

- Processes one optical device per timer tick to reduce burst pressure on BLE notifications.
- Uses queued optical samples before flushing to BLE.
- Allows runtime adjustment of integration time and LED behavior, enabling tradeoffs between signal strength, duty cycle, and battery life.
- Keeps radio work connection-aware.
