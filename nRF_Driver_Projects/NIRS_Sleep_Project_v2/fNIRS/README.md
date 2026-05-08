# fNIRS Sleep Project v2

Cleaner second-generation fNIRS firmware focused on AS7341 red/NIR optical sensing, LED control, and BLE-efficient sample queueing.

## Snapshot

| Category | Details |
| --- | --- |
| Signal focus | fNIRS-style red/NIR optical telemetry for sleep research |
| MCU platform | Nordic nRF52, nRF5 SDK 17.1.0 |
| Sensors | AS7341 spectral sensor |
| Actuation | LED intensity and location interval control |
| Main entry | `main.c` |

## What Changed From v1

- Removed the active ADS1299 path from the main application flow to focus the firmware on fNIRS acquisition.
- Kept AS7341 queueing and round-robin sensor scheduling.
- Preserved runtime BLE configuration for integration timing and LED behavior.
- Simplified sample-rate diagnostics around optical devices.

## Project Media

| Asset | Caption |
| --- | --- |
| <img src="image%20%2810%29.png" alt="fNIRS v2 PCB design" width="420"> | fNIRS v2 PCB design overview showing board dimensions, LED/sensor placement, routed layouts, and expected PCB thickness. |

## Firmware Architecture

```text
AS7341 readout: red 630 nm, red 680 nm, NIR
LED control: intensity and location interval
BLE control path: configuration characteristic
BLE data path: queued optical samples in packed notifications
```

## Key Modules

| Module | Role |
| --- | --- |
| `src/as7341.c` | Spectral sensor driver |
| `src/ble_as7341.c` | Optical BLE service and configuration characteristic |
| `src/led.c` | LED intensity/location control |
| `src/nrf_usr_defined.c` | Nordic platform support |
| `pca10040/s132/config/as7341.h` | Sensor definitions and project configuration |
| `pca10040/s132/config/ble_as7341.h` | BLE payload and service definitions |

## Engineering Depth

This version shows the kind of refinement expected during prototype iteration: narrower scope, less competing sensor work, and more predictable timing for the target modality. The code emphasizes a controlled optical acquisition loop, BLE-side configurability, and payload queueing for stable streaming.

## Power And Signal Considerations

- LED intensity and location interval can be adjusted from the BLE central.
- Sensor sampling is timer-driven and can be aligned to integration time.
- Packed notifications reduce radio overhead.
- Round-robin processing avoids excessive BLE bursts when multiple optical devices are present.
