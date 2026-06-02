# Vacuum Gauge

Firmware for a digital vacuum pressure gauge built around a **Posifa** I2C
pressure sensor and a **LILYGO T-QT Pro (ESP32-S3)** with a 128 × 128 GC9A01
TFT display. The gauge reads the sensor, auto-ranges the pressure between Torr
and milli-Torr, shows it on the TFT, and exposes a serial command interface for
configuration and data readout.

## Features

- Periodic sensor reads (every 500 ms) with conversion of raw counts to Torr.
- Auto-ranging display: whole Torr, tenths of a Torr, or mTorr depending on the
  reading.
- User-trimmable zero offsets — a coarse **Torr** offset and a fine **mTorr**
  offset — adjustable from the two on-board buttons or over serial.
- Settings persisted to SPIFFS and reloaded on boot; changes are auto-saved
  every 10 s (only when something actually changed).
- Serial command processor for version, naming, live/raw readings, offset
  configuration, and save/load.

## Hardware

| Component        | Detail                                            |
| ---------------- | ------------------------------------------------- |
| MCU board        | LILYGO T-QT Pro (ESP32-S3)                         |
| Display          | 128 × 128 GC9A01 TFT (via TFT_eSPI)               |
| Sensor           | Posifa pressure sensor, I2C address `0x50`        |
| I2C pins         | SDA = GPIO 43, SCL = GPIO 44 @ 100 kHz            |
| Buttons          | Button A = GPIO 0, Button B = GPIO 47             |

### Button behavior

Each button press nudges the active zero offset. Which trim is adjusted follows
the current reading range (coarse Torr above ~10 Torr, fine mTorr below):

- **Button A** — increase offset (+1 Torr or +10 mTorr)
- **Button B** — decrease offset (−1 Torr or −10 mTorr)

## Serial commands

| Command     | Action                                |
| ----------- | ------------------------------------- |
| `GVER`      | Return the firmware version string    |
| `?NAME`     | Get/set the device name               |
| `GPRES`     | Return calibrated pressure (Torr)     |
| `GRPRES`    | Return raw pressure sensor counts     |
| `GRTEMP`    | Return raw temperature sensor counts  |
| `?TOFFSET`  | Get/set the coarse Torr offset        |
| `?MTOFFSET` | Get/set the fine milli-Torr offset    |
| `SAVE`      | Save parameters to flash              |
| `LOAD`      | Load the saved parameters from flash  |

A `?`-prefixed command with no argument reads the value; with an argument it
sets it.

## Building

This is a [PlatformIO](https://platformio.org/) project targeting the
`espressif32` platform with the Arduino framework.

```bash
# Build
pio run

# Build, upload, and open the serial monitor
pio run -t upload -t monitor
```

The required libraries are vendored under [`lib/`](lib/) so the build is
self-contained:

- **GAACE** — command processor, debug, button, ring buffer, char allocator
- **arduino-timer** — lightweight periodic task scheduler
- **TFT_eSPI** — display driver, pre-configured for the LilyGo T-QT Pro S3
  (`Setup211_LilyGo_T_QT_Pro_S3.h`)

Because these are vendored snapshots, upstream library updates won't flow in
automatically — update the copies in `lib/` if you need newer versions.

## Project layout

```
.
├── platformio.ini        PlatformIO environment (board: lilygo-t3-s3)
├── src/VacuumGauge.cpp    Application firmware
├── include/VacuumGauge.h  Function prototypes
└── lib/                   Vendored libraries (GAACE, arduino-timer, TFT_eSPI)
```

## License

Copyright © GAACE.
