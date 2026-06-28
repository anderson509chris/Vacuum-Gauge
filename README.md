# Vacuum Gauge

Firmware for a digital vacuum pressure gauge built around a **Posifa** I2C
pressure sensor and a **Waveshare ESP32-S3-Touch-AMOLED-1.64** with a
280 × 456 CO5300 AMOLED display (rendered in landscape at 456 × 280). The gauge
reads the sensor every 500 ms, auto-ranges the pressure between Torr and mTorr,
and exposes a REST HTTP API so the **GAA-CE desktop application** can discover
and read any gauge on the local network by name.

## Features

- Periodic sensor reads (500 ms) with auto-ranging display: whole Torr, tenths
  of a Torr, or mTorr.
- **Captive-portal first-boot setup** — on first power-up (or after a reset)
  the gauge becomes a WiFi access point. Connect your phone, pick your network
  from the scanned list, choose the gauge identity (Gauge A–H), and tap
  **Save & Connect**. No typing network names.
- **Fixed gauge identities (Gauge A–H)** chosen at setup so the desktop app
  can discover gauges by known names.
- **HTTP REST API** on port 80 — `GET /pressure` and `GET /status` return JSON
  with CORS headers; no TCP bridge or virtual COM port required.
- **User-trimmable zero offsets** — coarse Torr and fine mTorr — set over the
  USB serial command interface.
- **Two pressure-conversion modes:** factory formula, or a field-editable
  piece-wise-linear (PWL) calibration table.
- Settings (offsets, calibration table, WiFi credentials, gauge name) persisted
  to NVS (flash) via the ESP32 Preferences library; auto-saved when changed.
- Long-press the **BOOT button** (3 s) to clear WiFi credentials and reboot
  into portal mode.

## Hardware

| Component   | Detail                                                         |
| ----------- | -------------------------------------------------------------- |
| MCU board   | Waveshare ESP32-S3-Touch-AMOLED-1.64                           |
| MCU         | ESP32-S3R8, 16 MB Flash (QIO), 8 MB OPI PSRAM                 |
| Display     | 280 × 456 CO5300 AMOLED, QSPI interface (landscape, software-rotated) |
| Sensor      | Posifa pressure sensor, I2C address `0x50`                     |
| I2C pins    | SDA = GPIO 1, SCL = GPIO 2 @ 100 kHz                          |
| BOOT button | GPIO 0 — long-press (3 s) resets WiFi credentials             |

### QSPI display wiring

| Signal | GPIO |
| ------ | ---- |
| CS     | 9    |
| SCK    | 10   |
| D0     | 11   |
| D1     | 12   |
| D2     | 13   |
| D3     | 14   |
| RST    | 21   |

## First-boot setup

1. Power on the gauge. It starts a WiFi access point named **VacuumGauge-Setup**.
2. On your phone or laptop, connect to that network (no password).
3. A setup page opens automatically (captive portal). If it doesn't, browse to
   **192.168.4.1**.
4. **Step 1** — choose the gauge identity (Gauge A through Gauge H).
5. **Step 2** — tap your WiFi network from the scanned list.
6. **Step 3** — enter the WiFi password (leave blank for open networks).
7. Tap **Save & Connect**. The gauge restarts and joins your network.

To reconfigure at any time, long-press the **BOOT button** for 3 seconds. The
gauge clears its saved credentials and restarts in portal mode.

## REST API

Once connected, the gauge exposes an HTTP server on port 80. All endpoints
return JSON and include `Access-Control-Allow-Origin: *`.

### `GET /pressure`

```json
{
  "device":        "Gauge A",
  "pressure":      0.012,
  "unit":          "Torr",
  "display_value": 12.0,
  "display_unit":  "mTorr",
  "raw":           31200,
  "cal_mode":      0,
  "timestamp":     143
}
```

`display_value` / `display_unit` auto-range: mTorr when pressure < 1 Torr,
Torr otherwise.

### `GET /status`

```json
{
  "device":     "Gauge A",
  "ip":         "192.168.1.55",
  "rssi":       -52,
  "cal_mode":   0,
  "cal_points": 12
}
```

### Desktop app discovery

The app scans for devices named **Gauge A** through **Gauge H** on the local
network (mDNS or direct IP poll via `/status`). Because the gauge identity is
chosen from a fixed list at setup time, the app always knows the full set of
possible names.

## USB serial command interface

Connect at **115200 baud** over USB. Commands are ASCII lines; `?`-prefixed
commands are get/set pairs.

### General

| Command       | Action                                                      |
| ------------- | ----------------------------------------------------------- |
| `GVER`        | Firmware version string                                     |
| `?NAME`       | Get / set the device name (read-only after portal setup)    |
| `GPRES`       | Calibrated pressure (Torr)                                  |
| `GRPRES`      | Last raw pressure counts                                    |
| `GRAW`        | Fresh averaged raw reading (8 samples)                      |
| `GRTEMP`      | Raw temperature counts                                      |
| `?TOFFSET`    | Get / set coarse Torr zero offset                           |
| `?MTOFFSET`   | Get / set fine mTorr zero offset                            |
| `SAVE`        | Save all parameters to NVS                                  |
| `LOAD`        | Load saved parameters from NVS                              |

### WiFi (serial backdoor)

| Command              | Action                                              |
| -------------------- | --------------------------------------------------- |
| `SSID` / `SSID,<x>` | Get / set saved SSID                                |
| `PASS` / `PASS,<x>` | Get / set saved password                            |
| `WIFICONN`           | Save credentials and reconnect in STA mode          |
| `WIFI`               | Report state, IP address                            |

### Calibration

| Command           | Action                                                         |
| ----------------- | -------------------------------------------------------------- |
| `?CALMODE`        | `0` = factory formula, `1` = PWL table                         |
| `CALPRES,<torr>`  | Capture a cal point at the current raw reading                 |
| `CALDUMP`         | List the calibration table                                     |
| `CALCLEAR`        | Clear the table                                                |
| `CALDEF`          | Restore the factory-default calibration table                  |

## Calibration procedure (PWL table)

1. `?CALMODE,1` to enable the table.
2. Bring the system to a **known** pressure (atmosphere = 760 Torr is a free
   anchor; other points need a reference gauge).
3. `CALPRES,760` — the firmware averages 8 raw readings and stores the point.
   - If the new raw value is within 100 counts of an existing point, that point
     is replaced; otherwise a new point is added.
   - The table is sorted by raw value and saved to NVS immediately.
4. Repeat at other pressures. Use `CALDUMP` to review, `CALCLEAR` to start
   over, `CALDEF` to restore the factory table.

The table holds up to **32 points**. More points near the steep part of the
sensor curve (near atmosphere) improve accuracy. Readings outside the
calibrated range are extrapolated along the nearest end segment.

## Building

[PlatformIO](https://platformio.org/) project — Arduino framework on the
pioarduino ESP32 platform (required for QSPI support).

```bash
pio run                        # build only
pio run -t upload -t monitor   # build, flash, open serial monitor
```

`patch_libs.py` runs automatically before each build and patches
`GAACE_Core/debug.cpp` for ESP32 Arduino core v3 API compatibility.

### Dependencies (fetched automatically by PlatformIO)

| Library                          | Purpose                          |
| -------------------------------- | -------------------------------- |
| moononournation/GFX Library for Arduino | CO5300 QSPI display driver  |
| me-no-dev/AsyncTCP               | Async TCP for the web server     |
| me-no-dev/ESPAsyncWebServer      | HTTP REST API + captive portal   |
| bblanchon/ArduinoJson            | JSON serialisation               |
| GordonAnderson/GAACE_Core        | Serial command processor         |

## Project layout

```
.
├── platformio.ini          PlatformIO environment (board: esp32-s3-devkitc-1)
├── patch_libs.py           Pre-build library compatibility patch
├── rename_firmware.py      Post-build firmware versioning / release copy
├── src/VacuumGauge.cpp     Application firmware
├── include/
│   ├── VacuumGauge.h       Function prototypes
│   ├── version.h           Firmware version number
│   └── build_info.h        Auto-generated build timestamp (do not edit)
└── releases/               Versioned .bin files safe to commit
```

## License

Copyright © GAA Custom Electronics, LLC. All rights reserved.
