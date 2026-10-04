# ESP32 GPS logger

A resilient GPS track logger for the existing ESP32, u-blox receiver, microSD card, and
128x32 SSD1306 OLED hardware. The firmware stores checksum-protected UBX-NAV-PVT packets as
the canonical track and exposes live data and completed tracks over encrypted Bluetooth LE.
GPX is generated later on a computer, so the original receiver data is retained.

## What changed

- PlatformIO build with pinned dependencies and separate host-side tests
- Raw UBX-NAV-PVT logging at 1 Hz instead of formatting text in the capture loop
- Buffered GNSS reads, periodic SD sync, automatic SD/GNSS retry, and no permanent blocking
  error screens
- A new file on every boot, with monotonically increasing session IDs in ESP32 NVS
- Secure BLE status, live fixes, track listing, and binary download
- A dependency-free, checksum-validating UBX-to-GPX converter

## Unchanged hardware

The pin assignments match the original sketch:

| Device | ESP32 connection |
| --- | --- |
| u-blox TX -> ESP32 RX | GPIO 16 |
| u-blox RX <- ESP32 TX | GPIO 17 |
| OLED SDA | GPIO 21 |
| OLED SCL | GPIO 22 |
| microSD CS | GPIO 5 |
| microSD SCK | GPIO 18 |
| microSD MISO | GPIO 19 |
| microSD MOSI | GPIO 23 |

The build currently targets PlatformIO's `esp32dev` board definition. If the board is a more
specific ESP32 variant, change only `board` in `platformio.ini`; the wiring constants live in
`src/hardware_config.h`.

## Build, test, and upload

Install [PlatformIO Core](https://docs.platformio.org/en/latest/core/installation/index.html),
then run:

```sh
pio test -e native
pio run -e esp32
pio run -e esp32 -t upload
pio device monitor -b 115200
```

The firmware first tries the GNSS at 38400 baud, then at the common 9600-baud fallback. When
found at 9600, it changes the active UART rate to 38400 without factory-resetting the receiver
or writing its nonvolatile configuration.

## Track storage

Tracks are stored under `/tracks` on the microSD card:

```text
/tracks/S00000001.ubx
/tracks/S00000002.ubx
```

Each boot allocates a new ID. Data is flushed every five seconds, balancing card wear and
power-loss exposure; an abrupt power loss can therefore lose the most recent few seconds. A
partial final UBX packet is harmless and is ignored by the converter. Logging continues if
the OLED is missing, and the firmware periodically retries a missing SD card or GNSS receiver.

The active file is write-only from the firmware's point of view. BLE download deliberately
rejects it to avoid simultaneous access. Rebooting starts a new session, making the previous
one available for download.

## Bluetooth LE

The device advertises as `GPS-Logger-XXXXXX`. Pair using the six-digit PIN shown on the OLED
and serial console. The connection requires authenticated encryption and is bonded by the ESP32.

It uses the Nordic UART Service layout:

| Purpose | UUID |
| --- | --- |
| Service | `6e400001-b5a3-f393-e0a9-e50e24dcca9e` |
| Commands: client -> logger | `6e400002-b5a3-f393-e0a9-e50e24dcca9e` |
| Notifications: logger -> client | `6e400003-b5a3-f393-e0a9-e50e24dcca9e` |

Subscribe to the notification characteristic, then write UTF-8 commands to the command
characteristic. Commands are case-insensitive and may end with CR/LF.

| Command | Result |
| --- | --- |
| `HELP` | List commands |
| `STATUS` | GNSS, SD, fix, session, and byte counters |
| `LIVE ON` | Send one CSV-like `FIX` line per NAV-PVT update |
| `LIVE OFF` | Stop live updates |
| `LIST` | Stream `FILE S######## size` records; the current file is marked `ACTIVE` |
| `GET S########` | Download a completed raw UBX session |
| `CANCEL` | Stop a list or download |

Live records use `FIX timestamp,latitude,longitude,altitude_m,satellites,fix_type,hacc_mm`.

A download begins with `BEGIN S######## N\n`. The following notifications contain exactly
`N` binary UBX bytes, followed by `END S########\n`. Treat the byte count—not newlines inside
the binary data—as the boundary. Other commands are ignored during those `N` bytes so text
cannot be inserted into the file stream. `CANCEL` is the only exception and ends the transfer.

## Convert UBX to GPX

The converter needs only Python 3.9 or newer:

```sh
python3 tools/ubx_to_gpx.py S00000042.ubx
python3 tools/ubx_to_gpx.py S00000042.ubx morning-ride.gpx
```

It scans incrementally, validates every UBX checksum, resynchronizes after corrupt bytes, and
ignores incomplete data at the end. Only NAV-PVT records with resolved UTC and a valid 2D/3D
position enter the GPX track. The original `.ubx` file should be kept as the archival source.

Run converter tests with:

```sh
python3 -m unittest discover -s tools/tests -v
```

## Runtime diagnostics

The serial monitor prints startup, retry, session, pairing, and fix information. The OLED shows:

1. active session or SD status;
2. GNSS fix type, satellite count, and PDOP;
3. current coordinates or a waiting message;
4. BLE pairing PIN/connection state, or an SD write error.

If SD writing fails, the file is closed immediately and the retry loop attempts to mount the
card and start a fresh session. Bytes received while storage is unavailable are counted as
dropped instead of blocking GNSS processing.
