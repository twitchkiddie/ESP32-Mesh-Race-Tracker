# ESP32 Mesh Race Tracker — V2

A GPS tracking system for crew boat racing. Multiple ESP32 nodes form a self-healing mesh network using ESP-NOW. Live position data is forwarded to a laptop running a Python/Flask dashboard with race timing and route mapping.

---

## How It Works

```
[Tracker] ──ESP-NOW──> [Relay] ──ESP-NOW──> [Gateway] ──USB──> [Laptop Dashboard]
  (GPS)                 (optional)           (bridge)            (Flask + browser)
```

Every device runs **identical firmware**. Role is determined automatically at boot:

| Role | How detected | What it does |
|---|---|---|
| **Tracker** | GPS module found on Serial2 | Broadcasts GPS + heartbeat over mesh |
| **Relay** | No GPS, not gateway | Forwards mesh packets, extends range |
| **Gateway** | USB connected + server sends `role gateway now` | Bridges mesh → laptop via serial |

No manual configuration required for trackers or relays. The gateway is activated automatically when the server connects to it over USB.

---

## Hardware

- **ESP32 dev board** (any standard 38-pin module)
- **GPS module** (UART, wired to GPIO16 RX / GPIO17 TX) — trackers only
- **NeoPixel LED** (GPIO25) — device ID color indicator
- **Battery ADC** (GPIO34) — optional, via voltage divider
- **SD card** (SPI, CS GPIO5) — optional GPS track logging on trackers

All nodes use the same hardware; GPS module presence determines the role.

---

## Firmware

### Build & Flash

Requires [PlatformIO](https://platformio.org/).

```bash
# Build
pio run

# Flash a specific port
pio run -t upload --upload-port COM7      # Windows
pio run -t upload --upload-port /dev/tty.usbserial-0001   # Mac/Linux
```

### Device ID

Each tracker/relay has an ID (0–30) stored in NVS. Hold the **BOOT button** for 3 seconds to cycle to the next ID. The NeoPixel shows the assigned color.

### Role Commands (serial monitor, 115200 baud)

| Command | Effect |
|---|---|
| `role` | Print current role |
| `role gateway now` | Activate gateway mode immediately (session only, reverts on reboot) |
| `role gateway` | Save GATEWAY to NVS (persists across reboots) |
| `role relay` | Save RELAY to NVS |
| `role tracker` | Save TRACKER to NVS |
| `role auto` | Clear NVS — return to hardware auto-detect |
| `status` | Firmware version, MAC, role, battery, nodes seen |
| `id <n>` | Set device ID (0–30) |
| `reboot` | Restart |

---

## Server

### Requirements

```bash
pip install flask pyserial
```

### Run

```bash
cd server

# Windows
python mesh_server.py --port COM7

# Mac / Linux
python mesh_server.py --port /dev/tty.usbserial-0001

# With Google Maps (optional — OpenStreetMap is the default)
python mesh_server.py --port COM7 --gmaps-key YOUR_API_KEY
```

Then open **http://localhost:5000** in a browser.

| URL | Page |
|---|---|
| `/` | Live GPS dashboard (map + trails) |
| `/race` | Race timing — line crossing, leader board, ETA |
| `/admin` | Node list, signal strength, firmware versions, raw frame log |

### Finding the serial port

```bash
# Windows
pio device list

# Mac / Linux
ls /dev/tty.usb*
```

---

## Protocol

All mesh messages are **binary packed structs** — no JSON, no string parsing.

| Message | Size | Description |
|---|---|---|
| `GPS` | 30 bytes | Lat/lon/alt/speed/heading/sats/HDOP/battery/RSSI |
| `Heartbeat` | 21 + N×7 bytes | Role/battery/uptime/nodes seen + per-peer RSSI table |
| `OTA_CTRL` | 33 bytes | Start/abort/status for over-the-air update |
| `OTA_DATA` | 13–213 bytes | Firmware chunk (200 bytes max, stays under ESP-NOW 250B limit) |
| `CMD` | 33 bytes | Gateway → node: set ID, reboot, set interval |
| `ACK` | 13 bytes | Acknowledgment |

Every message has a 10-byte `PacketHeader` (type, hop count, device ID, sequence number, source MAC). The gateway wraps messages in a CRC16-framed serial envelope before forwarding to the laptop.

---

## Scalability

Tested with 3 nodes. Designed for up to 30 boats:

| Parameter | Value | Notes |
|---|---|---|
| Max device IDs | 31 (0–30) | Boot button cycles 0→30 |
| Dedup buffer | 512 entries | Handles 30 boats @ 1 Hz over 10 s expiry window |
| Per-peer RSSI | 16 peers/node | Covers staging area density |
| Channel load (staging, stationary) | ~54% | 30 boats × idle interval, workable with CSMA/CA |
| Max hops | 5 | Covers ~6 km at ~1.2 km/hop range |

---

## GPS Update Rates

Trackers auto-adjust broadcast rate based on speed:

| Speed | Interval |
|---|---|
| ≥ 10 km/h (racing) | 1 s |
| 5–10 km/h | 2 s |
| 1–5 km/h | 5 s |
| < 1 km/h (stationary) | 10 s |

---

## Known Issues

- **Battery voltage** reading may be inaccurate — ESP32 ADC on GPIO34 is nonlinear and the divider ratio may not match hardware. Fix before race day.
- **OTA** firmware transfer crashes at ~212 chunks. Direct USB flash is used for all reachable nodes. Investigate before race day.

---

## Project Structure

```
ESP32_Mesh/
├── platformio.ini
├── src/
│   ├── main.cpp            # Role detection, setup, loop
│   ├── config.h            # All constants and pin definitions
│   ├── protocol.h          # Binary packed structs + CRC16
│   ├── espnow_mesh.cpp/h   # ESP-NOW send/receive, relay, dedup
│   ├── gps_handler.cpp/h   # NMEA parsing, speed-adaptive interval
│   ├── gateway_serial.cpp/h # Binary framing over USB serial
│   ├── battery_monitor.cpp/h
│   ├── led_controller.cpp/h # NeoPixel device color
│   ├── device_manager.cpp/h # Boot button, NVS device ID
│   ├── power_manager.cpp/h  # CPU frequency per role
│   ├── sd_logger.cpp/h      # GPS track logging to SD card
│   └── ota_handler.cpp/h    # Chunked OTA firmware receive
├── server/
│   ├── mesh_server.py      # Flask REST API, entry point
│   ├── serial_bridge.py    # Binary frame parser, node state
│   ├── race_engine.py      # Course, line crossing, leader board
│   ├── requirements.txt
│   └── templates/
│       ├── dashboard.html  # Live map
│       ├── race.html       # Race timing
│       └── admin.html      # Node/signal/firmware debug
└── tools/
    └── firmware_upload.py  # OTA firmware push via gateway
```
