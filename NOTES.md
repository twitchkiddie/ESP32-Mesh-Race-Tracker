# Project Notes

---

## Scalability Limits (>24 Devices)

### ✅ FIXED — all three concrete limits resolved (2026-02-26)

---

### Limit 1 — Device IDs: hard cap at 31 (IDs 0–30)
- `MAX_DEVICE_ID = 30` in `config.h`
- The boot-button cycle only goes 0 → 30. That's the intended ceiling.
- If you ever need more: bump `MAX_DEVICE_ID` in `config.h`.

---

### Limit 2 — Per-peer RSSI tracking ✅ FIXED: now 16 peers per node
- `PEER_RSSI_MAX = 16` in `espnow_mesh.h` (was 8)
- Bump was 7 bytes × 16 = 112 bytes of RAM per node — negligible.
- Max HB packet with 16 peers: 21 + 16×7 = **133 bytes** — well under the 250-byte ESP-NOW limit.
- In staging area (all 30 boats in range of each other), a relay might hear 29 neighbors.
  16 covers the vast majority of real-world scenarios; trackers far from shore rarely hear more than 4–5 peers.
- If you need LRU eviction (drop weakest/oldest peer for a new one), that's a future firmware change.

---

### Limit 3 — Dedup buffer ✅ FIXED: now 512 entries
- `MESH_DEDUP_SIZE = 512` in `config.h` (was 128), `MESH_DEDUP_EXPIRY_MS = 10000` ms
- Memory cost: ~12 bytes/entry × 512 = **~6 KB** — fine on ESP32's 320 KB RAM

**Staging area math (worst case — 30 boats powered up, stationary):**
- Stationary boats → GPS interval = 10s (`GPS_INTERVAL_IDLE_MS`)
- 30 boats × (1 GPS/10s + 1 HB/10s) = **6 original packets/sec**
- Each packet heard by all 30 nodes → each relays it once, then dedup blocks further relays
- Total transmissions: 6 × 30 = **180/sec** on channel 6
- LR mode ~250kbps, 30-byte frame ≈ 3ms → 180 × 3ms = **~54% channel utilization** — workable with CSMA/CA backoff

**Racing math (worst case — 30 boats moving fast, 1 Hz GPS):**
- 30 boats × 1 GPS/sec = 30 entries/sec into dedup table
- 30/sec × 10s expiry window = **300 live entries needed**
- 512 slots → safe headroom ✅

---

### Limit 4 — seq_num rollover: not a real problem
- `seq_num` is `uint8_t` (0–255 per sender).
- At 1 Hz, rollover happens every ~4 minutes. Dedup entries expire in 10s — by the time a seq number repeats, the old entry is long gone. No phantom relays.

---

### Limit 5 — Admin dashboard colors ✅ FIXED: now 31 unique colors
- `DEVICE_COLORS` in `server/serial_bridge.py` expanded from 12 → **31 entries** (IDs 0–30)
- Each device ID now has a unique, visually distinct color on the dark dashboard.
- Color is still assigned as `device_id % len(DEVICE_COLORS)` — with 31 entries matching MAX_DEVICE_ID+1, modulo never triggers.

---

### Limit 6 — Relay storm (staging area)
- `MESH_MAX_HOPS = 5`, relay random delay 0.5–5 ms
- In staging (all boats in range of each other): each packet relayed **once per node in range**, then dedup prevents further cascade.
  - NOT N×MAX_HOPS — dedup cuts it off after the first relay round.
- At staging speeds (10s interval) the ~54% channel load is acceptable.
- At racing speeds (1s interval) with 30 boats: 30 × 30 = 900 transmissions/sec → **channel will be congested**.
  - Mitigation: boats spread out during racing, reducing simultaneous neighbors per node.
  - No code change needed — this is an RF geometry problem, not a firmware bug.

---

### Summary Table (current state after fixes)

| Limit | **Current value** | Safe for 30 boats? | Notes |
|---|---|---|---|
| Device IDs | 0–30 (31 total) | ✅ Yes | Bump `MAX_DEVICE_ID` if ever >30 |
| Peer RSSI per node | **16 peers** | ✅ Yes | Was 8; covers staging proximity |
| **Dedup buffer** | **512 entries** | ✅ Yes | Was 128; 6KB RAM cost |
| seq_num rollover | uint8 / 255 | ✅ Yes | 4+ min @ 1 Hz |
| Admin panel colors | **31 unique** | ✅ Yes | Was 12; covers IDs 0–30 |
| Channel load (staging, stationary) | ~54% | ✅ Yes | Workable with CSMA/CA |
| Channel load (racing, 1Hz, all in range) | ~900 tx/sec | ⚠️ Congested | RF geometry helps at race spread |

---

## Battery Voltage Reading Is Wrong
- `BATTERY_VOLTAGE_DIVIDER_RATIO = 2.0f` assumes equal resistors (47k + 47k).
- Verify the actual resistor values on the hardware — if they differ, update the ratio.
- ADC on ESP32 GPIO34 is nonlinear; may need the ESP-IDF ADC calibration API or a lookup table for accurate mV readings.
- **Deferred — fix before race day.**

---

## COM Port Reassignment (Windows)
- Tracker 0 Red (MAC `30:c6:f7:20:a8:e8`) was on COM6, moved to **COM10** after plugging into a different USB port.
- Windows assigns COM ports per USB hub location, not per device.
- To find the current port: `pio device list` or Device Manager → Ports.
- Current port assignments (as of last flash):
  - **COM5** — Tracker 1 Green
  - **COM10** — Tracker 0 Red (MAC `30:c6:f7:20:a8:e8`)
  - **COM7** — Gateway (also has NVS ROLE_GATEWAY set from earlier; harmless — server activation works either way)

---

## Gateway Role — Server-Side Activation

Gateway mode is now activated by the server, not baked into the device:

- **`role gateway now`** — activates gateway mode in RAM immediately, no NVS write, no reboot.
  The device reverts to auto-detect (RELAY if no GPS) on the next reboot.
- The **`serial_bridge.py`** sends `role gateway now` automatically every time it connects to the serial port.
  Any device plugged into the server's USB port becomes a gateway for that session.
- Only works on a device currently in **RELAY** mode. A TRACKER (GPS detected) ignores the command.
- A device already in GATEWAY mode (via NVS) is already binary-framing and won't see the text command — which is fine, it's already correct.

### Full role command set (all roles, for reference)
| Command | Persists? | Effect |
|---|---|---|
| `role gateway now` | ❌ Session only | Immediate RELAY→GATEWAY, reverts on reboot |
| `role gateway` | ✅ NVS | Saves GATEWAY; applies on reboot |
| `role relay` | ✅ NVS | Saves RELAY override; applies on reboot |
| `role tracker` | ✅ NVS | Saves TRACKER; GPS detection still takes hardware priority |
| `role auto` | ✅ NVS cleared | Returns to hardware auto-detect on reboot |
| `role` | — | Prints current role |

---

## OTA Known Issue
- OTA transfer crashes at ~212 chunks (out of ~600 for a full firmware image).
- Root cause not yet identified. Deprioritized — direct USB flash is used for all reachable nodes.
- Remote-only tracker (battery-powered, no USB) is the only node that requires OTA.
- **Deferred — investigate before race day.**
