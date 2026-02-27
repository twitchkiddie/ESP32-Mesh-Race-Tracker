"""
ESP32 Mesh V2 - Serial Bridge
Parses binary frames from the gateway ESP32 and maintains live node state.

Frame format: [0xAA][0x55][LEN_LO][LEN_HI][PAYLOAD...][CRC_LO][CRC_HI]
CRC: CRC16-CCITT (XModem) over PAYLOAD only.
"""

import json
import os
import struct
import threading
import time
import logging
from collections import deque
from dataclasses import dataclass, field
from typing import Callable, Dict, Optional, List

import serial
import serial.tools.list_ports

logger = logging.getLogger(__name__)

# ===========================================================================
# Protocol constants (must match config.h / protocol.h)
# ===========================================================================

SYNC1 = 0xAA
SYNC2 = 0x55
MAX_FRAME = 256

MSG_GPS       = 0x01
MSG_HEARTBEAT = 0x02
MSG_CMD       = 0x10
MSG_OTA_CTRL  = 0x20
MSG_OTA_DATA  = 0x21
MSG_ACK       = 0x30
MSG_GW_STATUS = 0xFF  # Gateway's own heartbeat

ROLE_TRACKER = 0
ROLE_RELAY   = 1
ROLE_GATEWAY = 2

ROLE_NAMES = {ROLE_TRACKER: "Tracker", ROLE_RELAY: "Relay", ROLE_GATEWAY: "Gateway"}
MSG_NAMES  = {MSG_GPS: "GPS", MSG_HEARTBEAT: "Heartbeat", MSG_CMD: "CMD",
              MSG_GW_STATUS: "GW-Status"}

# Device ID → (name, hex color)
# 31 entries covering IDs 0–30 (MAX_DEVICE_ID).
# Colors chosen for visual distinctiveness on a dark dashboard background.
DEVICE_COLORS = [
    ("Red",          "#FF4444"),  #  0
    ("Green",        "#44FF44"),  #  1
    ("Blue",         "#4488FF"),  #  2
    ("Yellow",       "#FFFF00"),  #  3
    ("Orange",       "#FF8800"),  #  4
    ("Cyan",         "#00FFFF"),  #  5
    ("Pink",         "#FF69B4"),  #  6
    ("Lime",         "#AAFF00"),  #  7
    ("Violet",       "#CC44FF"),  #  8
    ("Gold",         "#FFD700"),  #  9
    ("Sky",          "#87CEEB"),  # 10
    ("Coral",        "#FF6B6B"),  # 11
    ("Mint",         "#00FA9A"),  # 12
    ("Amber",        "#FFBF00"),  # 13
    ("Lavender",     "#CC99FF"),  # 14
    ("Teal",         "#00CED1"),  # 15
    ("Salmon",       "#FA8072"),  # 16
    ("Spring",       "#00FF7F"),  # 17
    ("Peach",        "#FFCBA4"),  # 18
    ("Rose",         "#FF007F"),  # 19
    ("Steel",        "#6699CC"),  # 20
    ("Chartreuse",   "#DFFF00"),  # 21
    ("Tomato",       "#FF6347"),  # 22
    ("Aqua",         "#7FFFD4"),  # 23
    ("Orchid",       "#DA70D6"),  # 24
    ("Turquoise",    "#40E0D0"),  # 25
    ("Periwinkle",   "#AAAAFF"),  # 26
    ("Apricot",      "#FBCEB1"),  # 27
    ("Jade",         "#00A86B"),  # 28
    ("Magenta",      "#FF00CC"),  # 29
    ("White",        "#FFFFFF"),  # 30
]

MAX_TRAIL_POINTS = 50  # GPS history per boat

# ===========================================================================
# Struct formats (little-endian, packed)
# ===========================================================================
# PacketHeader: msg_type(B) hop_count(B) device_id(B) seq_num(B) src_mac(6s) = 10 bytes
HEADER_FMT = struct.Struct('<BBBB6s')
assert HEADER_FMT.size == 10

# GPS body (after stripping 10-byte header from 30-byte GPSMessage):
# lat_e7(i) lon_e7(i) alt_dm(h signed) speed_cmps(H) heading_cd(H) satellites(B) hdop_tenths(B) batt_mv(H) batt_pct(B) rssi(b)
# = 4+4+2+2+2+1+1+2+1+1 = 20 bytes
GPS_BODY_FMT = struct.Struct('<iihHHBBHBb')
assert GPS_BODY_FMT.size == 20

# Heartbeat body — two variants for backward compatibility:
#   2.0.0 (10 bytes): role(B) batt_mv(H) batt_pct(B) nodes_seen(B) uptime_s(I) rssi(b)
#   2.0.1 (11 bytes): same + fw_patch(B)
HB_BODY_FMT_OLD = struct.Struct('<BHBBIb')   # 10 bytes — firmware 2.0.0
HB_BODY_FMT     = struct.Struct('<BHBBIbB')  # 11 bytes — firmware 2.0.1+
assert HB_BODY_FMT_OLD.size == 10
assert HB_BODY_FMT.size == 11

# Gateway heartbeat (10 bytes, type=0xFF, no PacketHeader):
# type(B) node_count(B) uptime_s(I) free_heap(I) = 1+1+4+4 = 10 bytes
GW_HB_FMT = struct.Struct('<BBII')
assert GW_HB_FMT.size == 10

# Per-peer RSSI entry appended after HB body (firmware 2.0.2+):
# mac(6s) rssi(b) = 7 bytes each
PEER_ENTRY_FMT = struct.Struct('<6sb')
assert PEER_ENTRY_FMT.size == 7


# ===========================================================================
# CRC16-CCITT (XModem) — must match SerialProto::crc16 in protocol.h
# ===========================================================================

def crc16(data: bytes) -> int:
    """CRC16-CCITT with init=0x0000 — matches SerialProto::crc16 in protocol.h."""
    crc = 0x0000
    for byte in data:
        crc ^= (byte << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def mac_to_str(mac_bytes: bytes) -> str:
    return ':'.join(f'{b:02X}' for b in mac_bytes)


def device_color(device_id: int):
    return DEVICE_COLORS[device_id % len(DEVICE_COLORS)]


# ===========================================================================
# Node State
# ===========================================================================

@dataclass
class NodeState:
    mac: str
    device_id: int = 0
    role: int = ROLE_TRACKER
    last_seen: float = field(default_factory=time.time)

    # GPS (trackers)
    has_gps: bool = False
    lat: float = 0.0
    lon: float = 0.0
    alt_m: float = 0.0
    speed_mps: float = 0.0
    heading_deg: float = 0.0
    satellites: int = 0
    hdop: float = 99.0
    trail: deque = field(default_factory=lambda: deque(maxlen=MAX_TRAIL_POINTS))

    # Battery
    batt_mv: int = 0
    batt_pct: int = 0

    # Mesh
    nodes_seen: int = 0
    uptime_s: int = 0
    rssi: int = 0
    hop_count: int = 0
    seq_num: int = 0

    # Packet counts
    gps_count: int = 0
    hb_count: int = 0

    # Firmware version
    fw_patch: int = 0   # populated from HB fw_patch field (2.0.1+)

    # Per-peer RSSI table: {mac_str: rssi_int} — populated from trailing HB entries (2.0.2+)
    peer_rssi: dict = field(default_factory=dict)

    @property
    def color(self) -> str:
        return device_color(self.device_id)[1]

    @property
    def color_name(self) -> str:
        return device_color(self.device_id)[0]

    @property
    def role_name(self) -> str:
        return ROLE_NAMES.get(self.role, f"Unknown({self.role})")

    @property
    def label(self) -> str:
        return f"{self.role_name} {self.device_id}"

    @property
    def age_s(self) -> float:
        return time.time() - self.last_seen

    @property
    def is_stale(self) -> bool:
        return self.age_s > 30

    def to_dict(self) -> dict:
        return {
            "mac": self.mac,
            "device_id": self.device_id,
            "role": self.role,
            "role_name": self.role_name,
            "label": self.label,
            "color": self.color,
            "color_name": self.color_name,
            "last_seen": self.last_seen,
            "age_s": round(self.age_s, 1),
            "is_stale": self.is_stale,
            "has_gps": self.has_gps,
            "lat": self.lat,
            "lon": self.lon,
            "alt_m": round(self.alt_m, 1),
            "speed_mps": round(self.speed_mps, 2),
            "speed_kmph": round(self.speed_mps * 3.6, 1),
            "heading_deg": round(self.heading_deg, 1),
            "satellites": self.satellites,
            "hdop": round(self.hdop, 1),
            "trail": list(self.trail),
            "batt_mv": self.batt_mv,
            "batt_pct": self.batt_pct,
            "nodes_seen": self.nodes_seen,
            "uptime_s": self.uptime_s,
            "rssi": self.rssi,
            "hop_count": self.hop_count,
            "seq_num": self.seq_num,
            "gps_count": self.gps_count,
            "hb_count": self.hb_count,
            "fw_version": f"2.0.{self.fw_patch}",
            "peer_rssi": self.peer_rssi,
        }


@dataclass
class GatewayStatus:
    connected: bool = False
    port: str = ""
    node_count: int = 0
    uptime_s: int = 0
    free_heap: int = 0
    last_seen: float = 0.0
    frames_received: int = 0
    frames_dropped: int = 0
    # Per-peer RSSI as seen by the gateway: {mac_str: rssi_int}
    peer_rssi: dict = field(default_factory=dict)

    def to_dict(self) -> dict:
        return {
            "connected": self.connected,
            "port": self.port,
            "node_count": self.node_count,
            "uptime_s": self.uptime_s,
            "free_heap": self.free_heap,
            "last_seen": self.last_seen,
            "frames_received": self.frames_received,
            "frames_dropped": self.frames_dropped,
            "peer_rssi": self.peer_rssi,
        }


# ===========================================================================
# Frame Parser (state machine)
# ===========================================================================

class FrameParser:
    """
    Stateful byte-stream parser for the gateway binary protocol.
    Feed bytes one at a time via feed(b). Yields complete frames as bytes.
    """

    def __init__(self):
        self._state = 'SYNC1'
        self._buf = bytearray()
        self._expected_len = 0

    def feed(self, byte: int):
        """Feed one byte. Returns parsed payload bytes if frame complete, else None."""
        if self._state == 'SYNC1':
            if byte == SYNC1:
                self._state = 'SYNC2'

        elif self._state == 'SYNC2':
            if byte == SYNC2:
                self._state = 'LEN_LO'
            else:
                self._state = 'SYNC1'

        elif self._state == 'LEN_LO':
            self._expected_len = byte
            self._state = 'LEN_HI'

        elif self._state == 'LEN_HI':
            self._expected_len |= (byte << 8)
            if self._expected_len == 0 or self._expected_len > MAX_FRAME:
                self._state = 'SYNC1'
            else:
                self._buf = bytearray()
                self._state = 'PAYLOAD'

        elif self._state == 'PAYLOAD':
            self._buf.append(byte)
            if len(self._buf) >= self._expected_len:
                self._state = 'CRC_LO'

        elif self._state == 'CRC_LO':
            self._crc_lo = byte
            self._state = 'CRC_HI'

        elif self._state == 'CRC_HI':
            received_crc = self._crc_lo | (byte << 8)
            payload = bytes(self._buf)
            self._state = 'SYNC1'
            computed_crc = crc16(payload)
            if computed_crc == received_crc:
                return payload
            else:
                logger.debug(f"CRC mismatch: got 0x{received_crc:04X}, expected 0x{computed_crc:04X}")
                return None

        return None


# ===========================================================================
# Serial Bridge
# ===========================================================================

class SerialBridge:
    """
    Reads binary frames from the gateway's serial port, decodes them,
    and maintains a live dictionary of node states.

    Usage:
        bridge = SerialBridge(port='COM7', baud=115200)
        bridge.on_update(lambda nodes, gw: ...)  # called on any new data
        bridge.start()
        # ... bridge.nodes, bridge.gateway
        bridge.stop()
    """

    def __init__(self, port: str, baud: int = 115200, reconnect_delay: float = 3.0,
                 names_file: Optional[str] = None):
        self.port = port
        self.baud = baud
        self.reconnect_delay = reconnect_delay

        self.nodes: Dict[str, NodeState] = {}
        self.gateway = GatewayStatus(port=port)
        self.frame_log: deque = deque(maxlen=200)  # newest first

        # Custom display names: {mac_str: name_str}
        self._names_file = names_file or os.path.join(os.path.dirname(__file__), 'names.json')
        self._custom_names: Dict[str, str] = self._load_names()

        self._lock = threading.Lock()
        self._callbacks: List[Callable] = []
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._parser = FrameParser()
        self._ser: Optional[serial.Serial] = None  # set while connected, for sending commands

    def on_update(self, callback: Callable):
        """Register callback: fn(nodes: dict, gateway: GatewayStatus)"""
        self._callbacks.append(callback)

    def start(self):
        """Start the background serial read thread."""
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run, daemon=True, name="SerialBridge")
        self._thread.start()
        logger.info(f"[Bridge] Started on {self.port} @ {self.baud} baud")

    def stop(self):
        """Stop the background thread gracefully."""
        self._stop_event.set()
        if self._thread:
            self._thread.join(timeout=5)
        logger.info("[Bridge] Stopped")

    def get_state(self) -> dict:
        """Thread-safe snapshot of current state."""
        with self._lock:
            nodes = {}
            for mac, n in self.nodes.items():
                d = n.to_dict()
                custom = self._custom_names.get(mac, '')
                d['custom_name'] = custom
                d['display_name'] = custom if custom else d['label']
                nodes[mac] = d
            return {
                "nodes": nodes,
                "gateway": self.gateway.to_dict(),
                "timestamp": time.time(),
            }

    def get_log(self, limit: int = 100) -> list:
        """Thread-safe snapshot of recent frame log (newest first)."""
        with self._lock:
            return list(self.frame_log)[:limit]

    # ------------------------------------------------------------------
    # Custom names
    # ------------------------------------------------------------------

    def _load_names(self) -> Dict[str, str]:
        try:
            with open(self._names_file, 'r') as f:
                return json.load(f)
        except Exception:
            return {}

    def _save_names(self):
        try:
            with open(self._names_file, 'w') as f:
                json.dump(self._custom_names, f, indent=2)
        except Exception as e:
            logger.error(f"[Bridge] Could not save names: {e}")

    def set_name(self, mac: str, name: str):
        """Set a custom display name for a node. Persists to names.json."""
        with self._lock:
            self._custom_names[mac] = name[:32].strip()
        self._save_names()

    def clear_name(self, mac: str):
        """Remove the custom display name for a node."""
        with self._lock:
            self._custom_names.pop(mac, None)
        self._save_names()

    def get_names(self) -> Dict[str, str]:
        with self._lock:
            return dict(self._custom_names)

    # ------------------------------------------------------------------
    # Outbound command sending (laptop → gateway → mesh)
    # ------------------------------------------------------------------

    def _build_frame(self, msg_payload: bytes) -> bytes:
        """Wrap a binary message payload in the serial framing envelope."""
        c = crc16(msg_payload)
        return (bytes([SYNC1, SYNC2,
                       len(msg_payload) & 0xFF,
                       (len(msg_payload) >> 8) & 0xFF])
                + msg_payload
                + bytes([c & 0xFF, (c >> 8) & 0xFF]))

    def send_cmd_set_device_id(self, target_mac_str: str, new_id: int) -> bool:
        """
        Send CMD_SET_DEVICE_ID to a specific node via the gateway.
        The gateway rewrites src_mac before forwarding to the mesh.
        Returns True if the frame was written to the serial port.
        """
        if self._ser is None:
            logger.warning("[Bridge] send_cmd_set_device_id: not connected")
            return False
        try:
            target_mac = bytes(int(x, 16) for x in target_mac_str.split(':'))
            # CommandMessage layout (33 bytes total):
            #   PacketHeader (10): msg_type hop dev_id seq src_mac(6)
            #   Body       (23): cmd_type target_mac(6) payload(16)
            payload_bytes = bytes([new_id]) + b'\x00' * 15
            msg = struct.pack('<BBBB6sB6s16s',
                              0x10,        # MSG_CMD
                              0,           # hop_count (gateway sets)
                              0,           # device_id (don't care)
                              0,           # seq_num (don't care)
                              b'\x00' * 6, # src_mac (gateway overwrites)
                              0x01,        # CMD_SET_DEVICE_ID
                              target_mac,
                              payload_bytes)
            frame = self._build_frame(msg)
            with self._lock:
                self._ser.write(frame)
            logger.info(f"[Bridge] CMD_SET_DEVICE_ID → {target_mac_str} new_id={new_id}")
            return True
        except Exception as e:
            logger.error(f"[Bridge] send_cmd_set_device_id error: {e}")
            return False

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _run(self):
        while not self._stop_event.is_set():
            try:
                self._connect_and_read()
            except Exception as e:
                logger.warning(f"[Bridge] Error: {e}. Reconnecting in {self.reconnect_delay}s...")
                with self._lock:
                    self.gateway.connected = False
                self._notify()
                time.sleep(self.reconnect_delay)

    def _connect_and_read(self):
        logger.info(f"[Bridge] Connecting to {self.port}...")
        with serial.Serial(
            port=self.port,
            baudrate=self.baud,
            timeout=1.0,
            dsrdtr=False,
            rtscts=False,
        ) as ser:
            ser.setDTR(False)
            ser.setRTS(False)
            self._ser = ser          # store ref so send_cmd_* can write
            self._parser = FrameParser()

            # --- Gateway activation (server-side logic) ---
            # The server is the one that knows a device is intended as a gateway
            # (it's the device plugged into this USB port). Rather than relying
            # on persistent NVS configuration, we send 'role gateway now' each
            # time we connect. This activates gateway mode in RAM only — the
            # device reverts to auto-detect on reboot. No manual setup required.
            #
            # Safety: the firmware only allows this switch from RELAY → GATEWAY.
            # A tracker (GPS detected at boot) ignores the command and stays TRACKER.
            # A device already in GATEWAY mode (NVS) is already binary-framing and
            # won't be reading text commands — which is fine, it's already correct.
            time.sleep(0.5)          # wait for device to be ready after port open
            ser.write(b'role gateway now\r\n')
            time.sleep(0.3)          # let device process command and switch modes
            ser.reset_input_buffer() # discard text ACK and any lingering boot noise
            logger.info(f"[Bridge] Sent gateway activation to {self.port}")
            # --------------------------------------------------------------

            with self._lock:
                self.gateway.connected = True

            logger.info(f"[Bridge] Connected to {self.port}")
            self._notify()

            while not self._stop_event.is_set():
                raw = ser.read(256)
                if not raw:
                    continue
                for byte in raw:
                    payload = self._parser.feed(byte)
                    if payload is not None:
                        self._process_frame(payload)

        self._ser = None
        with self._lock:
            self.gateway.connected = False
        self._notify()

    def _process_frame(self, payload: bytes):
        if len(payload) < 1:
            return

        msg_type = payload[0]
        entry = {
            "t": time.time(),
            "type": msg_type,
            "type_name": MSG_NAMES.get(msg_type, f"0x{msg_type:02X}"),
            "len": len(payload),
            "hex": payload.hex(),
        }

        with self._lock:
            self.gateway.frames_received += 1

            if msg_type == MSG_GW_STATUS:
                self._decode_gw_heartbeat(payload)
                entry.update({
                    "label": "Gateway",
                    "color": "#0080FF",
                    "uptime_s": self.gateway.uptime_s,
                    "free_heap": self.gateway.free_heap,
                    "node_count": self.gateway.node_count,
                })
            elif len(payload) >= HEADER_FMT.size:
                hdr = HEADER_FMT.unpack_from(payload, 0)
                msg_type_hdr, hop_count, device_id, seq_num, src_mac_bytes = hdr
                mac = mac_to_str(src_mac_bytes)

                if mac not in self.nodes:
                    self.nodes[mac] = NodeState(mac=mac, device_id=device_id)
                    logger.info(f"[Bridge] New node: {mac} (ID {device_id})")

                node = self.nodes[mac]
                node.device_id = device_id
                node.hop_count = hop_count
                node.seq_num = seq_num
                node.last_seen = time.time()

                entry.update({
                    "mac": mac,
                    "device_id": device_id,
                    "hop_count": hop_count,
                    "seq_num": seq_num,
                    "label": node.label,
                    "color": node.color,
                })

                if msg_type_hdr == MSG_GPS and len(payload) >= HEADER_FMT.size + GPS_BODY_FMT.size:
                    self._decode_gps(node, payload)
                    entry.update({
                        "type_name": "GPS",
                        "lat": round(node.lat, 6),
                        "lon": round(node.lon, 6),
                        "speed_kmph": round(node.speed_mps * 3.6, 1),
                        "sats": node.satellites,
                        "hdop": node.hdop,
                        "rssi": node.rssi,
                    })
                elif msg_type_hdr == MSG_HEARTBEAT and len(payload) >= HEADER_FMT.size + HB_BODY_FMT_OLD.size:
                    self._decode_heartbeat(node, payload)
                    entry.update({
                        "type_name": "HB",
                        "role_name": node.role_name,
                        "nodes_seen": node.nodes_seen,
                        "uptime_s": node.uptime_s,
                        "rssi": node.rssi,
                    })
            else:
                self.gateway.frames_dropped += 1
                entry["dropped"] = True

            self.frame_log.appendleft(entry)  # newest first

        self._notify()

    def _decode_gw_heartbeat(self, payload: bytes):
        if len(payload) < GW_HB_FMT.size:
            return
        _, node_count, uptime_s, free_heap = GW_HB_FMT.unpack_from(payload, 0)
        self.gateway.node_count = node_count
        self.gateway.uptime_s = uptime_s
        self.gateway.free_heap = free_heap
        self.gateway.last_seen = time.time()

        # Parse optional per-peer RSSI extension:
        # byte[10] = peer_count, then peer_count × {mac(6s)+rssi(b)}
        if len(payload) >= GW_HB_FMT.size + 1:
            peer_count = payload[GW_HB_FMT.size]
            peer_rssi = {}
            offset = GW_HB_FMT.size + 1
            for _ in range(peer_count):
                if offset + PEER_ENTRY_FMT.size <= len(payload):
                    mac_bytes, rssi_val = PEER_ENTRY_FMT.unpack_from(payload, offset)
                    peer_rssi[mac_to_str(mac_bytes)] = int(rssi_val)
                    offset += PEER_ENTRY_FMT.size
            self.gateway.peer_rssi = peer_rssi

    def _decode_gps(self, node: NodeState, payload: bytes):
        body = GPS_BODY_FMT.unpack_from(payload, HEADER_FMT.size)
        lat_e7, lon_e7, alt_dm, speed_cmps, heading_cd, satellites, hdop_tenths, batt_mv, batt_pct, rssi = body

        node.has_gps = True
        node.lat = lat_e7 / 1e7
        node.lon = lon_e7 / 1e7
        node.alt_m = alt_dm / 10.0
        node.speed_mps = speed_cmps / 100.0
        node.heading_deg = heading_cd / 100.0
        node.satellites = satellites
        node.hdop = hdop_tenths / 10.0
        node.batt_mv = batt_mv
        node.batt_pct = batt_pct if batt_pct != 255 else -1  # 255 = USB powered
        node.rssi = rssi
        node.role = ROLE_TRACKER
        node.gps_count += 1

        # Append to trail
        node.trail.append({"lat": node.lat, "lon": node.lon, "t": time.time()})

    def _decode_heartbeat(self, node: NodeState, payload: bytes):
        # Support both 2.0.0 (10-byte body) and 2.0.1+ (11-byte body)
        has_fw = len(payload) >= HEADER_FMT.size + HB_BODY_FMT.size
        fmt = HB_BODY_FMT if has_fw else HB_BODY_FMT_OLD
        body = fmt.unpack_from(payload, HEADER_FMT.size)

        if has_fw:
            role, batt_mv, batt_pct, nodes_seen, uptime_s, rssi, fw_patch = body
            node.fw_patch = fw_patch
        else:
            role, batt_mv, batt_pct, nodes_seen, uptime_s, rssi = body

        node.role = role
        node.batt_mv = batt_mv
        node.batt_pct = batt_pct if batt_pct != 255 else -1
        node.nodes_seen = nodes_seen
        node.uptime_s = uptime_s
        node.rssi = rssi
        node.hb_count += 1

        # Parse trailing per-peer RSSI entries (firmware 2.0.2+)
        # Each entry: mac(6s) + rssi(b) = 7 bytes, appended after the HB body
        peer_base = HEADER_FMT.size + fmt.size
        peer_rssi = {}
        offset = peer_base
        while offset + PEER_ENTRY_FMT.size <= len(payload):
            mac_bytes, peer_rssi_val = PEER_ENTRY_FMT.unpack_from(payload, offset)
            peer_rssi[mac_to_str(mac_bytes)] = int(peer_rssi_val)
            offset += PEER_ENTRY_FMT.size
        if peer_rssi:
            node.peer_rssi = peer_rssi

    def _notify(self):
        state = {mac: n.to_dict() for mac, n in self.nodes.items()}
        gw = self.gateway.to_dict()
        for cb in self._callbacks:
            try:
                cb(state, gw)
            except Exception as e:
                logger.error(f"[Bridge] Callback error: {e}")


# ===========================================================================
# CLI: run standalone for debugging
# ===========================================================================

if __name__ == '__main__':
    import sys
    import json

    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s %(levelname)s %(message)s',
        datefmt='%H:%M:%S',
    )

    port = sys.argv[1] if len(sys.argv) > 1 else 'COM7'
    print(f"[Bridge] Connecting to {port}. Press Ctrl+C to stop.")
    print(f"[Bridge] Available ports: {[p.device for p in serial.tools.list_ports.comports()]}")

    bridge = SerialBridge(port=port)

    def on_update(nodes, gw):
        print(f"\r[GW] connected={gw['connected']} uptime={gw['uptime_s']}s nodes={gw['node_count']} "
              f"frames={gw['frames_received']}  ", end='', flush=True)
        for mac, node in nodes.items():
            if node['has_gps']:
                print(f"\n  [GPS] {node['label']} ({node['color_name']}) "
                      f"lat={node['lat']:.6f} lon={node['lon']:.6f} "
                      f"speed={node['speed_kmph']:.1f}km/h sats={node['satellites']} "
                      f"age={node['age_s']:.0f}s")
            else:
                print(f"\n  [HB]  {node['label']} ({node['color_name']}) "
                      f"role={node['role_name']} uptime={node['uptime_s']}s "
                      f"nodes={node['nodes_seen']} age={node['age_s']:.0f}s")

    bridge.on_update(on_update)
    bridge.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n[Bridge] Stopping...")
        bridge.stop()
