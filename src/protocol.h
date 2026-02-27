#pragma once

// =============================================================================
// ESP32 Mesh V2 - Binary Protocol Definitions
// All structs are packed for direct ESP-NOW transmission
// =============================================================================

#include <Arduino.h>
#include <cstring>

// --- Message Types ---
enum MsgType : uint8_t {
    MSG_GPS       = 0x01,   // GPS position data
    MSG_HEARTBEAT = 0x02,   // Alive + battery status
    MSG_CMD       = 0x10,   // Command from gateway to node(s)
    MSG_OTA_CTRL  = 0x20,   // OTA control (start/abort/status)
    MSG_OTA_DATA  = 0x21,   // OTA firmware data chunk
    MSG_ACK       = 0x30,   // Acknowledgment
};

// --- Command Types (for MSG_CMD) ---
enum CmdType : uint8_t {
    CMD_SET_DEVICE_ID   = 0x01,
    CMD_SET_INTERVAL    = 0x02,
    CMD_REBOOT          = 0x03,
    CMD_SET_ROLE        = 0x04,
    CMD_SILENT_MODE     = 0x05,
    CMD_ACTIVE_MODE     = 0x06,
    CMD_REQUEST_STATUS  = 0x07,
};

// --- Broadcast MAC (all nodes) ---
static const uint8_t BROADCAST_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// --- Packet Header (10 bytes, prepended to every message) ---
struct __attribute__((packed)) PacketHeader {
    uint8_t  msg_type;      // MsgType enum
    uint8_t  hop_count;     // Incremented on each relay, capped at MESH_MAX_HOPS
    uint8_t  device_id;     // Boat/device identifier (0-30)
    uint8_t  seq_num;       // Per-sender rolling sequence (0-255)
    uint8_t  src_mac[6];    // Original sender MAC address (for dedup)
};
static_assert(sizeof(PacketHeader) == 10, "PacketHeader must be 10 bytes");

// --- GPS Message (30 bytes total) ---
struct __attribute__((packed)) GPSMessage {
    PacketHeader header;        // 10 bytes
    int32_t  lat_e7;            // Latitude  × 1e7 (~1cm precision)
    int32_t  lon_e7;            // Longitude × 1e7
    int16_t  alt_dm;            // Altitude in decimeters (±3276m range)
    uint16_t speed_cmps;        // Speed in cm/s (0-655 m/s range)
    uint16_t heading_cd;        // Heading in centidegrees (0-35999)
    uint8_t  satellites;        // Satellite count
    uint8_t  hdop_tenths;       // HDOP × 10
    uint16_t batt_mv;           // Battery millivolts
    uint8_t  batt_pct;          // Battery percentage (0-100, 255=USB)
    int8_t   rssi;              // Signal strength dBm
};
static_assert(sizeof(GPSMessage) == 30, "GPSMessage must be 30 bytes");

// --- Heartbeat Message (21 bytes total) ---
struct __attribute__((packed)) HeartbeatMessage {
    PacketHeader header;        // 10 bytes
    uint8_t  role;              // NodeRole enum
    uint16_t batt_mv;           // Battery millivolts
    uint8_t  batt_pct;          // Battery percentage (0-100, 255=USB)
    uint8_t  nodes_seen;        // Unique nodes heard recently
    uint32_t uptime_s;          // Seconds since boot
    int8_t   rssi;              // Last RSSI
    uint8_t  fw_patch;          // Firmware patch version (FW_VERSION_PATCH)
};
static_assert(sizeof(HeartbeatMessage) == 21, "HeartbeatMessage must be 21 bytes");

// --- Command Message (33 bytes total) ---
struct __attribute__((packed)) CommandMessage {
    PacketHeader header;        // 10 bytes
    uint8_t  cmd_type;          // CmdType enum
    uint8_t  target_mac[6];     // Target node or BROADCAST_MAC
    uint8_t  payload[16];       // Command-specific data
};
static_assert(sizeof(CommandMessage) == 33, "CommandMessage must be 33 bytes");

// --- OTA Control Message ---
struct __attribute__((packed)) OTAControlMessage {
    PacketHeader header;        // 10 bytes
    uint8_t  action;            // 0=start, 1=abort, 2=status_request
    uint32_t firmware_size;     // Total firmware size in bytes
    uint16_t total_chunks;      // Total number of chunks
    uint8_t  hash[16];          // MD5 hash of firmware
};
static_assert(sizeof(OTAControlMessage) == 33, "OTAControlMessage must be 33 bytes");

// --- OTA Data Chunk Message ---
struct __attribute__((packed)) OTADataMessage {
    PacketHeader header;        // 10 bytes
    uint16_t chunk_index;       // Which chunk this is
    uint8_t  chunk_len;         // Actual data length in this chunk
    uint8_t  data[200];         // Chunk data (200 bytes max to stay under 250)
};
// OTADataMessage is variable size: 10 + 3 + chunk_len

// --- ACK Message ---
struct __attribute__((packed)) AckMessage {
    PacketHeader header;        // 10 bytes
    uint8_t  ack_msg_type;      // What message type this ACKs
    uint8_t  ack_seq_num;       // Sequence number being acknowledged
    uint8_t  status;            // 0=OK, 1=error, 2=busy
};
static_assert(sizeof(AckMessage) == 13, "AckMessage must be 13 bytes");

// --- Gateway Serial Frame ---
// Wire format: [0xAA][0x55][LEN_LO][LEN_HI][PAYLOAD...][CRC_LO][CRC_HI]
// PAYLOAD = the raw binary struct (including PacketHeader)
// CRC16-CCITT over PAYLOAD bytes

namespace SerialProto {
    // CRC16-CCITT (KERMIT variant: init=0x0000, poly=0x1021, no reflect)
    inline uint16_t crc16(const uint8_t* data, size_t len) {
        uint16_t crc = 0x0000;
        for (size_t i = 0; i < len; i++) {
            crc ^= (uint16_t)data[i] << 8;
            for (int j = 0; j < 8; j++) {
                if (crc & 0x8000)
                    crc = (crc << 1) ^ 0x1021;
                else
                    crc <<= 1;
            }
        }
        return crc;
    }
}

// --- Helper: Fill PacketHeader ---
inline void fillHeader(PacketHeader& hdr, MsgType type, uint8_t device_id,
                        uint8_t seq_num, const uint8_t* my_mac) {
    hdr.msg_type = type;
    hdr.hop_count = 0;
    hdr.device_id = device_id;
    hdr.seq_num = seq_num;
    memcpy(hdr.src_mac, my_mac, 6);
}
