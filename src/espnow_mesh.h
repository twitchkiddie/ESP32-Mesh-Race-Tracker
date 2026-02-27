#pragma once

// =============================================================================
// ESP32 Mesh V2 - ESP-NOW Mesh Layer
// Handles init, send, receive, multi-hop relay, and dedup
// =============================================================================

#include <Arduino.h>
#include "config.h"
#include "protocol.h"

// Callback type for when a valid (non-duplicate) packet is received
using MeshReceiveCallback = void (*)(const uint8_t* data, uint8_t len, int rssi);

class EspNowMesh {
public:
    // Per-peer link quality entry (7 bytes when serialised: mac[6] + rssi[1])
    struct PeerRssi {
        uint8_t mac[6];
        int8_t  rssi;
    };
    static const uint8_t PEER_RSSI_MAX = 16; // track up to 16 unique peers (covers staging area)

    bool init();
    void setReceiveCallback(MeshReceiveCallback cb);

    // Send a packet (broadcast)
    bool send(const uint8_t* data, uint8_t len);

    // Send to specific MAC (unicast)
    bool sendTo(const uint8_t* mac, const uint8_t* data, uint8_t len);

    // Get this node's MAC address
    const uint8_t* getMyMac() const { return _myMac; }

    // Get count of unique nodes seen in dedup window
    uint8_t getNodesSeen() const;

    // Get last RSSI value (most recently received packet, any peer)
    int8_t getLastRssi() const { return _lastRssi; }

    // Per-peer RSSI table
    uint8_t getPeerCount() const { return _peerRssiCount; }
    const PeerRssi* getPeerTable() const { return _peerRssi; }

private:
    uint8_t _myMac[6] = {0};
    int8_t  _lastRssi = 0;

    // Per-peer RSSI tracking
    PeerRssi _peerRssi[PEER_RSSI_MAX] = {};
    uint8_t  _peerRssiCount = 0;

    // Dedup ring buffer
    struct DedupeEntry {
        uint8_t  mac[6];
        uint8_t  seq_num;
        uint32_t timestamp_ms;
    };
    DedupeEntry _dedup[MESH_DEDUP_SIZE] = {};
    uint8_t _dedupIdx = 0;

    bool isDuplicate(const uint8_t* mac, uint8_t seq);
    void recordSeen(const uint8_t* mac, uint8_t seq);
    void relayPacket(const uint8_t* data, uint8_t len);

    // Static callback bridge (ESP-NOW requires C function pointer)
    static void onReceiveStatic(const uint8_t* mac, const uint8_t* data, int len);
    static EspNowMesh* _instance;
    static MeshReceiveCallback _userCallback;
};
