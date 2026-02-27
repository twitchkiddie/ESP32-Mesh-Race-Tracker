// =============================================================================
// ESP32 Mesh V2 - ESP-NOW Mesh Layer Implementation
// =============================================================================

#include "espnow_mesh.h"
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>

// Static members
EspNowMesh* EspNowMesh::_instance = nullptr;
MeshReceiveCallback EspNowMesh::_userCallback = nullptr;

// ---------------------------------------------------------------------------
// Promiscuous RSSI capture
// ESP-NOW uses WiFi management Action frames. The promiscuous callback fires
// before the ESP-NOW receive callback, so s_pktRssi holds the RSSI of the
// packet that is currently being handed to onReceiveStatic.
// (WiFi.RSSI() only works when connected to an AP — useless here.)
// ---------------------------------------------------------------------------
static volatile int8_t s_pktRssi = 0;
static void IRAM_ATTR espnowPromiscuousCb(void* buf, wifi_promiscuous_pkt_type_t type) {
    if (type == WIFI_PKT_MGMT) {
        s_pktRssi = ((const wifi_promiscuous_pkt_t*)buf)->rx_ctrl.rssi;
    }
}

bool EspNowMesh::init() {
    _instance = this;

    // Put WiFi in STA mode (required for ESP-NOW)
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    // Set channel
    esp_wifi_set_channel(MESH_CHANNEL, WIFI_SECOND_CHAN_NONE);

    // Enable Long Range mode alongside 11b (LR-only can break ESP-NOW broadcasts)
    esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_LR);

    // Set maximum TX power
    esp_wifi_set_max_tx_power(MESH_TX_POWER);

    // Disable WiFi sleep for consistent performance
    esp_wifi_set_ps(WIFI_PS_NONE);

    // Enable promiscuous mode to capture RSSI from raw packet headers.
    // Must be set before ESP-NOW init.
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_promiscuous_rx_cb(espnowPromiscuousCb);

    // Read our MAC
    esp_read_mac(_myMac, ESP_MAC_WIFI_STA);

    Serial.printf("[MESH] MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  _myMac[0], _myMac[1], _myMac[2],
                  _myMac[3], _myMac[4], _myMac[5]);
    Serial.printf("[MESH] Channel: %d, 11B+LR mode, TX power: %d (quarter-dBm)\n",
                  MESH_CHANNEL, MESH_TX_POWER);

    // Initialize ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("[MESH] ESP-NOW init failed!");
        return false;
    }

    // Register broadcast peer (required for broadcast sends)
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, BROADCAST_MAC, 6);
    peerInfo.channel = MESH_CHANNEL;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("[MESH] Failed to add broadcast peer!");
        return false;
    }

    // Register receive callback
    esp_now_register_recv_cb(onReceiveStatic);

    Serial.println("[MESH] ESP-NOW initialized successfully");
    return true;
}

void EspNowMesh::setReceiveCallback(MeshReceiveCallback cb) {
    _userCallback = cb;
}

bool EspNowMesh::send(const uint8_t* data, uint8_t len) {
    esp_err_t result = esp_now_send(BROADCAST_MAC, data, len);
    if (result != ESP_OK) {
        Serial.printf("[MESH] Send failed: %s\n", esp_err_to_name(result));
        return false;
    }
    return true;
}

bool EspNowMesh::sendTo(const uint8_t* mac, const uint8_t* data, uint8_t len) {
    // Add peer if not already added
    if (!esp_now_is_peer_exist(mac)) {
        esp_now_peer_info_t peerInfo = {};
        memcpy(peerInfo.peer_addr, mac, 6);
        peerInfo.channel = MESH_CHANNEL;
        peerInfo.encrypt = false;
        esp_now_add_peer(&peerInfo);
    }

    esp_err_t result = esp_now_send(mac, data, len);
    if (result != ESP_OK) {
        Serial.printf("[MESH] SendTo failed: %s\n", esp_err_to_name(result));
        return false;
    }
    return true;
}

uint8_t EspNowMesh::getNodesSeen() const {
    // Count unique MACs in the dedup buffer that are still fresh
    uint32_t now = millis();
    uint8_t seen_macs[MESH_DEDUP_SIZE][6];
    uint8_t seen_count = 0;

    for (int i = 0; i < MESH_DEDUP_SIZE; i++) {
        if (_dedup[i].timestamp_ms == 0) continue;
        if (now - _dedup[i].timestamp_ms > MESH_DEDUP_EXPIRY_MS) continue;

        // Check if this MAC is already counted
        bool already = false;
        for (uint8_t j = 0; j < seen_count; j++) {
            if (memcmp(seen_macs[j], _dedup[i].mac, 6) == 0) {
                already = true;
                break;
            }
        }
        if (!already && seen_count < MESH_DEDUP_SIZE) {
            memcpy(seen_macs[seen_count], _dedup[i].mac, 6);
            seen_count++;
        }
    }
    return seen_count;
}

bool EspNowMesh::isDuplicate(const uint8_t* mac, uint8_t seq) {
    uint32_t now = millis();
    for (int i = 0; i < MESH_DEDUP_SIZE; i++) {
        if (_dedup[i].timestamp_ms == 0) continue;
        if (now - _dedup[i].timestamp_ms > MESH_DEDUP_EXPIRY_MS) continue;
        if (memcmp(_dedup[i].mac, mac, 6) == 0 && _dedup[i].seq_num == seq) {
            return true;
        }
    }
    return false;
}

void EspNowMesh::recordSeen(const uint8_t* mac, uint8_t seq) {
    _dedup[_dedupIdx].timestamp_ms = millis();
    _dedup[_dedupIdx].seq_num = seq;
    memcpy(_dedup[_dedupIdx].mac, mac, 6);
    _dedupIdx = (_dedupIdx + 1) % MESH_DEDUP_SIZE;
}

void EspNowMesh::relayPacket(const uint8_t* data, uint8_t len) {
    // Copy data so we can modify hop_count
    uint8_t buf[250];
    if (len > sizeof(buf)) return;
    memcpy(buf, data, len);

    PacketHeader* hdr = (PacketHeader*)buf;
    if (hdr->hop_count >= MESH_MAX_HOPS) return;

    hdr->hop_count++;

    // Small random delay to reduce broadcast collisions
    delayMicroseconds(random(RELAY_DELAY_MIN_US, RELAY_DELAY_MAX_US));

    esp_now_send(BROADCAST_MAC, buf, len);
}

void EspNowMesh::onReceiveStatic(const uint8_t* mac, const uint8_t* data, int len) {
    if (!_instance || len < (int)sizeof(PacketHeader)) return;

    const PacketHeader* hdr = (const PacketHeader*)data;

    // Ignore our own packets
    if (memcmp(hdr->src_mac, _instance->_myMac, 6) == 0) return;

    // Dedup check
    if (_instance->isDuplicate(hdr->src_mac, hdr->seq_num)) return;

    // Record in dedup buffer
    _instance->recordSeen(hdr->src_mac, hdr->seq_num);

    // Capture RSSI from the promiscuous callback (s_pktRssi was set by
    // espnowPromiscuousCb just before this ESP-NOW callback fired).
    _instance->_lastRssi = s_pktRssi;

    // Per-peer RSSI tracking: upsert entry keyed by src_mac
    {
        bool found = false;
        for (uint8_t i = 0; i < _instance->_peerRssiCount; i++) {
            if (memcmp(_instance->_peerRssi[i].mac, hdr->src_mac, 6) == 0) {
                _instance->_peerRssi[i].rssi = s_pktRssi;
                found = true;
                break;
            }
        }
        if (!found && _instance->_peerRssiCount < PEER_RSSI_MAX) {
            memcpy(_instance->_peerRssi[_instance->_peerRssiCount].mac, hdr->src_mac, 6);
            _instance->_peerRssi[_instance->_peerRssiCount].rssi = s_pktRssi;
            _instance->_peerRssiCount++;
        }
    }

    // Deliver to application
    Serial.printf("[MESH] RX type=0x%02X from=%02X:%02X:%02X:%02X:%02X:%02X hop=%d\n",
                  hdr->msg_type,
                  hdr->src_mac[0], hdr->src_mac[1], hdr->src_mac[2],
                  hdr->src_mac[3], hdr->src_mac[4], hdr->src_mac[5],
                  hdr->hop_count);
    if (_userCallback) {
        _userCallback(data, (uint8_t)len, _instance->_lastRssi);
    }

    // Relay to other nodes (if under hop limit)
    _instance->relayPacket(data, (uint8_t)len);
}
