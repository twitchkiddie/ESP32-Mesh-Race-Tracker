#pragma once

// =============================================================================
// ESP32 Mesh V2 - OTA Handler
// Receives firmware chunks via ESP-NOW mesh, writes to OTA partition,
// verifies MD5, and reboots.
// =============================================================================

#include <Arduino.h>
#include "config.h"
#include "protocol.h"

class OtaHandler {
public:
    enum State : uint8_t { IDLE, RECEIVING, DONE, FAILED };

    // Call in setup() with our MAC and device ID
    void init(const uint8_t* myMac, uint8_t deviceId);

    // Process incoming OTA frames (call from mesh receive callback)
    void handleOtaCtrl(const uint8_t* data, uint8_t len);
    void handleOtaData(const uint8_t* data, uint8_t len);

    // Call every iteration of loopTracker() — writes the pending chunk on Core 1
    // (Update.write/esp_ota_write must NOT run inside the WiFi callback on Core 0)
    void loop();

    State    getState()       const { return _state; }
    bool     isActive()       const { return _state == RECEIVING; }
    uint16_t getProgress()    const { return _nextChunk; }
    uint16_t getTotalChunks() const { return _totalChunks; }

    // Provide a function that sends an ESP-NOW packet (use mesh.send)
    using SendFn = bool (*)(const uint8_t* data, uint8_t len);
    void onSend(SendFn fn) { _sendFn = fn; }

private:
    State    _state        = IDLE;
    uint8_t  _myMac[6]    = {};
    uint8_t  _deviceId    = 0;
    uint32_t _fwSize      = 0;
    uint16_t _totalChunks = 0;
    uint16_t _nextChunk   = 0;
    uint8_t  _md5[16]     = {};
    uint32_t _startMs     = 0;
    SendFn   _sendFn      = nullptr;

    void sendAck(uint8_t seqNum, uint8_t status);
    void abort(const char* reason);

    // Pending chunk: written by WiFi callback (Core 0), consumed by loop() (Core 1).
    // Safe without mutex: stop-and-wait guarantees no ACK is sent until loop()
    // processes the chunk, so no second chunk can arrive while one is pending.
    volatile bool _chunkPending = false;
    uint16_t      _pendingIndex  = 0;
    uint8_t       _pendingSeqNum = 0;
    uint8_t       _pendingData[OTA_CHUNK_SIZE];
    uint8_t       _pendingLen    = 0;
};
