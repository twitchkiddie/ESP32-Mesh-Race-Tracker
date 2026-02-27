#pragma once

// =============================================================================
// ESP32 Mesh V2 - Gateway Serial Bridge
// Framed binary serial protocol for laptop/Pi communication
// =============================================================================

#include <Arduino.h>
#include "config.h"
#include "protocol.h"

// Callback when a command frame is received from the laptop
using SerialCommandCallback = void (*)(const uint8_t* data, uint8_t len);

class GatewaySerial {
public:
    bool init();

    // Send a mesh packet to the laptop (wraps in serial frame)
    void sendToLaptop(const uint8_t* data, uint8_t len);

    // Send a gateway status heartbeat.
    // Optional peerData: peerCount entries of {mac[6], rssi[1]} = 7 bytes each,
    // appended after the base 10-byte frame for per-peer RSSI reporting.
    void sendHeartbeat(uint8_t nodeCount, uint32_t uptimeS, uint32_t freeHeap,
                       uint8_t peerCount = 0, const uint8_t* peerData = nullptr);

    // Call in loop() to read incoming commands from laptop
    void update();

    // Register callback for commands received from laptop
    void onCommand(SerialCommandCallback cb) { _cmdCallback = cb; }

private:
    SerialCommandCallback _cmdCallback = nullptr;

    // Receive state machine
    enum RxState { WAIT_SYNC1, WAIT_SYNC2, WAIT_LEN_LO, WAIT_LEN_HI, WAIT_PAYLOAD, WAIT_CRC_LO, WAIT_CRC_HI };
    RxState _rxState = WAIT_SYNC1;
    uint8_t _rxBuf[SERIAL_MAX_FRAME] = {0};
    uint16_t _rxLen = 0;
    uint16_t _rxIdx = 0;
    uint16_t _rxCrc = 0;

    void sendFrame(const uint8_t* payload, uint16_t len);
    void processFrame();
};
