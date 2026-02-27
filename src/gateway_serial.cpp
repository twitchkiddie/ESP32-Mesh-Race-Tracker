// =============================================================================
// ESP32 Mesh V2 - Gateway Serial Bridge Implementation
// =============================================================================

#include "gateway_serial.h"

bool GatewaySerial::init() {
    // Serial is already initialized by Arduino framework at SERIAL_BAUD
    // but we ensure it here
    Serial.begin(SERIAL_BAUD);
    Serial.println("[GW] Gateway serial bridge initialized");
    return true;
}

void GatewaySerial::sendToLaptop(const uint8_t* data, uint8_t len) {
    sendFrame(data, len);
}

void GatewaySerial::sendHeartbeat(uint8_t nodeCount, uint32_t uptimeS, uint32_t freeHeap,
                                   uint8_t peerCount, const uint8_t* peerData) {
    // Base: 10 bytes. Extended: +1 (peer_count) + peerCount*7 (mac+rssi each)
    // Max extended: 10 + 1 + 16*7 = 123 bytes — well within SERIAL_MAX_FRAME
    uint8_t buf[10 + 1 + 16 * 7];
    buf[0] = 0xFF;          // Gateway heartbeat marker
    buf[1] = nodeCount;
    buf[2] = (uptimeS >>  0) & 0xFF;
    buf[3] = (uptimeS >>  8) & 0xFF;
    buf[4] = (uptimeS >> 16) & 0xFF;
    buf[5] = (uptimeS >> 24) & 0xFF;
    buf[6] = (freeHeap >>  0) & 0xFF;
    buf[7] = (freeHeap >>  8) & 0xFF;
    buf[8] = (freeHeap >> 16) & 0xFF;
    buf[9] = (freeHeap >> 24) & 0xFF;

    uint8_t len = 10;
    if (peerCount > 0 && peerData != nullptr) {
        buf[10] = peerCount;                           // peer count byte
        memcpy(buf + 11, peerData, peerCount * 7);    // {mac[6]+rssi[1]} × N
        len = 11 + peerCount * 7;
    }

    sendFrame(buf, len);
}

void GatewaySerial::sendFrame(const uint8_t* payload, uint16_t len) {
    uint16_t crc = SerialProto::crc16(payload, len);

    Serial.write(SERIAL_SYNC_BYTE1);
    Serial.write(SERIAL_SYNC_BYTE2);
    Serial.write((uint8_t)(len & 0xFF));        // LEN low
    Serial.write((uint8_t)((len >> 8) & 0xFF)); // LEN high
    Serial.write(payload, len);
    Serial.write((uint8_t)(crc & 0xFF));        // CRC low
    Serial.write((uint8_t)((crc >> 8) & 0xFF)); // CRC high
}

void GatewaySerial::update() {
    while (Serial.available()) {
        uint8_t b = Serial.read();

        switch (_rxState) {
        case WAIT_SYNC1:
            if (b == SERIAL_SYNC_BYTE1) _rxState = WAIT_SYNC2;
            break;

        case WAIT_SYNC2:
            if (b == SERIAL_SYNC_BYTE2) _rxState = WAIT_LEN_LO;
            else _rxState = WAIT_SYNC1;
            break;

        case WAIT_LEN_LO:
            _rxLen = b;
            _rxState = WAIT_LEN_HI;
            break;

        case WAIT_LEN_HI:
            _rxLen |= ((uint16_t)b << 8);
            if (_rxLen == 0 || _rxLen > SERIAL_MAX_FRAME) {
                _rxState = WAIT_SYNC1; // Invalid length
            } else {
                _rxIdx = 0;
                _rxState = WAIT_PAYLOAD;
            }
            break;

        case WAIT_PAYLOAD:
            _rxBuf[_rxIdx++] = b;
            if (_rxIdx >= _rxLen) {
                _rxState = WAIT_CRC_LO;
            }
            break;

        case WAIT_CRC_LO:
            _rxCrc = b;
            _rxState = WAIT_CRC_HI;
            break;

        case WAIT_CRC_HI:
            _rxCrc |= ((uint16_t)b << 8);
            processFrame();
            _rxState = WAIT_SYNC1;
            break;
        }
    }
}

void GatewaySerial::processFrame() {
    // Verify CRC
    uint16_t expected = SerialProto::crc16(_rxBuf, _rxLen);
    if (_rxCrc != expected) {
        // CRC mismatch — drop frame
        return;
    }

    // Deliver to callback
    if (_cmdCallback) {
        _cmdCallback(_rxBuf, (uint8_t)_rxLen);
    }
}
