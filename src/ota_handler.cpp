// =============================================================================
// ESP32 Mesh V2 - OTA Handler Implementation
// Uses Arduino Update library to write firmware to the OTA partition.
// =============================================================================

#include "ota_handler.h"
#include <Update.h>

void OtaHandler::init(const uint8_t* myMac, uint8_t deviceId) {
    memcpy(_myMac, myMac, 6);
    _deviceId = deviceId;
    _state = IDLE;
}

// ---------------------------------------------------------------------------
// OTA Control (start / abort)
// ---------------------------------------------------------------------------

void OtaHandler::handleOtaCtrl(const uint8_t* data, uint8_t len) {
    if (len < (int)sizeof(OTAControlMessage)) return;
    const OTAControlMessage* msg = (const OTAControlMessage*)data;

    if (msg->action == 1) {   // Abort
        abort("remote abort");
        sendAck(msg->header.seq_num, 2); // status=2: aborted
        return;
    }

    if (msg->action != 0) return; // Unknown action

    // --- Start ---
    if (_state == RECEIVING) {
        Update.abort();
    }

    _fwSize      = msg->firmware_size;
    _totalChunks = msg->total_chunks;
    _nextChunk   = 0;
    _startMs     = millis();
    memcpy(_md5, msg->hash, 16);

    if (_fwSize == 0 || _totalChunks == 0) {
        Serial.println("[OTA] Invalid start params");
        sendAck(msg->header.seq_num, 1);
        _state = FAILED;
        return;
    }

    if (!Update.begin(_fwSize)) {
        Serial.printf("[OTA] begin() failed: %s\n", Update.errorString());
        sendAck(msg->header.seq_num, 1); // error
        _state = FAILED;
        return;
    }

    // Set the expected MD5 — Update library will verify on end()
    char md5str[33];
    for (int i = 0; i < 16; i++) sprintf(md5str + i * 2, "%02x", _md5[i]);
    md5str[32] = '\0';
    Update.setMD5(md5str);

    _state = RECEIVING;
    Serial.printf("[OTA] Started: %lu bytes, %u chunks, MD5=%s\n",
                  (unsigned long)_fwSize, _totalChunks, md5str);
    sendAck(msg->header.seq_num, 0); // OK
}

// ---------------------------------------------------------------------------
// OTA Data chunk
// ---------------------------------------------------------------------------

void OtaHandler::handleOtaData(const uint8_t* data, uint8_t len) {
    if (_state != RECEIVING) return;
    if (len < (int)(sizeof(PacketHeader) + 3)) return;

    const PacketHeader* hdr    = (const PacketHeader*)data;
    uint16_t chunk_index;
    uint8_t  chunk_len;
    memcpy(&chunk_index, data + sizeof(PacketHeader), 2);
    chunk_len = data[sizeof(PacketHeader) + 2];

    if (len < (int)(sizeof(PacketHeader) + 3 + chunk_len)) return;
    const uint8_t* chunk_data = data + sizeof(PacketHeader) + 3;

    // Timeout check
    if (millis() - _startMs > OTA_TIMEOUT_MS) {
        abort("timeout");
        return;
    }

    // Already have this chunk (duplicate) — re-ACK so sender can advance
    if (chunk_index < _nextChunk) {
        sendAck(hdr->seq_num, 0);
        return;
    }

    // Future chunk we're not ready for (shouldn't happen in stop-and-wait)
    if (chunk_index > _nextChunk) {
        return;
    }

    // Queue the chunk for loop() on Core 1.
    // Update.write() (esp_ota_write) must NOT be called from the WiFi receive
    // callback (Core 0): the WiFi driver may hold the flash SPI mutex, causing
    // a deadlock and write failure that aborts the OTA.
    if (_chunkPending) {
        // Previous chunk still being written — drop; laptop will retry with new seq.
        return;
    }
    memcpy(_pendingData, chunk_data, chunk_len);
    _pendingIndex  = chunk_index;
    _pendingSeqNum = hdr->seq_num;
    _pendingLen    = chunk_len;
    _chunkPending  = true;
    // ACK is sent from loop() after the write completes.
}

// ---------------------------------------------------------------------------
// Main-loop processing: write pending chunk on Core 1
// ---------------------------------------------------------------------------

void OtaHandler::loop() {
    if (!_chunkPending || _state != RECEIVING) return;

    // Timeout check
    if (millis() - _startMs > OTA_TIMEOUT_MS) {
        abort("timeout");
        _chunkPending = false;
        return;
    }

    // Write to flash from Core 1 — safe here
    size_t written = Update.write(_pendingData, _pendingLen);
    if (written != _pendingLen) {
        Serial.printf("[OTA] Write error chunk %u: %s\n", _pendingIndex, Update.errorString());
        abort("write error");
        sendAck(_pendingSeqNum, 1);
        _chunkPending = false;
        return;
    }

    _nextChunk++;
    sendAck(_pendingSeqNum, 0); // OK
    _chunkPending = false;

    if (_nextChunk % 100 == 0 || _nextChunk == _totalChunks) {
        float pct = (100.0f * _nextChunk) / _totalChunks;
        Serial.printf("[OTA] Progress: %u/%u (%.0f%%)\n", _nextChunk, _totalChunks, pct);
    }

    // All chunks received?
    if (_nextChunk >= _totalChunks) {
        if (!Update.end(true)) {
            Serial.printf("[OTA] Verify failed: %s\n", Update.errorString());
            _state = FAILED;
            return;
        }
        Serial.println("[OTA] Complete! Rebooting in 2s...");
        _state = DONE;
        delay(2000);
        ESP.restart();
    }
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

void OtaHandler::sendAck(uint8_t seqNum, uint8_t status) {
    if (!_sendFn) return;
    AckMessage ack;
    fillHeader(ack.header, MSG_ACK, _deviceId, seqNum, _myMac);
    ack.ack_msg_type = MSG_OTA_DATA;
    ack.ack_seq_num  = seqNum;
    ack.status       = status;
    _sendFn((const uint8_t*)&ack, sizeof(ack));
}

void OtaHandler::abort(const char* reason) {
    Serial.printf("[OTA] Aborted: %s\n", reason);
    if (_state == RECEIVING) Update.abort();
    _state = IDLE;
    _nextChunk = 0;
}
