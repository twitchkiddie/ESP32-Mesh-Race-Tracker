#pragma once

// =============================================================================
// ESP32 Mesh V2 - SD Card Logger
// CSV logging of full-resolution GPS data for post-race analysis
// =============================================================================

#include <Arduino.h>
#include <SD.h>
#include "config.h"
#include "protocol.h"

class SdLogger {
public:
    // Initialize SD card and create log file
    bool init();

    // Log a GPS reading (called on every valid GPS update, not just broadcasts)
    void logGps(double lat, double lon, double altM, double speedMps,
                double headingDeg, uint8_t sats, double hdop,
                uint16_t battMv, uint8_t battPct, int8_t rssi, uint8_t deviceId);

    // Flush buffered writes to SD
    void flush();

    // Close the log file gracefully
    void close();

    // Is SD card available?
    bool isReady() const { return _ready; }

    // Get current log filename
    const char* getFilename() const { return _filename; }

private:
    bool _ready = false;
    char _filename[64] = {0};
    File _file;
    uint16_t _writesSinceFlush = 0;

    void writeHeader();
};
