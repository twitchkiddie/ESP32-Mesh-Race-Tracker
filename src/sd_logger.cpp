// =============================================================================
// ESP32 Mesh V2 - SD Card Logger Implementation
// =============================================================================

#include "sd_logger.h"
#include <SD.h>
#include <SPI.h>

bool SdLogger::init() {
    Serial.println("[SD] Initializing SD card...");

    if (!SD.begin(SD_CS_PIN)) {
        Serial.println("[SD] No SD card detected — logging disabled");
        _ready = false;
        return false;
    }

    Serial.printf("[SD] Card size: %lluMB\n", SD.cardSize() / (1024 * 1024));

    // Create logs directory if it doesn't exist
    if (!SD.exists(SD_LOG_DIR)) {
        SD.mkdir(SD_LOG_DIR);
    }

    // Generate filename based on boot time (since no RTC, use millis as session ID)
    // Format: /logs/session_XXXXX.csv where XXXXX is a simple counter
    // Once GPS fix is acquired, the race engine on the laptop can rename by date
    uint32_t sessionId = esp_random() % 100000;
    snprintf(_filename, sizeof(_filename), "%s/session_%05u.csv", SD_LOG_DIR, sessionId);

    _file = SD.open(_filename, FILE_WRITE);
    if (!_file) {
        Serial.printf("[SD] Failed to create %s\n", _filename);
        _ready = false;
        return false;
    }

    writeHeader();
    _ready = true;
    Serial.printf("[SD] Logging to %s\n", _filename);
    return true;
}

void SdLogger::writeHeader() {
    _file.println("timestamp_ms,lat,lon,alt_m,speed_mps,heading_deg,satellites,hdop,batt_mv,batt_pct,rssi,device_id");
    _file.flush();
}

void SdLogger::logGps(double lat, double lon, double altM, double speedMps,
                       double headingDeg, uint8_t sats, double hdop,
                       uint16_t battMv, uint8_t battPct, int8_t rssi, uint8_t deviceId) {
    if (!_ready) return;

    _file.printf("%lu,%.8f,%.8f,%.1f,%.2f,%.1f,%u,%.1f,%u,%u,%d,%u\n",
                 millis(), lat, lon, altM, speedMps, headingDeg,
                 sats, hdop, battMv, battPct, rssi, deviceId);

    _writesSinceFlush++;
    if (_writesSinceFlush >= SD_LOG_FLUSH_INTERVAL) {
        flush();
    }
}

void SdLogger::flush() {
    if (!_ready) return;
    _file.flush();
    _writesSinceFlush = 0;
}

void SdLogger::close() {
    if (!_ready) return;
    _file.flush();
    _file.close();
    _ready = false;
    Serial.printf("[SD] Log file closed: %s\n", _filename);
}
