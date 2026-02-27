// =============================================================================
// ESP32 Mesh V2 - GPS Handler Implementation
// =============================================================================

#include "gps_handler.h"

bool GpsHandler::init() {
    Serial.println("[GPS] Initializing...");
    GPSSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    Serial.printf("[GPS] Serial2 on RX=%d TX=%d @ %d baud\n", GPS_RX_PIN, GPS_TX_PIN, GPS_BAUD);

    // Give GPS module time to power up
    delay(1000);

    // Listen for NMEA sentences to detect GPS module
    unsigned long start = millis();
    int charCount = 0;
    bool foundNMEA = false;
    String buf;

    Serial.println("[GPS] Listening for NMEA data...");
    while (millis() - start < GPS_DETECT_TIMEOUT_MS) {
        while (GPSSerial.available()) {
            char c = GPSSerial.read();
            charCount++;
            _gps.encode(c);

            if (c >= 32 && c <= 126) {
                buf += c;
                if (buf.length() > 200) {
                    buf = buf.substring(buf.length() - 200);
                }
            }
        }
        delay(10);
    }

    // Check for NMEA sentence markers
    if (buf.indexOf("$GP") >= 0 || buf.indexOf("$GN") >= 0 ||
        buf.indexOf("$GL") >= 0 || buf.indexOf("$GPGGA") >= 0 ||
        buf.indexOf("$GPRMC") >= 0) {
        foundNMEA = true;
    }

    if (foundNMEA && charCount > 10) {
        Serial.printf("[GPS] Module detected! %d chars, valid NMEA found\n", charCount);
        _detected = true;
        return true;
    }

    Serial.printf("[GPS] No GPS module detected (%d chars received)\n", charCount);
    _detected = false;
    return false;
}

void GpsHandler::update() {
    while (GPSSerial.available()) {
        _gps.encode(GPSSerial.read());
    }
}

void GpsHandler::fillGpsMessage(GPSMessage& msg, uint8_t deviceId, uint8_t seqNum,
                                 const uint8_t* myMac, uint16_t battMv, uint8_t battPct, int8_t rssi) {
    fillHeader(msg.header, MSG_GPS, deviceId, seqNum, myMac);

    msg.lat_e7      = (int32_t)(_gps.location.lat() * 1e7);
    msg.lon_e7      = (int32_t)(_gps.location.lng() * 1e7);
    msg.alt_dm      = _gps.altitude.isValid() ? (int16_t)(_gps.altitude.meters() * 10.0) : 0;
    msg.speed_cmps  = _gps.speed.isValid()    ? (uint16_t)(_gps.speed.mps() * 100.0)    : 0;
    msg.heading_cd  = _gps.course.isValid()   ? (uint16_t)(_gps.course.deg() * 100.0)   : 0;
    msg.satellites  = (uint8_t)_gps.satellites.value();
    msg.hdop_tenths = _gps.hdop.isValid()     ? (uint8_t)(_gps.hdop.hdop() * 10.0)      : 255;
    msg.batt_mv     = battMv;
    msg.batt_pct    = battPct;
    msg.rssi        = rssi;
}

uint32_t GpsHandler::getUpdateIntervalMs() {
    if (!_gps.speed.isValid()) return GPS_INTERVAL_IDLE_MS;

    float kmph = _gps.speed.kmph();
    if (kmph >= SPEED_THRESHOLD_FAST)   return GPS_INTERVAL_FAST_MS;
    if (kmph >= SPEED_THRESHOLD_MEDIUM) return GPS_INTERVAL_MEDIUM_MS;
    if (kmph >= SPEED_THRESHOLD_SLOW)   return GPS_INTERVAL_SLOW_MS;
    return GPS_INTERVAL_IDLE_MS;
}

void GpsHandler::printStatus() {
    Serial.println("[GPS] Status:");
    Serial.printf("  Detected: %s\n", _detected ? "yes" : "no");
    Serial.printf("  Fix valid: %s\n", hasValidFix() ? "yes" : "no");
    if (hasValidFix()) {
        Serial.printf("  Lat: %.8f  Lon: %.8f\n", _gps.location.lat(), _gps.location.lng());
        Serial.printf("  Alt: %.1f m  Speed: %.1f m/s\n", _gps.altitude.meters(), _gps.speed.mps());
        Serial.printf("  Heading: %.1f deg  Sats: %d  HDOP: %.1f\n",
                      _gps.course.deg(), _gps.satellites.value(),
                      _gps.hdop.isValid() ? _gps.hdop.hdop() : -1.0);
    }
}
