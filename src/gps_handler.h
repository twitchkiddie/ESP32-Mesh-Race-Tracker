#pragma once

// =============================================================================
// ESP32 Mesh V2 - GPS Handler
// TinyGPS++ wrapper with auto-detection and power-save support
// =============================================================================

#include <Arduino.h>
#include <TinyGPS++.h>
#include "config.h"
#include "protocol.h"

class GpsHandler {
public:
    // Initialize GPS serial and detect module presence
    bool init();

    // Call frequently to feed GPS parser
    void update();

    // Was a GPS module detected at boot?
    bool isDetected() const { return _detected; }

    // Is there a valid GPS fix?
    bool hasValidFix() { return _gps.location.isValid() && _gps.location.age() < 5000; }

    // Fill a GPSMessage struct with current data
    void fillGpsMessage(GPSMessage& msg, uint8_t deviceId, uint8_t seqNum,
                        const uint8_t* myMac, uint16_t battMv, uint8_t battPct, int8_t rssi);

    // Get adaptive update interval based on speed
    uint32_t getUpdateIntervalMs();

    // Accessors (TinyGPS++ methods mutate internal 'updated' flag, so not const)
    double getLatitude()   { return _gps.location.lat(); }
    double getLongitude()  { return _gps.location.lng(); }
    double getAltitudeM()  { return _gps.altitude.meters(); }
    double getSpeedMps()   { return _gps.speed.mps(); }
    double getSpeedKmph()  { return _gps.speed.kmph(); }
    double getHeadingDeg() { return _gps.course.deg(); }
    uint32_t getSatellites() { return _gps.satellites.value(); }
    double getHdop()       { return _gps.hdop.hdop(); }
    TinyGPSPlus& raw()    { return _gps; }

    // Print status to serial
    void printStatus();

    // Were RX/TX found to be swapped at boot? (running corrected, but wiring should be fixed)
    bool isSwapped() const { return _swapped; }

private:
    TinyGPSPlus _gps;
    bool     _detected = false;
    bool     _swapped  = false;   // true if module only worked with pins swapped
    uint8_t  _rxPin    = GPS_RX_PIN;
    uint8_t  _txPin    = GPS_TX_PIN;
};
