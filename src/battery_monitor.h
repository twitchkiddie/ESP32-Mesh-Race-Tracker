#pragma once

// =============================================================================
// ESP32 Mesh V2 - Battery Monitor
// ADC-based battery voltage measurement with voltage divider
// =============================================================================

#include <Arduino.h>
#include "config.h"

class BatteryMonitor {
public:
    bool init();

    // Read battery voltage (cached for BATTERY_CACHE_MS)
    float readVoltage();

    // Get battery percentage (0-100), or 255 if USB powered
    uint8_t getPercentage();

    // Get millivolts (for protocol struct)
    uint16_t getMillivolts();

    // Is USB power detected?
    bool isUSBConnected();

    void printDiagnostics();

private:
    float _cachedVoltage = 0.0f;
    unsigned long _lastReadMs = 0;

    uint32_t readAverageADC();
};
