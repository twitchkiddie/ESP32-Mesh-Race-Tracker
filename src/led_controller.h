#pragma once

// =============================================================================
// ESP32 Mesh V2 - LED Controller
// NeoPixel device ID color indication and status patterns
// =============================================================================

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include "config.h"

class LedController {
public:
    bool init();

    // Set the active device ID (updates color)
    void setDeviceId(uint8_t id);

    // Turn on with current device color
    void on();

    // Turn off
    void off();

    // Blink the device color N times
    void blink(int count = 1, int onMs = 200, int offMs = 200);

    // Quick flash for TX indication (non-blocking intent, but uses small delay)
    void flashTx();

    // Get the color name for a device ID
    static const char* getColorName(uint8_t id);

    // Get hex color string for dashboard use
    static const char* getColorHex(uint8_t id);

private:
    Adafruit_NeoPixel _pixel;
    uint8_t _deviceId = 0;

    uint32_t getColor(uint8_t id);
};
