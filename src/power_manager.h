#pragma once

// =============================================================================
// ESP32 Mesh V2 - Power Manager
// CPU frequency control and light sleep for battery life
// =============================================================================

#include <Arduino.h>
#include "config.h"

class PowerManager {
public:
    // Set CPU frequency based on node role
    void configureForRole(NodeRole role);

    // Enter light sleep for a duration (preserves WiFi/ESP-NOW state)
    void lightSleep(uint32_t ms);

    // Get uptime in seconds
    uint32_t getUptimeSeconds() const { return millis() / 1000; }

    // Print power info
    void printInfo();
};
