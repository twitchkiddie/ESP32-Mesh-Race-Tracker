#pragma once

// =============================================================================
// ESP32 Mesh V2 - Device Manager
// NVS-backed device ID and role persistence, boot button handling
// =============================================================================

#include <Arduino.h>
#include "config.h"

// Callback when device ID changes
using DeviceIdChangeCallback = void (*)(uint8_t newId);

class DeviceManager {
public:
    // Load device ID and role from NVS
    bool init();

    // Call in loop() to poll boot button
    void update();

    // Get/set device ID (0 - MAX_DEVICE_ID)
    uint8_t getDeviceId() const { return _deviceId; }
    void setDeviceId(uint8_t id);
    void nextDeviceId();

    // Get/set node role (persisted to NVS)
    NodeRole getRole() const { return _role; }
    void setRole(NodeRole role);

    // Register callback for ID changes
    void onDeviceIdChange(DeviceIdChangeCallback cb) { _callback = cb; }

private:
    uint8_t  _deviceId = 0;
    NodeRole _role = ROLE_RELAY;  // Default until auto-detected
    DeviceIdChangeCallback _callback = nullptr;

    // Boot button state
    bool _buttonPressed = false;
    unsigned long _buttonDownMs = 0;
    unsigned long _lastDebounceMs = 0;

    void saveDeviceId();
    void saveRole();
};
