// =============================================================================
// ESP32 Mesh V2 - Device Manager Implementation
// =============================================================================

#include "device_manager.h"
#include <Preferences.h>

static Preferences prefs;

bool DeviceManager::init() {
    pinMode(BOOT_BUTTON_PIN, INPUT_PULLUP);

    prefs.begin(NVS_NAMESPACE, false);

    // Load device ID
    _deviceId = prefs.getUChar(NVS_KEY_DEVICE_ID, 0);
    if (_deviceId > MAX_DEVICE_ID) _deviceId = 0;

    // Load role override (255 = no override, use auto-detect)
    uint8_t savedRole = prefs.getUChar(NVS_KEY_ROLE, 255);
    if (savedRole <= ROLE_GATEWAY) {
        _role = (NodeRole)savedRole;
    }

    Serial.printf("[DEV] Device ID: %d, Saved role: %s\n",
                  _deviceId,
                  savedRole == 255 ? "auto" :
                  savedRole == 0 ? "tracker" :
                  savedRole == 1 ? "relay" : "gateway");
    return true;
}

void DeviceManager::update() {
    bool pressed = (digitalRead(BOOT_BUTTON_PIN) == LOW);
    unsigned long now = millis();

    if (pressed && !_buttonPressed) {
        // Button just pressed
        if (now - _lastDebounceMs > DEBOUNCE_MS) {
            _buttonPressed = true;
            _buttonDownMs = now;
            _lastDebounceMs = now;
        }
    } else if (!pressed && _buttonPressed) {
        // Button released
        _buttonPressed = false;
        unsigned long held = now - _buttonDownMs;

        if (held >= LONG_PRESS_MS) {
            // Long press: cycle device ID
            nextDeviceId();
            Serial.printf("[DEV] Long press → Device ID: %d\n", _deviceId);
        }
        _lastDebounceMs = now;
    }
}

void DeviceManager::setDeviceId(uint8_t id) {
    if (id > MAX_DEVICE_ID) id = 0;
    _deviceId = id;
    saveDeviceId();
    Serial.printf("[DEV] Device ID set to %d\n", _deviceId);
    if (_callback) _callback(_deviceId);
}

void DeviceManager::nextDeviceId() {
    uint8_t next = (_deviceId + 1) % (MAX_DEVICE_ID + 1);
    setDeviceId(next);
}

void DeviceManager::setRole(NodeRole role) {
    _role = role;
    saveRole();
    Serial.printf("[DEV] Role set to %d\n", (int)role);
}

void DeviceManager::saveDeviceId() {
    prefs.putUChar(NVS_KEY_DEVICE_ID, _deviceId);
}

void DeviceManager::saveRole() {
    prefs.putUChar(NVS_KEY_ROLE, (uint8_t)_role);
}
