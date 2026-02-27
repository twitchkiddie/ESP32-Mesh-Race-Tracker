// =============================================================================
// ESP32 Mesh V2 - Power Manager Implementation
// =============================================================================

#include "power_manager.h"
#include <esp_sleep.h>
#include <esp_wifi.h>

void PowerManager::configureForRole(NodeRole role) {
    uint32_t freq;
    switch (role) {
        case ROLE_TRACKER: freq = CPU_FREQ_TRACKER; break;
        case ROLE_RELAY:   freq = CPU_FREQ_RELAY;   break;
        case ROLE_GATEWAY: freq = CPU_FREQ_GATEWAY; break;
        default:           freq = CPU_FREQ_GATEWAY; break;
    }

    setCpuFrequencyMhz(freq);
    Serial.printf("[PWR] CPU frequency set to %d MHz for role %d\n", freq, (int)role);
}

void PowerManager::lightSleep(uint32_t ms) {
    if (ms == 0) return;

    // Configure timer wake source
    esp_sleep_enable_timer_wakeup((uint64_t)ms * 1000ULL); // microseconds

    // Enter light sleep (WiFi state preserved, wakes on timer)
    esp_light_sleep_start();
}

void PowerManager::printInfo() {
    Serial.println("[PWR] Power Info:");
    Serial.printf("  CPU freq: %d MHz\n", getCpuFrequencyMhz());
    Serial.printf("  Uptime: %lu s\n", getUptimeSeconds());
    Serial.printf("  Free heap: %u bytes\n", ESP.getFreeHeap());
    Serial.printf("  Min free heap: %u bytes\n", ESP.getMinFreeHeap());
}
