// =============================================================================
// ESP32 Mesh V2 - Battery Monitor Implementation
// =============================================================================

#include "battery_monitor.h"

bool BatteryMonitor::init() {
    Serial.println("[BATT] Initializing ADC...");
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);

    uint32_t test = analogRead(BATTERY_ADC_PIN);
    Serial.printf("[BATT] Test reading: %u\n", test);

    if (test == 0) {
        Serial.println("[BATT] WARNING: ADC reads 0 — check wiring");
        return false;
    }

    Serial.println("[BATT] ADC initialized OK");
    return true;
}

float BatteryMonitor::readVoltage() {
    unsigned long now = millis();
    if (now - _lastReadMs < BATTERY_CACHE_MS && _cachedVoltage > 0.0f) {
        return _cachedVoltage;
    }

    uint32_t avg = readAverageADC();
    float pinVoltage = (avg / 4095.0f) * 3.3f;
    float battVoltage = pinVoltage * BATTERY_VOLTAGE_DIVIDER_RATIO;

    _cachedVoltage = battVoltage;
    _lastReadMs = now;
    return battVoltage;
}

uint8_t BatteryMonitor::getPercentage() {
    if (isUSBConnected()) return 255; // Special value = USB powered

    float v = readVoltage();
    int pct = (int)(((v - BATTERY_EMPTY_VOLTAGE) / (BATTERY_FULL_VOLTAGE - BATTERY_EMPTY_VOLTAGE)) * 100.0f);
    if (pct < 0) pct = 0;
    if (pct > 100) pct = 100;
    return (uint8_t)pct;
}

uint16_t BatteryMonitor::getMillivolts() {
    return (uint16_t)(readVoltage() * 1000.0f);
}

bool BatteryMonitor::isUSBConnected() {
    return readVoltage() > BATTERY_USB_THRESHOLD;
}

uint32_t BatteryMonitor::readAverageADC() {
    uint32_t total = 0;
    for (int i = 0; i < BATTERY_ADC_SAMPLES; i++) {
        total += analogRead(BATTERY_ADC_PIN);
    }
    return total / BATTERY_ADC_SAMPLES;
}

void BatteryMonitor::printDiagnostics() {
    uint32_t avg = readAverageADC();
    float pinV = (avg / 4095.0f) * 3.3f;
    float battV = pinV * BATTERY_VOLTAGE_DIVIDER_RATIO;

    Serial.println("[BATT] Diagnostics:");
    Serial.printf("  ADC pin: GPIO%d\n", BATTERY_ADC_PIN);
    Serial.printf("  Raw ADC avg: %u\n", avg);
    Serial.printf("  Pin voltage: %.3f V\n", pinV);
    Serial.printf("  Battery voltage: %.3f V\n", battV);
    Serial.printf("  Percentage: %d%%\n", getPercentage());
    Serial.printf("  USB connected: %s\n", isUSBConnected() ? "yes" : "no");
}
