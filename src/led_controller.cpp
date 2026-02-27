// =============================================================================
// ESP32 Mesh V2 - LED Controller Implementation
// =============================================================================

#include "led_controller.h"

// Color table: 12 distinct colors, cycling for IDs > 11
struct ColorEntry {
    const char* name;
    const char* hex;
    uint8_t r, g, b;
};

static const ColorEntry COLORS[] = {
    {"Red",     "#FF0000", 255,   0,   0},
    {"Green",   "#00FF00",   0, 255,   0},
    {"Blue",    "#0000FF",   0,   0, 255},
    {"Yellow",  "#FFFF00", 255, 255,   0},
    {"Purple",  "#FF00FF", 255,   0, 255},
    {"Cyan",    "#00FFFF",   0, 255, 255},
    {"Orange",  "#FF8000", 255, 128,   0},
    {"Pink",    "#FF1493", 255,  20, 147},
    {"Lime",    "#32CD32",  50, 205,  50},
    {"Teal",    "#008080",   0, 128, 128},
    {"White",   "#FFFFFF", 255, 255, 255},
    {"Magenta", "#FF0080", 255,   0, 128},
};
static const int NUM_COLORS = sizeof(COLORS) / sizeof(COLORS[0]);

bool LedController::init() {
    _pixel = Adafruit_NeoPixel(NUM_PIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
    _pixel.begin();
    _pixel.setBrightness(LED_BRIGHTNESS);
    _pixel.clear();
    _pixel.show();
    Serial.printf("[LED] NeoPixel on GPIO%d initialized\n", NEOPIXEL_PIN);
    return true;
}

void LedController::setDeviceId(uint8_t id) {
    _deviceId = id;
}

void LedController::on() {
    _pixel.setPixelColor(0, getColor(_deviceId));
    _pixel.show();
}

void LedController::off() {
    _pixel.clear();
    _pixel.show();
}

void LedController::blink(int count, int onMs, int offMs) {
    uint32_t color = getColor(_deviceId);
    for (int i = 0; i < count; i++) {
        _pixel.setPixelColor(0, color);
        _pixel.show();
        delay(onMs);
        _pixel.clear();
        _pixel.show();
        if (i < count - 1) delay(offMs);
    }
}

void LedController::flashTx() {
    _pixel.setPixelColor(0, getColor(_deviceId));
    _pixel.show();
    delay(20);
    _pixel.clear();
    _pixel.show();
}

uint32_t LedController::getColor(uint8_t id) {
    const ColorEntry& c = COLORS[id % NUM_COLORS];
    return _pixel.Color(c.r, c.g, c.b);
}

const char* LedController::getColorName(uint8_t id) {
    return COLORS[id % NUM_COLORS].name;
}

const char* LedController::getColorHex(uint8_t id) {
    return COLORS[id % NUM_COLORS].hex;
}
