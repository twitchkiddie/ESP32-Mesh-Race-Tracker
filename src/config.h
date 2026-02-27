#pragma once

// =============================================================================
// ESP32 Mesh V2 - Crew Boat Race Tracker
// Configuration Constants
// =============================================================================

#include <Arduino.h>

// --- Firmware Version ---
#define FW_VERSION_MAJOR 2
#define FW_VERSION_MINOR 0
#define FW_VERSION_PATCH 1
#define FW_VERSION_STRING "2.0.1"

// --- Node Roles ---
enum NodeRole : uint8_t {
    ROLE_TRACKER  = 0,  // Boat GPS tracker
    ROLE_RELAY    = 1,  // Shore relay node
    ROLE_GATEWAY  = 2,  // USB-connected gateway
};

// --- Pin Definitions ---
#define GPS_RX_PIN        16
#define GPS_TX_PIN        17
#define GPS_RESET_PIN     13
#define NEOPIXEL_PIN      25
#define BATTERY_ADC_PIN   34
#define BOOT_BUTTON_PIN   0
#define SD_CS_PIN         5    // SD card chip select (SPI)

// --- GPS Configuration ---
#define GPS_BAUD          9600
#define GPS_DETECT_TIMEOUT_MS  5000   // Time to listen for NMEA on boot
#define GPS_SERIAL_NUM    2           // HardwareSerial number (Serial2)

// --- NeoPixel Configuration ---
#define NUM_PIXELS        1
#define LED_BRIGHTNESS    32

// --- Battery Monitoring ---
#define BATTERY_VOLTAGE_DIVIDER_RATIO  2.0f   // (R1+R2)/R2 for 47k+47k
#define BATTERY_FULL_VOLTAGE           4.2f
#define BATTERY_EMPTY_VOLTAGE          3.3f
#define BATTERY_LOW_VOLTAGE            3.3f
#define BATTERY_USB_THRESHOLD          4.5f
#define BATTERY_ADC_SAMPLES            5
#define BATTERY_CACHE_MS               5000

// --- ESP-NOW Mesh Configuration ---
#define MESH_CHANNEL      6           // WiFi channel (6 = less congested)
#define MESH_MAX_HOPS     5           // Max relay hops (covers ~6km at ~1.2km/hop)
#define MESH_TX_POWER     84          // Quarter-dBm (84 = 21dBm max)
#define MESH_DEDUP_SIZE   512         // Dedup ring buffer entries (512 handles 30 boats @ 1Hz over 10s window)
#define MESH_DEDUP_EXPIRY_MS  10000   // Dedup entry lifetime

// --- Update Intervals ---
#define GPS_INTERVAL_FAST_MS      1000   // Speed >= 10 km/h
#define GPS_INTERVAL_MEDIUM_MS    2000   // Speed 5-10 km/h
#define GPS_INTERVAL_SLOW_MS      5000   // Speed 1-5 km/h
#define GPS_INTERVAL_IDLE_MS     10000   // Speed < 1 km/h
#define HEARTBEAT_INTERVAL_MS    10000   // Heartbeat broadcast
#define GATEWAY_STATUS_INTERVAL_MS 5000  // Gateway serial heartbeat

// --- Speed Thresholds (km/h) ---
#define SPEED_THRESHOLD_FAST    10.0f
#define SPEED_THRESHOLD_MEDIUM   5.0f
#define SPEED_THRESHOLD_SLOW     1.0f

// --- Device ID ---
#define MAX_DEVICE_ID     30
#define NVS_NAMESPACE     "boatmesh"
#define NVS_KEY_DEVICE_ID "dev_id"
#define NVS_KEY_ROLE      "role"
#define DEBOUNCE_MS       50
#define LONG_PRESS_MS     3000   // Hold boot button to cycle ID

// --- Power Management ---
#define CPU_FREQ_TRACKER  80     // MHz - sufficient for GPS + ESP-NOW
#define CPU_FREQ_RELAY    80     // MHz
#define CPU_FREQ_GATEWAY  160    // MHz - needs more for serial throughput
#define RELAY_WAKE_INTERVAL_MS  100   // Light sleep wake window

// --- SD Card Logging ---
#define SD_LOG_FLUSH_INTERVAL  10   // Flush every N writes
#define SD_LOG_DIR         "/logs"

// --- Gateway Serial Protocol ---
#define SERIAL_SYNC_BYTE1  0xAA
#define SERIAL_SYNC_BYTE2  0x55
#define SERIAL_BAUD        115200
#define SERIAL_MAX_FRAME   256

// --- OTA Configuration ---
#define OTA_CHUNK_SIZE     200    // Bytes per ESP-NOW OTA frame (< 250)
#define OTA_TIMEOUT_MS     300000 // 5 minutes
#define OTA_RETRY_COUNT    3

// --- Relay Random Delay ---
#define RELAY_DELAY_MIN_US  500    // Minimum relay delay (microseconds)
#define RELAY_DELAY_MAX_US  5000   // Maximum relay delay (microseconds)

// --- Hardware Serial for GPS ---
#define GPSSerial Serial2
