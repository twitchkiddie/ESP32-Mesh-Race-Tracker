// =============================================================================
// ESP32 Mesh V2 - Crew Boat Race Tracker
// Main entry point: role detection, initialization, and task orchestration
// =============================================================================

#include <Arduino.h>
#include <Preferences.h>
#include "config.h"
#include "protocol.h"
#include "espnow_mesh.h"
#include "gps_handler.h"
#include "battery_monitor.h"
#include "led_controller.h"
#include "device_manager.h"
#include "power_manager.h"
#include "sd_logger.h"
#include "gateway_serial.h"
#include "ota_handler.h"

// --- Global instances ---
static EspNowMesh    mesh;
static GpsHandler    gps;
static BatteryMonitor battery;
static LedController  led;
static DeviceManager  device;
static PowerManager   power;
static SdLogger       sdLog;
static GatewaySerial  gwSerial;
static OtaHandler     ota;

// --- State ---
static NodeRole  currentRole = ROLE_RELAY;
static uint8_t   seqNum = 0;        // Rolling sequence number for outgoing packets
static unsigned long lastGpsSendMs = 0;
static unsigned long lastHeartbeatMs = 0;
static unsigned long lastGatewayStatusMs = 0;

// =============================================================================
// Callbacks
// =============================================================================

// Called when a non-duplicate mesh packet arrives
static void onMeshReceive(const uint8_t* data, uint8_t len, int rssi) {
    if (len < sizeof(PacketHeader)) return;
    const PacketHeader* hdr = (const PacketHeader*)data;

    switch (currentRole) {
    case ROLE_GATEWAY:
        // Forward everything to laptop via serial
        Serial.printf("[RX] type=0x%02X from=%02X:%02X:%02X:%02X:%02X:%02X hop=%d seq=%d\n",
                      hdr->msg_type,
                      hdr->src_mac[0], hdr->src_mac[1], hdr->src_mac[2],
                      hdr->src_mac[3], hdr->src_mac[4], hdr->src_mac[5],
                      hdr->hop_count, hdr->seq_num);
        gwSerial.sendToLaptop(data, len);
        break;

    case ROLE_TRACKER:
        // OTA messages
        if (hdr->msg_type == MSG_OTA_CTRL) {
            ota.handleOtaCtrl(data, len);
            break;
        }
        if (hdr->msg_type == MSG_OTA_DATA) {
            ota.handleOtaData(data, len);
            break;
        }
        // Commands directed at us or broadcast
        if (hdr->msg_type == MSG_CMD && len >= sizeof(CommandMessage)) {
            const CommandMessage* cmd = (const CommandMessage*)data;
            if (memcmp(cmd->target_mac, mesh.getMyMac(), 6) == 0 ||
                memcmp(cmd->target_mac, BROADCAST_MAC, 6) == 0) {
                switch (cmd->cmd_type) {
                case CMD_SET_DEVICE_ID:
                    device.setDeviceId(cmd->payload[0]);
                    led.setDeviceId(cmd->payload[0]);
                    led.blink(3);
                    break;
                case CMD_REBOOT:
                    Serial.println("[CMD] Reboot requested");
                    delay(100);
                    ESP.restart();
                    break;
                case CMD_SET_INTERVAL:
                    break;
                default:
                    break;
                }
            }
        }
        break;

    case ROLE_RELAY:
        // Relays just relay (handled by mesh layer) — process commands too
        if (hdr->msg_type == MSG_CMD && len >= sizeof(CommandMessage)) {
            const CommandMessage* cmd = (const CommandMessage*)data;
            if (memcmp(cmd->target_mac, mesh.getMyMac(), 6) == 0 ||
                memcmp(cmd->target_mac, BROADCAST_MAC, 6) == 0) {
                if (cmd->cmd_type == CMD_REBOOT) {
                    ESP.restart();
                }
            }
        }
        break;
    }
}

// Called when device ID changes via boot button
static void onDeviceIdChange(uint8_t newId) {
    led.setDeviceId(newId);
    led.blink(3);
    Serial.printf("[MAIN] Device ID changed to %d (%s)\n",
                  newId, LedController::getColorName(newId));
}

// Called when gateway receives a command from the laptop
// Substitutes the gateway's own MAC as src_mac before forwarding —
// the laptop has no mesh MAC, and dedup uses src_mac.
static void onSerialCommand(const uint8_t* data, uint8_t len) {
    if (len < (int)sizeof(PacketHeader)) return;
    const PacketHeader* hdr = (const PacketHeader*)data;

    switch (hdr->msg_type) {
    case MSG_CMD:
    case MSG_OTA_CTRL:
    case MSG_OTA_DATA: {
        // Copy the frame, fix up the src_mac to use our own MAC
        uint8_t buf[250];
        if (len > sizeof(buf)) return;
        memcpy(buf, data, len);
        PacketHeader* outHdr = (PacketHeader*)buf;
        memcpy(outHdr->src_mac, mesh.getMyMac(), 6);
        outHdr->hop_count = 0;
        mesh.send(buf, len);
        if (hdr->msg_type == MSG_CMD) {
            Serial.println("[GW] Forwarded command to mesh");
        } else {
            Serial.printf("[GW] Forwarded OTA 0x%02X chunk/seq=%d\n",
                          hdr->msg_type, hdr->seq_num);
        }
        break;
    }
    default:
        break;
    }
}

// =============================================================================
// Role Detection
// =============================================================================

static NodeRole detectRole() {
    // Role detection priority — same firmware binary on every device:
    //
    //  1. GPS module found on Serial2  →  TRACKER   (hardware tells us)
    //  2. NVS explicitly set GATEWAY   →  GATEWAY   (set once on the gateway unit, persists forever)
    //  3. Everything else              →  RELAY     (zero config, default)
    //
    // Trackers and relays are plug-and-play with no configuration.
    // Only the gateway unit needs one-time setup: open serial monitor and type "role gateway".
    // That setting survives all future firmware flashes (NVS is separate flash storage).

    // 1. GPS detected → TRACKER
    if (gps.init()) {
        Serial.println("[ROLE] GPS detected → TRACKER");
        return ROLE_TRACKER;
    }

    // 2. Explicitly configured as gateway via NVS → GATEWAY
    //    USB power alone is NOT used — a tracker on a USB power bank or bench power
    //    looks the same as a gateway on USB, so we require explicit configuration.
    Preferences p;
    p.begin(NVS_NAMESPACE, true);
    uint8_t savedRole = p.getUChar(NVS_KEY_ROLE, 255);
    p.end();
    if (savedRole == ROLE_GATEWAY) {
        Serial.println("[ROLE] NVS configured → GATEWAY");
        return ROLE_GATEWAY;
    }

    // 3. No GPS, not configured as gateway → RELAY
    Serial.println("[ROLE] No GPS, not gateway → RELAY");
    return ROLE_RELAY;
}

// =============================================================================
// Role-Specific Loop Functions
// =============================================================================

static void loopTracker() {
    // 0. Process any pending OTA chunk on Core 1 (safe flash write context)
    ota.loop();

    // 1. Update GPS parser
    gps.update();

    // 2. Update device manager (boot button)
    device.update();

    // 3. Check if it's time to broadcast GPS
    unsigned long now = millis();
    uint32_t interval = gps.getUpdateIntervalMs();

    if (now - lastGpsSendMs >= interval) {
        if (gps.hasValidFix()) {
            // Build GPS message
            GPSMessage msg;
            gps.fillGpsMessage(msg, device.getDeviceId(), seqNum++,
                               mesh.getMyMac(),
                               battery.getMillivolts(), battery.getPercentage(),
                               mesh.getLastRssi());

            // Broadcast over mesh
            bool ok = mesh.send((const uint8_t*)&msg, sizeof(msg));
            Serial.printf("[TX] GPS seq=%d ok=%d lat=%.6f lon=%.6f\n",
                          msg.header.seq_num, ok, gps.getLatitude(), gps.getLongitude());
            led.flashTx();

            // Log to SD (full resolution — every fix, not just broadcasts)
            sdLog.logGps(gps.getLatitude(), gps.getLongitude(),
                         gps.getAltitudeM(), gps.getSpeedMps(),
                         gps.getHeadingDeg(), gps.getSatellites(),
                         gps.getHdop(),
                         battery.getMillivolts(), battery.getPercentage(),
                         mesh.getLastRssi(), device.getDeviceId());
        }
        lastGpsSendMs = now;
    }

    // 4. Send heartbeat periodically (even if no GPS fix yet)
    if (now - lastHeartbeatMs >= HEARTBEAT_INTERVAL_MS) {
        HeartbeatMessage hb;
        fillHeader(hb.header, MSG_HEARTBEAT, device.getDeviceId(), seqNum++, mesh.getMyMac());
        hb.role = ROLE_TRACKER;
        hb.batt_mv = battery.getMillivolts();
        hb.batt_pct = battery.getPercentage();
        hb.nodes_seen = mesh.getNodesSeen();
        hb.uptime_s = power.getUptimeSeconds();
        hb.rssi = mesh.getLastRssi();
        hb.fw_patch = FW_VERSION_PATCH;

        // Append per-peer RSSI table after the fixed struct
        uint8_t peerCount = mesh.getPeerCount();
        const EspNowMesh::PeerRssi* peers = mesh.getPeerTable();
        uint8_t pktBuf[sizeof(HeartbeatMessage) + EspNowMesh::PEER_RSSI_MAX * 7];
        memcpy(pktBuf, &hb, sizeof(hb));
        for (uint8_t i = 0; i < peerCount; i++) {
            uint8_t* p = pktBuf + sizeof(HeartbeatMessage) + i * 7;
            memcpy(p, peers[i].mac, 6);
            memcpy(p + 6, &peers[i].rssi, 1);
        }
        uint8_t pktLen = (uint8_t)(sizeof(HeartbeatMessage) + peerCount * 7);
        bool ok = mesh.send(pktBuf, pktLen);
        Serial.printf("[TX] Heartbeat seq=%d ok=%d nodes=%d peers=%d\n",
                      hb.header.seq_num, ok, hb.nodes_seen, peerCount);
        lastHeartbeatMs = now;
    }

    // 5. Delay instead of light sleep (light sleep disables WiFi radio)
    delay(10);
}

static void loopRelay() {
    // Relays are passive — ESP-NOW receive callback handles relay automatically
    // Just need to:
    // 1. Update device manager (boot button)
    device.update();

    // 2. Send heartbeat
    unsigned long now = millis();
    if (now - lastHeartbeatMs >= HEARTBEAT_INTERVAL_MS) {
        HeartbeatMessage hb;
        fillHeader(hb.header, MSG_HEARTBEAT, device.getDeviceId(), seqNum++, mesh.getMyMac());
        hb.role = ROLE_RELAY;
        hb.batt_mv = battery.getMillivolts();
        hb.batt_pct = battery.getPercentage();
        hb.nodes_seen = mesh.getNodesSeen();
        hb.uptime_s = power.getUptimeSeconds();
        hb.rssi = mesh.getLastRssi();
        hb.fw_patch = FW_VERSION_PATCH;

        // Append per-peer RSSI table after the fixed struct
        uint8_t peerCount = mesh.getPeerCount();
        const EspNowMesh::PeerRssi* peers = mesh.getPeerTable();
        uint8_t pktBuf[sizeof(HeartbeatMessage) + EspNowMesh::PEER_RSSI_MAX * 7];
        memcpy(pktBuf, &hb, sizeof(hb));
        for (uint8_t i = 0; i < peerCount; i++) {
            uint8_t* p = pktBuf + sizeof(HeartbeatMessage) + i * 7;
            memcpy(p, peers[i].mac, 6);
            memcpy(p + 6, &peers[i].rssi, 1);
        }
        uint8_t pktLen = (uint8_t)(sizeof(HeartbeatMessage) + peerCount * 7);
        bool ok = mesh.send(pktBuf, pktLen);
        Serial.printf("[TX] Relay heartbeat seq=%d ok=%d nodes=%d peers=%d\n",
                      hb.header.seq_num, ok, hb.nodes_seen, peerCount);
        lastHeartbeatMs = now;
    }

    // 3. Delay instead of light sleep (light sleep disables WiFi radio)
    delay(10);
}

static void loopGateway() {
    // Gateway: receive from mesh (callback), forward to laptop via serial
    // Also: receive commands from laptop, forward to mesh

    // 1. Process incoming serial commands from laptop
    gwSerial.update();

    // 2. Update device manager
    device.update();

    // 3. Send gateway status heartbeat to laptop (includes per-peer RSSI)
    unsigned long now = millis();
    if (now - lastGatewayStatusMs >= GATEWAY_STATUS_INTERVAL_MS) {
        uint8_t peerCount = mesh.getPeerCount();
        const EspNowMesh::PeerRssi* peers = mesh.getPeerTable();
        uint8_t peerBuf[EspNowMesh::PEER_RSSI_MAX * 7];
        for (uint8_t i = 0; i < peerCount; i++) {
            uint8_t* p = peerBuf + i * 7;
            memcpy(p, peers[i].mac, 6);
            memcpy(p + 6, &peers[i].rssi, 1);
        }
        gwSerial.sendHeartbeat(mesh.getNodesSeen(), power.getUptimeSeconds(), ESP.getFreeHeap(),
                               peerCount, peerBuf);
        lastGatewayStatusMs = now;
    }

    // 4. Send mesh heartbeat
    if (now - lastHeartbeatMs >= HEARTBEAT_INTERVAL_MS) {
        HeartbeatMessage hb;
        fillHeader(hb.header, MSG_HEARTBEAT, device.getDeviceId(), seqNum++, mesh.getMyMac());
        hb.role = ROLE_GATEWAY;
        hb.batt_mv = battery.getMillivolts();
        hb.batt_pct = battery.getPercentage();
        hb.nodes_seen = mesh.getNodesSeen();
        hb.uptime_s = power.getUptimeSeconds();
        hb.rssi = mesh.getLastRssi();
        hb.fw_patch = FW_VERSION_PATCH;

        // Append per-peer RSSI table after the fixed struct
        uint8_t peerCount = mesh.getPeerCount();
        const EspNowMesh::PeerRssi* peers = mesh.getPeerTable();
        uint8_t pktBuf[sizeof(HeartbeatMessage) + EspNowMesh::PEER_RSSI_MAX * 7];
        memcpy(pktBuf, &hb, sizeof(hb));
        for (uint8_t i = 0; i < peerCount; i++) {
            uint8_t* p = pktBuf + sizeof(HeartbeatMessage) + i * 7;
            memcpy(p, peers[i].mac, 6);
            memcpy(p + 6, &peers[i].rssi, 1);
        }
        mesh.send(pktBuf, (uint8_t)(sizeof(HeartbeatMessage) + peerCount * 7));
        lastHeartbeatMs = now;
    }

    // No sleep — gateway needs to be responsive
    delay(1);
}

// =============================================================================
// Serial Command Handler (for direct serial debugging, all roles)
// =============================================================================

static void handleSerialInput() {
    if (!Serial.available()) return;

    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.length() == 0) return;

    if (cmd == "status" || cmd == "info") {
        Serial.println("=== ESP32 Mesh V2 ===");
        Serial.printf("  Firmware: %s\n", FW_VERSION_STRING);
        Serial.printf("  Role: %s\n",
                      currentRole == ROLE_TRACKER ? "TRACKER" :
                      currentRole == ROLE_RELAY   ? "RELAY"   : "GATEWAY");
        Serial.printf("  Device ID: %d (%s)\n",
                      device.getDeviceId(),
                      LedController::getColorName(device.getDeviceId()));
        Serial.printf("  MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                      mesh.getMyMac()[0], mesh.getMyMac()[1], mesh.getMyMac()[2],
                      mesh.getMyMac()[3], mesh.getMyMac()[4], mesh.getMyMac()[5]);
        Serial.printf("  Nodes seen: %d\n", mesh.getNodesSeen());
        power.printInfo();
    }
    else if (cmd == "battery") {
        battery.printDiagnostics();
    }
    else if (cmd == "gps") {
        gps.printStatus();
    }
    else if (cmd == "sd") {
        Serial.printf("[SD] Ready: %s, File: %s\n",
                      sdLog.isReady() ? "yes" : "no",
                      sdLog.isReady() ? sdLog.getFilename() : "none");
    }
    else if (cmd.startsWith("id ")) {
        int id = cmd.substring(3).toInt();
        device.setDeviceId((uint8_t)id);
        led.setDeviceId((uint8_t)id);
        led.blink(3);
    }
    else if (cmd == "id next") {
        device.nextDeviceId();
        led.setDeviceId(device.getDeviceId());
        led.blink(3);
    }
    else if (cmd == "role") {
        const char* roleName = currentRole == ROLE_TRACKER ? "TRACKER" :
                               currentRole == ROLE_RELAY   ? "RELAY"   : "GATEWAY";
        Serial.printf("[ROLE] Current: %s\n", roleName);
    }
    else if (cmd == "role gateway now") {
        // Activate gateway mode immediately — RAM only, no NVS write, no reboot.
        // The server sends this command automatically on every connect, so any
        // device plugged into the server's USB port becomes a gateway for the
        // duration of that session. Rebooting reverts to auto-detect.
        // Only switches from RELAY; TRACKER (GPS present) is never overridden.
        if (currentRole == ROLE_RELAY) {
            currentRole = ROLE_GATEWAY;
            gwSerial.init();
            gwSerial.onCommand(onSerialCommand);
            power.configureForRole(ROLE_GATEWAY);
            Serial.println("[ROLE] GATEWAY active (session only — reboot to revert)");
        } else {
            const char* roleName = currentRole == ROLE_TRACKER ? "TRACKER" : "GATEWAY";
            Serial.printf("[ROLE] Already %s — no change\n", roleName);
        }
    }
    else if (cmd == "role gateway") {
        // Persist GATEWAY role to NVS — survives all future reflashes.
        // Use this only if you need the device to auto-start as gateway without
        // the server's connect-time activation (e.g. standalone serial monitor use).
        Preferences p;
        p.begin(NVS_NAMESPACE, false);
        p.putUChar(NVS_KEY_ROLE, ROLE_GATEWAY);
        p.end();
        Serial.println("[ROLE] GATEWAY saved to NVS. Reboot to apply.");
    }
    else if (cmd == "role relay") {
        // Force RELAY in NVS — useful to pin a device as relay even if GPS appears.
        Preferences p;
        p.begin(NVS_NAMESPACE, false);
        p.putUChar(NVS_KEY_ROLE, ROLE_RELAY);
        p.end();
        Serial.println("[ROLE] RELAY saved to NVS. Reboot to apply.");
    }
    else if (cmd == "role tracker") {
        // Force TRACKER in NVS — clears any relay/gateway override.
        // Hardware GPS detection still takes priority on boot.
        Preferences p;
        p.begin(NVS_NAMESPACE, false);
        p.putUChar(NVS_KEY_ROLE, ROLE_TRACKER);
        p.end();
        Serial.println("[ROLE] TRACKER saved to NVS. Reboot to apply.");
    }
    else if (cmd == "role auto") {
        // Clear NVS role — returns to auto-detection (GPS=tracker, else=relay).
        Preferences p;
        p.begin(NVS_NAMESPACE, false);
        p.putUChar(NVS_KEY_ROLE, 255);
        p.end();
        Serial.println("[ROLE] NVS cleared. Reboot to auto-detect.");
    }
    else if (cmd == "reboot") {
        Serial.println("Rebooting...");
        delay(100);
        ESP.restart();
    }
    else if (cmd == "led on") {
        led.on();
    }
    else if (cmd == "led off") {
        led.off();
    }
    else if (cmd == "led blink") {
        led.blink(5);
    }
    else if (cmd == "help") {
        Serial.println("Commands: status, battery, gps, sd, id <n>, id next,");
        Serial.println("  role, role gateway now (session), role gateway/relay/tracker (NVS), role auto,");
        Serial.println("  reboot, led on|off|blink, help");
    }
    else {
        Serial.printf("Unknown command: %s (type 'help')\n", cmd.c_str());
    }
}

// =============================================================================
// Arduino Setup & Loop
// =============================================================================

void setup() {
    Serial.begin(SERIAL_BAUD);
    delay(500);
    Serial.println();
    Serial.println("========================================");
    Serial.printf("  ESP32 Mesh V2 - %s\n", FW_VERSION_STRING);
    Serial.println("  Crew Boat Race Tracker");
    Serial.println("========================================");

    // 1. Initialize device manager (loads ID and role from NVS)
    device.init();
    device.onDeviceIdChange(onDeviceIdChange);

    // 2. Initialize LED
    led.init();
    led.setDeviceId(device.getDeviceId());
    led.blink(2); // Boot indicator

    // 3. Initialize battery monitor
    battery.init();

    // 4. Detect role from hardware (GPS = tracker, USB = gateway, else = relay)
    //    No NVS override — identical firmware on every device, hardware decides.
    currentRole = detectRole();

    // 5. Configure power for detected role
    power.configureForRole(currentRole);

    // 6. Initialize ESP-NOW mesh
    if (!mesh.init()) {
        Serial.println("[FATAL] ESP-NOW init failed!");
        led.blink(10, 100, 100); // Error pattern
        ESP.restart();
    }
    mesh.setReceiveCallback(onMeshReceive);

    // 7. Role-specific initialization
    switch (currentRole) {
    case ROLE_TRACKER:
        // Initialize OTA handler
        ota.init(mesh.getMyMac(), device.getDeviceId());
        ota.onSend([](const uint8_t* d, uint8_t l) { return mesh.send(d, l); });
        // Initialize SD card for logging
        sdLog.init(); // Non-fatal if SD not present
        Serial.println("[MAIN] Tracker mode — broadcasting GPS");
        break;

    case ROLE_RELAY:
        Serial.println("[MAIN] Relay mode — forwarding packets");
        break;

    case ROLE_GATEWAY:
        gwSerial.init();
        gwSerial.onCommand(onSerialCommand);
        Serial.println("[MAIN] Gateway mode — bridging mesh ↔ serial");
        break;
    }

    // 8. Show startup summary
    Serial.println("----------------------------------------");
    Serial.printf("  Role:      %s\n",
                  currentRole == ROLE_TRACKER ? "TRACKER" :
                  currentRole == ROLE_RELAY   ? "RELAY"   : "GATEWAY");
    Serial.printf("  Device ID: %d (%s)\n",
                  device.getDeviceId(),
                  LedController::getColorName(device.getDeviceId()));
    Serial.printf("  MAC:       %02X:%02X:%02X:%02X:%02X:%02X\n",
                  mesh.getMyMac()[0], mesh.getMyMac()[1], mesh.getMyMac()[2],
                  mesh.getMyMac()[3], mesh.getMyMac()[4], mesh.getMyMac()[5]);
    Serial.printf("  GPS:       %s\n", gps.isDetected() ? "detected" : "not found");
    Serial.printf("  SD card:   %s\n", sdLog.isReady() ? sdLog.getFilename() : "not available");
    Serial.printf("  Battery:   %.2fV (%d%%)\n", battery.readVoltage(), battery.getPercentage());
    Serial.printf("  CPU:       %d MHz\n", getCpuFrequencyMhz());
    Serial.println("----------------------------------------");
    Serial.println("[MAIN] Ready. Type 'help' for commands.");

    // Visual confirmation: blink device color 3 times
    led.blink(3, 300, 200);
    led.on(); // Stay on with device color
}

void loop() {
    // Handle serial debug commands (relay/tracker only — gateway uses binary framing)
    if (currentRole != ROLE_GATEWAY) handleSerialInput();

    // Role-specific main loop
    switch (currentRole) {
    case ROLE_TRACKER: loopTracker(); break;
    case ROLE_RELAY:   loopRelay();   break;
    case ROLE_GATEWAY: loopGateway(); break;
    }
}
