#include <Arduino.h>
#include <esp_log.h>
#include <nvs_flash.h>
#include <nvs.h>
#include <ArduinoOTA.h>
#include <inttypes.h>

#include "Config.h"
#include "MatterBridge.h"
#include "MatterDevices.h"
#include "SolarLogic.h"
#include "CirculationLogic.h"
#include "Display.h"
#include "WebUI.h"
#include "WebSerial.h"
#include <apps/esp_sntp.h>
#include <time.h>

static const char* TAG = "main";

// Timing
static unsigned long lastTempRead      = 0;
static unsigned long lastLevelRead     = 0;
static unsigned long lastMatterUpdate  = 0;
static unsigned long lastDisplayUpdate = 0;
static unsigned long lastCircUpdate    = 0;

// WebUI + OTA – einmalig starten wenn WiFi verbunden
static bool wifiServicesStarted = false;
static void startWifiServices();   // fwd-decl, Implementierung weiter unten

// ── Serial-Kommando-Dispatcher ────────────────────────────────────────────────
//
// Bewusst quellen-agnostisch: nimmt eine bereits getrimmte Zeile entgegen und
// verwendet WebSerial::printf/println für die Antwort. WebSerial::* schreibt
// IMMER auf Serial (Hardware-UART) und tee't bei aktiver WebSerial-Sitzung
// zusätzlich in den Browser-Ring-Buffer. So erhalten beide Senken (Hardware-
// Monitor & WebUI-Terminal) identischen Output.
//
static void processCommandLine(const String& line)
{
    if (line.isEmpty()) return;

    if (line.startsWith("SET_POOL_NODE ")) {
        char     nodeStr[20] = {0};
        uint16_t epTemp = 0, epSoll = 0, epMode = 0;
        int parsed = sscanf(line.c_str(),
                            "SET_POOL_NODE %19s %hu %hu %hu",
                            nodeStr, &epTemp, &epSoll, &epMode);
        if (parsed != 4) {
            WebSerial::println("Format: SET_POOL_NODE <nodeId_hex> <epTemp> <epSoll> <epMode>");
            return;
        }
        uint64_t nodeId = strtoull(nodeStr, nullptr, 16);
        nvs_handle_t nvs;
        if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs) == ESP_OK) {
            nvs_set_u64(nvs, NVS_KEY_POOL_NODE_ID, nodeId);
            nvs_set_u16(nvs, NVS_KEY_POOL_EP_TEMP, epTemp);
            nvs_set_u16(nvs, NVS_KEY_POOL_EP_SOLL, epSoll);
            nvs_set_u16(nvs, NVS_KEY_POOL_EP_MODE, epMode);
            nvs_commit(nvs);
            nvs_close(nvs);
            WebSerial::printf("OK: NodeID=0x%016llX ep=%d/%d/%d\n",
                              nodeId, epTemp, epSoll, epMode);
        }
        MatterDevices::subscribeToPoolMaster(nodeId, epTemp, epSoll, epMode);
    }
    else if (line == "GET_POOL_NODE") {
        nvs_handle_t nvs;
        uint64_t nodeId = 0;
        uint16_t epTemp = 0xFFFF, epSoll = 0xFFFF, epMode = 0xFFFF;
        if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs) == ESP_OK) {
            nvs_get_u64(nvs, NVS_KEY_POOL_NODE_ID, &nodeId);
            nvs_get_u16(nvs, NVS_KEY_POOL_EP_TEMP, &epTemp);
            nvs_get_u16(nvs, NVS_KEY_POOL_EP_SOLL, &epSoll);
            nvs_get_u16(nvs, NVS_KEY_POOL_EP_MODE, &epMode);
            nvs_close(nvs);
        }
        WebSerial::printf("NodeID: 0x%016llX  epTemp:%d  epSoll:%d  epMode:%d\n",
                          nodeId, epTemp, epSoll, epMode);
    }
    else if (line == "GET_STATE") {
        const SolarLogic::State& s = SolarLogic::state;
        WebSerial::println("=== SolarControl Zustand ===");
        WebSerial::printf("Modus:   %s\n",
            s.currentMode == SolarLogic::Mode::AUTO        ? "AUTO"   :
            s.currentMode == SolarLogic::Mode::MANUAL_POOL ? "POOL"   : "BOILER");
        WebSerial::printf("Dach:    %.2f C\n", s.roofTemp);
        WebSerial::printf("Boiler:  %.2f C\n", s.boilerTemp);
        WebSerial::printf("Puffer:  %.2f C\n", s.storageTemp);
        WebSerial::printf("Rueckl:  %.2f C\n", s.backflowTemp);
        WebSerial::printf("Pool:    %.2f C / Soll: %.2f C\n", s.poolTemp, s.poolSollTemp);
        WebSerial::printf("Pumpe:   %s\n", s.pumpRunning    ? "EIN" : "AUS");
        WebSerial::printf("Ventil:  %s\n", s.valvePool      ? "POOL"   : "BOILER");
        WebSerial::printf("Zirk:    %s\n", s.circulationOn  ? "EIN"    : "AUS");
        WebSerial::printf("Bel:     %s\n", s.illuminationOn ? "EIN"    : "AUS");
        WebSerial::printf("Ventil-Status: %s\n", s.valveStatusOK  ? "OK"   : "FEHLER");
        WebSerial::printf("Bewegung:      %s\n", s.motionDetected ? "JA"   : "NEIN");
        WebSerial::printf("Pegel:         %.1f%%  %s\n",
                          s.levelPct, s.levelWarn ? "WARNUNG" : "OK");
        WebSerial::printf("Pool-Anforderung: %s\n", s.poolModeRequest ? "JA" : "NEIN");
        WebSerial::println("============================");
    }
    else if (line == "GET_QR") {
        char qr[96] = {0}, manual[32] = {0};
        MatterBridge::getQRCode(qr, sizeof(qr));
        MatterBridge::getManualPairingCode(manual, sizeof(manual));
        WebSerial::printf("QR-Payload    : %s\n", qr[0]     ? qr     : "(noch nicht verfügbar)");
        WebSerial::printf("Pairing-Code  : %s\n", manual[0] ? manual : "(noch nicht verfügbar)");
        WebSerial::printf("Setup-PIN     : %08" PRIu32 "\n", MatterBridge::getSetupPin());
        WebSerial::printf("Discriminator : %u\n", MatterBridge::getDiscriminator());
        WebSerial::printf("Fabrics       : %u\n", MatterBridge::fabricCount());
    }
    else if (line == "OPEN_COMMISSIONING") {
        bool ok = MatterBridge::openCommissioningWindow(900);
        WebSerial::println(ok ? "OK: Commissioning-Fenster geöffnet (900 s)"
                              : "FEHLER: Commissioning-Fenster konnte nicht geöffnet werden");
    }
    else if (line == "WIFI_STATUS") {
        MatterBridge::WifiStatus s = MatterBridge::getWifiStatus();
        WebSerial::println("=== WiFi / Matter Status ===");
        WebSerial::printf("STA aktiv : %s\n", s.staActive ? "JA" : "NEIN");
        WebSerial::printf("Verbunden : %s\n", s.connected ? "JA" : "NEIN");
        WebSerial::printf("SSID(NVS) : %s\n", s.ssid[0] ? s.ssid : "(leer)");
        WebSerial::printf("IP        : %s\n", s.ip);
        WebSerial::printf("RSSI      : %d dBm\n", s.rssi);
        WebSerial::printf("Fabrics   : %u\n", s.fabricCount);
        WebSerial::println("============================");
    }
    else if (line == "WIFI_RECONNECT") {
        bool ok = MatterBridge::connectWifi();
        WebSerial::println(ok ? "OK: WiFi-Reconnect getriggert"
                              : "FEHLER: WiFi-Reconnect (vor esp_matter::start?)");
    }
    else if (line == "FACTORY_RESET") {
        MatterBridge::factoryReset();
    }
    else if (line == "MOTION_ON") {
        SolarLogic::setMotionPower(true);
        WebSerial::println("OK: Bewegungsmelder EIN");
    }
    else if (line == "MOTION_OFF") {
        SolarLogic::setMotionPower(false);
        WebSerial::println("OK: Bewegungsmelder AUS");
    }
    else if (line == "REBOOT") {
        WebSerial::println("Reboot in 500 ms...");
        delay(500);
        esp_restart();
    }
    else if (line == "HELP" || line == "?") {
        WebSerial::println("Kommandos: SET_POOL_NODE | GET_POOL_NODE | GET_STATE | "
                           "GET_QR | OPEN_COMMISSIONING | FACTORY_RESET | "
                           "WIFI_STATUS | WIFI_RECONNECT | "
                           "MOTION_ON | MOTION_OFF | REBOOT | HELP");
    }
    else {
        WebSerial::printf("Unbekanntes Kommando: '%s'  (HELP für Liste)\n", line.c_str());
    }
}

// Kommando-Dispatcher: leert beide Quellen (Hardware-Serial + WebSerial-Queue).
static void handleSerialCommands()
{
    // 1) Hardware-Serial (FTDI / USB-CDC)
    if (Serial.available()) {
        String line = Serial.readStringUntil('\n');
        line.trim();
        processCommandLine(line);
    }

    // 2) WebSerial-Eingaben aus dem Browser (max. ein paar pro Loop)
    char wsLine[128];
    for (int i = 0; i < 4 && WebSerial::popInput(wsLine, sizeof(wsLine)); i++) {
        // Echo damit der User seine Eingabe im Terminal sieht
        WebSerial::printf("> %s\n", wsLine);
        String line(wsLine);
        line.trim();
        processCommandLine(line);
    }
}

// ── setup() ───────────────────────────────────────────────────────────────────
void setup()
{
    Serial.begin(115200);
    delay(500);
    ESP_LOGI(TAG, "SolarControl-Matter startet (v%s)", APP_VERSION);

    // NVS (vor Matter — wird auch vom CHIP-Stack genutzt)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    // Hardware
    SolarLogic::init();
    CirculationLogic::init();

    // Display (vor Matter damit Boot-Screen erscheint)
    Display::init();

    // ── Phase 1: Matter-Node + Endpoints anlegen ─────────────────────────────
    if (!MatterBridge::init()) {
        Display::showError("MatterBridge::init");
        delay(5000);
        esp_restart();
    }

    // ── Phase 2: Matter-Stack starten + Onboarding-Codes cachen ──────────────
    if (!MatterBridge::start()) {
        Display::showError("MatterBridge::start");
        delay(5000);
        esp_restart();
    }

    // ── Commissioning-Screen anzeigen falls noch nicht eingebunden ───────────
    if (!Display::isCommissioned) {
        char qr[96] = {0}, manual[32] = {0};
        MatterBridge::getQRCode(qr, sizeof(qr));
        MatterBridge::getManualPairingCode(manual, sizeof(manual));
        Display::showCommissioningScreen(qr, manual);
    }

    ESP_LOGI(TAG, "Setup abgeschlossen. Kommandos: GET_STATE | GET_QR | "
                  "OPEN_COMMISSIONING | FACTORY_RESET | SET_POOL_NODE | GET_POOL_NODE");
}

// ── WiFi-Services einmalig starten (WebUI + ArduinoOTA + SNTP) ───────────────
static void startWifiServices()
{
    // SNTP-Server aus NVS laden (per WebUI änderbar) — Defaults aus Config.h
    static char ntp1[64] = NTP_SERVER_PRIMARY;
    static char ntp2[64] = NTP_SERVER_SECONDARY;
    static char tz[64]   = TIMEZONE_POSIX;
    {
        nvs_handle_t nvs;
        if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs) == ESP_OK) {
            size_t l = sizeof(tz);   nvs_get_str(nvs, NVS_KEY_TZ,   tz,   &l);
            l = sizeof(ntp1);        nvs_get_str(nvs, NVS_KEY_NTP1, ntp1, &l);
            l = sizeof(ntp2);        nvs_get_str(nvs, NVS_KEY_NTP2, ntp2, &l);
            nvs_close(nvs);
        }
    }
    setenv("TZ", tz, 1);
    tzset();
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, ntp1);
    esp_sntp_setservername(1, ntp2);
    esp_sntp_init();
    ESP_LOGI("main", "SNTP gestartet (TZ=%s NTP1=%s NTP2=%s)", tz, ntp1, ntp2);

    // WebUI (HTTP-Server Port 80, WebSocket /ws, OTA /update)
    WebUI::begin();

    // ArduinoOTA (PlatformIO espota Protokoll, Port OTA_PORT)
    // Hinweis: mDNS-Discovery kann mit Matter-Stack kollidieren.
    // Direkter IP-Upload (upload_port = <IP> in platformio.ini) funktioniert
    // aber ohne mDNS zuverlässig.
    ArduinoOTA.setHostname("solarcontrol");
    ArduinoOTA.setPort(OTA_PORT);
    ArduinoOTA.setPassword(OTA_PASSWORD);
    ArduinoOTA.onStart([]() {
        ESP_LOGI("OTA", "Start");
        WebUI::stop();   // HTTP-Server vor Flash-Vorgang stoppen
    });
    ArduinoOTA.onEnd([]()   { ESP_LOGI("OTA", "Ende"); });
    ArduinoOTA.onError([](ota_error_t e) {
        ESP_LOGE("OTA", "Fehler[%u]", e);
        WebUI::begin();  // HTTP-Server wieder starten
    });
    ArduinoOTA.begin();
    ESP_LOGI("main", "ArduinoOTA gestartet (Port %d)", OTA_PORT);
}

// ── loop() ────────────────────────────────────────────────────────────────────
void loop()
{
    unsigned long now = millis();

    // WiFi-Services einmalig starten sobald verbunden
    // MatterBridge setzt das Event im kWiFiConnectivityChange-Callback;
    // consumeWifiUpEvent() liefert true beim ersten Lesen und resettet das Flag.
    if (!wifiServicesStarted && MatterBridge::consumeWifiUpEvent()) {
        wifiServicesStarted = true;
        startWifiServices();
    }

    // Temperaturen & Steuerlogik (alle 5s)
    if (now - lastTempRead >= TEMP_READ_INTERVAL_MS) {
        lastTempRead = now;
        SolarLogic::readTemperatures();
        SolarLogic::update();
    }

    // Pegelsonde (alle 30s)
    if (now - lastLevelRead >= 30000UL) {
        lastLevelRead = now;
        SolarLogic::readLevelSensor();
    }

    // Matter Attribut-Updates (alle 10s)
    if (now - lastMatterUpdate >= MATTER_UPDATE_INTERVAL_MS) {
        lastMatterUpdate = now;

        // EP1: Betriebsmodus (spiegelt den aktuellen SolarLogic-Modus wider)
        MatterDevices::updateControlMode(
            static_cast<uint8_t>(SolarLogic::state.currentMode));

        // EP2: Pumpenstatus (read-only aus Matter-Sicht)
        MatterDevices::updatePumpState(SolarLogic::state.pumpRunning);

        // EP3: Ventil-Rückmeldung – Hardware-Pin, entkoppelt von EP1
        // valvePool=true → POOL(1), valvePool=false → BOILER(0)
        MatterDevices::updateValveFeedback(
            SolarLogic::state.valvePool
                ? MatterDevices::VALVE_POOL
                : MatterDevices::VALVE_BOILER);

        // EP4: Dach-Kollektor Temperatur
        MatterDevices::updateTemperature(MatterDevices::epRoofTemp,
                                          SolarLogic::state.roofTemp);

        // EP5: Boiler Temperatur
        MatterDevices::updateTemperature(MatterDevices::epBoilerTemp,
                                          SolarLogic::state.boilerTemp);
    }

    // Display Update (100ms)
    if (now - lastDisplayUpdate >= 100UL) {
        lastDisplayUpdate = now;
        if (!Display::isCommissioned) {
            char qr[96] = {0}, manual[32] = {0};
            MatterBridge::getQRCode(qr, sizeof(qr));
            MatterBridge::getManualPairingCode(manual, sizeof(manual));
            Display::showCommissioningScreen(qr, manual);
        } else {
            Display::update(5000);
        }
    }

    // Zirkulationslogik (jede Sekunde: Timer, Auto-Trigger, SNTP-Check)
    if (now - lastCircUpdate >= 1000UL) {
        lastCircUpdate = now;
        CirculationLogic::update();
    }

    // Button & Bewegungsmelder (jeden Loop)
    SolarLogic::handleButton();
    SolarLogic::handleMotion();

    // Serial-Kommandos
    handleSerialCommands();

    // WebUI-Kommandos verarbeiten (Relay/Modus von WebSocket)
    WebUI::processCommands();

    // ArduinoOTA (espota Protokoll via PlatformIO)
    if (wifiServicesStarted) ArduinoOTA.handle();
}
