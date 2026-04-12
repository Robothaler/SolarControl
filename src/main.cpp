#include <Arduino.h>
#include <esp_matter.h>
#include <esp_matter_cluster.h>
#include <esp_log.h>
#include <nvs_flash.h>
#include <nvs.h>
#include <esp_netif.h>
#include <ArduinoOTA.h>
#include <app/server/Server.h>

#include "Config.h"
#include "MatterDevices.h"
#include "SolarLogic.h"
#include "CirculationLogic.h"
#include "Display.h"
#include "WebUI.h"
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
static volatile bool wifiServicesPending = false;
static bool          wifiServicesStarted = false;

// QR-Code / Pairing
static char matterQRPayload[64]   = {0};
static char matterPairingCode[16] = {0};

// ── Matter Attribute Callback ─────────────────────────────────────────────────
static esp_err_t matterAttributeCallback(
    const esp_matter::attribute::callback_type_t type,
    uint16_t endpoint_id,
    uint32_t cluster_id,
    uint32_t attribute_id,
    esp_matter_attr_val_t* val,
    void* priv_data)
{
    return MatterDevices::attributeChangeCallback(
        type, endpoint_id, cluster_id, attribute_id, val);
}

// ── Matter Identification Callback ───────────────────────────────────────────
// KORRIGIERT: Signatur muss esp_matter::identification::callback_t entsprechen
static esp_err_t matterIdentifyCallback(
    esp_matter::identification::callback_type type,
    uint16_t endpoint_id,
    uint8_t  effect_id,
    uint8_t  effect_variant,
    void*    priv_data)
{
    ESP_LOGI(TAG, "Identify EP%d effect=%d", endpoint_id, effect_id);
    return ESP_OK;
}

// ── Matter Event Callback ─────────────────────────────────────────────────────
static void matterEventCallback(
    const chip::DeviceLayer::ChipDeviceEvent* event,
    intptr_t arg)
{
    switch (event->Type) {
        case chip::DeviceLayer::DeviceEventType::kCommissioningComplete:
            ESP_LOGI(TAG, "Matter Commissioning abgeschlossen!");
            Display::isCommissioned = true;
            break;

        case chip::DeviceLayer::DeviceEventType::kWiFiConnectivityChange:
            {
                esp_netif_ip_info_t ipInfo;
                esp_netif_t* netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
                if (netif && esp_netif_get_ip_info(netif, &ipInfo) == ESP_OK
                    && ipInfo.ip.addr != 0)
                {
                    Display::wifiConnected = true;
                    snprintf(Display::wifiIP, sizeof(Display::wifiIP),
                             IPSTR, IP2STR(&ipInfo.ip));
                    ESP_LOGI(TAG, "WiFi IP: %s", Display::wifiIP);
                    wifiServicesPending = true;   // WebUI + OTA im Loop starten
                } else {
                    Display::wifiConnected = false;
                    strncpy(Display::wifiIP, "---", sizeof(Display::wifiIP));
                }
            }
            break;

        case chip::DeviceLayer::DeviceEventType::kFabricRemoved:
            ESP_LOGW(TAG, "Matter Fabric entfernt");
            Display::isCommissioned = false;
            break;

        case chip::DeviceLayer::DeviceEventType::kBLEDeinitialized:
            ESP_LOGI(TAG, "BLE deinitialisiert");
            break;

        default:
            break;
    }
}

// ── PoolMaster Subscription Report Callback ───────────────────────────────────
static void poolMasterReportCallback(
    uint64_t nodeId,
    uint16_t endpointId,
    uint32_t clusterId,
    uint32_t attributeId,
    esp_matter_attr_val_t* val)
{
    nvs_handle_t nvs;
    uint16_t epTemp = 0xFFFF, epSoll = 0xFFFF, epMode = 0xFFFF;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs) == ESP_OK) {
        nvs_get_u16(nvs, NVS_KEY_POOL_EP_TEMP, &epTemp);
        nvs_get_u16(nvs, NVS_KEY_POOL_EP_SOLL, &epSoll);
        nvs_get_u16(nvs, NVS_KEY_POOL_EP_MODE, &epMode);
        nvs_close(nvs);
    }

    if (clusterId == chip::app::Clusters::TemperatureMeasurement::Id &&
        attributeId == chip::app::Clusters::TemperatureMeasurement::Attributes::MeasuredValue::Id)
    {
        float tempC = val->val.i16 / 100.0f;
        if (endpointId == epTemp)      SolarLogic::onPoolTempReceived(tempC);
        else if (endpointId == epSoll) SolarLogic::onPoolSolltempReceived(tempC);
        return;
    }

    if (clusterId  == chip::app::Clusters::OnOff::Id &&
        attributeId == chip::app::Clusters::OnOff::Attributes::OnOff::Id &&
        endpointId  == epMode)
    {
        SolarLogic::onPoolModeRequestReceived(val->val.b);
    }
}

// ── PoolMaster Subscription starten ──────────────────────────────────────────
static void startPoolMasterSubscription()
{
    nvs_handle_t nvs;
    uint64_t nodeId  = 0;
    // KORRIGIERT: Syntaxfehler "0    xFFFF" → korrekte Initialisierung
    uint16_t epTemp  = 0xFFFF;
    uint16_t epSoll  = 0xFFFF;
    uint16_t epMode  = 0xFFFF;

    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "NVS nicht lesbar: %s", esp_err_to_name(err));
        return;
    }
    nvs_get_u64(nvs, NVS_KEY_POOL_NODE_ID, &nodeId);
    nvs_get_u16(nvs, NVS_KEY_POOL_EP_TEMP, &epTemp);
    nvs_get_u16(nvs, NVS_KEY_POOL_EP_SOLL, &epSoll);
    nvs_get_u16(nvs, NVS_KEY_POOL_EP_MODE, &epMode);
    nvs_close(nvs);

    if (nodeId == 0 || epTemp == 0xFFFF || epSoll == 0xFFFF || epMode == 0xFFFF) {
        ESP_LOGW(TAG, "PoolMaster nicht konfiguriert.");
        ESP_LOGW(TAG, "Kommando: SET_POOL_NODE <nodeId_hex> <epTemp> <epSoll> <epMode>");
        return;
    }
    MatterDevices::subscribeToPoolMaster(nodeId, epTemp, epSoll, epMode);
}

// ── Serial-Kommandos ──────────────────────────────────────────────────────────
static void handleSerialCommands()
{
    if (!Serial.available()) return;
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.isEmpty()) return;

    if (line.startsWith("SET_POOL_NODE ")) {
        char     nodeStr[20] = {0};
        uint16_t epTemp = 0, epSoll = 0, epMode = 0;
        int parsed = sscanf(line.c_str(),
                            "SET_POOL_NODE %19s %hu %hu %hu",
                            nodeStr, &epTemp, &epSoll, &epMode);
        if (parsed != 4) {
            Serial.println("Format: SET_POOL_NODE <nodeId_hex> <epTemp> <epSoll> <epMode>");
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
            Serial.printf("OK: NodeID=0x%016llX ep=%d/%d/%d\n",
                          nodeId, epTemp, epSoll, epMode);
        }
        startPoolMasterSubscription();
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
        Serial.printf("NodeID: 0x%016llX  epTemp:%d  epSoll:%d  epMode:%d\n",
                      nodeId, epTemp, epSoll, epMode);
    }
    else if (line == "GET_STATE") {
        const SolarLogic::State& s = SolarLogic::state;
        Serial.println("=== SolarControl Zustand ===");
        Serial.printf("Modus:   %s\n",
            s.currentMode == SolarLogic::Mode::AUTO        ? "AUTO"   :
            s.currentMode == SolarLogic::Mode::MANUAL_POOL ? "POOL"   : "BOILER");
        Serial.printf("Dach:    %.2f C\n", s.roofTemp);
        Serial.printf("Boiler:  %.2f C\n", s.boilerTemp);
        Serial.printf("Puffer:  %.2f C\n", s.storageTemp);
        Serial.printf("Rueckl:  %.2f C\n", s.backflowTemp);
        Serial.printf("Pool:    %.2f C / Soll: %.2f C\n", s.poolTemp, s.poolSollTemp);
        Serial.printf("Pumpe:   %s\n", s.pumpRunning    ? "EIN" : "AUS");
        Serial.printf("Ventil:  %s\n", s.valvePool      ? "POOL"   : "BOILER");
        Serial.printf("Zirk:    %s\n", s.circulationOn  ? "EIN"    : "AUS");
        Serial.printf("Bel:     %s\n", s.illuminationOn ? "EIN"    : "AUS");
        Serial.printf("Ventil-Status: %s\n", s.valveStatusOK  ? "OK"   : "FEHLER");
        Serial.printf("Bewegung:      %s\n", s.motionDetected ? "JA"   : "NEIN");
        Serial.printf("Pegel:         %.1f%%  %s\n",
                      s.levelPct, s.levelWarn ? "WARNUNG" : "OK");
        Serial.printf("Pool-Anforderung: %s\n", s.poolModeRequest ? "JA" : "NEIN");
        Serial.println("============================");
    }
        else if (line.startsWith("SET_QR_PAYLOAD ")) {
        // Format: SET_QR_PAYLOAD MT:Y.K9042C00KA0648G00
        String payload = line.substring(15); // nach "SET_QR_PAYLOAD "
        payload.trim();

        if (payload.length() < 4 || !payload.startsWith("MT:")) {
            Serial.println("FEHLER: Payload muss mit 'MT:' beginnen");
            return;
        }

        nvs_handle_t nvs;
        if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs) == ESP_OK) {
            nvs_set_str(nvs, "qr_payload", payload.c_str());
            nvs_commit(nvs);
            nvs_close(nvs);
            strncpy(matterQRPayload, payload.c_str(), sizeof(matterQRPayload)-1);
            // QR-Code Cache invalidieren damit neu generiert wird
            Serial.printf("OK: QR-Payload gespeichert: %s\n", matterQRPayload);
        }
    }
    else if (line == "GET_QR") {
        Serial.printf("QR-Payload : %s\n", matterQRPayload);
        Serial.printf("Pairing    : %s\n", matterPairingCode);
    }
    else if (line == "FACTORY_RESET") {
        Serial.println("Matter Factory Reset...");
        Display::showResetWarning();
        delay(2000);
        esp_matter::factory_reset();
    }
    else if (line == "MOTION_ON") {
        SolarLogic::setMotionPower(true);
        Serial.println("OK: Bewegungsmelder EIN");
    }
    else if (line == "MOTION_OFF") {
        SolarLogic::setMotionPower(false);
        Serial.println("OK: Bewegungsmelder AUS");
    }
    else {
        Serial.println("Kommandos: SET_POOL_NODE | GET_POOL_NODE | GET_STATE | MOTION_ON | MOTION_OFF | FACTORY_RESET");
    }
}

// ── setup() ───────────────────────────────────────────────────────────────────
void setup()
{
    Serial.begin(115200);
    delay(500);
    ESP_LOGI(TAG, "SolarControl-Matter startet...");

    // NVS
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

    // ── Matter Node erstellen ─────────────────────────────────────────────────
    // KORRIGIERT: vendor_id/product_id werden über chip::DeviceLayer gesetzt,
    // nicht über nodeConfig direkt
    esp_matter::node::config_t nodeConfig;

    esp_matter::node_t* node = esp_matter::node::create(
        &nodeConfig,
        matterAttributeCallback,
        matterIdentifyCallback);   // ← KORRIGIERT: identify callback, nicht event

    if (!node) {
        ESP_LOGE(TAG, "Matter Node erstellen fehlgeschlagen!");
        Display::showError("Matter Node");
        delay(5000);
        esp_restart();
    }

    // Endpoints registrieren
    ret = MatterDevices::init(node);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "MatterDevices::init() fehlgeschlagen: %s",
                 esp_err_to_name(ret));
        Display::showError("Matter Endpoints");
        delay(5000);
        esp_restart();
    }

    // Heap-Diagnose VOR Matter-Start: BLE-Controller braucht MALLOC_CAP_INTERNAL|DMA.
    // Der CHIP-Task (höhere FreeRTOS-Priorität) preempt sofort nach esp_matter::start()
    // → Logs nach start() sind unerreichbar bis nach dem BLE-Init-Crash.
    ESP_LOGI(TAG, "RAM vor Matter-Start: %u B frei, groesster Block: %u B",
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));

    // Matter Stack starten
    // KORRIGIERT: esp_matter::start() nimmt nur den Event Callback
    ret = esp_matter::start(matterEventCallback);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_matter::start() fehlgeschlagen: %s",
                 esp_err_to_name(ret));
        Display::showError("Matter Start");
        delay(5000);
        esp_restart();
    }
    ESP_LOGI(TAG, "Matter Stack gestartet");

        // ── QR-Code und Pairing-Code auslesen ────────────────────────────────────
    // KORRIGIERT: Direkte NVS-Methode – vermeidet nicht verfügbare CHIP Headers
    // esp-matter speichert Setup-PIN in "chip-factory" NVS Namespace
    {
        // Setup PIN aus NVS lesen (von esp-matter dort gespeichert)
        uint32_t setupPinCode = 20202021; // Matter Default-PIN als Fallback
        nvs_handle_t factoryNvs;
        if (nvs_open("chip-factory", NVS_READONLY, &factoryNvs) == ESP_OK) {
            nvs_get_u32(factoryNvs, "pin-code", &setupPinCode);
            nvs_close(factoryNvs);
            ESP_LOGI(TAG, "Setup PIN aus NVS gelesen: %" PRIu32, setupPinCode);
        } else {
            ESP_LOGW(TAG, "chip-factory NVS nicht lesbar → Default PIN");
        }

        // Pairing-Code formatieren (8-stellig mit führenden Nullen)
        snprintf(matterPairingCode, sizeof(matterPairingCode),
                 "%08" PRIu32, setupPinCode);

        // QR-Code Payload: esp-matter gibt diesen beim Start ins Log aus.
        // Format: "MT:Y.K9042C00KA0648G00" (gerätespezifisch)
        // Wir zeigen den Pairing-Code auf dem Display – der QR-Code
        // wird parallel im Serial-Monitor ausgegeben und kann dort
        // abgescannt werden.
        // Für Display-QR-Code: Payload aus Log kopieren und in NVS speichern.
        snprintf(matterQRPayload, sizeof(matterQRPayload), "");

        // Versuche QR-Payload aus eigenem NVS-Eintrag zu lesen
        // (wird beim ersten Commissioning manuell eingetragen via Serial)
        nvs_handle_t solarNvs;
        if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &solarNvs) == ESP_OK) {
            size_t qrLen = sizeof(matterQRPayload);
            nvs_get_str(solarNvs, "qr_payload", matterQRPayload, &qrLen);
            nvs_close(solarNvs);
        }

        ESP_LOGI(TAG, "Pairing-Code: %s", matterPairingCode);
        if (strlen(matterQRPayload) > 0) {
            ESP_LOGI(TAG, "QR-Payload (aus NVS): %s", matterQRPayload);
        } else {
            ESP_LOGW(TAG, "QR-Payload nicht in NVS → Serial-Monitor für QR-Code");
        }
    }

    // ── Commissioning-Status prüfen ───────────────────────────────────────────
    Display::isCommissioned =
        chip::Server::GetInstance().GetFabricTable().FabricCount() > 0;

    if (!Display::isCommissioned) {
        ESP_LOGI(TAG, "Nicht commissioned → QR-Code anzeigen");
        Display::showCommissioningScreen(matterQRPayload, matterPairingCode);
    } else {
        ESP_LOGI(TAG, "Bereits commissioned → Normal-Betrieb");
    }

    // ── PoolMaster Subscription ───────────────────────────────────────────────
    startPoolMasterSubscription();

    ESP_LOGI(TAG, "Setup abgeschlossen. Kommandos: GET_STATE | GET_POOL_NODE");
}

// ── WiFi-Services einmalig starten (WebUI + ArduinoOTA + SNTP) ───────────────
static void startWifiServices()
{
    // SNTP – Echtzeit für CirculationLogic (Präsenzfenster, Legionellenschutz)
    // Timezone aus Config.h setzen (POSIX TZ-String, z.B. CET/CEST).
    setenv("TZ", TIMEZONE_POSIX, 1);
    tzset();
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_setservername(1, "time.cloudflare.com");
    esp_sntp_init();
    ESP_LOGI("main", "SNTP gestartet (TZ=%s)", TIMEZONE_POSIX);

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
    if (wifiServicesPending && !wifiServicesStarted) {
        wifiServicesPending = false;
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
            Display::showCommissioningScreen(matterQRPayload, matterPairingCode);
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
