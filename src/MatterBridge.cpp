// =============================================================================
//  MatterBridge.cpp — SolarControl Matter Bridge Implementation
// =============================================================================
//  Verschoben aus main.cpp:
//    • Matter-Node-Erzeugung + Endpoints (über MatterDevices::init/postStart)
//    • Attribute-/Identify-/Event-Callbacks
//    • PoolMaster-Subscription-Restore aus NVS
//
//  Neu (analog ESP32-PoolMaster Matter_dev):
//    • two-phase Init (init/start)
//    • Onboarding-Cache: QR-Payload + Manual-Pairing-Code via chip::GetQRCode()
//    • openCommissioningWindow() via PlatformMgr().ScheduleWork()
//    • factoryReset() Helper
// =============================================================================

// WICHTIG: KEIN <Arduino.h> in dieser Übersetzungseinheit einbinden!
// MatterDevices.h zieht via esp_matter -> lwip die Makros IPADDR_NONE /
// INADDR_NONE rein. Würde danach Arduino.h folgen, expandiert das Makro
// in die Klassendeklaration "extern const IPAddress INADDR_NONE;" und
// erzeugt den Build-Fehler "expected ')' before numeric constant".
// (Identisches Vorgehen wie ESP32-PoolMaster/Matter_dev/src/MatterBridge.cpp)

#include "MatterBridge.h"
#include "Config.h"
#include "MatterDevices.h"
#include "Display.h"

#include <esp_log.h>
#include <esp_matter.h>
#include <esp_matter_cluster.h>
#include <esp_netif.h>
#include <esp_heap_caps.h>
#include <esp_wifi.h>
#include <esp_event.h>
#include <inttypes.h>
#include <nvs.h>
#include <string.h>

#include <freertos/FreeRTOS.h>
#include <freertos/timers.h>
#include <freertos/task.h>

#include <cstring>

// CHIP / Matter-Stack-Header
#include <app/server/Server.h>
#include <app/server/OnboardingCodesUtil.h>
#include <platform/CHIPDeviceLayer.h>
#include <platform/CommissionableDataProvider.h>
#include <lib/support/Span.h>

static const char* TAG = "MatterBridge";

// =============================================================================
//  Modul-private State
// =============================================================================
static char     s_qr_code[96]         = {0};
static char     s_pairing_code[32]    = {0};
static uint32_t s_setup_pin           = 0;
static uint16_t s_discriminator       = 0;
static bool     s_started             = false;
static volatile bool s_wifi_up_event  = false;

// WiFi-Reconnect-Timer analog PoolMaster Matter_dev — **Callback entkoppelt**:
// Der Timer feuert nur noch ein Worker-Task-Notify. `esp_wifi_connect()` darf nicht
// direkt aus dem Timer-Daemon laufen (Tmr Svc-Stack oft 2048 B); sonst Overflow
// + Reboot („kein WebUI“). Siehe Feld-Log: stack overflow in task Tmr Svc.
static TimerHandle_t s_wifi_reconnect_timer  = nullptr;
static TaskHandle_t  s_wifi_reconnect_worker = nullptr;
static bool          s_wifi_handlers_done    = false;

static constexpr uint32_t kWifiReconnectWorkerStackWords = 3072;
static constexpr UBaseType_t kWifiReconnectWorkerPrio      = (tskIDLE_PRIORITY + 5);

static void wifiReconnectWorkerMain(void* /*arg*/)
{
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        ESP_LOGW(TAG, "WiFi: Reconnect-Versuch (Worker-Task)…");
        esp_err_t err = esp_wifi_connect();
        if (err != ESP_OK && err != ESP_ERR_WIFI_CONN) {
            ESP_LOGW(TAG, "esp_wifi_connect (reconnect) -> %s", esp_err_to_name(err));
            if (s_wifi_reconnect_timer) {
                xTimerStop(s_wifi_reconnect_timer, pdMS_TO_TICKS(50));
                xTimerStart(s_wifi_reconnect_timer, pdMS_TO_TICKS(50));
            }
        }
    }
}

static void wifiReconnectTimerCb(TimerHandle_t /*xTimer*/)
{
    if (s_wifi_reconnect_worker)
        xTaskNotifyGive(s_wifi_reconnect_worker);
}

// =============================================================================
//  Matter-Callbacks (vorher in main.cpp)
// =============================================================================
static esp_err_t attributeCallback(
    const esp_matter::attribute::callback_type_t type,
    uint16_t endpoint_id,
    uint32_t cluster_id,
    uint32_t attribute_id,
    esp_matter_attr_val_t* val,
    void* /*priv_data*/)
{
    return MatterDevices::attributeChangeCallback(
        type, endpoint_id, cluster_id, attribute_id, val);
}

static esp_err_t identifyCallback(
    esp_matter::identification::callback_type type,
    uint16_t endpoint_id,
    uint8_t  effect_id,
    uint8_t  /*effect_variant*/,
    void*    /*priv_data*/)
{
    ESP_LOGI(TAG, "Identify EP%u effect=%u", endpoint_id, effect_id);
    return ESP_OK;
}

// PoolMaster-Subscription nach Commissioning automatisch wiederherstellen
static void restorePoolMasterSubscription();

static void deviceEventCallback(
    const chip::DeviceLayer::ChipDeviceEvent* event,
    intptr_t /*arg*/)
{
    using namespace chip::DeviceLayer;

    switch (event->Type) {
        case DeviceEventType::kCommissioningComplete:
            ESP_LOGI(TAG, "Matter Commissioning abgeschlossen!");
            Display::isCommissioned = true;
            // Falls PoolMaster-NodeId schon konfiguriert: Subscription neu starten
            restorePoolMasterSubscription();
            break;

        case DeviceEventType::kWiFiConnectivityChange: {
            esp_netif_ip_info_t ipInfo{};
            esp_netif_t* netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
            if (netif && esp_netif_get_ip_info(netif, &ipInfo) == ESP_OK
                && ipInfo.ip.addr != 0)
            {
                Display::wifiConnected = true;
                snprintf(Display::wifiIP, sizeof(Display::wifiIP),
                         IPSTR, IP2STR(&ipInfo.ip));
                ESP_LOGI(TAG, "WiFi IP: %s", Display::wifiIP);
                s_wifi_up_event = true;   // wird im loop() konsumiert
            } else {
                Display::wifiConnected = false;
                strncpy(Display::wifiIP, "---", sizeof(Display::wifiIP));
            }
            break;
        }

        case DeviceEventType::kFabricRemoved:
            ESP_LOGW(TAG, "Matter Fabric entfernt");
            Display::isCommissioned = false;
            break;

        default:
            break;
    }
}

// =============================================================================
//  PoolMaster-Subscription nach Commissioning automatisch wiederherstellen
// =============================================================================
/** Wenn gesetzt (NVS pool_http_base), läuft die Pool-Anbindung per HTTP —
 *  keine Matter-case-Session zur Pool-ID nötig. */
static bool poolHttpConfiguredInNvs()
{
    char   base[141] = {0};
    size_t l         = sizeof(base);
    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs) == ESP_OK) {
        esp_err_t e = nvs_get_str(nvs, NVS_KEY_POOL_HTTP_BASE, base, &l);
        nvs_close(nvs);
        if (e == ESP_OK && base[0] != '\0')
            return true;
    }
    const char* def = POOL_HTTP_BASE_DEFAULT;
    return (def != nullptr && def[0] != '\0');
}

static void restorePoolMasterSubscription()
{
    if (poolHttpConfiguredInNvs()) {
        ESP_LOGI(TAG, "pool_http_base gesetzt → Pool per HTTP — überspringe Matter-Subscribe.");
        return;
    }

    nvs_handle_t nvs;
    uint64_t nodeId = 0;
    uint16_t epTemp = 0xFFFF, epSoll = 0xFFFF, epMode = 0xFFFF;

    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs) != ESP_OK) {
        ESP_LOGW(TAG, "NVS nicht lesbar — PoolMaster-Subscription übersprungen");
        return;
    }
    nvs_get_u64(nvs, NVS_KEY_POOL_NODE_ID, &nodeId);
    nvs_get_u16(nvs, NVS_KEY_POOL_EP_TEMP, &epTemp);
    nvs_get_u16(nvs, NVS_KEY_POOL_EP_SOLL, &epSoll);
    nvs_get_u16(nvs, NVS_KEY_POOL_EP_MODE, &epMode);
    nvs_close(nvs);

    if (nodeId == 0 || epTemp == 0xFFFF || epSoll == 0xFFFF || epMode == 0xFFFF) {
        ESP_LOGI(TAG, "PoolMaster noch nicht konfiguriert (SET_POOL_NODE via Serial)");
        return;
    }
    MatterDevices::subscribeToPoolMaster(nodeId, epTemp, epSoll, epMode);
}

// =============================================================================
//  WiFi-Direktanmeldung (analog ESP32-PoolMaster Matter_dev/connectToWiFi)
//
//  Der Matter-Stack ruft esp_wifi_init() bereits in esp_matter::start() auf.
//  Eigenes WiFi.begin() / esp_wifi_init() würde die Netifs ein zweites Mal
//  registrieren ("duplicate key" assert -> Reboot). Daher hier nur:
//      esp_wifi_set_config(STA, …) + esp_wifi_connect()
//  Zusätzlich registrieren wir einen einfachen Reconnect-Handler analog dem
//  PoolMaster-Pattern.
// =============================================================================

static void wifiEventHandler(void* /*arg*/, esp_event_base_t base,
                             int32_t event_id, void* /*data*/)
{
    if (base != WIFI_EVENT) return;
    if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
        Display::wifiConnected = false;
        ESP_LOGW(TAG, "WiFi: STA_DISCONNECTED — schedule reconnect in 2s");
        if (s_wifi_reconnect_timer) {
            xTimerStop(s_wifi_reconnect_timer, 0);
            xTimerStart(s_wifi_reconnect_timer, 0);
        }
    } else if (event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "WiFi: STA_START");
    }
}

// IP_EVENT_STA_GOT_IP ist die verlässliche Quelle für "Netzwerk steht".
// Der CHIP-native DeviceEventType::kWiFiConnectivityChange feuert in einigen
// esp-matter-Versionen erst beim Commissioning — bevor ein Gerät commissioned
// ist oder wenn WiFi-Credentials direkt aus dem NVS kommen (PoolMaster-Pattern)
// bleibt er still. Daher hier zusätzlich auf den ESP-IDF-Event hören und das
// s_wifi_up_event-Flag setzen, auf das startWifiServices() in main.cpp wartet.
static void ipEventHandler(void* /*arg*/, esp_event_base_t base,
                           int32_t event_id, void* data)
{
    if (base != IP_EVENT || event_id != IP_EVENT_STA_GOT_IP) return;
    auto* ev = static_cast<ip_event_got_ip_t*>(data);
    Display::wifiConnected = true;
    snprintf(Display::wifiIP, sizeof(Display::wifiIP),
             IPSTR, IP2STR(&ev->ip_info.ip));
    ESP_LOGI(TAG, "=============== WiFi Online ================");
    ESP_LOGI(TAG, " IP: %s  GW: " IPSTR "  Mask: " IPSTR,
             Display::wifiIP,
             IP2STR(&ev->ip_info.gw),
             IP2STR(&ev->ip_info.netmask));
    ESP_LOGI(TAG, "============================================");
    s_wifi_up_event = true;   // wird in loop() konsumiert -> startWifiServices()
}

static void registerWifiHandlersOnce()
{
    if (s_wifi_handlers_done) return;
    if (!s_wifi_reconnect_worker) {
        const BaseType_t ok = xTaskCreate(
            wifiReconnectWorkerMain,
            "wifi_rc_wk",
            kWifiReconnectWorkerStackWords,
            nullptr,
            kWifiReconnectWorkerPrio,
            &s_wifi_reconnect_worker);
        if (ok != pdPASS || !s_wifi_reconnect_worker)
            ESP_LOGE(TAG,
                     "WiFi-Reconnect: Worker-Task konnte nicht gestartet werden");
    }
    if (!s_wifi_reconnect_timer) {
        s_wifi_reconnect_timer = xTimerCreate(
            "wifi_rc", pdMS_TO_TICKS(2000), pdFALSE, nullptr, wifiReconnectTimerCb);
    }
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifiEventHandler, nullptr);
    esp_event_handler_register(IP_EVENT,   IP_EVENT_STA_GOT_IP, ipEventHandler, nullptr);
    s_wifi_handlers_done = true;
    ESP_LOGI(TAG, "WiFi: Reconnect- + IP-Event-Handler registriert");
}

static esp_err_t connectWifiInternal()
{
    char ssid[33] = {0};
    char pass[65] = {0};
    {
        nvs_handle_t nvs;
        if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs) == ESP_OK) {
            size_t l = sizeof(ssid); nvs_get_str(nvs, NVS_KEY_WIFI_SSID, ssid, &l);
            l = sizeof(pass);        nvs_get_str(nvs, NVS_KEY_WIFI_PASS, pass, &l);
            nvs_close(nvs);
        }
    }

    if (ssid[0] == '\0') {
        ESP_LOGW(TAG, "WiFi: keine SSID im NVS — bleibt unverbunden, "
                      "nutzt Matter Network-Commissioning oder WebUI");
        return ESP_ERR_INVALID_STATE;
    }

    registerWifiHandlersOnce();

    wifi_config_t cfg = {};
    strncpy(reinterpret_cast<char*>(cfg.sta.ssid),     ssid, sizeof(cfg.sta.ssid)     - 1);
    strncpy(reinterpret_cast<char*>(cfg.sta.password), pass, sizeof(cfg.sta.password) - 1);
    // Open Networks zulassen wenn Password leer, sonst WPA2-PSK fordern.
    cfg.sta.threshold.authmode = (pass[0] == '\0') ? WIFI_AUTH_OPEN : WIFI_AUTH_WPA2_PSK;
    cfg.sta.pmf_cfg.capable    = true;

    esp_err_t err = esp_wifi_set_config(WIFI_IF_STA, &cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_set_config -> %s", esp_err_to_name(err));
        return err;
    }

    err = esp_wifi_connect();
    if (err == ESP_OK || err == ESP_ERR_WIFI_CONN) {
        ESP_LOGI(TAG, "WiFi: connect-Trigger gesendet (SSID='%s')", ssid);
        return ESP_OK;
    }
    ESP_LOGW(TAG, "esp_wifi_connect -> %s — versuche es erneut in 2s",
             esp_err_to_name(err));
    if (s_wifi_reconnect_timer) {
        xTimerStop(s_wifi_reconnect_timer, 0);
        xTimerStart(s_wifi_reconnect_timer, 0);
    }
    return err;
}

// Schreibt Defaults aus credentials.h einmalig in den NVS, sofern dort noch
// keine SSID hinterlegt ist (Seed-on-empty). overwrite=true zwingt das
// Überschreiben — wird aktuell nirgends gebraucht, aber nützlich falls man
// später in einem Service-Modus die Werte zurücksetzen will.
static bool seedWifiCredentials(bool overwrite)
{
    const char* defSsid = WIFI_SSID;
    const char* defPass = WIFI_PASSWORD;
    if (defSsid[0] == '\0') {
        // credentials.h fehlt oder leer — nichts zu tun. Das Gerät verbleibt
        // im "warten auf Pairing"-Modus bis Matter-Network-Commissioning oder
        // WebUI die Daten setzt.
        ESP_LOGI(TAG, "WiFi-Seed: WIFI_SSID leer — überspringe Default-Seed");
        return false;
    }

    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs) != ESP_OK) {
        ESP_LOGE(TAG, "WiFi-Seed: NVS-Open fehlgeschlagen");
        return false;
    }

    bool wrote = false;
    char  cur[65] = {0};
    size_t cl    = sizeof(cur);
    bool ssidEmpty = (nvs_get_str(nvs, NVS_KEY_WIFI_SSID, cur, &cl) != ESP_OK
                      || cur[0] == '\0');
    if (overwrite || ssidEmpty) {
        if (nvs_set_str(nvs, NVS_KEY_WIFI_SSID, defSsid) == ESP_OK) wrote = true;
        if (nvs_set_str(nvs, NVS_KEY_WIFI_PASS, defPass) == ESP_OK) wrote = true;
        ESP_LOGI(TAG, "================== WiFi-Seed ==================");
        ESP_LOGI(TAG, " NVS aus credentials.h befüllt: SSID='%s'", defSsid);
        ESP_LOGI(TAG, "===============================================");
    } else {
        ESP_LOGI(TAG, "================ WiFi-NVS  =================");
        ESP_LOGI(TAG, " bereits gefüllt: SSID='%s' (NVS hat Vorrang)", cur);
        ESP_LOGI(TAG, "============================================");
    }
    if (wrote) nvs_commit(nvs);
    nvs_close(nvs);
    return wrote;
}

// =============================================================================
//  Onboarding-Cache befüllen (einmalig nach esp_matter::start)
//
//  Verwendet die Helper aus app/server/OnboardingCodesUtil.h, die intern
//  SetupPayload + QR-/Manual-Generator korrekt konstruieren — robust gegen
//  esp-matter-Versionswechsel.
// =============================================================================
static void cacheOnboardingCodes()
{
    using chip::MutableCharSpan;
    const auto rendezvous = chip::RendezvousInformationFlags(
        chip::RendezvousInformationFlag::kBLE);

    {
        MutableCharSpan span(s_qr_code, sizeof(s_qr_code) - 1);
        if (GetQRCode(span, rendezvous) == CHIP_NO_ERROR) {
            s_qr_code[span.size()] = '\0';
        } else {
            s_qr_code[0] = '\0';
        }
    }
    {
        MutableCharSpan span(s_pairing_code, sizeof(s_pairing_code) - 1);
        if (GetManualPairingCode(span, rendezvous) == CHIP_NO_ERROR) {
            s_pairing_code[span.size()] = '\0';
        } else {
            s_pairing_code[0] = '\0';
        }
    }

    // Setup-PIN + Discriminator zusätzlich für Diagnose-Anzeige cachen.
    // Der CommissionableDataProvider liest die Werte aus dem chip-factory-NVS.
    if (auto* p = chip::DeviceLayer::GetCommissionableDataProvider()) {
        uint32_t pin  = 0;
        uint16_t disc = 0;
        if (p->GetSetupPasscode(pin)       == CHIP_NO_ERROR) s_setup_pin     = pin;
        if (p->GetSetupDiscriminator(disc) == CHIP_NO_ERROR) s_discriminator = disc;
    }

    ESP_LOGI(TAG, "Setup-PIN     : %08" PRIu32, s_setup_pin);
    ESP_LOGI(TAG, "Discriminator : %u", s_discriminator);
    ESP_LOGI(TAG, "QR-Payload    : %s", s_qr_code[0] ? s_qr_code : "(leer)");
    ESP_LOGI(TAG, "Pairing-Code  : %s", s_pairing_code[0] ? s_pairing_code : "(leer)");
}

// =============================================================================
//  Public API
// =============================================================================
namespace MatterBridge {

bool init()
{
    ESP_LOGI(TAG, "Phase 1: Node + Endpoints anlegen...");

    // WiFi-Credentials aus credentials.h beim ersten Boot ins NVS spiegeln.
    // So steht beim allerersten Start direkt nach dem Flashen schon eine
    // gültige SSID/PSK im NVS bereit (analog ESP32-PoolMaster Matter_dev).
    seedWifiCredentials(/*overwrite=*/false);

    esp_matter::node::config_t nodeConfig;
    esp_matter::node_t* node = esp_matter::node::create(
        &nodeConfig, attributeCallback, identifyCallback);
    if (!node) {
        ESP_LOGE(TAG, "esp_matter::node::create() fehlgeschlagen!");
        return false;
    }

    esp_err_t err = MatterDevices::init(node);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "MatterDevices::init() fehlgeschlagen: %s", esp_err_to_name(err));
        return false;
    }

    ESP_LOGI(TAG, "Phase 1 abgeschlossen.");
    return true;
}

bool start()
{
    ESP_LOGI(TAG, "Phase 2: esp_matter::start()...");
    ESP_LOGI(TAG, "RAM vor Matter-Start: %u B frei, groesster Block: %u B",
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));

    esp_err_t err = esp_matter::start(deviceEventCallback);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_matter::start() fehlgeschlagen: %s", esp_err_to_name(err));
        return false;
    }
    s_started = true;

    // SupportedModes nach NVS-Restore zurücksetzen (Workaround in MatterDevices)
    MatterDevices::postStart();

    // Onboarding-Codes lesen + im Log + Cache ablegen
    cacheOnboardingCodes();
    PrintOnboardingCodes(chip::RendezvousInformationFlags(
        chip::RendezvousInformationFlag::kBLE));

    // Initialer Commissioning-Status für Display + WebUI
    Display::isCommissioned =
        chip::Server::GetInstance().GetFabricTable().FabricCount() > 0;

    if (Display::isCommissioned) {
        ESP_LOGI(TAG, "Bereits commissioned (Fabrics=%u) — Normal-Betrieb",
                 chip::Server::GetInstance().GetFabricTable().FabricCount());
        // Subscription unmittelbar starten, falls PoolMaster bereits eingerichtet
        restorePoolMasterSubscription();
    } else {
        ESP_LOGI(TAG, "Noch nicht commissioned — wartet auf Pairing");
    }

    // IP-/WiFi-Event-Handler SOFORT nach esp_matter::start() registrieren
    // (dort wird esp_wifi_init() + esp_netif_init() bereits erledigt).
    // Muss vor connectWifiInternal() passieren, damit ein eventuell sehr
    // frühes IP_EVENT_STA_GOT_IP nicht verpasst wird.
    registerWifiHandlersOnce();

    // Direkter WiFi-Login mit den NVS-Credentials.
    // Wenn bereits über Matter-Network-Commissioning Daten provisioniert
    // wurden, überschreibt esp_wifi_set_config() diese — das ist gewünscht,
    // damit unser persistierter Wert (NVS / WebUI) immer Vorrang hat.
    // Bei leerer SSID (kein credentials.h) tut die Funktion nichts und das
    // Gerät bleibt im Pairing-Modus.
    connectWifiInternal();

    return true;
}

uint8_t fabricCount()
{
    if (!s_started) return 0;
    return chip::Server::GetInstance().GetFabricTable().FabricCount();
}

bool getQRCode(char* buf, size_t size)
{
    if (!buf || size < 2 || s_qr_code[0] == '\0') return false;
    snprintf(buf, size, "%s", s_qr_code);
    return true;
}

bool getManualPairingCode(char* buf, size_t size)
{
    if (!buf || size < 2 || s_pairing_code[0] == '\0') return false;
    snprintf(buf, size, "%s", s_pairing_code);
    return true;
}

uint32_t getSetupPin()      { return s_setup_pin; }
uint16_t getDiscriminator() { return s_discriminator; }

// ScheduleWork-Callback: muss aus dem CHIP-Task laufen
static void openWindowWork(intptr_t arg)
{
    const uint16_t timeoutSec = static_cast<uint16_t>(arg);
    auto err = chip::Server::GetInstance().GetCommissioningWindowManager()
        .OpenBasicCommissioningWindow(chip::System::Clock::Seconds32(timeoutSec));
    if (err != CHIP_NO_ERROR) {
        ESP_LOGE(TAG, "OpenBasicCommissioningWindow failed: %" CHIP_ERROR_FORMAT, err.Format());
    } else {
        ESP_LOGI(TAG, "Commissioning-Fenster geöffnet (%u s)", timeoutSec);
    }
}

bool openCommissioningWindow(uint16_t timeoutSec)
{
    if (!s_started) return false;
    if (timeoutSec > 900) timeoutSec = 900;
    auto err = chip::DeviceLayer::PlatformMgr().ScheduleWork(
        openWindowWork, static_cast<intptr_t>(timeoutSec));
    if (err != CHIP_NO_ERROR) {
        ESP_LOGE(TAG, "ScheduleWork(openWindow) failed: %" CHIP_ERROR_FORMAT, err.Format());
        return false;
    }
    return true;
}

void factoryReset()
{
    ESP_LOGW(TAG, "Matter Factory-Reset angefordert!");
    Display::showResetWarning();
    vTaskDelay(pdMS_TO_TICKS(2000));   // statt Arduino's delay() (kein <Arduino.h>!)
    esp_matter::factory_reset();   // löst intern Reboot aus
}

bool consumeWifiUpEvent()
{
    if (!s_wifi_up_event) return false;
    s_wifi_up_event = false;
    return true;
}

// ---------------------------------------------------------------------------
// WiFi-Direktanmeldung — Public Wrapper
// ---------------------------------------------------------------------------

bool connectWifi()
{
    if (!s_started) {
        ESP_LOGW(TAG, "connectWifi: esp_matter::start() wurde noch nicht ausgeführt");
        return false;
    }
    return connectWifiInternal() == ESP_OK;
}

bool setWifiCredentials(const char* ssid, const char* pass)
{
    if (!ssid || ssid[0] == '\0') {
        ESP_LOGW(TAG, "setWifiCredentials: leere SSID — abgelehnt");
        return false;
    }
    if (strlen(ssid) > 32 || (pass && strlen(pass) > 64)) {
        ESP_LOGW(TAG, "setWifiCredentials: SSID/Pass zu lang (max 32/64)");
        return false;
    }

    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs) != ESP_OK) {
        ESP_LOGE(TAG, "setWifiCredentials: NVS-Open fehlgeschlagen");
        return false;
    }
    nvs_set_str(nvs, NVS_KEY_WIFI_SSID, ssid);
    nvs_set_str(nvs, NVS_KEY_WIFI_PASS, pass ? pass : "");
    esp_err_t err = nvs_commit(nvs);
    nvs_close(nvs);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "setWifiCredentials: NVS-Commit -> %s", esp_err_to_name(err));
        return false;
    }
    ESP_LOGI(TAG, "WiFi-Credentials aktualisiert (SSID='%s') — reconnect…", ssid);

    // Disconnect + Reconnect mit neuen Credentials. Failure ist hier nicht
    // hart, da der nächste Reconnect-Timer ohnehin greift.
    if (s_started) {
        esp_wifi_disconnect();
        connectWifiInternal();
    }
    return true;
}

bool seedWifiCredentialsFromDefaults(bool overwrite)
{
    return seedWifiCredentials(overwrite);
}

// ---------------------------------------------------------------------------
// Diagnose-Snapshot — wird von "WIFI_STATUS"-Kommando aufgerufen
// ---------------------------------------------------------------------------

WifiStatus getWifiStatus()
{
    WifiStatus s = {};
    s.fabricCount = fabricCount();
    strncpy(s.ip, "0.0.0.0", sizeof(s.ip));

    // SSID aus NVS lesen — was wir tatsächlich beim nächsten Connect verwenden
    {
        nvs_handle_t nvs;
        if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs) == ESP_OK) {
            size_t l = sizeof(s.ssid);
            nvs_get_str(nvs, NVS_KEY_WIFI_SSID, s.ssid, &l);
            nvs_close(nvs);
        }
    }

    wifi_mode_t mode = WIFI_MODE_NULL;
    if (esp_wifi_get_mode(&mode) == ESP_OK) {
        s.staActive = (mode == WIFI_MODE_STA || mode == WIFI_MODE_APSTA);
    }

    esp_netif_t* netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    esp_netif_ip_info_t ipInfo{};
    if (netif && esp_netif_get_ip_info(netif, &ipInfo) == ESP_OK
        && ipInfo.ip.addr != 0)
    {
        s.connected = true;
        snprintf(s.ip, sizeof(s.ip), IPSTR, IP2STR(&ipInfo.ip));
    }

    wifi_ap_record_t apInfo{};
    if (s.connected && esp_wifi_sta_get_ap_info(&apInfo) == ESP_OK) {
        s.rssi = apInfo.rssi;
    }

    return s;
}

} // namespace MatterBridge
