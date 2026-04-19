#pragma once
// =============================================================================
//  MatterBridge.h — SolarControl Matter Bridge
// =============================================================================
//  Kapselt die komplette Matter-Lebenszyklus-Logik (Node-Erzeugung, Endpoints,
//  esp_matter::start, Onboarding-Codes, Re-Commissioning, Factory-Reset).
//
//  Architektur (zwei-phasig, analog ESP32-PoolMaster Matter_dev):
//    Phase 1  matterBridgeInit()
//        • esp_matter::node::create() + MatterDevices::init()
//        • Lädt PoolMaster-Subscription-Konfig aus NVS
//        • MUSS aufgerufen werden, BEVOR esp_matter::start() läuft
//
//    Phase 2  matterBridgeStart()
//        • esp_matter::start() (registriert eigenen Device-Event-Callback)
//        • MatterDevices::postStart() (SupportedModes nach NVS-Load setzen)
//        • Liest QR-Code + Manual-Pairing-Code via chip::GetQRCode() /
//          chip::GetManualPairingCode() und cached sie für die WebUI
//        • Startet PoolMaster-Subscription falls bereits commissioned
//
//  Thread-Safety:
//    Die Helfer matterGetQRCode / matterGetManualPairingCode lesen aus einem
//    Cache, der einmalig in matterBridgeStart() geschrieben wird → lock-frei.
//    matterOpenCommissioningWindow() postet via PlatformMgr().ScheduleWork()
//    in den CHIP-Event-Loop und ist damit aus jedem Task aufrufbar.
// =============================================================================

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

namespace MatterBridge {

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

// Phase 1: Node + Endpoints anlegen. Vor esp_matter::start() aufrufen.
// Gibt true zurück wenn Node + alle Endpoints erfolgreich erzeugt wurden.
bool init();

// Phase 2: Matter-Stack starten + Onboarding-Codes cachen.
// Setzt intern Display::isCommissioned anhand FabricTable().FabricCount().
// Gibt true zurück wenn esp_matter::start() ESP_OK liefert.
bool start();

// ---------------------------------------------------------------------------
// Onboarding (für WebUI / Display)
// ---------------------------------------------------------------------------

// Anzahl commissionierter Fabrics (0 = noch nicht eingebunden).
uint8_t fabricCount();

// QR-Code-Payload (z.B. "MT:Y.K9042C00KA0648G00") in den übergebenen Puffer.
// Empfohlene Puffergröße ≥ 96 Bytes. Liefert false wenn Stack noch nicht
// gestartet wurde oder der Cache leer ist.
bool getQRCode(char* buf, size_t size);

// Manueller 11-stelliger Pairing-Code (mit Bindestrichen, z.B. "1234-567-8901").
// Empfohlene Puffergröße ≥ 32 Bytes.
bool getManualPairingCode(char* buf, size_t size);

// 8-stellige Setup-PIN als Zahl (für Logging / Backup-Anzeige).
uint32_t getSetupPin();

// 12-bit Discriminator (für Logging / BLE-Diagnose).
uint16_t getDiscriminator();

// Öffnet ein Basic-Commissioning-Fenster (DNS-SD only, also ohne BLE).
// Damit kann ein bereits commissionertes Gerät erneut von einer App
// gefunden werden, ohne Factory-Reset. timeoutSec = Fensterdauer (max. 900).
// Gibt true zurück wenn das Schedule-Work erfolgreich abgesetzt wurde.
bool openCommissioningWindow(uint16_t timeoutSec = 900);

// Vollständiger Factory-Reset: löscht Fabrics + NVS, danach Reboot.
// Wird typischerweise vom WebUI-Button oder Serial-Kommando aufgerufen.
void factoryReset();

// ---------------------------------------------------------------------------
// WiFi-Up-Event (für Lazy-Init von WebUI/OTA/SNTP in main.cpp)
// ---------------------------------------------------------------------------

// Wird intern auf true gesetzt, sobald die WiFi-Verbindung steht. Liefert true
// genau einmal (consume-on-read) — danach wieder false. Der Aufrufer (main)
// soll WebUI/OTA/SNTP nur ein einziges Mal initialisieren.
bool consumeWifiUpEvent();

// ---------------------------------------------------------------------------
// WiFi-Direktanmeldung (analog ESP32-PoolMaster Matter_dev)
// ---------------------------------------------------------------------------

// Liest SSID + Password aus NVS (Keys NVS_KEY_WIFI_SSID / NVS_KEY_WIFI_PASS),
// fällt auf die Defaults aus credentials.h zurück (WIFI_SSID / WIFI_PASSWORD)
// und triggert einen Connect via esp_wifi_set_config + esp_wifi_connect.
//
// Muss NACH esp_matter::start() laufen (esp_wifi_init wird dort durchgeführt;
// ein vorgezogener Aufruf würde "duplicate netif"-Asserts produzieren).
//
// Wird intern aus MatterBridge::start() angestoßen — kann zusätzlich aus
// der WebUI aufgerufen werden, nachdem neue Credentials geschrieben wurden.
// Liefert true wenn der Connect-Versuch abgesetzt werden konnte.
bool connectWifi();

// Schreibt neue WiFi-Credentials in den NVS und löst sofort connectWifi() aus.
// Liefert true wenn NVS-Write + Connect-Trigger erfolgreich waren.
// SSID darf max. 32 Bytes, Password max. 64 Bytes haben (esp_wifi-Limit).
bool setWifiCredentials(const char* ssid, const char* pass);

// Stellt sicher, dass NVS-Keys WiFi_SSID/PASS gefüllt sind. Werden sie
// erstmals (oder nach Factory-Reset) leer angetroffen, kommen die Defaults
// aus credentials.h hinein. Wird automatisch aus MatterBridge::init()
// aufgerufen — kann von außen erneut getriggert werden um z.B. nach einem
// Update der credentials.h die NVS-Werte einmalig nachzuziehen.
// Liefert true wenn etwas geschrieben wurde, false wenn die Werte schon da
// waren oder beide Defaults leer sind.
bool seedWifiCredentialsFromDefaults(bool overwrite = false);

// ---------------------------------------------------------------------------
// Diagnose
// ---------------------------------------------------------------------------

// Aktueller WiFi-Status: STA-Modus aktiv? IP bezogen? RSSI? SSID?
struct WifiStatus {
    bool     staActive;          // esp_wifi_get_mode liefert STA oder APSTA
    bool     connected;          // STA hat eine IP (≠ 0.0.0.0)
    char     ssid[33];           // konfigurierte SSID (aus NVS)
    char     ip[16];             // dotted IPv4 oder "0.0.0.0"
    int8_t   rssi;               // RSSI in dBm (0 wenn nicht verfügbar)
    uint8_t  fabricCount;        // Matter Fabric-Count
};

// Liefert einen Snapshot des aktuellen WiFi/Matter-Status. Schreibt nicht
// in den NVS und blockiert nicht. Geeignet für Diagnose-Kommandos.
WifiStatus getWifiStatus();

} // namespace MatterBridge
