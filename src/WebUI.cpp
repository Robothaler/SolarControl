#include "WebUI.h"
#include "SolarLogic.h"
#include "Display.h"
#include "Config.h"
#include "MatterBridge.h"
#include "WebSerial.h"

#include <esp_http_server.h>
#include <esp_log.h>
#include <esp_task_wdt.h>
#include <esp_chip_info.h>
#include <esp_flash.h>
#include <esp_system.h>
#include <esp_app_format.h>
#include <esp_ota_ops.h>
#include <esp_mac.h>
#include <esp_heap_caps.h>
#include <esp_wifi.h>     // esp_wifi_sta_get_ap_info, wifi_ap_record_t
#include <Update.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <Arduino.h>
#include <nvs.h>
#include <time.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Generated GZIP header – produced by scripts/compress_ui.py before each build
// Located in include/index_html_gz.h (auto-generated, in .gitignore)
#include <index_html_gz.h>

static const char* TAG = "WebUI";

// ─── Auth ────────────────────────────────────────────────────────────────────
// Basic-Auth wurde bewusst entfernt:
//   • viele Browser senden den Authorization-Header beim WebSocket-Upgrade
//     NICHT mit (auch wenn die Hauptseite ihn hatte) → /ws scheitert mit 401,
//     UI bleibt leer trotz funktionierendem HTTP-Server.
//   • Das Gerät läuft im internen Netz hinter dem Router. Schutz erfolgt
//     auf Netzwerkebene (kein Port-Forwarding!).
// WEBUI_USERNAME/_PASSWORD aus credentials.h bleiben definiert, werden hier
// aber nicht mehr ausgewertet.

// ─── Server handle ────────────────────────────────────────────────────────────
static httpd_handle_t _server = nullptr;

// ─── WebSocket client FD list ─────────────────────────────────────────────────
static constexpr int MAX_WS_CLIENTS = 4;
static int           _ws_fds[MAX_WS_CLIENTS];
static portMUX_TYPE  _ws_mux = portMUX_INITIALIZER_UNLOCKED;

static void ws_add(int fd) {
    portENTER_CRITICAL(&_ws_mux);
    for (int i = 0; i < MAX_WS_CLIENTS; i++) {
        if (_ws_fds[i] < 0) { _ws_fds[i] = fd; break; }
    }
    portEXIT_CRITICAL(&_ws_mux);
}

static void ws_remove(int fd) {
    bool anyLeft = false;
    portENTER_CRITICAL(&_ws_mux);
    for (int i = 0; i < MAX_WS_CLIENTS; i++) {
        if (_ws_fds[i] == fd) { _ws_fds[i] = -1; }
        if (_ws_fds[i] >= 0) { anyLeft = true; }
    }
    portEXIT_CRITICAL(&_ws_mux);

    // Sicherheitsnetz: wenn der letzte Client geht, WebSerial automatisch
    // stoppen — sonst bleibt der vprintf-Hook aktiv und der Ring-Buffer
    // läuft im Hintergrund voll (kein Lauschen mehr).
    if (!anyLeft && WebSerial::isActive()) {
        WebSerial::stop();
    }
}

// ─── Pending commands (HTTP task → Arduino loop) ──────────────────────────────
struct PendingCmd {
    int8_t mode             = -1;  // -1 = no change; 0=AUTO 1=POOL 2=BOILER
    int8_t pump             = -1;
    int8_t valve            = -1;
    int8_t circ             = -1;
    int8_t illum            = -1;
    int8_t motionPower      = -1;
    int8_t webSerial        = -1;  // -1 = unverändert, 0 = stop, 1 = start
    bool   factoryReset     = false;
    bool   openCommissioning = false;   // Matter Commissioning-Fenster (15 min)
    bool   reboot           = false;

    void clear() {
        mode = pump = valve = circ = illum = motionPower = webSerial = -1;
        factoryReset = openCommissioning = reboot = false;
    }
};

static PendingCmd   _pending;
static portMUX_TYPE _cmd_mux = portMUX_INITIALIZER_UNLOCKED;

// ─── JSON-String-Escape (für WebSerial-Output-Frames) ────────────────────────
// Schreibt eine RFC-8259-konforme JSON-String-Repräsentation OHNE umschließende
// Anführungszeichen in `out`. Liefert die Anzahl geschriebener Bytes (ohne NUL).
// Wird vor allem für Roh-Log-Zeilen mit \n, \r, ANSI-Escapes etc. gebraucht.
static size_t json_escape(const char* in, size_t in_len, char* out, size_t out_cap)
{
    if (!in || !out || out_cap < 2) { if (out_cap) out[0] = '\0'; return 0; }
    size_t j = 0;
    const size_t LIMIT = out_cap - 1;  // Platz für NUL

    for (size_t i = 0; i < in_len; i++) {
        unsigned char c = (unsigned char)in[i];
        // Worst-case 6 Bytes (\uXXXX) — vorab prüfen
        if (j + 6 > LIMIT) break;
        switch (c) {
            case '"':  out[j++] = '\\'; out[j++] = '"';  break;
            case '\\': out[j++] = '\\'; out[j++] = '\\'; break;
            case '\n': out[j++] = '\\'; out[j++] = 'n';  break;
            case '\r': out[j++] = '\\'; out[j++] = 'r';  break;
            case '\t': out[j++] = '\\'; out[j++] = 't';  break;
            case '\b': out[j++] = '\\'; out[j++] = 'b';  break;
            case '\f': out[j++] = '\\'; out[j++] = 'f';  break;
            default:
                if (c < 0x20) {
                    j += snprintf(out + j, out_cap - j, "\\u%04x", c);
                } else {
                    out[j++] = (char)c;
                }
        }
    }
    out[j] = '\0';
    return j;
}

// ─── Reset reason → string ────────────────────────────────────────────────────
static const char* reset_reason_str(esp_reset_reason_t r) {
    switch (r) {
        case ESP_RST_POWERON:  return "POWERON";
        case ESP_RST_EXT:      return "EXT";
        case ESP_RST_SW:       return "SW";
        case ESP_RST_PANIC:    return "PANIC";
        case ESP_RST_INT_WDT:  return "INT_WDT";
        case ESP_RST_TASK_WDT: return "TASK_WDT";
        case ESP_RST_WDT:      return "WDT";
        case ESP_RST_DEEPSLEEP:return "DEEPSLEEP";
        case ESP_RST_BROWNOUT: return "BROWNOUT";
        case ESP_RST_SDIO:     return "SDIO";
        default:               return "UNKNOWN";
    }
}

// Cache für statische System-Info (wird einmalig in begin() gefüllt)
static char     _sys_chip_model[16] = "ESP32";
static uint32_t _sys_flash_mb       = 0;
static uint8_t  _sys_chip_rev       = 0;
static uint8_t  _sys_cpu_cores      = 0;
static char     _sys_mac[18]        = "00:00:00:00:00:00";

static void cache_system_info()
{
    esp_chip_info_t info;
    esp_chip_info(&info);
    _sys_chip_rev  = info.revision;
    _sys_cpu_cores = info.cores;
    switch (info.model) {
        case CHIP_ESP32:    snprintf(_sys_chip_model, sizeof(_sys_chip_model), "ESP32");    break;
        case CHIP_ESP32S2:  snprintf(_sys_chip_model, sizeof(_sys_chip_model), "ESP32-S2"); break;
        case CHIP_ESP32S3:  snprintf(_sys_chip_model, sizeof(_sys_chip_model), "ESP32-S3"); break;
        case CHIP_ESP32C3:  snprintf(_sys_chip_model, sizeof(_sys_chip_model), "ESP32-C3"); break;
        case CHIP_ESP32H2:  snprintf(_sys_chip_model, sizeof(_sys_chip_model), "ESP32-H2"); break;
        default:            snprintf(_sys_chip_model, sizeof(_sys_chip_model), "ESP32-?");  break;
    }

    uint32_t flashSize = 0;
    if (esp_flash_get_size(nullptr, &flashSize) == ESP_OK) {
        _sys_flash_mb = flashSize / (1024 * 1024);
    }

    uint8_t mac[6];
    if (esp_read_mac(mac, ESP_MAC_WIFI_STA) == ESP_OK) {
        snprintf(_sys_mac, sizeof(_sys_mac),
                 "%02X:%02X:%02X:%02X:%02X:%02X",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    }
}

// ─── Status JSON builder ──────────────────────────────────────────────────────
//
// Hinweis: WebUI braucht zwei JSON-Sichten:
//   • build_json()        → Hauptstatus (Sensoren + Aktoren, ~700 B)
//   • build_system_json() → Einmal-Snapshot Chip/Flash/MAC/Heap-Min (~400 B)
//   • build_matter_json() → Matter-Commissioning-Status (~250 B)
//
// Aufgeteilt damit der periodische Broadcast (alle 2 s) nicht unnötig die teuren
// System-Felder mitsendet. WebUI ruft build_system_json/build_matter_json on
// demand via WS-Kommando bzw. /api/status?include=full.
//
static void build_json(char* buf, size_t len, bool include_full) {
    const SolarLogic::State& s = SolarLogic::state;
    const char* mode =
        s.currentMode == SolarLogic::Mode::AUTO         ? "AUTO"         :
        s.currentMode == SolarLogic::Mode::MANUAL_POOL  ? "MANUAL_POOL"  :
                                                          "MANUAL_BOILER";

    // RSSI direkt aus dem ESP-IDF-Stack lesen — esp_wifi_sta_get_ap_info
    // ist auch dann korrekt, wenn wir den Connect via esp_wifi_set_config()
    // (nicht über die Arduino-WiFi-Klasse) gemacht haben.
    int rssi = 0;
    bool linkUp = (Display::wifiConnected && Display::wifiIP[0] != '-');
    if (linkUp) {
        wifi_ap_record_t ap{};
        if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) rssi = ap.rssi;
    }

    int n = snprintf(buf, len,
        "{"
        "\"type\":\"status\","
        "\"roofTemp\":%.2f,"
        "\"boilerTemp\":%.2f,"
        "\"storageTemp\":%.2f,"
        "\"backflowTemp\":%.2f,"
        "\"poolTemp\":%.2f,"
        "\"poolSollTemp\":%.2f,"
        "\"pumpRunning\":%s,"
        "\"valvePool\":%s,"
        "\"circulationOn\":%s,"
        "\"illuminationOn\":%s,"
        "\"motionPower\":%s,"
        "\"motionDetected\":%s,"
        "\"valveStatusOK\":%s,"
        "\"levelPct\":%.1f,"
        "\"levelWarn\":%s,"
        "\"mode\":\"%s\","
        "\"commissioned\":%s,"
        "\"fabrics\":%u,"
        "\"wifiIP\":\"%s\","
        "\"rssi\":%d,"
        "\"uptime\":%lu,"
        "\"freeHeap\":%u,"
        "\"webSerial\":%s,"
        "\"version\":\"%s\"",
        s.roofTemp, s.boilerTemp, s.storageTemp, s.backflowTemp,
        s.poolTemp, s.poolSollTemp,
        s.pumpRunning     ? "true" : "false",
        s.valvePool       ? "true" : "false",
        s.circulationOn   ? "true" : "false",
        s.illuminationOn  ? "true" : "false",
        s.motionPowerOn   ? "true" : "false",
        s.motionDetected  ? "true" : "false",
        s.valveStatusOK   ? "true" : "false",
        s.levelPct,
        s.levelWarn       ? "true" : "false",
        mode,
        Display::isCommissioned ? "true" : "false",
        (unsigned)MatterBridge::fabricCount(),
        Display::wifiIP,
        rssi,
        millis() / 1000UL,
        (unsigned)ESP.getFreeHeap(),
        WebSerial::isActive() ? "true" : "false",
        APP_VERSION);

    // Echtzeit-Uhr (lokale Zeit) für UI-Anzeige
    if (n > 0 && (size_t)n < len) {
        time_t now = time(nullptr);
        struct tm tm_info;
        localtime_r(&now, &tm_info);
        char tbuf[32];
        strftime(tbuf, sizeof(tbuf), "%Y-%m-%d %H:%M:%S", &tm_info);
        n += snprintf(buf + n, len - n, ",\"localTime\":\"%s\"", tbuf);
    }

    // Optional: System-Snapshot (Chip/Flash/PSRAM/MAC/HeapMin/Reset) anhängen
    if (include_full && n > 0 && (size_t)n < len) {
        size_t psramFree = 0;
#ifdef BOARD_HAS_PSRAM
        psramFree = ESP.getFreePsram();
#else
        psramFree = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
#endif
        n += snprintf(buf + n, len - n,
            ",\"sys\":{"
            "\"chip\":\"%s\",\"rev\":%u,\"cores\":%u,"
            "\"flashMB\":%u,\"psramFreeKB\":%u,"
            "\"heapMinB\":%u,\"mac\":\"%s\","
            "\"resetReason\":\"%s\""
            "}",
            _sys_chip_model, _sys_chip_rev, _sys_cpu_cores,
            (unsigned)_sys_flash_mb,
            (unsigned)(psramFree / 1024),
            (unsigned)esp_get_minimum_free_heap_size(),
            _sys_mac,
            reset_reason_str(esp_reset_reason()));
    }

    if (n > 0 && (size_t)n < len) {
        snprintf(buf + n, len - n, "}");
    }
}

// Convenience-Wrapper (Default: kompaktes Status-JSON)
static inline void build_json(char* buf, size_t len) { build_json(buf, len, false); }

// Matter-Commissioning-Snapshot (für WS-Kommando "matter_status")
static void build_matter_json(char* buf, size_t len)
{
    char qr[96] = {0}, manual[32] = {0};
    MatterBridge::getQRCode(qr, sizeof(qr));
    MatterBridge::getManualPairingCode(manual, sizeof(manual));

    snprintf(buf, len,
        "{"
        "\"type\":\"matter\","
        "\"commissioned\":%s,"
        "\"fabrics\":%u,"
        "\"vendorId\":%u,"
        "\"productId\":%u,"
        "\"deviceName\":\"%s\","
        "\"setupPin\":\"%08lu\","
        "\"discriminator\":%u,"
        "\"qrCode\":\"%s\","
        "\"manualPairingCode\":\"%s\""
        "}",
        Display::isCommissioned ? "true" : "false",
        (unsigned)MatterBridge::fabricCount(),
        (unsigned)MATTER_VENDOR_ID,
        (unsigned)MATTER_PRODUCT_ID,
        MATTER_DEVICE_NAME,
        (unsigned long)MatterBridge::getSetupPin(),
        (unsigned)MatterBridge::getDiscriminator(),
        qr,
        manual);
}

// ─── HTTP Handlers ────────────────────────────────────────────────────────────

// GET /  →  compressed HTML
static esp_err_t root_handler(httpd_req_t* req) {
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
    httpd_resp_set_hdr(req, "Cache-Control",    "no-cache");
    return httpd_resp_send(req,
        reinterpret_cast<const char*>(index_html_gz),
        static_cast<ssize_t>(index_html_gz_len));
}

// GET /api/status  →  JSON (kompakt; ?full=1 hängt sys-Snapshot an)
static esp_err_t status_handler(httpd_req_t* req) {
    bool full = false;
    char query[32];
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        char val[8];
        if (httpd_query_key_value(query, "full", val, sizeof(val)) == ESP_OK) {
            full = (val[0] == '1' || val[0] == 't');
        }
    }

    char buf[1024];
    build_json(buf, sizeof(buf), full);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, buf);
}

// GET /api/matter  →  Matter-Commissioning-Snapshot
static esp_err_t matter_handler(httpd_req_t* req) {
    char buf[400];
    build_matter_json(buf, sizeof(buf));
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, buf);
}

// =============================================================================
//  Settings-Persistenz (NVS)
//
//  Felder werden im Namespace NVS_NAMESPACE unter den Schlüsseln NVS_KEY_TZ,
//  NVS_KEY_NTP1, NVS_KEY_NTP2, NVS_KEY_AP_FALLBACK abgelegt.
//  Defaults stammen aus Config.h (TIMEZONE_POSIX, NTP_SERVER_PRIMARY/SECONDARY).
//  Werden hier per kleiner Helfer-Klasse gekapselt damit settings_handler kurz
//  bleibt — die WebUI bleibt ein dünner Layer über dem NVS.
// =============================================================================
struct WebSettings {
    char tz[64];
    char ntp1[64];
    char ntp2[64];
    char ssid[33];
    char pass[65];
    bool apFallback;

    void loadDefaults() {
        snprintf(tz,   sizeof(tz),   "%s", TIMEZONE_POSIX);
        snprintf(ntp1, sizeof(ntp1), "%s", NTP_SERVER_PRIMARY);
        snprintf(ntp2, sizeof(ntp2), "%s", NTP_SERVER_SECONDARY);
        ssid[0] = '\0';
        pass[0] = '\0';
        apFallback = true;
    }
};

static void settings_load(WebSettings& s) {
    s.loadDefaults();
    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs) != ESP_OK) return;
    size_t l;
    l = sizeof(s.tz);   nvs_get_str(nvs, NVS_KEY_TZ,        s.tz,   &l);
    l = sizeof(s.ntp1); nvs_get_str(nvs, NVS_KEY_NTP1,      s.ntp1, &l);
    l = sizeof(s.ntp2); nvs_get_str(nvs, NVS_KEY_NTP2,      s.ntp2, &l);
    l = sizeof(s.ssid); nvs_get_str(nvs, NVS_KEY_WIFI_SSID, s.ssid, &l);
    l = sizeof(s.pass); nvs_get_str(nvs, NVS_KEY_WIFI_PASS, s.pass, &l);
    uint8_t ap = 1;
    nvs_get_u8(nvs, NVS_KEY_AP_FALLBACK, &ap);
    s.apFallback = (ap != 0);
    nvs_close(nvs);
}

// Persistiert nur die NICHT-WiFi-Felder (TZ/NTP/AP-Fallback). WiFi-Credentials
// gehen über MatterBridge::setWifiCredentials() — der schreibt das NVS und
// triggert einen Reconnect in einem Rutsch.
static esp_err_t settings_save(const WebSettings& s) {
    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs) != ESP_OK) return ESP_FAIL;
    nvs_set_str(nvs, NVS_KEY_TZ,   s.tz);
    nvs_set_str(nvs, NVS_KEY_NTP1, s.ntp1);
    nvs_set_str(nvs, NVS_KEY_NTP2, s.ntp2);
    nvs_set_u8 (nvs, NVS_KEY_AP_FALLBACK, s.apFallback ? 1 : 0);
    esp_err_t err = nvs_commit(nvs);
    nvs_close(nvs);
    return err;
}

// GET /api/settings → JSON
// Aus Sicherheitsgründen wird das WLAN-Passwort niemals zurück ans Frontend
// gegeben — stattdessen nur ein Boolean "passSet". So lässt sich im UI ein
// Platzhalter "(unverändert)" anzeigen ohne das Klartext-Passwort zu leaken.
static esp_err_t settings_get_handler(httpd_req_t* req) {
    WebSettings s;
    settings_load(s);
    char buf[480];
    snprintf(buf, sizeof(buf),
        "{\"type\":\"settings\","
         "\"tz\":\"%s\",\"ntp1\":\"%s\",\"ntp2\":\"%s\","
         "\"apFallback\":%s,"
         "\"ssid\":\"%s\",\"passSet\":%s}",
        s.tz, s.ntp1, s.ntp2,
        s.apFallback ? "true" : "false",
        s.ssid,
        (s.pass[0] != '\0') ? "true" : "false");
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, buf);
}

// POST /api/settings → akzeptiert JSON-Body { tz, ntp1, ntp2, apFallback }
static esp_err_t settings_post_handler(httpd_req_t* req) {
    int total = req->content_len;
    if (total <= 0 || total > 1024) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid body size");
        return ESP_FAIL;
    }
    char body[1024];
    int  off = 0;
    while (off < total) {
        int r = httpd_req_recv(req, body + off, total - off);
        if (r <= 0) {
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "recv failed");
            return ESP_FAIL;
        }
        off += r;
    }
    body[off] = '\0';

    StaticJsonDocument<512> doc;
    if (deserializeJson(doc, body) != DeserializationError::Ok) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }

    WebSettings s;
    settings_load(s); // start from current values to allow partial updates

    if (doc.containsKey("tz"))         snprintf(s.tz,   sizeof(s.tz),   "%s", doc["tz"  ].as<const char*>());
    if (doc.containsKey("ntp1"))       snprintf(s.ntp1, sizeof(s.ntp1), "%s", doc["ntp1"].as<const char*>());
    if (doc.containsKey("ntp2"))       snprintf(s.ntp2, sizeof(s.ntp2), "%s", doc["ntp2"].as<const char*>());
    if (doc.containsKey("apFallback")) s.apFallback = doc["apFallback"].as<bool>();

    if (settings_save(s) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "NVS write failed");
        return ESP_FAIL;
    }

    // Timezone direkt anwenden — NTP-Server-Wechsel erfordert Reboot
    setenv("TZ", s.tz, 1);
    tzset();

    // ── WiFi-Credentials (optional) ────────────────────────────────────────
    // Schreiben + sofort reconnect über MatterBridge (geht über die korrekte
    // esp_wifi_set_config()/esp_wifi_connect()-Sequenz und respektiert die
    // bestehende Reconnect-Timer-Logik).
    bool wifiChanged = false;
    if (doc.containsKey("ssid") || doc.containsKey("pass")) {
        const char* newSsid = doc.containsKey("ssid") ? doc["ssid"].as<const char*>() : s.ssid;
        // Leeres oder fehlendes Pass-Feld bedeutet "unverändert"; explizit
        // gesetzter Leerstring überschreibt (Open-Networks).
        const char* newPass = s.pass; // default: alter Wert
        if (doc.containsKey("pass")) {
            const char* p = doc["pass"].as<const char*>();
            if (p && *p) newPass = p;
            else if (doc.containsKey("clearPass") && doc["clearPass"].as<bool>())
                newPass = "";
        }
        if (newSsid && *newSsid) {
            wifiChanged = MatterBridge::setWifiCredentials(newSsid, newPass);
        }
    }

    httpd_resp_set_type(req, "application/json");
    char ok[64];
    snprintf(ok, sizeof(ok), "{\"ok\":true,\"wifiReconfigured\":%s}",
             wifiChanged ? "true" : "false");
    return httpd_resp_sendstr(req, ok);
}

// GET /favicon.ico  →  204
static esp_err_t favicon_handler(httpd_req_t* req) {
    httpd_resp_set_status(req, "204 No Content");
    return httpd_resp_send(req, nullptr, 0);
}

// WebSocket /ws  →  real-time status + commands
static esp_err_t ws_handler(httpd_req_t* req) {
    // Initial HTTP handshake
    if (req->method == HTTP_GET) {
        ws_add(httpd_req_to_sockfd(req));
        ESP_LOGI(TAG, "WS client connected fd=%d", httpd_req_to_sockfd(req));
        // Send initial status (mit System-Snapshot, damit das UI direkt alle
        // System-Info-Felder befüllen kann ohne extra Roundtrip).
        char buf[1024];
        build_json(buf, sizeof(buf), true);
        httpd_ws_frame_t frame{};
        frame.type    = HTTPD_WS_TYPE_TEXT;
        frame.payload = reinterpret_cast<uint8_t*>(buf);
        frame.len     = strlen(buf);
        httpd_ws_send_frame(req, &frame);
        return ESP_OK;
    }

    // Receive frame
    httpd_ws_frame_t rx{};
    uint8_t          rxbuf[256] = {0};
    rx.payload = rxbuf;
    esp_err_t ret = httpd_ws_recv_frame(req, &rx, sizeof(rxbuf) - 1);
    if (ret != ESP_OK) {
        ws_remove(httpd_req_to_sockfd(req));
        return ret;
    }
    if (rx.type == HTTPD_WS_TYPE_CLOSE) {
        ws_remove(httpd_req_to_sockfd(req));
        ESP_LOGI(TAG, "WS client disconnected fd=%d", httpd_req_to_sockfd(req));
        return ESP_OK;
    }
    if (rx.type != HTTPD_WS_TYPE_TEXT) return ESP_OK;

    // Parse JSON command
    StaticJsonDocument<128> doc;
    if (deserializeJson(doc, reinterpret_cast<char*>(rxbuf)) != DeserializationError::Ok)
        return ESP_OK;

    const char*  c   = doc["cmd"]   | "";
    JsonVariant  val = doc["value"];
    bool reply_matter = false;
    bool reply_full   = false;

    // WebSerial-Input wird sofort ausserhalb des Spinlocks verarbeitet
    // (pushInput() hat eigenen Mutex). Andere Kommandos landen in _pending.
    if (strcmp(c, "webserial_input") == 0) {
        const char* l = val | "";
        if (l && *l) WebSerial::pushInput(l);
    }

    portENTER_CRITICAL(&_cmd_mux);
    if      (strcmp(c, "set_pump")         == 0) _pending.pump        = val.as<bool>() ? 1 : 0;
    else if (strcmp(c, "set_valve")        == 0) _pending.valve       = val.as<bool>() ? 1 : 0;
    else if (strcmp(c, "set_circ")         == 0) _pending.circ        = val.as<bool>() ? 1 : 0;
    else if (strcmp(c, "set_illum")        == 0) _pending.illum       = val.as<bool>() ? 1 : 0;
    else if (strcmp(c, "set_motion_power") == 0) _pending.motionPower = val.as<bool>() ? 1 : 0;
    else if (strcmp(c, "factory_reset")    == 0) _pending.factoryReset = true;
    else if (strcmp(c, "matter_open_commissioning") == 0) _pending.openCommissioning = true;
    else if (strcmp(c, "reboot")           == 0) _pending.reboot      = true;
    else if (strcmp(c, "matter_status")    == 0) reply_matter = true;
    else if (strcmp(c, "system_info")      == 0) reply_full   = true;
    else if (strcmp(c, "webserial_start")  == 0) _pending.webSerial   = 1;
    else if (strcmp(c, "webserial_stop")   == 0) _pending.webSerial   = 0;
    else if (strcmp(c, "set_mode")         == 0) {
        const char* m = val | "";
        if      (strcmp(m, "AUTO")         == 0) _pending.mode = 0;
        else if (strcmp(m, "MANUAL_POOL")  == 0) _pending.mode = 1;
        else if (strcmp(m, "MANUAL_BOILER")== 0) _pending.mode = 2;
    }
    portEXIT_CRITICAL(&_cmd_mux);

    // Reply gemäß Kommandotyp
    char resp[1024];
    if (reply_matter) {
        build_matter_json(resp, sizeof(resp));
    } else {
        build_json(resp, sizeof(resp), reply_full);
    }
    httpd_ws_frame_t tx{};
    tx.type    = HTTPD_WS_TYPE_TEXT;
    tx.payload = reinterpret_cast<uint8_t*>(resp);
    tx.len     = strlen(resp);
    httpd_ws_send_frame(req, &tx);

    return ESP_OK;
}

// POST /update  →  OTA firmware upload (multipart/form-data)
// The browser JS sends header X-Firmware-Size with the binary size,
// so we can stop reading at exactly the right byte count.
static esp_err_t update_handler(httpd_req_t* req) {
    int total = req->content_len;
    if (total <= 0 || total > 3 * 1024 * 1024) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid size");
        return ESP_FAIL;
    }

    // Optional: exact firmware size from custom header
    char fw_size_str[16] = {0};
    int  fw_bytes = -1;
    if (httpd_req_get_hdr_value_str(req, "X-Firmware-Size",
                                    fw_size_str, sizeof(fw_size_str)) == ESP_OK) {
        fw_bytes = atoi(fw_size_str);
    }

    if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
        ESP_LOGE(TAG, "Update.begin() failed");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Update.begin() failed");
        return ESP_FAIL;
    }

    static uint8_t buf[1024];
    int  received      = 0;
    int  fw_written    = 0;
    bool header_done   = false;

    while (received < total) {
        int to_read = std::min(static_cast<int>(sizeof(buf)), total - received);
        int r = httpd_req_recv(req, reinterpret_cast<char*>(buf), to_read);
        if (r <= 0) {
            Update.abort();
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "recv failed");
            return ESP_FAIL;
        }
        received += r;
        esp_task_wdt_reset();

        if (!header_done) {
            // Find end of multipart part-header (\r\n\r\n)
            const uint8_t* pos = buf;
            const uint8_t* end = buf + r;
            while (pos + 3 < end) {
                if (pos[0] == '\r' && pos[1] == '\n' && pos[2] == '\r' && pos[3] == '\n') {
                    pos += 4;
                    header_done = true;
                    int data_len = end - pos;
                    if (fw_bytes > 0) data_len = std::min(data_len, fw_bytes - fw_written);
                    if (data_len > 0) { Update.write(const_cast<uint8_t*>(pos), data_len); fw_written += data_len; }
                    break;
                }
                pos++;
            }
        } else {
            int data_len = r;
            if (fw_bytes > 0) data_len = std::min(data_len, fw_bytes - fw_written);
            if (data_len > 0) { Update.write(buf, data_len); fw_written += data_len; }
            if (fw_bytes > 0 && fw_written >= fw_bytes) break;
        }
    }

    // Drain remaining request body (multipart footer)
    while (received < total) {
        int r = httpd_req_recv(req, reinterpret_cast<char*>(buf),
                               std::min(static_cast<int>(sizeof(buf)), total - received));
        if (r <= 0) break;
        received += r;
    }

    if (!Update.end(true)) {
        ESP_LOGE(TAG, "Update.end() failed: %s", Update.errorString());
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, Update.errorString());
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "OTA erfolgreich! Neustart...");
    httpd_resp_sendstr(req, "OK");
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
    return ESP_OK;
}

// ─── Periodic status broadcast task ──────────────────────────────────────────
//
// Wird mit einer schnelleren Tickrate (250 ms) gefahren damit das WebSerial-
// Terminal sich responsiv anfühlt. Der teure Status-JSON wird nur alle 8 Ticks
// (≈ 2 s) gebaut + gesendet — die WebSerial-Drain läuft jeden Tick.
//
static char _bcast_buf[1024];   // static: valid for async send lifetime
static char _ws_buf[1024];      // WebSerial-Drain-Frame
static char _ws_json[1280];     // {"type":"webserial","data":"<escaped>"}

static void send_to_all_ws(const char* data, size_t len)
{
    httpd_ws_frame_t frame{};
    frame.type    = HTTPD_WS_TYPE_TEXT;
    frame.payload = reinterpret_cast<uint8_t*>(const_cast<char*>(data));
    frame.len     = len;

    int fds[MAX_WS_CLIENTS];
    portENTER_CRITICAL(&_ws_mux);
    memcpy(fds, _ws_fds, sizeof(fds));
    portEXIT_CRITICAL(&_ws_mux);

    for (int i = 0; i < MAX_WS_CLIENTS; i++) {
        if (fds[i] < 0) continue;
        esp_err_t err = httpd_ws_send_frame_async(_server, fds[i], &frame);
        if (err != ESP_OK) {
            ws_remove(fds[i]);
            ESP_LOGD(TAG, "WS fd=%d removed (send error)", fds[i]);
        }
    }
}

static void broadcast_task(void* /*arg*/) {
    int statusCountdown = 0;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(250));
        if (!_server) continue;

        // 1) WebSerial-Stream — bei jedem Tick falls aktiv UND Daten anliegen
        if (WebSerial::isActive()) {
            size_t n = 0;
            if (WebSerial::drainOutput(_ws_buf, sizeof(_ws_buf), &n) && n > 0) {
                int prefix = snprintf(_ws_json, sizeof(_ws_json),
                                      "{\"type\":\"webserial\",\"data\":\"");
                size_t esc = json_escape(_ws_buf, n,
                                         _ws_json + prefix,
                                         sizeof(_ws_json) - prefix - 3);
                size_t pos = (size_t)prefix + esc;
                _ws_json[pos++] = '"';
                _ws_json[pos++] = '}';
                _ws_json[pos]   = '\0';
                send_to_all_ws(_ws_json, pos);
            }
        }

        // 2) Status-JSON — alle 8 Ticks (≈ 2 s)
        if (--statusCountdown <= 0) {
            statusCountdown = 8;
            build_json(_bcast_buf, sizeof(_bcast_buf), false);
            send_to_all_ws(_bcast_buf, strlen(_bcast_buf));
        }
    }
}

// ─── Public API ───────────────────────────────────────────────────────────────
namespace WebUI {

// Settings beim Start aus NVS laden und SNTP/TZ damit konfigurieren.
// Wird sowohl von WebUI::begin() (nach WiFi-Up) als auch früh in main.cpp
// indirekt erreicht, weil setenv("TZ", ...) idempotent ist.
static void apply_persisted_settings()
{
    WebSettings s;
    settings_load(s);
    setenv("TZ", s.tz, 1);
    tzset();
    ESP_LOGI(TAG, "Settings geladen: TZ=%s NTP1=%s NTP2=%s AP-Fallback=%d",
             s.tz, s.ntp1, s.ntp2, s.apFallback ? 1 : 0);
}

void begin() {
    for (int i = 0; i < MAX_WS_CLIENTS; i++) _ws_fds[i] = -1;
    _pending.clear();

    cache_system_info();
    apply_persisted_settings();
    WebSerial::begin();   // installiert vprintf-Hook (passiv bis start())

    httpd_config_t cfg  = HTTPD_DEFAULT_CONFIG();
    cfg.server_port     = WEBUI_PORT;
    cfg.max_open_sockets = MAX_WS_CLIENTS + 2;
    cfg.max_uri_handlers = 12;
    cfg.stack_size       = 8192;
    cfg.lru_purge_enable = true;

    if (httpd_start(&_server, &cfg) != ESP_OK) {
        ESP_LOGE(TAG, "httpd_start() fehlgeschlagen!");
        return;
    }

    // Register URI handlers
    httpd_uri_t uri_root      = { "/",             HTTP_GET,  root_handler,         nullptr };
    httpd_uri_t uri_status    = { "/api/status",   HTTP_GET,  status_handler,       nullptr };
    httpd_uri_t uri_matter    = { "/api/matter",   HTTP_GET,  matter_handler,       nullptr };
    httpd_uri_t uri_set_get   = { "/api/settings", HTTP_GET,  settings_get_handler, nullptr };
    httpd_uri_t uri_set_post  = { "/api/settings", HTTP_POST, settings_post_handler,nullptr };
    httpd_uri_t uri_fav       = { "/favicon.ico",  HTTP_GET,  favicon_handler,      nullptr };
    httpd_uri_t uri_upd       = { "/update",       HTTP_POST, update_handler,       nullptr };

    httpd_uri_t uri_ws;
    memset(&uri_ws, 0, sizeof(uri_ws));
    uri_ws.uri           = "/ws";
    uri_ws.method        = HTTP_GET;
    uri_ws.handler       = ws_handler;
    uri_ws.is_websocket  = true;

    httpd_register_uri_handler(_server, &uri_root);
    httpd_register_uri_handler(_server, &uri_status);
    httpd_register_uri_handler(_server, &uri_matter);
    httpd_register_uri_handler(_server, &uri_set_get);
    httpd_register_uri_handler(_server, &uri_set_post);
    httpd_register_uri_handler(_server, &uri_fav);
    httpd_register_uri_handler(_server, &uri_upd);
    httpd_register_uri_handler(_server, &uri_ws);

    // Broadcast task – low priority, 4 kB stack
    xTaskCreate(broadcast_task, "webui_bcast", 4096, nullptr, 2, nullptr);

    ESP_LOGI(TAG, "WebUI gestartet auf Port %d (Auth deaktiviert)", WEBUI_PORT);
}

void stop() {
    if (_server) {
        httpd_stop(_server);
        _server = nullptr;
        ESP_LOGI(TAG, "WebUI gestoppt");
    }
}

void processCommands() {
    // Snapshot & clear atomically
    PendingCmd cmd;
    portENTER_CRITICAL(&_cmd_mux);
    cmd = _pending;
    _pending.clear();
    portEXIT_CRITICAL(&_cmd_mux);

    if (cmd.mode >= 0)        SolarLogic::setMode(static_cast<SolarLogic::Mode>(cmd.mode));
    if (cmd.pump >= 0)        SolarLogic::setPump(cmd.pump != 0);
    if (cmd.valve >= 0)       SolarLogic::setValve(cmd.valve != 0);
    if (cmd.circ >= 0)        SolarLogic::setCirculation(cmd.circ != 0);
    if (cmd.illum >= 0)       SolarLogic::setIllumination(cmd.illum != 0);
    if (cmd.motionPower >= 0) SolarLogic::setMotionPower(cmd.motionPower != 0);
    if (cmd.webSerial == 1) {
        ESP_LOGI(TAG, "WebSerial-Stream gestartet via WebUI");
        WebSerial::start();
    } else if (cmd.webSerial == 0) {
        ESP_LOGI(TAG, "WebSerial-Stream gestoppt via WebUI");
        WebSerial::stop();
    }

    if (cmd.openCommissioning) {
        ESP_LOGI(TAG, "Commissioning-Fenster via WebUI angefordert");
        MatterBridge::openCommissioningWindow(900);
    }
    if (cmd.factoryReset) {
        ESP_LOGW(TAG, "Factory Reset via WebUI angefordert!");
        MatterBridge::factoryReset();   // löst intern Reboot aus
    }
    if (cmd.reboot) {
        ESP_LOGW(TAG, "Reboot via WebUI angefordert!");
        delay(500);
        esp_restart();
    }
}

} // namespace WebUI
