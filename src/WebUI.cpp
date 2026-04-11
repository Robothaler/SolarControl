#include "WebUI.h"
#include "SolarLogic.h"
#include "Display.h"
#include "Config.h"

#include <esp_http_server.h>
#include <esp_log.h>
#include <esp_task_wdt.h>
#include <esp_matter.h>
#include <Update.h>
#include <ArduinoJson.h>
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Generated GZIP header – produced by scripts/compress_ui.py before each build
// Located in include/index_html_gz.h (auto-generated, in .gitignore)
#include <index_html_gz.h>

static const char* TAG = "WebUI";

// ─── Credentials (override in /home/robothaler/credentials/credentials.h) ─────
#ifndef WEBUI_USERNAME
  #define WEBUI_USERNAME "admin"
#endif
#ifndef WEBUI_PASSWORD
  #define WEBUI_PASSWORD "solarcontrol"
#endif

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
    portENTER_CRITICAL(&_ws_mux);
    for (int i = 0; i < MAX_WS_CLIENTS; i++) {
        if (_ws_fds[i] == fd) { _ws_fds[i] = -1; break; }
    }
    portEXIT_CRITICAL(&_ws_mux);
}

// ─── Pending commands (HTTP task → Arduino loop) ──────────────────────────────
struct PendingCmd {
    int8_t mode        = -1;  // -1 = no change; 0=AUTO 1=POOL 2=BOILER
    int8_t pump        = -1;
    int8_t valve       = -1;
    int8_t circ        = -1;
    int8_t illum       = -1;
    int8_t motionPower = -1;
    bool   factoryReset = false;

    void clear() {
        mode = pump = valve = circ = illum = motionPower = -1;
        factoryReset = false;
    }
};

static PendingCmd   _pending;
static portMUX_TYPE _cmd_mux = portMUX_INITIALIZER_UNLOCKED;

// ─── Basic Auth ───────────────────────────────────────────────────────────────
static const char BASE64[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

static bool base64_match(const char* encoded, const char* expected) {
    char decoded[128] = {0};
    int  out = 0;
    uint32_t val  = 0;
    int      bits = 0;
    for (int i = 0; encoded[i] && encoded[i] != '=' && out < 127; i++) {
        const char* p = strchr(BASE64, encoded[i]);
        if (!p) continue;
        val = (val << 6) | (p - BASE64);
        bits += 6;
        if (bits >= 8) { decoded[out++] = (val >> (bits - 8)) & 0xFF; bits -= 8; }
    }
    decoded[out] = '\0';
    return strcmp(decoded, expected) == 0;
}

static bool auth_ok(httpd_req_t* req) {
    char hdr[128];
    if (httpd_req_get_hdr_value_str(req, "Authorization", hdr, sizeof(hdr)) != ESP_OK)
        return false;
    if (strncmp(hdr, "Basic ", 6) != 0) return false;
    char expected[96];
    snprintf(expected, sizeof(expected), "%s:%s", WEBUI_USERNAME, WEBUI_PASSWORD);
    return base64_match(hdr + 6, expected);
}

static esp_err_t auth_challenge(httpd_req_t* req) {
    httpd_resp_set_status(req, "401 Unauthorized");
    httpd_resp_set_hdr(req, "WWW-Authenticate", "Basic realm=\"SolarControl\"");
    return httpd_resp_sendstr(req, "Unauthorized");
}

// ─── Status JSON builder ──────────────────────────────────────────────────────
static void build_json(char* buf, size_t len) {
    const SolarLogic::State& s = SolarLogic::state;
    const char* mode =
        s.currentMode == SolarLogic::Mode::AUTO         ? "AUTO"         :
        s.currentMode == SolarLogic::Mode::MANUAL_POOL  ? "MANUAL_POOL"  :
                                                          "MANUAL_BOILER";
    snprintf(buf, len,
        "{"
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
        "\"wifiIP\":\"%s\","
        "\"uptime\":%lu,"
        "\"freeHeap\":%u,"
        "\"version\":\"%s\""
        "}",
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
        Display::wifiIP,
        millis() / 1000UL,
        (unsigned)ESP.getFreeHeap(),
        APP_VERSION
    );
}

// ─── HTTP Handlers ────────────────────────────────────────────────────────────

// GET /  →  compressed HTML
static esp_err_t root_handler(httpd_req_t* req) {
    if (!auth_ok(req)) return auth_challenge(req);
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
    httpd_resp_set_hdr(req, "Cache-Control",    "no-cache");
    return httpd_resp_send(req,
        reinterpret_cast<const char*>(index_html_gz),
        static_cast<ssize_t>(index_html_gz_len));
}

// GET /api/status  →  JSON
static esp_err_t status_handler(httpd_req_t* req) {
    if (!auth_ok(req)) return auth_challenge(req);
    char buf[640];
    build_json(buf, sizeof(buf));
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, buf);
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
        if (!auth_ok(req)) return auth_challenge(req);
        ws_add(httpd_req_to_sockfd(req));
        ESP_LOGI(TAG, "WS client connected fd=%d", httpd_req_to_sockfd(req));
        // Send initial status
        char buf[640];
        build_json(buf, sizeof(buf));
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

    portENTER_CRITICAL(&_cmd_mux);
    if      (strcmp(c, "set_pump")         == 0) _pending.pump        = val.as<bool>() ? 1 : 0;
    else if (strcmp(c, "set_valve")        == 0) _pending.valve       = val.as<bool>() ? 1 : 0;
    else if (strcmp(c, "set_circ")         == 0) _pending.circ        = val.as<bool>() ? 1 : 0;
    else if (strcmp(c, "set_illum")        == 0) _pending.illum       = val.as<bool>() ? 1 : 0;
    else if (strcmp(c, "set_motion_power") == 0) _pending.motionPower = val.as<bool>() ? 1 : 0;
    else if (strcmp(c, "factory_reset")    == 0) _pending.factoryReset = true;
    else if (strcmp(c, "set_mode")         == 0) {
        const char* m = val | "";
        if      (strcmp(m, "AUTO")         == 0) _pending.mode = 0;
        else if (strcmp(m, "MANUAL_POOL")  == 0) _pending.mode = 1;
        else if (strcmp(m, "MANUAL_BOILER")== 0) _pending.mode = 2;
    }
    portEXIT_CRITICAL(&_cmd_mux);

    // Echo current status back immediately
    char resp[640];
    build_json(resp, sizeof(resp));
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
    if (!auth_ok(req)) return auth_challenge(req);

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
static char _bcast_buf[640];   // static: valid for async send lifetime

static void broadcast_task(void* /*arg*/) {
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(2000));
        if (!_server) continue;

        build_json(_bcast_buf, sizeof(_bcast_buf));

        httpd_ws_frame_t frame{};
        frame.type    = HTTPD_WS_TYPE_TEXT;
        frame.payload = reinterpret_cast<uint8_t*>(_bcast_buf);
        frame.len     = strlen(_bcast_buf);

        // Snapshot FD list to avoid holding spinlock during sends
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
}

// ─── Public API ───────────────────────────────────────────────────────────────
namespace WebUI {

void begin() {
    for (int i = 0; i < MAX_WS_CLIENTS; i++) _ws_fds[i] = -1;
    _pending.clear();

    httpd_config_t cfg  = HTTPD_DEFAULT_CONFIG();
    cfg.server_port     = WEBUI_PORT;
    cfg.max_open_sockets = MAX_WS_CLIENTS + 2;
    cfg.max_uri_handlers = 8;
    cfg.stack_size       = 8192;
    cfg.lru_purge_enable = true;

    if (httpd_start(&_server, &cfg) != ESP_OK) {
        ESP_LOGE(TAG, "httpd_start() fehlgeschlagen!");
        return;
    }

    // Register URI handlers
    httpd_uri_t uri_root   = { "/",           HTTP_GET,  root_handler,   nullptr };
    httpd_uri_t uri_status = { "/api/status", HTTP_GET,  status_handler, nullptr };
    httpd_uri_t uri_fav    = { "/favicon.ico",HTTP_GET,  favicon_handler,nullptr };
    httpd_uri_t uri_upd    = { "/update",     HTTP_POST, update_handler, nullptr };

    httpd_uri_t uri_ws;
    memset(&uri_ws, 0, sizeof(uri_ws));
    uri_ws.uri           = "/ws";
    uri_ws.method        = HTTP_GET;
    uri_ws.handler       = ws_handler;
    uri_ws.is_websocket  = true;

    httpd_register_uri_handler(_server, &uri_root);
    httpd_register_uri_handler(_server, &uri_status);
    httpd_register_uri_handler(_server, &uri_fav);
    httpd_register_uri_handler(_server, &uri_upd);
    httpd_register_uri_handler(_server, &uri_ws);

    // Broadcast task – low priority, 4 kB stack
    xTaskCreate(broadcast_task, "webui_bcast", 4096, nullptr, 2, nullptr);

    ESP_LOGI(TAG, "WebUI gestartet auf Port %d (Benutzer: %s)", WEBUI_PORT, WEBUI_USERNAME);
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
    if (cmd.factoryReset) {
        ESP_LOGW(TAG, "Factory Reset via WebUI angefordert!");
        Display::showResetWarning();
        delay(2000);
        esp_matter::factory_reset();
    }
}

} // namespace WebUI
