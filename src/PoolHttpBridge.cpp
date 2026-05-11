#include "PoolHttpBridge.h"
#include "Config.h"
#include "Display.h"
#include "SolarLogic.h"

#include <Arduino.h>
#include <ArduinoJson.h>
#include <nvs.h>
#include <esp_log.h>
#include <esp_http_client.h>
#include <cstring>

static const char* TAG = "PoolHttpBr";

/** Einheitlicher Pfad — PoolMaster muss gleichen Pfad bereitstellen. */
static constexpr char kReadPath[] = "/api/pool-solar/v1/read";

static char     s_base[140]      = {0};
static char     s_token[80]      = {0};
static uint32_t s_intervalMs     = 5000;
static uint32_t s_lastPollMs     = 0;
static bool     s_lastOk         = false;
static uint32_t s_warnThrottleMs = 0;

bool PoolHttpBridge::isConfigured()
{
    return s_base[0] != '\0';
}

void PoolHttpBridge::loadFromNvs()
{
    s_base[0]     = '\0';
    s_token[0]    = '\0';
    s_intervalMs  = 5000;

    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs) != ESP_OK)
        return;

    size_t l = sizeof(s_base);
    if (nvs_get_str(nvs, NVS_KEY_POOL_HTTP_BASE, s_base, &l) != ESP_OK)
        s_base[0] = '\0';

    l = sizeof(s_token);
    nvs_get_str(nvs, NVS_KEY_POOL_HTTP_TOK, s_token, &l);

    uint32_t iv = 5000;
    if (nvs_get_u32(nvs, NVS_KEY_POOL_HTTP_IV, &iv) == ESP_OK) {
        if (iv >= 1000 && iv <= 600000)
            s_intervalMs = iv;
    }
    nvs_close(nvs);

    if (s_base[0] == '\0') {
        const char* def = POOL_HTTP_BASE_DEFAULT;
        if (def && def[0] != '\0') {
            strncpy(s_base, def, sizeof(s_base) - 1);
            s_base[sizeof(s_base) - 1] = '\0';
        }
    }
}

bool PoolHttpBridge::saveConfig(const char* baseUrl,
                                TokenSaveMode tokenMode,
                                const char* bearerTokenNonNullOnlyIfSetting,
                                uint32_t pollIntervalMs)
{
    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs) != ESP_OK)
        return false;

    esp_err_t e = ESP_OK;

    if (!baseUrl || !baseUrl[0]) {
        esp_err_t e1 = nvs_erase_key(nvs, NVS_KEY_POOL_HTTP_BASE);
        esp_err_t e2 = nvs_erase_key(nvs, NVS_KEY_POOL_HTTP_TOK);
        esp_err_t e3 = nvs_erase_key(nvs, NVS_KEY_POOL_HTTP_IV);
        e              = ESP_OK;
        if (e1 != ESP_OK && e1 != ESP_ERR_NVS_NOT_FOUND)
            e = e1;
        if (e2 != ESP_OK && e2 != ESP_ERR_NVS_NOT_FOUND && e == ESP_OK)
            e = e2;
        if (e3 != ESP_OK && e3 != ESP_ERR_NVS_NOT_FOUND && e == ESP_OK)
            e = e3;
    } else {
        e            = nvs_set_str(nvs, NVS_KEY_POOL_HTTP_BASE, baseUrl);
        uint32_t iv  = pollIntervalMs;
        if (iv < 1000)
            iv = 1000;
        if (iv > 600000)
            iv = 600000;
        e = (e == ESP_OK) ? nvs_set_u32(nvs, NVS_KEY_POOL_HTTP_IV, iv) : e;

        if (e == ESP_OK) {
            if (tokenMode == TokenSaveMode::Clear) {
                esp_err_t et = nvs_erase_key(nvs, NVS_KEY_POOL_HTTP_TOK);
                if (et != ESP_OK && et != ESP_ERR_NVS_NOT_FOUND)
                    e = et;
            } else if (tokenMode == TokenSaveMode::Set && bearerTokenNonNullOnlyIfSetting
                                              && bearerTokenNonNullOnlyIfSetting[0]) {
                e = nvs_set_str(nvs, NVS_KEY_POOL_HTTP_TOK, bearerTokenNonNullOnlyIfSetting);
            }
        }
    }

    if (e == ESP_OK)
        e = nvs_commit(nvs);
    nvs_close(nvs);

    loadFromNvs();
    return e == ESP_OK;
}

bool PoolHttpBridge::lastFetchOk()
{
    return s_lastOk;
}

bool PoolHttpBridge::hasTokenConfigured()
{
    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs) != ESP_OK)
        return false;
    char      buf[4];
    size_t    l = sizeof(buf);
    esp_err_t e = nvs_get_str(nvs, NVS_KEY_POOL_HTTP_TOK, buf, &l);
    nvs_close(nvs);
    return e == ESP_OK || e == ESP_ERR_NVS_INVALID_LENGTH;
}

struct HttpBodyBuf {
    char*  data;
    size_t cap;
    size_t len;
};

static esp_err_t on_http_event(esp_http_client_event_t* ev)
{
    if (ev->event_id != HTTP_EVENT_ON_DATA || ev->data_len <= 0)
        return ESP_OK;
    auto* rb = static_cast<HttpBodyBuf*>(ev->user_data);
    if (!rb || !rb->data || rb->cap <= 1)
        return ESP_OK;
    const size_t maxAdd = (rb->cap > rb->len + 1) ? (rb->cap - 1 - rb->len) : 0;
    if (maxAdd == 0)
        return ESP_OK;
    const size_t chunk = (static_cast<size_t>(ev->data_len) < maxAdd)
                             ? static_cast<size_t>(ev->data_len)
                             : maxAdd;
    memcpy(rb->data + rb->len, ev->data, chunk);
    rb->len += chunk;
    rb->data[rb->len] = '\0';
    return ESP_OK;
}

void PoolHttpBridge::poll()
{
    if (!PoolHttpBridge::isConfigured())
        return;
    // WICHTIG: STA läuft über esp_wifi_* (MatterBridge) — Arduino WiFi.status()
    // bleibt oft WL_DISCONNECTED, obwohl IP da ist (Display::* wird korrekt gesetzt).
    if (!Display::wifiConnected || Display::wifiIP[0] == '\0'
        || Display::wifiIP[0] == '-')
        return;

    const uint32_t now = millis();
    if (now - s_lastPollMs < s_intervalMs)
        return;
    s_lastPollMs = now;

    char url[192];
    if (strlen(s_base) + strlen(kReadPath) + 1 >= sizeof(url)) {
        ESP_LOGW(TAG, "pool_http_base zu lang");
        s_lastOk = false;
        return;
    }
    snprintf(url, sizeof(url), "%s%s", s_base, kReadPath);

    char        body[896];
    HttpBodyBuf rb = {body, sizeof(body), 0};
    body[0]        = '\0';

    esp_http_client_config_t cfg = {};
    cfg.url            = url;
    cfg.event_handler  = on_http_event;
    cfg.user_data      = &rb;
    cfg.timeout_ms     = 5000;
    cfg.buffer_size    = 1024;

    esp_http_client_handle_t hc = esp_http_client_init(&cfg);
    if (!hc) {
        s_lastOk = false;
        return;
    }
    if (s_token[0] != '\0') {
        char auth[120];
        snprintf(auth, sizeof(auth), "Bearer %s", s_token);
        esp_http_client_set_header(hc, "Authorization", auth);
    }

    esp_err_t err       = esp_http_client_perform(hc);
    const int status    = esp_http_client_get_status_code(hc);
    esp_http_client_cleanup(hc);

    if (err != ESP_OK || status != 200) {
        s_lastOk = false;
        const uint32_t t = millis();
        if (t - s_warnThrottleMs > 30000) {
            s_warnThrottleMs = t;
            ESP_LOGW(TAG, "GET %s err=%s HTTP %d", url, esp_err_to_name(err), status);
        }
        return;
    }

    StaticJsonDocument<448> doc;
    if (deserializeJson(doc, body, rb.len) != DeserializationError::Ok) {
        ESP_LOGW(TAG, "JSON parse error");
        s_lastOk = false;
        return;
    }

    float tmp = NAN;
    if (doc.containsKey("poolTemp_C"))
        tmp = doc["poolTemp_C"].as<float>();
    else if (doc.containsKey("poolTemp"))
        tmp = doc["poolTemp"].as<float>();

    float soll = NAN;
    if (doc.containsKey("poolSollTemp_C"))
        soll = doc["poolSollTemp_C"].as<float>();
    else if (doc.containsKey("poolSollTemp"))
        soll = doc["poolSollTemp"].as<float>();
    else if (doc.containsKey("poolSetTemp_C"))
        soll = doc["poolSetTemp_C"].as<float>();

    if (!(tmp == tmp) || !(soll == soll)) {
        ESP_LOGW(TAG,
                 "JSON ohne gültige Pool-Temperaturen "
                 "(erwarte poolTemp_C/poolTemp, poolSollTemp_C/…)");
        s_lastOk = false;
        return;
    }

    bool req = false;
    if (doc.containsKey("solarModeRequest"))
        req = doc["solarModeRequest"].as<bool>();
    else if (doc.containsKey("poolSolarRequest"))
        req = doc["poolSolarRequest"].as<bool>();
    else if (doc.containsKey("solarRequest"))
        req = doc["solarRequest"].as<bool>();

    SolarLogic::onPoolTempReceived(tmp);
    SolarLogic::onPoolSolltempReceived(soll);
    SolarLogic::onPoolModeRequestReceived(req);
    SolarLogic::update();

    ESP_LOGD(TAG, "Pool HTTP: Ist=%.2f Soll=%.2f SolarReq=%s", tmp, soll,
             req ? "JA" : "NEIN");
    s_lastOk = true;
}