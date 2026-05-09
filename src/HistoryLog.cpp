#include "HistoryLog.h"
#include "SolarLogic.h"
#include "Display.h"

#include <Arduino.h>
#include <SPIFFS.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>

#include <esp_log.h>
#include <freertos/portmacro.h>
#include <freertos/semphr.h>

static const char* TAG = "HistoryLog";

// ── RAM ring (WebSocket / GET /api/history ohne Query) ───────────────────────
static constexpr uint16_t HIST_CAP            = 240;
static constexpr uint32_t SAMPLE_INTERVAL_MS  = 30000;

struct Sample {
    uint32_t ts;
    float    roof;
    float    boiler;
    float    storage;
    float    backflow;
    float    pool;
    float    poolSoll;
    uint8_t  pump;
    uint8_t  valve;
    uint8_t  circ;
    uint8_t  illum;
};

static Sample           s_buf[HIST_CAP];
static uint16_t         s_count = 0;
static uint16_t         s_head  = 0;
static uint32_t         s_lastSampleMs = 0;
static portMUX_TYPE     s_mux = portMUX_INITIALIZER_UNLOCKED;

// ── SPIFFS append-only log (längerer Verlauf, Delta + Rotation) ─────────────
namespace {

#pragma pack(push, 1)
struct SolarSensorRec {
    uint32_t ts;
    int16_t  roof10;
    int16_t  boiler10;
    int16_t  storage10;
    int16_t  backflow10;
    int16_t  pool10;
    int16_t  poolSoll10;
    uint8_t  actors;
    uint8_t  reserved[3];
};
#pragma pack(pop)
static_assert(sizeof(SolarSensorRec) == 20, "SolarSensorRec");

static constexpr const char* SPIFFS_CUR = "/solc.bin";
static constexpr const char* SPIFFS_BAK = "/solb.bin";

static constexpr size_t SENSOR_MAX_BYTES   = 400UL * 1024;
static constexpr size_t SPIFFS_MIN_FREE    = 80UL * 1024;

static constexpr int16_t TEMP_DELTA_TENTHS = 2;
static constexpr uint32_t MIN_INTERVAL_S   = 60;
static constexpr uint32_t MAX_INTERVAL_S   = 600;

static SemaphoreHandle_t s_spiffsMtx = nullptr;
static SolarSensorRec    s_lastRec{};
static uint32_t          s_lastFlashTs = 0;

static uint8_t actorsMask(const SolarLogic::State& st)
{
    uint8_t m = 0;
    if (st.pumpRunning)     m |= (1u << 0);
    if (st.valvePool)       m |= (1u << 1);
    if (st.circulationOn)   m |= (1u << 2);
    if (st.illuminationOn)  m |= (1u << 3);
    return m;
}

static SolarSensorRec makeRec(const SolarLogic::State& st, uint32_t wallTs)
{
    SolarSensorRec r{};
    r.ts         = wallTs;
    r.roof10     = (int16_t)(st.roofTemp * 10.0f);
    r.boiler10   = (int16_t)(st.boilerTemp * 10.0f);
    r.storage10  = (int16_t)(st.storageTemp * 10.0f);
    r.backflow10 = (int16_t)(st.backflowTemp * 10.0f);
    r.pool10     = (int16_t)(st.poolTemp * 10.0f);
    r.poolSoll10 = (int16_t)(st.poolSollTemp * 10.0f);
    r.actors     = actorsMask(st);
    r.reserved[0] = r.reserved[1] = r.reserved[2] = 0;
    return r;
}

static bool appendBytes(const char* path, const void* data, size_t sz, size_t maxBytes)
{
    if (SPIFFS.totalBytes() - SPIFFS.usedBytes() < SPIFFS_MIN_FREE) {
        ESP_LOGW(TAG, "SPIFFS fast voll, Log-Write übersprungen");
        return false;
    }
    {
        File fr = SPIFFS.open(path, FILE_READ);
        bool rotate = fr && (fr.size() + sz > maxBytes);
        if (fr) fr.close();
        if (rotate) {
            if (SPIFFS.exists(SPIFFS_BAK)) SPIFFS.remove(SPIFFS_BAK);
            SPIFFS.rename(path, SPIFFS_BAK);
            ESP_LOGI(TAG, "Log rotiert → %s", SPIFFS_BAK);
        }
    }
    File f = SPIFFS.open(path, FILE_APPEND);
    if (!f) {
        ESP_LOGW(TAG, "Kann %s nicht anhängen", path);
        return false;
    }
    size_t written = f.write((const uint8_t*)data, sz);
    if (written == sz) {
        f.flush();
    }
    f.close();
    return written == sz;
}

static void spiffsTryRecord(const SolarLogic::State& st)
{
    if (!s_spiffsMtx) return;

    uint32_t nowTs = (uint32_t)time(nullptr);
    if (nowTs < 1704067200UL) return;

    SolarSensorRec cur = makeRec(st, nowTs);
    uint32_t       elapsed =
        (s_lastFlashTs == 0) ? MAX_INTERVAL_S : (nowTs - s_lastFlashTs);

    bool significant =
        (elapsed >= MAX_INTERVAL_S) ||
        (elapsed >= MIN_INTERVAL_S &&
         (std::abs((int)cur.roof10 - (int)s_lastRec.roof10) >= TEMP_DELTA_TENTHS ||
          std::abs((int)cur.boiler10 - (int)s_lastRec.boiler10) >= TEMP_DELTA_TENTHS ||
          std::abs((int)cur.storage10 - (int)s_lastRec.storage10) >= TEMP_DELTA_TENTHS ||
          std::abs((int)cur.backflow10 - (int)s_lastRec.backflow10) >= TEMP_DELTA_TENTHS ||
          std::abs((int)cur.pool10 - (int)s_lastRec.pool10) >= TEMP_DELTA_TENTHS ||
          std::abs((int)cur.poolSoll10 - (int)s_lastRec.poolSoll10) >= TEMP_DELTA_TENTHS ||
          cur.actors != s_lastRec.actors));

    if (s_lastFlashTs == 0) significant = true;
    if (!significant) return;

    if (xSemaphoreTake(s_spiffsMtx, pdMS_TO_TICKS(80)) != pdTRUE) return;
    appendBytes(SPIFFS_CUR, &cur, sizeof(cur), SENSOR_MAX_BYTES);
    s_lastRec     = cur;
    s_lastFlashTs = nowTs;
    xSemaphoreGive(s_spiffsMtx);
}

static size_t flashCountInRange(uint32_t fromTs, uint32_t toTs)
{
    static const char* paths[] = {SPIFFS_BAK, SPIFFS_CUR};
    uint32_t           n      = 0;
    for (const char* path : paths) {
        if (!SPIFFS.exists(path)) continue;
        File f = SPIFFS.open(path, FILE_READ);
        if (!f) continue;
        SolarSensorRec rec;
        while (f.available() >= (int)sizeof(rec)) {
            f.read((uint8_t*)&rec, sizeof(rec));
            if (rec.ts >= fromTs && rec.ts <= toTs) n++;
        }
        f.close();
    }
    return n;
}

static uint32_t maxTsInFlashInRange(uint32_t fromTs, uint32_t toTs)
{
    static const char* paths[] = {SPIFFS_BAK, SPIFFS_CUR};
    uint32_t           mx = 0;
    for (const char* path : paths) {
        if (!SPIFFS.exists(path)) continue;
        File f = SPIFFS.open(path, FILE_READ);
        if (!f) continue;
        SolarSensorRec rec;
        while (f.available() >= (int)sizeof(rec)) {
            f.read((uint8_t*)&rec, sizeof(rec));
            if (rec.ts >= fromTs && rec.ts <= toTs && rec.ts > mx) mx = rec.ts;
        }
        f.close();
    }
    return mx;
}

static void sampleToJsonRow(const Sample& p, char* rowBuf, size_t rowCap)
{
    snprintf(rowBuf, rowCap,
             "[%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%u,%u,%u,%u]",
             (unsigned long)p.ts,
             p.roof, p.boiler, p.storage, p.backflow, p.pool, p.poolSoll,
             (unsigned)p.pump, (unsigned)p.valve, (unsigned)p.circ, (unsigned)p.illum);
}

static void recToSample(const SolarSensorRec& rec, Sample& s)
{
    s.ts       = rec.ts;
    s.roof     = rec.roof10 / 10.0f;
    s.boiler   = rec.boiler10 / 10.0f;
    s.storage  = rec.storage10 / 10.0f;
    s.backflow = rec.backflow10 / 10.0f;
    s.pool     = rec.pool10 / 10.0f;
    s.poolSoll = rec.poolSoll10 / 10.0f;
    s.pump     = (rec.actors >> 0) & 1;
    s.valve    = (rec.actors >> 1) & 1;
    s.circ     = (rec.actors >> 2) & 1;
    s.illum    = (rec.actors >> 3) & 1;
}

} // namespace

void HistoryLog_init()
{
    portENTER_CRITICAL(&s_mux);
    s_count        = 0;
    s_head         = 0;
    s_lastSampleMs = 0;
    memset(s_buf, 0, sizeof(s_buf));
    portEXIT_CRITICAL(&s_mux);

    if (!s_spiffsMtx) s_spiffsMtx = xSemaphoreCreateMutex();

    if (!SPIFFS.begin(true)) {
        ESP_LOGE(TAG, "SPIFFS.begin fehlgeschlagen");
    } else {
        ESP_LOGI(TAG, "SPIFFS ok, frei ~%u KiB",
                 (unsigned)((SPIFFS.totalBytes() - SPIFFS.usedBytes()) / 1024));
    }
    s_lastFlashTs = 0;
    memset(&s_lastRec, 0, sizeof(s_lastRec));
}

static void appendLocked(const SolarLogic::State& st)
{
    uint32_t wall = static_cast<uint32_t>(time(nullptr));
    if (wall < 1700000000U) {
        wall = 0;
    }

    Sample s{};
    s.ts       = wall;
    s.roof     = st.roofTemp;
    s.boiler   = st.boilerTemp;
    s.storage  = st.storageTemp;
    s.backflow = st.backflowTemp;
    s.pool     = st.poolTemp;
    s.poolSoll = st.poolSollTemp;
    s.pump     = st.pumpRunning ? 1 : 0;
    s.valve    = st.valvePool ? 1 : 0;
    s.circ     = st.circulationOn ? 1 : 0;
    s.illum    = st.illuminationOn ? 1 : 0;

    if (s_count < HIST_CAP) {
        uint16_t idx = (s_head + s_count) % HIST_CAP;
        s_buf[idx]   = s;
        s_count++;
    } else {
        s_buf[s_head] = s;
        s_head        = (s_head + 1) % HIST_CAP;
    }
}

void HistoryLog_maybeSample()
{
    uint32_t now = millis();
    if (s_lastSampleMs != 0 && (now - s_lastSampleMs) < SAMPLE_INTERVAL_MS) {
        return;
    }
    s_lastSampleMs = now;

    SolarLogic::State st = SolarLogic::state;

    portENTER_CRITICAL(&s_mux);
    appendLocked(st);
    portEXIT_CRITICAL(&s_mux);

    // SPIFFS während Matter-Commissioning aus: weniger interner RAM + Flash-I/O;
    // gleiche Ursachenkette wie wifi:m f null / AddNOC-OOM.
    if (Display::isCommissioned) {
        spiffsTryRecord(st);
    }
}

size_t HistoryLog_buildJson(char* buf, size_t cap)
{
    if (!buf || cap < 64) return 0;

    // Nicht auf dem httpd-/WS-Task-Stack (~8 kB): 240×Sample würde überschreiten.
    Sample* local = static_cast<Sample*>(malloc(HIST_CAP * sizeof(Sample)));
    if (!local) return 0;

    size_t w = snprintf(buf, cap,
                        "{\"type\":\"history\",\"intervalSec\":%u,\"source\":\"ram\",\"data\":[",
                        static_cast<unsigned>(SAMPLE_INTERVAL_MS / 1000));
    if (w >= cap) {
        free(local);
        return 0;
    }

    uint16_t n = 0;
    portENTER_CRITICAL(&s_mux);
    const uint16_t nSnap    = s_count;
    const uint16_t headSnap = s_head;
    if (nSnap > 0) {
        for (uint16_t i = 0; i < nSnap; i++) {
            local[i] = s_buf[(headSnap + i) % HIST_CAP];
        }
    }
    portEXIT_CRITICAL(&s_mux);
    n = nSnap;

    for (uint16_t i = 0; i < n; i++) {
        char row[128];
        sampleToJsonRow(local[i], row, sizeof(row));
        int chunk = snprintf(buf + w, cap - w, "%s%s", (i == 0) ? "" : ",", row);
        if (chunk < 0 || (size_t)chunk >= cap - w) {
            free(local);
            return 0;
        }
        w += static_cast<size_t>(chunk);
    }
    free(local);

    if (w + 8 > cap) return 0;
    memcpy(buf + w, "]}", 3);
    w += 2;
    return w;
}

size_t HistoryLog_buildJsonRange(uint32_t fromTs, uint32_t toTs, uint16_t maxPts,
                                 char* buf, size_t cap)
{
    if (!buf || cap < 128 || !s_spiffsMtx) return 0;
    if (fromTs > toTs) return 0;
    if (maxPts == 0) maxPts = 600;
    if (maxPts > 600) maxPts = 600;

    if (xSemaphoreTake(s_spiffsMtx, pdMS_TO_TICKS(3000)) != pdTRUE) return 0;

    const uint32_t flashTotal = flashCountInRange(fromTs, toTs);
    const uint32_t step       = (flashTotal > maxPts) ? (flashTotal / maxPts) : 1;

    xSemaphoreGive(s_spiffsMtx);

    size_t w = snprintf(buf, cap,
                        "{\"type\":\"history\",\"intervalSec\":0,\"source\":\"spiffs\",\"step\":%u,\"flashPoints\":%u,\"data\":[",
                        (unsigned)step, (unsigned)flashTotal);
    if (w >= cap) return 0;

    if (xSemaphoreTake(s_spiffsMtx, pdMS_TO_TICKS(3000)) != pdTRUE) return 0;

    uint32_t            idx   = 0;
    bool                first = true;
    static const char*  paths[] = {SPIFFS_BAK, SPIFFS_CUR};
    char                row[144];

    for (const char* path : paths) {
        if (!SPIFFS.exists(path)) continue;
        File f = SPIFFS.open(path, FILE_READ);
        if (!f) continue;
        SolarSensorRec rec;
        while (f.available() >= (int)sizeof(rec)) {
            f.read((uint8_t*)&rec, sizeof(rec));
            if (rec.ts < fromTs || rec.ts > toTs) continue;
            if ((idx % step) != 0) {
                idx++;
                continue;
            }
            Sample s;
            recToSample(rec, s);
            sampleToJsonRow(s, row, sizeof(row));
            int chunk = snprintf(buf + w, cap - w, "%s%s", first ? "" : ",", row);
            first = false;
            idx++;
            if (chunk < 0 || (size_t)chunk >= cap - w) {
                f.close();
                xSemaphoreGive(s_spiffsMtx);
                return 0;
            }
            w += static_cast<size_t>(chunk);
        }
        f.close();
    }

    const uint32_t maxFlashTs = maxTsInFlashInRange(fromTs, toTs);

    Sample* ramLocal = static_cast<Sample*>(malloc(HIST_CAP * sizeof(Sample)));
    if (!ramLocal) {
        xSemaphoreGive(s_spiffsMtx);
        return 0;
    }
    uint16_t rn = 0;
    portENTER_CRITICAL(&s_mux);
    const uint16_t rnSnap    = s_count;
    const uint16_t rheadSnap = s_head;
    if (rnSnap > 0) {
        for (uint16_t i = 0; i < rnSnap; i++) {
            ramLocal[i] = s_buf[(rheadSnap + i) % HIST_CAP];
        }
    }
    portEXIT_CRITICAL(&s_mux);
    rn = rnSnap;

    for (uint16_t i = 0; i < rn; i++) {
        const Sample& p = ramLocal[i];
        if (p.ts < 1700000000U) continue;
        if (p.ts < fromTs || p.ts > toTs) continue;
        if (p.ts <= maxFlashTs) continue;
        sampleToJsonRow(p, row, sizeof(row));
        int chunk = snprintf(buf + w, cap - w, "%s%s", first ? "" : ",", row);
        first = false;
        if (chunk < 0 || (size_t)chunk >= cap - w) {
            free(ramLocal);
            xSemaphoreGive(s_spiffsMtx);
            return 0;
        }
        w += static_cast<size_t>(chunk);
    }
    free(ramLocal);

    xSemaphoreGive(s_spiffsMtx);

    if (w + 8 > cap) return 0;
    memcpy(buf + w, "]}", 3);
    w += 2;
    return w;
}
