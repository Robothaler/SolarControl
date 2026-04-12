#include "CirculationLogic.h"
#include "Config.h"
#include <Arduino.h>
#include <nvs_flash.h>
#include <nvs.h>
#include <esp_log.h>
#include <apps/esp_sntp.h>
#include <time.h>

// Forward-Deklarationen (kein direktes Include wegen LWIP-Konflikt)
namespace SolarLogic {
    void setCirculation(bool on);
}
namespace MatterDevices {
    esp_err_t updateCirculationState(bool on);
}

static const char* TAG = "CirculationLogic";

namespace CirculationLogic {

// ── Interner Zustand ──────────────────────────────────────────────────────────
static Status s_status;

// Flags für thread-sicheren Trigger aus Matter-Callback (CHIP-Task):
static volatile bool s_runRequested  = false;
static volatile bool s_stopRequested = false;

// Tages-Trigger: wurde der erste Lauf heute bereits ausgelöst?
// Wird um Mitternacht (tm_hour == 0) zurückgesetzt.
static int  s_dailyRunDate = -1;  // tm_yday des letzten Tagesstart-Laufs

// ── NVS Persistenz ───────────────────────────────────────────────────────────
static void saveLastRunEnd()
{
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_u32(h, NVS_KEY_CIRC_LAST_RUN,
                    static_cast<uint32_t>(s_status.lastRunEnd));
        nvs_commit(h);
        nvs_close(h);
    }
}

static void loadLastRunEnd()
{
    nvs_handle_t h;
    uint32_t val = 0;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) == ESP_OK) {
        nvs_get_u32(h, NVS_KEY_CIRC_LAST_RUN, &val);
        nvs_close(h);
    }
    s_status.lastRunEnd = static_cast<time_t>(val);
    if (s_status.lastRunEnd > 0) {
        ESP_LOGI(TAG, "Letzter Lauf (aus NVS): %ld", (long)s_status.lastRunEnd);
    }
}

// ── Interne Hilfsfunktionen ───────────────────────────────────────────────────

// Gibt true zurück wenn SNTP synchronisiert ist.
static bool checkTimeSynced()
{
    if (!s_status.timeSynced) {
        s_status.timeSynced =
            (esp_sntp_get_sync_status() == SNTP_SYNC_STATUS_COMPLETED);
        if (s_status.timeSynced) {
            ESP_LOGI(TAG, "SNTP synchronisiert – zeitbasierte Logik aktiv");
        }
    }
    return s_status.timeSynced;
}

// Gibt true zurück wenn die aktuelle Uhrzeit im Präsenzfenster liegt.
static bool inPresenceWindow(const struct tm& t)
{
    int h = t.tm_hour;
    return (h >= CIRC_WINDOW_START_H && h < CIRC_WINDOW_END_H);
}

// Startet einen Lauf (ohne Guard-Prüfung – nur intern aufrufen).
static void _startRun()
{
    s_status.state        = State::RUNNING;
    s_status.pumpOn       = true;
    s_status.stateEnterMs = millis();
    SolarLogic::setCirculation(true);
    MatterDevices::updateCirculationState(true);
    ESP_LOGI(TAG, "Zirkulation EIN (Laufzeit %lu s)",
             (unsigned long)(CIRC_RUN_DURATION_MS / 1000));
}

// Beendet einen Lauf und wechselt in LOCKED.
static void _stopRun()
{
    s_status.state        = State::LOCKED;
    s_status.pumpOn       = false;
    s_status.stateEnterMs = millis();
    s_status.lastRunEnd   = time(nullptr);
    SolarLogic::setCirculation(false);
    MatterDevices::updateCirculationState(false);
    saveLastRunEnd();
    ESP_LOGI(TAG, "Zirkulation AUS → Sperrzeit %lu min",
             (unsigned long)(CIRC_LOCKOUT_MS / 60000));
}

// ─────────────────────────────────────────────────────────────────────────────
// init()
// ─────────────────────────────────────────────────────────────────────────────
void init()
{
    loadLastRunEnd();

    // Falls das Gerät nach 06:00 startet, den Tagesstart-Lauf für heute
    // nicht mehr auslösen (er hat ja schon stattgefunden, oder ist vergangen).
    // Wir merken uns das über das tm_yday des letzten bekannten Laufs.
    // Wenn lastRunEnd im heutigen Tagesstart-Fenster liegt, markieren wir
    // den Daily-Run als erledigt – sonst nicht (Neustart vor 06:00 = OK).
    if (s_status.lastRunEnd > 0) {
        struct tm t;
        localtime_r(&s_status.lastRunEnd, &t);
        // Wenn der letzte Lauf heute nach CIRC_DAY_START_HOUR war,
        // Daily-Run für heute als erledigt markieren.
        time_t now = time(nullptr);
        struct tm t_now;
        localtime_r(&now, &t_now);
        if (t.tm_yday == t_now.tm_yday && t.tm_year == t_now.tm_year
            && t.tm_hour >= CIRC_DAY_START_HOUR) {
            s_dailyRunDate = t_now.tm_yday;
        }
    }

    ESP_LOGI(TAG, "CirculationLogic initialisiert");
}

// ─────────────────────────────────────────────────────────────────────────────
// update()  – jede Sekunde aufrufen
// ─────────────────────────────────────────────────────────────────────────────
void update()
{
    uint32_t now_ms = millis();

    // ── 1. Externe Stopp-Anforderung (aus Matter) ────────────────────────────
    if (s_stopRequested) {
        s_stopRequested = false;
        if (s_status.state == State::RUNNING) {
            ESP_LOGI(TAG, "Manueller Stop via Matter");
            _stopRun();
        }
        return;
    }

    // ── 2. Zustandsübergänge (Timer-basiert) ─────────────────────────────────
    if (s_status.state == State::RUNNING) {
        if (now_ms - s_status.stateEnterMs >= CIRC_RUN_DURATION_MS) {
            ESP_LOGI(TAG, "Auto-Off nach %lu s", (unsigned long)(CIRC_RUN_DURATION_MS/1000));
            _stopRun();
        }
        // Während Pumpe läuft: keine weiteren Checks nötig
        return;
    }

    if (s_status.state == State::LOCKED) {
        if (now_ms - s_status.stateEnterMs >= CIRC_LOCKOUT_MS) {
            s_status.state        = State::IDLE;
            s_status.stateEnterMs = now_ms;
            ESP_LOGI(TAG, "Sperrzeit abgelaufen → IDLE");
        }
        // Während Sperrzeit: keine Starts
        return;
    }

    // ── State == IDLE: externe Anforderung und Auto-Trigger prüfen ───────────

    // ── 2a. Externe Start-Anforderung (aus Matter) ───────────────────────────
    if (s_runRequested) {
        s_runRequested = false;
        // Präsenzfenster wird NICHT geprüft – manueller Trigger darf immer
        _startRun();
        return;
    }

    // ── 2b. Zeitbasierte Auto-Trigger (nur mit gültiger Uhrzeit) ─────────────
    if (!checkTimeSynced()) return;

    time_t     t_now = time(nullptr);
    struct tm  tm;
    localtime_r(&t_now, &tm);

    if (!inPresenceWindow(tm)) {
        // Außerhalb Präsenzfenster: tages-Flag bei Mitternacht zurücksetzen
        if (tm.tm_hour == 0) {
            s_dailyRunDate = -1;
        }
        return;
    }

    // ── 2c. Tagesstart-Lauf (06:xx, einmalig pro Tag) ────────────────────────
    if (tm.tm_hour == CIRC_DAY_START_HOUR && s_dailyRunDate != tm.tm_yday) {
        s_dailyRunDate = tm.tm_yday;
        ESP_LOGI(TAG, "Tagesstart-Lauf (06:00)");
        _startRun();
        return;
    }

    // ── 2d. Legionellenschutz-Lauf (alle 8 h nach letztem Laufende) ──────────
    if (s_status.lastRunEnd > 0) {
        time_t elapsed = t_now - s_status.lastRunEnd;
        if (elapsed >= static_cast<time_t>(CIRC_LEGIONELLA_INTERVAL_S)) {
            ESP_LOGI(TAG, "Legionellenschutz-Lauf (%.1f h seit letztem Lauf)",
                     static_cast<float>(elapsed) / 3600.0f);
            _startRun();
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// requestRun() – aus Matter-Callback (CHIP-Task)
// ─────────────────────────────────────────────────────────────────────────────
bool requestRun()
{
    if (s_status.state == State::LOCKED) {
        uint32_t remaining_ms =
            CIRC_LOCKOUT_MS - (millis() - s_status.stateEnterMs);
        ESP_LOGW(TAG, "Anforderung abgelehnt – Sperrzeit noch %lu min",
                 (unsigned long)(remaining_ms / 60000));
        return false;
    }
    s_runRequested = true;
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// stop() – aus Matter-Callback (CHIP-Task)
// ─────────────────────────────────────────────────────────────────────────────
void stop()
{
    s_stopRequested = true;
}

// ─────────────────────────────────────────────────────────────────────────────
const Status& getStatus()
{
    return s_status;
}

} // namespace CirculationLogic
