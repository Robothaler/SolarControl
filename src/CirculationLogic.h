#pragma once
#include <cstdint>
#include <ctime>

// ─────────────────────────────────────────────────────────────────────────────
// CirculationLogic – Steuerung der Zirkulationspumpe
//
// Zustands-Automat:
//   IDLE    → kein Lauf, Anforderung möglich
//   RUNNING → Pumpe läuft, Auto-Off nach CIRC_RUN_DURATION_MS (3 min)
//   LOCKED  → Sperrzeit nach Lauf, kein Start für CIRC_LOCKOUT_MS (45 min)
//
// Automatische Läufe (nur wenn SNTP synchronisiert):
//   1. Tagesstart-Lauf: erster Lauf des Tages in Stunde CIRC_DAY_START_HOUR (06:xx)
//   2. Legionellenschutz: alle CIRC_LEGIONELLA_INTERVAL_S (8 h) nach Laufende
//      → nur innerhalb des Präsenzfensters (CIRC_WINDOW_START_H…CIRC_WINDOW_END_H)
//
// Matter-Trigger: requestRun() / stop() aus dem Matter-Attribut-Callback heraus.
// GPIO-Kontrolle: über SolarLogic::setCirculation(bool).
// ─────────────────────────────────────────────────────────────────────────────

namespace CirculationLogic {

enum class State : uint8_t {
    IDLE    = 0,
    RUNNING = 1,
    LOCKED  = 2,
};

struct Status {
    State    state          = State::IDLE;
    bool     pumpOn         = false;
    time_t   lastRunEnd     = 0;     // Unix-Timestamp letztes Laufende (0 = nie)
    uint32_t stateEnterMs   = 0;     // millis() beim State-Eintritt
    bool     timeSynced     = false; // SNTP-Synchronisation abgeschlossen
};

// ── Lebenszyklus ─────────────────────────────────────────────────────────────
void init();            // Einmalig in setup() aufrufen, lädt lastRunEnd aus NVS
void update();          // Jede Sekunde aus loop() aufrufen

// ── Externe Trigger ──────────────────────────────────────────────────────────
// requestRun():
//   Fordert einen Lauf an (z.B. aus Matter-Callback oder manuell).
//   Gibt true zurück wenn der Lauf sofort gestartet wird,
//   false wenn abgelehnt (LOCKED oder außerhalb Präsenzfenster).
//   Thread-safe: setzt nur ein Flag, Ausführung in update().
bool requestRun();

// stop():
//   Beendet einen laufenden Zyklus sofort und wechselt in LOCKED.
void stop();

// ── Status-Abfrage ───────────────────────────────────────────────────────────
const Status& getStatus();

} // namespace CirculationLogic
