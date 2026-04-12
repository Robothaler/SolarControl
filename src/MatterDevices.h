#pragma once

#include <esp_matter.h>
#include <esp_matter_cluster.h>
#include "Config.h"

// ─────────────────────────────────────────────────────────────────────────────
// Matter Datenmodell – Pool/Boiler Steuerung (Composed Device)
//
// EP 0  Root Node          (automatisch von esp-matter erstellt)
// EP 1  Betriebsmodus      Mode Select Device (0x0027) – AUTO / POOL / BOILER
// EP 2  Solarpumpe         On/Off Plugin Unit (0x010A) – read-only
// EP 3  Ventil-Rückmeldung Mode Select Device (0x0027) – BOILER / POOL, read-only
// EP 4  Dach-Kollektor     Temperature Sensor (0x0302)
// EP 5  Boiler             Temperature Sensor (0x0302)
// ─────────────────────────────────────────────────────────────────────────────

namespace MatterDevices {

// ── Modus-Konstanten für EP1 (Betriebsmodus) ─────────────────────────────────
static constexpr uint8_t MODE_AUTO   = 0;
static constexpr uint8_t MODE_POOL   = 1;
static constexpr uint8_t MODE_BOILER = 2;

// ── Modus-Konstanten für EP3 (Ventil-Rückmeldung) ───────────────────────────
static constexpr uint8_t VALVE_BOILER = 0;
static constexpr uint8_t VALVE_POOL   = 1;

// ── Endpoint Handles ─────────────────────────────────────────────────────────
extern esp_matter::endpoint_t* epControlMode;    // EP1: Betriebsmodus
extern esp_matter::endpoint_t* epPump;           // EP2: Solarpumpe
extern esp_matter::endpoint_t* epValveFeedback;  // EP3: Ventil-Rückmeldung
extern esp_matter::endpoint_t* epRoofTemp;       // EP4: Dach-Kollektor
extern esp_matter::endpoint_t* epBoilerTemp;     // EP5: Boiler
extern esp_matter::endpoint_t* epCirculation;    // EP6: Zirkulationspumpe

// ── Initialisierung ───────────────────────────────────────────────────────────
esp_err_t init(esp_matter::node_t* node);

// ── Attribut-Updates (aus SolarLogic → Matter) ───────────────────────────────

// EP1: Betriebsmodus – aktuellen Modus publizieren (z.B. nach NVS-Restore)
// mode: MODE_AUTO(0), MODE_POOL(1), MODE_BOILER(2)
esp_err_t updateControlMode(uint8_t mode);

// EP2: Pumpenstatus (true = läuft)
esp_err_t updatePumpState(bool running);

// EP3: Ventil-Rückmeldung – Hardware-Pin-Status
// mode: VALVE_BOILER(0) oder VALVE_POOL(1)
esp_err_t updateValveFeedback(uint8_t mode);

// EP4 / EP5: Temperatur in °C (−40…150)
esp_err_t updateTemperature(esp_matter::endpoint_t* ep, float tempCelsius);

// EP6: Zirkulationspumpe – tatsächlichen Hardware-Zustand spiegeln
// Wird von CirculationLogic nach jedem Start/Stop aufgerufen.
esp_err_t updateCirculationState(bool running);

// ── Attribute Change Callback (von esp_matter::start() registriert) ──────────
// Verarbeitet ChangeToMode auf EP1 → SolarLogic::setMode()
// Lehnt ChangeToMode auf EP3 ab (read-only)
// Lehnt direkte OnOff-Schreibzugriffe auf EP2 ab (read-only)
// EP6 OnOff=true  → CirculationLogic::requestRun() (abgelehnt wenn LOCKED)
// EP6 OnOff=false → CirculationLogic::stop()
esp_err_t attributeChangeCallback(
    const esp_matter::attribute::callback_type_t type,
    uint16_t endpoint_id,
    uint32_t cluster_id,
    uint32_t attribute_id,
    esp_matter_attr_val_t* val);

// ── PoolMaster Matter Subscription ───────────────────────────────────────────
esp_err_t subscribeToPoolMaster(uint64_t poolNodeId,
                                 uint16_t epPoolTemp,
                                 uint16_t epPoolSolltemp,
                                 uint16_t epSolarMode);

} // namespace MatterDevices
