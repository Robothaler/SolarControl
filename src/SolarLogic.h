#pragma once

#include <cstdint>
#include "Config.h"

namespace SolarLogic {

enum class Mode : uint8_t {
    AUTO           = 0,   // Automatik: Pool wenn angefordert, sonst Boiler
    MANUAL_POOL    = 1,   // Manuell: immer Pool-Heizkreis
    MANUAL_BOILER  = 2    // Manuell: immer Boiler-Heizkreis
};

struct State {
    // ── Temperaturen ──────────────────────────────────────────
    // 2× PT1000 via MAX31865 (SPI2, je eigener CS)
    float roofTemp      = 0.0f;  // Kollektor/Dach   → MAX31865 #1 (CS=GPIO10)
    float boilerTemp    = 0.0f;  // Boiler/Speicher  → MAX31865 #2 (CS=GPIO14)

    // 2× DS18B20 via OneWire
    float storageTemp   = 0.0f;  // Pufferspeicher oben → DS18B20 Index 0
    float backflowTemp  = 0.0f;  // Rücklauf Solar      → DS18B20 Index 1

    // Von PoolMaster via Matter Subscription
    float poolTemp      = 0.0f;  // Aktuelle Pooltemperatur
    float poolSollTemp  = 0.0f;  // Ziel-Pooltemperatur

    // ── Aktoren ───────────────────────────────────────────────
    bool  pumpRunning    = false; // Solarpumpe
    bool  valvePool      = false; // true=Pool-Heizkreis, false=Boiler
    bool  circulationOn  = false; // Zirkulationspumpe
    bool  illuminationOn = false; // Beleuchtung

    // ── Digitale Eingänge ─────────────────────────────────────
    bool  valveStatusOK  = false; // Ventilendschalter OK
    bool  motionDetected = false; // Bewegungsmelder aktiv
    bool  motionPowerOn  = true;  // HC-SR501 Versorgungsspannung ein (GPIO21)

    // ── Pegelsonde ────────────────────────────────────────────
    float levelPct       = 0.0f; // Füllstand [%]
    bool  levelWarn      = false; // true wenn < LEVEL_WARN_PCT

    // ── Steuerung ─────────────────────────────────────────────
    Mode  currentMode       = Mode::AUTO;
    bool  poolModeRequest   = false; // PoolMaster fordert Solar an
};

// Globaler Zustand
extern State state;

// Initialisierung
void init();

// Zyklische Funktionen
void readTemperatures();  // Alle 4 Sensoren lesen
void readLevelSensor();   // Pegelsonde ADC lesen
void update();            // Steuerlogik ausführen
void handleButton();      // Taster pollen
void handleMotion();      // Bewegungsmelder pollen

// Aktoren
void setPump(bool on);
void setValve(bool poolMode);
void setCirculation(bool on);
void setIllumination(bool on);
void setMotionPower(bool on);
void setMode(Mode mode);

// Matter Subscription Callbacks (Daten vom PoolMaster)
void onPoolTempReceived(float temp);
void onPoolSolltempReceived(float solltemp);
void onPoolModeRequestReceived(bool requested);

// NVS Persistenz
void saveConfigToNVS();
void loadConfigFromNVS();

} // namespace SolarLogic
