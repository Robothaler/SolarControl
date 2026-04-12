#include "SolarLogic.h"
#include "Display.h"
#include "Config.h"

#include <Adafruit_MAX31865.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <nvs_flash.h>
#include <nvs.h>
#include <esp_log.h>
#include <esp_matter.h>    // ← nur esp_matter direkt, nicht MatterDevices.h
#include <Arduino.h>

// Vorwärts-Deklarationen statt #include "MatterDevices.h"
// → vermeidet LWIP/IPAddress Namenskonflikt
// Nur noch die Funktionen, die SolarLogic tatsächlich aufruft:
namespace MatterDevices {
    esp_err_t updatePumpState(bool running);
    esp_err_t updateValveFeedback(uint8_t mode);    // 0=BOILER, 1=POOL
    esp_err_t updateCirculationState(bool on);
}

static const char* TAG = "SolarLogic";

// ── 2× MAX31865 auf gemeinsamem SPI2-Bus ──────────────────────────────────
// Konstruktor: Adafruit_MAX31865(cs, mosi, miso, sck) → Software-SPI
// ODER:        Adafruit_MAX31865(cs)                  → Hardware-SPI
// Wir nutzen Hardware-SPI2 mit expliziter Bus-Konfiguration:
static SPIClass          spi2(HSPI);
static Adafruit_MAX31865 max31865_roof(PIN_CS_MAX31865_ROOF,
                                       &spi2);   // Kollektor/Dach
static Adafruit_MAX31865 max31865_boiler(PIN_CS_MAX31865_BOILER,
                                          &spi2); // Boiler

// ── DS18B20 (2 Sensoren, OneWire-Bus) ─────────────────────────────────────
static OneWire           oneWire(PIN_DS18B20);
static DallasTemperature dsSensors(&oneWire);

// ── Timing ────────────────────────────────────────────────────────────────
static unsigned long lastButtonPress   = 0;
static unsigned long dsRequestTime     = 0;
static bool          dsConversionPending = false;

namespace SolarLogic {

State state;

// ────────────────────────────────────────────────────────────────────────────
// Initialisierung
// ────────────────────────────────────────────────────────────────────────────
void init()
{
    // ── Relay-Pins ──────────────────────────────────────────────────────────
    pinMode(PIN_RELAY_PUMP,  OUTPUT); digitalWrite(PIN_RELAY_PUMP,  RELAY_OFF);
    pinMode(PIN_RELAY_VALVE, OUTPUT); digitalWrite(PIN_RELAY_VALVE, RELAY_OFF);
    pinMode(PIN_RELAY_CIRC,  OUTPUT); digitalWrite(PIN_RELAY_CIRC,  RELAY_OFF);
    pinMode(PIN_RELAY_ILLUM, OUTPUT); digitalWrite(PIN_RELAY_ILLUM, RELAY_OFF);

    // ── Eingabe-Pins ────────────────────────────────────────────────────────
    pinMode(PIN_VALVE_STATUS, INPUT_PULLUP);
    pinMode(PIN_BUTTON,       INPUT_PULLUP);
    pinMode(PIN_MOTION,       INPUT);
    pinMode(PIN_MOTION_POWER, OUTPUT); digitalWrite(PIN_MOTION_POWER, HIGH); // HC-SR501 einschalten
    // Pegelsonde: ADC1, kein pinMode nötig (analogRead direkt)

    // ── SPI2 Bus initialisieren ─────────────────────────────────────────────
    spi2.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI);
    ESP_LOGI(TAG, "SPI2 Bus initialisiert: SCK=%d MISO=%d MOSI=%d",
             PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI);

    // ── MAX31865 #1 – Kollektor/Dach (PT1000, 3-Draht) ─────────────────────
    if (!max31865_roof.begin(MAX31865_3WIRE)) {
        ESP_LOGE(TAG, "MAX31865 Dach (CS=GPIO%d) nicht gefunden!", PIN_CS_MAX31865_ROOF);
    } else {
        ESP_LOGI(TAG, "MAX31865 Dach (CS=GPIO%d) initialisiert", PIN_CS_MAX31865_ROOF);
    }

    // ── MAX31865 #2 – Boiler (PT1000, 3-Draht) ─────────────────────────────
    if (!max31865_boiler.begin(MAX31865_3WIRE)) {
        ESP_LOGE(TAG, "MAX31865 Boiler (CS=GPIO%d) nicht gefunden!", PIN_CS_MAX31865_BOILER);
    } else {
        ESP_LOGI(TAG, "MAX31865 Boiler (CS=GPIO%d) initialisiert", PIN_CS_MAX31865_BOILER);
    }

    // ── DS18B20 ─────────────────────────────────────────────────────────────
    dsSensors.begin();
    // Nicht-blockierendes Konvertierungsmodell aktivieren
    dsSensors.setWaitForConversion(false);
    uint8_t count = dsSensors.getDeviceCount();
    ESP_LOGI(TAG, "DS18B20 Sensoren gefunden: %d (erwartet: 2)", count);
    if (count < 2) {
        ESP_LOGW(TAG, "Zu wenig DS18B20! Puffer- und Rücklauf-Sensor prüfen.");
    }

    // ── NVS Konfiguration laden ─────────────────────────────────────────────
    loadConfigFromNVS();

    ESP_LOGI(TAG, "SolarLogic initialisiert. Modus: %d", (int)state.currentMode);
}

// ────────────────────────────────────────────────────────────────────────────
// Temperaturen lesen (nicht-blockierend)
// Ablauf DS18B20:
//   1. requestTemperatures() → startet Konvertierung (~750ms)
//   2. Nach DS18B20_CONVERSION_MS → getTempCByIndex() lesen
// ────────────────────────────────────────────────────────────────────────────
void readTemperatures()
{
    unsigned long now = millis();

    // ── MAX31865 #1 – Kollektor/Dach ────────────────────────────────────────
    {
        uint8_t fault = max31865_roof.readFault();
        if (fault) {
            ESP_LOGE(TAG, "MAX31865 Dach Fault: 0x%02X", fault);
            if (fault & MAX31865_FAULT_HIGHTHRESH)  ESP_LOGE(TAG, "  → RTD High Threshold");
            if (fault & MAX31865_FAULT_LOWTHRESH)   ESP_LOGE(TAG, "  → RTD Low Threshold");
            if (fault & MAX31865_FAULT_REFINLOW)    ESP_LOGE(TAG, "  → REFIN- > 0.85 x Bias");
            if (fault & MAX31865_FAULT_REFINHIGH)   ESP_LOGE(TAG, "  → REFIN- < 0.85 x Bias (FORCE- open)");
            if (fault & MAX31865_FAULT_RTDINLOW)    ESP_LOGE(TAG, "  → RTDIN- < 0.85 x Bias (FORCE- open)");
            if (fault & MAX31865_FAULT_OVUV)        ESP_LOGE(TAG, "  → Under/Overvoltage");
            max31865_roof.clearFault();
            // Letzten gültigen Wert beibehalten
        } else {
            float t = max31865_roof.temperature(PT1000_NOMINAL_R, PT1000_REF_R);
            if (t > TEMP_SENSOR_INVALID && t < TEMP_MAX_COLLECTOR) {
                state.roofTemp = t;
                ESP_LOGD(TAG, "Dach (MAX31865 #1): %.2f°C", state.roofTemp);
            } else {
                ESP_LOGW(TAG, "Dach Temp ausserhalb Plausibilitaet: %.2f°C", t);
            }
        }
    }

    // ── MAX31865 #2 – Boiler ────────────────────────────────────────────────
    {
        uint8_t fault = max31865_boiler.readFault();
        if (fault) {
            ESP_LOGE(TAG, "MAX31865 Boiler Fault: 0x%02X", fault);
            max31865_boiler.clearFault();
        } else {
            float t = max31865_boiler.temperature(PT1000_NOMINAL_R, PT1000_REF_R);
            if (t > TEMP_SENSOR_INVALID && t < TEMP_MAX_BOILER) {
                state.boilerTemp = t;
                ESP_LOGD(TAG, "Boiler (MAX31865 #2): %.2f°C", state.boilerTemp);
            } else {
                ESP_LOGW(TAG, "Boiler Temp ausserhalb Plausibilitaet: %.2f°C", t);
            }
        }
    }

    // ── DS18B20 – nicht-blockierendes Modell ────────────────────────────────
    if (!dsConversionPending) {
        // Konvertierung starten
        dsSensors.requestTemperatures();
        dsRequestTime       = now;
        dsConversionPending = true;
        ESP_LOGD(TAG, "DS18B20 Konvertierung gestartet");
    }
    else if (now - dsRequestTime >= DS18B20_CONVERSION_MS) {
        // Konvertierung abgeschlossen → Werte lesen
        float t0 = dsSensors.getTempCByIndex(DS18B20_IDX_STORAGE);
        float t1 = dsSensors.getTempCByIndex(DS18B20_IDX_BACKFLOW);

        if (t0 != DEVICE_DISCONNECTED_C && t0 > TEMP_SENSOR_INVALID) {
            state.storageTemp = t0;
            ESP_LOGD(TAG, "Puffer (DS18B20 #0): %.2f°C", state.storageTemp);
        } else {
            ESP_LOGW(TAG, "DS18B20 Puffer (Index %d) nicht verbunden oder Fehler!",
                     DS18B20_IDX_STORAGE);
        }

        if (t1 != DEVICE_DISCONNECTED_C && t1 > TEMP_SENSOR_INVALID) {
            state.backflowTemp = t1;
            ESP_LOGD(TAG, "Ruecklauf (DS18B20 #1): %.2f°C", state.backflowTemp);
        } else {
            ESP_LOGW(TAG, "DS18B20 Ruecklauf (Index %d) nicht verbunden oder Fehler!",
                     DS18B20_IDX_BACKFLOW);
        }

        dsConversionPending = false;
        // DS18B20-Werte werden in main.cpp loop() via MatterDevices::updateTemperature()
        // publiziert – DS18B20 Endpoints (Puffer, Rücklauf) sind im neuen
        // Matter-Modell nicht mehr vorgesehen (nur EP4=Dach, EP5=Boiler).
    }
}

// ────────────────────────────────────────────────────────────────────────────
// Pegelsonde lesen (ADC1, WiFi-sicher)
// ────────────────────────────────────────────────────────────────────────────
void readLevelSensor()
{
    // Mehrfach lesen + Mittelwert für stabilere ADC-Werte
    uint32_t adcSum = 0;
    constexpr uint8_t SAMPLES = 8;
    for (uint8_t i = 0; i < SAMPLES; i++) {
        adcSum += analogRead(PIN_LEVEL_SENSOR);
        delayMicroseconds(100);
    }
    uint16_t adcVal = static_cast<uint16_t>(adcSum / SAMPLES);

    // ADC-Wert → Prozent umrechnen
    float pct = 0.0f;
    if (adcVal <= LEVEL_ADC_MIN) {
        pct = 0.0f;
    } else if (adcVal >= LEVEL_ADC_MAX) {
        pct = 100.0f;
    } else {
        pct = (static_cast<float>(adcVal - LEVEL_ADC_MIN) /
               static_cast<float>(LEVEL_ADC_MAX - LEVEL_ADC_MIN)) * 100.0f;
    }

    state.levelPct  = pct;
    state.levelWarn = (pct < LEVEL_WARN_PCT);

    ESP_LOGD(TAG, "Pegelsonde: ADC=%d → %.1f%% %s",
             adcVal, pct, state.levelWarn ? "⚠ WARNUNG" : "OK");
}

// ────────────────────────────────────────────────────────────────────────────
// Steuerlogik (zyklisch via update())
// ────────────────────────────────────────────────────────────────────────────
void update()
{
    // ── Sicherheitscheck: Sensoren plausibel? ───────────────────────────────
    bool roofOk    = (state.roofTemp    > TEMP_SENSOR_INVALID);
    bool boilerOk  = (state.boilerTemp  > TEMP_SENSOR_INVALID);
    bool storageOk = (state.storageTemp > TEMP_SENSOR_INVALID);

    if (!roofOk) {
        ESP_LOGE(TAG, "Sicherheit: Dach-Sensor ungueltig → Pumpe AUS");
        setPump(false);
        return;
    }

    // ── Ventilposition bestimmen ────────────────────────────────────────────
    bool usePool = false;
    switch (state.currentMode) {
        case Mode::AUTO:
            // Pool-Heizung wenn:
            //   1. PoolMaster hat Solar angefordert
            //   2. Pooltemp unter Sollwert
            //   3. Kollektor wärmer als Pool (sonst sinnlos)
            usePool = state.poolModeRequest
                   && (state.poolTemp < state.poolSollTemp)
                   && (state.roofTemp > state.poolTemp);
            break;
        case Mode::MANUAL_POOL:
            usePool = true;
            break;
        case Mode::MANUAL_BOILER:
        default:
            usePool = false;
            break;
    }

    // ── Ventil umschalten (mit Verzögerungsschutz) ──────────────────────────
    if (usePool != state.valvePool) {
        ESP_LOGI(TAG, "Ventil wechselt: %s → %s",
                 state.valvePool ? "POOL" : "BOILER",
                 usePool         ? "POOL" : "BOILER");
        // Pumpe kurz stoppen vor Ventilumschaltung (Druckschutz)
        if (state.pumpRunning) {
            setPump(false);
            delay(VALVE_SWITCH_DELAY_MS);
        }
        setValve(usePool);
    }

    // ── Referenztemperatur für Differenzregelung ────────────────────────────
    float refTemp = 0.0f;
    if (usePool) {
        // Pool-Modus: Kollektor muss wärmer als Pool-Solltemp sein
        refTemp = state.poolSollTemp;
    } else {
        // Boiler-Modus: Kollektor muss wärmer als Boiler sein
        refTemp = boilerOk ? state.boilerTemp : state.storageTemp;
    }

    float diff = state.roofTemp - refTemp;

    ESP_LOGD(TAG, "Regelung: Dach=%.1f°C Ref=%.1f°C Diff=%.1f°C Pumpe=%s",
             state.roofTemp, refTemp, diff,
             state.pumpRunning ? "AN" : "AUS");

    // ── Pumpen-Hysterese ────────────────────────────────────────────────────
    if (!state.pumpRunning && diff >= TEMP_DIFF_ON) {
        ESP_LOGI(TAG, "Pumpe EIN: Diff=%.1f°C >= %.1f°C", diff, TEMP_DIFF_ON);
        setPump(true);
    } else if (state.pumpRunning && diff <= TEMP_DIFF_OFF) {
        ESP_LOGI(TAG, "Pumpe AUS: Diff=%.1f°C <= %.1f°C", diff, TEMP_DIFF_OFF);
        setPump(false);
    }

    // ── Überhitzungsschutz Boiler ───────────────────────────────────────────
    if (boilerOk && state.boilerTemp >= TEMP_MAX_BOILER && state.pumpRunning) {
        ESP_LOGW(TAG, "Boiler-Überhitzung! %.1f°C >= %.1f°C → Pumpe AUS",
                 state.boilerTemp, TEMP_MAX_BOILER);
        setPump(false);
    }

    // ── Überhitzungsschutz Kollektor ────────────────────────────────────────
    if (state.roofTemp >= TEMP_MAX_COLLECTOR && state.pumpRunning) {
        ESP_LOGW(TAG, "Kollektor-Überhitzung! %.1f°C >= %.1f°C → Pumpe AUS",
                 state.roofTemp, TEMP_MAX_COLLECTOR);
        setPump(false);
    }
}

// ────────────────────────────────────────────────────────────────────────────
// Aktoren
// ────────────────────────────────────────────────────────────────────────────
void setPump(bool on)
{
    if (state.pumpRunning == on) return; // Kein unnötiges Schalten
    state.pumpRunning = on;
    digitalWrite(PIN_RELAY_PUMP, on ? RELAY_ON : RELAY_OFF);
    MatterDevices::updatePumpState(on);
    ESP_LOGI(TAG, "Pumpe: %s", on ? "EIN" : "AUS");
}

void setValve(bool poolMode)
{
    state.valvePool = poolMode;
    digitalWrite(PIN_RELAY_VALVE, poolMode ? RELAY_ON : RELAY_OFF);
    MatterDevices::updateValveFeedback(poolMode ? 1 : 0);  // 1=POOL, 0=BOILER
    ESP_LOGI(TAG, "Ventil: %s", poolMode ? "POOL" : "BOILER");
}

void setCirculation(bool on)
{
    if (state.circulationOn == on) return;
    state.circulationOn = on;
    digitalWrite(PIN_RELAY_CIRC, on ? RELAY_ON : RELAY_OFF);
    MatterDevices::updateCirculationState(on);
    ESP_LOGI(TAG, "Zirkulation: %s", on ? "EIN" : "AUS");
}

void setIllumination(bool on)
{
    if (state.illuminationOn == on) return;
    state.illuminationOn = on;
    digitalWrite(PIN_RELAY_ILLUM, on ? RELAY_ON : RELAY_OFF);
    ESP_LOGI(TAG, "Beleuchtung: %s", on ? "EIN" : "AUS");
}

void setMotionPower(bool on)
{
    state.motionPowerOn = on;
    digitalWrite(PIN_MOTION_POWER, on ? HIGH : LOW);
    if (!on) state.motionDetected = false;
    ESP_LOGI(TAG, "Bewegungsmelder Power: %s", on ? "EIN" : "AUS");
}

void setMode(Mode mode)
{
    if (state.currentMode == mode) return;
    state.currentMode = mode;
    const char* modeStr =
        (mode == Mode::AUTO)          ? "AUTO"   :
        (mode == Mode::MANUAL_POOL)   ? "POOL"   :
                                        "BOILER";
    ESP_LOGI(TAG, "Modus: %s", modeStr);
    saveConfigToNVS();
}

// ────────────────────────────────────────────────────────────────────────────
// Matter Subscription Callbacks
// ────────────────────────────────────────────────────────────────────────────
void onPoolTempReceived(float temp)
{
    if (temp == state.poolTemp) return;
    state.poolTemp = temp;
    ESP_LOGD(TAG, "Pool-Temp (Matter): %.2f°C", temp);
}

void onPoolSolltempReceived(float solltemp)
{
    if (solltemp == state.poolSollTemp) return;
    state.poolSollTemp = solltemp;
    ESP_LOGD(TAG, "Pool-Solltemp (Matter): %.2f°C", solltemp);
}

void onPoolModeRequestReceived(bool requested)
{
    if (requested == state.poolModeRequest) return;
    state.poolModeRequest = requested;
    ESP_LOGI(TAG, "Pool-Modus-Anforderung (Matter): %s",
             requested ? "JA (Pool heizen)" : "NEIN (Boiler)");
    // Steuerlogik sofort auslösen (nicht auf nächsten Zyklus warten)
    update();
}

// ────────────────────────────────────────────────────────────────────────────
// Button Handler (zyklisch pollen, Entprellung)
// Kurzdruck: Modus weiterschalten AUTO→POOL→BOILER→AUTO
// Langdruck (>3s): Werksreset Matter Commissioning
// ────────────────────────────────────────────────────────────────────────────
void handleButton()
{
    static bool          lastState        = HIGH;
    static unsigned long pressStart       = 0;
    static bool          longPressHandled = false;

    bool currentState = digitalRead(PIN_BUTTON);

    if (lastState == HIGH && currentState == LOW) {
        pressStart       = millis();
        longPressHandled = false;
    }

        // Langdruck (>15s): Matter Factory Reset
    if (currentState == LOW && !longPressHandled) {
        if (millis() - pressStart >= 15000) {
            longPressHandled = true;
            ESP_LOGW(TAG, "Langdruck → Matter Factory Reset!");
            Display::showResetWarning();   // ← korrigiert: kein u8g2_extern
            delay(2000);
            esp_matter::factory_reset();
        }
    }

    // Flanke: Taste losgelassen
    if (lastState == LOW && currentState == HIGH) {
        unsigned long held = millis() - pressStart;

        if (held >= BUTTON_DEBOUNCE_MS && !longPressHandled) {

            if (!Display::isCommissioned) {
                // Commissioning-Modus: Button macht nichts
            } else if (held < 1000) {
                // Kurzdruck (<1s) → Display-Screen weiterschalten
                uint8_t next =
                    (static_cast<uint8_t>(Display::currentScreen) + 1) %
                     static_cast<uint8_t>(Display::Screen::SCREEN_COUNT);
                Display::setScreen(static_cast<Display::Screen>(next));
                ESP_LOGD(TAG, "Kurzdruck → Display Screen %d", next);
            } else {
                // Mitteldruck (1-3s) → Solar-Modus wechseln
                switch (state.currentMode) {
                    case Mode::AUTO:          setMode(Mode::MANUAL_POOL);    break;
                    case Mode::MANUAL_POOL:   setMode(Mode::MANUAL_BOILER);  break;
                    case Mode::MANUAL_BOILER: setMode(Mode::AUTO);           break;
                }
            }
        }
    }

    lastState = currentState;
}


// ────────────────────────────────────────────────────────────────────────────
// Bewegungsmelder Handler (Flanken-Erkennung)
// ────────────────────────────────────────────────────────────────────────────
void handleMotion()
{
    static bool lastMotion = false;
    bool detected = (digitalRead(PIN_MOTION) == HIGH);

    if (detected != lastMotion) {
        lastMotion           = detected;
        state.motionDetected = detected;
        ESP_LOGI(TAG, "Bewegung: %s", detected ? "ERKANNT" : "KEINE");
    }
}

// ────────────────────────────────────────────────────────────────────────────
// NVS Persistenz – Konfiguration speichern
// ────────────────────────────────────────────────────────────────────────────
void saveConfigToNVS()
{
    nvs_handle_t handle;
    esp_err_t    err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS öffnen fehlgeschlagen: %s", esp_err_to_name(err));
        return;
    }

    nvs_set_u8(handle, NVS_KEY_SOLAR_MODE, static_cast<uint8_t>(state.currentMode));

    // Float als uint32 (Bit-cast) speichern
    uint32_t diffOn, diffOff;
    memcpy(&diffOn,  &TEMP_DIFF_ON,  sizeof(float));
    memcpy(&diffOff, &TEMP_DIFF_OFF, sizeof(float));
    nvs_set_u32(handle, NVS_KEY_TEMP_DIFF_ON,  diffOn);
    nvs_set_u32(handle, NVS_KEY_TEMP_DIFF_OFF, diffOff);

    err = nvs_commit(handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS commit fehlgeschlagen: %s", esp_err_to_name(err));
    } else {
        ESP_LOGD(TAG, "Konfiguration in NVS gespeichert");
    }
    nvs_close(handle);
}

// ────────────────────────────────────────────────────────────────────────────
// NVS Persistenz – Konfiguration laden
// ────────────────────────────────────────────────────────────────────────────
void loadConfigFromNVS()
{
    nvs_handle_t handle;
    esp_err_t    err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGI(TAG, "Kein NVS-Eintrag gefunden → Standardwerte verwenden");
        return;
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS lesen fehlgeschlagen: %s", esp_err_to_name(err));
        return;
    }

    // Modus laden
    uint8_t modeVal = static_cast<uint8_t>(Mode::AUTO);
    nvs_get_u8(handle, NVS_KEY_SOLAR_MODE, &modeVal);
    state.currentMode = static_cast<Mode>(modeVal);

    nvs_close(handle);
    ESP_LOGI(TAG, "Konfiguration aus NVS geladen: Modus=%d",
             static_cast<int>(state.currentMode));
}

} // namespace SolarLogic


