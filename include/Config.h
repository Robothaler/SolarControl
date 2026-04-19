#pragma once

// ============================================================
//  SolarControl-Matter – Config.h
//  Board : ESP32-S3-DevKitC-1 N16R8V
//          16 MB Flash | 8 MB Octal-PSRAM
//  Sensoren:
//    2× MAX31865 PT1000 (Dach + Boiler) via SPI2
//    2× DS18B20        (Puffer + Rücklauf) via OneWire
//  Author: Robothaler
// ============================================================

// ────────────────────────────────────────────────────────────
// CREDENTIALS (außerhalb des Repos)
// credentials.h liegt unter /home/robothaler/credentials/credentials.h und
// definiert WIFI_SSID, WIFI_PASSWORD, WEBUI_USERNAME/_PASSWORD, OTA_PASSWORD …
// (-I "/home/robothaler/credentials" wird in platformio.ini gesetzt).
// Im CI / auf fremden Rechnern darf die Datei fehlen — dann greifen die
// __has_include-Defaults weiter unten und das Gerät wartet auf Eingabe via
// WebUI / Matter-Network-Commissioning.
// ────────────────────────────────────────────────────────────
#if defined(__has_include)
  #if __has_include("credentials.h")
    #include "credentials.h"
  #endif
#endif

// Fallback-Defaults — niemals echte Zugangsdaten hier hinterlegen, diese Datei
// landet im Repo!
#ifndef WIFI_SSID
  #define WIFI_SSID     ""
#endif
#ifndef WIFI_PASSWORD
  #define WIFI_PASSWORD ""
#endif

// ────────────────────────────────────────────────────────────
// SPI2 BUS – 2× MAX31865 PT1000
// Shared Bus: SCK, MOSI, MISO gemeinsam
// Eigener CS-Pin pro Sensor!
// ────────────────────────────────────────────────────────────
#ifndef PIN_SPI_SCK
  #define PIN_SPI_SCK             12
#endif
#ifndef PIN_SPI_MOSI
  #define PIN_SPI_MOSI            11
#endif
#ifndef PIN_SPI_MISO
  #define PIN_SPI_MISO            13
#endif

// MAX31865 #1 – Kollektor / Dach
#ifndef PIN_CS_MAX31865_ROOF
  #define PIN_CS_MAX31865_ROOF    10
#endif

// MAX31865 #2 – Boiler / Warmwasser
#ifndef PIN_CS_MAX31865_BOILER
  #define PIN_CS_MAX31865_BOILER  14
#endif

// ────────────────────────────────────────────────────────────
// ONEWIRE BUS – 2× DS18B20
// Pufferspeicher + Rücklauf
// ────────────────────────────────────────────────────────────
#ifndef PIN_DS18B20
  #define PIN_DS18B20              4   // 4,7kΩ Pull-up nach 3,3V!
#endif

// DS18B20 Sensor-Indizes auf dem OneWire-Bus
constexpr uint8_t DS18B20_IDX_STORAGE  = 0;  // Pufferspeicher oben
constexpr uint8_t DS18B20_IDX_BACKFLOW = 1;  // Rücklauf Solar

// ────────────────────────────────────────────────────────────
// I2C BUS – OLED Display (optional)
// ────────────────────────────────────────────────────────────
#ifndef PIN_I2C_SDA
  #define PIN_I2C_SDA              8   // 4,7kΩ Pull-up nach 3,3V!
#endif
#ifndef PIN_I2C_SCL
  #define PIN_I2C_SCL              9   // 4,7kΩ Pull-up nach 3,3V!
#endif

// ────────────────────────────────────────────────────────────
// RELAIS-AUSGÄNGE (Active-LOW: LOW = Relais EIN, HIGH = Relais AUS)
// ────────────────────────────────────────────────────────────
#define RELAY_ON   LOW    // Active-LOW: LOW aktiviert das Relais
#define RELAY_OFF  HIGH   // Active-LOW: HIGH deaktiviert das Relais
#ifndef PIN_RELAY_PUMP
  #define PIN_RELAY_PUMP          15   // Solarpumpe
#endif
#ifndef PIN_RELAY_VALVE
  #define PIN_RELAY_VALVE         16   // 3-Wege-Ventil (HIGH=Pool, LOW=Boiler)
#endif
#ifndef PIN_RELAY_CIRC
  #define PIN_RELAY_CIRC          17   // Zirkulationspumpe
#endif
#ifndef PIN_RELAY_ILLUM
  #define PIN_RELAY_ILLUM         18   // Beleuchtung
#endif

// ────────────────────────────────────────────────────────────
// DIGITALE EINGÄNGE
// ────────────────────────────────────────────────────────────
#ifndef PIN_VALVE_STATUS
  #define PIN_VALVE_STATUS         5   // Ventilendschalter (INPUT_PULLUP)
#endif
#ifndef PIN_BUTTON
  #define PIN_BUTTON               6   // Taster Modus (INPUT_PULLUP)
#endif
#ifndef PIN_MOTION
  #define PIN_MOTION               7   // Bewegungsmelder HC-SR501 (INPUT)
#endif
#ifndef PIN_MOTION_POWER
  #define PIN_MOTION_POWER        21   // HC-SR501 Stromversorgung (via N-MOSFET/NPN)
#endif

// ────────────────────────────────────────────────────────────
// ANALOGER EINGANG – PEGELSONDE
// ADC1_CH1 → WiFi/Matter-sicher!
// ────────────────────────────────────────────────────────────
#ifndef PIN_LEVEL_SENSOR
  #define PIN_LEVEL_SENSOR         2
#endif

// ────────────────────────────────────────────────────────────
// COMPILE-TIME PIN-SICHERHEITSPRÜFUNGEN
// ────────────────────────────────────────────────────────────
static_assert(PIN_CS_MAX31865_ROOF   != PIN_CS_MAX31865_BOILER,
    "FEHLER: Beide MAX31865 haben denselben CS-Pin!");
static_assert(PIN_CS_MAX31865_ROOF   != PIN_DS18B20,
    "FEHLER: MAX31865_ROOF CS und DS18B20 belegen denselben Pin!");
static_assert(PIN_CS_MAX31865_BOILER != PIN_DS18B20,
    "FEHLER: MAX31865_BOILER CS und DS18B20 belegen denselben Pin!");

#if (PIN_CS_MAX31865_ROOF >= 26 && PIN_CS_MAX31865_ROOF <= 37)
  #error "PIN_CS_MAX31865_ROOF liegt im internen Flash/PSRAM-Bereich (GPIO26-37)!"
#endif
#if (PIN_CS_MAX31865_BOILER >= 26 && PIN_CS_MAX31865_BOILER <= 37)
  #error "PIN_CS_MAX31865_BOILER liegt im internen Flash/PSRAM-Bereich (GPIO26-37)!"
#endif
#if (PIN_RELAY_PUMP >= 26 && PIN_RELAY_PUMP <= 37)
  #error "PIN_RELAY_PUMP liegt im internen Flash/PSRAM-Bereich (GPIO26-37)!"
#endif

// ────────────────────────────────────────────────────────────
// MAX31865 SENSOR-KONFIGURATION
// ────────────────────────────────────────────────────────────
constexpr float PT1000_NOMINAL_R        = 1000.0f; // PT1000 Nennwiderstand [Ω]
constexpr float PT1000_REF_R            = 4300.0f; // Referenzwiderstand [Ω]
// → Standard MAX31865-Breakout mit 4K3 Ref-R für PT1000
// → Bei abweichendem Breakout-Board hier anpassen!

// ────────────────────────────────────────────────────────────
// STEUERPARAMETER – DIFFERENZTEMPERATUR-REGELUNG
// ────────────────────────────────────────────────────────────
constexpr float TEMP_DIFF_ON            =  8.0f;  // Pumpe EIN-Schwelle  [°C]
constexpr float TEMP_DIFF_OFF           =  2.0f;  // Pumpe AUS-Schwelle  [°C]
constexpr float TEMP_SENSOR_INVALID     = -40.0f; // Fehlerwert Sensor   [°C]
constexpr float TEMP_MAX_COLLECTOR      = 120.0f; // Sicherheits-Abschaltung Kollektor [°C]
constexpr float TEMP_MAX_BOILER         =  95.0f; // Sicherheits-Abschaltung Boiler    [°C]

// ────────────────────────────────────────────────────────────
// STEUERPARAMETER – PEGELSONDE
// ────────────────────────────────────────────────────────────
constexpr uint16_t LEVEL_ADC_MIN        =  400;   // ADC-Wert = Leer  (0%)
constexpr uint16_t LEVEL_ADC_MAX        = 3800;   // ADC-Wert = Voll (100%)
constexpr float    LEVEL_WARN_PCT       = 20.0f;  // Warnschwelle [%]

// ────────────────────────────────────────────────────────────
// TIMING
// ────────────────────────────────────────────────────────────
constexpr uint32_t TEMP_READ_INTERVAL_MS      =  5000; // Temp-Messzyklus          [ms]
constexpr uint32_t MATTER_UPDATE_INTERVAL_MS  = 10000; // Matter Attribut-Update   [ms]
constexpr uint32_t BUTTON_DEBOUNCE_MS         =   200; // Taster Entprellung       [ms]
constexpr uint32_t VALVE_SWITCH_DELAY_MS      =  2000; // Pause nach Ventilschalten[ms]
constexpr uint32_t DS18B20_CONVERSION_MS      =   750; // DS18B20 Konvertierungszeit[ms]

// ────────────────────────────────────────────────────────────
// ZIRKULATIONSPUMPE – Steuerparameter
// ────────────────────────────────────────────────────────────
// Laufzeit pro Zyklus: 3 Minuten, danach Auto-Off
constexpr uint32_t CIRC_RUN_DURATION_MS       =  3 * 60 * 1000;  // 3 min
// Sperrzeit nach jedem Lauf: 45 Minuten (kein erneuter Start möglich)
constexpr uint32_t CIRC_LOCKOUT_MS            = 45 * 60 * 1000;  // 45 min
// Legionellenschutz-Intervall: 8 Stunden nach Laufende
constexpr uint32_t CIRC_LEGIONELLA_INTERVAL_S =  8 * 3600;       // 8 h in Sekunden
// Tagesstart-Stunde: 06:xx → erster automatischer Tageslauf
constexpr int      CIRC_DAY_START_HOUR        =  6;               // 06:00
// Präsenzfenster: Pumpe läuft nur zwischen WINDOW_START und WINDOW_END Uhr
constexpr int      CIRC_WINDOW_START_H        =  6;               // 06:00
constexpr int      CIRC_WINDOW_END_H          = 23;               // 23:00

// ────────────────────────────────────────────────────────────
// ZEITZONE (POSIX TZ-String)
// Wird nach SNTP-Start gesetzt damit localtime_r() korrekte
// Ortszeit liefert. Standardwert: Mitteleuropäische Zeit (CET/CEST).
// Kann zur Laufzeit per WebUI überschrieben werden (NVS-Key tz).
// ────────────────────────────────────────────────────────────
#ifndef TIMEZONE_POSIX
  #define TIMEZONE_POSIX "CET-1CEST,M3.5.0,M10.5.0/3"
#endif

// SNTP-Server (Defaults; per WebUI überschreibbar)
#ifndef NTP_SERVER_PRIMARY
  #define NTP_SERVER_PRIMARY   "pool.ntp.org"
#endif
#ifndef NTP_SERVER_SECONDARY
  #define NTP_SERVER_SECONDARY "time.cloudflare.com"
#endif

// NVS-Key für letzten Zirkulations-Lauf (uint32, Unix-Timestamp)
constexpr char NVS_KEY_CIRC_LAST_RUN[] = "circ_last_run";

// ────────────────────────────────────────────────────────────
// APP VERSION
// ────────────────────────────────────────────────────────────────────────────
constexpr char APP_VERSION[] = "1.2.0";

// ────────────────────────────────────────────────────────────────────────────
// WEB-UI (HTTP-Server + OTA)
// Override WEBUI_USERNAME / WEBUI_PASSWORD / OTA_PASSWORD in credentials.h
// ────────────────────────────────────────────────────────────────────────────
#ifndef WEBUI_PORT
  #define WEBUI_PORT      80
#endif
#ifndef WEBUI_USERNAME
  #define WEBUI_USERNAME  "admin"
#endif
#ifndef WEBUI_PASSWORD
  #define WEBUI_PASSWORD  "solarcontrol"
#endif
#ifndef OTA_PORT
  #define OTA_PORT        8063
#endif
#ifndef OTA_PASSWORD
  #define OTA_PASSWORD    "solarota"
#endif

// ────────────────────────────────────────────────────────────────────────────
// MATTER DEVICE IDENTIFIKATION
// ────────────────────────────────────────────────────────────
constexpr uint16_t MATTER_VENDOR_ID           = 0xFFF1; // Test Vendor (Espressif)
constexpr uint16_t MATTER_PRODUCT_ID          = 0x8003; // SolarControl
                                                         // PoolMaster = 0x8002
constexpr char     MATTER_DEVICE_NAME[]       = "SolarControl";
constexpr char     MATTER_DEVICE_TYPE[]       = "Solar Controller";

// ────────────────────────────────────────────────────────────
// MATTER ENDPOINT IDs
// Fest definiert – müssen mit MatterDevices.cpp übereinstimmen
// ────────────────────────────────────────────────────────────

// Temperatursensoren (read-only → Matter Temperature Sensor Device Type)
constexpr uint16_t EP_ROOF_TEMP       = 1;  // Kollektor/Dach   (MAX31865 #1)
constexpr uint16_t EP_BOILER_TEMP     = 2;  // Boiler           (MAX31865 #2)
constexpr uint16_t EP_STORAGE_TEMP    = 3;  // Pufferspeicher   (DS18B20 #0)
constexpr uint16_t EP_BACKFLOW_TEMP   = 4;  // Rücklauf Solar   (DS18B20 #1)

// Aktoren (steuerbar → Matter On/Off Plugin Unit Device Type)
constexpr uint16_t EP_PUMP            = 5;  // Solarpumpe
constexpr uint16_t EP_VALVE           = 6;  // 3-Wege-Ventil (ON=Pool, OFF=Boiler)
constexpr uint16_t EP_CIRCULATION     = 7;  // Zirkulationspumpe
constexpr uint16_t EP_ILLUMINATION    = 8;  // Beleuchtung

// Binäre Sensoren (read-only)
constexpr uint16_t EP_VALVE_STATUS    = 9;  // Ventilendschalter (Contact Sensor)
constexpr uint16_t EP_MOTION          = 10; // Bewegungsmelder   (Occupancy Sensor)
constexpr uint16_t EP_LEVEL_WARN      = 11; // Pegelsonde Warnung(Contact Sensor)

// ────────────────────────────────────────────────────────────
// NVS (Non-Volatile Storage) – Matter Fabric Persistenz
// ────────────────────────────────────────────────────────────
constexpr char NVS_NAMESPACE[]          = "solar_cfg";

// PoolMaster Matter Node-Referenz (nach Commissioning gespeichert)
constexpr char NVS_KEY_POOL_NODE_ID[]   = "pool_node_id";  // uint64_t
constexpr char NVS_KEY_POOL_EP_TEMP[]   = "pool_ep_temp";  // uint16_t
constexpr char NVS_KEY_POOL_EP_SOLL[]   = "pool_ep_soll";  // uint16_t
constexpr char NVS_KEY_POOL_EP_MODE[]   = "pool_ep_mode";  // uint16_t

// Persistente Solar-Konfiguration
constexpr char NVS_KEY_SOLAR_MODE[]     = "solar_mode";    // uint8_t (SolarMode enum)
constexpr char NVS_KEY_TEMP_DIFF_ON[]   = "tdiff_on";      // float
constexpr char NVS_KEY_TEMP_DIFF_OFF[]  = "tdiff_off";     // float

// WebUI-Settings (per HTTP/POST änderbar)
constexpr char NVS_KEY_TZ[]             = "tz";            // string (POSIX TZ)
constexpr char NVS_KEY_NTP1[]           = "ntp1";          // string
constexpr char NVS_KEY_NTP2[]           = "ntp2";          // string
constexpr char NVS_KEY_AP_FALLBACK[]    = "ap_fallback";   // uint8_t (0/1)

// WiFi-Zugangsdaten (analog ESP32-PoolMaster Matter_dev).
// Werden beim ersten Boot aus credentials.h vorbefüllt (Seed-on-empty) und
// können über die WebUI-Settings überschrieben werden. Direkter Login per
// esp_wifi_set_config() + esp_wifi_connect() — siehe MatterBridge::start().
constexpr char NVS_KEY_WIFI_SSID[]      = "wifi_ssid";     // string (max 32)
constexpr char NVS_KEY_WIFI_PASS[]      = "wifi_pass";     // string (max 64)

