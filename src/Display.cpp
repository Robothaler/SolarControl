#include "../include/Display.h"
#include "SolarLogic.h"
#include "Config.h"

#include <U8g2lib.h>
#include <Wire.h>
#include <esp_log.h>
#include <qrcode.h>   // espressif__qrcode (via idf_component.yml)
#include <esp_netif.h>

static const char* TAG = "Display";

// ─────────────────────────────────────────────────────────────────────────────
// U8g2 Display-Objekt
// Exakt gleicher Typ wie Original (SSD1306_128X64_NONAME)
// Einzige Änderung: Explizite Pin-Angabe für ESP32-S3 GPIO8/GPIO9
// I2C-Adresse 0x3C ist in NONAME-Variante fest hinterlegt
// ─────────────────────────────────────────────────────────────────────────────
static U8G2_SSD1306_128X64_NONAME_F_HW_I2C
    u8g2(U8G2_R0,
         /* reset=*/ U8X8_PIN_NONE,
         /* clock=*/ PIN_I2C_SCL,    // GPIO9
         /* data= */ PIN_I2C_SDA);   // GPIO8

// ─────────────────────────────────────────────────────────────────────────────
// Vergleich Original → ESP32-S3
// ─────────────────────────────────────────────────────────────────────────────
// Original: U8G2_SSD1306_128X64_NONAME_F_HW_I2C display(U8G2_R0, U8X8_PIN_NONE)
//           → nutzt Wire-Default-Pins (SCL=21, SDA=20 auf Mega)
//
// ESP32-S3: U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE,
//                                                     PIN_I2C_SCL, PIN_I2C_SDA)
//           → explizit GPIO9 (SCL) + GPIO8 (SDA)
//           → I2C-Adresse 0x3C bleibt identisch (NONAME = immer 0x3C)
// ─────────────────────────────────────────────────────────────────────────────

namespace Display {

// Öffentliche Status-Flags
bool isCommissioned = false;
bool wifiConnected  = false;
char wifiIP[16]     = "---";
Screen currentScreen  = Screen::TEMPERATURES;

// Interne Variablen
static unsigned long lastScreenChange = 0;
static bool          initialized      = false;

// QR-Code Bitmap-Cache (Version 2 = 25×25 = 625 Module = 79 Bytes)
// Wird einmalig beim ersten Aufruf mit gültigem Payload befüllt.
static constexpr uint8_t QR_MODULES = 25;
static uint8_t  qrBits[(QR_MODULES * QR_MODULES + 7) / 8] = {0};
static uint8_t  qrSize     = 0;
static bool     qrGenerated = false;

// Display-Callback für esp_qrcode_generate() – speichert Module als Bitfeld
static void qr_store_cb(esp_qrcode_handle_t qrcode)
{
    uint8_t sz = static_cast<uint8_t>(esp_qrcode_get_size(qrcode));
    qrSize = (sz < QR_MODULES) ? sz : QR_MODULES;
    memset(qrBits, 0, sizeof(qrBits));
    for (uint8_t y = 0; y < qrSize; y++) {
        for (uint8_t x = 0; x < qrSize; x++) {
            if (esp_qrcode_get_module(qrcode, x, y)) {
                uint16_t bit = static_cast<uint16_t>(y) * qrSize + x;
                qrBits[bit >> 3] |= static_cast<uint8_t>(1u << (bit & 7));
            }
        }
    }
    qrGenerated = true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Initialisierung
// ─────────────────────────────────────────────────────────────────────────────
void init()
{
    // Wire.begin() is called internally by u8g2.begin() with the pins
    // passed to the U8G2 constructor — calling it twice triggers the
    // double i2cInit that leaves the bus in ESP_ERR_INVALID_STATE.
    if (!u8g2.begin()) {
        ESP_LOGE(TAG, "SSD1306 nicht gefunden! SDA=GPIO%d SCL=GPIO%d",
                 PIN_I2C_SDA, PIN_I2C_SCL);
        initialized = false;
        return;
    }
    // setClock NACH begin() – sonst überschreibt u8g2.begin() die Einstellung
    Wire.setClock(400000);
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.setContrast(180);
    initialized = true;
    ESP_LOGI(TAG, "SSD1306 128x64 OK (SDA=GPIO%d SCL=GPIO%d)",
             PIN_I2C_SDA, PIN_I2C_SCL);
    showBootScreen();
}

void showResetWarning()
{
    if (!initialized) return;
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_9x15B_tf);
    u8g2.drawStr(5, 20,  "FACTORY");
    u8g2.drawStr(5, 38,  "RESET!");
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(0, 55, "Matter wird");
    u8g2.drawStr(0, 64, "zurueckgesetzt...");
    u8g2.sendBuffer();
}

// ─────────────────────────────────────────────────────────────────────────────
// Boot-Screen
// ─────────────────────────────────────────────────────────────────────────────
void showBootScreen()
{
    if (!initialized) return;

    u8g2.clearBuffer();

    u8g2.setFont(u8g2_font_9x15B_tf);
    u8g2.drawStr(10, 20, "SolarControl");

    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(20, 35, "Matter Edition");
    u8g2.drawStr(15, 48, "ESP32-S3 N16R8V");

    u8g2.drawHLine(0, 52, 128);
    u8g2.drawStr(20, 63, "Initialisiere...");

    u8g2.sendBuffer();
}

// ─────────────────────────────────────────────────────────────────────────────
// Commissioning-Screen mit QR-Code + Pairing-Code
// QR-Code wird nur einmal generiert und im RAM gecacht
// ─────────────────────────────────────────────────────────────────────────────
void showCommissioningScreen(const char* qrCodePayload,
                              const char* pairingCode)
{
    if (!initialized) return;

    if (!qrGenerated && qrCodePayload != nullptr && strlen(qrCodePayload) > 0) {
        esp_qrcode_config_t cfg = {
            .display_func       = qr_store_cb,
            .max_qrcode_version = 2,
            .qrcode_ecc_level   = ESP_QRCODE_ECC_LOW,
        };
        esp_err_t err = esp_qrcode_generate(&cfg, qrCodePayload);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "QR-Code generiert (%dx%d)", qrSize, qrSize);
        } else {
            ESP_LOGE(TAG, "QR-Code Generierung fehlgeschlagen: %s", esp_err_to_name(err));
        }
    }

    u8g2.clearBuffer();

    if (qrGenerated) {
        // QR-Code aus Bitmap-Cache zeichnen: 2px/Modul, Position x=1 y=7
        constexpr uint8_t QR_PIXEL = 2;
        constexpr uint8_t QR_X     = 1;
        constexpr uint8_t QR_Y     = 7;
        for (uint8_t y = 0; y < qrSize; y++) {
            for (uint8_t x = 0; x < qrSize; x++) {
                uint16_t bit = static_cast<uint16_t>(y) * qrSize + x;
                if (qrBits[bit >> 3] & (1u << (bit & 7))) {
                    u8g2.drawBox(QR_X + x*QR_PIXEL,
                                  QR_Y + y*QR_PIXEL,
                                  QR_PIXEL, QR_PIXEL);
                }
            }
        }
    }

    // Pairing-Code rechts
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(55, 10, "Pairing:");

    char line1[10] = {0};
    char line2[6]  = {0};
    size_t len = strlen(pairingCode);
    if (len >= 8) {
        snprintf(line1, sizeof(line1), "%c%c%c%c-%c%c%c",
                 pairingCode[0], pairingCode[1], pairingCode[2], pairingCode[3],
                 pairingCode[4], pairingCode[5], pairingCode[6]);
        snprintf(line2, sizeof(line2), "%c", pairingCode[7]);
    } else {
        strncpy(line1, pairingCode, sizeof(line1)-1);
    }

    u8g2.setFont(u8g2_font_7x13B_tf);
    u8g2.drawStr(54, 27, line1);
    u8g2.drawStr(54, 43, line2);

    u8g2.setFont(u8g2_font_5x7_tf);
    u8g2.drawStr(0, 6, "Matter Commissioning");
    u8g2.drawHLine(0, 55, 128);
    u8g2.drawStr(5, 64, "Scan mit Home-App");
    u8g2.sendBuffer();
}

// ─────────────────────────────────────────────────────────────────────────────
// Screen 1/3: Temperaturen
// Wie Original – alle Temperaturen in einer Übersicht
// ─────────────────────────────────────────────────────────────────────────────
void showTemperatures()
{
    if (!initialized) return;

    const SolarLogic::State& s = SolarLogic::state;
    char buf[22];

    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x10_tf);

    // Header (Original-Style)
    u8g2.drawStr(0, 9, "-- Temperaturen --");
    u8g2.setFont(u8g2_font_5x7_tf);
    u8g2.drawStr(110, 9, "1/3");
    u8g2.drawHLine(0, 11, 128);

    // Temperaturen (Original-Font: u8g2_font_6x10_tf)
    u8g2.setFont(u8g2_font_6x10_tf);

    snprintf(buf, sizeof(buf), "Dach:   %5.1f\xb0""C",
             s.roofTemp);
    u8g2.drawStr(0, 22, buf);

    snprintf(buf, sizeof(buf), "Boiler: %5.1f\xb0""C",
             s.boilerTemp);
    u8g2.drawStr(0, 33, buf);

    snprintf(buf, sizeof(buf), "Puffer: %5.1f\xb0""C",
             s.storageTemp);
    u8g2.drawStr(0, 44, buf);

    snprintf(buf, sizeof(buf), "Rueckl: %5.1f\xb0""C",
             s.backflowTemp);
    u8g2.drawStr(0, 55, buf);

    snprintf(buf, sizeof(buf), "Pool:   %5.1f\xb0""C",
             s.poolTemp);
    u8g2.drawStr(0, 64, buf);

    u8g2.sendBuffer();
}

// ─────────────────────────────────────────────────────────────────────────────
// Screen 2/3: Status
// ─────────────────────────────────────────────────────────────────────────────
void showStatus()
{
    if (!initialized) return;

    const SolarLogic::State& s = SolarLogic::state;

    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x10_tf);

    u8g2.drawStr(0, 9, "---- Status ----");
    u8g2.setFont(u8g2_font_5x7_tf);
    u8g2.drawStr(110, 9, "2/3");
    u8g2.drawHLine(0, 11, 128);

    // Modus groß zentriert
    const char* modeStr =
        s.currentMode == SolarLogic::Mode::AUTO        ? "AUTO"   :
        s.currentMode == SolarLogic::Mode::MANUAL_POOL ? "POOL"   :
                                                          "BOILER";
    u8g2.setFont(u8g2_font_9x15B_tf);
    uint8_t modeX = (128 - (strlen(modeStr) * 9)) / 2;
    u8g2.drawStr(modeX, 27, modeStr);
    u8g2.drawHLine(0, 30, 128);

    // Status-Spalten
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(0,  42, "Pumpe");
    u8g2.drawStr(35, 42, "    Ventil");
    u8g2.drawStr(68, 42, "Zirk.");
    u8g2.drawStr(100, 42, "Bel.");

    u8g2.setFont(u8g2_font_7x13B_tf);
    u8g2.drawStr(2,  55, s.pumpRunning    ? "AN"  : "AUS");
    u8g2.setFont(u8g2_font_5x7_tf);
    u8g2.drawStr(33, 55, s.valvePool      ? "POOL" : "BOILR");
    u8g2.setFont(u8g2_font_7x13B_tf);
    u8g2.drawStr(70, 55, s.circulationOn  ? "AN"  : "AUS");
    u8g2.drawStr(100, 55, s.illuminationOn ? "AN"  : "AUS");

    // Pegelstand + Balken
    u8g2.drawHLine(0, 57, 128);
    u8g2.setFont(u8g2_font_5x7_tf);
    char levelBuf[16];
    snprintf(levelBuf, sizeof(levelBuf), "Pegel:%4.0f%%", s.levelPct);
    u8g2.drawStr(0, 64, levelBuf);

    // Balken: x=52..127 (75px breit), y=59..63 (5px hoch)
    uint8_t barFill = static_cast<uint8_t>(s.levelPct * 75.0f / 100.0f);
    u8g2.drawFrame(52, 59, 75, 5);
    if (barFill > 0) u8g2.drawBox(52, 59, barFill, 5);

    // Warnung blinken
    if (s.levelWarn && (millis() / 400) % 2 == 0) {
        u8g2.drawStr(120, 64, "!");
    }

    u8g2.sendBuffer();
}

// ─────────────────────────────────────────────────────────────────────────────
// Screen 3/3: Netzwerk & Matter
// ─────────────────────────────────────────────────────────────────────────────
void showNetwork()
{
    if (!initialized) return;
    const SolarLogic::State& s = SolarLogic::state;

    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(0, 9, "--- Netzwerk ---");
    u8g2.setFont(u8g2_font_5x7_tf);
    u8g2.drawStr(110, 9, "3/3");
    u8g2.drawHLine(0, 11, 128);

    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(0, 22, "WiFi:");
    if (wifiConnected) {
        u8g2.drawStr(36, 22, "OK");
        u8g2.drawStr(0,  33, "IP:");
        u8g2.setFont(u8g2_font_5x7_tf);
        u8g2.drawStr(18, 33, wifiIP);
    } else {
        u8g2.setFont(u8g2_font_7x13B_tf);
        u8g2.drawStr(36, 22, "KEIN WiFi!");
        u8g2.setFont(u8g2_font_6x10_tf);
        u8g2.drawStr(0, 33, "IP: ---");
    }

    u8g2.drawHLine(0, 35, 128);
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(0, 46, "Matter:");
    if (isCommissioned) {
        u8g2.drawStr(50, 46, "aktiv");
    } else {
        if ((millis() / 500) % 2 == 0) {
            u8g2.setFont(u8g2_font_7x13B_tf);
            u8g2.drawStr(50, 46, "PAIRING");
        }
    }

    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(0, 57, "Pool:");
    // KORRIGIERT: TEMP_SENSOR_INVALID direkt (ist globales constexpr, kein Namespace)
    bool poolConnected = (s.poolTemp > TEMP_SENSOR_INVALID);
    if (poolConnected) {
        char poolBuf[18];
        u8g2.setFont(u8g2_font_5x7_tf);
        snprintf(poolBuf, sizeof(poolBuf), "%.1f\xb0""C/%.1f\xb0""C",
                 s.poolTemp, s.poolSollTemp);
        u8g2.drawStr(32, 57, poolBuf);
    } else {
        u8g2.setFont(u8g2_font_5x7_tf);
        u8g2.drawStr(32, 57, "keine Daten");
    }

    u8g2.drawHLine(0, 58, 128);
    u8g2.setFont(u8g2_font_5x7_tf);
    float refTemp = s.valvePool ? s.poolSollTemp : s.boilerTemp;
    float diff    = s.roofTemp - refTemp;
    char diffBuf[26];
    snprintf(diffBuf, sizeof(diffBuf), "dT:%+.1f EIN:%.0f AUS:%.0f",
             diff, TEMP_DIFF_ON, TEMP_DIFF_OFF);
    u8g2.drawStr(0, 64, diffBuf);
    u8g2.sendBuffer();
}

// ─────────────────────────────────────────────────────────────────────────────
// Fehler-Screen
// ─────────────────────────────────────────────────────────────────────────────
void showError(const char* errorMsg)
{
    if (!initialized) return;

    u8g2.clearBuffer();

    // Blinkender Rahmen
    if ((millis() / 300) % 2 == 0) {
        u8g2.drawFrame(0, 0, 128, 64);
        u8g2.drawFrame(2, 2, 124, 60);
    }

    u8g2.setFont(u8g2_font_9x15B_tf);
    u8g2.drawStr(22, 20, "FEHLER!");

    u8g2.setFont(u8g2_font_6x10_tf);
    // Max 21 Zeichen pro Zeile bei Font 6x10
    char line1[22] = {0};
    char line2[22] = {0};
    size_t msgLen  = strlen(errorMsg);

    if (msgLen <= 21) {
        strncpy(line1, errorMsg, 21);
    } else {
        strncpy(line1, errorMsg,      21);
        strncpy(line2, errorMsg + 21, 21);
    }

    u8g2.drawStr(0, 38, line1);
    if (strlen(line2) > 0) {
        u8g2.drawStr(0, 50, line2);
    }

    u8g2.sendBuffer();
}

// ─────────────────────────────────────────────────────────────────────────────
// Screen manuell setzen (z.B. via Tastendruck)
// ─────────────────────────────────────────────────────────────────────────────
void setScreen(Screen screen)
{
    currentScreen    = screen;
    lastScreenChange = millis();
}

// ─────────────────────────────────────────────────────────────────────────────
// Update – automatische Screen-Rotation im Normal-Betrieb
// ─────────────────────────────────────────────────────────────────────────────
void update(uint32_t screenIntervalMs)
{
    if (!initialized)    return;
    if (!isCommissioned) return;

    unsigned long now = millis();
    if (now - lastScreenChange >= screenIntervalMs) {
        lastScreenChange = now;
        currentScreen = static_cast<Screen>(
            (static_cast<uint8_t>(currentScreen) + 1) %
             static_cast<uint8_t>(Screen::SCREEN_COUNT));
    }

    switch (currentScreen) {
        case Screen::TEMPERATURES: showTemperatures(); break;
        case Screen::STATUS:       showStatus();       break;
        case Screen::NETWORK:      showNetwork();      break;
        default:                   showTemperatures(); break;
    }
}

} // namespace Display

