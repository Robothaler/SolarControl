#include "../include/Display.h"
#include "SolarLogic.h"
#include "Config.h"

#include <Arduino.h>
#include <cstring>
#include <U8g2lib.h>
#include <Wire.h>
#include <esp_log.h>
#include <qrcode.h>   // espressif__qrcode (via idf_component.yml)
#include <esp_netif.h>
#include <driver/gpio.h>
#include <rom/ets_sys.h>

static const char* TAG = "Display";

// Drehung wie Arduino-Original (Robothaler/SolarControl main.ino):
//   u8g2.setDisplayRotation(U8G2_R3);
// Hochformat → logische Zeichenfläche typ. 64×128 (nach begin + Rotation).
// Falls Inhalt 90° falsch: U8G2_R1 hier setzen.
#ifndef DISPLAY_U8G2_ROTATION
  #define DISPLAY_U8G2_ROTATION U8G2_R3
#endif

// ─────────────────────────────────────────────────────────────────────────────
// U8g2 Display-Objekt
// SSD1309, Full-Buffer (_F_): clearBuffer/sendBuffer aktualisiert den kompletten
// Schirm. (Der Arduino-Sketch nutzte _1_ mit firstPage/nextPage – äquivalent.)
// NONAME0 = SSD1306-kompatible Init (Beipack-Zettel).
// ─────────────────────────────────────────────────────────────────────────────
static U8G2_SSD1309_128X64_NONAME0_F_SW_I2C u8g2(
    U8G2_R0,
    /* clock=*/ PIN_I2C_SCL,
    /* data= */ PIN_I2C_SDA,
    /* reset=*/ U8X8_PIN_NONE);

/** Nach setDisplayRotation(): effektive Breite/Höhe (Hochformat: 64×128). */
static uint8_t dW = 128;
static uint8_t dH = 64;

namespace Display {

// Öffentliche Status-Flags
bool isCommissioned = false;
bool wifiConnected  = false;
char wifiIP[16]     = "---";
Screen currentScreen  = Screen::TEMPERATURES;

// Interne Variablen
static unsigned long lastScreenChange = 0;
static bool          initialized      = false;

// QR-Code Bitmap-Cache — Matter-Payload braucht oft Version 4–6 (33×33 … 41×41).
// Zuvor: max. 25×25 erzwungen → bei größerem QR nur Ecke gespeichert → ungültiger Code.
// Max. Seitenlänge Version 7 = 45 (reicht für MT:-Payload + ECC Low).
static constexpr uint8_t QR_MODULES = 45;
static uint8_t  qrBits[(QR_MODULES * QR_MODULES + 7) / 8] = {0};
static uint8_t  qrSize      = 0;   // Module pro Seite (nach erfolgreicher Generierung)
static bool     qrGenerated = false;
static char     s_cachedQrPayload[192] = {0};

// Display-Callback für esp_qrcode_generate() – speichert Module als Bitfeld
static void qr_store_cb(esp_qrcode_handle_t qrcode)
{
    uint8_t sz = static_cast<uint8_t>(esp_qrcode_get_size(qrcode));
    if (sz == 0 || sz > QR_MODULES) {
        ESP_LOGW(TAG, "QR: ungültige Größe %u (max %u) — Generierung verworfen",
                 sz, QR_MODULES);
        qrSize      = 0;
        qrGenerated = false;
        return;
    }
    qrSize = sz;
    memset(qrBits, 0, sizeof(qrBits));
    for (uint8_t y = 0; y < sz; y++) {
        for (uint8_t x = 0; x < sz; x++) {
            if (esp_qrcode_get_module(qrcode, x, y)) {
                const uint16_t bit = static_cast<uint16_t>(y) * sz + x;
                qrBits[bit >> 3] |= static_cast<uint8_t>(1u << (bit & 7));
            }
        }
    }
    qrGenerated = true;
}

// Manueller Pairing-Code: CHIP liefert z. B. "12345-67890" oder 11 Ziffern ohne Bindestrich.
static void splitManualPairingCode(const char* pairingCode,
                                     char*       line1,
                                     size_t      cap1,
                                     char*       line2,
                                     size_t      cap2)
{
    if (line1 && cap1) line1[0] = '\0';
    if (line2 && cap2) line2[0] = '\0';
    if (!pairingCode || !line1 || cap1 < 2 || !line2 || cap2 < 2) return;

    const char* hy = strchr(pairingCode, '-');
    if (hy) {
        size_t n1 = static_cast<size_t>(hy - pairingCode);
        if (n1 >= cap1) n1 = cap1 - 1;
        memcpy(line1, pairingCode, n1);
        line1[n1] = '\0';
        strncpy(line2, hy + 1, cap2 - 1);
        line2[cap2 - 1] = '\0';
    } else {
        const size_t n = strlen(pairingCode);
        const size_t n1 = (n >= 5) ? 5 : n;
        memcpy(line1, pairingCode, n1);
        line1[n1] = '\0';
        if (n > n1) {
            strncpy(line2, pairingCode + n1, cap2 - 1);
            line2[cap2 - 1] = '\0';
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Initialisierung
// ─────────────────────────────────────────────────────────────────────────────
// I2C bus recovery: clock 9 SCL cycles to release a slave stuck mid-byte.
// RTC_SW_CPU_RST does not reset I2C peripherals or connected devices.
// After a crash the SSD1306 may hold SDA low, blocking the bus.
// Standard recovery (NXP UM10204 §3.1.16): clock SCL 9× with SDA high,
// then issue a manual STOP (SDA low→high while SCL high).
static void recoverI2cBus()
{
    // Configure both lines as open-drain outputs, pull high
    gpio_config_t cfg = {};
    cfg.pin_bit_mask = (1ULL << PIN_I2C_SCL) | (1ULL << PIN_I2C_SDA);
    cfg.mode         = GPIO_MODE_OUTPUT_OD;
    cfg.pull_up_en   = GPIO_PULLUP_ENABLE;
    cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    cfg.intr_type    = GPIO_INTR_DISABLE;
    gpio_config(&cfg);

    gpio_set_level((gpio_num_t)PIN_I2C_SDA, 1);
    gpio_set_level((gpio_num_t)PIN_I2C_SCL, 1);
    ets_delay_us(10);

    for (int i = 0; i < 9; i++) {
        gpio_set_level((gpio_num_t)PIN_I2C_SCL, 0);
        ets_delay_us(5);
        gpio_set_level((gpio_num_t)PIN_I2C_SCL, 1);
        ets_delay_us(5);
    }

    // Manual STOP condition: SDA low → high while SCL high
    gpio_set_level((gpio_num_t)PIN_I2C_SDA, 0);
    ets_delay_us(5);
    gpio_set_level((gpio_num_t)PIN_I2C_SCL, 1);
    ets_delay_us(5);
    gpio_set_level((gpio_num_t)PIN_I2C_SDA, 1);
    ets_delay_us(5);

    // Release to input so Wire.begin() can reconfigure as I2C
    gpio_set_direction((gpio_num_t)PIN_I2C_SCL, GPIO_MODE_INPUT);
    gpio_set_direction((gpio_num_t)PIN_I2C_SDA, GPIO_MODE_INPUT);
    ets_delay_us(10);
}

// Set DISPLAY_ENABLED 0 when no SSD1306 is wired (avoids ~80 I2C error lines
// on every boot from u8g2's init sequence writing to a non-existent device).
#define DISPLAY_ENABLED 1

void setOledSleep(bool sleep)
{
#if !DISPLAY_ENABLED
    (void)sleep;
    return;
#else
    if (!initialized) return;
    u8g2.setPowerSave(sleep ? 1 : 0);
#endif
}

void init()
{
#if !DISPLAY_ENABLED
    ESP_LOGW(TAG, "Display deaktiviert (DISPLAY_ENABLED=0)");
    initialized = false;
    return;
#else
    // Release any slave (SSD1306) stuck mid-byte from the previous boot.
    // Must run before Wire.begin() / u8g2.begin() so the bus is idle.
    recoverI2cBus();

    u8g2.begin();
    u8g2.setDisplayRotation(DISPLAY_U8G2_ROTATION);
    dW = u8g2.getDisplayWidth();
    dH = u8g2.getDisplayHeight();
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.setContrast(180);
    initialized = true;
    ESP_LOGI(TAG, "SSD1309 I2C OK %ux%u (SDA=GPIO%d SCL=GPIO%d)",
             dW, dH, PIN_I2C_SDA, PIN_I2C_SCL);
    showBootScreen();
#endif
}

void showResetWarning()
{
    if (!initialized) return;
    setOledSleep(false);
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_9x15B_tf);
    u8g2.drawStr(2, 28,  "FACTORY");
    u8g2.drawStr(2, 48,  "RESET!");
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(0, 72, "Matter wird");
    u8g2.drawStr(0, 88, "zurueckgesetzt...");
    u8g2.sendBuffer();
}

// ─────────────────────────────────────────────────────────────────────────────
// Boot-Screen
// ─────────────────────────────────────────────────────────────────────────────
void showBootScreen()
{
    if (!initialized) return;

    setOledSleep(false);
    u8g2.clearBuffer();

    u8g2.setFont(u8g2_font_9x15B_tf);
    u8g2.drawStr(2, 24, "Solar-");
    u8g2.drawStr(0, 44, "Control");

    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(0, 64, "Matter Edition");
    u8g2.drawStr(0, 78, "ESP32-S3");

    u8g2.drawHLine(0, 92, dW);
    u8g2.drawStr(0, 108, "Initialisiere...");

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

    setOledSleep(false);

    if (qrCodePayload && qrCodePayload[0]) {
        if (strncmp(s_cachedQrPayload, qrCodePayload, sizeof(s_cachedQrPayload) - 1) != 0) {
            strncpy(s_cachedQrPayload, qrCodePayload, sizeof(s_cachedQrPayload) - 1);
            s_cachedQrPayload[sizeof(s_cachedQrPayload) - 1] = '\0';
            qrGenerated = false;
            qrSize      = 0;
        }
    } else {
        s_cachedQrPayload[0] = '\0';
        qrGenerated          = false;
        qrSize               = 0;
    }

    if (!qrGenerated && qrCodePayload != nullptr && qrCodePayload[0] != '\0') {
        esp_qrcode_config_t cfg = {
            .display_func       = qr_store_cb,
            .max_qrcode_version = 7,
            .qrcode_ecc_level   = ESP_QRCODE_ECC_LOW,
        };
        esp_err_t err = esp_qrcode_generate(&cfg, qrCodePayload);
        if (err == ESP_OK && qrGenerated) {
            ESP_LOGI(TAG, "QR-Code generiert (%ux%u Module)", qrSize, qrSize);
        } else {
            ESP_LOGE(TAG, "QR-Code Generierung fehlgeschlagen: %s",
                     esp_err_to_name(err));
        }
    }

    u8g2.clearBuffer();
    u8g2.setFontMode(1);
    u8g2.setDrawColor(1);
    u8g2.setFont(u8g2_font_5x7_tf);
    u8g2.drawStr(0, 8, "Matter Pairing");

    const uint8_t headerEnd  = 12;
    const uint8_t codeBlockH = 34;
    const uint8_t footerH    = 18;
    uint8_t         pairYActual = 40;

    if (qrGenerated && qrSize > 0) {
        // Schwarze Module auf weißem Grund + ruhige Zone (ISO/IEC 18004); vorher: weiß auf schwarz.
        uint8_t px = 2;
        while (qrSize * px > dW && px > 1) {
            --px;
        }
        int maxQrPx = static_cast<int>(dH) - static_cast<int>(headerEnd)
                    - static_cast<int>(codeBlockH) - static_cast<int>(footerH);
        if (maxQrPx < static_cast<int>(qrSize)) {
            maxQrPx = static_cast<int>(qrSize);
        }
        while (static_cast<int>(qrSize) * px > maxQrPx && px > 1) {
            --px;
        }

        const uint8_t qrPx      = static_cast<uint8_t>(qrSize * px);
        const uint8_t quiet     = static_cast<uint8_t>(4 * px);
        const uint8_t totalSide = static_cast<uint8_t>(qrPx + 2 * quiet);
        const uint8_t qx = (dW > totalSide) ? static_cast<uint8_t>((dW - totalSide) / 2) : 0;
        const uint8_t qy = headerEnd;

        u8g2.setDrawColor(1);
        u8g2.drawBox(qx, qy, totalSide, totalSide);
        u8g2.setDrawColor(0);
        for (uint8_t y = 0; y < qrSize; y++) {
            for (uint8_t x = 0; x < qrSize; x++) {
                const uint16_t bit = static_cast<uint16_t>(y) * qrSize + x;
                if (qrBits[bit >> 3] & (1u << (bit & 7))) {
                    u8g2.drawBox(static_cast<uint8_t>(qx + quiet + x * px),
                                 static_cast<uint8_t>(qy + quiet + y * px),
                                 px, px);
                }
            }
        }
        u8g2.setDrawColor(1);

        pairYActual = static_cast<uint8_t>(qy + totalSide + 4);
    }

    u8g2.setFont(u8g2_font_5x7_tf);
    u8g2.setDrawColor(1);
    u8g2.drawStr(0, pairYActual, "Manuell:");
    char line1[16] = {0};
    char line2[16] = {0};
    if (pairingCode) {
        splitManualPairingCode(pairingCode, line1, sizeof(line1), line2, sizeof(line2));
    }
    u8g2.drawStr(0, static_cast<uint8_t>(pairYActual + 10), line1[0] ? line1 : "----");
    if (line2[0]) {
        u8g2.drawStr(0, static_cast<uint8_t>(pairYActual + 20), line2);
    }

    u8g2.setFont(u8g2_font_5x7_tf);
    if (dH > 100) {
        u8g2.drawHLine(0, static_cast<uint8_t>(dH - 14), dW);
        u8g2.drawStr(0, static_cast<uint8_t>(dH - 4), "Apple Home: Hinzuf.");
    }
    u8g2.sendBuffer();
}

// ─── Grafik-Stil Arduino Mega main.ino (write_lcd): Banner, Fonts, Buttons ─
static constexpr float kDisplayTankLitersFull = 4500.f; // wie max_liter im Original

static void drawOriginalBanner(const char* title)
{
    u8g2.drawBox(0, 0, dW, 12);
    u8g2.setFont(u8g2_font_beanstalk_mel_tr);
    u8g2.setFontMode(0);
    u8g2.setDrawColor(0);
    u8g2.drawStr(0, 11, title);
    u8g2.setFontMode(1);
    u8g2.setDrawColor(1);
}

/** IPv4 zweizeilig (~11 Zeichen/Zeile mit 5x7), Umbruch nach letztem Punkt vor Limit. */
static void drawIpAddressWrapped(const char* ip, uint8_t yBaselineFirstLine)
{
    u8g2.setFont(u8g2_font_5x7_tf);
    u8g2.setFontMode(1);
    u8g2.setDrawColor(1);
    if (ip == nullptr || ip[0] == '\0') {
        u8g2.drawStr(0, yBaselineFirstLine, "---");
        return;
    }
    const size_t n = strlen(ip);
    constexpr size_t kMaxCharsFirst = 11;
    if (n <= kMaxCharsFirst) {
        u8g2.drawStr(0, yBaselineFirstLine, ip);
        return;
    }
    size_t splitAt = 0;
    for (size_t i = 0; i < n && i < kMaxCharsFirst; ++i) {
        if (ip[i] == '.') {
            splitAt = i + 1;
        }
    }
    if (splitAt == 0) {
        splitAt = kMaxCharsFirst;
    }
    char line1[16];
    char line2[16];
    if (splitAt >= sizeof(line1)) {
        splitAt = sizeof(line1) - 1;
    }
    memcpy(line1, ip, splitAt);
    line1[splitAt] = '\0';
    strncpy(line2, ip + splitAt, sizeof(line2) - 1);
    line2[sizeof(line2) - 1] = '\0';

    u8g2.drawStr(0, yBaselineFirstLine, line1);
    if (line2[0] != '\0') {
        u8g2.drawStr(0, static_cast<uint8_t>(yBaselineFirstLine + 9), line2);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Screen 1/3: Temperaturen (Layout wie Mega write_lcd LCD_Page==false)
// ─────────────────────────────────────────────────────────────────────────────
void showTemperatures()
{
    if (!initialized) return;

    const SolarLogic::State& s = SolarLogic::state;

    u8g2.clearBuffer();
    drawOriginalBanner("TEMPERATUREN");

    u8g2.setFont(u8g2_font_squeezed_r6_tr);
    u8g2.setFontMode(1);
    u8g2.setDrawColor(1);
    u8g2.drawStr(0, 22, "Kollektor:");
    u8g2.setFont(u8g2_font_luBS08_tf);
    u8g2.setCursor(8, 35);
    if (s.roofTemp > TEMP_SENSOR_INVALID && s.roofTemp < TEMP_MAX_COLLECTOR) {
        u8g2.print(s.roofTemp, 1);
        u8g2.print("\xb0""C");
    } else {
        u8g2.print("---\xb0""C");
    }

    u8g2.setFont(u8g2_font_squeezed_r6_tr);
    u8g2.drawStr(0, 47, "Puffer:");
    u8g2.setFont(u8g2_font_luBS08_tf);
    u8g2.setCursor(8, 60);
    if (s.boilerTemp > TEMP_SENSOR_INVALID && s.boilerTemp < TEMP_MAX_BOILER) {
        u8g2.print(s.boilerTemp, 1);
        u8g2.print("\xb0""C");
    } else {
        u8g2.print("---\xb0""C");
    }

    u8g2.setFont(u8g2_font_squeezed_r6_tr);
    u8g2.drawStr(0, 72, "Pool:");
    u8g2.setFont(u8g2_font_luBS08_tf);
    if (s.poolTemp > TEMP_SENSOR_INVALID) {
        u8g2.setCursor(8, 85);
        u8g2.print(s.poolTemp, 1);
        u8g2.print("\xb0""C");
    } else {
        u8g2.setCursor(0, 85);
        u8g2.print("OFFLINE");
    }

    int fuell = static_cast<int>(s.levelPct * 61.0f / 100.0f);
    if (fuell > 61) {
        fuell = 61;
    }
    if (fuell < 0) {
        fuell = 0;
    }
    const int litDisp =
        static_cast<int>(s.levelPct * 0.01f * kDisplayTankLitersFull + 0.5f);

    u8g2.setFont(u8g2_font_squeezed_r6_tr);
    u8g2.setDrawColor(1);
    u8g2.drawStr(0, 100, "Heizoel:");
    u8g2.setDrawColor(2);
    u8g2.setFont(u8g2_font_squeezed_b7_tr);
    u8g2.setCursor(32, 101);
    u8g2.print(litDisp);
    u8g2.print(" l");
    u8g2.setCursor(20, 111);
    u8g2.print(static_cast<int>(s.levelPct + 0.5f));
    u8g2.print(" %");
    const uint8_t barOuterW = static_cast<uint8_t>((dW >= 63) ? 63 : dW);
    u8g2.drawFrame(0, 102, barOuterW, 11);
    if (fuell > 0) {
        u8g2.drawBox(1, 103, static_cast<uint8_t>(fuell), 9);
    }
    u8g2.setDrawColor(1);

    u8g2.drawLine(0, 118, static_cast<uint8_t>(dW - 1), 118);
    u8g2.setFont(u8g2_font_baby_tr);
    if (s.currentMode == SolarLogic::Mode::AUTO) {
        u8g2.drawStr(0, 126, "Auto: ");
        u8g2.setCursor(24, 126);
    } else {
        u8g2.drawStr(0, 126, "Manuell: ");
        u8g2.setCursor(35, 126);
    }
    u8g2.print(s.valvePool ? "POOL" : "PUFFER");

    u8g2.sendBuffer();
}

// ─────────────────────────────────────────────────────────────────────────────
// Screen 2/3: Status (ON/OFF-Buttons wie Mega; WiFi statt MQTT-Glyphe)
// ─────────────────────────────────────────────────────────────────────────────
void showStatus()
{
    if (!initialized) return;

    const SolarLogic::State& s = SolarLogic::state;

    u8g2.clearBuffer();

    u8g2.drawBox(0, 0, static_cast<uint8_t>((dW >= 63) ? 63 : dW), 12);
    u8g2.setFont(u8g2_font_beanstalk_mel_tr);
    u8g2.setFontMode(0);
    u8g2.setDrawColor(0);
    u8g2.drawStr(5, 11, "STATUS");
    if (wifiConnected) {
        u8g2.setFont(u8g2_font_waffle_t_all);
        u8g2.drawGlyph(46, 11, 0xe29b);
    } else {
        u8g2.setFont(u8g2_font_siji_t_6x10);
        u8g2.drawGlyph(46, 11, 0xe21b);
    }
    u8g2.setFontMode(1);
    u8g2.setDrawColor(1);

    const bool solarOn   = s.pumpRunning;
    const bool valveOn   = (digitalRead(PIN_RELAY_VALVE) == RELAY_ON);
    const bool zircOn    = (digitalRead(PIN_RELAY_CIRC) == RELAY_ON);
    const bool lightOn   = (digitalRead(PIN_RELAY_ILLUM) == RELAY_ON);
    const bool poolSense = (digitalRead(PIN_VALVE_STATUS) == LOW);

    if (solarOn) {
        u8g2.setFont(u8g2_font_squeezed_r6_tr);
        u8g2.drawStr(0, 22, "Solarpumpe:");
        u8g2.drawButtonUTF8(33, 33, U8G2_BTN_INV | U8G2_BTN_BW2, 10, 1, 1, "ON");
        u8g2.drawButtonUTF8(56, 33, U8G2_BTN_HCENTER | U8G2_BTN_BW2, 10, 1, 1, "OFF");
    } else {
        u8g2.setFont(u8g2_font_squeezed_r6_tr);
        u8g2.drawStr(0, 22, "Solarpumpe:");
        u8g2.drawButtonUTF8(38, 33, U8G2_BTN_HCENTER | U8G2_BTN_BW2, 10, 1, 1, "ON");
        u8g2.drawButtonUTF8(51, 33, U8G2_BTN_INV | U8G2_BTN_BW2, 10, 1, 1, "OFF");
    }

    if (valveOn) {
        u8g2.setFont(u8g2_font_squeezed_r6_tr);
        u8g2.drawStr(0, 46, "3-Wege-Ventil:");
        u8g2.drawButtonUTF8(33, 57, U8G2_BTN_INV | U8G2_BTN_BW2, 10, 1, 1, "ON");
        u8g2.drawButtonUTF8(56, 57, U8G2_BTN_HCENTER | U8G2_BTN_BW2, 10, 1, 1, "OFF");
    } else {
        u8g2.setFont(u8g2_font_squeezed_r6_tr);
        u8g2.drawStr(0, 46, "3-Wege-Ventil:");
        u8g2.drawButtonUTF8(38, 57, U8G2_BTN_HCENTER | U8G2_BTN_BW2, 10, 1, 1, "ON");
        u8g2.drawButtonUTF8(51, 57, U8G2_BTN_INV | U8G2_BTN_BW2, 10, 1, 1, "OFF");
    }
    if (poolSense) {
        u8g2.drawButtonUTF8(3, 57, U8G2_BTN_INV | U8G2_BTN_BW2, 0, 1, 1, " POOL ");
    } else {
        u8g2.drawButtonUTF8(3, 57, U8G2_BTN_INV | U8G2_BTN_BW2, 0, 1, 1, "PUFFER");
    }

    if (zircOn) {
        u8g2.setFont(u8g2_font_squeezed_r6_tr);
        u8g2.drawStr(0, 70, "Zirkulationspumpe:");
        u8g2.drawButtonUTF8(33, 81, U8G2_BTN_INV | U8G2_BTN_BW2, 10, 1, 1, "ON");
        u8g2.drawButtonUTF8(56, 81, U8G2_BTN_HCENTER | U8G2_BTN_BW2, 10, 1, 1, "OFF");
    } else {
        u8g2.setFont(u8g2_font_squeezed_r6_tr);
        u8g2.drawStr(0, 70, "Zirkulationspumpe:");
        u8g2.drawButtonUTF8(38, 81, U8G2_BTN_HCENTER | U8G2_BTN_BW2, 10, 1, 1, "ON");
        u8g2.drawButtonUTF8(51, 81, U8G2_BTN_INV | U8G2_BTN_BW2, 10, 1, 1, "OFF");
    }

    if (lightOn) {
        u8g2.setFont(u8g2_font_squeezed_r6_tr);
        u8g2.drawStr(0, 94, "Beleuchtung:");
        u8g2.drawButtonUTF8(33, 105, U8G2_BTN_INV | U8G2_BTN_BW2, 10, 1, 1, "ON");
        u8g2.drawButtonUTF8(56, 105, U8G2_BTN_HCENTER | U8G2_BTN_BW2, 10, 1, 1, "OFF");
    } else {
        u8g2.setFont(u8g2_font_squeezed_r6_tr);
        u8g2.drawStr(0, 94, "Beleuchtung:");
        u8g2.drawButtonUTF8(38, 105, U8G2_BTN_HCENTER | U8G2_BTN_BW2, 10, 1, 1, "ON");
        u8g2.drawButtonUTF8(51, 105, U8G2_BTN_INV | U8G2_BTN_BW2, 10, 1, 1, "OFF");
    }

    u8g2.drawLine(0, 118, static_cast<uint8_t>(dW - 1), 118);
    u8g2.setFont(u8g2_font_baby_tr);
    if (s.currentMode == SolarLogic::Mode::AUTO) {
        u8g2.drawStr(0, 126, "Auto: ");
        u8g2.setCursor(24, 126);
    } else {
        u8g2.drawStr(0, 126, "Manuell: ");
        u8g2.setCursor(35, 126);
    }
    u8g2.print(s.valvePool ? "POOL" : "PUFFER");

    u8g2.sendBuffer();
}

// ─────────────────────────────────────────────────────────────────────────────
// Screen 3/3: Netzwerk & Matter (gleicher Schrift-/Banner-Stil)
// ─────────────────────────────────────────────────────────────────────────────
void showNetwork()
{
    if (!initialized) return;
    const SolarLogic::State& s = SolarLogic::state;

    u8g2.clearBuffer();
    drawOriginalBanner("NETZWERK");

    u8g2.setFont(u8g2_font_squeezed_r6_tr);
    u8g2.setFontMode(1);
    u8g2.setDrawColor(1);
    u8g2.drawStr(0, 22, "WiFi:");
    u8g2.setFont(u8g2_font_luBS08_tf);
    u8g2.setCursor(8, 35);
    if (wifiConnected) {
        u8g2.print("OK");
    } else {
        u8g2.print("aus");
    }

    u8g2.setFont(u8g2_font_squeezed_r6_tr);
    u8g2.drawStr(0, 44, "IP:");
    if (wifiConnected) {
        drawIpAddressWrapped(wifiIP, 54);
    } else {
        drawIpAddressWrapped("---", 54);
    }

    u8g2.setFont(u8g2_font_squeezed_r6_tr);
    u8g2.drawStr(0, 72, "Matter:");
    u8g2.setFont(u8g2_font_luBS08_tf);
    u8g2.setCursor(8, 84);
    if (isCommissioned) {
        u8g2.print("OK");
    } else if ((millis() / 500) % 2 == 0) {
        u8g2.print("Pairing");
    } else {
        u8g2.print("...");
    }

    // Trenner: darunter Solar-Pumpen-Info (gehört zusammen mit EIN/AUS-Schwellen)
    u8g2.drawLine(0, 90, static_cast<uint8_t>(dW - 1), 90);

    u8g2.setFont(u8g2_font_squeezed_r6_tr);
    u8g2.drawStr(0, 98, "Solar dT:");
    u8g2.setFont(u8g2_font_luBS08_tf);
    u8g2.setCursor(0, 109);
    float refTemp = s.valvePool ? s.poolSollTemp : s.boilerTemp;
    float diff    = s.roofTemp - refTemp;
    char diffBuf[20];
    snprintf(diffBuf, sizeof(diffBuf), "%+.1fK", diff);
    u8g2.print(diffBuf);

    u8g2.setFont(u8g2_font_baby_tr);
    char einLine[18];
    char ausLine[18];
    snprintf(einLine, sizeof(einLine), "EIN: %d\xb0""C",
             static_cast<int>(SolarLogic::tunables.tempDiffOn + 0.5f));
    snprintf(ausLine, sizeof(ausLine), "AUS: %d\xb0""C",
             static_cast<int>(SolarLogic::tunables.tempDiffOff + 0.5f));
    u8g2.drawStr(0, 118, einLine);
    u8g2.drawStr(0, 126, ausLine);

    u8g2.sendBuffer();
}

// ─────────────────────────────────────────────────────────────────────────────
// Fehler-Screen
// ─────────────────────────────────────────────────────────────────────────────
void showError(const char* errorMsg)
{
    if (!initialized) return;

    setOledSleep(false);
    u8g2.clearBuffer();

    // Blinkender Rahmen
    if ((millis() / 300) % 2 == 0) {
        u8g2.drawFrame(0, 0, dW, dH);
        if (dW > 4 && dH > 4) {
            u8g2.drawFrame(2, 2, static_cast<uint8_t>(dW - 4),
                            static_cast<uint8_t>(dH - 4));
        }
    }

    u8g2.setFont(u8g2_font_9x15B_tf);
    u8g2.drawStr(4, 28, "FEHLER!");

    u8g2.setFont(u8g2_font_6x10_tf);
    // Hochformat: ~10 Zeichen/Zeile bei 6x10 auf 64px
    char line1[14] = {0};
    char line2[14] = {0};
    size_t msgLen  = strlen(errorMsg);
    constexpr size_t kWrap = 10;

    if (msgLen <= kWrap) {
        strncpy(line1, errorMsg, sizeof(line1) - 1);
    } else {
        strncpy(line1, errorMsg, kWrap);
        strncpy(line2, errorMsg + kWrap, sizeof(line2) - 1);
    }

    u8g2.drawStr(0, 52, line1);
    if (strlen(line2) > 0) {
        u8g2.drawStr(0, 64, line2);
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

