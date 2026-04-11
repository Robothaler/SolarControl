#pragma once
#include <Arduino.h>

namespace Display {

enum class Screen : uint8_t {
    TEMPERATURES = 0,
    STATUS       = 1,
    NETWORK      = 2,
    SCREEN_COUNT = 3
};

// ── Öffentliche Status-Flags ──────────────────────────────────────────────────
extern bool    isCommissioned;
extern bool    wifiConnected;
extern char    wifiIP[16];

// ── currentScreen öffentlich damit SolarLogic darauf zugreifen kann ──────────
extern Screen  currentScreen;   // ← war private static, jetzt extern

// ── Initialisierung ───────────────────────────────────────────────────────────
void init();

// ── Screens ───────────────────────────────────────────────────────────────────
void showBootScreen();
void showCommissioningScreen(const char* qrCodePayload,
                              const char* pairingCode);
void showTemperatures();
void showStatus();
void showNetwork();
void showError(const char* errorMsg);
void showResetWarning();    // ← neu: ersetzt u8g2_extern_show_reset_warning()

// ── Update & Navigation ───────────────────────────────────────────────────────
void update(uint32_t screenIntervalMs = 5000);
void setScreen(Screen screen);

} // namespace Display
