#pragma once
// Bewusst KEIN <Arduino.h> — würde via Arduino's IPAddress.h mit lwip's
// INADDR_NONE-Makro kollidieren, sobald MatterBridge.cpp uns nach
// MatterDevices.h (esp_matter -> lwip) einbindet. Wir brauchen hier nur
// uint8_t, das liefert <cstdint>.
#include <cstdint>

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

/** OLED u8g2 Power-Save (0=an, 1=Panel aus). Wird von SolarLogic PIR-Logik gesteuert. */
void setOledSleep(bool sleep);

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
