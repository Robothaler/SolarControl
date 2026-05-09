#pragma once

#include <cstdint>

// Pollt den PoolMaster per HTTP über dieselben logischen Daten wie Matter-
// Subscribe (Temperatur Ist/Soll °C + Solar-Anforderung bool).
namespace PoolHttpBridge {

enum class TokenSaveMode : uint8_t {
    Keep,  // NVS-Token unverändert lassen
    Clear,
    Set
};

bool isConfigured();

/**
 * Basis-URL ohne abschließenden Schrägstrich (z. B. http://pool:8080).
 * Leeres baseUrl → löscht Brücke in NVS.
 */
bool saveConfig(const char* baseUrl,
                TokenSaveMode tokenMode,
                const char* bearerTokenNonNullOnlyIfSetting,
                uint32_t pollIntervalMs);

void loadFromNvs();

/** HTTP GET wenn WLAN steht — Intervall aus NVS (Default 5000 ms). */
void poll();

/** True wenn die letzte Anfrage JSON mit poolTemp geliefert hat (innerhalb Sessionsfenster nicht zwinglich). */
bool lastFetchOk();

/** Bearer-Token in NVS hinterlegt (für Settings-GET ohne Klartext). */
bool hasTokenConfigured();

} // namespace PoolHttpBridge
