#pragma once

#include <stddef.h>
#include <stdint.h>

// Ringpuffer für WebUI-Verlaufsdiagramm (RAM, ~30 s Raster, ~2 h).
void HistoryLog_init();
void HistoryLog_maybeSample();

// JSON {"type":"history","intervalSec":30,"data":[[ts,roof,boiler,storage,backflow,pool,poolSoll,pump,valve,circ,illum],...]}
// Rückgabe: geschriebene Bytes (ohne NUL) oder 0 bei Fehler.
size_t HistoryLog_buildJson(char* buf, size_t cap);

// Langzeit: SPIFFS (PoolMaster-ähnlich), gleiches JSON-Format; optional RAM-Tail
// (frischere Punkte nach letztem Flash-Eintrag). fromTs/toTs = Unix-UTC.
// maxPts: Ziel-Decimation aus den Flash-Daten (0 → 800).
size_t HistoryLog_buildJsonRange(uint32_t fromTs, uint32_t toTs, uint16_t maxPts,
                                   char* buf, size_t cap);
