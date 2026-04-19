#pragma once
// =============================================================================
//  WebSerial.h — Opt-in Remote-Serial für die WebUI
// =============================================================================
//  Die WebSerial-Komponente ist standardmässig PASSIV.
//
//  • begin()  installiert einmalig einen globalen vprintf-Hook für ESP_LOG*.
//             Solange isActive() == false bleibt der Hook ein reiner Pass-
//             Through (eine Boolean-Abfrage pro Log-Zeile, sonst Null-Cost).
//
//  • start()  wird vom Browser via WS-Kommando "webserial_start" ausgelöst.
//             Ab dann werden alle ESP_LOG*-Zeilen UND alle Aufrufe von
//             WebSerial::printf/println in einen Ring-Buffer geschrieben, den
//             der WebUI-Broadcast-Task an die verbundenen Clients pusht.
//
//  • stop()   deaktiviert das Streaming (Hook bleibt installiert).
//
//  Eingaben (Tastatur im Terminal) landen via pushInput() in einer kleinen
//  FIFO und werden im Arduino-loop() von handleSerialCommands() konsumiert —
//  d.h. dieselbe Pipeline wie der Hardware-Serialport. So funktionieren
//  GET_STATE, OPEN_COMMISSIONING etc. auch remote.
//
//  Speicher: 4 KB Ring-Buffer (Out) + 8×128 B Input-Queue ≈ 5 KB im DRAM.
// =============================================================================

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdarg.h>

namespace WebSerial {

// Einmalig in WebUI::begin() aufrufen — installiert den vprintf-Hook.
void begin();

// Aktivieren / Deaktivieren via Browser. start() lässt eine Begrüssungszeile
// im Output erscheinen, damit der User Feedback hat.
bool start();
void stop();
bool isActive();

// Vom Broadcast-Task gerufen: kopiert pending Output (max. cap-1 Bytes) in
// dst (NUL-terminiert) und gibt true zurück, wenn Daten geliefert wurden.
// outLen wird mit der Anzahl Bytes (ohne NUL) befüllt.
bool drainOutput(char* dst, size_t cap, size_t* outLen);

// Tee-Print: schreibt immer auf die Hardware-UART (Serial), und zusätzlich
// in den Ring-Buffer wenn isActive(). Drop-in-Ersatz für Serial.print*.
void  print  (const char* s);
void  println(const char* s);
void  println();
int   printf (const char* fmt, ...) __attribute__((format(printf, 1, 2)));

// Eingabe-FIFO. pushInput() vom WS-Handler, popInput() von main loop.
// line darf KEIN '\n' am Ende enthalten; Größe ≤ 127 Bytes ratsam.
void pushInput(const char* line);
bool popInput (char* buf, size_t size);

} // namespace WebSerial
