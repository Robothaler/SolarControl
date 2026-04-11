#pragma once

namespace WebUI {

// Startet HTTP-Server (Port 80), WebSocket (/ws) und OTA-Endpoint (/update).
// Muss nach WiFi-Verbindung aufgerufen werden.
void begin();

// Stoppt den HTTP-Server.
void stop();

// Verarbeitet ausstehende WebSocket-Kommandos (Relay/Modus-Änderungen).
// Muss im Arduino loop() aufgerufen werden – führt SolarLogic-Aktionen aus.
void processCommands();

} // namespace WebUI
