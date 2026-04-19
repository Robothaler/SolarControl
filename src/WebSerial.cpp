// =============================================================================
//  WebSerial.cpp
// =============================================================================
//  Implementierung siehe WebSerial.h.
//
//  Synchronisation:
//    • _outmux   schützt _outbuf / _wpos / _rpos (Ring-Buffer)
//    • _inmux    schützt _input_q / _in_w / _in_r (Eingabe-FIFO)
//    • portMUX_TYPE = Spinlock — billig, aus jedem Task aufrufbar.
//
//  Der vprintf-Hook (log_vprintf) läuft im Kontext des aufrufenden Tasks
//  (typ. CHIP/Matter, Arduino-loop, broadcast_task). Da Spinlocks IRQ-safe
//  sind, ist der Hook auch dann unkritisch wenn ESP-IDF mal aus IRQ-Kontext
//  loggt (selten, aber möglich).
// =============================================================================

#include "WebSerial.h"

#include <Arduino.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

// ── Ring-Buffer für Output ────────────────────────────────────────────────────
static constexpr size_t OUTBUF_SIZE = 4096;
static char             _outbuf[OUTBUF_SIZE];
static volatile size_t  _wpos = 0;
static volatile size_t  _rpos = 0;
static portMUX_TYPE     _outmux = portMUX_INITIALIZER_UNLOCKED;

// ── Eingabe-FIFO ──────────────────────────────────────────────────────────────
static constexpr size_t  INPUT_LINE_MAX = 128;
static constexpr uint8_t INPUT_QUEUE    = 8;
static char              _input_q[INPUT_QUEUE][INPUT_LINE_MAX];
static volatile uint8_t  _in_w   = 0;
static volatile uint8_t  _in_r   = 0;
static portMUX_TYPE      _inmux  = portMUX_INITIALIZER_UNLOCKED;

// ── State ─────────────────────────────────────────────────────────────────────
static volatile bool   _active        = false;
static vprintf_like_t  _orig_vprintf  = nullptr;
static volatile bool   _hook_installed = false;

// ── Interne Helfer ────────────────────────────────────────────────────────────
static void buf_write(const char* s, size_t n)
{
    if (!s || n == 0) return;
    portENTER_CRITICAL(&_outmux);
    for (size_t i = 0; i < n; i++) {
        size_t next = (_wpos + 1) % OUTBUF_SIZE;
        if (next == _rpos) {
            // Overflow → ältestes Byte verwerfen (FIFO bleibt voll)
            _rpos = (_rpos + 1) % OUTBUF_SIZE;
        }
        _outbuf[_wpos] = s[i];
        _wpos = next;
    }
    portEXIT_CRITICAL(&_outmux);
}

// vprintf-Hook: tee zu Original (Serial/UART) + Ring-Buffer (wenn aktiv).
// Wir müssen die va_list für den Original-Aufruf erhalten → va_copy().
static int log_vprintf(const char* fmt, va_list args)
{
    if (_active) {
        char    tmp[256];
        va_list ac;
        va_copy(ac, args);
        int n = vsnprintf(tmp, sizeof(tmp), fmt, ac);
        va_end(ac);
        if (n > 0) {
            buf_write(tmp, (size_t)((n >= (int)sizeof(tmp)) ? sizeof(tmp) - 1 : n));
        }
    }
    return _orig_vprintf ? _orig_vprintf(fmt, args) : vprintf(fmt, args);
}

// =============================================================================
//  Public API
// =============================================================================
namespace WebSerial {

void begin()
{
    if (_hook_installed) return;
    _orig_vprintf   = esp_log_set_vprintf(log_vprintf);
    _hook_installed = true;
    ESP_LOGI("WebSerial", "vprintf-Hook installiert (passiv)");
}

bool start()
{
    if (_active) return true;
    _active = true;
    const char* msg =
        "\r\n"
        "=== WebSerial gestartet ===\r\n"
        "Tippe Kommandos genau wie auf der Hardware-UART (z.B. GET_STATE).\r\n"
        "============================\r\n";
    buf_write(msg, strlen(msg));
    return true;
}

void stop()
{
    if (!_active) return;
    const char* msg = "\r\n=== WebSerial gestoppt ===\r\n";
    buf_write(msg, strlen(msg));
    _active = false;
}

bool isActive() { return _active; }

bool drainOutput(char* dst, size_t cap, size_t* outLen)
{
    if (!dst || cap < 2) { if (outLen) *outLen = 0; return false; }

    size_t n = 0;
    portENTER_CRITICAL(&_outmux);
    while (_rpos != _wpos && n < cap - 1) {
        dst[n++] = _outbuf[_rpos];
        _rpos = (_rpos + 1) % OUTBUF_SIZE;
    }
    portEXIT_CRITICAL(&_outmux);

    dst[n] = '\0';
    if (outLen) *outLen = n;
    return n > 0;
}

void print(const char* s)
{
    if (!s) return;
    Serial.print(s);
    if (_active) buf_write(s, strlen(s));
}

void println(const char* s)
{
    if (s) Serial.println(s); else Serial.println();
    if (_active) {
        if (s) buf_write(s, strlen(s));
        buf_write("\r\n", 2);
    }
}

void println() { println(nullptr); }

int printf(const char* fmt, ...)
{
    char    buf[256];
    va_list args;
    va_start(args, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    Serial.print(buf);
    if (_active && n > 0) {
        buf_write(buf, (size_t)((n >= (int)sizeof(buf)) ? sizeof(buf) - 1 : n));
    }
    return n;
}

void pushInput(const char* line)
{
    if (!line || !*line) return;
    portENTER_CRITICAL(&_inmux);
    uint8_t next = (_in_w + 1) % INPUT_QUEUE;
    if (next != _in_r) {
        strncpy(_input_q[_in_w], line, INPUT_LINE_MAX - 1);
        _input_q[_in_w][INPUT_LINE_MAX - 1] = '\0';
        _in_w = next;
    }
    portEXIT_CRITICAL(&_inmux);
}

bool popInput(char* buf, size_t size)
{
    if (!buf || size < 2) return false;
    bool ok = false;
    portENTER_CRITICAL(&_inmux);
    if (_in_r != _in_w) {
        strncpy(buf, _input_q[_in_r], size - 1);
        buf[size - 1] = '\0';
        _in_r = (_in_r + 1) % INPUT_QUEUE;
        ok = true;
    }
    portEXIT_CRITICAL(&_inmux);
    return ok;
}

} // namespace WebSerial
