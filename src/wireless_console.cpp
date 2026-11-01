#include "wireless_console.h"
#include <esp_heap_caps.h>
#include <cstring>

WirelessConsole Console(&Serial);

WirelessConsole::WirelessConsole(Print* hwSerial)
    : _hwSerial(hwSerial)
{
    memset(_curLine, 0, sizeof(_curLine));
}

void WirelessConsole::_ensureAllocated()
{
    if (_lines) return;
    const size_t bytes = kMaxLogs * kLineBufSize;
    _lines = static_cast<char(*)[kLineBufSize]>(
        heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (!_lines) {
        _lines = static_cast<char(*)[kLineBufSize]>(malloc(bytes));
    }
    if (_lines) {
        memset(_lines, 0, bytes);
    }
}

void WirelessConsole::begin(unsigned long baud)
{
    (void)baud;
    _ensureAllocated();
}

void WirelessConsole::loop()
{
    // Guard: Only initialize Telnet server once Wi-Fi or AP is actively running
    if (WiFi.status() == WL_CONNECTED || WiFi.getMode() == WIFI_AP || WiFi.getMode() == WIFI_AP_STA) {
        if (!_telnetServer) {
            _telnetServer = new WiFiServer(23);
            _telnetServer->begin();
            _telnetServer->setNoDelay(true);
        }
        if (_telnetServer->hasClient()) {
            if (!_telnetClient || !_telnetClient.connected()) {
                if (_telnetClient) _telnetClient.stop();
                _telnetClient = _telnetServer->available();
                _telnetClient.println("\n=== Connected to ESP32-S3-DSP Live Console (Telnet) ===");
                _telnetClient.print(getRecentLogs());
            } else {
                WiFiClient newClient = _telnetServer->available();
                newClient.println("ESP32-S3-DSP: another telnet session is active. Closing.");
                newClient.stop();
            }
        }
    }
}

size_t WirelessConsole::write(uint8_t c)
{
    // 1. Output to physical USB-CDC / UART Serial
    if (_hwSerial) {
        _hwSerial->write(c);
    }

    // 2. Buffer line
    if (c == '\r') {
        return 1;
    }

    if (c == '\n') {
        _curLine[_curPos] = '\0';

        _ensureAllocated();
        if (_lines) {
            strncpy(_lines[_head], _curLine, kLineBufSize - 1);
            _lines[_head][kLineBufSize - 1] = '\0';
            _head = (_head + 1) % kMaxLogs;
            if (_count < kMaxLogs) _count++;
        }

        // Stream to active Telnet client if connected
        if (_telnetClient && _telnetClient.connected()) {
            _telnetClient.println(_curLine);
        }

        // Broadcast to WebSocket clients if callback is set
        if (_broadcast) {
            _broadcast(_curLine);
        }

        _curPos = 0;
        _curLine[0] = '\0';
        return 1;
    }

    if (_curPos < kLineBufSize - 1) {
        _curLine[_curPos++] = static_cast<char>(c);
    }

    return 1;
}

size_t WirelessConsole::write(const uint8_t *buffer, size_t size)
{
    if (!buffer || size == 0) return 0;
    for (size_t i = 0; i < size; ++i) {
        write(buffer[i]);
    }
    return size;
}

String WirelessConsole::getRecentLogs() const
{
    if (!_lines || _count == 0) return String();

    String out;
    out.reserve(_count * 80);

    size_t start = (_count < kMaxLogs) ? 0 : _head;
    for (size_t i = 0; i < _count; ++i) {
        size_t idx = (start + i) % kMaxLogs;
        out += _lines[idx];
        out += "\n";
    }
    return out;
}

void WirelessConsole::clear()
{
    _head = 0;
    _count = 0;
    _curPos = 0;
    if (_lines) {
        memset(_lines, 0, kMaxLogs * kLineBufSize);
    }
    memset(_curLine, 0, sizeof(_curLine));
}

extern "C" void console_log(const char* buf) {
    if (buf) {
        Console.print(buf);
    }
}
