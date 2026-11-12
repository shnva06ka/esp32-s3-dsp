#include "wireless_console.h"
#include <esp_heap_caps.h>
#include <cstring>

WirelessConsole Console(&Serial);

WirelessConsole::WirelessConsole(Print* hwSerial)
    : _hwSerial(hwSerial),
      _mutex(xSemaphoreCreateRecursiveMutex())
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
            if (_mutex) xSemaphoreTakeRecursive(_mutex, portMAX_DELAY);
            if (!_telnetClient || !_telnetClient.connected()) {
                if (_telnetClient) _telnetClient.stop();
                _telnetClient = _telnetServer->available();
                if (_telnetClient) {
                    _telnetClient.setNoDelay(true);
                    static const char kBanner[] = "\n=== Connected to ESP32-S3-DSP Live Console (Telnet) ===\r\n";
                    size_t totalLen = sizeof(kBanner) - 1;
                    if (_lines && _count > 0) {
                        size_t start = (_count < kMaxLogs) ? 0 : _head;
                        for (size_t i = 0; i < _count; ++i) {
                            size_t idx = (start + i) % kMaxLogs;
                            totalLen += strlen(_lines[idx]) + 2;
                        }
                    }
                    String replay;
                    if (replay.reserve(totalLen)) {
                        replay += kBanner;
                        if (_lines && _count > 0) {
                            size_t start = (_count < kMaxLogs) ? 0 : _head;
                            for (size_t i = 0; i < _count; ++i) {
                                size_t idx = (start + i) % kMaxLogs;
                                replay += _lines[idx];
                                replay += "\r\n";
                            }
                        }
                        _telnetClient.write(reinterpret_cast<const uint8_t*>(replay.c_str()), replay.length());
                    } else {
                        _telnetClient.write(reinterpret_cast<const uint8_t*>(kBanner), sizeof(kBanner) - 1);
                    }
                }
            } else {
                WiFiClient newClient = _telnetServer->available();
                if (newClient) {
                    newClient.setNoDelay(true);
                    static const char kBusyMsg[] = "ESP32-S3-DSP: another telnet session is active. Closing.\r\n";
                    newClient.write(reinterpret_cast<const uint8_t*>(kBusyMsg), sizeof(kBusyMsg) - 1);
                    newClient.stop();
                }
            }
            if (_mutex) xSemaphoreGiveRecursive(_mutex);
        }
        if (_mutex) xSemaphoreTakeRecursive(_mutex, portMAX_DELAY);
        while (_telnetQCount > 0) {
            char lineBuf[kLineBufSize + 2];
            size_t lineLen = strlen(_telnetQ[_telnetQTail]);
            if (lineLen > kLineBufSize - 1) lineLen = kLineBufSize - 1;
            memcpy(lineBuf, _telnetQ[_telnetQTail], lineLen);
            lineBuf[lineLen] = '\r';
            lineBuf[lineLen + 1] = '\n';
            _telnetQTail = (_telnetQTail + 1) % kTelnetQueueSize;
            _telnetQCount--;
            if (_telnetClient && _telnetClient.connected()) {
                if (_mutex) xSemaphoreGiveRecursive(_mutex);
                _telnetClient.write(reinterpret_cast<const uint8_t*>(lineBuf), lineLen + 2);
                if (_mutex) xSemaphoreTakeRecursive(_mutex, portMAX_DELAY);
            }
        }
        if (_telnetClient && _telnetClient.connected()) {
            while (_telnetClient.available()) {
                _telnetClient.read();
            }
        }
        if (_mutex) xSemaphoreGiveRecursive(_mutex);
    }
}

size_t WirelessConsole::write(uint8_t c)
{
    return write(&c, 1);
}

size_t WirelessConsole::write(const uint8_t *buffer, size_t size)
{
    if (!buffer || size == 0) return 0;
    if (_mutex) xSemaphoreTakeRecursive(_mutex, portMAX_DELAY);

    // 1. Output entire block atomically to USB-CDC / UART Serial
    if (_hwSerial) {
        _hwSerial->write(buffer, size);
    }

    // 2. Buffer lines for Telnet and Web Console
    for (size_t i = 0; i < size; ++i) {
        uint8_t c = buffer[i];
        if (c == '\r') continue;

        if (c == '\n') {
            _curLine[_curPos] = '\0';

            _ensureAllocated();
            if (_lines) {
                strncpy(_lines[_head], _curLine, kLineBufSize - 1);
                _lines[_head][kLineBufSize - 1] = '\0';
                _head = (_head + 1) % kMaxLogs;
                if (_count < kMaxLogs) _count++;
            }

            if (_telnetClient && _telnetClient.connected()) {
                if (_telnetQCount == kTelnetQueueSize) {
                    _telnetQTail = (_telnetQTail + 1) % kTelnetQueueSize;
                    _telnetQCount--;
                }
                strncpy(_telnetQ[_telnetQHead], _curLine, kLineBufSize - 1);
                _telnetQ[_telnetQHead][kLineBufSize - 1] = '\0';
                _telnetQHead = (_telnetQHead + 1) % kTelnetQueueSize;
                _telnetQCount++;
            }

            if (_broadcast) {
                _broadcast(_curLine);
            }

            _curPos = 0;
            _curLine[0] = '\0';
        } else if (_curPos < kLineBufSize - 1) {
            _curLine[_curPos++] = static_cast<char>(c);
        }
    }

    if (_mutex) xSemaphoreGiveRecursive(_mutex);
    return size;
}

String WirelessConsole::getRecentLogs() const
{
    if (_mutex) xSemaphoreTakeRecursive(_mutex, portMAX_DELAY);

    if (!_lines || _count == 0) {
        if (_mutex) xSemaphoreGiveRecursive(_mutex);
        return String();
    }

    String out;
    out.reserve(_count * 80);

    size_t start = (_count < kMaxLogs) ? 0 : _head;
    for (size_t i = 0; i < _count; ++i) {
        size_t idx = (start + i) % kMaxLogs;
        out += _lines[idx];
        out += "\n";
    }

    if (_mutex) xSemaphoreGiveRecursive(_mutex);
    return out;
}

void WirelessConsole::clear()
{
    if (_mutex) xSemaphoreTakeRecursive(_mutex, portMAX_DELAY);
    _head = 0;
    _count = 0;
    _curPos = 0;
    if (_lines) {
        memset(_lines, 0, kMaxLogs * kLineBufSize);
    }
    memset(_curLine, 0, sizeof(_curLine));
    if (_mutex) xSemaphoreGiveRecursive(_mutex);
}

extern "C" void console_log(const char* buf) {
    if (buf) {
        Console.print(buf);
    }
}
