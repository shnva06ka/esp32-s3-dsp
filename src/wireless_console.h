#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiServer.h>
#include <WiFiClient.h>

class WirelessConsole : public Print {
public:
    explicit WirelessConsole(Print* hwSerial = nullptr);
    void begin(unsigned long baud = 0);
    void loop();

    virtual size_t write(uint8_t c) override;
    virtual size_t write(const uint8_t *buffer, size_t size) override;

    using BroadcastFunc = void (*)(const char* msg);
    void setBroadcastFunc(BroadcastFunc fn) { _broadcast = fn; }

    String getRecentLogs() const;
    void clear();

private:
    Print* _hwSerial{nullptr};
    static constexpr size_t kMaxLogs = 150;
    static constexpr size_t kLineBufSize = 256;

    char   _lines[kMaxLogs][kLineBufSize];
    size_t _head{0};
    size_t _count{0};

    char   _curLine[kLineBufSize];
    size_t _curPos{0};

    WiFiServer*   _telnetServer{nullptr};
    WiFiClient    _telnetClient;
    BroadcastFunc _broadcast{nullptr};
};

extern WirelessConsole Console;

#ifdef __cplusplus
extern "C" {
#endif
void console_log(const char* buf);
#ifdef __cplusplus
}
#endif
