#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <atomic>
#include "param_store.h"

class SlimprotoClient {
public:
    explicit SlimprotoClient(ParamStore& params);
    ~SlimprotoClient();

    bool begin();
    void stop();

private:
    ParamStore&       _params;
    TaskHandle_t      _telemetryTaskHandle{nullptr};
    std::atomic<bool> _running{false};
    std::atomic<bool> _telemetryExited{true};

    static void _telemetryTaskTrampoline(void* arg);
    void        _runTelemetry();
};
