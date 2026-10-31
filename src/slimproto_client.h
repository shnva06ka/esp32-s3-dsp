#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include "param_store.h"

class SlimprotoClient {
public:
    explicit SlimprotoClient(ParamStore& params);
    ~SlimprotoClient();

    bool begin();
    void stop();

private:
    ParamStore&  _params;
    TaskHandle_t _telemetryTaskHandle{nullptr};
    bool         _running{false};

    static void _telemetryTaskTrampoline(void* arg);
    void        _runTelemetry();
};
