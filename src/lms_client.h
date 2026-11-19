#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <atomic>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "param_store.h"
#include "lms_codecs.h"

struct LmsStatus {
    bool     connected;
    char     serverIp[16];
    uint8_t  phase;
    uint32_t trackFrames;
    uint32_t outFrames;
    uint32_t streamBytes;
    uint32_t sourceRate;
    uint32_t sourceBits;
};

size_t       lms_output_frames(float* left, float* right, size_t frames);
bool         lms_get_status(LmsStatus* out);
bool         lms_get_resampler_stats(resampler_stats_t* out);
TaskHandle_t lms_decode_task_handle(void);
void         lms_pause_for_ota(void);

class LmsClient {
public:
    explicit LmsClient(ParamStore& params);
    ~LmsClient();

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
