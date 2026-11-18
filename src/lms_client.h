#pragma once

// LMS (Lyrion Music Server / SlimProto) streaming client for the ESP32-S3 DSP.
// Pipeline: TCP control + HTTP stream -> byte ring -> FLAC/PCM decode -> resample to 96 kHz
//           -> float ring -> lms_output_frames() (called from the I2S audio task).

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
    uint8_t  phase;          // lms::Phase
    uint32_t trackFrames;    // frames played of the current track (at the output rate)
    uint32_t outFrames;      // decoded frames waiting in the output ring
    uint32_t streamBytes;    // bytes waiting in the stream ring
    uint32_t sourceRate;     // sample rate of the stream being decoded (0 = none)
    uint32_t sourceBits;
};

// Called from the audio task. Returns frames written (the rest of the block is left for the caller to zero).
size_t       lms_output_frames(float* left, float* right, size_t frames);
bool         lms_get_status(LmsStatus* out);
bool         lms_get_resampler_stats(resampler_stats_t* out);
TaskHandle_t lms_decode_task_handle(void);
// Drops the server connection and stops playback so an OTA update has the heap and network to itself.
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
