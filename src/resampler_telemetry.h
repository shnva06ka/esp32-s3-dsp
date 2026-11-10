#pragma once

#include <Arduino.h>
#include <esp_system.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "sl_resample.h"
#include "i2s_audio.h"

extern I2SAudio g_audio;
extern "C" bool sl_flac_get_resampler_stats(resampler_stats_t *out);
extern "C" TaskHandle_t sl_decode_task_handle(void);

class ResamplerTelemetry {
public:
    struct SystemDiag {
        const char* resetReason;
        float       temperatureC;
        uint32_t    btStackHighWaterBytes;
        uint32_t    audioStackHighWaterBytes;
        uint32_t    webStackHighWaterBytes;
        uint32_t    decodeStackHighWaterBytes;
    };

    static void setTaskHandles(TaskHandle_t bt, TaskHandle_t audio, TaskHandle_t web, TaskHandle_t decode) {
        _s().btTask     = bt;
        _s().audioTask  = audio;
        _s().webTask    = web;
        _s().decodeTask = decode;
    }

    static void setBtTaskHandle(TaskHandle_t h)     { _s().btTask = h; }
    static void setAudioTaskHandle(TaskHandle_t h)  { _s().audioTask = h; }
    static void setWebTaskHandle(TaskHandle_t h)    { _s().webTask = h; }
    static void setDecodeTaskHandle(TaskHandle_t h) { _s().decodeTask = h; }

    static void setLoggingEnabled(bool en) {
        _s().loggingEnabled = en;
    }

    static bool isLoggingEnabled() {
        return _s().loggingEnabled;
    }

    static bool getBtStats(resampler_stats_t* out) {
        return g_audio.getBtResamplerStats(out);
    }

    static bool getFlacStats(resampler_stats_t* out) {
        return sl_flac_get_resampler_stats(out);
    }

    static const char* getResetReasonString() {
        switch (esp_reset_reason()) {
            case ESP_RST_POWERON:   return "Power-on";
            case ESP_RST_EXT:       return "External pin";
            case ESP_RST_SW:        return "Software reboot";
            case ESP_RST_PANIC:     return "Software panic / exception";
            case ESP_RST_INT_WDT:   return "Interrupt watchdog";
            case ESP_RST_TASK_WDT:  return "Task watchdog";
            case ESP_RST_WDT:       return "Watchdog reset";
            case ESP_RST_DEEPSLEEP: return "Deep sleep wake";
            case ESP_RST_BROWNOUT:  return "Brownout reset";
            case ESP_RST_SDIO:      return "SDIO reset";
            default:                return "Unknown";
        }
    }

    static SystemDiag getSystemDiag() {
        SystemDiag d{};
        d.resetReason = getResetReasonString();
        d.temperatureC = temperatureRead();

        if (_s().btTask && eTaskGetState(_s().btTask) != eDeleted) {
            d.btStackHighWaterBytes = (uint32_t)uxTaskGetStackHighWaterMark(_s().btTask) * sizeof(StackType_t);
        }
        if (_s().audioTask && eTaskGetState(_s().audioTask) != eDeleted) {
            d.audioStackHighWaterBytes = (uint32_t)uxTaskGetStackHighWaterMark(_s().audioTask) * sizeof(StackType_t);
        }
        if (_s().webTask && eTaskGetState(_s().webTask) != eDeleted) {
            d.webStackHighWaterBytes = (uint32_t)uxTaskGetStackHighWaterMark(_s().webTask) * sizeof(StackType_t);
        }
        TaskHandle_t dec = _s().decodeTask ? _s().decodeTask : sl_decode_task_handle();
        if (dec && eTaskGetState(dec) != eDeleted) {
            d.decodeStackHighWaterBytes = (uint32_t)uxTaskGetStackHighWaterMark(dec) * sizeof(StackType_t);
        }
        return d;
    }

    /*
     * Rate-limited periodic report (every 5000ms).
     * Only emits logs when resampling is active so normal playback is not disturbed.
     */
    static void periodicReport(const resampler_stats_t* btStats, const resampler_stats_t* flacStats) {
        if (!_s().loggingEnabled) return;

        unsigned long now = millis();
        if (now - _s().lastReportMs < 5000) return;
        _s().lastReportMs = now;

        SystemDiag diag = getSystemDiag();

        bool hasBt = (btStats && !btStats->bypass && btStats->total_calls > 0);
        bool hasFlac = (flacStats && !flacStats->bypass && flacStats->total_calls > 0);

        if (hasBt) {
            Serial.printf("[RESAMPLE-BT] %u->%u Hz (%u/%u, P=%u, %s) | "
                          "proc: %u us (avg %u, max %u, worst %u) | CPU: %.1f%% | "
                          "Ring: %u/%u (%u%%) | U/O/D: %u/%u/%u | Stack: Bt=%u Aud=%u | Temp: %.1fC | Reset: %s\n",
                          btStats->in_rate, btStats->out_rate,
                          btStats->ratio_L, btStats->ratio_M, btStats->phases,
                          btStats->is_psram ? "PSRAM" : "DRAM",
                          btStats->last_duration_us, btStats->avg_duration_us,
                          btStats->max_duration_us, btStats->recent_worst_us,
                          btStats->cpu_util_pct,
                          btStats->buf_occupancy_frames, btStats->buf_capacity_frames,
                          btStats->buf_capacity_frames ? (btStats->buf_occupancy_frames * 100 / btStats->buf_capacity_frames) : 0,
                          btStats->underruns, btStats->overruns, btStats->dropped_frames,
                          diag.btStackHighWaterBytes, diag.audioStackHighWaterBytes,
                          diag.temperatureC, diag.resetReason);
        }

        if (hasFlac) {
            Serial.printf("[RESAMPLE-FLAC] %u->%u Hz (%u/%u, P=%u, %s) | "
                          "proc: %u us (avg %u, max %u, worst %u) | CPU: %.1f%% | "
                          "OutBuf: %u/%u | U/O/D: %u/%u/%u | Stack: Dec=%u Aud=%u\n",
                          flacStats->in_rate, flacStats->out_rate,
                          flacStats->ratio_L, flacStats->ratio_M, flacStats->phases,
                          flacStats->is_psram ? "PSRAM" : "DRAM",
                          flacStats->last_duration_us, flacStats->avg_duration_us,
                          flacStats->max_duration_us, flacStats->recent_worst_us,
                          flacStats->cpu_util_pct,
                          flacStats->buf_occupancy_frames, flacStats->buf_capacity_frames,
                          flacStats->underruns, flacStats->overruns, flacStats->dropped_frames,
                          diag.decodeStackHighWaterBytes, diag.audioStackHighWaterBytes);
        }
    }

    static void poll() {
        resampler_stats_t btStats{}, flacStats{};
        bool hasBt = getBtStats(&btStats);
        bool hasFlac = getFlacStats(&flacStats);
        periodicReport(hasBt ? &btStats : nullptr, hasFlac ? &flacStats : nullptr);
    }

private:
    struct State {
        TaskHandle_t  btTask{nullptr};
        TaskHandle_t  audioTask{nullptr};
        TaskHandle_t  webTask{nullptr};
        TaskHandle_t  decodeTask{nullptr};
        bool          loggingEnabled{true};
        unsigned long lastReportMs{0};
    };

    static State& _s() {
        static State s;
        return s;
    }
};
