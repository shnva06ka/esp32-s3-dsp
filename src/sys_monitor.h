#pragma once

#include <Arduino.h>
#include <esp_freertos_hooks.h>

class SystemMonitor {
private:
    struct State {
        volatile uint32_t idleCount0{0};
        volatile uint32_t idleCount1{0};
        uint32_t          maxIdleRate0{0};
        uint32_t          maxIdleRate1{0};
        unsigned long     lastSampleTime{0};
        float             cpu0Usage{0.0f};
        float             cpu1Usage{0.0f};
    };

    static State& _s() {
        static State s;
        return s;
    }

    static bool _idleHook0() {
        _s().idleCount0++;
        return false;
    }

    static bool _idleHook1() {
        _s().idleCount1++;
        return false;
    }

    static float _clamp(float v, float lo, float hi) {
        return v < lo ? lo : (v > hi ? hi : v);
    }

public:
    struct Stats {
        float    cpu0;
        float    cpu1;
        uint32_t heapUsed;
        uint32_t heapTotal;
        uint32_t psramUsed;
        uint32_t psramTotal;
        uint32_t flashUsed;
        uint32_t flashTotal;
    };

    static void begin() {
        esp_register_freertos_idle_hook_for_cpu(_idleHook0, 0);
        esp_register_freertos_idle_hook_for_cpu(_idleHook1, 1);
        _s().lastSampleTime = millis();
    }

    static Stats getStats() {
        update();
        Stats s;
        s.cpu0 = _s().cpu0Usage;
        s.cpu1 = _s().cpu1Usage;
        s.heapTotal  = ESP.getHeapSize();
        s.heapUsed   = s.heapTotal - ESP.getFreeHeap();
        s.psramTotal = psramFound() ? ESP.getPsramSize() : 0;
        s.psramUsed  = psramFound() ? (s.psramTotal - ESP.getFreePsram()) : 0;
        s.flashUsed  = ESP.getSketchSize();
        s.flashTotal = ESP.getFlashChipSize();
        return s;
    }

    static void update() {
        unsigned long now = millis();
        unsigned long elapsed = now - _s().lastSampleTime;
        if (elapsed < 500) return;

        uint32_t count0 = _s().idleCount0;
        uint32_t count1 = _s().idleCount1;
        _s().idleCount0 = 0;
        _s().idleCount1 = 0;
        _s().lastSampleTime = now;

        uint32_t rate0 = (uint32_t)(((uint64_t)count0 * 1000) / elapsed);
        uint32_t rate1 = (uint32_t)(((uint64_t)count1 * 1000) / elapsed);

        if (rate0 > _s().maxIdleRate0) _s().maxIdleRate0 = rate0;
        if (rate1 > _s().maxIdleRate1) _s().maxIdleRate1 = rate1;

        if (_s().maxIdleRate0 > 1000) {
            float idlePct0 = ((float)rate0 / (float)_s().maxIdleRate0) * 100.0f;
            float raw0 = _clamp(100.0f - idlePct0, 0.0f, 100.0f);
            _s().cpu0Usage = (_s().cpu0Usage * 0.4f) + (raw0 * 0.6f);
        }

        if (_s().maxIdleRate1 > 1000) {
            float idlePct1 = ((float)rate1 / (float)_s().maxIdleRate1) * 100.0f;
            float raw1 = _clamp(100.0f - idlePct1, 0.0f, 100.0f);
            _s().cpu1Usage = (_s().cpu1Usage * 0.4f) + (raw1 * 0.6f);
        }
    }
};

