/**
 * @file dsp_pipeline.h
 * @brief Independent & Dynamic Modular DSP pipeline.
 *
 * Runs on Core 1. Modules are completely decoupled, implement IDSPModule,
 * and can be dynamically reordered at runtime. If any module is omitted,
 * disabled, or removed, the remaining modules continue to process audio seamlessly.
 */
#pragma once

#include <cmath>
#include <cstring>
#include "param_store.h"
#include "biquad_eq.h"
#include "compander.h"
#include "limiter.h"
#include "speaker_correction.h"

// -- Base interface for independent DSP processing blocks -----
class IDSPModule {
public:
    virtual ~IDSPModule() = default;
    virtual const char* id() const = 0;
    virtual const char* name() const = 0;
    virtual void process(float* left, float* right, size_t frames) = 0;
    virtual void syncParams(ParamStore& params) = 0;
    virtual void reset() {}
};

// -- 1. Parametric Equalizer Module ---------------------------
class EQModule : public IDSPModule {
public:
    const char* id() const override { return "eq"; }
    const char* name() const override { return "Parametric EQ"; }

    void syncParams(ParamStore& params) override {
        EQBandParams eqBands[kEQBands];
        if (params.getEQStateIfChanged(eqBands, _localEqCount, _localEqEnabled, _localPreampDb, _localEqVersion)) {
            _localPreampGain = powf(10.0f, _localPreampDb / 20.0f);
            for (int b = 0; b < _localEqCount; ++b) {
                _eq.setBand(b, eqBands[b]);
            }
        }
    }

    void process(float* left, float* right, size_t frames) override {
        if (!_localEqEnabled || frames == 0) return;
        if (fabsf(_localPreampGain - 1.0f) > 0.001f) {
            for (size_t i = 0; i < frames; ++i) {
                left[i]  *= _localPreampGain;
                right[i] *= _localPreampGain;
            }
        }
        _eq.process(left, right, frames, _localEqCount);
    }

    void reset() override {
        _eq.reset();
    }

private:
    BiquadEQ _eq;
    uint32_t _localEqVersion{0};
    int      _localEqCount{5};
    bool     _localEqEnabled{true};
    float    _localPreampDb{0.0f};
    float    _localPreampGain{1.0f};
};

// -- 2. Speakers Correction Module (Subsonic, Balance, Delay, Phase)
class SubsonicModule : public IDSPModule {
public:
    const char* id() const override { return "subsonic"; }
    const char* name() const override { return "Speakers Correction"; }

    void syncParams(ParamStore& params) override {
        SubsonicParams sub;
        if (params.getSubsonicIfChanged(sub, _localSubVersion)) {
            _speakers.setParams(sub);
        }
    }

    void process(float* left, float* right, size_t frames) override {
        _speakers.process(left, right, frames);
    }

    void reset() override {
        _speakers.reset();
    }

private:
    SpeakerCorrection _speakers;
    uint32_t          _localSubVersion{0};
};

// -- 3. Compander / Compressor Module -------------------------
class CompanderModule : public IDSPModule {
public:
    const char* id() const override { return "comp"; }
    const char* name() const override { return "Compressor"; }

    void syncParams(ParamStore& params) override {
        CompanderParams comp;
        if (params.getCompanderIfChanged(comp, _localCompVersion)) {
            _comp.setParams(comp);
        }
    }

    void process(float* left, float* right, size_t frames) override {
        _comp.process(left, right, frames);
    }

    void reset() override {
        // Compander reset
    }

private:
    Compander _comp;
    uint32_t  _localCompVersion{0};
};

// -- 4. Soft Limiter Module -----------------------------------
class LimiterModule : public IDSPModule {
public:
    const char* id() const override { return "lim"; }
    const char* name() const override { return "Limiter"; }

    void syncParams(ParamStore& params) override {
        LimiterParams lim;
        if (params.getLimiterIfChanged(lim, _localLimVersion)) {
            _limiter.setParams(lim);
        }
    }

    void process(float* left, float* right, size_t frames) override {
        _limiter.process(left, right, frames);
    }

    void reset() override {
        // SoftLimiter is instantaneous soft knee curve, no state history to reset
    }

private:
    SoftLimiter _limiter;
    uint32_t    _localLimVersion{0};
};

// -- Main Dynamic Pipeline Manager ----------------------------
class DSPPipeline {
public:
    static constexpr size_t kMaxModules = 8;

    explicit DSPPipeline(ParamStore& params)
        : _params(params)
    {
        // Register available modules
        _registerModule(&_eqModule);
        _registerModule(&_subsonicModule);
        _registerModule(&_compModule);
        _registerModule(&_limiterModule);

        // Initial default order
        _updateOrder("eq,subsonic,comp,lim");
    }

    /**
     * @brief Run the full dynamic DSP chain on one buffer of stereo audio.
     *
     * Called from I2SAudio::process() on Core 1.
     * Modifies left[] and right[] in-place according to active module order.
     *
     * @param left   Left-channel samples  [-1, +1]
     * @param right  Right-channel samples [-1, +1]
     * @param frames Number of frames in the buffer
     */
    void process(float* left, float* right, size_t frames) {
        _syncParams();

        for (size_t i = 0; i < _activeModuleCount; ++i) {
            if (_activeModules[i]) {
                _activeModules[i]->process(left, right, frames);
            }
        }
    }

private:
    ParamStore&       _params;

    // Independent module instances
    EQModule          _eqModule;
    SubsonicModule    _subsonicModule;
    CompanderModule   _compModule;
    LimiterModule     _limiterModule;

    // Registered modules catalog
    IDSPModule*       _registeredModules[kMaxModules]{nullptr};
    size_t            _registeredCount{0};

    // Active execution list in user-configured order
    IDSPModule*       _activeModules[kMaxModules]{nullptr};
    size_t            _activeModuleCount{0};

    uint32_t          _localOrderVersion{0};
    uint32_t          _syncCounter{0};
    static constexpr uint32_t kSyncInterval = 4; // Check every 4 buffers (~21 ms)

    void _registerModule(IDSPModule* mod) {
        if (!mod || _registeredCount >= kMaxModules) return;
        _registeredModules[_registeredCount++] = mod;
    }

    IDSPModule* _findModule(const char* id) {
        if (!id) return nullptr;
        for (size_t i = 0; i < _registeredCount; ++i) {
            if (_registeredModules[i] && (strcmp(_registeredModules[i]->id(), id) == 0 ||
                (strcmp(id, "sub") == 0 && strcmp(_registeredModules[i]->id(), "subsonic") == 0))) {
                return _registeredModules[i];
            }
        }
        return nullptr;
    }

    void _updateOrder(const char* orderStr) {
        _activeModuleCount = 0;
        if (!orderStr || strlen(orderStr) == 0) return;

        char buf[64];
        strncpy(buf, orderStr, sizeof(buf) - 1);
        buf[sizeof(buf) - 1] = '\0';

        char* token = strtok(buf, ",");
        while (token && _activeModuleCount < kMaxModules) {
            while (*token == ' ') token++;
            char* end = token + strlen(token) - 1;
            while (end > token && *end == ' ') { *end = '\0'; end--; }

            IDSPModule* m = _findModule(token);
            if (m) {
                _activeModules[_activeModuleCount++] = m;
            }
            token = strtok(nullptr, ",");
        }

        // Fallback: if no valid module matched, register all available modules in default order
        if (_activeModuleCount == 0) {
            for (size_t i = 0; i < _registeredCount && i < kMaxModules; ++i) {
                _activeModules[i] = _registeredModules[i];
            }
            _activeModuleCount = _registeredCount;
        }
    }

    void _syncParams() {
        if (++_syncCounter < kSyncInterval) return;
        _syncCounter = 0;

        // Check if execution order changed
        char orderBuf[64];
        if (_params.getModuleOrderIfChanged(orderBuf, sizeof(orderBuf), _localOrderVersion)) {
            _updateOrder(orderBuf);
            Serial.printf("[DSP] Updated signal processing order: %s\n", orderBuf);
        }

        // Sync parameters for all registered modules
        for (size_t i = 0; i < _registeredCount; ++i) {
            if (_registeredModules[i]) {
                _registeredModules[i]->syncParams(_params);
            }
        }
    }
};
