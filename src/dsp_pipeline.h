#pragma once

#include <cmath>
#include <cstring>
#if __has_include(<esp_dsp.h>)
#include <esp_dsp.h>
#define HAVE_ESP_DSP 1
#endif
#include "param_store.h"
#include "biquad_eq.h"
#include "compander.h"
#include "limiter.h"
#include "speaker_correction.h"
#include "dsp_module_id.h"

class IDSPModule {
public:
    virtual ~IDSPModule() = default;
    virtual DspModuleId moduleId() const = 0;
    virtual const char* id() const = 0;
    virtual const char* name() const = 0;
    virtual void process(float* left, float* right, size_t frames) = 0;
    virtual void syncParams(ParamStore& params) = 0;
    virtual void reset() {}
};

class EQModule : public IDSPModule {
public:
    DspModuleId moduleId() const override { return DspModuleId::Eq; }
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
#if defined(HAVE_ESP_DSP)
            dsps_mulc_f32(left,  left,  static_cast<int>(frames), _localPreampGain, 1, 1);
            dsps_mulc_f32(right, right, static_cast<int>(frames), _localPreampGain, 1, 1);
#else
            for (size_t i = 0; i < frames; ++i) {
                left[i]  *= _localPreampGain;
                right[i] *= _localPreampGain;
            }
#endif
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

class SubsonicModule : public IDSPModule {
public:
    DspModuleId moduleId() const override { return DspModuleId::Subsonic; }
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

class CompanderModule : public IDSPModule {
public:
    DspModuleId moduleId() const override { return DspModuleId::Compander; }
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

    void reset() override {}

private:
    Compander _comp;
    uint32_t  _localCompVersion{0};
};

class LimiterModule : public IDSPModule {
public:
    DspModuleId moduleId() const override { return DspModuleId::Limiter; }
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

    void reset() override {}

private:
    SoftLimiter _limiter;
    uint32_t    _localLimVersion{0};
};

class DSPPipeline {
public:
    static constexpr size_t kMaxModules = 8;

    explicit DSPPipeline(ParamStore& params)
        : _params(params)
    {
        _registerModule(&_eqModule);
        _registerModule(&_subsonicModule);
        _registerModule(&_compModule);
        _registerModule(&_limiterModule);

        _updateOrder("eq,subsonic,comp,lim");
    }

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

    EQModule          _eqModule;
    SubsonicModule    _subsonicModule;
    CompanderModule   _compModule;
    LimiterModule     _limiterModule;

    IDSPModule*       _registeredModules[kMaxModules]{nullptr};
    size_t            _registeredCount{0};

    IDSPModule*       _activeModules[kMaxModules]{nullptr};
    size_t            _activeModuleCount{0};

    uint32_t          _localOrderVersion{0};
    uint32_t          _syncCounter{0};
    static constexpr uint32_t kSyncInterval = 4;

    void _registerModule(IDSPModule* mod) {
        if (!mod || _registeredCount >= kMaxModules) return;
        _registeredModules[_registeredCount++] = mod;
    }

    IDSPModule* _findModule(DspModuleId id) {
        if (id == DspModuleId::Unknown) return nullptr;
        for (size_t i = 0; i < _registeredCount; ++i) {
            if (_registeredModules[i] && _registeredModules[i]->moduleId() == id) {
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

            IDSPModule* m = _findModule(parseDspModuleId(token));
            if (m) {
                _activeModules[_activeModuleCount++] = m;
            }
            token = strtok(nullptr, ",");
        }

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

        char orderBuf[64];
        if (_params.getModuleOrderIfChanged(orderBuf, sizeof(orderBuf), _localOrderVersion)) {
            _updateOrder(orderBuf);
            Serial.printf("[DSP] Updated signal processing order: %s\n", orderBuf);
        }

        for (size_t i = 0; i < _registeredCount; ++i) {
            if (_registeredModules[i]) {
                _registeredModules[i]->syncParams(_params);
            }
        }
    }
};
