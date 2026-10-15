/**
 * @file dsp_pipeline.h
 * @brief Modular DSP pipeline: EQ → Compressor → Limiter
 *
 * Runs on Core 1.  Reads parameters from the shared ParamStore on
 * every buffer boundary (cheap mutex + struct copy), so updates from
 * the web UI are picked up with at most one buffer's latency
 * (~5.3 ms at 48 kHz / 256 frames).
 */
#pragma once

#include "param_store.h"
#include "biquad_eq.h"
#include "compander.h"
#include "limiter.h"

class DSPPipeline {
public:
    explicit DSPPipeline(ParamStore& params) : _params(params) {}

    /**
     * @brief Run the full DSP chain on one buffer of stereo audio.
     *
     * Called from I2SAudio::process() on Core 1.
     * Modifies left[] and right[] in-place.
     *
     * @param left   Left-channel samples  [-1, +1]
     * @param right  Right-channel samples [-1, +1]
     * @param frames Number of frames in the buffer
     */
    void process(float* left, float* right, size_t frames) {
        _syncParams();

        // ── Stage 1: Parametric EQ Module ───────────────────
        if (_localEqEnabled) {
            if (fabsf(_localPreampGain - 1.0f) > 0.001f) {
                for (size_t i = 0; i < frames; ++i) {
                    left[i]  *= _localPreampGain;
                    right[i] *= _localPreampGain;
                }
            }
            _eq.process(left, right, frames, _localEqCount);
        }

        // ── Stage 2: Compressor Module ──────────────────────
        _comp.process(left, right, frames);

        // ── Stage 3: Soft Limiter Module ──────────────────────
        _limiter.process(left, right, frames);
    }

private:
    ParamStore& _params;
    BiquadEQ    _eq;
    Compander   _comp;
    SoftLimiter _limiter;

    uint32_t _syncCounter{0};
    uint32_t _localEqVersion{0};
    uint32_t _localCompVersion{0};
    uint32_t _localLimVersion{0};
    int      _localEqCount{5};
    bool     _localEqEnabled{true};
    float    _localPreampDb{0.0f};
    float    _localPreampGain{1.0f};
    static constexpr uint32_t kSyncInterval = 4;   // check every 4 buffers (~21 ms)

    /** @brief Pull fresh parameters only when changed. */
    void _syncParams() {
        if (++_syncCounter < kSyncInterval) return;
        _syncCounter = 0;

        // EQ bands (single mutex lock, only if version changed)
        EQBandParams eqBands[kEQBands];
        if (_params.getEQStateIfChanged(eqBands, _localEqCount, _localEqEnabled, _localPreampDb, _localEqVersion)) {
            _localPreampGain = powf(10.0f, _localPreampDb / 20.0f);
            for (int b = 0; b < _localEqCount; ++b) {
                _eq.setBand(b, eqBands[b]);
            }
        }

        // Compander
        CompanderParams comp;
        if (_params.getCompanderIfChanged(comp, _localCompVersion)) {
            _comp.setParams(comp);
        }

        // Limiter
        LimiterParams lim;
        if (_params.getLimiterIfChanged(lim, _localLimVersion)) {
            _limiter.setParams(lim);
        }
    }
};

