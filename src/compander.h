/**
 * @file compander.h
 * @brief Stereo RMS Feed-Forward Compander.
 *
 * Architecture:
 *  +----------+    +----------+    +----------+
 *  |  Level   |--->|  Gain    |--->|  Gain    |
 *  | Detector |    | Computer |    |  Apply   |
 *  +----------+    +----------+    +----------+
 *      RMS             dB                linear
 *
 * The RMS level is computed per-buffer (sqrt(mean(x^2))),
 * then smoothed with independent attack/release IIR envelopes.
 * Gain reduction is computed in dB using the standard compressor
 * transfer function, then converted to linear and applied sample-by-sample.
 */
#pragma once

#include <cmath>
#include "audio_config.h"
#include "param_store.h"

class Compander {
public:
    Compander() { _update({ -18.0f, 4.0f, 5.0f, 150.0f, 0.0f, true }); }

    void setParams(const CompanderParams& p) { _update(p); }

    /**
     * @brief Apply compression/expansion to stereo buffers in-place.
     *
     * Stereo-linked: the detector uses the louder channel's RMS so both
     * channels receive identical gain changes (preserving stereo image).
     */
    void process(float* left, float* right, size_t frames) {
        if (!_enabled || frames == 0) return;

        // -- RMS detection -------------------------------------
        float sumL = 0, sumR = 0;
        for (size_t i = 0; i < frames; ++i) {
            sumL += left[i]  * left[i];
            sumR += right[i] * right[i];
        }
        const float rmsL = sqrtf(sumL / frames);
        const float rmsR = sqrtf(sumR / frames);
        const float rms  = (rmsL > rmsR) ? rmsL : rmsR;   // stereo-linked

        // -- Envelope smoothing --------------------------------
        const float coeff = (rms > _envRMS) ? _attackCoeff : _releaseCoeff;
        _envRMS = _envRMS * coeff + rms * (1.0f - coeff);

        // -- Gain computer (dB) --------------------------------
        const float levelDb = 20.0f * log10f(_envRMS + 1e-9f);
        float gainDb = 0.0f;
        if (levelDb > _thresholdDb) {
            gainDb = (_thresholdDb - levelDb) * (1.0f - 1.0f / _ratio);
        }
        gainDb += _makeupGainDb;

        // -- Apply smoothed linear gain -------------------------
        const float targetGain = powf(10.0f, gainDb / 20.0f);
        const float gainStep   = (targetGain - _currentGain) / static_cast<float>(frames);
        for (size_t i = 0; i < frames; ++i) {
            _currentGain += gainStep;
            left[i]  *= _currentGain;
            right[i] *= _currentGain;
        }
    }

    /** @brief Current gain reduction in dB (negative = gain reduction). */
    float gainReductionDb() const {
        const float levelDb = 20.0f * log10f(_envRMS + 1e-9f);
        if (levelDb <= _thresholdDb) return 0.0f;
        return (_thresholdDb - levelDb) * (1.0f - 1.0f / _ratio);
    }

private:
    bool  _enabled{true};
    float _thresholdDb{-18.f};
    float _ratio{4.f};
    float _makeupGainDb{0.f};
    float _attackCoeff{0.f};
    float _releaseCoeff{0.f};
    float _envRMS{0.f};
    float _currentGain{1.0f};

    void _update(const CompanderParams& p) {
        _enabled     = p.enabled;
        _thresholdDb = p.threshold;
        _ratio       = p.ratio < 1.0f ? 1.0f : p.ratio;
        _makeupGainDb= p.makeupGain;

        // Detector updates once per buffer (block rate = Fs / kBufferFrames)
        const float F_block = static_cast<float>(kSampleRate) / static_cast<float>(kBufferFrames);
        auto tc = [&](float ms) -> float {
            float sec = (ms < 1.0f ? 1.0f : ms) / 1000.0f;
            return expf(-2.2f / (F_block * sec));
        };
        _attackCoeff  = tc(p.attackMs);
        _releaseCoeff = tc(p.releaseMs);
    }
};

