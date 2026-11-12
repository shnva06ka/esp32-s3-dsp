#pragma once

#include <cmath>
#include <algorithm>
#include "audio_config.h"
#include "param_store.h"

class SoftLimiter {
public:
    SoftLimiter() {
        _update({ -0.5f, 3.0f, true });
    }

    void reset() {}

    void setParams(const LimiterParams& p) { _update(p); }

    void process(float* left, float* right, size_t frames) {
        if (!_enabled || frames == 0) return;

        for (size_t i = 0; i < frames; ++i) {
            float sL = left[i];
            float sR = right[i];
            if (!std::isfinite(sL)) { sL = 0.0f; left[i] = 0.0f; }
            if (!std::isfinite(sR)) { sR = 0.0f; right[i] = 0.0f; }

            const float peakL = fabsf(sL);
            const float peakR = fabsf(sR);
            const float peak  = (peakL > peakR) ? peakL : peakR;

            if (peak <= _lowerLin || peak <= 1e-9f) continue;

            float sGain = 1.0f;
            if (peak >= _upperLin) {
                sGain = _ceilingLin / peak;
            } else {
                const float peakDb = 20.0f * log10f(peak);
                const float t      = (peakDb - _lowerDb) * _invKneeDb;
                const float gainDb = -0.5f * _kneeDb * t * t;
                sGain = powf(10.0f, gainDb * 0.05f);
            }

            if (!std::isfinite(sGain) || sGain < 0.0f) sGain = 1.0f;
            if (sGain > 1.0f) sGain = 1.0f;

            left[i]  *= sGain;
            right[i] *= sGain;
        }
    }

private:
    bool  _enabled{true};
    float _ceilingDb{-0.5f};
    float _kneeDb{3.0f};
    float _invKneeDb{1.0f / 3.0f};
    float _halfKnee{1.5f};
    float _lowerDb{-2.0f};
    float _upperDb{1.0f};
    float _lowerLin{0.794f};
    float _upperLin{1.122f};
    float _ceilingLin{1.0f};

    void _update(const LimiterParams& p) {
        _enabled    = p.enabled;
        const float rawCeiling = std::isfinite(p.ceilingDb) ? p.ceilingDb : -0.5f;
        const float rawKnee    = std::isfinite(p.kneeDb)    ? p.kneeDb    : 3.0f;
        _ceilingDb  = (rawCeiling < -30.0f) ? -30.0f : ((rawCeiling > 0.0f) ? 0.0f : rawCeiling);
        _kneeDb     = (rawKnee < 0.1f) ? 0.1f : ((rawKnee > 12.0f) ? 12.0f : rawKnee);
        _invKneeDb  = 1.0f / _kneeDb;
        _halfKnee   = _kneeDb * 0.5f;
        _lowerDb    = _ceilingDb - _halfKnee;
        _upperDb    = _ceilingDb + _halfKnee;
        _lowerLin   = powf(10.0f, _lowerDb / 20.0f);
        _upperLin   = powf(10.0f, _upperDb / 20.0f);
        _ceilingLin = powf(10.0f, _ceilingDb / 20.0f);
    }
};
