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

    void setParams(const LimiterParams& p) { _update(p); }

    void process(float* left, float* right, size_t frames) {
        if (!_enabled || frames == 0) return;

        for (size_t i = 0; i < frames; ++i) {
            const float peakL = fabsf(left[i]);
            const float peakR = fabsf(right[i]);
            const float peak  = (peakL > peakR) ? peakL : peakR;

            if (peak <= _lowerLin) continue;

            float gain;
            if (peak >= _upperLin) {
                gain = _ceilingLin / peak;
            } else {
                const float peakDb = 20.0f * log10f(peak);
                const float t      = (peakDb - _lowerDb) * _invKneeDb;
                const float gainDb = -0.5f * _kneeDb * t * t;
                gain = powf(10.0f, gainDb * 0.05f);
            }

            left[i]  *= gain;
            right[i] *= gain;
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
        _ceilingDb  = p.ceilingDb;
        _kneeDb     = (p.kneeDb < 0.01f) ? 0.01f : p.kneeDb;
        _invKneeDb  = 1.0f / _kneeDb;
        _halfKnee   = _kneeDb * 0.5f;
        _lowerDb    = _ceilingDb - _halfKnee;
        _upperDb    = _ceilingDb + _halfKnee;
        _lowerLin   = powf(10.0f, _lowerDb / 20.0f);
        _upperLin   = powf(10.0f, _upperDb / 20.0f);
        _ceilingLin = powf(10.0f, _ceilingDb / 20.0f);
    }
};
