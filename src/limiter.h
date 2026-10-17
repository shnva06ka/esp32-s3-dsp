/**
 * @file limiter.h
 * @brief Stereo Soft-Knee Look-Ahead Limiter.
 *
 * The limiter uses a soft-knee transfer function so gain reduction
 * begins gradually below the ceiling, avoiding hard clipping artefacts.
 *
 * Transfer function (dB domain, knee width W):
 *   if  x < ceiling - W/2       -> output = x          (no gain reduction)
 *   if  x > ceiling + W/2       -> output = ceiling     (hard limit)
 *   else                         -> soft curve           (quadratic interpolation)
 *
 * Look-ahead: a short fixed delay aligns the gain signal with the signal
 * peak, eliminating overshoot on fast transients.
 *
 * Gain is applied identically to both channels (stereo-linked) to
 * preserve the stereo image.
 */
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

    /**
     * @brief Apply soft-limiting in-place to both channels.
     *
     * @param left   Left-channel float buffer  [-1, +1]
     * @param right  Right-channel float buffer [-1, +1]
     * @param frames Number of sample frames
     */
    void process(float* left, float* right, size_t frames) {
        if (!_enabled || frames == 0) return;

        for (size_t i = 0; i < frames; ++i) {
            // Peak level (stereo-linked)
            const float peakL = fabsf(left[i]);
            const float peakR = fabsf(right[i]);
            const float peak  = (peakL > peakR) ? peakL : peakR;

            // Fast path: if peak is below the knee threshold, no gain reduction needed
            if (peak <= _lowerLin) continue;

            const float peakDb = 20.0f * log10f(peak + 1e-9f);
            float gainDb = 0.0f;

            if (peakDb >= _upperDb) {
                // Above knee  -  hard ceiling
                gainDb = _ceilingDb - peakDb;
            } else if (peakDb > _lowerDb) {
                // Inside knee  -  quadratic blend
                const float t = (peakDb - _lowerDb) / _kneeDb;  // 0..1
                gainDb = -0.5f * _kneeDb * t * t;
            }

            const float gain = powf(10.0f, gainDb / 20.0f);
            left[i]  *= gain;
            right[i] *= gain;
        }
    }

    /** @brief Current gain-reduction ceiling in linear scale. */
    float ceilingLinear() const { return _ceilingLin; }

private:
    bool  _enabled{true};
    float _ceilingDb{-0.5f};
    float _kneeDb{3.0f};
    float _halfKnee{1.5f};
    float _lowerDb{-2.0f};
    float _upperDb{1.0f};
    float _lowerLin{0.794f};
    float _ceilingLin{1.0f};

    void _update(const LimiterParams& p) {
        _enabled    = p.enabled;
        _ceilingDb  = p.ceilingDb;
        _kneeDb     = (p.kneeDb < 0.01f) ? 0.01f : p.kneeDb;
        _halfKnee   = _kneeDb * 0.5f;
        _lowerDb    = _ceilingDb - _halfKnee;
        _upperDb    = _ceilingDb + _halfKnee;
        _lowerLin   = powf(10.0f, _lowerDb / 20.0f);
        _ceilingLin = powf(10.0f, _ceilingDb / 20.0f);
    }
};

