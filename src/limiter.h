#pragma once

#include <cmath>
#include <cstdint>
#include <cstring>
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

        const uint32_t lowerLinBits = _lowerLinBits;

        for (size_t i = 0; i < frames; ++i) {
            float sL = left[i];
            float sR = right[i];

            // Fast-path threshold check
            uint32_t uL = 0, uR = 0;
            std::memcpy(&uL, &sL, sizeof(uL));
            std::memcpy(&uR, &sR, sizeof(uR));
            uL &= 0x7FFFFFFFu;
            uR &= 0x7FFFFFFFu;
            const uint32_t uMax = (uL > uR) ? uL : uR;
            if (uMax <= lowerLinBits) continue;

            if (!std::isfinite(sL)) { sL = 0.0f; left[i]  = 0.0f; }
            if (!std::isfinite(sR)) { sR = 0.0f; right[i] = 0.0f; }

            const float peakL = fabsf(sL);
            const float peakR = fabsf(sR);
            const float peak  = (peakL > peakR) ? peakL : peakR;

            if (peak <= _lowerLin || peak <= 1e-9f) continue;

            float sGain = 1.0f;
            if (peak >= _upperLin) {
                sGain = _ceilingLin / peak;
            } else {
                const float t = (_fastLog2f(peak) - _lowerLog2) * _invKneeLog2;
                sGain = _fastExp2f(_negHalfKneeLog2 * t * t);
            }

            if (!std::isfinite(sGain) || sGain < 0.0f) sGain = 0.0f;
            if (sGain > 1.0f) sGain = 1.0f;

            left[i]  = sL * sGain;
            right[i] = sR * sGain;
        }
    }

private:
    bool     _enabled{true};
    float    _ceilingDb{-0.5f};
    float    _kneeDb{3.0f};
    float    _invKneeDb{1.0f / 3.0f};
    float    _halfKnee{1.5f};
    float    _lowerDb{-2.0f};
    float    _upperDb{1.0f};
    float    _lowerLin{0.794f};
    float    _upperLin{1.122f};
    float    _ceilingLin{1.0f};
    uint32_t _lowerLinBits{0x3F4B4B3Bu};
    float    _lowerLog2{0.0f};
    float    _invKneeLog2{0.0f};
    float    _negHalfKneeLog2{0.0f};

    // Fast log2f approximation
    static inline float _fastLog2f(float x) {
        uint32_t ix = 0;
        std::memcpy(&ix, &x, sizeof(ix));
        const int32_t exp = static_cast<int32_t>((ix >> 23) & 0xFFu) - 127;
        const uint32_t mantBits = (ix & 0x007FFFFFu) | 0x3F800000u;
        float m = 1.0f;
        std::memcpy(&m, &mantBits, sizeof(m));

        float p = -0.0816158131f;
        p = p * m +  0.6451423764f;
        p = p * m -  2.1206750870f;
        p = p * m +  4.0700907707f;
        p = p * m -  2.5128545761f;
        return static_cast<float>(exp) + p;
    }

    // Fast exp2f approximation
    static inline float _fastExp2f(float x) {
        const float xc = (x >= -126.0f) ? ((x <= 127.0f) ? x : 127.0f) : -126.0f;
        int32_t k = static_cast<int32_t>(xc);
        k -= static_cast<int32_t>(static_cast<float>(k) > xc);
        const float f = xc - static_cast<float>(k);

        float p = 0.0136976643f;
        p = p * f + 0.0516903587f;
        p = p * f + 0.2416384518f;
        p = p * f + 0.6929661036f;
        p = p * f + 1.0000036955f;

        const uint32_t scaleBits = (x >= -126.0f) ? (static_cast<uint32_t>(k + 127) << 23) : 0u;
        float scale = 0.0f;
        std::memcpy(&scale, &scaleBits, sizeof(scale));
        return scale * p;
    }

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
        std::memcpy(&_lowerLinBits, &_lowerLin, sizeof(_lowerLinBits));
        _lowerLog2       = _lowerDb * 0.16609640474436812f;
        _invKneeLog2     = 6.020599913279624f * _invKneeDb;
        _negHalfKneeLog2 = -0.5f * _kneeDb * 0.16609640474436812f;
    }
};
