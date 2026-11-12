#pragma once

#include <cmath>
#include <cstring>
#include "audio_config.h"
#include "param_store.h"

class SubsonicFilter {
public:
    SubsonicFilter() {
        reset();
        _update({ 80.0f, 2, false });
    }

    void reset() {
        memset(_zL, 0, sizeof(_zL));
        memset(_zR, 0, sizeof(_zR));
    }

    void setParams(const SubsonicParams& p) {
        _update(p);
    }

    void process(float* left, float* right, size_t frames) {
        if (!_enabled || frames == 0) return;

        for (size_t i = 0; i < frames; ++i) {
            if (!std::isfinite(left[i]))  left[i]  = 0.0f;
            if (!std::isfinite(right[i])) right[i] = 0.0f;
        }

        _processStage(left, right, frames, _coeffs[0], _zL[0], _zR[0]);
        if (_hasSecondStage) {
            _processStage(left, right, frames, _coeffs[1], _zL[1], _zR[1]);
        }
    }

private:
    // Hardware-FPU 32-bit float Transposed Direct Form II keeps internal state
    // bounded to O(1) (avoiding DF-II's ~583,000 state blowup at 20 Hz / 96 kHz).
    static inline void _processStage(float* left, float* right, size_t frames,
                                     const float* c, float* zL, float* zR) {
        const float b0 = c[0], b1 = c[1], b2 = c[2], a1 = c[3], a2 = c[4];
        for (size_t i = 0; i < frames; ++i) {
            float inL = std::isfinite(left[i]) ? left[i] : 0.0f;
            float outL = b0 * inL + zL[0];
            zL[0] = b1 * inL - a1 * outL + zL[1];
            zL[1] = b2 * inL - a2 * outL;
            if (!std::isfinite(outL)) { outL = 0.0f; zL[0] = 0.0f; zL[1] = 0.0f; }
            if (fabsf(zL[0]) < 1e-15f) zL[0] = 0.0f;
            if (fabsf(zL[1]) < 1e-15f) zL[1] = 0.0f;
            left[i] = outL;

            float inR = std::isfinite(right[i]) ? right[i] : 0.0f;
            float outR = b0 * inR + zR[0];
            zR[0] = b1 * inR - a1 * outR + zR[1];
            zR[1] = b2 * inR - a2 * outR;
            if (!std::isfinite(outR)) { outR = 0.0f; zR[0] = 0.0f; zR[1] = 0.0f; }
            if (fabsf(zR[0]) < 1e-15f) zR[0] = 0.0f;
            if (fabsf(zR[1]) < 1e-15f) zR[1] = 0.0f;
            right[i] = outR;
        }
    }

    bool  _enabled{false};
    float _freq{80.0f};
    uint8_t _order{2};

    float _coeffs[2][5]{};
    float _zL[2][2]{};
    float _zR[2][2]{};
    bool  _hasSecondStage{false};

    void _update(const SubsonicParams& p) {
        _enabled = p.enabled;
        const double rawFreq = std::isfinite(p.freq) ? static_cast<double>(p.freq) : 80.0;
        const double safeFreq = (rawFreq < 20.0) ? 20.0 : ((rawFreq > 100.0) ? 100.0 : rawFreq);
        _freq    = static_cast<float>(safeFreq);
        _order   = (p.order >= 1 && p.order <= 4) ? p.order : 2;

        const double Fs   = static_cast<double>(kSampleRate);
        const double w0   = 2.0 * M_PI * safeFreq / Fs;
        const double cos0 = cos(w0);
        const double sin0 = sin(w0);
        const double tan0 = tan(w0 * 0.5);

        _hasSecondStage = false;

        if (_order == 1) {
            const double b0 = 1.0 / (1.0 + tan0);
            const double a1 = (tan0 - 1.0) / (1.0 + tan0);
            _coeffs[0][0] = static_cast<float>(b0);
            _coeffs[0][1] = static_cast<float>(-b0);
            _coeffs[0][2] = 0.0f;
            _coeffs[0][3] = static_cast<float>(a1);
            _coeffs[0][4] = 0.0f;
        } else if (_order == 3) {
            const double b0 = 1.0 / (1.0 + tan0);
            const double a1 = (tan0 - 1.0) / (1.0 + tan0);
            _coeffs[0][0] = static_cast<float>(b0);
            _coeffs[0][1] = static_cast<float>(-b0);
            _coeffs[0][2] = 0.0f;
            _coeffs[0][3] = static_cast<float>(a1);
            _coeffs[0][4] = 0.0f;

            const double alpha = sin0 / 2.0;
            const double a0_2  = 1.0 + alpha;
            const double invA0 = (fabs(a0_2) > 1e-12) ? (1.0 / a0_2) : 1.0;
            _coeffs[1][0] = static_cast<float>((1.0 + cos0) * 0.5 * invA0);
            _coeffs[1][1] = static_cast<float>(-(1.0 + cos0) * invA0);
            _coeffs[1][2] = static_cast<float>((1.0 + cos0) * 0.5 * invA0);
            _coeffs[1][3] = static_cast<float>((-2.0 * cos0) * invA0);
            _coeffs[1][4] = static_cast<float>((1.0 - alpha) * invA0);
            _hasSecondStage = true;
        } else if (_order == 4) {
            constexpr double Q_stages[2] = { 0.541196100146197, 1.3065629648763766 };
            for (int s = 0; s < 2; ++s) {
                const double alpha = sin0 / (2.0 * Q_stages[s]);
                const double a0_s  = 1.0 + alpha;
                const double invA0 = (fabs(a0_s) > 1e-12) ? (1.0 / a0_s) : 1.0;
                _coeffs[s][0] = static_cast<float>((1.0 + cos0) * 0.5 * invA0);
                _coeffs[s][1] = static_cast<float>(-(1.0 + cos0) * invA0);
                _coeffs[s][2] = static_cast<float>((1.0 + cos0) * 0.5 * invA0);
                _coeffs[s][3] = static_cast<float>((-2.0 * cos0) * invA0);
                _coeffs[s][4] = static_cast<float>((1.0 - alpha) * invA0);
            }
            _hasSecondStage = true;
        } else {
            const double alpha = sin0 / (2.0 * 0.7071067811865476);
            const double a0_s  = 1.0 + alpha;
            const double invA0 = (fabs(a0_s) > 1e-12) ? (1.0 / a0_s) : 1.0;
            _coeffs[0][0] = static_cast<float>((1.0 + cos0) * 0.5 * invA0);
            _coeffs[0][1] = static_cast<float>(-(1.0 + cos0) * invA0);
            _coeffs[0][2] = static_cast<float>((1.0 + cos0) * 0.5 * invA0);
            _coeffs[0][3] = static_cast<float>((-2.0 * cos0) * invA0);
            _coeffs[0][4] = static_cast<float>((1.0 - alpha) * invA0);
        }
    }
};
