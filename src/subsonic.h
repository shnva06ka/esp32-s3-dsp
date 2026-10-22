// Subsonic high-pass, 1st to 4th order Butterworth. Keeps the sub from unloading below its tuning.
#pragma once

#include <cmath>
#include <cstring>
#if __has_include(<esp_dsp.h>)
#include <esp_dsp.h>
#define HAVE_ESP_DSP 1
#endif
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

#if defined(HAVE_ESP_DSP)
        dsps_biquad_f32(left,  left,  static_cast<int>(frames), _coeffs[0], _zL[0]);
        dsps_biquad_f32(right, right, static_cast<int>(frames), _coeffs[0], _zR[0]);
        if (_hasSecondStage) {
            dsps_biquad_f32(left,  left,  static_cast<int>(frames), _coeffs[1], _zL[1]);
            dsps_biquad_f32(right, right, static_cast<int>(frames), _coeffs[1], _zR[1]);
        }
#else
        _processStage(left, right, frames, _coeffs[0], _zL[0], _zR[0]);
        if (_hasSecondStage) {
            _processStage(left, right, frames, _coeffs[1], _zL[1], _zR[1]);
        }
#endif
    }

private:
    static inline void _processStage(float* left, float* right, size_t frames,
                                     const float* c, float* wL, float* wR) {
        for (size_t i = 0; i < frames; ++i) {
            float d0L = left[i] - c[3] * wL[0] - c[4] * wL[1];
            left[i]   = c[0] * d0L + c[1] * wL[0] + c[2] * wL[1];
            wL[1] = wL[0];
            wL[0] = d0L;

            float d0R = right[i] - c[3] * wR[0] - c[4] * wR[1];
            right[i]  = c[0] * d0R + c[1] * wR[0] + c[2] * wR[1];
            wR[1] = wR[0];
            wR[0] = d0R;
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
        _freq    = (p.freq < 20.0f) ? 20.0f : ((p.freq > 100.0f) ? 100.0f : p.freq);
        _order   = (p.order >= 1 && p.order <= 4) ? p.order : 2;

        const float Fs   = static_cast<float>(kSampleRate);
        const float w0   = 2.0f * M_PI * _freq / Fs;
        const float cos0 = cosf(w0);
        const float sin0 = sinf(w0);
        const float tan0 = tanf(w0 * 0.5f);

        _hasSecondStage = false;

        if (_order == 1) {
            const float b0 = 1.0f / (1.0f + tan0);
            const float a1 = (tan0 - 1.0f) / (1.0f + tan0);
            _coeffs[0][0] = b0;
            _coeffs[0][1] = -b0;
            _coeffs[0][2] = 0.0f;
            _coeffs[0][3] = a1;
            _coeffs[0][4] = 0.0f;
        } else if (_order == 3) {
            const float b0 = 1.0f / (1.0f + tan0);
            const float a1 = (tan0 - 1.0f) / (1.0f + tan0);
            _coeffs[0][0] = b0;
            _coeffs[0][1] = -b0;
            _coeffs[0][2] = 0.0f;
            _coeffs[0][3] = a1;
            _coeffs[0][4] = 0.0f;

            const float alpha = sin0 / (2.0f * 1.0f);
            const float a0_2  = 1.0f + alpha;
            const float invA0 = (fabsf(a0_2) > 1e-12f) ? (1.0f / a0_2) : 1.0f;
            _coeffs[1][0] = (1.0f + cos0) * 0.5f * invA0;
            _coeffs[1][1] = -(1.0f + cos0) * invA0;
            _coeffs[1][2] = (1.0f + cos0) * 0.5f * invA0;
            _coeffs[1][3] = (-2.0f * cos0) * invA0;
            _coeffs[1][4] = (1.0f - alpha) * invA0;
            _hasSecondStage = true;
        } else if (_order == 4) {
            constexpr float Q_stages[2] = { 1.30656296f, 0.5411961f };
            for (int s = 0; s < 2; ++s) {
                const float alpha = sin0 / (2.0f * Q_stages[s]);
                const float a0_s  = 1.0f + alpha;
                const float invA0 = (fabsf(a0_s) > 1e-12f) ? (1.0f / a0_s) : 1.0f;
                _coeffs[s][0] = (1.0f + cos0) * 0.5f * invA0;
                _coeffs[s][1] = -(1.0f + cos0) * invA0;
                _coeffs[s][2] = (1.0f + cos0) * 0.5f * invA0;
                _coeffs[s][3] = (-2.0f * cos0) * invA0;
                _coeffs[s][4] = (1.0f - alpha) * invA0;
            }
            _hasSecondStage = true;
        } else {
            const float alpha = sin0 / (2.0f * 0.70710678f);
            const float a0_s  = 1.0f + alpha;
            const float invA0 = (fabsf(a0_s) > 1e-12f) ? (1.0f / a0_s) : 1.0f;
            _coeffs[0][0] = (1.0f + cos0) * 0.5f * invA0;
            _coeffs[0][1] = -(1.0f + cos0) * invA0;
            _coeffs[0][2] = (1.0f + cos0) * 0.5f * invA0;
            _coeffs[0][3] = (-2.0f * cos0) * invA0;
            _coeffs[0][4] = (1.0f - alpha) * invA0;
        }
    }
};

