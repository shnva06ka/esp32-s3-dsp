#pragma once

#include <cmath>
#include <cstring>
#if __has_include(<esp_dsp.h>)
#include <esp_dsp.h>
#ifndef HAVE_ESP_DSP
#define HAVE_ESP_DSP 1
#endif
#endif
#include "audio_config.h"
#include "param_store.h"

class BiquadEQ {
public:
    BiquadEQ() {
        reset();
        constexpr float defaultFreqs[kEQBands] = {
            100.0f, 1000.0f, 10000.0f, 250.0f, 500.0f,
            2000.0f, 4000.0f, 8000.0f, 16000.0f, 50.0f
        };
        for (int b = 0; b < kEQBands; ++b) {
            _hasSecondStage[b] = false;
            _calcCoeffs(b, {
                defaultFreqs[b], 0.0f, 0.707f, true, EQ_FILTER_PEAK
            });
        }
    }

    void reset() {
        memset(_zL, 0, sizeof(_zL));
        memset(_zR, 0, sizeof(_zR));
    }

    void setBand(int band, const EQBandParams& p) {
        if (band < 0 || band >= kEQBands) return;
        _calcCoeffs(band, p);
        _enabled[band] = p.enabled;
    }

    void process(float* left, float* right, size_t frames, int numBands = kEQBands) {
        if (frames == 0 || numBands <= 0) return;
        if (numBands > kEQBands) numBands = kEQBands;
        bool anyEnabled = false;
        for (int b = 0; b < numBands; ++b) {
            if (!_enabled[b]) continue;
            anyEnabled = true;
#if defined(HAVE_ESP_DSP)
            dsps_biquad_f32(left,  left,  static_cast<int>(frames), _coeffs[b][0], _zL[b][0]);
            dsps_biquad_f32(right, right, static_cast<int>(frames), _coeffs[b][0], _zR[b][0]);
            _sanitizeBlockAndState(left, frames, _zL[b][0], _coeffs[b][0]);
            _sanitizeBlockAndState(right, frames, _zR[b][0], _coeffs[b][0]);
            if (_hasSecondStage[b]) {
                dsps_biquad_f32(left,  left,  static_cast<int>(frames), _coeffs[b][1], _zL[b][1]);
                dsps_biquad_f32(right, right, static_cast<int>(frames), _coeffs[b][1], _zR[b][1]);
                _sanitizeBlockAndState(left, frames, _zL[b][1], _coeffs[b][1]);
                _sanitizeBlockAndState(right, frames, _zR[b][1], _coeffs[b][1]);
            }
#else
            _processStageChannel(left,  frames, _coeffs[b][0], _zL[b][0]);
            _processStageChannel(right, frames, _coeffs[b][0], _zR[b][0]);
            if (_hasSecondStage[b]) {
                _processStageChannel(left,  frames, _coeffs[b][1], _zL[b][1]);
                _processStageChannel(right, frames, _coeffs[b][1], _zR[b][1]);
            }
#endif
        }
        if (!anyEnabled) {
            _scrubBlockNonFinite(left, frames);
            _scrubBlockNonFinite(right, frames);
        }
    }

private:
    static inline void _scrubBlockNonFinite(float* buf, size_t frames) {
        for (size_t i = 0; i < frames; ++i) {
            if (!std::isfinite(buf[i])) buf[i] = 0.0f;
        }
    }

    static inline void _sanitizeBlockAndState(float* buf, size_t frames, float* z, const float* c) {
        const bool zeroFeedback = (c[3] == 0.0f && c[4] == 0.0f);
        if (!std::isfinite(z[0]) || !std::isfinite(z[1])) {
            z[0] = 0.0f;
            z[1] = 0.0f;
            _scrubBlockNonFinite(buf, frames);
        } else {
            if (zeroFeedback) {
                _scrubBlockNonFinite(buf, frames);
            }
            if (fabsf(z[0]) < 1e-15f) z[0] = 0.0f;
            if (fabsf(z[1]) < 1e-15f) z[1] = 0.0f;
        }
    }

    static inline void _processStageChannel(float* buf, size_t frames,
                                            const float* c, float* w) {
        const float b0 = c[0], b1 = c[1], b2 = c[2], a1 = c[3], a2 = c[4];
        float w0 = w[0], w1 = w[1];
        for (size_t i = 0; i < frames; ++i) {
            const float d0 = buf[i] - a1 * w0 - a2 * w1;
            buf[i] = b0 * d0 + b1 * w0 + b2 * w1;
            w1 = w0;
            w0 = d0;
        }
        w[0] = w0;
        w[1] = w1;
        _sanitizeBlockAndState(buf, frames, w, c);
    }

    float _coeffs[kEQBands][2][5]{};
    float _zL[kEQBands][2][2]{};
    float _zR[kEQBands][2][2]{};
    bool  _hasSecondStage[kEQBands]{};
    bool  _enabled[kEQBands]{};

    void _calcCoeffs(int b, const EQBandParams& p) {
        const double Fs       = static_cast<double>(kSampleRate);
        const double rawFreq  = std::isfinite(p.freq) ? static_cast<double>(p.freq) : 1000.0;
        const double rawGain  = std::isfinite(p.gain) ? static_cast<double>(p.gain) : 0.0;
        const double rawQ     = std::isfinite(p.q)    ? static_cast<double>(p.q)    : 0.70710678;

        const double safeFreq = (rawFreq < 20.0) ? 20.0 : ((rawFreq > (Fs * 0.49)) ? (Fs * 0.49) : rawFreq);
        const double safeGain = (rawGain < -24.0) ? -24.0 : ((rawGain > 24.0) ? 24.0 : rawGain);
        const double safeQ    = (rawQ < 0.05) ? 0.05 : ((rawQ > 10.0) ? 10.0 : rawQ);

        const double w0       = 2.0 * M_PI * safeFreq / Fs;
        const double cos0     = cos(w0);
        const double sin0     = sin(w0);
        const double tan0     = tan(w0 * 0.5);

        uint8_t type  = p.type;
        uint8_t order = (p.order >= 1 && p.order <= 4) ? p.order : 2;

        if (type == EQ_FILTER_FLAT_LOW_PASS) {
            type = EQ_FILTER_LOW_PASS;
            order = 4;
        } else if (type == EQ_FILTER_FLAT_HIGH_PASS) {
            type = EQ_FILTER_HIGH_PASS;
            order = 4;
        }

        _hasSecondStage[b] = false;

        if (type == EQ_FILTER_LOW_PASS) {
            if (order == 1) {
                const double b0_1 = tan0 / (1.0 + tan0);
                const double a1_1 = (tan0 - 1.0) / (1.0 + tan0);
                _coeffs[b][0][0] = static_cast<float>(b0_1);
                _coeffs[b][0][1] = static_cast<float>(b0_1);
                _coeffs[b][0][2] = 0.0f;
                _coeffs[b][0][3] = static_cast<float>(a1_1);
                _coeffs[b][0][4] = 0.0f;
            } else if (order == 3) {
                const double b0_1 = tan0 / (1.0 + tan0);
                const double a1_1 = (tan0 - 1.0) / (1.0 + tan0);
                _coeffs[b][0][0] = static_cast<float>(b0_1);
                _coeffs[b][0][1] = static_cast<float>(b0_1);
                _coeffs[b][0][2] = 0.0f;
                _coeffs[b][0][3] = static_cast<float>(a1_1);
                _coeffs[b][0][4] = 0.0f;

                const double alpha = sin0 / 2.0;
                const double a0_2  = 1.0 + alpha;
                const double invA0 = (fabs(a0_2) > 1e-12) ? (1.0 / a0_2) : 1.0;
                _coeffs[b][1][0] = static_cast<float>((1.0 - cos0) * 0.5 * invA0);
                _coeffs[b][1][1] = static_cast<float>((1.0 - cos0) * invA0);
                _coeffs[b][1][2] = static_cast<float>((1.0 - cos0) * 0.5 * invA0);
                _coeffs[b][1][3] = static_cast<float>((-2.0 * cos0) * invA0);
                _coeffs[b][1][4] = static_cast<float>((1.0 - alpha) * invA0);
                _hasSecondStage[b] = true;
            } else if (order == 4) {
                constexpr double Q_stages[2] = { 0.541196100146197, 1.3065629648763766 };
                for (int s = 0; s < 2; ++s) {
                    const double alpha = sin0 / (2.0 * Q_stages[s]);
                    const double a0_s  = 1.0 + alpha;
                    const double invA0 = (fabs(a0_s) > 1e-12) ? (1.0 / a0_s) : 1.0;
                    _coeffs[b][s][0] = static_cast<float>((1.0 - cos0) * 0.5 * invA0);
                    _coeffs[b][s][1] = static_cast<float>((1.0 - cos0) * invA0);
                    _coeffs[b][s][2] = static_cast<float>((1.0 - cos0) * 0.5 * invA0);
                    _coeffs[b][s][3] = static_cast<float>((-2.0 * cos0) * invA0);
                    _coeffs[b][s][4] = static_cast<float>((1.0 - alpha) * invA0);
                }
                _hasSecondStage[b] = true;
            } else {
                constexpr double kButterQ2 = 0.7071067811865476;
                const double alpha = sin0 / (2.0 * kButterQ2);
                const double a0_s  = 1.0 + alpha;
                const double invA0 = (fabs(a0_s) > 1e-12) ? (1.0 / a0_s) : 1.0;
                _coeffs[b][0][0] = static_cast<float>((1.0 - cos0) * 0.5 * invA0);
                _coeffs[b][0][1] = static_cast<float>((1.0 - cos0) * invA0);
                _coeffs[b][0][2] = static_cast<float>((1.0 - cos0) * 0.5 * invA0);
                _coeffs[b][0][3] = static_cast<float>((-2.0 * cos0) * invA0);
                _coeffs[b][0][4] = static_cast<float>((1.0 - alpha) * invA0);
            }
            return;
        }

        if (type == EQ_FILTER_HIGH_PASS) {
            if (order == 1) {
                const double b0_1 = 1.0 / (1.0 + tan0);
                const double a1_1 = (tan0 - 1.0) / (1.0 + tan0);
                _coeffs[b][0][0] = static_cast<float>(b0_1);
                _coeffs[b][0][1] = static_cast<float>(-b0_1);
                _coeffs[b][0][2] = 0.0f;
                _coeffs[b][0][3] = static_cast<float>(a1_1);
                _coeffs[b][0][4] = 0.0f;
            } else if (order == 3) {
                const double b0_1 = 1.0 / (1.0 + tan0);
                const double a1_1 = (tan0 - 1.0) / (1.0 + tan0);
                _coeffs[b][0][0] = static_cast<float>(b0_1);
                _coeffs[b][0][1] = static_cast<float>(-b0_1);
                _coeffs[b][0][2] = 0.0f;
                _coeffs[b][0][3] = static_cast<float>(a1_1);
                _coeffs[b][0][4] = 0.0f;

                const double alpha = sin0 / 2.0;
                const double a0_2  = 1.0 + alpha;
                const double invA0 = (fabs(a0_2) > 1e-12) ? (1.0 / a0_2) : 1.0;
                _coeffs[b][1][0] = static_cast<float>((1.0 + cos0) * 0.5 * invA0);
                _coeffs[b][1][1] = static_cast<float>(-(1.0 + cos0) * invA0);
                _coeffs[b][1][2] = static_cast<float>((1.0 + cos0) * 0.5 * invA0);
                _coeffs[b][1][3] = static_cast<float>((-2.0 * cos0) * invA0);
                _coeffs[b][1][4] = static_cast<float>((1.0 - alpha) * invA0);
                _hasSecondStage[b] = true;
            } else if (order == 4) {
                constexpr double Q_stages[2] = { 0.541196100146197, 1.3065629648763766 };
                for (int s = 0; s < 2; ++s) {
                    const double alpha = sin0 / (2.0 * Q_stages[s]);
                    const double a0_s  = 1.0 + alpha;
                    const double invA0 = (fabs(a0_s) > 1e-12) ? (1.0 / a0_s) : 1.0;
                    _coeffs[b][s][0] = static_cast<float>((1.0 + cos0) * 0.5 * invA0);
                    _coeffs[b][s][1] = static_cast<float>(-(1.0 + cos0) * invA0);
                    _coeffs[b][s][2] = static_cast<float>((1.0 + cos0) * 0.5 * invA0);
                    _coeffs[b][s][3] = static_cast<float>((-2.0 * cos0) * invA0);
                    _coeffs[b][s][4] = static_cast<float>((1.0 - alpha) * invA0);
                }
                _hasSecondStage[b] = true;
            } else {
                constexpr double kButterQ2 = 0.7071067811865476;
                const double alpha = sin0 / (2.0 * kButterQ2);
                const double a0_s  = 1.0 + alpha;
                const double invA0 = (fabs(a0_s) > 1e-12) ? (1.0 / a0_s) : 1.0;
                _coeffs[b][0][0] = static_cast<float>((1.0 + cos0) * 0.5 * invA0);
                _coeffs[b][0][1] = static_cast<float>(-(1.0 + cos0) * invA0);
                _coeffs[b][0][2] = static_cast<float>((1.0 + cos0) * 0.5 * invA0);
                _coeffs[b][0][3] = static_cast<float>((-2.0 * cos0) * invA0);
                _coeffs[b][0][4] = static_cast<float>((1.0 - alpha) * invA0);
            }
            return;
        }

        const double alpha = sin0 / (2.0 * safeQ);
        double b0 = 1.0, b1 = 0.0, b2 = 0.0;
        double a0 = 1.0, a1 = 0.0, a2 = 0.0;

        switch (type) {
            case EQ_FILTER_LOW_SHELF: {
                const double A = pow(10.0, safeGain / 40.0);
                const double twoSqrtAAlpha = 2.0 * sqrt(A) * alpha;
                b0 =    A * ((A + 1.0) - (A - 1.0) * cos0 + twoSqrtAAlpha);
                b1 = 2.0 * A * ((A - 1.0) - (A + 1.0) * cos0);
                b2 =    A * ((A + 1.0) - (A - 1.0) * cos0 - twoSqrtAAlpha);
                a0 =         (A + 1.0) + (A - 1.0) * cos0 + twoSqrtAAlpha;
                a1 = -2.0 * ((A - 1.0) + (A + 1.0) * cos0);
                a2 =         (A + 1.0) + (A - 1.0) * cos0 - twoSqrtAAlpha;
                break;
            }
            case EQ_FILTER_HIGH_SHELF: {
                const double A = pow(10.0, safeGain / 40.0);
                const double twoSqrtAAlpha = 2.0 * sqrt(A) * alpha;
                b0 =    A * ((A + 1.0) + (A - 1.0) * cos0 + twoSqrtAAlpha);
                b1 = -2.0 * A * ((A - 1.0) + (A + 1.0) * cos0);
                b2 =    A * ((A + 1.0) + (A - 1.0) * cos0 - twoSqrtAAlpha);
                a0 =         (A + 1.0) - (A - 1.0) * cos0 + twoSqrtAAlpha;
                a1 =  2.0 * ((A - 1.0) - (A + 1.0) * cos0);
                a2 =         (A + 1.0) - (A - 1.0) * cos0 - twoSqrtAAlpha;
                break;
            }
            case EQ_FILTER_PEAK:
            default: {
                const double A = pow(10.0, safeGain / 40.0);
                b0 =  1.0 + alpha * A;
                b1 = -2.0 * cos0;
                b2 =  1.0 - alpha * A;
                a0 =  1.0 + alpha / A;
                a1 = -2.0 * cos0;
                a2 =  1.0 - alpha / A;
                break;
            }
        }

        const double invA0 = (fabs(a0) > 1e-12) ? (1.0 / a0) : 1.0;
        _coeffs[b][0][0] = static_cast<float>(b0 * invA0);
        _coeffs[b][0][1] = static_cast<float>(b1 * invA0);
        _coeffs[b][0][2] = static_cast<float>(b2 * invA0);
        _coeffs[b][0][3] = static_cast<float>(a1 * invA0);
        _coeffs[b][0][4] = static_cast<float>(a2 * invA0);
    }
};
