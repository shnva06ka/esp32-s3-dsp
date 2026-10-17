/**
 * @file biquad_eq.h
 * @brief 10-band stereo Parametric EQ using Direct Form II Transposed biquads.
 *
 * Each band is a peaking (bell) filter recalculated whenever the host
 * changes frequency, gain or Q.  Coefficients are stored as
 * esp_dsp-compatible b0/b1/b2/a1/a2 arrays and processed with the
 * optimised dsps_biquad_f32() SIMD function.
 *
 * Coefficient reference: Audio EQ Cookbook (R. Bristow-Johnson)
 *   H(z) peaking = (b0 + b1*z^-1 + b2*z^-^2) / (1 + a1*z^-1 + a2*z^-^2)
 */
#pragma once

#include <cmath>
#include <cstring>
#if __has_include(<esp_dsp.h>)
#include <esp_dsp.h>
#define HAVE_ESP_DSP 1
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
        // Load flat (0 dB) coefficients for all bands
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

    /** @brief Recalculate band when params change. */
    void setBand(int band, const EQBandParams& p) {
        if (band < 0 || band >= kEQBands) return;
        _calcCoeffs(band, p);
        _enabled[band] = p.enabled;
    }

    /**
     * @brief Process stereo audio through all active enabled bands in-place.
     * @param left     Left channel float buffer
     * @param right    Right channel float buffer
     * @param frames   Number of sample frames
     * @param numBands Number of active bands to process (1 to kEQBands)
     */
    void process(float* left, float* right, size_t frames, int numBands = kEQBands) {
        if (numBands > kEQBands) numBands = kEQBands;
        for (int b = 0; b < numBands; ++b) {
            if (!_enabled[b]) continue;
#if defined(HAVE_ESP_DSP)
            dsps_biquad_f32(left,  left,  static_cast<int>(frames), _coeffs[b][0], _zL[b][0]);
            dsps_biquad_f32(right, right, static_cast<int>(frames), _coeffs[b][0], _zR[b][0]);
            if (_hasSecondStage[b]) {
                dsps_biquad_f32(left,  left,  static_cast<int>(frames), _coeffs[b][1], _zL[b][1]);
                dsps_biquad_f32(right, right, static_cast<int>(frames), _coeffs[b][1], _zR[b][1]);
            }
#else
            _processStage(left, right, frames, _coeffs[b][0], _zL[b][0], _zR[b][0]);
            if (_hasSecondStage[b]) {
                _processStage(left, right, frames, _coeffs[b][1], _zL[b][1], _zR[b][1]);
            }
#endif
        }
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

    // esp-dsp biquad: coeffs[5] = {b0, b1, b2, a1, a2}  (IIR direct form II)
    // 2 biquad stages per band to support 3rd (18 dB/oct) and 4th (24 dB/oct) order filters
    float _coeffs[kEQBands][2][5];
    float _zL[kEQBands][2][2];
    float _zR[kEQBands][2][2];
    bool  _hasSecondStage[kEQBands]{};
    bool  _enabled[kEQBands]{};

    void _calcCoeffs(int b, const EQBandParams& p) {
        const float Fs       = static_cast<float>(kSampleRate);
        const float safeFreq = (p.freq < 20.0f) ? 20.0f : ((p.freq > (Fs * 0.49f)) ? (Fs * 0.49f) : p.freq);
        const float safeQ    = (p.q < 0.05f) ? 0.05f : p.q;
        const float w0       = 2.0f * M_PI * safeFreq / Fs;
        const float cos0     = cosf(w0);
        const float sin0     = sinf(w0);
        const float tan0     = tanf(w0 * 0.5f);

        uint8_t type  = p.type;
        uint8_t order = (p.order >= 1 && p.order <= 4) ? p.order : 2;

        // Map legacy flat types to order 4
        if (type == EQ_FILTER_FLAT_LOW_PASS) {
            type = EQ_FILTER_LOW_PASS;
            order = 4;
        } else if (type == EQ_FILTER_FLAT_HIGH_PASS) {
            type = EQ_FILTER_HIGH_PASS;
            order = 4;
        }

        _hasSecondStage[b] = false;

        if (type == EQ_FILTER_LOW_PASS) {
            if (order == 1) { // 1st Order (6 dB/oct)
                const float b0 = tan0 / (1.0f + tan0);
                const float a1 = (tan0 - 1.0f) / (1.0f + tan0);
                _coeffs[b][0][0] = b0;
                _coeffs[b][0][1] = b0;
                _coeffs[b][0][2] = 0.0f;
                _coeffs[b][0][3] = a1;
                _coeffs[b][0][4] = 0.0f;
            } else if (order == 3) { // 3rd Order Butterworth (18 dB/oct: 1st order + 2nd order Q=1.0)
                // Stage 0: 1st order LP
                const float b0 = tan0 / (1.0f + tan0);
                const float a1 = (tan0 - 1.0f) / (1.0f + tan0);
                _coeffs[b][0][0] = b0;
                _coeffs[b][0][1] = b0;
                _coeffs[b][0][2] = 0.0f;
                _coeffs[b][0][3] = a1;
                _coeffs[b][0][4] = 0.0f;

                // Stage 1: 2nd order LP with Q = 1.0
                const float alpha = sin0 / (2.0f * 1.0f);
                const float a0_2  = 1.0f + alpha;
                const float invA0 = (fabsf(a0_2) > 1e-12f) ? (1.0f / a0_2) : 1.0f;
                _coeffs[b][1][0] = (1.0f - cos0) * 0.5f * invA0;
                _coeffs[b][1][1] = (1.0f - cos0) * invA0;
                _coeffs[b][1][2] = (1.0f - cos0) * 0.5f * invA0;
                _coeffs[b][1][3] = (-2.0f * cos0) * invA0;
                _coeffs[b][1][4] = (1.0f - alpha) * invA0;
                _hasSecondStage[b] = true;
            } else if (order == 4) { // 4th Order Butterworth (24 dB/oct: 2 stages Q1=1.30656, Q2=0.5412)
                constexpr float Q_stages[2] = { 1.30656296f, 0.5411961f };
                for (int s = 0; s < 2; ++s) {
                    const float alpha = sin0 / (2.0f * Q_stages[s]);
                    const float a0_s  = 1.0f + alpha;
                    const float invA0 = (fabsf(a0_s) > 1e-12f) ? (1.0f / a0_s) : 1.0f;
                    _coeffs[b][s][0] = (1.0f - cos0) * 0.5f * invA0;
                    _coeffs[b][s][1] = (1.0f - cos0) * invA0;
                    _coeffs[b][s][2] = (1.0f - cos0) * 0.5f * invA0;
                    _coeffs[b][s][3] = (-2.0f * cos0) * invA0;
                    _coeffs[b][s][4] = (1.0f - alpha) * invA0;
                }
                _hasSecondStage[b] = true;
            } else { // 2nd Order (12 dB/oct standard)
                const float alpha = sin0 / (2.0f * safeQ);
                const float a0_s  = 1.0f + alpha;
                const float invA0 = (fabsf(a0_s) > 1e-12f) ? (1.0f / a0_s) : 1.0f;
                _coeffs[b][0][0] = (1.0f - cos0) * 0.5f * invA0;
                _coeffs[b][0][1] = (1.0f - cos0) * invA0;
                _coeffs[b][0][2] = (1.0f - cos0) * 0.5f * invA0;
                _coeffs[b][0][3] = (-2.0f * cos0) * invA0;
                _coeffs[b][0][4] = (1.0f - alpha) * invA0;
            }
            return;
        }

        if (type == EQ_FILTER_HIGH_PASS) {
            if (order == 1) { // 1st Order (6 dB/oct)
                const float b0 = 1.0f / (1.0f + tan0);
                const float a1 = (tan0 - 1.0f) / (1.0f + tan0);
                _coeffs[b][0][0] = b0;
                _coeffs[b][0][1] = -b0;
                _coeffs[b][0][2] = 0.0f;
                _coeffs[b][0][3] = a1;
                _coeffs[b][0][4] = 0.0f;
            } else if (order == 3) { // 3rd Order Butterworth (18 dB/oct)
                // Stage 0: 1st order HP
                const float b0 = 1.0f / (1.0f + tan0);
                const float a1 = (tan0 - 1.0f) / (1.0f + tan0);
                _coeffs[b][0][0] = b0;
                _coeffs[b][0][1] = -b0;
                _coeffs[b][0][2] = 0.0f;
                _coeffs[b][0][3] = a1;
                _coeffs[b][0][4] = 0.0f;

                // Stage 1: 2nd order HP with Q = 1.0
                const float alpha = sin0 / (2.0f * 1.0f);
                const float a0_2  = 1.0f + alpha;
                const float invA0 = (fabsf(a0_2) > 1e-12f) ? (1.0f / a0_2) : 1.0f;
                _coeffs[b][1][0] = (1.0f + cos0) * 0.5f * invA0;
                _coeffs[b][1][1] = -(1.0f + cos0) * invA0;
                _coeffs[b][1][2] = (1.0f + cos0) * 0.5f * invA0;
                _coeffs[b][1][3] = (-2.0f * cos0) * invA0;
                _coeffs[b][1][4] = (1.0f - alpha) * invA0;
                _hasSecondStage[b] = true;
            } else if (order == 4) { // 4th Order Butterworth (24 dB/oct)
                constexpr float Q_stages[2] = { 1.30656296f, 0.5411961f };
                for (int s = 0; s < 2; ++s) {
                    const float alpha = sin0 / (2.0f * Q_stages[s]);
                    const float a0_s  = 1.0f + alpha;
                    const float invA0 = (fabsf(a0_s) > 1e-12f) ? (1.0f / a0_s) : 1.0f;
                    _coeffs[b][s][0] = (1.0f + cos0) * 0.5f * invA0;
                    _coeffs[b][s][1] = -(1.0f + cos0) * invA0;
                    _coeffs[b][s][2] = (1.0f + cos0) * 0.5f * invA0;
                    _coeffs[b][s][3] = (-2.0f * cos0) * invA0;
                    _coeffs[b][s][4] = (1.0f - alpha) * invA0;
                }
                _hasSecondStage[b] = true;
            } else { // 2nd Order (12 dB/oct standard)
                const float alpha = sin0 / (2.0f * safeQ);
                const float a0_s  = 1.0f + alpha;
                const float invA0 = (fabsf(a0_s) > 1e-12f) ? (1.0f / a0_s) : 1.0f;
                _coeffs[b][0][0] = (1.0f + cos0) * 0.5f * invA0;
                _coeffs[b][0][1] = -(1.0f + cos0) * invA0;
                _coeffs[b][0][2] = (1.0f + cos0) * 0.5f * invA0;
                _coeffs[b][0][3] = (-2.0f * cos0) * invA0;
                _coeffs[b][0][4] = (1.0f - alpha) * invA0;
            }
            return;
        }

        // Standard 2nd-order filters for stage 0
        const float alpha = sin0 / (2.0f * safeQ);
        float b0 = 1.0f, b1 = 0.0f, b2 = 0.0f;
        float a0 = 1.0f, a1 = 0.0f, a2 = 0.0f;

        switch (type) {
            case EQ_FILTER_LOW_SHELF: {
                const float A = powf(10.0f, p.gain / 40.0f);
                const float twoSqrtAAlpha = 2.0f * sqrtf(A) * alpha;
                b0 =    A * ((A + 1.0f) - (A - 1.0f) * cos0 + twoSqrtAAlpha);
                b1 = 2.0f * A * ((A - 1.0f) - (A + 1.0f) * cos0);
                b2 =    A * ((A + 1.0f) - (A - 1.0f) * cos0 - twoSqrtAAlpha);
                a0 =         (A + 1.0f) + (A - 1.0f) * cos0 + twoSqrtAAlpha;
                a1 = -2.0f * ((A - 1.0f) + (A + 1.0f) * cos0);
                a2 =         (A + 1.0f) + (A - 1.0f) * cos0 - twoSqrtAAlpha;
                break;
            }
            case EQ_FILTER_HIGH_SHELF: {
                const float A = powf(10.0f, p.gain / 40.0f);
                const float twoSqrtAAlpha = 2.0f * sqrtf(A) * alpha;
                b0 =    A * ((A + 1.0f) + (A - 1.0f) * cos0 + twoSqrtAAlpha);
                b1 = -2.0f * A * ((A - 1.0f) + (A + 1.0f) * cos0);
                b2 =    A * ((A + 1.0f) + (A - 1.0f) * cos0 - twoSqrtAAlpha);
                a0 =         (A + 1.0f) - (A - 1.0f) * cos0 + twoSqrtAAlpha;
                a1 =  2.0f * ((A - 1.0f) - (A + 1.0f) * cos0);
                a2 =         (A + 1.0f) - (A - 1.0f) * cos0 - twoSqrtAAlpha;
                break;
            }
            case EQ_FILTER_PEAK:
            default: {
                const float A = powf(10.0f, p.gain / 40.0f);
                b0 =  1.0f + alpha * A;
                b1 = -2.0f * cos0;
                b2 =  1.0f - alpha * A;
                a0 =  1.0f + alpha / A;
                a1 = -2.0f * cos0;
                a2 =  1.0f - alpha / A;
                break;
            }
        }

        // Normalise by a0 for stage 0
        const float invA0 = (fabsf(a0) > 1e-12f) ? (1.0f / a0) : 1.0f;
        _coeffs[b][0][0] = b0 * invA0;
        _coeffs[b][0][1] = b1 * invA0;
        _coeffs[b][0][2] = b2 * invA0;
        _coeffs[b][0][3] = a1 * invA0;
        _coeffs[b][0][4] = a2 * invA0;
        // Do NOT reset delay lines here: preserving state prevents audible clicks on knob movement
    }
};

