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

    bool isEnabled() const { return _enabled; }

    void process(float* left, float* right, size_t frames) {
        if (!_enabled || frames == 0) return;

        _processStage(left, right, frames, _coeffs[0], _zL[0], _zR[0]);
        if (_hasSecondStage) {
            _processStage(left, right, frames, _coeffs[1], _zL[1], _zR[1]);
        }
    }

private:
    static inline void _sanitizeBlockAndState(float* buf, size_t frames, float* z) {
        if (!std::isfinite(z[0]) || !std::isfinite(z[1])) {
            z[0] = 0.0f;
            z[1] = 0.0f;
            for (size_t i = 0; i < frames; ++i) {
                if (!std::isfinite(buf[i])) buf[i] = 0.0f;
            }
        } else {
            if (fabsf(z[0]) < 1e-15f) z[0] = 0.0f;
            if (fabsf(z[1]) < 1e-15f) z[1] = 0.0f;
        }
    }

    // Transposed Direct Form II biquad
    static inline void _processStageChannel(float* buf, size_t frames,
                                            const float* c, float* z) {
        const float b0 = c[0], b1 = c[1], b2 = c[2], a1 = c[3], a2 = c[4];
        float z0 = z[0], z1 = z[1];
#if defined(__XTENSA__)
        float in, out, t0;
        uint32_t cnt = static_cast<uint32_t>(frames);
        asm volatile(
            "loopnez %[cnt], 1f              \n\t"
            "lsi     %[in],  %[buf], 0       \n\t"
            "mov.s   %[out], %[z0]           \n\t"
            "madd.s  %[out], %[b0],  %[in]   \n\t"
            "mul.s   %[z0],  %[b1],  %[in]   \n\t"
            "mul.s   %[t0],  %[b2],  %[in]   \n\t"
            "msub.s  %[z0],  %[a1],  %[out]  \n\t"
            "add.s   %[z0],  %[z0],  %[z1]   \n\t"
            "msub.s  %[t0],  %[a2],  %[out]  \n\t"
            "ssi     %[out], %[buf], 0       \n\t"
            "mov.s   %[z1],  %[t0]           \n\t"
            "addi    %[buf], %[buf], 4       \n\t"
            "1:                              \n\t"
            : [buf] "+&r" (buf),
              [z0]  "+&f" (z0),
              [z1]  "+&f" (z1),
              [in]  "=&f" (in),
              [out] "=&f" (out),
              [t0]  "=&f" (t0)
            : [cnt] "r"  (cnt),
              [b0]  "f"  (b0),
              [b1]  "f"  (b1),
              [b2]  "f"  (b2),
              [a1]  "f"  (a1),
              [a2]  "f"  (a2)
            : "memory"
        );
#else
        for (size_t i = 0; i < frames; ++i) {
            const float in = buf[i];
            const float out = b0 * in + z0;
            z0 = b1 * in - a1 * out + z1;
            z1 = b2 * in - a2 * out;
            buf[i] = out;
        }
#endif
        z[0] = z0;
        z[1] = z1;
    }

    static inline void _processStage(float* left, float* right, size_t frames,
                                     const float* c, float* zL, float* zR) {
        _processStageChannel(left,  frames, c, zL);
        _sanitizeBlockAndState(left,  frames, zL);
        _processStageChannel(right, frames, c, zR);
        _sanitizeBlockAndState(right, frames, zR);
    }

    bool  _enabled{false};
    float _freq{80.0f};
    uint8_t _order{2};

    float _coeffs[2][5]{};
    float _zL[2][2]{};
    float _zR[2][2]{};
    bool  _hasSecondStage{false};

    void _update(const SubsonicParams& p) {
        _enabled = p.enabled && (p.order >= 1 && p.order <= 4);
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
