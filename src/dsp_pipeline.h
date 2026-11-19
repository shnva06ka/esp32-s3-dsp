#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <cfloat>
#if __has_include(<esp_dsp.h>)
#include <esp_dsp.h>
#ifndef HAVE_ESP_DSP
#define HAVE_ESP_DSP 1
#endif
#endif
#include "audio_config.h"
#if defined(ESP_PLATFORM) || defined(ARDUINO_ARCH_ESP32)
#include "param_store.h"
#include "wireless_console.h"
#endif

enum class DspModuleId : uint8_t {
    Eq = 0,
    Subsonic,
    Compander,
    Limiter,
    Unknown
};

static inline DspModuleId parseDspModuleId(const char* token) {
    if (!token || *token == '\0') return DspModuleId::Unknown;
    if (strcmp(token, "eq") == 0) return DspModuleId::Eq;
    if (strcmp(token, "subsonic") == 0 || strcmp(token, "sub") == 0) return DspModuleId::Subsonic;
    if (strcmp(token, "comp") == 0 || strcmp(token, "compressor") == 0) return DspModuleId::Compander;
    if (strcmp(token, "lim") == 0 || strcmp(token, "limiter") == 0) return DspModuleId::Limiter;
    return DspModuleId::Unknown;
}

static inline const char* toDspModuleToken(DspModuleId id) {
    switch (id) {
        case DspModuleId::Eq:        return "eq";
        case DspModuleId::Subsonic:  return "subsonic";
        case DspModuleId::Compander: return "comp";
        case DspModuleId::Limiter:   return "lim";
        default:                     return "";
    }
}

#if defined(ESP_PLATFORM)
#  include "esp_heap_caps.h"
#endif

#if !defined(__XTENSA__) && !defined(__GNUC__) && !defined(__clang__)
#  include <math.h>
#endif

#if defined(__GNUC__) || defined(__clang__)
#  define DSP_RESTRICT __restrict__
#else
#  define DSP_RESTRICT
#endif

static inline float dsp_fclamp_pm1(float x) {
#if defined(__XTENSA__)
    const float hi = 1.0f;
    const float lo = -1.0f;
    __asm__ (
        "olt.s  b0, %[hi], %[x]\n\t"
        "movt.s %[x], %[hi], b0\n\t"
        "olt.s  b0, %[x], %[lo]\n\t"
        "movt.s %[x], %[lo], b0"
        : [x] "+&f" (x)
        : [hi] "f" (hi), [lo] "f" (lo)
        : "b0"
    );
    return x;
#else
    if (x > 1.0f) x = 1.0f;
    if (x < -1.0f) x = -1.0f;
    return x;
#endif
}

static inline int32_t dsp_sanitize_clamp_f32_to_i32(float x) {
#if defined(__XTENSA__)
    const float fltmax = FLT_MAX;
    const float zero   = 0.0f;
    const float hi     = 1.0f;
    const float lo     = -1.0f;
    const float scale  = 2147483392.0f;
    float abs_x;
    int32_t res;
    __asm__ (
        "abs.s   %[absx], %[x]\n\t"
        "ult.s   b0, %[fltmax], %[absx]\n\t"
        "movt.s  %[x], %[zero], b0\n\t"
        "olt.s   b0, %[hi], %[x]\n\t"
        "movt.s  %[x], %[hi], b0\n\t"
        "olt.s   b0, %[x], %[lo]\n\t"
        "movt.s  %[x], %[lo], b0\n\t"
        "mul.s   %[x], %[x], %[scale]\n\t"
        "trunc.s %[res], %[x], 0"
        : [x] "+&f" (x), [absx] "=&f" (abs_x), [res] "=r" (res)
        : [fltmax] "f" (fltmax), [zero] "f" (zero), [hi] "f" (hi), [lo] "f" (lo), [scale] "f" (scale)
        : "b0"
    );
    return res;
#else
#  if defined(__GNUC__) || defined(__clang__)
    float c = __builtin_isfinite(x) ? x : 0.0f;
#  else
    float c = isfinite(x) ? x : 0.0f;
#  endif
    if (c > 1.0f) c = 1.0f;
    if (c < -1.0f) c = -1.0f;
    return (int32_t)(c * 2147483392.0f);
#endif
}

static inline void *dsp_alloc_staging(size_t bytes) {
    if (bytes == 0 || bytes > (SIZE_MAX - 15u)) {
        return NULL;
    }
    const size_t aligned_bytes = (bytes + 15u) & ~((size_t)15u);
#if defined(ESP_PLATFORM)
    const size_t kInternalCapBytes      = 8192u;
    const size_t kInternalHeadroomBytes = 98304u;
    const uint32_t int_caps   = MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT;
    const uint32_t psram_caps = MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT;

    void *p = NULL;
    if (aligned_bytes <= kInternalCapBytes) {
        const size_t free_internal = heap_caps_get_free_size(int_caps);
        if (free_internal >= (aligned_bytes + kInternalHeadroomBytes)) {
            p = heap_caps_aligned_alloc(16, aligned_bytes, int_caps);
            if (!p) {
                p = heap_caps_malloc(aligned_bytes, int_caps);
            }
            if (p) {
                return p;
            }
        }
    }

    p = heap_caps_aligned_alloc(16, aligned_bytes, psram_caps);
    if (!p) {
        p = heap_caps_malloc(aligned_bytes, psram_caps);
    }
    if (!p) {
        p = heap_caps_aligned_alloc(16, aligned_bytes, int_caps);
    }
    if (!p) {
        p = heap_caps_malloc(aligned_bytes, int_caps);
    }
    return p;
#else
    return malloc(aligned_bytes);
#endif
}

static inline void dsp_unpack_i32_stereo_to_f32(const int32_t * DSP_RESTRICT src,
                                                float * DSP_RESTRICT dstL,
                                                float * DSP_RESTRICT dstR,
                                                size_t frames,
                                                float scaleL,
                                                float scaleR) {
    size_t i = 0;
    for (; i + 4u <= frames; i += 4u) {
        const int32_t s0L = src[0], s0R = src[1];
        const int32_t s1L = src[2], s1R = src[3];
        dstL[0] = dsp_fclamp_pm1((float)s0L * scaleL);
        dstR[0] = dsp_fclamp_pm1((float)s0R * scaleR);
        dstL[1] = dsp_fclamp_pm1((float)s1L * scaleL);
        dstR[1] = dsp_fclamp_pm1((float)s1R * scaleR);

        const int32_t s2L = src[4], s2R = src[5];
        const int32_t s3L = src[6], s3R = src[7];
        dstL[2] = dsp_fclamp_pm1((float)s2L * scaleL);
        dstR[2] = dsp_fclamp_pm1((float)s2R * scaleR);
        dstL[3] = dsp_fclamp_pm1((float)s3L * scaleL);
        dstR[3] = dsp_fclamp_pm1((float)s3R * scaleR);

        src  += 8;
        dstL += 4;
        dstR += 4;
    }
    for (; i < frames; ++i) {
        *dstL++ = dsp_fclamp_pm1((float)src[0] * scaleL);
        *dstR++ = dsp_fclamp_pm1((float)src[1] * scaleR);
        src += 2;
    }
}

static inline void dsp_pack_f32_to_i32(int32_t * DSP_RESTRICT dst,
                                       const float * DSP_RESTRICT srcL,
                                       const float * DSP_RESTRICT srcR,
                                       size_t frames) {
    size_t i = 0;
    for (; i + 4u <= frames; i += 4u) {
        const float l0 = srcL[0], r0 = srcR[0];
        const float l1 = srcL[1], r1 = srcR[1];
        dst[0] = dsp_sanitize_clamp_f32_to_i32(l0);
        dst[1] = dsp_sanitize_clamp_f32_to_i32(r0);
        dst[2] = dsp_sanitize_clamp_f32_to_i32(l1);
        dst[3] = dsp_sanitize_clamp_f32_to_i32(r1);

        const float l2 = srcL[2], r2 = srcR[2];
        const float l3 = srcL[3], r3 = srcR[3];
        dst[4] = dsp_sanitize_clamp_f32_to_i32(l2);
        dst[5] = dsp_sanitize_clamp_f32_to_i32(r2);
        dst[6] = dsp_sanitize_clamp_f32_to_i32(l3);
        dst[7] = dsp_sanitize_clamp_f32_to_i32(r3);

        srcL += 4;
        srcR += 4;
        dst  += 8;
    }
    for (; i < frames; ++i) {
        dst[0] = dsp_sanitize_clamp_f32_to_i32(*srcL++);
        dst[1] = dsp_sanitize_clamp_f32_to_i32(*srcR++);
        dst += 2;
    }
}

static inline void dsp_ring_pack_f32_to_i32(int32_t *writep,
                                            int32_t *buf_start,
                                            const int32_t *buf_wrap,
                                            const float *srcL,
                                            const float *srcR,
                                            size_t frames) {
    if (frames == 0) return;
    if (writep >= buf_wrap) {
        writep = buf_start;
    }
    const size_t cont_frames = (size_t)(buf_wrap - writep) >> 1;
    const size_t first = (frames <= cont_frames) ? frames : cont_frames;
    dsp_pack_f32_to_i32(writep, srcL, srcR, first);
    if (frames > first) {
        dsp_pack_f32_to_i32(buf_start, srcL + first, srcR + first, frames - first);
    }
}

class IirEQ {
public:
    IirEQ() {
        reset();
        constexpr float defaultFreqs[kEQBands] = {
            100.0f, 1000.0f, 10000.0f, 250.0f, 500.0f,
            2000.0f, 4000.0f, 8000.0f, 16000.0f, 50.0f
        };
        for (int b = 0; b < kEQBands; ++b) {
            _hasSecondStage[b] = false;
            _bandParams[b] = { defaultFreqs[b], 0.0f, 0.707f, true, EQ_FILTER_PEAK };
            _calcCoeffs(b, _bandParams[b]);
        }
    }

    void reset() {
        memset(_zL, 0, sizeof(_zL));
        memset(_zR, 0, sizeof(_zR));
    }

    void setSampleRate(uint32_t fs) {
        if (fs == 0 || fs == _sampleRate) return;
        _sampleRate = fs;
        for (int b = 0; b < kEQBands; ++b) {
            _calcCoeffs(b, _bandParams[b]);
        }
    }

    void setBand(int band, const EQBandParams& p) {
        if (band < 0 || band >= kEQBands) return;
        _bandParams[band] = p;
        _calcCoeffs(band, p);
        _enabled[band] = p.enabled;
    }

    void process(float* left, float* right, size_t frames, int numBands = kEQBands) {
        if (frames == 0 || numBands <= 0) return;
        if (numBands > kEQBands) numBands = kEQBands;
        for (int b = 0; b < numBands; ++b) {
            if (!_enabled[b]) continue;
#if defined(HAVE_ESP_DSP)
            dsps_biquad_f32(left,  left,  static_cast<int>(frames), _coeffs[b][0], _zL[b][0]);
            dsps_biquad_f32(right, right, static_cast<int>(frames), _coeffs[b][0], _zR[b][0]);
            _sanitizeState(_zL[b][0]);
            _sanitizeState(_zR[b][0]);
            if (_hasSecondStage[b]) {
                dsps_biquad_f32(left,  left,  static_cast<int>(frames), _coeffs[b][1], _zL[b][1]);
                dsps_biquad_f32(right, right, static_cast<int>(frames), _coeffs[b][1], _zR[b][1]);
                _sanitizeState(_zL[b][1]);
                _sanitizeState(_zR[b][1]);
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
    }

private:
    static inline void _sanitizeState(float* z) {
        if (!std::isfinite(z[0]) || !std::isfinite(z[1])) {
            z[0] = 0.0f;
            z[1] = 0.0f;
        } else {
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
        _sanitizeState(w);
    }

    uint32_t     _sampleRate{audio_rate::kDefaultRate};
    EQBandParams _bandParams[kEQBands]{};
    float _coeffs[kEQBands][2][5]{};
    float _zL[kEQBands][2][2]{};
    float _zR[kEQBands][2][2]{};
    bool  _hasSecondStage[kEQBands]{};
    bool  _enabled[kEQBands]{};

    void _calcCoeffs(int b, const EQBandParams& p) {
        const double Fs       = static_cast<double>(_sampleRate);
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

    void setSampleRate(uint32_t fs) {
        if (fs == 0 || fs == _sampleRate) return;
        _sampleRate = fs;
        _update(_params);
    }

    void setParams(const SubsonicParams& p) {
        _params = p;
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
    static inline void _sanitizeState(float* z) {
        if (!std::isfinite(z[0]) || !std::isfinite(z[1])) {
            z[0] = 0.0f;
            z[1] = 0.0f;
        } else {
            if (fabsf(z[0]) < 1e-15f) z[0] = 0.0f;
            if (fabsf(z[1]) < 1e-15f) z[1] = 0.0f;
        }
    }

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
        _sanitizeState(zL);
        _processStageChannel(right, frames, c, zR);
        _sanitizeState(zR);
    }

    uint32_t       _sampleRate{audio_rate::kDefaultRate};
    SubsonicParams _params{80.0f, 2, false};
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

        const double Fs   = static_cast<double>(_sampleRate);
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

class SpeakerCorrection {
public:
    static constexpr size_t kDelayBufSize = 8192;
    static_assert((kDelayBufSize & (kDelayBufSize - 1)) == 0, "kDelayBufSize must be a power of 2");

    SpeakerCorrection() {
#if defined(ESP_PLATFORM)
        _delayBufL = static_cast<float*>(heap_caps_calloc(kDelayBufSize, sizeof(float), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
        _delayBufR = static_cast<float*>(heap_caps_calloc(kDelayBufSize, sizeof(float), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
#endif
        if (!_delayBufL) _delayBufL = static_cast<float*>(calloc(kDelayBufSize, sizeof(float)));
        if (!_delayBufR) _delayBufR = static_cast<float*>(calloc(kDelayBufSize, sizeof(float)));
        reset();
        SubsonicParams defaultParams;
        setParams(defaultParams);
    }

    ~SpeakerCorrection() {
        free(_delayBufL);
        free(_delayBufR);
        _delayBufL = nullptr;
        _delayBufR = nullptr;
    }

    SpeakerCorrection(const SpeakerCorrection&) = delete;
    SpeakerCorrection& operator=(const SpeakerCorrection&) = delete;

    void reset() {
        _subsonic.reset();
        if (_delayBufL) memset(_delayBufL, 0, kDelayBufSize * sizeof(float));
        if (_delayBufR) memset(_delayBufR, 0, kDelayBufSize * sizeof(float));
        _delayWriteIdx = 0;
        memset(_hStateL, 0, sizeof(_hStateL));
        memset(_hStateR, 0, sizeof(_hStateR));
    }

    void setSampleRate(uint32_t fs) {
        if (fs == 0 || fs == _sampleRate) return;
        _sampleRate = fs;
        _subsonic.setSampleRate(fs);
        setParams(_params);
    }

    void setParams(const SubsonicParams& p) {
        _params = p;
        _enabled = p.enabled;
        _subsonic.setParams(p);

        float bal = p.balance;
        if (!std::isfinite(bal) || bal < -100.0f) bal = -100.0f;
        if (bal >  100.0f) bal =  100.0f;

        if (bal <= 0.0f) {
            float lDb = (-bal / 100.0f) * 6.0f;
            float rDb = (bal / 100.0f) * 24.0f;
            _gainL = powf(10.0f, lDb / 20.0f);
            _gainR = powf(10.0f, rDb / 20.0f);
        } else {
            float rDb = (bal / 100.0f) * 6.0f;
            float lDb = (-bal / 100.0f) * 24.0f;
            _gainL = powf(10.0f, lDb / 20.0f);
            _gainR = powf(10.0f, rDb / 20.0f);
        }

        const float Fs = static_cast<float>(_sampleRate);
        float dMsL = (p.delayMsL < -20.0f) ? -20.0f : ((p.delayMsL > 20.0f) ? 20.0f : p.delayMsL);
        float dMsR = (p.delayMsR < -20.0f) ? -20.0f : ((p.delayMsR > 20.0f) ? 20.0f : p.delayMsR);
        if (!std::isfinite(dMsL)) dMsL = 0.0f;
        if (!std::isfinite(dMsR)) dMsR = 0.0f;

        float minD = (dMsL < dMsR) ? dMsL : dMsR;
        float effL = (minD < 0.0f) ? (dMsL - minD) : dMsL;
        float effR = (minD < 0.0f) ? (dMsR - minD) : dMsR;

        const bool hadDelayL = (_delaySamplesL > 0.01f);
        const bool hadDelayR = (_delaySamplesR > 0.01f);

        _delaySamplesL = effL * (Fs / 1000.0f);
        _delaySamplesR = effR * (Fs / 1000.0f);

        constexpr float maxDelaySamples = static_cast<float>(kDelayBufSize - 2);
        if (_delaySamplesL > maxDelaySamples) _delaySamplesL = maxDelaySamples;
        if (_delaySamplesR > maxDelaySamples) _delaySamplesR = maxDelaySamples;

        const bool hasDelayL = (_delaySamplesL > 0.01f);
        const bool hasDelayR = (_delaySamplesR > 0.01f);
        if (!hadDelayL && hasDelayL && _delayBufL) {
            memset(_delayBufL, 0, kDelayBufSize * sizeof(float));
        }
        if (!hadDelayR && hasDelayR && _delayBufR) {
            memset(_delayBufR, 0, kDelayBufSize * sizeof(float));
        }
        if (!hadDelayL && !hadDelayR && (hasDelayL || hasDelayR)) {
            _delayWriteIdx = 0;
        }

        _phaseDegL = _quantizePhase(p.phaseL);
        _phaseDegR = _quantizePhase(p.phaseR);
    }

    void process(float* left, float* right, size_t frames) {
        if (!_enabled || frames == 0) return;

        if (_subsonic.isEnabled()) {
            _subsonic.process(left, right, frames);
        }

#if defined(HAVE_ESP_DSP)
        if (fabsf(_gainL - 1.0f) > 1e-4f) {
            dsps_mulc_f32(left,  left,  static_cast<int>(frames), _gainL, 1, 1);
        }
        if (fabsf(_gainR - 1.0f) > 1e-4f) {
            dsps_mulc_f32(right, right, static_cast<int>(frames), _gainR, 1, 1);
        }
#else
        if (fabsf(_gainL - 1.0f) > 1e-4f) {
            for (size_t i = 0; i < frames; ++i) {
                left[i] *= _gainL;
            }
        }
        if (fabsf(_gainR - 1.0f) > 1e-4f) {
            for (size_t i = 0; i < frames; ++i) {
                right[i] *= _gainR;
            }
        }
#endif

        const bool needDelayL = (_delaySamplesL > 0.01f);
        const bool needDelayR = (_delaySamplesR > 0.01f);

        if (needDelayL) {
            _processDelayChannel(left, frames, _delayBufL, _delayWriteIdx, _delaySamplesL);
        }
        if (needDelayR) {
            _processDelayChannel(right, frames, _delayBufR, _delayWriteIdx, _delaySamplesR);
        }
        if (needDelayL || needDelayR) {
            _delayWriteIdx = (_delayWriteIdx + frames) & (kDelayBufSize - 1);
        }

        _processPhaseChannel(left,  frames, _phaseDegL, _hStateL);
        _processPhaseChannel(right, frames, _phaseDegR, _hStateR);
    }

private:
    SubsonicFilter _subsonic;
    bool           _enabled{false};
    float          _gainL{1.0f};
    float          _gainR{1.0f};
    uint32_t       _sampleRate{audio_rate::kDefaultRate};
    SubsonicParams _params{};
    float          _delaySamplesL{0.0f};
    float          _delaySamplesR{0.0f};
    int16_t        _phaseDegL{0};
    int16_t        _phaseDegR{0};

    float*         _delayBufL{nullptr};
    float*         _delayBufR{nullptr};
    size_t         _delayWriteIdx{0};

    float          _hStateL[14]{};
    float          _hStateR[14]{};

    static inline int16_t _quantizePhase(int16_t deg) {
        if (deg < -157) return -180;
        if (deg < -112) return -135;
        if (deg < -67)  return -90;
        if (deg < -22)  return -45;
        if (deg <= 22)  return 0;
        if (deg <= 67)  return 45;
        if (deg <= 112) return 90;
        if (deg <= 157) return 135;
        return 180;
    }

    static inline bool _sanitizeHilbertState(float* state, size_t count) {
        bool fault = false;
        for (size_t k = 0; k < count; ++k) {
            if (!std::isfinite(state[k])) {
                fault = true;
                break;
            }
        }
        if (fault) {
            memset(state, 0, count * sizeof(float));
            return true;
        }
        for (size_t k = 0; k < count; ++k) {
            if (fabsf(state[k]) < 1e-15f) {
                state[k] = 0.0f;
            }
        }
        return false;
    }

    static inline void _processDelayChannel(float* buf, size_t frames, float* delayBuf,
                                            size_t writeIdx, float delaySamples) {
        constexpr size_t kMask = kDelayBufSize - 1;
        const size_t delayInt = static_cast<size_t>(delaySamples);
        const float  frac     = delaySamples - static_cast<float>(delayInt);
        const float  invFrac  = 1.0f - frac;

        for (size_t i = 0; i < frames; ++i) {
            delayBuf[writeIdx] = buf[i];
            const size_t idxA = (writeIdx - delayInt) & kMask;
            const size_t idxB = (idxA - 1) & kMask;
            buf[i] = delayBuf[idxA] * invFrac + delayBuf[idxB] * frac;
            writeIdx = (writeIdx + 1) & kMask;
        }
    }

    static inline float _ap1(float x, float c, float& s) {
        const float y = c * x + s;
        s = x - c * y;
        return y;
    }

    static inline void _processPhaseChannel(float* buf, size_t frames, int16_t deg,
                                            float* state) {
        if (deg == 0) return;
        if (deg == 180 || deg == -180) {
#if defined(HAVE_ESP_DSP)
            dsps_mulc_f32(buf, buf, static_cast<int>(frames), -1.0f, 1, 1);
#else
            for (size_t i = 0; i < frames; ++i) {
                buf[i] = -buf[i];
            }
#endif
            return;
        }

        static constexpr float kSqrtHalf = 0.70710678f;
        float wI = 0.0f;
        float wQ = 0.0f;
        switch (deg) {
            case 90:   wI =  0.0f;       wQ = -1.0f;       break;
            case -90:  wI =  0.0f;       wQ =  1.0f;       break;
            case 45:   wI =  kSqrtHalf;  wQ = -kSqrtHalf;  break;
            case -45:  wI =  kSqrtHalf;  wQ =  kSqrtHalf;  break;
            case 135:  wI = -kSqrtHalf;  wQ = -kSqrtHalf;  break;
            case -135: wI = -kSqrtHalf;  wQ =  kSqrtHalf;  break;
            default:   return;
        }

        static constexpr float kCoeffsI[7] = {
            -0.99972040f, -0.99822995f, -0.98883874f, -0.93133342f, -0.63232144f, 0.17593976f, 0.85340188f
        };
        static constexpr float kCoeffsQ[7] = {
            -0.99992052f, -0.99929639f, -0.99555072f, -0.97214268f, -0.83572099f, -0.27635952f, 0.56445904f
        };

        for (size_t i = 0; i < frames; ++i) {
            float xI = buf[i];
            float xQ = xI;
            for (int k = 0; k < 7; ++k) {
                xI = _ap1(xI, kCoeffsI[k], state[k]);
                xQ = _ap1(xQ, kCoeffsQ[k], state[7 + k]);
            }
            buf[i] = wI * xI + wQ * xQ;
        }

        _sanitizeHilbertState(state, 14);
    }
};

class Compander {
public:
    Compander() {
        _params = { -18.0f, 4.0f, 5.0f, 150.0f, 0.0f, true };
        _update(_params);
    }

    void reset() {
        _envRMS = 0.0f;
        _currentGain = 1.0f;
    }

    void setSampleRate(uint32_t fs) {
        if (fs == 0 || fs == _sampleRate) return;
        _sampleRate = fs;
        _update(_params);
    }

    void setParams(const CompanderParams& p) {
        _params = p;
        _update(p);
    }

    void process(float* left, float* right, size_t frames) {
        if (!_enabled || frames == 0) return;

        float sumL = 0.0f, sumR = 0.0f;
#if defined(HAVE_ESP_DSP)
        dsps_dotprod_f32(left,  left,  &sumL, static_cast<int>(frames));
        dsps_dotprod_f32(right, right, &sumR, static_cast<int>(frames));
#else
        for (size_t i = 0; i < frames; ++i) {
            sumL += left[i]  * left[i];
            sumR += right[i] * right[i];
        }
#endif
        if (!std::isfinite(sumL) || sumL < 0.0f || !std::isfinite(sumR) || sumR < 0.0f) {
            sumL = 0.0f;
            sumR = 0.0f;
            for (size_t i = 0; i < frames; ++i) {
                if (!std::isfinite(left[i]))  left[i]  = 0.0f;
                if (!std::isfinite(right[i])) right[i] = 0.0f;
                sumL += left[i]  * left[i];
                sumR += right[i] * right[i];
            }
        }

        const float rmsL = sqrtf(sumL / static_cast<float>(frames));
        const float rmsR = sqrtf(sumR / static_cast<float>(frames));
        float rms  = (rmsL > rmsR) ? rmsL : rmsR;
        if (!std::isfinite(rms)) rms = 0.0f;

        if (!std::isfinite(_envRMS)) _envRMS = 0.0f;
        if (!std::isfinite(_currentGain) || _currentGain < 0.0f) _currentGain = 1.0f;
        float attackCoeff  = _attackCoeff;
        float releaseCoeff = _releaseCoeff;
        if (frames != _coeffFrames) {
            const float blockRate = static_cast<float>(_sampleRate) / static_cast<float>(frames);
            attackCoeff  = expf(-2.2f / (blockRate * (_attackMs * 0.001f)));
            releaseCoeff = expf(-2.2f / (blockRate * (_releaseMs * 0.001f)));
        }
        const float coeff = (rms > _envRMS) ? attackCoeff : releaseCoeff;
        _envRMS = _envRMS * coeff + rms * (1.0f - coeff);
        if (fabsf(_envRMS) < 1e-15f) _envRMS = 0.0f;

        const float levelDb = 20.0f * log10f(_envRMS + 1e-9f);
        float gainDb = 0.0f;
        if (levelDb > _thresholdDb) {
            gainDb = (_thresholdDb - levelDb) * (1.0f - 1.0f / _ratio);
        }
        gainDb += _makeupGainDb;

        float targetGain = powf(10.0f, gainDb / 20.0f);
        if (!std::isfinite(targetGain) || targetGain < 0.0f) targetGain = 1.0f;
        if (targetGain > 16.0f) targetGain = 16.0f;

        if (fabsf(targetGain - _currentGain) < 1e-5f) {
            _currentGain = targetGain;
            if (fabsf(_currentGain - 1.0f) > 1e-4f) {
#if defined(HAVE_ESP_DSP)
                dsps_mulc_f32(left,  left,  static_cast<int>(frames), _currentGain, 1, 1);
                dsps_mulc_f32(right, right, static_cast<int>(frames), _currentGain, 1, 1);
#else
                for (size_t i = 0; i < frames; ++i) {
                    left[i]  *= _currentGain;
                    right[i] *= _currentGain;
                }
#endif
            }
        } else {
            const float startGain = _currentGain;
            const float gainStep  = (targetGain - startGain) / static_cast<float>(frames);
            const float step4     = gainStep * 4.0f;

            float g0 = startGain + gainStep;
            float g1 = startGain + gainStep * 2.0f;
            float g2 = startGain + gainStep * 3.0f;
            float g3 = startGain + step4;

            size_t i = 0;
            for (; i + 3 < frames; i += 4) {
                left[i]      *= g0;
                left[i + 1]  *= g1;
                left[i + 2]  *= g2;
                left[i + 3]  *= g3;

                right[i]     *= g0;
                right[i + 1] *= g1;
                right[i + 2] *= g2;
                right[i + 3] *= g3;

                g0 += step4;
                g1 += step4;
                g2 += step4;
                g3 += step4;
            }
            for (; i < frames; ++i) {
                left[i]  *= g0;
                right[i] *= g0;
                g0 += gainStep;
            }
            _currentGain = startGain + gainStep * static_cast<float>(frames);
        }
    }

private:
    uint32_t _sampleRate{audio_rate::kDefaultRate};
    size_t   _coeffFrames{256};
    CompanderParams _params{};
    bool  _enabled{true};
    float _thresholdDb{-18.f};
    float _ratio{4.f};
    float _makeupGainDb{0.f};
    float _attackMs{5.0f};
    float _releaseMs{150.0f};
    float _attackCoeff{0.f};
    float _releaseCoeff{0.f};
    float _envRMS{0.f};
    float _currentGain{1.0f};

    void _update(const CompanderParams& p) {
        _enabled      = p.enabled;
        const float rawThresh = std::isfinite(p.threshold)  ? p.threshold  : -18.0f;
        const float rawRatio  = std::isfinite(p.ratio)      ? p.ratio      : 4.0f;
        const float rawMakeup = std::isfinite(p.makeupGain) ? p.makeupGain : 0.0f;
        const float rawAtt    = std::isfinite(p.attackMs)   ? p.attackMs   : 5.0f;
        const float rawRel    = std::isfinite(p.releaseMs)  ? p.releaseMs  : 150.0f;

        _thresholdDb  = (rawThresh < -60.0f) ? -60.0f : ((rawThresh > 0.0f) ? 0.0f : rawThresh);
        _ratio        = (rawRatio < 1.0f) ? 1.0f : ((rawRatio > 20.0f) ? 20.0f : rawRatio);
        _makeupGainDb = (rawMakeup < -24.0f) ? -24.0f : ((rawMakeup > 24.0f) ? 24.0f : rawMakeup);

        _attackMs  = (rawAtt < 0.1f) ? 0.1f : ((rawAtt > 500.0f) ? 500.0f : rawAtt);
        _releaseMs = (rawRel < 10.0f) ? 10.0f : ((rawRel > 2000.0f) ? 2000.0f : rawRel);

        _coeffFrames = audio_rate::blockFramesForRate(_sampleRate);
        const float F_block = static_cast<float>(_sampleRate) / static_cast<float>(_coeffFrames);
        auto tc = [&](float ms) -> float {
            float sec = ms / 1000.0f;
            return expf(-2.2f / (F_block * sec));
        };
        _attackCoeff  = tc(_attackMs);
        _releaseCoeff = tc(_releaseMs);
    }
};

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

            uint32_t uL = 0, uR = 0;
            std::memcpy(&uL, &sL, sizeof(uL));
            std::memcpy(&uR, &sR, sizeof(uR));
            uL &= 0x7FFFFFFFu;
            uR &= 0x7FFFFFFFu;
            const uint32_t uMax = (uL > uR) ? uL : uR;
            if (uMax <= lowerLinBits) continue;

            if (uL >= 0x7F800000u) { sL = 0.0f; left[i]  = 0.0f; }
            if (uR >= 0x7F800000u) { sR = 0.0f; right[i] = 0.0f; }

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

#if defined(ESP_PLATFORM) || defined(ARDUINO_ARCH_ESP32)

class IDSPModule {
public:
    virtual ~IDSPModule() = default;
    virtual DspModuleId moduleId() const = 0;
    virtual const char* id() const = 0;
    virtual const char* name() const = 0;
    virtual void process(float* left, float* right, size_t frames) = 0;
    virtual void syncParams(ParamStore& params) = 0;
    virtual void setSampleRate(uint32_t) {}
    virtual void reset() {}
};

class EQModule : public IDSPModule {
public:
    DspModuleId moduleId() const override { return DspModuleId::Eq; }
    const char* id() const override { return "eq"; }
    const char* name() const override { return "Parametric EQ"; }

    void syncParams(ParamStore& params) override {
        EQBandParams eqBands[kEQBands];
        if (params.getEQStateIfChanged(eqBands, _localEqCount, _localEqEnabled, _localPreampDb, _localEqVersion)) {
            _localPreampGain = powf(10.0f, _localPreampDb / 20.0f);
            for (int b = 0; b < _localEqCount; ++b) {
                _eq.setBand(b, eqBands[b]);
            }
        }
    }

    void process(float* left, float* right, size_t frames) override {
        if (!_localEqEnabled || frames == 0) return;
        if (fabsf(_localPreampGain - 1.0f) > 0.001f) {
#if defined(HAVE_ESP_DSP)
            dsps_mulc_f32(left,  left,  static_cast<int>(frames), _localPreampGain, 1, 1);
            dsps_mulc_f32(right, right, static_cast<int>(frames), _localPreampGain, 1, 1);
#else
            for (size_t i = 0; i < frames; ++i) {
                left[i]  *= _localPreampGain;
                right[i] *= _localPreampGain;
            }
#endif
        }
        _eq.process(left, right, frames, _localEqCount);
    }

    void setSampleRate(uint32_t fs) override {
        _eq.setSampleRate(fs);
    }

    void reset() override {
        _eq.reset();
    }

private:
    IirEQ    _eq;
    uint32_t _localEqVersion{0};
    int      _localEqCount{5};
    bool     _localEqEnabled{false};
    float    _localPreampDb{0.0f};
    float    _localPreampGain{1.0f};
};

class SubsonicModule : public IDSPModule {
public:
    DspModuleId moduleId() const override { return DspModuleId::Subsonic; }
    const char* id() const override { return "subsonic"; }
    const char* name() const override { return "Speakers Correction"; }

    void syncParams(ParamStore& params) override {
        SubsonicParams sub;
        if (params.getSubsonicIfChanged(sub, _localSubVersion)) {
            _speakers.setParams(sub);
        }
    }

    void process(float* left, float* right, size_t frames) override {
        _speakers.process(left, right, frames);
    }

    void setSampleRate(uint32_t fs) override {
        _speakers.setSampleRate(fs);
    }

    void reset() override {
        _speakers.reset();
    }

private:
    SpeakerCorrection _speakers;
    uint32_t          _localSubVersion{0};
};

class CompanderModule : public IDSPModule {
public:
    DspModuleId moduleId() const override { return DspModuleId::Compander; }
    const char* id() const override { return "comp"; }
    const char* name() const override { return "Compressor"; }

    void syncParams(ParamStore& params) override {
        CompanderParams comp;
        if (params.getCompanderIfChanged(comp, _localCompVersion)) {
            _comp.setParams(comp);
        }
    }

    void process(float* left, float* right, size_t frames) override {
        _comp.process(left, right, frames);
    }

    void setSampleRate(uint32_t fs) override {
        _comp.setSampleRate(fs);
    }

    void reset() override {
        _comp.reset();
    }

private:
    Compander _comp;
    uint32_t  _localCompVersion{0};
};

class LimiterModule : public IDSPModule {
public:
    DspModuleId moduleId() const override { return DspModuleId::Limiter; }
    const char* id() const override { return "lim"; }
    const char* name() const override { return "Limiter"; }

    void syncParams(ParamStore& params) override {
        LimiterParams lim;
        if (params.getLimiterIfChanged(lim, _localLimVersion)) {
            _limiter.setParams(lim);
        }
    }

    void process(float* left, float* right, size_t frames) override {
        _limiter.process(left, right, frames);
    }

    void reset() override {
        _limiter.reset();
    }

private:
    SoftLimiter _limiter;
    uint32_t    _localLimVersion{0};
};

class DSPPipeline {
public:
    static constexpr size_t kMaxModules = 8;

    explicit DSPPipeline(ParamStore& params)
        : _params(params)
    {
        _registerModule(&_eqModule);
        _registerModule(&_subsonicModule);
        _registerModule(&_compModule);
        _registerModule(&_limiterModule);

        _updateOrder("eq,subsonic,comp,lim");
    }

    void setSampleRate(uint32_t fs) {
        if (fs == 0) return;
        for (size_t i = 0; i < _registeredCount; ++i) {
            if (_registeredModules[i]) {
                _registeredModules[i]->setSampleRate(fs);
                _registeredModules[i]->reset();
            }
        }
    }

    void process(float* left, float* right, size_t frames) {
        _syncParams();

        for (size_t i = 0; i < _activeModuleCount; ++i) {
            if (_activeModules[i]) {
                _activeModules[i]->process(left, right, frames);
            }
        }
    }

private:
    ParamStore&       _params;

    EQModule          _eqModule;
    SubsonicModule    _subsonicModule;
    CompanderModule   _compModule;
    LimiterModule     _limiterModule;

    IDSPModule*       _registeredModules[kMaxModules]{nullptr};
    size_t            _registeredCount{0};

    IDSPModule*       _activeModules[kMaxModules]{nullptr};
    size_t            _activeModuleCount{0};

    uint32_t          _localOrderVersion{0};
    uint32_t          _syncCounter{kSyncInterval - 1};
    static constexpr uint32_t kSyncInterval = 4;

    void _registerModule(IDSPModule* mod) {
        if (!mod || _registeredCount >= kMaxModules) return;
        _registeredModules[_registeredCount++] = mod;
    }

    IDSPModule* _findModule(DspModuleId id) {
        if (id == DspModuleId::Unknown) return nullptr;
        for (size_t i = 0; i < _registeredCount; ++i) {
            if (_registeredModules[i] && _registeredModules[i]->moduleId() == id) {
                return _registeredModules[i];
            }
        }
        return nullptr;
    }

    void _updateOrder(const char* orderStr) {
        _activeModuleCount = 0;
        if (!orderStr || strlen(orderStr) == 0) return;

        char buf[64];
        strncpy(buf, orderStr, sizeof(buf) - 1);
        buf[sizeof(buf) - 1] = '\0';

        char* savePtr = nullptr;
        char* token = strtok_r(buf, ",", &savePtr);
        while (token && _activeModuleCount < kMaxModules) {
            while (*token == ' ') token++;
            if (*token != '\0') {
                char* end = token + strlen(token) - 1;
                while (end > token && *end == ' ') { *end = '\0'; end--; }

                IDSPModule* m = _findModule(parseDspModuleId(token));
                if (m) {
                    bool dup = false;
                    for (size_t k = 0; k < _activeModuleCount; ++k) {
                        if (_activeModules[k] == m) { dup = true; break; }
                    }
                    if (!dup) {
                        _activeModules[_activeModuleCount++] = m;
                    }
                }
            }
            token = strtok_r(nullptr, ",", &savePtr);
        }

        if (_activeModuleCount == 0) {
            for (size_t i = 0; i < _registeredCount && i < kMaxModules; ++i) {
                _activeModules[i] = _registeredModules[i];
            }
            _activeModuleCount = _registeredCount;
        }
    }

    void _syncParams() {
        if (++_syncCounter < kSyncInterval) return;
        _syncCounter = 0;

        char orderBuf[64];
        if (_params.getModuleOrderIfChanged(orderBuf, sizeof(orderBuf), _localOrderVersion)) {
            _updateOrder(orderBuf);
        }

        for (size_t i = 0; i < _registeredCount; ++i) {
            if (_registeredModules[i]) {
                _registeredModules[i]->syncParams(_params);
            }
        }
    }
};

#endif
