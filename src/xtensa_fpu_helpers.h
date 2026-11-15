#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <float.h>

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

// Clamp float sample to [-1.0f, 1.0f]
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

// Sanitize, clamp, and convert float to i32
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

// 16-byte aligned staging buffer allocation
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

// Unpack interleaved i32 stereo to planar f32
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

// Pack planar f32 to interleaved i32
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

// Pack planar f32 into ring buffer
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
