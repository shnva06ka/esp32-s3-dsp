#define DR_FLAC_IMPLEMENTATION
#define DR_FLAC_NO_STDIO
#define DR_FLAC_NO_CRC
#define DR_FLAC_BUFFER_SIZE 4096

#include "dr_flac.h"
#include "lms_codecs.h"

// === src/lms_flac.c ===


#ifdef ESP_PLATFORM
#include <esp_heap_caps.h>
static void *lms_flac_malloc(size_t sz, void *user) {
    (void)user;
    void *p = heap_caps_malloc(sz, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    return p ? p : heap_caps_malloc(sz, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
}
static void *lms_flac_realloc(void *ptr, size_t sz, void *user) {
    (void)user;
    void *p = heap_caps_realloc(ptr, sz, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    return p ? p : heap_caps_realloc(ptr, sz, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
}
static void lms_flac_free(void *ptr, void *user) {
    (void)user;
    heap_caps_free(ptr);
}
static const drflac_allocation_callbacks kFlacAlloc = {
    .pUserData = NULL,
    .onMalloc  = lms_flac_malloc,
    .onRealloc = lms_flac_realloc,
    .onFree    = lms_flac_free
};
static const drflac_allocation_callbacks *kFlacAllocPtr = &kFlacAlloc;
#else
static const drflac_allocation_callbacks *kFlacAllocPtr = NULL;
#endif

struct lms_flac {
    drflac *flac;
};

static drflac_bool32 lms_flac_seek(void *user, int offset, drflac_seek_origin origin) {
    (void)user; (void)offset; (void)origin;
    return DRFLAC_FALSE;   /* sequential stream: seeking is never possible */
}

lms_flac_t *lms_flac_open(lms_flac_read_fn read_fn, void *user,
                          unsigned *rate, unsigned *channels, unsigned *bits) {
    drflac *d = drflac_open((drflac_read_proc)read_fn, lms_flac_seek, NULL, user, kFlacAllocPtr);
    if (!d) return NULL;
    if (d->channels == 0 || d->channels > 2) {
        drflac_close(d);
        return NULL;
    }
#ifdef ESP_PLATFORM
    lms_flac_t *f = (lms_flac_t *)lms_flac_malloc(sizeof(*f), NULL);
#else
    lms_flac_t *f = (lms_flac_t *)drflac__malloc_default(sizeof(*f), NULL);
#endif
    if (!f) {
        drflac_close(d);
        return NULL;
    }
    f->flac = d;
    if (rate)     *rate = d->sampleRate;
    if (channels) *channels = d->channels;
    if (bits)     *bits = d->bitsPerSample;
    return f;
}

size_t lms_flac_read(lms_flac_t *f, int32_t *out, size_t frames) {
    if (!f || !f->flac || !out || frames == 0) return 0;
    return (size_t)drflac_read_pcm_frames_s32(f->flac, (drflac_uint64)frames, (drflac_int32 *)out);
}

void lms_flac_close(lms_flac_t *f) {
    if (!f) return;
    if (f->flac) drflac_close(f->flac);
#ifdef ESP_PLATFORM
    lms_flac_free(f, NULL);
#else
    drflac__free_default(f, NULL);
#endif
}

// === src/lms_resample.c ===
/*
 * Polyphase windowed-sinc sample-rate converter (lms_resample.h)
 * License: see the LICENSE file at the repository root
 */

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <esp_heap_caps.h>
#include <esp_timer.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>

extern void console_log(const char* buf);
static void rs_log(const char* fmt, ...) {
    char buf[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    console_log(buf);
}

#define RS_TAG         "RESAMPLE"
#define RS_TAPS        24
#define RS_MAX_PHASES  64
#define RS_KAISER_B    6.5f

#ifdef __cplusplus
static_assert((RS_TAPS > 0) && ((RS_TAPS % 4) == 0), "RS_TAPS must be a multiple of 4");
#else
_Static_assert((RS_TAPS > 0) && ((RS_TAPS % 4) == 0), "RS_TAPS must be a multiple of 4");
#endif

struct resampler {
    unsigned  L, M, P;
    uint32_t  step_int;
    uint32_t  step_rem;
    uint32_t  phase_mul;
    int       bypass;
    uint32_t  pos;            /* output position in 1/L input-sample units, relative to first new sample */
    float    *coef;           /* [P][RS_TAPS] aligned to 16 bytes */
    float    *histL, *histR;  /* RS_TAPS-1 previous input samples */
    float    *bufL,  *bufR;   /* RS_TAPS-1 + RS_MAX_IN working buffer */

    /* Memory tracking */
    uint32_t  in_rate;
    uint32_t  out_rate;
    uint8_t   is_psram;
    uint32_t  alloc_failures;

    /* Telemetry metrics (concurrency-safe via spinlock + seq counter) */
    portMUX_TYPE stats_mux;
    volatile uint32_t seq;
    uint64_t  total_calls;
    uint64_t  in_frames_total;
    uint64_t  out_frames_total;
    uint32_t  last_duration_us;
    uint32_t  max_duration_us;
    uint32_t  recent_worst_us;
    uint32_t  recent_worst_reset_ms;
    uint64_t  total_duration_us;
    int32_t   frame_mismatch;

    /* Buffer & flow metrics */
    uint32_t  dropped_frames;
    uint32_t  underruns;
    uint32_t  overruns;
    uint32_t  buf_occupancy_frames;
    uint32_t  buf_capacity_frames;

    /* CPU load estimation */
    int64_t   last_calc_time_us;
    uint32_t  window_proc_time_us;
    float     estimated_cpu_pct;
};

static unsigned gcd_u(unsigned a, unsigned b) {
    while (b) { unsigned t = a % b; a = b; b = t; }
    return a;
}

static float bessel_i0(float x) {
    float sum = 1.0f, term = 1.0f, q = x * x * 0.25f;
    for (int k = 1; k < 50; ++k) {
        term *= q / ((float)k * (float)k);
        sum += term;
        if (term < 1e-8f * sum) break;
    }
    return sum;
}

/* Internal RAM staging with PSRAM fallback */
static void *rs_alloc(size_t bytes, int *used_psram) {
    size_t aligned_bytes = (bytes + 15) & ~((size_t)15);
    void *p = heap_caps_aligned_alloc(16, aligned_bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!p) {
        p = heap_caps_malloc(aligned_bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    }
    if (p) {
        if (used_psram && *used_psram == 0) *used_psram = 0;
        return p;
    }

    /* Fallback to PSRAM */
    p = heap_caps_aligned_alloc(16, aligned_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!p) {
        p = heap_caps_malloc(aligned_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    }
    if (p && used_psram) {
        *used_psram = 1;
    }
    return p;
}

/* Portable C reference dot product */
static inline void rs_dotprod_stereo_c(const float *h, const float *xl, const float *xr,
                                       float *outL, float *outR) {
    float accL0 = 0.0f, accL1 = 0.0f, accL2 = 0.0f, accL3 = 0.0f;
    float accR0 = 0.0f, accR1 = 0.0f, accR2 = 0.0f, accR3 = 0.0f;
    for (int j = 0; j < RS_TAPS; j += 4) {
        float h0 = h[j + 0], h1 = h[j + 1], h2 = h[j + 2], h3 = h[j + 3];
        accL0 += h0 * xl[j + 0];
        accR0 += h0 * xr[j + 0];
        accL1 += h1 * xl[j + 1];
        accR1 += h1 * xr[j + 1];
        accL2 += h2 * xl[j + 2];
        accR2 += h2 * xr[j + 2];
        accL3 += h3 * xl[j + 3];
        accR3 += h3 * xr[j + 3];
    }
    *outL = (accL0 + accL1) + (accL2 + accL3);
    *outR = (accR0 + accR1) + (accR2 + accR3);
}

/* Xtensa pipelined dot product */
static inline void rs_dotprod_stereo_unrolled(const float *h, const float *xl, const float *xr,
                                              float *outL, float *outR) {
#if defined(__XTENSA__) && defined(CONFIG_IDF_TARGET_ESP32S3)
    const float *hp  = h;
    const float *xlp = xl;
    const float *xrp = xr;
    uint32_t cnt = (uint32_t)(RS_TAPS / 4);

    float accL0 = 0.0f, accL1 = 0.0f, accL2 = 0.0f, accL3 = 0.0f;
    float accR0 = 0.0f, accR1 = 0.0f, accR2 = 0.0f, accR3 = 0.0f;
    float th0, th1, tL, tR;

    __asm__ __volatile__ (
        "loopnez %[cnt], 1f\n\t"
        "lsi     %[th0],   %[hp],  0\n\t"
        "lsi     %[tL],    %[xlp], 0\n\t"
        "lsi     %[tR],    %[xrp], 0\n\t"
        "lsi     %[th1],   %[hp],  4\n\t"
        "madd.s  %[accL0], %[th0], %[tL]\n\t"
        "madd.s  %[accR0], %[th0], %[tR]\n\t"
        "lsi     %[tL],    %[xlp], 4\n\t"
        "lsi     %[tR],    %[xrp], 4\n\t"
        "lsi     %[th0],   %[hp],  8\n\t"
        "madd.s  %[accL1], %[th1], %[tL]\n\t"
        "madd.s  %[accR1], %[th1], %[tR]\n\t"
        "lsi     %[tL],    %[xlp], 8\n\t"
        "lsi     %[tR],    %[xrp], 8\n\t"
        "lsi     %[th1],   %[hp],  12\n\t"
        "madd.s  %[accL2], %[th0], %[tL]\n\t"
        "madd.s  %[accR2], %[th0], %[tR]\n\t"
        "lsi     %[tL],    %[xlp], 12\n\t"
        "lsi     %[tR],    %[xrp], 12\n\t"
        "addi    %[hp],    %[hp],  16\n\t"
        "addi    %[xlp],   %[xlp], 16\n\t"
        "addi    %[xrp],   %[xrp], 16\n\t"
        "madd.s  %[accL3], %[th1], %[tL]\n\t"
        "madd.s  %[accR3], %[th1], %[tR]\n\t"
        "1:\n\t"
        : [hp]    "+&r" (hp),
          [xlp]   "+&r" (xlp),
          [xrp]   "+&r" (xrp),
          [accL0] "+&f" (accL0),
          [accL1] "+&f" (accL1),
          [accL2] "+&f" (accL2),
          [accL3] "+&f" (accL3),
          [accR0] "+&f" (accR0),
          [accR1] "+&f" (accR1),
          [accR2] "+&f" (accR2),
          [accR3] "+&f" (accR3),
          [th0]   "=&f" (th0),
          [th1]   "=&f" (th1),
          [tL]    "=&f" (tL),
          [tR]    "=&f" (tR)
        : [cnt]   "r" (cnt),
          "m" (*(const float (*)[RS_TAPS])h),
          "m" (*(const float (*)[RS_TAPS])xl),
          "m" (*(const float (*)[RS_TAPS])xr)
    );

    *outL = (accL0 + accL1) + (accL2 + accL3);
    *outR = (accR0 + accR1) + (accR2 + accR3);
#else
    rs_dotprod_stereo_c(h, xl, xr, outL, outR);
#endif
}

void resampler_destroy(resampler_t *r) {
    if (!r) return;
    if (r->coef)  heap_caps_free(r->coef);
    if (r->histL) heap_caps_free(r->histL);
    if (r->histR) heap_caps_free(r->histR);
    if (r->bufL)  heap_caps_free(r->bufL);
    if (r->bufR)  heap_caps_free(r->bufR);
    free(r);
}

void resampler_reset(resampler_t *r) {
    if (!r) return;
    r->pos = 0;
    if (r->histL) memset(r->histL, 0, (RS_TAPS - 1) * sizeof(float));
    if (r->histR) memset(r->histR, 0, (RS_TAPS - 1) * sizeof(float));
}

resampler_t *resampler_create(unsigned in_rate, unsigned out_rate) {
    if (in_rate < 8000 || in_rate > 384000 || out_rate < 8000 || out_rate > 384000) return NULL;

    resampler_t *r = (resampler_t *)calloc(1, sizeof(*r));
    if (!r) return NULL;

    r->in_rate  = in_rate;
    r->out_rate = out_rate;
    portMUX_INITIALIZE(&r->stats_mux);

    unsigned g = gcd_u(in_rate, out_rate);
    r->L = out_rate / g;
    r->M = in_rate / g;
    r->bypass = (in_rate == out_rate);

    if (r->bypass) {
        r->last_calc_time_us = esp_timer_get_time();
        ESP_LOGI(RS_TAG, "Resampler %u -> %u Hz: Passthrough bypass active", in_rate, out_rate);
        return r;
    }

    r->P = (r->L <= RS_MAX_PHASES) ? r->L : RS_MAX_PHASES;
    r->step_int  = r->M / r->L;
    r->step_rem  = r->M % r->L;
    r->phase_mul = (r->P == r->L) ? 0u : (uint32_t)((((uint64_t)r->P << 20) + r->L - 1u) / r->L);

    int used_psram = 0;
    const size_t ncoef = (size_t)r->P * RS_TAPS;
    r->coef  = (float *)rs_alloc(ncoef * sizeof(float), &used_psram);
    r->histL = (float *)rs_alloc((RS_TAPS - 1) * sizeof(float), &used_psram);
    r->histR = (float *)rs_alloc((RS_TAPS - 1) * sizeof(float), &used_psram);
    r->bufL  = (float *)rs_alloc((RS_TAPS - 1 + RS_MAX_IN) * sizeof(float), &used_psram);
    r->bufR  = (float *)rs_alloc((RS_TAPS - 1 + RS_MAX_IN) * sizeof(float), &used_psram);

    if (!r->coef || !r->histL || !r->histR || !r->bufL || !r->bufR) {
        ESP_LOGE(RS_TAG, "Memory allocation failed for %u -> %u Hz resampler", in_rate, out_rate);
        resampler_destroy(r);
        return NULL;
    }

    r->is_psram = (uint8_t)used_psram;
    ESP_LOGI(RS_TAG, "Resampler %u -> %u Hz created: L=%u M=%u P=%u | Memory: %s",
             in_rate, out_rate, r->L, r->M, r->P,
             r->is_psram ? "PSRAM fallback" : "Internal DRAM (16-byte aligned)");

    const float min_rate = (float)(in_rate < out_rate ? in_rate : out_rate);
    const float fc_hz    = 0.5f * min_rate * 0.92f;
    const float fc_rel   = fc_hz / ((float)r->P * (float)in_rate);

    const int   N      = (int)ncoef;
    const float c      = 0.5f * (float)(N - 1);
    const float i0b    = bessel_i0(RS_KAISER_B);
    const float two_fc = 2.0f * fc_rel;

    for (int n = 0; n < N; ++n) {
        float x   = (float)n - c;
        float arg = (float)M_PI * two_fc * x;
        float s   = (fabsf(arg) < 1e-6f) ? 1.0f : sinf(arg) / arg;
        float rr  = x / c;
        float w   = bessel_i0(RS_KAISER_B * sqrtf(fmaxf(0.0f, 1.0f - rr * rr))) / i0b;
        float h   = two_fc * s * w;
        unsigned phase = (unsigned)n % r->P;
        unsigned j     = (unsigned)n / r->P;
        r->coef[(size_t)phase * RS_TAPS + (RS_TAPS - 1 - j)] = h;
    }

    /* Normalize phase DC gain */
    for (unsigned ph = 0; ph < r->P; ++ph) {
        float *row = r->coef + (size_t)ph * RS_TAPS;
        double sum = 0.0;
        for (unsigned j = 0; j < RS_TAPS; ++j) sum += row[j];
        if (fabs(sum) > 1e-9) {
            float inv = (float)(1.0 / sum);
            for (unsigned j = 0; j < RS_TAPS; ++j) row[j] *= inv;
        }
    }

    resampler_reset(r);
    r->last_calc_time_us = esp_timer_get_time();
    return r;
}

int resampler_is_passthrough(const resampler_t *r) {
    return r ? r->bypass : 1;
}

size_t resampler_max_out(const resampler_t *r, size_t n_in) {
    if (!r || r->bypass) return n_in;
    return (size_t)(((uint64_t)n_in * r->L + r->M - 1) / r->M + 2);
}

size_t resampler_in_for_out(const resampler_t *r, size_t out_frames) {
    if (!r || r->bypass) return out_frames;
    if (out_frames == 0) return 0;
    return (size_t)(((uint64_t)out_frames * (uint64_t)r->M) / r->L);
}

size_t resampler_process(resampler_t *r,
                         const float *inL, const float *inR, size_t n_in,
                         float *outL, float *outR, size_t max_out) {
    if (!r || !inL || !inR || !outL || !outR || n_in == 0 || max_out == 0) return 0;
    if (r->bypass) {
        size_t n = (n_in < max_out) ? n_in : max_out;
        memcpy(outL, inL, n * sizeof(float));
        memcpy(outR, inR, n * sizeof(float));
        portENTER_CRITICAL(&r->stats_mux);
        r->seq++;
        r->total_calls++;
        r->in_frames_total += n;
        r->out_frames_total += n;
        r->seq++;
        portEXIT_CRITICAL(&r->stats_mux);
        return n;
    }

    int64_t t0 = esp_timer_get_time();
    if (!r->coef || !r->histL || !r->histR || !r->bufL || !r->bufR) {
        return 0;
    }

    if (n_in > RS_MAX_IN) n_in = RS_MAX_IN;

    const unsigned T = RS_TAPS, L = r->L, P = r->P;
    const uint32_t step_int  = r->step_int;
    const uint32_t step_rem  = r->step_rem;
    const uint32_t phase_mul = r->phase_mul;

    memcpy(r->bufL, r->histL, (T - 1) * sizeof(float));
    memcpy(r->bufR, r->histR, (T - 1) * sizeof(float));
    memcpy(r->bufL + (T - 1), inL, n_in * sizeof(float));
    memcpy(r->bufR + (T - 1), inR, n_in * sizeof(float));

    size_t   produced = 0;
    uint32_t idx      = r->pos / L;
    uint32_t rem      = r->pos % L;
    while (produced < max_out && idx < (uint32_t)n_in) {
        uint32_t ph = (P == L) ? rem : (uint32_t)(((uint64_t)rem * phase_mul) >> 20);
        if (ph >= P) ph = P - 1u;

        const float *h  = r->coef + (size_t)ph * T;
        const float *xl = r->bufL + idx;
        const float *xr = r->bufR + idx;

        rs_dotprod_stereo_unrolled(h, xl, xr, &outL[produced], &outR[produced]);
        ++produced;
        idx += step_int;
        rem += step_rem;
        if (rem >= L) {
            rem -= L;
            ++idx;
        }
    }

    uint32_t consumed_in = (idx < (uint32_t)n_in) ? idx : (uint32_t)n_in;
    r->pos = (idx >= (uint32_t)n_in) ? ((idx - (uint32_t)n_in) * L + rem) : rem;

    memcpy(r->histL, r->bufL + consumed_in, (T - 1) * sizeof(float));
    memcpy(r->histR, r->bufR + consumed_in, (T - 1) * sizeof(float));

    int64_t t1 = esp_timer_get_time();
    uint32_t dur = (uint32_t)(t1 - t0);

    /* Telemetry update */
    portENTER_CRITICAL(&r->stats_mux);
    r->seq++;
    r->total_calls++;
    r->in_frames_total += n_in;
    r->out_frames_total += produced;
    r->last_duration_us = dur;
    r->total_duration_us += dur;
    if (dur > r->max_duration_us) {
        r->max_duration_us = dur;
    }

    uint32_t now_ms = (uint32_t)(t1 / 1000);
    if (now_ms - r->recent_worst_reset_ms >= 2000) {
        r->recent_worst_us = dur;
        r->recent_worst_reset_ms = now_ms;
    } else if (dur > r->recent_worst_us) {
        r->recent_worst_us = dur;
    }

    r->window_proc_time_us += dur;
    if (t1 - r->last_calc_time_us >= 500000) {
        uint32_t elapsed = (uint32_t)(t1 - r->last_calc_time_us);
        if (elapsed > 0) {
            r->estimated_cpu_pct = ((float)r->window_proc_time_us * 100.0f) / (float)elapsed;
        }
        r->window_proc_time_us = 0;
        r->last_calc_time_us = t1;
    }

    int64_t expected = (r->in_frames_total * (uint64_t)r->L) / r->M;
    r->frame_mismatch = (int32_t)((int64_t)r->out_frames_total - expected);
    r->seq++;
    portEXIT_CRITICAL(&r->stats_mux);

    return produced;
}

void resampler_get_stats(const resampler_t *r, resampler_stats_t *out) {
    if (!out) return;
    memset(out, 0, sizeof(*out));
    if (!r) return;

    resampler_t *mr = (resampler_t *)r;
    portENTER_CRITICAL(&mr->stats_mux);
    out->in_rate              = r->in_rate;
    out->out_rate             = r->out_rate;
    out->ratio_L              = r->L;
    out->ratio_M              = r->M;
    out->phases               = r->P;
    out->bypass               = (uint8_t)r->bypass;
    out->is_psram             = r->is_psram;
    out->alloc_failures       = r->alloc_failures;
    out->total_calls          = r->total_calls;
    out->in_frames_total      = r->in_frames_total;
    out->out_frames_total     = r->out_frames_total;
    out->last_duration_us     = r->last_duration_us;
    out->avg_duration_us      = (uint32_t)(r->total_calls ? (r->total_duration_us / r->total_calls) : 0);
    out->max_duration_us      = r->max_duration_us;
    out->recent_worst_us      = r->recent_worst_us;
    out->cpu_util_pct         = r->estimated_cpu_pct;
    out->frame_mismatch       = r->frame_mismatch;
    out->dropped_frames       = r->dropped_frames;
    out->underruns            = r->underruns;
    out->overruns             = r->overruns;
    out->buf_occupancy_frames = r->buf_occupancy_frames;
    out->buf_capacity_frames  = r->buf_capacity_frames;
    portEXIT_CRITICAL(&mr->stats_mux);
}

void resampler_reset_stats(resampler_t *r) {
    if (!r) return;
    portENTER_CRITICAL(&r->stats_mux);
    r->seq++;
    r->total_calls = 0;
    r->in_frames_total = 0;
    r->out_frames_total = 0;
    r->last_duration_us = 0;
    r->max_duration_us = 0;
    r->recent_worst_us = 0;
    r->total_duration_us = 0;
    r->window_proc_time_us = 0;
    r->estimated_cpu_pct = 0.0f;
    r->frame_mismatch = 0;
    r->dropped_frames = 0;
    r->underruns = 0;
    r->overruns = 0;
    r->seq++;
    portEXIT_CRITICAL(&r->stats_mux);
}

void resampler_record_flow(resampler_t *r, uint32_t underruns, uint32_t overruns,
                           uint32_t dropped, uint32_t occ, uint32_t cap) {
    if (!r) return;
    portENTER_CRITICAL(&r->stats_mux);
    r->seq++;
    r->underruns += underruns;
    r->overruns += overruns;
    r->dropped_frames += dropped;
    r->buf_occupancy_frames = occ;
    r->buf_capacity_frames = cap;
    r->seq++;
    portEXIT_CRITICAL(&r->stats_mux);
}

void resampler_run_validation_iters(int iters) {
    if (iters <= 0) iters = 1;
    const unsigned test_rates[][2] = {
        { 44100, 96000 },
        { 48000, 96000 },
        { 88200, 96000 },
        { 96000, 96000 },
        { 176400, 44100 },
        { 192000, 96000 },
        { 192000, 44100 }
    };
    const size_t n_rates = sizeof(test_rates) / sizeof(test_rates[0]);
    const size_t test_n_in = 256;
    const size_t max_test_out = test_n_in * 3;

    float *inL  = (float *)rs_alloc(test_n_in * sizeof(float), NULL);
    float *inR  = (float *)rs_alloc(test_n_in * sizeof(float), NULL);
    float *outL = (float *)rs_alloc(max_test_out * sizeof(float), NULL);
    float *outR = (float *)rs_alloc(max_test_out * sizeof(float), NULL);

    if (!inL || !inR || !outL || !outR) {
        ESP_LOGE(RS_TAG, "Validation aborted: cannot allocate test vectors");
        if (inL) heap_caps_free(inL);
        if (inR) heap_caps_free(inR);
        if (outL) heap_caps_free(outL);
        if (outR) heap_caps_free(outR);
        return;
    }

    for (size_t i = 0; i < test_n_in; ++i) {
        float f = 1000.0f;
        inL[i] = sinf(2.0f * (float)M_PI * f * (float)i / 44100.0f) * 0.8f;
        inR[i] = cosf(2.0f * (float)M_PI * f * (float)i / 44100.0f) * 0.8f;
    }

    const int k_bench_iters = (iters > 1) ? (iters * 400) : 2000;
    volatile float bench_sink = 0.0f;

    rs_log("\n============================= RESAMPLER BENCHMARK & VALIDATION =============================\n");
    for (size_t idx = 0; idx < n_rates; ++idx) {
        unsigned in_r  = test_rates[idx][0];
        unsigned out_r = test_rates[idx][1];

        resampler_t *r = resampler_create(in_r, out_r);
        if (!r) {
            rs_log("[RESAMPLE] ERROR: Failed to create resampler %u -> %u Hz\n", in_r, out_r);
            continue;
        }

        size_t cap = resampler_max_out(r, test_n_in);

        resampler_reset(r);
        int64_t t0 = esp_timer_get_time();
        size_t n_opt = 0;
        for (int it = 0; it < iters; ++it) {
            n_opt = resampler_process(r, inL, inR, test_n_in, outL, outR, cap);
        }
        int64_t t_opt = (esp_timer_get_time() - t0) / iters;

        int out_ok = (n_opt > 0 && n_opt <= cap);
        for (size_t i = 0; i < n_opt; ++i) {
            if (!isfinite(outL[i]) || !isfinite(outR[i])) {
                out_ok = 0;
                break;
            }
        }

        resampler_stats_t st;
        resampler_get_stats(r, &st);

        if (!r->bypass && r->coef && r->P > 0 && test_n_in >= RS_TAPS) {
            float max_err = 0.0f;
            int kernel_ok = 1;
            const size_t max_off = test_n_in - RS_TAPS + 1;

            for (unsigned ph = 0; ph < r->P; ++ph) {
                const float *h_row = r->coef + (size_t)ph * RS_TAPS;
                size_t off = (size_t)ph % max_off;
                float optL = 0.0f, optR = 0.0f;
                float refL = 0.0f, refR = 0.0f;
                rs_dotprod_stereo_unrolled(h_row, inL + off, inR + off, &optL, &optR);
                rs_dotprod_stereo_c(h_row, inL + off, inR + off, &refL, &refR);
                float eL = fabsf(optL - refL);
                float eR = fabsf(optR - refR);
                if (eL > max_err) max_err = eL;
                if (eR > max_err) max_err = eR;
                if (!isfinite(optL) || !isfinite(optR) || !(eL < 1e-5f) || !(eR < 1e-5f)) {
                    kernel_ok = 0;
                }
            }

            float sum_optL = 0.0f, sum_optR = 0.0f;
            unsigned ph_opt = 0;
            int64_t kt0 = esp_timer_get_time();
            for (int k = 0; k < k_bench_iters; ++k) {
                size_t off = (size_t)k & 127u;
                const float *h_row = r->coef + (size_t)ph_opt * RS_TAPS;
                if (++ph_opt >= r->P) ph_opt = 0;
                float oL, oR;
                rs_dotprod_stereo_unrolled(h_row, inL + off, inR + off, &oL, &oR);
                sum_optL += oL;
                sum_optR += oR;
            }
            int64_t k_opt_us = esp_timer_get_time() - kt0;
            bench_sink = sum_optL + sum_optR;
            if (!isfinite(bench_sink)) kernel_ok = 0;

            float sum_refL = 0.0f, sum_refR = 0.0f;
            unsigned ph_ref = 0;
            int64_t kt1 = esp_timer_get_time();
            for (int k = 0; k < k_bench_iters; ++k) {
                size_t off = (size_t)k & 127u;
                const float *h_row = r->coef + (size_t)ph_ref * RS_TAPS;
                if (++ph_ref >= r->P) ph_ref = 0;
                float rL, rR;
                rs_dotprod_stereo_c(h_row, inL + off, inR + off, &rL, &rR);
                sum_refL += rL;
                sum_refR += rR;
            }
            int64_t k_ref_us = esp_timer_get_time() - kt1;
            bench_sink = sum_refL + sum_refR;
            if (!isfinite(bench_sink)) kernel_ok = 0;

            float speedup = (k_opt_us > 0) ? ((float)k_ref_us / (float)k_opt_us) : 0.0f;

            rs_log("[RESAMPLE] %5u -> %5u Hz | L=%3u M=%3u P=%2u | in:%3u -> out:%3u | %4lld us | opt:%4lld us ref:%4lld us (%.2fx) | err=%.2e | Mem: %s | %s\n",
                   in_r, out_r, st.ratio_L, st.ratio_M, st.phases,
                   (unsigned)test_n_in, (unsigned)n_opt,
                   (long long)t_opt,
                   (long long)k_opt_us, (long long)k_ref_us, (double)speedup,
                   (double)max_err,
                   st.is_psram ? "PSRAM" : "DRAM",
                   (out_ok && kernel_ok) ? "PASS" : "FAIL");
        } else {
            rs_log("[RESAMPLE] %5u -> %5u Hz | L=%3u M=%3u P=%2u | in:%3u -> out:%3u | %4lld us | bypass | Mem: %s | %s\n",
                   in_r, out_r, st.ratio_L, st.ratio_M, st.phases,
                   (unsigned)test_n_in, (unsigned)n_opt,
                   (long long)t_opt,
                   st.is_psram ? "PSRAM" : "DRAM",
                   out_ok ? "PASS" : "FAIL");
        }

        resampler_destroy(r);
    }
    rs_log("=============================================================================================\n\n");

    heap_caps_free(inL);
    heap_caps_free(inR);
    heap_caps_free(outL);
    heap_caps_free(outR);
}

void resampler_run_validation(void) {
    resampler_run_validation_iters(25);
}

void resampler_run_self_test(void) {
    resampler_run_validation_iters(1);
}
