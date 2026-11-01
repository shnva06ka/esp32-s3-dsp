/*
 * Polyphase windowed-sinc (Kaiser) sample-rate converter, see sl_resample.h
 *
 * Uses exact rational stepping (L/M) so there is zero long-term clock drift,
 * paired with a compact polyphase table (up to 64 phases x 24 taps = 6 KB)
 * that stays resident in the ESP32-S3 L1 data cache.
 */
#include "sl_resample.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <esp_heap_caps.h>

#define RS_TAPS        24
#define RS_MAX_PHASES  64
#define RS_KAISER_B    6.5f

struct resampler {
    unsigned  L, M, P;
    int       bypass;
    uint32_t  pos;            /* output position in 1/L input-sample units, relative to first new sample */
    float    *coef;           /* [P][RS_TAPS] */
    float    *histL, *histR;  /* RS_TAPS-1 previous input samples */
    float    *bufL,  *bufR;   /* RS_TAPS-1 + RS_MAX_IN working buffer */
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

static void *rs_alloc(size_t bytes) {
    void *p = heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!p) p = heap_caps_malloc(bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    return p;
}

void resampler_destroy(resampler_t *r) {
    if (!r) return;
    heap_caps_free(r->coef);
    heap_caps_free(r->histL);
    heap_caps_free(r->histR);
    heap_caps_free(r->bufL);
    heap_caps_free(r->bufR);
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

    unsigned g = gcd_u(in_rate, out_rate);
    r->L = out_rate / g;
    r->M = in_rate / g;
    r->bypass = (in_rate == out_rate);
    if (r->bypass) return r;

    r->P = (r->L <= RS_MAX_PHASES) ? r->L : RS_MAX_PHASES;

    const size_t ncoef = (size_t)r->P * RS_TAPS;
    r->coef  = (float *)rs_alloc(ncoef * sizeof(float));
    r->histL = (float *)rs_alloc((RS_TAPS - 1) * sizeof(float));
    r->histR = (float *)rs_alloc((RS_TAPS - 1) * sizeof(float));
    r->bufL  = (float *)rs_alloc((RS_TAPS - 1 + RS_MAX_IN) * sizeof(float));
    r->bufR  = (float *)rs_alloc((RS_TAPS - 1 + RS_MAX_IN) * sizeof(float));
    if (!r->coef || !r->histL || !r->histR || !r->bufL || !r->bufR) {
        resampler_destroy(r);
        return NULL;
    }

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
        r->coef[(size_t)phase * RS_TAPS + j] = h;
    }

    /* Normalise each phase individually for unity DC gain and zero phase-ripple */
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
    return r;
}

int resampler_is_passthrough(const resampler_t *r) {
    return r ? r->bypass : 1;
}

size_t resampler_max_out(const resampler_t *r, size_t n_in) {
    if (!r || r->bypass) return n_in;
    return (n_in * r->L + r->M - 1) / r->M + 2;
}

size_t resampler_in_for_out(const resampler_t *r, size_t out_frames) {
    if (!r || r->bypass) return out_frames;
    if (out_frames <= 3) return 0;
    return ((out_frames - 2) * r->M) / r->L;
}

size_t resampler_process(resampler_t *r,
                         const float *inL, const float *inR, size_t n_in,
                         float *outL, float *outR, size_t max_out) {
    if (!r || n_in == 0) return 0;

    if (r->bypass) {
        size_t n = n_in < max_out ? n_in : max_out;
        memcpy(outL, inL, n * sizeof(float));
        memcpy(outR, inR, n * sizeof(float));
        return n;
    }
    if (n_in > RS_MAX_IN) n_in = RS_MAX_IN;

    const unsigned T = RS_TAPS, L = r->L, M = r->M, P = r->P;

    memcpy(r->bufL, r->histL, (T - 1) * sizeof(float));
    memcpy(r->bufR, r->histR, (T - 1) * sizeof(float));
    memcpy(r->bufL + (T - 1), inL, n_in * sizeof(float));
    memcpy(r->bufR + (T - 1), inR, n_in * sizeof(float));

    size_t   produced = 0;
    uint32_t pos      = r->pos;

    while (produced < max_out) {
        uint32_t idx = pos / L;
        if (idx >= n_in) break;
        uint32_t rem = pos - idx * L;
        uint32_t ph  = (P == L) ? rem : (rem * P) / L;

        const float *h  = r->coef + (size_t)ph * T;
        const float *xl = r->bufL + (T - 1) + idx;
        const float *xr = r->bufR + (T - 1) + idx;

        float accL = 0.0f, accR = 0.0f;
        for (unsigned j = 0; j < T; ++j) {
            float hj = h[j];
            accL += hj * xl[-(int)j];
            accR += hj * xr[-(int)j];
        }
        outL[produced] = accL;
        outR[produced] = accR;
        ++produced;
        pos += M;
    }

    uint32_t consumed = (uint32_t)n_in * L;
    r->pos = (pos >= consumed) ? (pos - consumed) : 0;

    memcpy(r->histL, r->bufL + n_in, (T - 1) * sizeof(float));
    memcpy(r->histR, r->bufR + n_in, (T - 1) * sizeof(float));
    return produced;
}
