#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct lms_flac lms_flac_t;
typedef size_t (*lms_flac_read_fn)(void *user, void *buf, size_t bytes);

lms_flac_t *lms_flac_open(lms_flac_read_fn read_fn, void *user,
                          unsigned *rate, unsigned *channels, unsigned *bits);

size_t lms_flac_read(lms_flac_t *f, int32_t *out, size_t frames);

void lms_flac_close(lms_flac_t *f);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define RS_MAX_IN   1024

typedef struct resampler resampler_t;

typedef struct {
    uint32_t in_rate;
    uint32_t out_rate;
    uint32_t ratio_L;
    uint32_t ratio_M;
    uint32_t phases;
    uint8_t  bypass;
    uint8_t  is_psram;
    uint32_t alloc_failures;
    uint64_t total_calls;
    uint64_t in_frames_total;
    uint64_t out_frames_total;
    uint32_t last_duration_us;
    uint32_t avg_duration_us;
    uint32_t max_duration_us;
    uint32_t recent_worst_us;
    float    cpu_util_pct;
    int32_t  frame_mismatch;
    uint32_t dropped_frames;
    uint32_t underruns;
    uint32_t overruns;
    uint32_t buf_occupancy_frames;
    uint32_t buf_capacity_frames;
} resampler_stats_t;

resampler_t *resampler_create(unsigned in_rate, unsigned out_rate);
void         resampler_destroy(resampler_t *r);
void         resampler_reset(resampler_t *r);
int          resampler_is_passthrough(const resampler_t *r);

size_t resampler_max_out(const resampler_t *r, size_t n_in);
size_t resampler_in_for_out(const resampler_t *r, size_t out_frames);

size_t resampler_process(resampler_t *r,
                         const float *inL, const float *inR, size_t n_in,
                         float *outL, float *outR, size_t max_out);

void resampler_get_stats(const resampler_t *r, resampler_stats_t *out_stats);
void resampler_reset_stats(resampler_t *r);
void resampler_record_flow(resampler_t *r, uint32_t underruns, uint32_t overruns,
                           uint32_t dropped, uint32_t occ, uint32_t cap);
void resampler_run_validation(void);
void resampler_run_validation_iters(int iters);
void resampler_run_self_test(void);

#ifdef __cplusplus
}
#endif
