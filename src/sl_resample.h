/*
 * Polyphase windowed-sinc (Kaiser) sample rate converter
 */
#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RS_MAX_IN   1024   /* max input frames per resampler_process() call */

typedef struct resampler resampler_t;

typedef struct {
    uint32_t in_rate;
    uint32_t out_rate;
    uint32_t ratio_L;
    uint32_t ratio_M;
    uint32_t phases;
    uint8_t  bypass;
    uint8_t  is_psram;          /* 0 = Internal DRAM, 1 = PSRAM fallback */
    uint32_t alloc_failures;
    uint64_t total_calls;
    uint64_t in_frames_total;
    uint64_t out_frames_total;
    uint32_t last_duration_us;
    uint32_t avg_duration_us;
    uint32_t max_duration_us;
    uint32_t recent_worst_us;
    float    cpu_util_pct;      /* Estimated resampler CPU utilization % */
    int32_t  frame_mismatch;    /* Output frames generated vs exact rational expectation */
    uint32_t dropped_frames;
    uint32_t underruns;
    uint32_t overruns;
    uint32_t buf_occupancy_frames;
    uint32_t buf_capacity_frames;
} resampler_stats_t;

/* Returns NULL on invalid rates or allocation failure. in_rate == out_rate yields a pass-through. */
resampler_t *resampler_create(unsigned in_rate, unsigned out_rate);
void         resampler_destroy(resampler_t *r);
void         resampler_reset(resampler_t *r);
int          resampler_is_passthrough(const resampler_t *r);

/* Worst-case number of output frames produced by one call with n_in input frames. */
size_t resampler_max_out(const resampler_t *r, size_t n_in);
/* Largest n_in guaranteed to produce <= out_frames output frames. */
size_t resampler_in_for_out(const resampler_t *r, size_t out_frames);

/* n_in must be <= RS_MAX_IN and max_out >= resampler_max_out(r, n_in). Returns frames written. */
size_t resampler_process(resampler_t *r,
                         const float *inL, const float *inR, size_t n_in,
                         float *outL, float *outR, size_t max_out);

/* Telemetry & diagnostics */
void resampler_get_stats(const resampler_t *r, resampler_stats_t *out_stats);
void resampler_reset_stats(resampler_t *r);
void resampler_record_flow(resampler_t *r, uint32_t underruns, uint32_t overruns,
                           uint32_t dropped, uint32_t occ, uint32_t cap);
/* Run self-test validation on multiple sample rates (44.1->96, 48->96, 88.2->96, 96->96) */
void resampler_run_validation(void);
void resampler_run_validation_iters(int iters);
void resampler_run_self_test(void);

#ifdef __cplusplus
}
#endif
