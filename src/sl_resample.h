/*
 * Polyphase windowed-sinc (Kaiser) sample-rate converter for stereo float audio.
 * Used to bring Squeezebox FLAC streams and the Bluetooth I2S feed (44.1 kHz)
 * to the fixed DSP / I2S master rate (SAMPLE_RATE).
 */
#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RS_MAX_IN   1024   /* max input frames per resampler_process() call */

typedef struct resampler resampler_t;

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

#ifdef __cplusplus
}
#endif

