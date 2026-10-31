/*
 * Squeezelite gain, scaling, crossfade ported for ESP32-S3
 * Based on squeezelite (c) Adrian Smith, Ralph Irving
 */
#include "squeezelite.h"

#define MAX_SCALESAMPLE 0x7fffffffffffLL
#define MIN_SCALESAMPLE -MAX_SCALESAMPLE

s32_t gain(s32_t g, s32_t sample) {
    s64_t res = (s64_t)g * (s64_t)sample;
    if (res > MAX_SCALESAMPLE) res = MAX_SCALESAMPLE;
    if (res < MIN_SCALESAMPLE) res = MIN_SCALESAMPLE;
    return (s32_t)(res >> 16);
}

s32_t to_gain(float f) {
    return (s32_t)(f * 65536.0F);
}

void _apply_gain(struct buffer *outputbuf, frames_t count, s32_t gainL, s32_t gainR, u8_t flags) {
    if (gainL == FIXED_ONE && gainR == FIXED_ONE && !(flags & (MONO_LEFT | MONO_RIGHT))) {
        return;
    }

    if ((flags & MONO_LEFT) && (flags & MONO_RIGHT)) {
        ISAMPLE_T *ptrL = (ISAMPLE_T *)(void *)outputbuf->readp;
        ISAMPLE_T *ptrR = (ISAMPLE_T *)(void *)outputbuf->readp + 1;
        while (count--) {
            *ptrL = *ptrR = (gain(gainL, *ptrL) + gain(gainR, *ptrR)) / 2;
            ptrL += 2; ptrR += 2;
        }
    } else if (flags & MONO_RIGHT) {
        ISAMPLE_T *ptr = (ISAMPLE_T *)(void *)outputbuf->readp + 1;
        while (count--) {
            *(ptr - 1) = *ptr = gain(gainR, *ptr);
            ptr += 2;
        }
    } else if (flags & MONO_LEFT) {
        ISAMPLE_T *ptr = (ISAMPLE_T *)(void *)outputbuf->readp;
        while (count--) {
            *(ptr + 1) = *ptr = gain(gainL, *ptr);
            ptr += 2;
        }
    } else {
        ISAMPLE_T *ptrL = (ISAMPLE_T *)(void *)outputbuf->readp;
        ISAMPLE_T *ptrR = (ISAMPLE_T *)(void *)outputbuf->readp + 1;
        while (count--) {
            *ptrL = gain(gainL, *ptrL);
            *ptrR = gain(gainR, *ptrR);
            ptrL += 2; ptrR += 2;
        }
    }
}

void _apply_cross(struct buffer *outputbuf, frames_t out_frames, s32_t cross_gain_in, s32_t cross_gain_out, ISAMPLE_T **cross_ptr) {
    ISAMPLE_T *ptr = (ISAMPLE_T *)(void *)outputbuf->readp;
    frames_t count = out_frames * 2;
    while (count--) {
        if (*cross_ptr >= (ISAMPLE_T *)(void *)outputbuf->wrap) {
            *cross_ptr -= outputbuf->size / sizeof(ISAMPLE_T);
        }
        *ptr = gain(cross_gain_out, *ptr) + gain(cross_gain_in, **cross_ptr);
        ptr++;
        (*cross_ptr)++;
    }
}

