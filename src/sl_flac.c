/*
 * Squeezelite FLAC codec ported for ESP32-S3 using dr_flac
 * Integrates into Squeezelite codec interface
 */
#define DR_FLAC_IMPLEMENTATION
#define DR_FLAC_NO_STDIO
#define DR_FLAC_NO_CRC
#define DR_FLAC_BUFFER_SIZE 4096
#include "dr_flac.h"

#include "squeezelite.h"
#include "sl_resample.h"

extern struct buffer *streambuf;
extern struct buffer *outputbuf;
extern struct streamstate stream;
extern struct outputstate output;
extern struct decodestate decode;
extern bool decode_running;

#define LOCK_S   mutex_lock(streambuf->mutex)
#define UNLOCK_S mutex_unlock(streambuf->mutex)
#define LOCK_O   mutex_lock(outputbuf->mutex)
#define UNLOCK_O mutex_unlock(outputbuf->mutex)

static drflac *pFlac = NULL;
static drflac_int32 *s_flacDecBuf = NULL;

/* Resampling to the fixed DSP/I2S rate (SAMPLE_RATE) */
#define OUT_RATE        ((unsigned)SAMPLE_RATE)
#define RS_OUT_CAP      4096
static resampler_t *s_rs = NULL;
static float *s_inL = NULL, *s_inR = NULL, *s_outL = NULL, *s_outR = NULL;
volatile unsigned sl_source_rate = 0;   /* sample rate of the stream currently decoded (for UI) */
volatile unsigned sl_source_bits = 0;   /* bit depth of the stream currently decoded (for UI) */

static size_t _flac_read_cb(void *pUserData, void *buffer, size_t bytesToRead) {
    (void)pUserData;
    size_t totalRead = 0;
    uint8_t *dst = (uint8_t *)buffer;

    while (bytesToRead > 0) {
        LOCK_S;
        size_t used = _buf_used(streambuf);
        bool active = (stream.state > DISCONNECT);
        if (used > 0) {
            size_t cont = _buf_cont_read(streambuf);
            size_t chunk = min(cont, bytesToRead);
            memcpy(dst, streambuf->readp, chunk);
            _buf_inc_readp(streambuf, chunk);
            UNLOCK_S;

            dst += chunk;
            totalRead += chunk;
            bytesToRead -= chunk;
            continue;
        }
        UNLOCK_S;

        if (totalRead > 0) {
            // Return whatever bytes we have available so dr_flac can decode immediately
            break;
        }

        if (!active || !decode_running) {
            // Truly at EOF
            break;
        }

        // Stream is actively buffering - wait briefly for TCP packets to arrive into streambuf
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    return totalRead;
}

static drflac_bool32 _flac_seek_cb(void *pUserData, int offset, drflac_seek_origin origin) {
    (void)pUserData;
    (void)offset;
    (void)origin;
    return DRFLAC_FALSE; // Sequential streaming - seeking not supported
}

static decode_state flac_decode(void) {
    if (!s_flacDecBuf) {
        return DECODE_ERROR;
    }

    if (!pFlac) {
        LOCK_S;
        unsigned avail = _buf_used(streambuf);
        bool toend = (stream.state <= DISCONNECT);
        UNLOCK_S;

        if (avail < 1024 && !toend) {
            return DECODE_RUNNING;
        }

        pFlac = drflac_open(_flac_read_cb, _flac_seek_cb, NULL, NULL, NULL);
        if (!pFlac) {
            LOG_WARN("drflac_open failed (avail=%u, toend=%d)", avail, toend);
            return toend ? DECODE_COMPLETE : DECODE_RUNNING;
        }

        if (pFlac->channels > 2) {
            LOG_ERROR("Multichannel FLAC (%u channels) not supported (stereo/mono only)", pFlac->channels);
            drflac_close(pFlac);
            pFlac = NULL;
            return DECODE_ERROR;
        }

        LOG_INFO("FLAC stream opened: %u channels, %u Hz, %u bits/sample",
                 pFlac->channels, pFlac->sampleRate, pFlac->bitsPerSample);

        resampler_destroy(s_rs);
        s_rs = NULL;
        sl_source_rate = pFlac->sampleRate;
        sl_source_bits = pFlac->bitsPerSample;
        if (pFlac->sampleRate != OUT_RATE) {
            s_rs = resampler_create(pFlac->sampleRate, OUT_RATE);
            if (!s_rs) {
                LOG_ERROR("Cannot resample %u Hz -> %u Hz (unsupported rate or out of memory)",
                          pFlac->sampleRate, OUT_RATE);
                drflac_close(pFlac);
                pFlac = NULL;
                return DECODE_ERROR;
            }
            LOG_INFO("Resampling %u Hz -> %u Hz on Core %d", pFlac->sampleRate, OUT_RATE, xPortGetCoreID());
        }
    }

    if (decode.new_stream) {
        LOCK_O;
        output.next_sample_rate = OUT_RATE;
        output.format = S32_LE;
        output.track_start = outputbuf->writep;
        decode.new_stream = false;
        UNLOCK_O;
    }

    frames_t max_decode = 1024;
    LOCK_O;
    frames_t space_frames = _buf_space(outputbuf) / BYTES_PER_FRAME;
    UNLOCK_O;

    if (space_frames < 512) {
        return DECODE_RUNNING;
    }

    if (s_rs) {
        frames_t cap = space_frames < RS_OUT_CAP ? space_frames : RS_OUT_CAP;
        frames_t in_fit = (frames_t)resampler_in_for_out(s_rs, cap);
        if (in_fit < max_decode) max_decode = in_fit;
        if (max_decode < 1) max_decode = 1;
    } else if (space_frames < max_decode) {
        max_decode = space_frames;
    }

    if (max_decode == 0) {
        return DECODE_RUNNING;
    }

    drflac_uint64 framesRead = drflac_read_pcm_frames_s32(pFlac, max_decode, s_flacDecBuf);
    if (framesRead == 0) {
        LOCK_S;
        bool toend = (stream.state <= DISCONNECT && _buf_used(streambuf) == 0);
        UNLOCK_S;
        return toend ? DECODE_COMPLETE : DECODE_RUNNING;
    }

    if (s_rs) {
        // Deinterleave to float, resample to OUT_RATE, convert back to 32-bit PCM
        const drflac_int32 *rsrc = s_flacDecBuf;
        const bool stereo = (pFlac->channels > 1);
        for (frames_t i = 0; i < (frames_t)framesRead; ++i) {
            float l = (float)rsrc[0] * (1.0f / 2147483648.0f);
            float r = stereo ? (float)rsrc[1] * (1.0f / 2147483648.0f) : l;
            s_inL[i] = l;
            s_inR[i] = r;
            rsrc += stereo ? 2 : 1;
        }

        size_t outN = resampler_process(s_rs, s_inL, s_inR, (size_t)framesRead, s_outL, s_outR, RS_OUT_CAP);
        if (outN == 0) {
            return DECODE_RUNNING;
        }

        if (s_rs && outputbuf) {
            uint32_t occ = (uint32_t)(_buf_used(outputbuf) / BYTES_PER_FRAME);
            uint32_t cap = (uint32_t)(OUTPUTBUF_SIZE / BYTES_PER_FRAME);
            resampler_record_flow(s_rs, 0, 0, 0, occ, cap);
        }

        LOCK_O;
        ISAMPLE_T *rptr = (ISAMPLE_T *)(void *)outputbuf->writep;
        ISAMPLE_T *buf_start = (ISAMPLE_T *)(void *)outputbuf->buf;
        ISAMPLE_T *buf_wrap  = (ISAMPLE_T *)(void *)outputbuf->wrap;

        for (size_t i = 0; i < outN; ++i) {
            float l = fmaxf(-1.0f, fminf(1.0f, s_outL[i]));
            float r = fmaxf(-1.0f, fminf(1.0f, s_outR[i]));
            *rptr++ = (s32_t)(l * 2147483392.0f);
            *rptr++ = (s32_t)(r * 2147483392.0f);
            if (rptr >= buf_wrap) {
                rptr = buf_start;
            }
        }
        _buf_inc_writep(outputbuf, (unsigned)outN * BYTES_PER_FRAME);
        UNLOCK_O;
        return DECODE_RUNNING;
    }

    LOCK_O;
    ISAMPLE_T *optr = (ISAMPLE_T *)(void *)outputbuf->writep;
    ISAMPLE_T *buf_start = (ISAMPLE_T *)(void *)outputbuf->buf;
    ISAMPLE_T *buf_wrap  = (ISAMPLE_T *)(void *)outputbuf->wrap;

    const drflac_int32 *src = s_flacDecBuf;
    for (frames_t i = 0; i < (frames_t)framesRead; ++i) {
        if (pFlac->channels > 1) {
            *optr++ = (s32_t)(*src++);
            *optr++ = (s32_t)(*src++);
        } else {
            s32_t val = (s32_t)(*src++);
            *optr++ = val;
            *optr++ = val;
        }
        if (optr >= buf_wrap) {
            optr = buf_start;
        }
    }

    _buf_inc_writep(outputbuf, (unsigned)framesRead * BYTES_PER_FRAME);
    UNLOCK_O;

    return DECODE_RUNNING;
}

static void flac_open(u8_t sample_size, u8_t sample_rate, u8_t channels, u8_t endianness) {
    (void)sample_size;
    (void)sample_rate;
    (void)channels;
    (void)endianness;

    if (pFlac) {
        drflac_close(pFlac);
        pFlac = NULL;
    }
    LOG_INFO("Opening FLAC codec");
}

static void flac_close(void) {
    if (pFlac) {
        drflac_close(pFlac);
        pFlac = NULL;
    }
    resampler_destroy(s_rs);
    s_rs = NULL;
    sl_source_rate = 0;
    sl_source_bits = 0;
    LOG_INFO("Closing FLAC codec");
}

static struct codec flac_codec = {
    'f',
    "flc",
    512,
    512 * BYTES_PER_FRAME,
    flac_open,
    flac_close,
    flac_decode
};

struct codec *register_flac(void) {
    if (!s_flacDecBuf) {
        s_flacDecBuf = (drflac_int32 *)heap_caps_malloc(1024 * 2 * sizeof(drflac_int32), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (!s_flacDecBuf) {
            s_flacDecBuf = (drflac_int32 *)heap_caps_malloc(1024 * 2 * sizeof(drflac_int32), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        }
    }
    if (!s_inL) {
        s_inL  = (float *)heap_caps_aligned_alloc(16, RS_MAX_IN * sizeof(float), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (!s_inL) s_inL = (float *)heap_caps_malloc(RS_MAX_IN * sizeof(float), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (!s_inL) s_inL = (float *)heap_caps_malloc(RS_MAX_IN * sizeof(float), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

        s_inR  = (float *)heap_caps_aligned_alloc(16, RS_MAX_IN * sizeof(float), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (!s_inR) s_inR = (float *)heap_caps_malloc(RS_MAX_IN * sizeof(float), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (!s_inR) s_inR = (float *)heap_caps_malloc(RS_MAX_IN * sizeof(float), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

        s_outL = (float *)heap_caps_aligned_alloc(16, RS_OUT_CAP * sizeof(float), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (!s_outL) s_outL = (float *)heap_caps_malloc(RS_OUT_CAP * sizeof(float), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (!s_outL) s_outL = (float *)heap_caps_malloc(RS_OUT_CAP * sizeof(float), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

        s_outR = (float *)heap_caps_aligned_alloc(16, RS_OUT_CAP * sizeof(float), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (!s_outR) s_outR = (float *)heap_caps_malloc(RS_OUT_CAP * sizeof(float), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (!s_outR) s_outR = (float *)heap_caps_malloc(RS_OUT_CAP * sizeof(float), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

        if (!s_inL || !s_inR || !s_outL || !s_outR) {
            LOG_ERROR("FLAC resampler buffers allocation failed");
        }
    }
    return &flac_codec;
}

bool sl_flac_get_resampler_stats(resampler_stats_t *out) {
    if (!s_rs || !out) return false;
    resampler_get_stats(s_rs, out);
    return true;
}
