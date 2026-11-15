/*
 * Squeezelite PCM/WAV/AIFF codec ported for ESP32-S3
 * Based on squeezelite (c) 2012-2015 Adrian Smith, 2015-2021 Ralph Irving
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Modified for ESP32-S3-DSP (2026) under GNU GPL v3.0 (see LICENSE).
 * Polyphase Kaiser-sinc resampling to fixed 96 kHz DSP/I2S output rate
 */
#include "squeezelite.h"
#include "sl_resample.h"
#include "xtensa_fpu_helpers.h"

extern struct buffer *streambuf;
extern struct buffer *outputbuf;
extern struct streamstate stream;
extern struct outputstate output;
extern struct decodestate decode;
extern volatile unsigned sl_source_rate;
extern volatile unsigned sl_source_bits;

#define LOCK_S   mutex_lock(streambuf->mutex)
#define UNLOCK_S mutex_unlock(streambuf->mutex)
#define LOCK_O   mutex_lock(outputbuf->mutex)
#define UNLOCK_O mutex_unlock(outputbuf->mutex)

#define OUT_RATE          ((unsigned)SAMPLE_RATE)
#define RS_OUT_CAP        4096
#define MAX_DECODE_FRAMES 1024

static resampler_t *s_pcm_rs = NULL;
static u8_t  *s_pcmRawIn = NULL;
static float *s_pcmInL   = NULL;
static float *s_pcmInR   = NULL;
static float *s_pcmOutL  = NULL;
static float *s_pcmOutR  = NULL;

static u32_t pcm_sample_rate = 44100;
static u32_t sample_size = 2; // bytes per sample per channel
static u32_t channels = 2;
static bool  bigendian = false;
static bool  limit = false;
static u32_t audio_left = 0;
static bool  header_checked = false;
static bool  resampler_ok = true;

typedef enum { UNKNOWN = 0, WAVE, AIFF } header_format;

static unsigned _parse_ieee80_rate(const u8_t *p) {
    int expon = (((int)p[0] & 0x7F) << 8) | p[1];
    uint32_t hi = ((uint32_t)p[2] << 24) | ((uint32_t)p[3] << 16) | ((uint32_t)p[4] << 8) | p[5];
    uint32_t lo = ((uint32_t)p[6] << 24) | ((uint32_t)p[7] << 16) | ((uint32_t)p[8] << 8) | p[9];
    if (expon == 0 && hi == 0 && lo == 0) return 0;
    if (expon == 0x7FFF) return 0;
    expon -= 16383;
    if (expon < 0 || expon > 30) return 0;
    double mant = (double)hi + (double)lo / 4294967296.0;
    double val = ldexp(mant, expon - 31);
    if (val < 8000.0 || val > 384000.0) return 0;
    return (unsigned)(val + 0.5);
}

static unsigned pcm_parse_rate(u8_t rate_char) {
    switch (rate_char) {
        case '0': return 11025;
        case '1': return 22050;
        case '2': return 32000;
        case '3': return 44100;
        case '4': return 48000;
        case '5': return 8000;
        case '6': return 12000;
        case '7': return 16000;
        case '8': return 24000;
        case '9': return 96000;
        case ':': return 88200;
        case ';': return 176400;
        case '<': return 192000;
        default:  return 44100;
    }
}

static bool _update_resampler(unsigned rate) {
    pcm_sample_rate = rate;
    sl_source_rate  = rate;
    sl_source_bits  = sample_size * 8;
    if (s_pcm_rs) {
        resampler_destroy(s_pcm_rs);
        s_pcm_rs = NULL;
    }
    if (rate != OUT_RATE) {
        s_pcm_rs = resampler_create(rate, OUT_RATE);
        if (!s_pcm_rs) {
            LOG_ERROR("PCM: Cannot resample %u Hz -> %u Hz", rate, OUT_RATE);
            resampler_ok = false;
            return false;
        }
        LOG_INFO("PCM: Resampling %u Hz -> %u Hz on Core %d", rate, OUT_RATE, xPortGetCoreID());
    }
    resampler_ok = true;
    return true;
}

static void _buf_peek(struct buffer *b, u8_t *dst, unsigned len) {
    unsigned cont = _buf_cont_read(b);
    unsigned first = (len < cont) ? len : cont;
    memcpy(dst, b->readp, first);
    if (len > first) {
        memcpy(dst + first, b->buf, len - first);
    }
}

static void _check_header(bool toend) {
    unsigned used = _buf_used(streambuf);
    if (used < 12) {
        if (toend) header_checked = true;
        return;
    }
    if (!s_pcmRawIn) {
        header_checked = true;
        return;
    }

    unsigned bytes = (used < 4096) ? used : 4096;
    _buf_peek(streambuf, s_pcmRawIn, bytes);
    const u8_t *base = s_pcmRawIn;
    const u8_t *ptr  = base;
    header_format format = UNKNOWN;

    if (!memcmp(ptr, "RIFF", 4) && !memcmp(ptr + 8, "WAVE", 4)) {
        LOG_INFO("Detected WAVE container");
        format = WAVE;
    } else if (!memcmp(ptr, "FORM", 4) && (!memcmp(ptr + 8, "AIFF", 4) || !memcmp(ptr + 8, "AIFC", 4))) {
        LOG_INFO("Detected AIFF container");
        format = AIFF;
    }

    if (format != UNKNOWN) {
        ptr   += 12;
        bytes -= 12;

        while (bytes >= 8) {
            uint32_t len;
            if (format == WAVE) {
                len = (u32_t)ptr[4] | ((u32_t)ptr[5] << 8) | ((u32_t)ptr[6] << 16) | ((u32_t)ptr[7] << 24);
            } else {
                len = ((u32_t)ptr[4] << 24) | ((u32_t)ptr[5] << 16) | ((u32_t)ptr[6] << 8) | (u32_t)ptr[7];
            }

            if (format == WAVE && !memcmp(ptr, "fmt ", 4) && bytes >= 24) {
                channels    = (u32_t)ptr[10] | ((u32_t)ptr[11] << 8);
                u32_t r     = (u32_t)ptr[12] | ((u32_t)ptr[13] << 8) | ((u32_t)ptr[14] << 16) | ((u32_t)ptr[15] << 24);
                sample_size = ((u32_t)ptr[22] | ((u32_t)ptr[23] << 8)) / 8;
                bigendian   = false;
                if (channels < 1 || channels > 2) channels = 2;
                if (sample_size < 1 || sample_size > 4) sample_size = 2;
                if (r > 0 && r != pcm_sample_rate) {
                    _update_resampler(r);
                }
                sl_source_bits = sample_size * 8;
                LOG_INFO("WAV fmt: %u ch, %u Hz, %u bytes/sample", channels, pcm_sample_rate, sample_size);
            }

            if (format == WAVE && !memcmp(ptr, "data", 4)) {
                unsigned hdr_skip = (unsigned)((ptr + 8) - base);
                _buf_inc_readp(streambuf, hdr_skip);
                audio_left = len;
                limit = (audio_left != 0xFFFFFFFF && audio_left != 0x7FFFEFFC);
                LOG_INFO("WAV data chunk: %u bytes", audio_left);
                header_checked = true;
                return;
            }

            if (format == AIFF && !memcmp(ptr, "COMM", 4) && bytes >= 26) {
                channels    = ((u32_t)ptr[8] << 8) | (u32_t)ptr[9];
                sample_size = (((u32_t)ptr[14] << 8) | (u32_t)ptr[15]) / 8;
                bigendian   = true;
                if (channels < 1 || channels > 2) channels = 2;
                if (sample_size < 1 || sample_size > 4) sample_size = 2;
                unsigned r = _parse_ieee80_rate(ptr + 16);
                if (r > 0 && r != pcm_sample_rate) {
                    _update_resampler(r);
                }
                sl_source_bits = sample_size * 8;
                LOG_INFO("AIFF COMM: %u ch, %u Hz, %u bytes/sample", channels, pcm_sample_rate, sample_size);
            }

            if (format == AIFF && !memcmp(ptr, "SSND", 4) && bytes >= 16) {
                uint32_t offset = ((u32_t)ptr[8] << 24) | ((u32_t)ptr[9] << 16) | ((u32_t)ptr[10] << 8) | (u32_t)ptr[11];
                uint64_t hdr_base = (uint64_t)((ptr + 16) - base);
                uint64_t skip_ssnd = hdr_base + (uint64_t)offset;
                if (len >= 8 && (uint64_t)offset <= (uint64_t)len - 8ULL && skip_ssnd <= (uint64_t)used) {
                    _buf_inc_readp(streambuf, (unsigned)skip_ssnd);
                    audio_left = (u32_t)((uint64_t)len - 8ULL - (uint64_t)offset);
                    limit = true;
                    LOG_INFO("AIFF SSND chunk: %u bytes", audio_left);
                    header_checked = true;
                    return;
                }
                break;
            }

            uint64_t padded_len = ((uint64_t)len + 1ULL) & ~1ULL;
            uint64_t skip = 8ULL + padded_len;
            if (skip > bytes) break;
            ptr += (size_t)skip;
            bytes -= (size_t)skip;
        }
    }

    if (format == UNKNOWN || used > 4096 || toend) {
        header_checked = true;
    }
}

static inline s32_t _pcm_decode_s32(const u8_t *p, u32_t sz, bool be) {
    if (sz == 1) {
        return ((s32_t)*p - 128) << 24;
    }
    if (sz == 2) {
        return be
            ? (s32_t)(((u32_t)p[0] << 24) | ((u32_t)p[1] << 16))
            : (s32_t)(((u32_t)p[0] << 16) | ((u32_t)p[1] << 24));
    }
    if (sz == 3) {
        return be
            ? (s32_t)(((u32_t)p[0] << 24) | ((u32_t)p[1] << 16) | ((u32_t)p[2] << 8))
            : (s32_t)(((u32_t)p[0] << 8)  | ((u32_t)p[1] << 16) | ((u32_t)p[2] << 24));
    }
    return be
        ? (s32_t)(((u32_t)p[0] << 24) | ((u32_t)p[1] << 16) | ((u32_t)p[2] << 8) | (u32_t)p[3])
        : (s32_t)((u32_t)p[0] | ((u32_t)p[1] << 8) | ((u32_t)p[2] << 16) | ((u32_t)p[3] << 24));
}

static inline u32_t _pcm_load_u32_aligned(const u8_t *p) {
    u32_t w;
    memcpy(&w, p, sizeof(w));
    return w;
}

static void _pcm_unpack_to_float(const u8_t *iptr, frames_t frames,
                                 u32_t ch, u32_t sz, bool be,
                                 float * DSP_RESTRICT dstL, float * DSP_RESTRICT dstR) {
    const float kScale32 = 1.0f / 2147483648.0f;

    if (ch == 2 && !be) {
        if (sz == 4 && (((uintptr_t)iptr & 3u) == 0u)) {
            dsp_unpack_i32_stereo_to_f32((const int32_t *)(const void *)iptr,
                                         dstL, dstR, frames, kScale32, kScale32);
            return;
        }
        if (sz == 2 && (((uintptr_t)iptr & 3u) == 0u)) {
#if defined(__GNUC__) || defined(__clang__)
            iptr = (const u8_t *)__builtin_assume_aligned(iptr, 4);
#endif
            frames_t i = 0;
            for (; i + 4u <= frames; i += 4u) {
                const u32_t w0 = _pcm_load_u32_aligned(iptr);
                const u32_t w1 = _pcm_load_u32_aligned(iptr + 4);
                dstL[0] = (float)(s32_t)(w0 << 16)         * kScale32;
                dstR[0] = (float)(s32_t)(w0 & 0xFFFF0000u) * kScale32;
                dstL[1] = (float)(s32_t)(w1 << 16)         * kScale32;
                dstR[1] = (float)(s32_t)(w1 & 0xFFFF0000u) * kScale32;

                const u32_t w2 = _pcm_load_u32_aligned(iptr + 8);
                const u32_t w3 = _pcm_load_u32_aligned(iptr + 12);
                dstL[2] = (float)(s32_t)(w2 << 16)         * kScale32;
                dstR[2] = (float)(s32_t)(w2 & 0xFFFF0000u) * kScale32;
                dstL[3] = (float)(s32_t)(w3 << 16)         * kScale32;
                dstR[3] = (float)(s32_t)(w3 & 0xFFFF0000u) * kScale32;

                iptr += 16;
                dstL += 4;
                dstR += 4;
            }
            for (; i < frames; ++i) {
                const u32_t w = _pcm_load_u32_aligned(iptr);
                *dstL++ = (float)(s32_t)(w << 16)         * kScale32;
                *dstR++ = (float)(s32_t)(w & 0xFFFF0000u) * kScale32;
                iptr += 4;
            }
            return;
        }
        if (sz == 3) {
            frames_t i = 0;
            for (; i + 4u <= frames; i += 4u) {
                const s32_t sl0 = _pcm_decode_s32(iptr,      3, false);
                const s32_t sr0 = _pcm_decode_s32(iptr + 3,  3, false);
                const s32_t sl1 = _pcm_decode_s32(iptr + 6,  3, false);
                const s32_t sr1 = _pcm_decode_s32(iptr + 9,  3, false);
                dstL[0] = (float)sl0 * kScale32;
                dstR[0] = (float)sr0 * kScale32;
                dstL[1] = (float)sl1 * kScale32;
                dstR[1] = (float)sr1 * kScale32;

                const s32_t sl2 = _pcm_decode_s32(iptr + 12, 3, false);
                const s32_t sr2 = _pcm_decode_s32(iptr + 15, 3, false);
                const s32_t sl3 = _pcm_decode_s32(iptr + 18, 3, false);
                const s32_t sr3 = _pcm_decode_s32(iptr + 21, 3, false);
                dstL[2] = (float)sl2 * kScale32;
                dstR[2] = (float)sr2 * kScale32;
                dstL[3] = (float)sl3 * kScale32;
                dstR[3] = (float)sr3 * kScale32;

                iptr += 24;
                dstL += 4;
                dstR += 4;
            }
            for (; i < frames; ++i) {
                *dstL++ = (float)_pcm_decode_s32(iptr,     3, false) * kScale32;
                *dstR++ = (float)_pcm_decode_s32(iptr + 3, 3, false) * kScale32;
                iptr += 6;
            }
            return;
        }
    }

    if (ch == 2) {
        const size_t step = sz * 2u;
        for (frames_t i = 0; i < frames; ++i) {
            *dstL++ = (float)_pcm_decode_s32(iptr,      sz, be) * kScale32;
            *dstR++ = (float)_pcm_decode_s32(iptr + sz, sz, be) * kScale32;
            iptr += step;
        }
    } else {
        for (frames_t i = 0; i < frames; ++i) {
            const float s = (float)_pcm_decode_s32(iptr, sz, be) * kScale32;
            *dstL++ = s;
            *dstR++ = s;
            iptr += sz;
        }
    }
}

static decode_state pcm_decode(void) {
    LOCK_S;

    bool toend = (stream.state <= DISCONNECT);
    if (!header_checked) {
        _check_header(toend);
        if (!header_checked) {
            UNLOCK_S;
            return DECODE_RUNNING;
        }
    }

    if (!resampler_ok) {
        UNLOCK_S;
        return DECODE_ERROR;
    }

    if (limit && audio_left == 0) {
        UNLOCK_S;
        return DECODE_COMPLETE;
    }

    unsigned bytes = _buf_used(streambuf);

    u32_t bytes_per_frame = channels * sample_size;
    if (bytes_per_frame == 0) bytes_per_frame = 4;

    if (bytes < bytes_per_frame) {
        UNLOCK_S;
        return toend ? DECODE_COMPLETE : DECODE_RUNNING;
    }

    if (limit) {
        bytes = min(bytes, audio_left);
    }

    frames_t frames = bytes / bytes_per_frame;
    if (frames > MAX_DECODE_FRAMES) frames = MAX_DECODE_FRAMES;

    if (frames == 0) {
        UNLOCK_S;
        return toend ? DECODE_COMPLETE : DECODE_RUNNING;
    }

    if (decode.new_stream) {
        LOCK_O;
        output.next_sample_rate = OUT_RATE;
        output.track_start = outputbuf->writep;
        output.format = S32_LE;
        decode.new_stream = false;
        UNLOCK_O;
    }

    LOCK_O;
    frames_t space_frames = _buf_space(outputbuf) / BYTES_PER_FRAME;
    UNLOCK_O;

    if (space_frames < 512) {
        UNLOCK_S;
        return DECODE_RUNNING;
    }

    frames_t max_decode = MAX_DECODE_FRAMES;
    if (s_pcm_rs) {
        frames_t cap = space_frames < RS_OUT_CAP ? space_frames : RS_OUT_CAP;
        frames_t in_fit = (frames_t)resampler_in_for_out(s_pcm_rs, cap);
        if (in_fit < max_decode) max_decode = in_fit;
    } else if (space_frames < max_decode) {
        max_decode = space_frames;
    }

    if (frames > max_decode) frames = max_decode;

    if (frames == 0) {
        UNLOCK_S;
        return DECODE_RUNNING;
    }

    if (!s_pcmRawIn || !s_pcmInL || !s_pcmInR) {
        UNLOCK_S;
        return DECODE_ERROR;
    }
    const size_t raw_cap = RS_MAX_IN * 8;
    unsigned consumed = frames * bytes_per_frame;
    if (consumed > raw_cap) {
        frames = raw_cap / bytes_per_frame;
        consumed = frames * bytes_per_frame;
    }

    _buf_peek(streambuf, s_pcmRawIn, consumed);
    _buf_inc_readp(streambuf, consumed);
    if (limit && audio_left >= consumed) {
        audio_left -= consumed;
    }
    u32_t out_seq = outputbuf->flush_seq;
    UNLOCK_S;

    _pcm_unpack_to_float(s_pcmRawIn, frames, channels, sample_size, bigendian, s_pcmInL, s_pcmInR);

    size_t out_frames = 0;
    float *writeL = s_pcmInL;
    float *writeR = s_pcmInR;
    if (s_pcm_rs && s_pcmOutL && s_pcmOutR) {
        out_frames = resampler_process(s_pcm_rs, s_pcmInL, s_pcmInR, (size_t)frames, s_pcmOutL, s_pcmOutR, RS_OUT_CAP);
        writeL = s_pcmOutL;
        writeR = s_pcmOutR;
    } else {
        out_frames = (size_t)frames;
    }

    if (out_frames > 0 && writeL && writeR) {
        LOCK_O;
        if (outputbuf->flush_seq == out_seq) {
            ISAMPLE_T *optr = (ISAMPLE_T *)(void *)outputbuf->writep;
            ISAMPLE_T *buf_start = (ISAMPLE_T *)(void *)outputbuf->buf;
            ISAMPLE_T *buf_wrap  = (ISAMPLE_T *)(void *)outputbuf->wrap;

            dsp_ring_pack_f32_to_i32(optr, buf_start, buf_wrap, writeL, writeR, out_frames);
            _buf_inc_writep(outputbuf, (unsigned)out_frames * BYTES_PER_FRAME);
        }
        UNLOCK_O;
    }

    if (limit && audio_left == 0) {
        return DECODE_COMPLETE;
    }

    return DECODE_RUNNING;
}

static void pcm_open(u8_t size, u8_t rate, u8_t chan, u8_t endianness) {
    channels    = (chan >= '1') ? (chan - '0') : 2;
    if (channels < 1 || channels > 2) channels = 2;
    sample_size = size - '0' + 1;
    if (sample_size < 1 || sample_size > 4) sample_size = 2;
    bigendian   = (endianness == '0');

    limit = false;
    audio_left = 0;
    header_checked = false;

    unsigned parsed_rate = pcm_parse_rate(rate);
    _update_resampler(parsed_rate);

    LOG_INFO("Opening PCM codec: size=%u (%u bits) rate=%u chan=%u bigendian=%u",
             sample_size, sample_size * 8, pcm_sample_rate, channels, bigendian);
    LOCK_S;
    bool empty = (_buf_used(streambuf) == 0);
    UNLOCK_S;
    if (empty) {
        buf_adjust(streambuf, sample_size * channels);
    }
}

static void pcm_close(void) {
    LOG_INFO("Closing PCM codec");
    if (s_pcm_rs) {
        resampler_destroy(s_pcm_rs);
        s_pcm_rs = NULL;
    }
    buf_adjust(streambuf, 1);
    limit = false;
    audio_left = 0;
    header_checked = false;
}

static struct codec pcm_codec = {
    'p',
    "pcm,wav,aif",
    64,
    1024 * BYTES_PER_FRAME,
    pcm_open,
    pcm_close,
    pcm_decode
};

static void *pcm_alloc_staging(size_t bytes) {
    return dsp_alloc_staging(bytes);
}

struct codec *register_pcm(void) {
    if (!s_pcmInL) {
        s_pcmRawIn = (u8_t *) pcm_alloc_staging(RS_MAX_IN * 8);
        s_pcmInL   = (float *)pcm_alloc_staging(RS_MAX_IN * sizeof(float));
        s_pcmInR   = (float *)pcm_alloc_staging(RS_MAX_IN * sizeof(float));
        s_pcmOutL  = (float *)pcm_alloc_staging(RS_OUT_CAP * sizeof(float));
        s_pcmOutR  = (float *)pcm_alloc_staging(RS_OUT_CAP * sizeof(float));

        if (!s_pcmRawIn || !s_pcmInL || !s_pcmInR || !s_pcmOutL || !s_pcmOutR) {
            if (s_pcmRawIn) { heap_caps_free(s_pcmRawIn); s_pcmRawIn = NULL; }
            if (s_pcmInL)   { heap_caps_free(s_pcmInL);   s_pcmInL   = NULL; }
            if (s_pcmInR)   { heap_caps_free(s_pcmInR);   s_pcmInR   = NULL; }
            if (s_pcmOutL)  { heap_caps_free(s_pcmOutL);  s_pcmOutL  = NULL; }
            if (s_pcmOutR)  { heap_caps_free(s_pcmOutR);  s_pcmOutR  = NULL; }
            LOG_ERROR("PCM resampler buffers allocation failed");
            return NULL;
        }
    }
    return &pcm_codec;
}
