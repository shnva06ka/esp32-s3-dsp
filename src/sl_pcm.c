/*
 * Squeezelite PCM/WAV/AIFF codec ported for ESP32-S3
 * Based on squeezelite (c) Adrian Smith, Ralph Irving
 * Polyphase Kaiser-sinc resampling to fixed 96 kHz DSP/I2S output rate
 */
#include "squeezelite.h"
#include "sl_resample.h"

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
static float *s_pcmInL  = NULL;
static float *s_pcmInR  = NULL;
static float *s_pcmOutL = NULL;
static float *s_pcmOutR = NULL;

static u32_t pcm_sample_rate = 44100;
static u32_t sample_size = 2; // bytes per sample per channel
static u32_t channels = 2;
static bool  bigendian = false;
static bool  limit = false;
static u32_t audio_left = 0;
static bool  header_checked = false;

typedef enum { UNKNOWN = 0, WAVE, AIFF } header_format;

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

static void _update_resampler(unsigned rate) {
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
        } else {
            LOG_INFO("PCM: Resampling %u Hz -> %u Hz on Core %d", rate, OUT_RATE, xPortGetCoreID());
        }
    }
}

static void _check_header(void) {
    u8_t *ptr = streambuf->readp;
    unsigned bytes = min(_buf_used(streambuf), _buf_cont_read(streambuf));
    header_format format = UNKNOWN;

    if (bytes > 12) {
        if (!memcmp(ptr, "RIFF", 4) && !memcmp(ptr + 8, "WAVE", 4)) {
            LOG_INFO("Detected WAVE container");
            format = WAVE;
        } else if (!memcmp(ptr, "FORM", 4) && (!memcmp(ptr + 8, "AIFF", 4) || !memcmp(ptr + 8, "AIFC", 4))) {
            LOG_INFO("Detected AIFF container");
            format = AIFF;
        }
    }

    if (format != UNKNOWN) {
        ptr   += 12;
        bytes -= 12;

        while (bytes >= 8) {
            unsigned len;
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
                ptr += 8;
                _buf_inc_readp(streambuf, (unsigned)(ptr - streambuf->readp));
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
                sl_source_bits = sample_size * 8;
                LOG_INFO("AIFF COMM: %u ch, %u bytes/sample", channels, sample_size);
            }

            if (format == AIFF && !memcmp(ptr, "SSND", 4) && bytes >= 16) {
                unsigned offset = ((u32_t)ptr[8] << 24) | ((u32_t)ptr[9] << 16) | ((u32_t)ptr[10] << 8) | (u32_t)ptr[11];
                ptr += 16;
                unsigned skip_ssnd = (unsigned)(ptr + offset - streambuf->readp);
                if (len >= 8 + offset && skip_ssnd <= _buf_used(streambuf)) {
                    _buf_inc_readp(streambuf, skip_ssnd);
                    audio_left = len - 8 - offset;
                    limit = true;
                    LOG_INFO("AIFF SSND chunk: %u bytes", audio_left);
                    header_checked = true;
                    return;
                }
            }

            uint64_t skip = 8ULL + (uint64_t)len;
            if (skip > bytes) break;
            ptr += (size_t)skip;
            bytes -= (size_t)skip;
        }
    }

    if (format == UNKNOWN || _buf_used(streambuf) > 4096) {
        header_checked = true;
    }
}

static decode_state pcm_decode(void) {
    LOCK_S;

    if (!header_checked) {
        _check_header();
    }

    unsigned bytes = min(_buf_used(streambuf), _buf_cont_read(streambuf));
    bool toend = (stream.state <= DISCONNECT);

    u32_t bytes_per_frame = channels * sample_size;
    if (bytes_per_frame == 0) bytes_per_frame = 4;

    if (bytes < bytes_per_frame) {
        UNLOCK_S;
        return toend ? DECODE_COMPLETE : DECODE_RUNNING;
    }

    if (limit && audio_left > 0) {
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
        if (max_decode < 1) max_decode = 1;
    } else if (space_frames < max_decode) {
        max_decode = space_frames;
    }

    if (frames > max_decode) frames = max_decode;

    if (frames == 0) {
        UNLOCK_S;
        return DECODE_RUNNING;
    }

    static u8_t s_rawIn[RS_MAX_IN * 8];
    unsigned consumed = frames * bytes_per_frame;
    if (consumed > sizeof(s_rawIn)) {
        frames = sizeof(s_rawIn) / bytes_per_frame;
        consumed = frames * bytes_per_frame;
    }

    memcpy(s_rawIn, streambuf->readp, consumed);
    _buf_inc_readp(streambuf, consumed);
    if (limit && audio_left >= consumed) {
        audio_left -= consumed;
    }
    u32_t out_seq = outputbuf->flush_seq;
    UNLOCK_S;

    const u8_t *iptr = s_rawIn;
    static const float kScale32 = 1.0f / 2147483648.0f;

    for (frames_t i = 0; i < frames; ++i) {
        s32_t sl = 0, sr = 0;
        if (channels == 2) {
            if (sample_size == 1) {
                sl = ((s32_t)*iptr++ - 128) << 24;
                sr = ((s32_t)*iptr++ - 128) << 24;
            } else if (sample_size == 2) {
                if (bigendian) {
                    sl = (s32_t)(((u32_t)iptr[0] << 24) | ((u32_t)iptr[1] << 16));
                    sr = (s32_t)(((u32_t)iptr[2] << 24) | ((u32_t)iptr[3] << 16));
                } else {
                    sl = (s32_t)(((u32_t)iptr[0] << 16) | ((u32_t)iptr[1] << 24));
                    sr = (s32_t)(((u32_t)iptr[2] << 16) | ((u32_t)iptr[3] << 24));
                }
                iptr += 4;
            } else if (sample_size == 3) {
                if (bigendian) {
                    sl = (s32_t)(((u32_t)iptr[0] << 24) | ((u32_t)iptr[1] << 16) | ((u32_t)iptr[2] << 8));
                    sr = (s32_t)(((u32_t)iptr[3] << 24) | ((u32_t)iptr[4] << 16) | ((u32_t)iptr[5] << 8));
                } else {
                    sl = (s32_t)(((u32_t)iptr[0] << 8) | ((u32_t)iptr[1] << 16) | ((u32_t)iptr[2] << 24));
                    sr = (s32_t)(((u32_t)iptr[3] << 8) | ((u32_t)iptr[4] << 16) | ((u32_t)iptr[5] << 24));
                }
                iptr += 6;
            } else if (sample_size == 4) {
                if (bigendian) {
                    sl = (s32_t)(((u32_t)iptr[0] << 24) | ((u32_t)iptr[1] << 16) | ((u32_t)iptr[2] << 8) | (u32_t)iptr[3]);
                    sr = (s32_t)(((u32_t)iptr[4] << 24) | ((u32_t)iptr[5] << 16) | ((u32_t)iptr[6] << 8) | (u32_t)iptr[7]);
                } else {
                    sl = (s32_t)((u32_t)iptr[0] | ((u32_t)iptr[1] << 8) | ((u32_t)iptr[2] << 16) | ((u32_t)iptr[3] << 24));
                    sr = (s32_t)((u32_t)iptr[4] | ((u32_t)iptr[5] << 8) | ((u32_t)iptr[6] << 16) | ((u32_t)iptr[7] << 24));
                }
                iptr += 8;
            }
        } else {
            // Mono
            if (sample_size == 1) {
                sl = sr = ((s32_t)*iptr++ - 128) << 24;
            } else if (sample_size == 2) {
                if (bigendian) sl = sr = (s32_t)(((u32_t)iptr[0] << 24) | ((u32_t)iptr[1] << 16));
                else sl = sr = (s32_t)(((u32_t)iptr[0] << 16) | ((u32_t)iptr[1] << 24));
                iptr += 2;
            } else if (sample_size == 3) {
                if (bigendian) sl = sr = (s32_t)(((u32_t)iptr[0] << 24) | ((u32_t)iptr[1] << 16) | ((u32_t)iptr[2] << 8));
                else sl = sr = (s32_t)(((u32_t)iptr[0] << 8) | ((u32_t)iptr[1] << 16) | ((u32_t)iptr[2] << 24));
                iptr += 3;
            } else if (sample_size == 4) {
                if (bigendian) sl = sr = (s32_t)(((u32_t)iptr[0] << 24) | ((u32_t)iptr[1] << 16) | ((u32_t)iptr[2] << 8) | (u32_t)iptr[3]);
                else sl = sr = (s32_t)((u32_t)iptr[0] | ((u32_t)iptr[1] << 8) | ((u32_t)iptr[2] << 16) | ((u32_t)iptr[3] << 24));
                iptr += 4;
            }
        }
        if (s_pcmInL && s_pcmInR) {
            s_pcmInL[i] = (float)sl * kScale32;
            s_pcmInR[i] = (float)sr * kScale32;
        }
    }

    size_t out_frames = 0;
    float *writeL = s_pcmInL;
    float *writeR = s_pcmInR;
    if (s_pcm_rs && s_pcmInL && s_pcmInR && s_pcmOutL && s_pcmOutR) {
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

            for (size_t i = 0; i < out_frames; ++i) {
                float l = fmaxf(-1.0f, fminf(1.0f, writeL[i]));
                float r = fmaxf(-1.0f, fminf(1.0f, writeR[i]));
                *optr++ = (s32_t)(l * 2147483392.0f);
                *optr++ = (s32_t)(r * 2147483392.0f);
                if (optr >= buf_wrap) {
                    optr = buf_start;
                }
            }
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
    buf_adjust(streambuf, sample_size * channels);
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

struct codec *register_pcm(void) {
    if (!s_pcmInL) {
        s_pcmInL  = (float *)heap_caps_aligned_alloc(16, RS_MAX_IN * sizeof(float), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (!s_pcmInL) s_pcmInL = (float *)heap_caps_malloc(RS_MAX_IN * sizeof(float), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (!s_pcmInL) s_pcmInL = (float *)heap_caps_malloc(RS_MAX_IN * sizeof(float), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

        s_pcmInR  = (float *)heap_caps_aligned_alloc(16, RS_MAX_IN * sizeof(float), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (!s_pcmInR) s_pcmInR = (float *)heap_caps_malloc(RS_MAX_IN * sizeof(float), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (!s_pcmInR) s_pcmInR = (float *)heap_caps_malloc(RS_MAX_IN * sizeof(float), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

        s_pcmOutL = (float *)heap_caps_aligned_alloc(16, RS_OUT_CAP * sizeof(float), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (!s_pcmOutL) s_pcmOutL = (float *)heap_caps_malloc(RS_OUT_CAP * sizeof(float), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (!s_pcmOutL) s_pcmOutL = (float *)heap_caps_malloc(RS_OUT_CAP * sizeof(float), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

        s_pcmOutR = (float *)heap_caps_aligned_alloc(16, RS_OUT_CAP * sizeof(float), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (!s_pcmOutR) s_pcmOutR = (float *)heap_caps_malloc(RS_OUT_CAP * sizeof(float), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (!s_pcmOutR) s_pcmOutR = (float *)heap_caps_malloc(RS_OUT_CAP * sizeof(float), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

        if (!s_pcmInL || !s_pcmInR || !s_pcmOutL || !s_pcmOutR) {
            if (s_pcmInL)  { free(s_pcmInL);  s_pcmInL  = NULL; }
            if (s_pcmInR)  { free(s_pcmInR);  s_pcmInR  = NULL; }
            if (s_pcmOutL) { free(s_pcmOutL); s_pcmOutL = NULL; }
            if (s_pcmOutR) { free(s_pcmOutR); s_pcmOutR = NULL; }
            LOG_ERROR("PCM resampler buffers allocation failed");
            return NULL;
        }
    }
    return &pcm_codec;
}
