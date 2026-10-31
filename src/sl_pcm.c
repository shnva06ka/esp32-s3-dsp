/*
 * Squeezelite PCM/WAV/AIFF codec ported for ESP32-S3
 * Based on squeezelite (c) Adrian Smith, Ralph Irving
 */
#include "squeezelite.h"

extern struct buffer *streambuf;
extern struct buffer *outputbuf;
extern struct streamstate stream;
extern struct outputstate output;
extern struct decodestate decode;

#define LOCK_S   mutex_lock(streambuf->mutex)
#define UNLOCK_S mutex_unlock(streambuf->mutex)
#define LOCK_O   mutex_lock(outputbuf->mutex)
#define UNLOCK_O mutex_unlock(outputbuf->mutex)

#define MAX_DECODE_FRAMES 1024

static u32_t sample_size = 2; // bytes per sample per channel
static u32_t channels = 2;
static bool  bigendian = false;
static bool  limit = false;
static u32_t audio_left = 0;
static bool  header_checked = false;

typedef enum { UNKNOWN = 0, WAVE, AIFF } header_format;

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
                sample_size = ((u32_t)ptr[22] | ((u32_t)ptr[23] << 8)) / 8;
                bigendian   = false;
                LOG_INFO("WAV fmt: %u ch, 44100 Hz, %u bytes/sample", channels, sample_size);
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
                LOG_INFO("AIFF COMM: %u ch, %u bytes/sample", channels, sample_size);
            }

            if (format == AIFF && !memcmp(ptr, "SSND", 4) && bytes >= 16) {
                unsigned offset = ((u32_t)ptr[8] << 24) | ((u32_t)ptr[9] << 16) | ((u32_t)ptr[10] << 8) | (u32_t)ptr[11];
                ptr += 16;
                _buf_inc_readp(streambuf, (unsigned)(ptr + offset - streambuf->readp));
                audio_left = len - 8 - offset;
                limit = true;
                LOG_INFO("AIFF SSND chunk: %u bytes", audio_left);
                header_checked = true;
                return;
            }

            unsigned skip = 8 + len;
            if (skip > bytes) break;
            ptr += skip;
            bytes -= skip;
        }
    }

    header_checked = true;
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
        output.next_sample_rate = 44100;
        output.track_start = outputbuf->writep;
        output.format = S32_LE;
        decode.new_stream = false;
        UNLOCK_O;
    }

    LOCK_O;
    frames_t space_frames = min(_buf_space(outputbuf), _buf_cont_write(outputbuf)) / BYTES_PER_FRAME;
    if (space_frames < frames) frames = space_frames;
    u8_t *out_start = outputbuf->writep;
    ISAMPLE_T *optr = (ISAMPLE_T *)(void *)out_start;
    UNLOCK_O;

    if (frames == 0) {
        UNLOCK_S;
        return DECODE_RUNNING;
    }

    // Convert WITHOUT holding LOCK_S/LOCK_O: the rings are single-producer /
    // single-consumer, so the regions being read (stream) and written (output)
    // are owned by this task until committed below. Holding the locks here
    // stalled the real-time audio task and the socket drain.
    const u8_t *iptr = streambuf->readp;
    const u8_t *in_start = iptr;
    UNLOCK_S;
    frames_t count = frames;

    if (channels == 2) {
        if (sample_size == 1) {
            // 8-bit unsigned stereo
            while (count--) {
                *optr++ = ((s32_t)*iptr++ - 128) << 24;
                *optr++ = ((s32_t)*iptr++ - 128) << 24;
            }
        } else if (sample_size == 2) {
            // 16-bit stereo
            if (bigendian) {
                while (count--) {
                    *optr++ = (s32_t)(((u32_t)iptr[0] << 24) | ((u32_t)iptr[1] << 16));
                    *optr++ = (s32_t)(((u32_t)iptr[2] << 24) | ((u32_t)iptr[3] << 16));
                    iptr += 4;
                }
            } else {
                while (count--) {
                    *optr++ = (s32_t)(((u32_t)iptr[0] << 16) | ((u32_t)iptr[1] << 24));
                    *optr++ = (s32_t)(((u32_t)iptr[2] << 16) | ((u32_t)iptr[3] << 24));
                    iptr += 4;
                }
            }
        } else if (sample_size == 3) {
            // 24-bit stereo (packed 3 bytes per sample)
            if (bigendian) {
                while (count--) {
                    *optr++ = (s32_t)(((u32_t)iptr[0] << 24) | ((u32_t)iptr[1] << 16) | ((u32_t)iptr[2] << 8));
                    *optr++ = (s32_t)(((u32_t)iptr[3] << 24) | ((u32_t)iptr[4] << 16) | ((u32_t)iptr[5] << 8));
                    iptr += 6;
                }
            } else {
                while (count--) {
                    *optr++ = (s32_t)(((u32_t)iptr[0] << 8) | ((u32_t)iptr[1] << 16) | ((u32_t)iptr[2] << 24));
                    *optr++ = (s32_t)(((u32_t)iptr[3] << 8) | ((u32_t)iptr[4] << 16) | ((u32_t)iptr[5] << 24));
                    iptr += 6;
                }
            }
        } else if (sample_size == 4) {
            // 32-bit stereo
            if (bigendian) {
                while (count--) {
                    *optr++ = (s32_t)(((u32_t)iptr[0] << 24) | ((u32_t)iptr[1] << 16) | ((u32_t)iptr[2] << 8) | (u32_t)iptr[3]);
                    *optr++ = (s32_t)(((u32_t)iptr[4] << 24) | ((u32_t)iptr[5] << 16) | ((u32_t)iptr[6] << 8) | (u32_t)iptr[7]);
                    iptr += 8;
                }
            } else {
                while (count--) {
                    *optr++ = (s32_t)((u32_t)iptr[0] | ((u32_t)iptr[1] << 8) | ((u32_t)iptr[2] << 16) | ((u32_t)iptr[3] << 24));
                    *optr++ = (s32_t)((u32_t)iptr[4] | ((u32_t)iptr[5] << 8) | ((u32_t)iptr[6] << 16) | ((u32_t)iptr[7] << 24));
                    iptr += 8;
                }
            }
        }
    } else {
        // Mono (channels == 1)
        if (sample_size == 1) {
            while (count--) {
                s32_t v = ((s32_t)*iptr++ - 128) << 24;
                *optr++ = v;
                *optr++ = v;
            }
        } else if (sample_size == 2) {
            if (bigendian) {
                while (count--) {
                    s32_t v = (s32_t)(((u32_t)iptr[0] << 24) | ((u32_t)iptr[1] << 16));
                    iptr += 2;
                    *optr++ = v;
                    *optr++ = v;
                }
            } else {
                while (count--) {
                    s32_t v = (s32_t)(((u32_t)iptr[0] << 16) | ((u32_t)iptr[1] << 24));
                    iptr += 2;
                    *optr++ = v;
                    *optr++ = v;
                }
            }
        } else if (sample_size == 3) {
            if (bigendian) {
                while (count--) {
                    s32_t v = (s32_t)(((u32_t)iptr[0] << 24) | ((u32_t)iptr[1] << 16) | ((u32_t)iptr[2] << 8));
                    iptr += 3;
                    *optr++ = v;
                    *optr++ = v;
                }
            } else {
                while (count--) {
                    s32_t v = (s32_t)(((u32_t)iptr[0] << 8) | ((u32_t)iptr[1] << 16) | ((u32_t)iptr[2] << 24));
                    iptr += 3;
                    *optr++ = v;
                    *optr++ = v;
                }
            }
        } else if (sample_size == 4) {
            if (bigendian) {
                while (count--) {
                    s32_t v = (s32_t)(((u32_t)iptr[0] << 24) | ((u32_t)iptr[1] << 16) | ((u32_t)iptr[2] << 8) | (u32_t)iptr[3]);
                    iptr += 4;
                    *optr++ = v;
                    *optr++ = v;
                }
            } else {
                while (count--) {
                    s32_t v = (s32_t)((u32_t)iptr[0] | ((u32_t)iptr[1] << 8) | ((u32_t)iptr[2] << 16) | ((u32_t)iptr[3] << 24));
                    iptr += 4;
                    *optr++ = v;
                    *optr++ = v;
                }
            }
        }
    }

    unsigned consumed = frames * bytes_per_frame;

    // Commit. If a flush reset either buffer while converting, drop the result.
    LOCK_S;
    if (streambuf->readp != in_start) {
        UNLOCK_S;
        return DECODE_RUNNING;
    }
    LOCK_O;
    if (outputbuf->writep != out_start) {
        UNLOCK_O;
        UNLOCK_S;
        return DECODE_RUNNING;
    }
    _buf_inc_writep(outputbuf, frames * BYTES_PER_FRAME);
    UNLOCK_O;
    _buf_inc_readp(streambuf, consumed);
    if (limit && audio_left >= consumed) {
        audio_left -= consumed;
    }
    UNLOCK_S;

    if (limit && audio_left == 0) {
        return DECODE_COMPLETE;
    }

    return DECODE_RUNNING;
}

static void pcm_open(u8_t size, u8_t rate, u8_t chan, u8_t endianness) {
    (void)rate;
    sample_size = size - '0' + 1;
    channels    = (chan >= '1') ? (chan - '0') : 2;
    bigendian   = (endianness == '0');

    limit = false;
    audio_left = 0;
    header_checked = false;

    LOG_INFO("Opening PCM codec: size=%u (%u bits) rate=44100 chan=%u bigendian=%u",
             sample_size, sample_size * 8, channels, bigendian);
    buf_adjust(streambuf, sample_size * channels);
}

static void pcm_close(void) {
    LOG_INFO("Closing PCM codec");
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
    return &pcm_codec;
}
