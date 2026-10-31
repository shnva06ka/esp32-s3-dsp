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
static drflac_int32 s_flacDecBuf[1024 * 2];

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

        LOG_INFO("FLAC stream opened: %u channels, %u Hz, %u bits/sample",
                 pFlac->channels, pFlac->sampleRate, pFlac->bitsPerSample);
    }

    if (decode.new_stream) {
        LOCK_O;
        output.next_sample_rate = 44100;
        output.format = S32_LE;
        output.track_start = outputbuf->writep;
        decode.new_stream = false;
        UNLOCK_O;
    }

    frames_t max_decode = 1024;
    LOCK_O;
    frames_t space_frames = min(_buf_space(outputbuf), _buf_cont_write(outputbuf)) / BYTES_PER_FRAME;
    UNLOCK_O;
    if (space_frames < max_decode) max_decode = space_frames;

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

    LOCK_O;
    ISAMPLE_T *optr = (ISAMPLE_T *)(void *)outputbuf->writep;

    const drflac_int32 *src = s_flacDecBuf;
    frames_t count = (frames_t)framesRead;

    // drflac_read_pcm_frames_s32 outputs 32-bit MSB-aligned signed PCM samples
    if (pFlac->channels > 1) {
        while (count--) {
            *optr++ = (s32_t)(*src++);
            *optr++ = (s32_t)(*src++);
        }
    } else {
        while (count--) {
            s32_t val = (s32_t)(*src++);
            *optr++ = val;
            *optr++ = val;
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
    return &flac_codec;
}
