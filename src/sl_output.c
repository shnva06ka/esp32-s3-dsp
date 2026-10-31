/*
 * Squeezelite output subsystem ported for ESP32-S3
 * Based on squeezelite (c) Adrian Smith, Ralph Irving
 * Directly feeds synchronized 44.1 kHz stereo float PCM to the DSP/I2S pipeline
 * Hardware I2S DMA clock is the master clock pacing the entire pipeline
 */
#include "squeezelite.h"

static struct buffer buf;
struct buffer *outputbuf = &buf;
struct outputstate output;
extern struct streamstate stream;

#define LOCK_O   mutex_lock(outputbuf->mutex)
#define UNLOCK_O mutex_unlock(outputbuf->mutex)

static float *s_destLeft = NULL;
static float *s_destRight = NULL;

static int _direct_write_frames(frames_t out_frames, bool silence, s32_t gainL, s32_t gainR, u8_t flags,
                                s32_t cross_gain_in, s32_t cross_gain_out, ISAMPLE_T **cross_ptr) {
    if (!s_destLeft || !s_destRight) return 0;

    if (!silence) {
        if (output.fade == FADE_ACTIVE && output.fade_dir == FADE_CROSS && *cross_ptr) {
            _apply_cross(outputbuf, out_frames, cross_gain_in, cross_gain_out, cross_ptr);
        }
        _apply_gain(outputbuf, out_frames, gainL, gainR, flags);

        const s32_t *src = (const s32_t *)outputbuf->readp;
        for (frames_t i = 0; i < out_frames; ++i) {
            float l = (float)src[i * 2]     * (1.0f / 2147483648.0f);
            float r = (float)src[i * 2 + 1] * (1.0f / 2147483648.0f);
            s_destLeft[i]  = fmaxf(-1.0f, fminf(1.0f, l));
            s_destRight[i] = fmaxf(-1.0f, fminf(1.0f, r));
        }
    } else {
        memset(s_destLeft,  0, out_frames * sizeof(float));
        memset(s_destRight, 0, out_frames * sizeof(float));
    }

    s_destLeft += out_frames;
    s_destRight += out_frames;
    return (int)out_frames;
}

frames_t _output_frames(frames_t avail) {
    frames_t frames, size;
    bool silence = false;
    u8_t flags = output.channels;
    s32_t cross_gain_in = 0, cross_gain_out = 0;
    ISAMPLE_T *cross_ptr = NULL;

    s32_t gainL = output.current_replay_gain ? gain(output.gainL, output.current_replay_gain) : output.gainL;
    s32_t gainR = output.current_replay_gain ? gain(output.gainR, output.current_replay_gain) : output.gainR;
    if (output.invert) { gainL = -gainL; gainR = -gainR; }

    frames = _buf_used(outputbuf) / BYTES_PER_FRAME;

    // Check pre-buffer threshold
    if (output.state == OUTPUT_BUFFER) {
        unsigned min_thresh = 132300;
        unsigned thresh = (output.threshold * output.next_sample_rate / 10);
        if (thresh < min_thresh) thresh = min_thresh;
        if (thresh > 220500) thresh = 220500;
        if (frames >= thresh) {
            output.state = OUTPUT_RUNNING;
            LOG_INFO("Pre-buffer threshold reached (%u frames) -> STARTING PLAYBACK", (unsigned)frames);
            wake_controller();
        } else {
            return 0;
        }
    }

    // While stopped or off, do not output frames
    if (output.state < OUTPUT_BUFFER) {
        return 0;
    }

    // Skip ahead
    if (output.state == OUTPUT_SKIP_FRAMES) {
        if (frames > 0) {
            frames_t skip = min(frames, output.skip_frames);
            frames -= skip;
            output.frames_played += skip;
            while (skip > 0) {
                frames_t cont_frames = min(skip, _buf_cont_read(outputbuf) / BYTES_PER_FRAME);
                skip -= cont_frames;
                _buf_inc_readp(outputbuf, cont_frames * BYTES_PER_FRAME);
            }
        }
        output.state = OUTPUT_RUNNING;
    }

    // Pause frames
    if (output.state == OUTPUT_PAUSE_FRAMES) {
        if (output.pause_frames == 0) {
            output.state = OUTPUT_RUNNING;
        } else {
            silence = true;
            frames = min(avail, output.pause_frames);
            output.pause_frames -= frames;
        }
    }

    // Start at jiffies
    if (output.state == OUTPUT_START_AT) {
        u32_t now = gettime_ms();
        if (now >= output.start_at || output.start_at > now + 10000) {
            output.state = OUTPUT_RUNNING;
        } else {
            u32_t delta_frames = ((output.start_at - now) * output.current_sample_rate) / 1000;
            silence = true;
            frames = min(avail, delta_frames);
        }
    }

    if (frames == 0 && !silence) {
        return 0;
    }

    frames = min(frames, avail);
    size = frames;

    while (size > 0) {
        frames_t out_frames;
        frames_t cont_frames = _buf_cont_read(outputbuf) / BYTES_PER_FRAME;

        if (output.track_start && !silence) {
            if (output.track_start == outputbuf->readp) {
                LOG_INFO("Track start frame reached DAC boundary @ %u Hz (replay_gain=%u)",
                         output.next_sample_rate, output.next_replay_gain);
                output.frames_played = 0;
                output.track_started = true;
                output.track_start_time = gettime_ms();
                output.current_sample_rate = output.next_sample_rate;
                output.current_replay_gain = output.next_replay_gain;
                output.track_start = NULL;
                wake_controller();
            } else if (output.track_start > outputbuf->readp) {
                cont_frames = min(cont_frames, (frames_t)((output.track_start - outputbuf->readp) / BYTES_PER_FRAME));
            }
        }

        out_frames = !silence ? min(size, cont_frames) : size;

        int wrote = output.write_cb(out_frames, silence, gainL, gainR, flags, cross_gain_in, cross_gain_out, &cross_ptr);
        if (wrote <= 0) {
            frames -= size;
            break;
        }
        out_frames = (frames_t)wrote;
        size -= out_frames;

        if (!silence) {
            _buf_inc_readp(outputbuf, out_frames * BYTES_PER_FRAME);
            output.frames_played += out_frames;
        }
    }

    return frames;
}

size_t squeezelite_output_frames(float *left, float *right, size_t frames) {
    if (!left || !right || frames == 0) return 0;

    if (!outputbuf || !outputbuf->mutex || !outputbuf->buf) {
        memset(left,  0, frames * sizeof(float));
        memset(right, 0, frames * sizeof(float));
        return 0;
    }

    LOCK_O;
    if (output.state < OUTPUT_BUFFER) {
        UNLOCK_O;
        memset(left,  0, frames * sizeof(float));
        memset(right, 0, frames * sizeof(float));
        return 0;
    }

    output.updated = gettime_ms();
    output.frames_played_dmp = output.frames_played;
    output.device_frames = 2048; // I2S DMA FIFO latency (8 * 256 frames = ~46.4ms)

    s_destLeft  = left;
    s_destRight = right;

    frames_t wrote = _output_frames((frames_t)frames);

    if (wrote < (frames_t)frames) {
        memset(left + wrote,  0, (frames - wrote) * sizeof(float));
        memset(right + wrote, 0, (frames - wrote) * sizeof(float));
    }

    UNLOCK_O;
    return (size_t)wrote;
}

size_t squeezelite_output_available(void) {
    return outputbuf ? (_buf_used(outputbuf) / BYTES_PER_FRAME) : 0;
}

void _checkfade(bool start) {
    (void)start;
    output.fade = FADE_INACTIVE;
}

void set_volume(unsigned left, unsigned right) {
    LOG_INFO("Setting SlimProto digital volume: L=%u R=%u (0x10000 = unity)", left, right);
    LOCK_O;
    output.gainL = left;
    output.gainR = right;
    UNLOCK_O;
}

void output_init(log_level level, unsigned output_buf_size, unsigned rates[]) {
    (void)level;
    (void)rates;

    if (outputbuf && outputbuf->buf != NULL) {
        LOG_INFO("Output subsystem already initialized");
        return;
    }

    LOG_INFO("Initializing output subsystem (buf size: %u KB in PSRAM)", output_buf_size / 1024);

    buf_init(outputbuf, output_buf_size);

    memset(&output, 0, sizeof(output));
    output.state = OUTPUT_STOPPED;
    output.format = S32_LE;
    output.current_sample_rate = 44100;
    output.next_sample_rate = 44100;
    output.threshold = 30; // Default 3.0s pre-buffer
    output.start_frames = 132300; // 3.0s pre-buffer at 44.1kHz
    output.gainL = FIXED_ONE;
    output.gainR = FIXED_ONE;
    output.write_cb = &_direct_write_frames;
}

void output_close(void) {
    LOG_INFO("Closing output subsystem");
    output_flush();
    buf_destroy(outputbuf);
}

void output_flush(void) {
    LOG_INFO("Output flush");
    buf_flush(outputbuf);
    LOCK_O;
    output.state = OUTPUT_STOPPED;
    output.stop_time = gettime_ms();
    output.frames_played = 0;
    output.track_start = NULL;
    output.track_started = false;
    UNLOCK_O;
}

void output_pause(void) {
    LOG_INFO("Output pause");
    LOCK_O;
    output.state = OUTPUT_STOPPED;
    output.stop_time = gettime_ms();
    UNLOCK_O;
    wake_controller();
}

void output_resume(void) {
    LOG_INFO("Output resume");
    LOCK_O;
    output.state = OUTPUT_RUNNING;
    UNLOCK_O;
    wake_controller();
}

bool output_flush_streaming(void) {
    LOG_INFO("Output flush streaming");
    LOCK_O;
    bool flushed = (output.track_start != NULL);
    if (output.track_start) {
        outputbuf->writep = output.track_start;
        output.track_start = NULL;
    }
    UNLOCK_O;
    return flushed;
}
