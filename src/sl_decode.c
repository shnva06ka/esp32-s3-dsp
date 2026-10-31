/*
 * Squeezelite decode scheduling ported for ESP32-S3
 * Based on squeezelite (c) Adrian Smith, Ralph Irving
 */
#include "squeezelite.h"

extern struct buffer *streambuf;
extern struct buffer *outputbuf;
extern struct streamstate stream;
extern struct outputstate output;

struct decodestate decode;
static struct codec *codecs[MAX_CODECS];
static struct codec *codec = NULL;
bool decode_running = true;
static TaskHandle_t decode_task_handle = NULL;

#define LOCK_S   mutex_lock(streambuf->mutex)
#define UNLOCK_S mutex_unlock(streambuf->mutex)
#define LOCK_O   mutex_lock(outputbuf->mutex)
#define UNLOCK_O mutex_unlock(outputbuf->mutex)
#define LOCK_D   mutex_lock(decode.mutex)
#define UNLOCK_D mutex_unlock(decode.mutex)

static void decode_task(void *arg) {
    (void)arg;
    LOG_INFO("Decode task started on Core %d", xPortGetCoreID());

    while (decode_running) {
        size_t bytes, space;
        bool toend;
        bool ran = false;

        LOCK_S;
        bytes = _buf_used(streambuf);
        toend = (stream.state <= DISCONNECT);
        UNLOCK_S;

        LOCK_O;
        space = _buf_space(outputbuf);
        UNLOCK_O;

        LOCK_D;
        if (decode.state == DECODE_RUNNING && codec) {
            if (space > codec->min_space && (bytes > codec->min_read_bytes || toend)) {
                decode.state = codec->decode();

                if (decode.state != DECODE_RUNNING) {
                    LOG_INFO("Decode %s", decode.state == DECODE_COMPLETE ? "complete" : "error");
                    LOCK_O;
                    if (output.fade_mode) _checkfade(false);
                    UNLOCK_O;
                    wake_controller();
                }

                ran = true;
            }
        }
        UNLOCK_D;

        if (!ran) {
            vTaskDelay(pdMS_TO_TICKS(2));
        } else {
            taskYIELD();
        }
    }

    vTaskDelete(NULL);
}

void decode_init(log_level level, const char *include_codecs, const char *exclude_codecs) {
    (void)level;
    (void)include_codecs;
    (void)exclude_codecs;

    LOG_INFO("Initializing decode subsystem");

    mutex_create(decode.mutex);
    decode.new_stream = true;
    decode.state = DECODE_STOPPED;

    codecs[0] = register_pcm();
    codecs[1] = register_flac();
    codecs[2] = NULL;
    codec = NULL;

    decode_running = true;
    xTaskCreatePinnedToCore(
        decode_task,
        "sl_decode",
        16384,
        NULL,
        5,
        &decode_task_handle,
        1
    );
}

void decode_close(void) {
    LOG_INFO("Closing decode subsystem");
    decode_running = false;
    LOCK_D;
    if (codec) {
        codec->close();
        codec = NULL;
    }
    UNLOCK_D;

    if (decode_task_handle) {
        vTaskDelay(pdMS_TO_TICKS(50));
        decode_task_handle = NULL;
    }
    mutex_destroy(decode.mutex);
}

void decode_flush(bool close) {
    LOG_INFO("Decode flush (close=%d)", close);
    LOCK_D;
    decode.state = DECODE_STOPPED;
    if (close && codec) {
        codec->close();
        codec = NULL;
    }
    UNLOCK_D;
}

void codec_open(u8_t format, u8_t sample_size, u8_t sample_rate, u8_t channels, u8_t endianness) {
    LOG_INFO("Codec open: format='%c'", format);

    LOCK_D;
    decode.new_stream = true;
    decode.state = DECODE_STOPPED;

    for (int i = 0; i < MAX_CODECS; ++i) {
        if (codecs[i] && (codecs[i]->id == format || (codecs[i]->id == 'p' && (format == 'w' || format == 'a' || format == 'p')))) {
            if (codec && codec != codecs[i]) {
                LOG_INFO("Closing old codec: '%c'", codec->id);
                codec->close();
            }
            codec = codecs[i];
            codec->open(sample_size, sample_rate, channels, endianness);
            decode.state = DECODE_READY;
            UNLOCK_D;
            return;
        }
    }

    UNLOCK_D;
    LOG_ERROR("Codec for '%c' not found!", format);
}
