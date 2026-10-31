/*
 * Squeezelite port for ESP32-S3-DSP
 * Based on squeezelite-esp32 (c) Adrian Smith, Ralph Irving, Philippe G.
 */
#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/poll.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"

#ifdef __cplusplus
extern "C" {
#endif

#define VERSION "v1.0-esp32s3"
#define MODEL_NAME SqueezeLite-ESP32-S3
#define MODEL_NAME_STRING "SqueezeLite-ESP32-S3"

typedef uint8_t   u8_t;
typedef uint16_t  u16_t;
typedef uint32_t  u32_t;
typedef uint64_t  u64_t;
typedef int16_t   s16_t;
typedef int32_t   s32_t;
typedef int64_t   s64_t;
typedef uint32_t  frames_t;
typedef int       sockfd;

#define BYTES_PER_FRAME 8
#define ISAMPLE_T       s32_t

#define STREAMBUF_SIZE (2 * 1024 * 1024)   // 2 MB in Octal PSRAM
#define OUTPUTBUF_SIZE (2 * 1024 * 1024)   // 2 MB in Octal PSRAM (262,144 frames = 5.94s @ 44.1kHz)
#define MAX_HEADER 4096
#define MAX_SILENCE_FRAMES 2048
#define FIXED_ONE 0x10000

#ifndef __cplusplus
#ifndef min
#define min(a,b) (((a) < (b)) ? (a) : (b))
#endif
#ifndef max
#define max(a,b) (((a) > (b)) ? (a) : (b))
#endif
#endif

#define closesocket(s) close(s)
#define last_error() errno
#define ERROR_WOULDBLOCK EWOULDBLOCK

#define mutex_type SemaphoreHandle_t
#define mutex_create(m) m = xSemaphoreCreateRecursiveMutex()
#define mutex_create_p(m) mutex_create(m)
#define mutex_lock(m) xSemaphoreTakeRecursive(m, portMAX_DELAY)
#define mutex_unlock(m) xSemaphoreGiveRecursive(m)
#define mutex_destroy(m) if (m) { vSemaphoreDelete(m); m = NULL; }

#define thread_type TaskHandle_t

// Loopback wake mechanism
struct wake {
    int mfds;
    int fds[2];
};
#define event_event struct wake
#define event_handle struct pollfd
#define wake_create(e) _wake_create(&e)
#define wake_signal(e) send(e.fds[1], ".", 1, 0)
#define wake_clear(fd) do { char _c; recv(fd, &_c, 1, 0); } while(0)
#define wake_close(e) _wake_close(&e)

typedef enum { lERROR = 0, lWARN, lINFO, lDEBUG, lSDEBUG } log_level;
extern log_level loglevel;

const char *logtime(void);
void logprint(const char *fmt, ...);

#define LOG_ERROR(fmt, ...) logprint("[SLIMPROTO] ERROR: " fmt "\n", ##__VA_ARGS__)
#define LOG_WARN(fmt, ...)  if (loglevel >= lWARN)  logprint("[SLIMPROTO] WARN: " fmt "\n", ##__VA_ARGS__)
#define LOG_INFO(fmt, ...)  if (loglevel >= lINFO)  logprint("[SLIMPROTO] INFO: " fmt "\n", ##__VA_ARGS__)
#define LOG_DEBUG(fmt, ...) if (loglevel >= lDEBUG) logprint("[SLIMPROTO] DEBUG: " fmt "\n", ##__VA_ARGS__)
#define LOG_SDEBUG(fmt, ...) if (loglevel >= lSDEBUG) logprint("[SLIMPROTO] SDEBUG: " fmt "\n", ##__VA_ARGS__)

// buffer.c
struct buffer {
    u8_t *buf;
    u8_t *readp;
    u8_t *writep;
    u8_t *wrap;
    size_t size;
    size_t base_size;
    mutex_type mutex;
};

unsigned _buf_used(struct buffer *buf);
unsigned _buf_space(struct buffer *buf);
unsigned _buf_cont_read(struct buffer *buf);
unsigned _buf_cont_write(struct buffer *buf);
void _buf_inc_readp(struct buffer *buf, unsigned by);
void _buf_inc_writep(struct buffer *buf, unsigned by);
void buf_flush(struct buffer *buf);
void _buf_flush(struct buffer *buf);
void buf_adjust(struct buffer *buf, size_t mod);
void buf_init(struct buffer *buf, size_t size);
void buf_destroy(struct buffer *buf);

// stream.c
typedef enum {
    STOPPED = 0,
    DISCONNECT,
    STREAMING_WAIT,
    STREAMING_BUFFERING,
    STREAMING_FILE,
    STREAMING_HTTP,
    SEND_HEADERS,
    RECV_HEADERS
} stream_state;

typedef enum {
    DISCONNECT_OK = 0,
    LOCAL_DISCONNECT = 1,
    REMOTE_DISCONNECT = 2,
    UNREACHABLE = 3,
    TIMEOUT = 4
} disconnect_code;

struct streamstate {
    stream_state state;
    disconnect_code disconnect;
    char *header;
    size_t header_len;
    bool sent_headers;
    bool cont_wait;
    u64_t bytes;
    unsigned threshold;
    u32_t meta_interval;
    u32_t meta_next;
    u32_t meta_left;
    bool  meta_send;
};

void stream_init(log_level level, unsigned stream_buf_size);
void stream_close(void);
void stream_sock(u32_t ip, u16_t port, bool use_ssl, bool use_ogg, const char *header, size_t header_len, unsigned threshold, bool cont_wait);
bool stream_disconnect(void);

// decode.c
typedef enum {
    DECODE_STOPPED = 0,
    DECODE_READY,
    DECODE_RUNNING,
    DECODE_COMPLETE,
    DECODE_ERROR
} decode_state;

struct decodestate {
    decode_state state;
    bool new_stream;
    mutex_type mutex;
};

#define MAX_CODECS 4

struct codec {
    char id;
    const char *types;
    unsigned min_read_bytes;
    unsigned min_space;
    void (*open)(u8_t sample_size, u8_t sample_rate, u8_t channels, u8_t endianness);
    void (*close)(void);
    decode_state (*decode)(void);
};

void decode_init(log_level level, const char *include_codecs, const char *exclude_codecs);
void decode_close(void);
void decode_flush(bool close);
void codec_open(u8_t format, u8_t sample_size, u8_t sample_rate, u8_t channels, u8_t endianness);

struct codec *register_pcm(void);
struct codec *register_flac(void);

// output.c
typedef enum {
    OUTPUT_OFF = -1,
    OUTPUT_STOPPED = 0,
    OUTPUT_BUFFER,
    OUTPUT_RUNNING,
    OUTPUT_PAUSE_FRAMES,
    OUTPUT_SKIP_FRAMES,
    OUTPUT_START_AT
} output_state;

typedef enum { S32_LE, S16_LE } output_format;
typedef enum { FADE_INACTIVE = 0, FADE_DUE, FADE_ACTIVE } fade_state;
typedef enum { FADE_UP = 1, FADE_DOWN, FADE_CROSS } fade_dir;
typedef enum { FADE_NONE = 0, FADE_CROSSFADE, FADE_IN, FADE_OUT, FADE_INOUT } fade_mode;

#define MONO_RIGHT 0x02
#define MONO_LEFT  0x01

struct outputstate {
    output_state state;
    output_format format;
    u8_t  channels;
    bool  track_started;
    int (* write_cb)(frames_t out_frames, bool silence, s32_t gainL, s32_t gainR, u8_t flags, s32_t cross_gain_in, s32_t cross_gain_out, ISAMPLE_T **cross_ptr);
    unsigned start_frames;
    unsigned frames_played;
    unsigned frames_played_dmp;
    unsigned current_sample_rate;
    unsigned device_frames;
    u32_t updated;
    u32_t track_start_time;
    u32_t current_replay_gain;
    union {
        u32_t pause_frames;
        u32_t skip_frames;
        u32_t start_at;
    };
    unsigned next_sample_rate;
    u8_t  *track_start;
    u32_t gainL;
    u32_t gainR;
    bool  invert;
    u32_t next_replay_gain;
    unsigned threshold;
    fade_state fade;
    u8_t *fade_start;
    u8_t *fade_end;
    fade_dir fade_dir;
    fade_mode fade_mode;
    unsigned fade_secs;
    u32_t stop_time;
};

void output_init(log_level level, unsigned output_buf_size, unsigned rates[]);
void output_close(void);
void output_flush(void);
bool output_flush_streaming(void);
void output_pause(void);
void output_resume(void);
frames_t _output_frames(frames_t avail);
void _checkfade(bool start);
void set_volume(unsigned left, unsigned right);

// Direct output frame consumer (called by AudioTask / I2S master)
size_t squeezelite_output_frames(float *left, float *right, size_t frames);
size_t squeezelite_output_available(void);

// output_pack.c
void _apply_gain(struct buffer *outputbuf, frames_t count, s32_t gainL, s32_t gainR, u8_t flags);
void _apply_cross(struct buffer *outputbuf, frames_t out_frames, s32_t cross_gain_in, s32_t cross_gain_out, ISAMPLE_T **cross_ptr);
s32_t gain(s32_t gain, s32_t sample);
s32_t to_gain(float f);

// slimproto.c
void slimproto_start(log_level level, const char *server, const u8_t mac[6], const char *name, int maxSampleRate);
void slimproto_stop(void);
void wake_controller(void);
void slimproto_send_packet(const u8_t *packet, size_t len);

// utils.c
typedef enum { EVENT_TIMEOUT = 0, EVENT_READ, EVENT_WAKE } event_type;

u32_t gettime_ms(void);
void set_nonblock(sockfd s);
int connect_timeout(sockfd sock, const struct sockaddr *addr, socklen_t addrlen, int timeout);
void set_readwake_handles(event_handle handles[], sockfd s, event_event e);
event_type wait_readwake(event_handle handles[], int timeout);
void _wake_create(event_event *e);
void _wake_close(event_event *e);
void packN(u32_t *dest, u32_t val);
void packn(u16_t *dest, u16_t val);
u32_t unpackN(const u32_t *src);
u16_t unpackn(const u16_t *src);
in_addr_t discover_server(char *default_server, int max);

#ifdef __cplusplus
}
#endif
