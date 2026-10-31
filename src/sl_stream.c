/*
 * Squeezelite stream task ported for ESP32-S3
 * Based on squeezelite (c) Adrian Smith, Ralph Irving
 */
#include "squeezelite.h"

static struct buffer buf;
struct buffer *streambuf = &buf;
struct streamstate stream;

#define LOCK_S   mutex_lock(streambuf->mutex)
#define UNLOCK_S mutex_unlock(streambuf->mutex)

static sockfd fd = -1;
static bool running = true;
static TaskHandle_t stream_task_handle = NULL;

static void _disconnect(stream_state state, disconnect_code disconnect) {
    LOCK_S;
    stream.state = state;
    stream.disconnect = disconnect;
    if (fd >= 0) {
        closesocket(fd);
        fd = -1;
    }
    UNLOCK_S;
    wake_controller();
}

static bool _send_header(void) {
    const char *ptr = stream.header;
    int len = stream.header_len;
    unsigned try_count = 0;

    while (len > 0) {
        int n = send(fd, ptr, len, 0);
        if (n <= 0) {
            if (n < 0 && last_error() == ERROR_WOULDBLOCK && try_count++ < 10) {
                vTaskDelay(pdMS_TO_TICKS(2));
                continue;
            }
            LOG_WARN("Failed writing HTTP header to stream socket: %s", strerror(last_error()));
            _disconnect(DISCONNECT, LOCAL_DISCONNECT);
            return false;
        }
        ptr += n;
        len -= n;
    }
    return true;
}

#define BOUNCE_SIZE 16384
static DRAM_ATTR uint8_t s_rxBounce[BOUNCE_SIZE];

static void stream_task(void *arg) {
    (void)arg;
    LOG_INFO("Stream task started on Core %d", xPortGetCoreID());

    while (running) {
        struct pollfd pollinfo;
        size_t space;
        int cur_fd;
        stream_state cur_state;

        LOCK_S;
        cur_fd = fd;
        cur_state = stream.state;
        space = _buf_space(streambuf);
        UNLOCK_S;

        if (cur_fd < 0 || cur_state <= STREAMING_WAIT) {
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }

        if (space < sizeof(s_rxBounce)) {
            vTaskDelay(pdMS_TO_TICKS(2));
            continue;
        }

        pollinfo.fd = cur_fd;
        pollinfo.events = POLLIN;
        if (cur_state == SEND_HEADERS) {
            pollinfo.events |= POLLOUT;
        }

        if (poll(&pollinfo, 1, 10) > 0) {
            if ((pollinfo.revents & POLLOUT) && cur_state == SEND_HEADERS) {
                LOCK_S;
                if (fd >= 0 && stream.state == SEND_HEADERS) {
                    if (_send_header()) {
                        stream.state = RECV_HEADERS;
                        stream.header_len = 0;
                    }
                }
                UNLOCK_S;
                continue;
            }

            if (pollinfo.revents & (POLLIN | POLLHUP | POLLERR)) {
                if (cur_state == RECV_HEADERS) {
                    LOCK_S;
                    while (stream.state == RECV_HEADERS && fd >= 0) {
                        char c;
                        int n = recv(fd, &c, 1, 0);
                        if (n <= 0) {
                            if (n < 0 && last_error() == ERROR_WOULDBLOCK) {
                                break;
                            }
                            LOG_INFO("Error or socket closed reading HTTP headers: %s", n ? strerror(last_error()) : "closed");
                            UNLOCK_S;
                            _disconnect(STOPPED, LOCAL_DISCONNECT);
                            LOCK_S;
                            break;
                        }

                        if (stream.header_len < MAX_HEADER - 1) {
                            stream.header[stream.header_len++] = c;
                        }

                        if ((stream.header_len >= 4 && !memcmp(stream.header + stream.header_len - 4, "\r\n\r\n", 4)) ||
                            (stream.header_len >= 2 && !memcmp(stream.header + stream.header_len - 2, "\n\n", 2))) {
                            stream.header[stream.header_len] = '\0';
                            LOG_INFO("HTTP response headers complete (%u bytes)", stream.header_len);
                            stream.state = stream.cont_wait ? STREAMING_WAIT : STREAMING_BUFFERING;
                            wake_controller();
                            break;
                        }
                    }
                    UNLOCK_S;
                    continue;
                }

                // Drain socket via internal SRAM bounce buffer WITHOUT holding LOCK_S during recv()
                while (running) {
                    LOCK_S;
                    cur_fd = fd;
                    space = _buf_space(streambuf);
                    UNLOCK_S;

                    if (cur_fd < 0 || space == 0) break;

                    size_t to_read = min(space, (size_t)BOUNCE_SIZE);
                    int n = recv(cur_fd, (char *)s_rxBounce, to_read, 0);

                    if (n > 0) {
                        bool wake = false;
                        LOCK_S;
                        if (fd >= 0) {
                            size_t written = 0;
                            while (written < (size_t)n) {
                                size_t chunk = min((size_t)n - written, _buf_cont_write(streambuf));
                                if (chunk == 0) break;
                                memcpy(streambuf->writep, s_rxBounce + written, chunk);
                                _buf_inc_writep(streambuf, (unsigned)chunk);
                                written += chunk;
                            }
                            stream.bytes += written;
                            if (stream.state == STREAMING_BUFFERING && stream.bytes > stream.threshold) {
                                stream.state = STREAMING_HTTP;
                                wake = true;
                            }
                        }
                        UNLOCK_S;
                        if (wake) wake_controller();
                        continue;
                    }

                    if (n == 0) {
                        LOG_INFO("End of audio stream (%llu bytes received)", stream.bytes);
                        _disconnect(DISCONNECT, DISCONNECT_OK);
                        break;
                    } else {
                        if (last_error() != ERROR_WOULDBLOCK) {
                            LOG_INFO("Stream socket read error: %s", strerror(last_error()));
                            _disconnect(DISCONNECT, REMOTE_DISCONNECT);
                        }
                        break;
                    }
                }
            }
        }
    }

    vTaskDelete(NULL);
}

void stream_init(log_level level, unsigned stream_buf_size) {
    (void)level;
    LOG_INFO("Initializing stream subsystem (buf size: %u KB in PSRAM)", stream_buf_size / 1024);

    buf_init(streambuf, stream_buf_size);
    stream.state = STOPPED;
    stream.header = (char *)malloc(MAX_HEADER);
    if (stream.header) stream.header[0] = '\0';
    stream.header_len = 0;
    stream.sent_headers = false;
    stream.bytes = 0;
    fd = -1;

    running = true;
    xTaskCreatePinnedToCore(
        stream_task,
        "sl_stream",
        8192,
        NULL,
        7,
        &stream_task_handle,
        0
    );
}

void stream_close(void) {
    LOG_INFO("Closing stream subsystem");
    running = false;
    stream_disconnect();

    if (stream_task_handle) {
        vTaskDelay(pdMS_TO_TICKS(50));
        stream_task_handle = NULL;
    }

    if (stream.header) {
        free(stream.header);
        stream.header = NULL;
    }
    buf_destroy(streambuf);
}

void stream_sock(u32_t ip, u16_t port, bool use_ssl, bool use_ogg, const char *header, size_t header_len, unsigned threshold, bool cont_wait) {
    (void)use_ssl;
    (void)use_ogg;

    stream_disconnect();

    struct sockaddr_in addr;
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        LOG_ERROR("Failed to create stream socket");
        return;
    }

    int rxBufSize = 65536; // 64 KB TCP window for high-throughput streaming
    setsockopt(sock, SOL_SOCKET, SO_RCVBUF, &rxBufSize, sizeof(rxBufSize));
    int nodelay = 1;
    setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));

    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = ip;
    addr.sin_port = port;

    LOG_INFO("Connecting to audio stream at %s:%u", inet_ntoa(addr.sin_addr), ntohs(addr.sin_port));

    set_nonblock(sock);

    if (connect_timeout(sock, (struct sockaddr *)&addr, sizeof(addr), 5) < 0) {
        LOG_WARN("Unable to connect to stream server");
        closesocket(sock);
        LOCK_S;
        stream.state = DISCONNECT;
        stream.disconnect = UNREACHABLE;
        UNLOCK_S;
        wake_controller();
        return;
    }

    buf_flush(streambuf);

    LOCK_S;
    fd = sock;
    if (header_len == 0) {
        const char *default_req = "GET /stream.pcm HTTP/1.0\r\n\r\n";
        header = default_req;
        header_len = strlen(default_req);
    }
    stream.state = SEND_HEADERS;
    stream.cont_wait = cont_wait;
    stream.header_len = header_len;
    if (stream.header) {
        memcpy(stream.header, header, header_len);
        stream.header[header_len] = '\0';
    }
    stream.sent_headers = false;
    stream.bytes = 0;
    stream.threshold = threshold;
    UNLOCK_S;

    LOG_INFO("Stream socket connected successfully, threshold = %u bytes", threshold);
}

bool stream_disconnect(void) {
    bool disc = false;
    LOCK_S;
    if (fd != -1) {
        closesocket(fd);
        fd = -1;
        disc = true;
    }
    stream.state = STOPPED;
    UNLOCK_S;
    return disc;
}

