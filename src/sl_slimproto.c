/*
 * Squeezelite SlimProto state machine ported for ESP32-S3
 * Based on squeezelite (c) Adrian Smith, Ralph Irving
 */
#include "squeezelite.h"
#include "esp_wifi.h"

#pragma pack(push, 1)
struct HELO_packet {
    char  opcode[4];
    u32_t length;
    u8_t  deviceid;
    u8_t  revision;
    u8_t  mac[6];
    u8_t  uuid[16];
    u16_t wlan_channellist;
    u32_t bytes_received_H, bytes_received_L;
    char  lang[2];
};

struct STAT_packet {
    char  opcode[4];
    u32_t length;
    char  event[4];
    u8_t  num_crlf;
    u8_t  mas_initialized;
    u8_t  mas_mode;
    u32_t stream_buffer_size;
    u32_t stream_buffer_fullness;
    u32_t bytes_received_H;
    u32_t bytes_received_L;
    u16_t signal_strength;
    u32_t jiffies;
    u32_t output_buffer_size;
    u32_t output_buffer_fullness;
    u32_t elapsed_seconds;
    u16_t voltage;
    u32_t elapsed_milliseconds;
    u32_t server_timestamp;
    u16_t error_code;
};

struct DSCO_packet {
    char  opcode[4];
    u32_t length;
    u8_t  reason;
};

struct RESP_header {
    char  opcode[4];
    u32_t length;
};

struct SETD_header {
    char  opcode[4];
    u32_t length;
    u8_t  id;
};

struct strm_packet {
    char  opcode[4];
    char  command;
    u8_t  autostart;
    u8_t  format;
    u8_t  pcm_sample_size;
    u8_t  pcm_sample_rate;
    u8_t  pcm_channels;
    u8_t  pcm_endianness;
    u8_t  threshold;
    u8_t  spdif_enable;
    u8_t  transition_period;
    u8_t  transition_type;
    u8_t  flags;
    u8_t  output_threshold;
    u8_t  slaves;
    u32_t replay_gain;
    u16_t server_port;
    u32_t server_ip;
};

struct aude_packet {
    char  opcode[4];
    u8_t  enable_spdif;
    u8_t  enable_dac;
};

struct audg_packet {
    char  opcode[4];
    u32_t old_gainL;
    u32_t old_gainR;
    u8_t  adjust;
    u8_t  preamp;
    u32_t gainL;
    u32_t gainR;
};

struct cont_packet {
    char  opcode[4];
    u32_t metaint;
    u8_t  loop;
};

struct serv_packet {
    char  opcode[4];
    u32_t server_ip;
};
#pragma pack(pop)

extern struct buffer *streambuf;
extern struct buffer *outputbuf;
extern struct streamstate stream;
extern struct outputstate output;
extern struct decodestate decode;

#define LOCK_S   mutex_lock(streambuf->mutex)
#define UNLOCK_S mutex_unlock(streambuf->mutex)
#define LOCK_O   mutex_lock(outputbuf->mutex)
#define UNLOCK_O mutex_unlock(outputbuf->mutex)
#define LOCK_D   mutex_lock(decode.mutex)
#define UNLOCK_D mutex_unlock(decode.mutex)

static sockfd control_sock = -1;
static in_addr_t slimproto_ip = 0;
static event_event wake_e;
static TaskHandle_t slimproto_task_handle = NULL;
static bool running = true;

static struct {
    u32_t updated;
    u32_t stream_start;
    u32_t stream_full;
    u32_t stream_size;
    u64_t stream_bytes;
    u32_t output_full;
    u32_t output_size;
    u32_t frames_played;
    u32_t device_frames;
    u32_t current_sample_rate;
    u32_t last;
    stream_state stream_state;
} status;

static int autostart = 1;
static bool sentSTMu = false, sentSTMo = false, sentSTMl = false;
static char player_name[64] = "ESP32-S3-DSP";
static u8_t s_mac[6] = {0};

void wake_controller(void) {
    wake_signal(wake_e);
}

void slimproto_send_packet(const u8_t *packet, size_t len) {
    if (control_sock < 0) return;
    const u8_t *ptr = packet;
    unsigned try_count = 0;

    while (len > 0) {
        int n = send(control_sock, ptr, len, 0);
        if (n <= 0) {
            if (n < 0 && last_error() == ERROR_WOULDBLOCK && try_count++ < 10) {
                vTaskDelay(pdMS_TO_TICKS(2));
                continue;
            }
            LOG_WARN("Failed writing packet to LMS control socket: %s", strerror(last_error()));
            return;
        }
        ptr += n;
        len -= n;
    }
}

static void sendHELO(bool reconnect, const u8_t mac[6]) {
    const char *cap = "Model=squeezelite,AccuratePlayPoints=1,HasDigitalOut=1,HasPolarityInversion=1,Balance=1,Firmware=" VERSION ",ModelName=ESP32-S3-DSP,Rates=44100,flc";  /* FLAC only: forces LMS to compress; raw PCM needs 1.4-2.3 MB/s */    struct HELO_packet pkt;

    memset(&pkt, 0, sizeof(pkt));
    memcpy(pkt.opcode, "HELO", 4);
    pkt.length = htonl(sizeof(struct HELO_packet) - 8 + strlen(cap));
    pkt.deviceid = 12; // SqueezePlay / Squeezelite device ID
    pkt.revision = 0;
    packn(&pkt.wlan_channellist, reconnect ? 0x4000 : 0x0000);
    packN(&pkt.bytes_received_H, (u32_t)((u64_t)status.stream_bytes >> 32));
    packN(&pkt.bytes_received_L, (u32_t)((u64_t)status.stream_bytes & 0xffffffff));
    pkt.lang[0] = 'e';
    pkt.lang[1] = 'n';
    memcpy(pkt.mac, mac, 6);

    LOG_INFO("Sending HELO to LMS: MAC=%02X:%02X:%02X:%02X:%02X:%02X cap=%s",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], cap);

    slimproto_send_packet((const u8_t *)&pkt, sizeof(pkt));
    slimproto_send_packet((const u8_t *)cap, strlen(cap));
}

static void sendSTAT(const char *event, u32_t server_timestamp) {
    struct STAT_packet pkt;
    u32_t now = gettime_ms();
    u32_t ms_played = 0;

    LOCK_O;
    u32_t cur_sample_rate   = output.current_sample_rate;
    u32_t cur_frames_played = output.frames_played;
    u32_t cur_device_frames = output.device_frames;
    u32_t cur_updated       = output.updated;
    u32_t cur_stream_start  = output.track_start_time;
    UNLOCK_O;

    if (cur_sample_rate && cur_frames_played && cur_frames_played > cur_device_frames) {
        ms_played = (u32_t)(((u64_t)(cur_frames_played - cur_device_frames) * 1000ULL) / (u64_t)cur_sample_rate);
        if (now > cur_updated) ms_played += (now - cur_updated);
    } else if (cur_frames_played && now > cur_stream_start) {
        ms_played = now - cur_stream_start;
    }

    memset(&pkt, 0, sizeof(pkt));
    memcpy(pkt.opcode, "STAT", 4);
    pkt.length = htonl(sizeof(struct STAT_packet) - 8);
    memcpy(pkt.event, event, 4);

    u32_t stream_size = streambuf ? streambuf->size : STREAMBUF_SIZE;
    u32_t stream_used = streambuf ? _buf_used(streambuf) : 0;
    u32_t output_size = outputbuf ? outputbuf->size : OUTPUTBUF_SIZE;
    u32_t output_used = outputbuf ? _buf_used(outputbuf) : 0;
    if (output_used > output_size) output_used = output_size;

    LOCK_S;
    u64_t cur_stream_bytes = stream.bytes;
    UNLOCK_S;

    packN(&pkt.stream_buffer_fullness, stream_used);
    packN(&pkt.stream_buffer_size, stream_size);
    packN(&pkt.bytes_received_H, (u32_t)(cur_stream_bytes >> 32));
    packN(&pkt.bytes_received_L, (u32_t)(cur_stream_bytes & 0xffffffff));
    pkt.signal_strength = htons(0xffff);
    packN(&pkt.jiffies, now);
    packN(&pkt.output_buffer_size, output_size);
    packN(&pkt.output_buffer_fullness, output_used);
    packN(&pkt.elapsed_seconds, ms_played / 1000);
    packN(&pkt.elapsed_milliseconds, ms_played);
    pkt.server_timestamp = server_timestamp;

    if (!strcmp(event, "STMt")) {
        LOG_DEBUG("STAT: %s (ms_played=%u, stream_bytes=%llu, out_buf=%u/%u, dev_frames=%u)",
                  event, ms_played, cur_stream_bytes, output_used, output_size, cur_device_frames);
    } else {
        LOG_INFO("STAT: %s (ms_played=%u, stream_bytes=%llu, out_buf=%u/%u, dev_frames=%u)",
                 event, ms_played, cur_stream_bytes, output_used, output_size, cur_device_frames);
    }
    slimproto_send_packet((const u8_t *)&pkt, sizeof(pkt));
}

static void sendDSCO(disconnect_code disconnect) {
    struct DSCO_packet pkt;
    memset(&pkt, 0, sizeof(pkt));
    memcpy(pkt.opcode, "DSCO", 4);
    pkt.length = htonl(sizeof(pkt) - 8);
    pkt.reason = disconnect & 0xFF;

    LOG_INFO("Sending DSCO (reason=%d)", disconnect);
    slimproto_send_packet((const u8_t *)&pkt, sizeof(pkt));
}

static void sendRESP(const char *header, size_t len) {
    struct RESP_header pkt;
    memset(&pkt, 0, sizeof(pkt));
    memcpy(pkt.opcode, "RESP", 4);
    pkt.length = htonl(sizeof(pkt) + len - 8);

    LOG_INFO("Sending RESP (%u bytes header)", len);
    slimproto_send_packet((const u8_t *)&pkt, sizeof(pkt));
    slimproto_send_packet((const u8_t *)header, len);
}

static void sendSETDName(const char *name) {
    struct SETD_header pkt;
    memset(&pkt, 0, sizeof(pkt));
    memcpy(pkt.opcode, "SETD", 4);
    pkt.id = 0;
    pkt.length = htonl(sizeof(pkt) + strlen(name) + 1 - 8);

    LOG_INFO("Sending SETD player name: %s", name);
    slimproto_send_packet((const u8_t *)&pkt, sizeof(pkt));
    slimproto_send_packet((const u8_t *)name, strlen(name) + 1);
}

static void process_strm(u8_t *pkt, int len) {
    struct strm_packet *strm = (struct strm_packet *)pkt;
    if (strm->command == 't') {
        LOG_DEBUG("Command strm: '%c'", strm->command);
    } else {
        LOG_INFO("Command strm: '%c'", strm->command);
    }

    switch (strm->command) {
    case 't':
        sendSTAT("STMt", strm->replay_gain);
        break;

    case 'f': {
        decode_flush(false);
        bool flushed = output_flush_streaming();
        (void)flushed;
        stream_disconnect();
        sendSTAT("STMf", 0);
        buf_flush(streambuf);
        output.stop_time = gettime_ms();
        break;
    }

    case 'q':
        decode_flush(true);
        output_flush();
        status.frames_played = 0;
        stream_disconnect();
        sendSTAT("STMf", 0);
        buf_flush(streambuf);
        output.stop_time = gettime_ms();
        break;

    case 'p': {
        unsigned interval = unpackN(&strm->replay_gain);
        LOCK_O;
        output.pause_frames = interval * status.current_sample_rate / 1000;
        if (interval) {
            output.state = OUTPUT_PAUSE_FRAMES;
        } else if (output.state != OUTPUT_OFF) {
            output.state = OUTPUT_STOPPED;
            output.stop_time = gettime_ms();
        }
        UNLOCK_O;
        if (!interval) sendSTAT("STMp", 0);
        LOG_INFO("Pause interval: %u ms", interval);
        break;
    }

    case 'a': {
        unsigned interval = unpackN(&strm->replay_gain);
        LOCK_O;
        output.skip_frames = interval * status.current_sample_rate / 1000;
        output.state = OUTPUT_SKIP_FRAMES;
        UNLOCK_O;
        LOG_INFO("Skip ahead interval: %u ms", interval);
        break;
    }

    case 'u': {
        unsigned jiffies = unpackN(&strm->replay_gain);
        LOCK_O;
        output.state = jiffies ? OUTPUT_START_AT : OUTPUT_RUNNING;
        output.start_at = jiffies;
        UNLOCK_O;
        LOG_INFO("Unpause at jiffies: %u", jiffies);
        sendSTAT("STMr", 0);
        break;
    }

    case 's': {
        unsigned header_len = (unsigned)len - sizeof(struct strm_packet);
        char *header = (char *)(pkt + sizeof(struct strm_packet));
        in_addr_t ip = (in_addr_t)strm->server_ip;
        u16_t port = strm->server_port;
        if (ip == 0) ip = slimproto_ip;
        if (port == 0) port = htons(9000);

        LOG_INFO("strm 's': fmt='%c', autostart='%c', threshold=%u, trans_type=%c",
                 strm->format, strm->autostart, strm->threshold, strm->transition_type);

        autostart = strm->autostart - '0';
        sendSTAT("STMf", 0);

        if (strm->format != '?') {
            codec_open(strm->format, strm->pcm_sample_size, strm->pcm_sample_rate, strm->pcm_channels, strm->pcm_endianness);
        }

        unsigned stream_thresh = (unsigned)strm->threshold * 1024;
        if (stream_thresh > 32768) stream_thresh = 32768; // Cap input pre-buffer to 32KB so decoder starts immediately
        stream_sock(ip, port, strm->flags & 0x20, false, header, header_len, stream_thresh, autostart >= 2);

        sendSTAT("STMc", 0);
        sentSTMu = sentSTMo = sentSTMl = false;
        status.last = gettime_ms();

        LOCK_O;
        if (output.state == OUTPUT_STOPPED || output.state == OUTPUT_OFF) {
            output.state = OUTPUT_BUFFER;
            output.frames_played = 0;
            buf_flush(outputbuf);
        }
        output.threshold = strm->output_threshold;
        output.next_replay_gain = unpackN(&strm->replay_gain);
        output.fade_mode = (fade_mode)(strm->transition_type - '0');
        output.fade_secs = strm->transition_period;
        output.invert = ((strm->flags & 0x03) == 0x03);
        output.channels = (strm->flags & 0x0c) >> 2;
        UNLOCK_O;
        break;
    }

    default:
        LOG_WARN("Unhandled strm command: '%c'", strm->command);
        break;
    }
}

static void process_cont(u8_t *pkt, int len) {
    (void)len;
    struct cont_packet *cont = (struct cont_packet *)pkt;
    cont->metaint = unpackN(&cont->metaint);
    LOG_INFO("Command cont metaint: %u", cont->metaint);

    if (autostart > 1) {
        autostart -= 2;
        LOCK_S;
        if (stream.state == STREAMING_WAIT) {
            stream.state = STREAMING_BUFFERING;
            stream.meta_interval = stream.meta_next = cont->metaint;
        }
        UNLOCK_S;
        wake_controller();
    }
}

static void process_codc(u8_t *pkt, int len) {
    (void)len;
    struct strm_packet *codc = (struct strm_packet *)pkt;
    LOG_INFO("Command codc format: '%c'", codc->format);
    codec_open(codc->format, codc->pcm_sample_size, codc->pcm_sample_rate, codc->pcm_channels, codc->pcm_endianness);
}

static void process_audg(u8_t *pkt, int len) {
    (void)len;
    struct audg_packet *audg = (struct audg_packet *)pkt;
    u32_t gL = unpackN(&audg->gainL);
    u32_t gR = unpackN(&audg->gainR);
    set_volume(audg->adjust ? gL : FIXED_ONE, audg->adjust ? gR : FIXED_ONE);
}

static void process_aude(u8_t *pkt, int len) {
    (void)len;
    struct aude_packet *aude = (struct aude_packet *)pkt;
    LOG_INFO("Command aude enable_dac=%d enable_spdif=%d", aude->enable_dac, aude->enable_spdif);
    LOCK_O;
    if (!aude->enable_spdif && !aude->enable_dac && output.state != OUTPUT_OFF) {
        output.state = OUTPUT_OFF;
    } else if ((aude->enable_spdif || aude->enable_dac) && output.state == OUTPUT_OFF) {
        output.state = OUTPUT_STOPPED;
        output.stop_time = gettime_ms();
    }
    UNLOCK_O;
}

static void process_setd(u8_t *pkt, int len) {
    struct SETD_header *setd = (struct SETD_header *)pkt;
    if (setd->id == 0) {
        if (len == 5) {
            sendSETDName(player_name);
        } else if (len > 5) {
            const char *name = (const char *)(pkt + sizeof(struct SETD_header));
            strncpy(player_name, name, sizeof(player_name) - 1);
            player_name[sizeof(player_name) - 1] = '\0';
            LOG_INFO("Updated player name to '%s'", player_name);
            sendSETDName(player_name);
        }
    }
}

static void process_serv(u8_t *pkt, int len) {
    (void)len;
    struct serv_packet *serv = (struct serv_packet *)pkt;
    if (serv->server_ip != 0 && serv->server_ip != htonl(1)) {
        slimproto_ip = serv->server_ip;
        LOG_INFO("Switching server to: %s", inet_ntoa(*(struct in_addr *)&slimproto_ip));
    }
}

static void dispatch_packet(u8_t *pack, int len) {
    if (len < 4) return;
    if (!memcmp(pack, "strm", 4) && len >= (int)sizeof(struct strm_packet) && ((struct strm_packet *)pack)->command == 't') {
        LOG_DEBUG("Packet received: '%.4s' (len=%d)", pack, len);
    } else {
        LOG_INFO("Packet received: '%.4s' (len=%d)", pack, len);
    }
    if (!memcmp(pack, "strm", 4)) process_strm(pack, len);
    else if (!memcmp(pack, "cont", 4)) process_cont(pack, len);
    else if (!memcmp(pack, "codc", 4)) process_codc(pack, len);
    else if (!memcmp(pack, "audg", 4)) process_audg(pack, len);
    else if (!memcmp(pack, "aude", 4)) process_aude(pack, len);
    else if (!memcmp(pack, "setd", 4)) process_setd(pack, len);
    else if (!memcmp(pack, "serv", 4)) process_serv(pack, len);
    else {
        LOG_WARN("Unhandled LMS packet opcode: %.4s", pack);
    }
}

static void slimproto_task(void *arg) {
    (void)arg;
    static u8_t buffer[MAX_HEADER];
    int expect = 0;
    int got = 0;
    u32_t last = 0;
    event_handle ehandles[2];
    int timeouts = 0;

    LOG_INFO("SlimProto controller task started on Core %d", xPortGetCoreID());

    while (running) {
        // 1. Connection phase
        while (running && control_sock < 0) {
            if (slimproto_ip == 0) {
                slimproto_ip = discover_server(NULL, 5);
            }

            if (slimproto_ip == 0) {
                vTaskDelay(pdMS_TO_TICKS(2000));
                continue;
            }

            struct sockaddr_in addr;
            int sock = socket(AF_INET, SOCK_STREAM, 0);
            if (sock < 0) {
                vTaskDelay(pdMS_TO_TICKS(1000));
                continue;
            }

            memset(&addr, 0, sizeof(addr));
            addr.sin_family = AF_INET;
            addr.sin_addr.s_addr = slimproto_ip;
            addr.sin_port = htons(3483);

            LOG_INFO("Connecting to LMS control socket at %s:3483...", inet_ntoa(addr.sin_addr));
            set_nonblock(sock);

            if (connect_timeout(sock, (struct sockaddr *)&addr, sizeof(addr), 5) < 0) {
                LOG_WARN("Failed connecting to LMS control socket");
                closesocket(sock);
                vTaskDelay(pdMS_TO_TICKS(2000));
                continue;
            }

            control_sock = sock;
            int nodelay = 1;
            setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));
            LOG_INFO("Connected to LMS control socket!");
            sendHELO(false, s_mac);

            expect = 0;
            got = 0;
            last = gettime_ms();
            timeouts = 0;
        }

        if (!running) break;

        // 2. Event loop
        set_readwake_handles(ehandles, control_sock, wake_e);

        bool wake = false;
        event_type ev = wait_readwake(ehandles, 1000);

        if (ev == EVENT_READ) {
            timeouts = 0;
            if (expect > 0) {
                int n = recv(control_sock, buffer + got, expect, 0);
                if (n <= 0) {
                    if (n < 0 && last_error() == ERROR_WOULDBLOCK) continue;
                    LOG_INFO("Control socket closed or error: %s", n ? strerror(last_error()) : "closed");
                    closesocket(control_sock);
                    control_sock = -1;
                    continue;
                }
                expect -= n;
                got += n;
                if (expect == 0) {
                    dispatch_packet(buffer, got);
                    got = 0;
                }
            } else if (expect == 0) {
                int n = recv(control_sock, buffer + got, 2 - got, 0);
                if (n <= 0) {
                    if (n < 0 && last_error() == ERROR_WOULDBLOCK) continue;
                    LOG_INFO("Control socket closed or error: %s", n ? strerror(last_error()) : "closed");
                    closesocket(control_sock);
                    control_sock = -1;
                    continue;
                }
                got += n;
                if (got == 2) {
                    expect = (buffer[0] << 8) | buffer[1];
                    got = 0;
                    if (expect > (int)sizeof(buffer)) {
                        LOG_ERROR("Packet too large: %d bytes", expect);
                        closesocket(control_sock);
                        control_sock = -1;
                        continue;
                    }
                }
            }
        } else if (ev == EVENT_WAKE) {
            wake = true;
            timeouts = 0;
        } else {
            // Timeout (1 second)
            if (++timeouts > 35) {
                LOG_WARN("No packets from LMS in 35s - reconnecting...");
                closesocket(control_sock);
                control_sock = -1;
                continue;
            }
        }

        // 3. Playback state updates
        u32_t now = gettime_ms();
        if (wake || (now - last > 100) || (last > now)) {
            bool _sendSTMs = false;
            bool _sendDSCO = false;
            bool _sendRESP = false;
            bool _sendSTMd = false;
            bool _sendSTMt = false;
            bool _sendSTMl = false;
            bool _sendSTMu = false;
            bool _sendSTMo = false;
            bool _sendSTMn = false;
            bool _stream_disconnect = false;
            bool _start_output = false;
            disconnect_code disc_code = DISCONNECT_OK;
            static char s_headerCopy[MAX_HEADER];
            size_t s_headerLen = 0;

            last = now;

            LOCK_S;
            status.stream_full  = _buf_used(streambuf);
            status.stream_size  = streambuf->size;
            status.stream_bytes = stream.bytes;
            status.stream_state = stream.state;

            if (stream.state == DISCONNECT) {
                disc_code = stream.disconnect;
                stream.state = STOPPED;
                _sendDSCO = true;
            }

            if (!stream.sent_headers && (stream.state == STREAMING_HTTP || stream.state == STREAMING_WAIT || stream.state == STREAMING_BUFFERING)) {
                s_headerLen = stream.header_len;
                if (s_headerLen > 0 && s_headerLen < MAX_HEADER) {
                    memcpy(s_headerCopy, stream.header, s_headerLen);
                    s_headerCopy[s_headerLen] = '\0';
                    _sendRESP = true;
                }
                stream.sent_headers = true;
            }
            UNLOCK_S;

            LOCK_D;
            if ((status.stream_state == STREAMING_HTTP || status.stream_state == STREAMING_FILE ||
                 (status.stream_state == DISCONNECT && stream.disconnect == DISCONNECT_OK)) &&
                !sentSTMl && decode.state == DECODE_READY) {
                if (autostart == 0) {
                    decode.state = DECODE_RUNNING;
                    _sendSTMl = true;
                    sentSTMl = true;
                } else if (autostart == 1) {
                    decode.state = DECODE_RUNNING;
                    _start_output = true;
                    sentSTMl = true; 
                }
            }

            if (decode.state == DECODE_COMPLETE || decode.state == DECODE_ERROR) {
                if (decode.state == DECODE_COMPLETE) _sendSTMd = true;
                if (decode.state == DECODE_ERROR)    _sendSTMn = true;
                decode.state = DECODE_STOPPED;
                if (status.stream_state == STREAMING_HTTP || status.stream_state == STREAMING_FILE) {
                    _stream_disconnect = true;
                }
            }
            decode_state cur_decode_state = decode.state;
            UNLOCK_D;

            LOCK_O;
            status.output_full = _buf_used(outputbuf);
            status.output_size = outputbuf->size;
            status.frames_played = output.frames_played_dmp;
            status.current_sample_rate = output.current_sample_rate;
            status.updated = output.updated;
            status.device_frames = output.device_frames;

            if (output.track_started) {
                _sendSTMs = true;
                output.track_started = false;
                status.stream_start = output.track_start_time;
            }

            if (_sendSTMl || _start_output) {
                if (output.state != OUTPUT_BUFFER && output.state != OUTPUT_RUNNING) {
                    output.state = OUTPUT_BUFFER;
                }
            }

            if (output.state == OUTPUT_RUNNING && !sentSTMu && status.output_full == 0 &&
                status.stream_state <= DISCONNECT && cur_decode_state == DECODE_STOPPED) {
                _sendSTMu = true;
                sentSTMu = true;
                LOG_INFO("End of track reached: output drained -> STMu");
                output.state = OUTPUT_STOPPED;
                output.stop_time = now;
            }

            static u32_t s_starved_start = 0;
            if (status.output_full == 0 && status.stream_state == STREAMING_HTTP &&
                output.frames_played > 0 && output.state == OUTPUT_RUNNING) {
                if (s_starved_start == 0) {
                    s_starved_start = now;
                } else if (!sentSTMo && (now - s_starved_start > 5000)) {
                    _sendSTMo = true;
                    sentSTMo = true;
                    LOG_WARN("Output buffer persistently starved (>500ms) -> STMo");
                }
            } else {
                s_starved_start = 0;
            }

            if (output.state == OUTPUT_RUNNING && now - status.last > 1000) {
                _sendSTMt = true;
                status.last = now;
            }

            UNLOCK_O;

            if (_stream_disconnect) stream_disconnect();

            if (_sendDSCO) sendDSCO(disc_code);
            if (_sendRESP) sendRESP(s_headerCopy, s_headerLen);
            if (_sendSTMl) sendSTAT("STMl", 0);
            if (_sendSTMs) sendSTAT("STMs", 0);
            if (_sendSTMd) sendSTAT("STMd", 0);
            if (_sendSTMt) sendSTAT("STMt", 0);
            if (_sendSTMu) sendSTAT("STMu", 0);
            if (_sendSTMo) sendSTAT("STMo", 0);
            if (_sendSTMn) sendSTAT("STMn", 0);
        }
    }

    vTaskDelete(NULL);
}

void slimproto_start(log_level level, const char *server, const u8_t mac[6], const char *name, int maxSampleRate) {
    (void)level;
    (void)maxSampleRate;

    esp_wifi_set_ps(WIFI_PS_NONE);

    if (mac) {
        memcpy(s_mac, mac, 6);
    } else {
        esp_wifi_get_mac(WIFI_IF_STA, s_mac);
    }

    if (name) {
        strncpy(player_name, name, sizeof(player_name) - 1);
    }
    if (server && server[0]) {
        slimproto_ip = inet_addr(server);
    }

    _wake_create(&wake_e);
    running = true;

    xTaskCreatePinnedToCore(
        slimproto_task,
        "sl_control",
        8192,
        NULL,
        5,
        &slimproto_task_handle,
        0
    );
}

void slimproto_stop(void) {
    LOG_INFO("Stopping SlimProto subsystem");
    running = false;
    wake_controller();

    if (control_sock >= 0) {
        closesocket(control_sock);
        control_sock = -1;
    }

    _wake_close(&wake_e);
}
