/**
 * @file slimproto_client.cpp
 * @brief Squeezelite-compatible SlimProto and HTTP streaming client for LMS and Roon.
 */
#include "slimproto_client.h"

// -- Slimproto Packet Formats ---------------------------------
#pragma pack(push, 1)

struct HeloPacket {
    char     opcode[4];
    uint32_t length;
    uint8_t  deviceid;
    uint8_t  revision;
    uint8_t  mac[6];
    uint8_t  uuid[16];
    uint16_t wlan_channellist;
    uint32_t bytes_received_H;
    uint32_t bytes_received_L;
    char     lang[2];
};

struct StatPacket {
    char     opcode[4];
    uint32_t length;
    char     event[4];
    uint8_t  num_crlf;
    uint8_t  mas_initialized;
    uint8_t  mas_mode;
    uint32_t stream_buffer_size;
    uint32_t stream_buffer_fullness;
    uint32_t bytes_received_H;
    uint32_t bytes_received_L;
    uint16_t signal_strength;
    uint32_t jiffies;
    uint32_t output_buffer_size;
    uint32_t output_buffer_fullness;
    uint32_t elapsed_seconds;
    uint16_t voltage;
    uint32_t elapsed_milliseconds;
    uint32_t server_timestamp;
    uint16_t error_code;
};

struct StrmPacket {
    char     opcode[4];
    char     command;
    uint8_t  autostart;
    uint8_t  format;
    uint8_t  pcm_sample_size;
    uint8_t  pcm_sample_rate;
    uint8_t  pcm_channels;
    uint8_t  pcm_endianness;
    uint8_t  threshold;
    uint8_t  spdif_enable;
    uint8_t  transition_period;
    uint8_t  transition_type;
    uint8_t  flags;
    uint8_t  output_threshold;
    uint8_t  slaves;
    uint32_t replay_gain;
    uint16_t server_port;
    uint32_t server_ip;
};

struct AudgPacket {
    char     opcode[4];
    uint32_t old_gainL;
    uint32_t old_gainR;
    uint8_t  adjust;
    uint8_t  preamp;
    uint32_t gainL;
    uint32_t gainR;
};

struct SetdPacket {
    char     opcode[4];
    uint32_t length;
    uint8_t  id;
};

#pragma pack(pop)

// -------------------------------------------------------------
SlimprotoClient::SlimprotoClient(ParamStore& params, AudioRingBuffer<16384>& ringBuffer)
    : _params(params), _ringBuffer(ringBuffer)
{
}

SlimprotoClient::~SlimprotoClient()
{
    stop();
}

// -------------------------------------------------------------
bool SlimprotoClient::begin()
{
    if (_running) return true;
    _running = true;

    BaseType_t ret = xTaskCreatePinnedToCore(
        _taskTrampoline,
        "SlimProtoTask",
        8192,
        this,
        2,
        &_taskHandle,
        0 // Core 0 (Network and System core)
    );

    return (ret == pdPASS);
}

// -------------------------------------------------------------
void SlimprotoClient::stop()
{
    _running = false;
    _stopAudioStream();

    if (_controlSock >= 0) {
        close(_controlSock);
        _controlSock = -1;
    }

    _connected = false;
    _streaming = false;
    _paused = false;

    if (_taskHandle) {
        vTaskDelete(_taskHandle);
        _taskHandle = nullptr;
    }
}

// -------------------------------------------------------------
void SlimprotoClient::sendPause()
{
    if (!_connected || !_streaming) return;
    _paused = true;
    _sendStat("STMp");
    _updateMetadata(true, true, true, "Paused");
}

void SlimprotoClient::sendResume()
{
    if (!_connected || !_streaming) return;
    _paused = false;
    _sendStat("STMr");
    _updateMetadata(true, true, false, "Streaming PCM");
}

void SlimprotoClient::sendStop()
{
    if (!_connected) return;
    _stopAudioStream();
    _sendStat("STMf");
    _updateMetadata(true, false, false, "Connected (Idle)");
}

// -------------------------------------------------------------
void SlimprotoClient::_taskTrampoline(void* arg)
{
    SlimprotoClient* self = static_cast<SlimprotoClient*>(arg);
    self->_runTask();
}

// -------------------------------------------------------------
void SlimprotoClient::_runTask()
{
    Serial.println("[SLIMPROTO] SlimProto / Squeezelite client task started on Core 0");

    uint32_t lastStatTime = 0;

    while (_running) {
        // Wait until Wi-Fi is connected in station mode
        if (WiFi.status() != WL_CONNECTED || (WiFi.getMode() & WIFI_MODE_STA) == 0) {
            if (_connected) {
                _connected = false;
                _stopAudioStream();
                if (_controlSock >= 0) {
                    close(_controlSock);
                    _controlSock = -1;
                }
                _updateMetadata(false, false, false, "Wi-Fi Disconnected");
            }
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        // If not connected to LMS / Roon, run discovery and connect
        if (!_connected) {
            _updateMetadata(false, false, false, "Searching for server...");
            if (_discoverServer()) {
                if (_connectControl()) {
                    _connected = true;
                    _sendHelo();
                    _updateMetadata(true, false, false, "Connected to Server");
                    lastStatTime = millis();
                } else {
                    vTaskDelay(pdMS_TO_TICKS(3000));
                }
            } else {
                vTaskDelay(pdMS_TO_TICKS(4000));
            }
            continue;
        }

        // Handle control socket events
        fd_set readFds;
        FD_ZERO(&readFds);
        int maxFd = _controlSock;
        FD_SET(_controlSock, &readFds);

        if (_streaming && _streamSock >= 0) {
            FD_SET(_streamSock, &readFds);
            if (_streamSock > maxFd) maxFd = _streamSock;
        }

        struct timeval tv;
        tv.tv_sec  = 0;
        tv.tv_usec = 25000; // 25 ms timeout

        int sel = select(maxFd + 1, &readFds, nullptr, nullptr, &tv);

        if (sel > 0) {
            if (FD_ISSET(_controlSock, &readFds)) {
                if (!_handleControlPacket()) {
                    Serial.println("[SLIMPROTO] Control socket closed or error, reconnecting...");
                    _connected = false;
                    _stopAudioStream();
                    close(_controlSock);
                    _controlSock = -1;
                    _updateMetadata(false, false, false, "Connection Lost");
                    vTaskDelay(pdMS_TO_TICKS(1000));
                    continue;
                }
            }

            if (_streaming && _streamSock >= 0 && FD_ISSET(_streamSock, &readFds)) {
                _readAudioStreamData();
            }
        } else if (sel < 0) {
            if (errno != EINTR && errno != EAGAIN && errno != EWOULDBLOCK) {
                Serial.printf("[SLIMPROTO] Select error: %d, closing sockets\n", errno);
                _connected = false;
                _stopAudioStream();
                if (_controlSock >= 0) {
                    close(_controlSock);
                    _controlSock = -1;
                }
                vTaskDelay(pdMS_TO_TICKS(1000));
                continue;
            }
        }

        // If streaming and buffer has space, pull audio data without waiting on select
        if (_streaming && _streamSock >= 0 && !_paused) {
            if (_ringBuffer.space() > 1024) {
                _readAudioStreamData();
            }
        }

        // Periodic STAT heartbeat every 1000 ms
        uint32_t now = millis();
        if (_connected && (now - lastStatTime >= 1000)) {
            lastStatTime = now;
            _sendStat("STMt");
        }
    }
}

// -------------------------------------------------------------
bool SlimprotoClient::_discoverServer()
{
    int discSock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (discSock < 0) {
        Serial.println("[SLIMPROTO] Failed to create UDP discovery socket");
        return false;
    }

    int broadcastEnable = 1;
    setsockopt(discSock, SOL_SOCKET, SO_BROADCAST, &broadcastEnable, sizeof(broadcastEnable));

    struct timeval tv;
    tv.tv_sec  = 1;
    tv.tv_usec = 500000; // 1.5 seconds timeout
    setsockopt(discSock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    struct sockaddr_in destAddr;
    memset(&destAddr, 0, sizeof(destAddr));
    destAddr.sin_family      = AF_INET;
    destAddr.sin_port        = htons(3483);
    destAddr.sin_addr.s_addr = htonl(INADDR_BROADCAST);

    // Standard discovery packet: 'e' followed by "JSON" \0 "CLIP"
    const char discPacket[] = "eJSON\0CLIP";
    const size_t discPacketLen = 10;

    Serial.println("[SLIMPROTO] Broadcasting UDP discovery on port 3483...");
    if (sendto(discSock, discPacket, discPacketLen, 0, (struct sockaddr*)&destAddr, sizeof(destAddr)) < 0) {
        Serial.println("[SLIMPROTO] Discovery broadcast send failed");
        close(discSock);
        return false;
    }

    char rxBuf[128];
    struct sockaddr_in srcAddr;
    socklen_t srcLen = sizeof(srcAddr);

    int bytesRead = recvfrom(discSock, rxBuf, sizeof(rxBuf) - 1, 0, (struct sockaddr*)&srcAddr, &srcLen);
    close(discSock);

    if (bytesRead > 0) {
        rxBuf[bytesRead] = '\0';
        _serverIp = IPAddress(srcAddr.sin_addr.s_addr);
        Serial.printf("[SLIMPROTO] Discovered LMS / Roon server at: %s:%d\n",
                      _serverIp.toString().c_str(), ntohs(srcAddr.sin_port));
        return true;
    }

    return false;
}

// -------------------------------------------------------------
bool SlimprotoClient::_connectControl()
{
    if (_controlSock >= 0) {
        close(_controlSock);
        _controlSock = -1;
    }

    _controlSock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (_controlSock < 0) {
        Serial.println("[SLIMPROTO] Failed to create TCP control socket");
        return false;
    }

    int nodelay = 1;
    setsockopt(_controlSock, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));

    struct timeval tv;
    tv.tv_sec  = 3;
    tv.tv_usec = 0;
    setsockopt(_controlSock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    setsockopt(_controlSock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

    struct sockaddr_in serverAddr;
    memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_family      = AF_INET;
    serverAddr.sin_port        = htons(_controlPort);
    serverAddr.sin_addr.s_addr = static_cast<uint32_t>(_serverIp);

    Serial.printf("[SLIMPROTO] Connecting to control port %s:%d...\n",
                  _serverIp.toString().c_str(), _controlPort);

    if (connect(_controlSock, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) < 0) {
        Serial.println("[SLIMPROTO] Control connection failed");
        close(_controlSock);
        _controlSock = -1;
        return false;
    }

    Serial.println("[SLIMPROTO] Connected to SlimProto server!");
    return true;
}

// -------------------------------------------------------------
void SlimprotoClient::_sendHelo()
{
    if (_controlSock < 0) return;

    // Capabilities announcement: identify as Squeezelite player named ESP32-S3-DSP
    const char* baseCap = "Model=squeezelite,AccuratePlayPoints=1,HasDigitalOut=1,HasPolarityInversion=1,Firmware=v1.0";
    const char* customCap = ",ModelName=ESP32-S3-DSP,MaxSampleRate=48000,pcm";

    size_t capLen = strlen(baseCap) + strlen(customCap);

    HeloPacket pkt;
    memset(&pkt, 0, sizeof(pkt));
    memcpy(pkt.opcode, "HELO", 4);
    pkt.length            = htonl(sizeof(HeloPacket) - 8 + capLen);
    pkt.deviceid          = 12; // Squeezebox Receiver / Squeezelite
    pkt.revision          = 0;
    pkt.wlan_channellist  = htons(0x4000);
    pkt.bytes_received_H  = 0;
    pkt.bytes_received_L  = 0;
    pkt.lang[0]           = 'e';
    pkt.lang[1]           = 'n';

    uint8_t mac[6];
    WiFi.macAddress(mac);
    memcpy(pkt.mac, mac, 6);

    send(_controlSock, &pkt, sizeof(pkt), 0);
    send(_controlSock, baseCap, strlen(baseCap), 0);
    send(_controlSock, customCap, strlen(customCap), 0);

    Serial.printf("[SLIMPROTO] Sent HELO packet (MAC: %02X:%02X:%02X:%02X:%02X:%02X)\n",
                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

// -------------------------------------------------------------
void SlimprotoClient::_sendStat(const char* eventCode, uint32_t serverTimestamp)
{
    if (_controlSock < 0) return;

    StatPacket pkt;
    memset(&pkt, 0, sizeof(pkt));
    memcpy(pkt.opcode, "STAT", 4);
    pkt.length = htonl(sizeof(StatPacket) - 8);
    memcpy(pkt.event, eventCode, 4);

    uint32_t now = millis();
    uint32_t elapsedMs = (_streamStartTime > 0 && now >= _streamStartTime) ? (now - _streamStartTime) : 0;

    pkt.stream_buffer_size     = htonl(_ringBuffer.capacity());
    pkt.stream_buffer_fullness = htonl(_ringBuffer.available());
    pkt.output_buffer_size     = htonl(_ringBuffer.capacity());
    pkt.output_buffer_fullness = htonl(_ringBuffer.available());
    pkt.bytes_received_H       = 0;
    pkt.bytes_received_L       = htonl(_bytesReceived);
    pkt.signal_strength        = htons(static_cast<uint16_t>(WiFi.RSSI() & 0xFFFF));
    pkt.jiffies                = htonl(now);
    pkt.elapsed_seconds        = htonl(elapsedMs / 1000);
    pkt.elapsed_milliseconds   = htonl(elapsedMs);
    pkt.server_timestamp       = serverTimestamp; // Already in server byte order

    send(_controlSock, &pkt, sizeof(pkt), 0);
}

// -------------------------------------------------------------
bool SlimprotoClient::_handleControlPacket()
{
    // Server packets start with a 16-bit big-endian length prefix
    uint8_t lenBuf[2];
    int n = recv(_controlSock, lenBuf, 2, MSG_WAITALL);
    if (n != 2) return false;

    uint16_t payloadLen = (static_cast<uint16_t>(lenBuf[0]) << 8) | lenBuf[1];
    if (payloadLen == 0 || payloadLen > 4096) {
        Serial.printf("[SLIMPROTO] Invalid packet length: %u\n", payloadLen);
        return false;
    }

    uint8_t payload[4096];
    int bytesRead = 0;
    while (bytesRead < payloadLen) {
        int r = recv(_controlSock, payload + bytesRead, payloadLen - bytesRead, 0);
        if (r <= 0) return false;
        bytesRead += r;
    }

    if (payloadLen >= 4) {
        if (memcmp(payload, "strm", 4) == 0) {
            _processStrm(payload, payloadLen);
        } else if (memcmp(payload, "audg", 4) == 0) {
            _processAudg(payload, payloadLen);
        } else if (memcmp(payload, "setd", 4) == 0) {
            _processSetd(payload, payloadLen);
        } else if (memcmp(payload, "aude", 4) == 0) {
            Serial.println("[SLIMPROTO] Received aude (audio enable)");
        } else {
            char op[5] = {0};
            memcpy(op, payload, 4);
            Serial.printf("[SLIMPROTO] Received opcode: %s\n", op);
        }
    }

    return true;
}

// -------------------------------------------------------------
void SlimprotoClient::_processStrm(const uint8_t* payload, size_t len)
{
    if (len < sizeof(StrmPacket)) return;
    const StrmPacket* strm = reinterpret_cast<const StrmPacket*>(payload);

    Serial.printf("[SLIMPROTO] STRM command: '%c' (format: '%c')\n", strm->command, strm->format);

    switch (strm->command) {
        case 't': // Heartbeat ping
            _sendStat("STMt", strm->replay_gain);
            break;

        case 'q': // Stop
        case 'f': // Flush
            _stopAudioStream();
            _ringBuffer.clear();
            _sendStat("STMf");
            _updateMetadata(true, false, false, "Stopped");
            break;

        case 'p': // Pause
            _paused = true;
            _sendStat("STMp");
            _updateMetadata(true, true, true, "Paused");
            break;

        case 'u': // Resume
            _paused = false;
            _sendStat("STMr");
            _updateMetadata(true, true, false, "Streaming PCM");
            break;

        case 's': // Start stream
        {
            _stopAudioStream();
            _ringBuffer.clear();
            _bytesReceived = 0;
            _streamStartTime = millis();

            // Decode stream sample rate
            switch (strm->pcm_sample_rate) {
                case '0': _sampleRate = 11025; break;
                case '1': _sampleRate = 22050; break;
                case '2': _sampleRate = 31000; break;
                case '3': _sampleRate = 44100; break;
                case '4': _sampleRate = 48000; break;
                case '5': _sampleRate = 8000;  break;
                case '6': _sampleRate = 12000; break;
                case '7': _sampleRate = 16000; break;
                case '8': _sampleRate = 24000; break;
                case '9': _sampleRate = 96000; break;
                default:  _sampleRate = 44100; break;
            }

            // Decode bit depth
            switch (strm->pcm_sample_size) {
                case '0': _bitDepth = 16; break;
                case '1': _bitDepth = 24; break;
                case '2': _bitDepth = 32; break;
                case '3': _bitDepth = 8;  break;
                default:  _bitDepth = 16; break;
            }

            _channels  = (strm->pcm_channels == '1') ? 1 : 2;
            _bigEndian = (strm->pcm_endianness == '1');

            uint32_t sIp = strm->server_ip;
            if (sIp != 0) {
                _serverIp = IPAddress(sIp);
            }
            _streamPort = ntohs(strm->server_port);

            size_t reqLen = len - sizeof(StrmPacket);
            String httpRequest = "";
            if (reqLen > 0) {
                const char* reqPtr = reinterpret_cast<const char*>(payload + sizeof(StrmPacket));
                httpRequest = String(reqPtr).substring(0, reqLen);
            }

            Serial.printf("[SLIMPROTO] Audio format: PCM %u Hz, %u-bit, %u ch, %s-endian\n",
                          _sampleRate, _bitDepth, _channels, _bigEndian ? "big" : "little");
            Serial.printf("[SLIMPROTO] Stream target: %s:%d\n", _serverIp.toString().c_str(), _streamPort);

            _sendStat("STMf");
            _sendStat("STMs");

            if (_startAudioStream(httpRequest)) {
                _streaming = true;
                _paused = false;
                _sendStat("STMo");
                _updateMetadata(true, true, false, "Streaming PCM");
            } else {
                _sendStat("STMu");
            }
            break;
        }

        default:
            break;
    }
}

// -------------------------------------------------------------
void SlimprotoClient::_processAudg(const uint8_t* payload, size_t len)
{
    if (len < sizeof(AudgPacket)) return;
    const AudgPacket* audg = reinterpret_cast<const AudgPacket*>(payload);

    uint32_t gainL = ntohl(audg->gainL);
    uint32_t gainR = ntohl(audg->gainR);

    // Slimproto volume is 16.16 fixed-point (65536 = 1.0 = 0 dB)
    float g = (gainL > 0) ? (static_cast<float>(gainL) / 65536.0f) : 0.0f;
    float db = (g > 0.0001f) ? (20.0f * log10f(g)) : -60.0f;

    Serial.printf("[SLIMPROTO] AUDG volume update: gainL=%u (%.1f dB)\n", gainL, db);
}

// -------------------------------------------------------------
void SlimprotoClient::_processSetd(const uint8_t* payload, size_t len)
{
    if (len < 5) return;
    uint8_t id = payload[4];

    // id == 0: Query / Set Player Name
    if (id == 0) {
        const char name[] = "ESP32-S3-DSP";
        size_t nameLen = strlen(name) + 1;

        SetdPacket pkt;
        memset(&pkt, 0, sizeof(pkt));
        memcpy(pkt.opcode, "SETD", 4);
        pkt.id     = 0;
        pkt.length = htonl(sizeof(SetdPacket) + nameLen - 8);

        send(_controlSock, &pkt, sizeof(pkt), 0);
        send(_controlSock, name, nameLen, 0);
        Serial.println("[SLIMPROTO] Replied to SETD name query with 'ESP32-S3-DSP'");
    }
}

// -------------------------------------------------------------
bool SlimprotoClient::_startAudioStream(const String& httpRequest)
{
    _stopAudioStream();

    _streamSock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (_streamSock < 0) {
        Serial.println("[SLIMPROTO] Failed to create audio stream socket");
        return false;
    }

    int nodelay = 1;
    setsockopt(_streamSock, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));

    // Increase socket receive buffer to 16 KB for network burst absorption
    int rxBufSize = 16384;
    setsockopt(_streamSock, SOL_SOCKET, SO_RCVBUF, &rxBufSize, sizeof(rxBufSize));

    struct timeval tv;
    tv.tv_sec  = 3;
    tv.tv_usec = 0;
    setsockopt(_streamSock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    struct sockaddr_in streamAddr;
    memset(&streamAddr, 0, sizeof(streamAddr));
    streamAddr.sin_family      = AF_INET;
    streamAddr.sin_port        = htons(_streamPort);
    streamAddr.sin_addr.s_addr = static_cast<uint32_t>(_serverIp);

    if (connect(_streamSock, (struct sockaddr*)&streamAddr, sizeof(streamAddr)) < 0) {
        Serial.println("[SLIMPROTO] Audio stream socket connection failed");
        close(_streamSock);
        _streamSock = -1;
        return false;
    }

    // Send HTTP GET request
    if (httpRequest.length() > 0) {
        send(_streamSock, httpRequest.c_str(), httpRequest.length(), 0);
    } else {
        String defaultGet = "GET /stream.pcm HTTP/1.0\r\nConnection: close\r\n\r\n";
        send(_streamSock, defaultGet.c_str(), defaultGet.length(), 0);
    }

    // Read HTTP response header until "\r\n\r\n" or "\n\n"
    char headerBuf[512];
    int headerPos = 0;
    bool headerEnd = false;

    while (!headerEnd && headerPos < sizeof(headerBuf) - 1) {
        char c;
        int n = recv(_streamSock, &c, 1, 0);
        if (n <= 0) break;
        headerBuf[headerPos++] = c;
        if (headerPos >= 4 &&
            headerBuf[headerPos - 4] == '\r' && headerBuf[headerPos - 3] == '\n' &&
            headerBuf[headerPos - 2] == '\r' && headerBuf[headerPos - 1] == '\n') {
            headerEnd = true;
        } else if (headerPos >= 2 &&
                   headerBuf[headerPos - 2] == '\n' && headerBuf[headerPos - 1] == '\n') {
            headerEnd = true;
        }
    }

    headerBuf[headerPos] = '\0';
    Serial.println("[SLIMPROTO] HTTP stream headers received, playback commencing");

    // Reset resampler state
    _resampPhase = 0.0f;
    _lastSampleL = 0.0f;
    _lastSampleR = 0.0f;

    return true;
}

// -------------------------------------------------------------
void SlimprotoClient::_stopAudioStream()
{
    if (_streamSock >= 0) {
        close(_streamSock);
        _streamSock = -1;
    }
    _streaming = false;
    _paused = false;
}

// -------------------------------------------------------------
void SlimprotoClient::_readAudioStreamData()
{
    if (_streamSock < 0 || _paused) return;

    // Buffer to read incoming raw PCM bytes
    uint8_t rawBuf[1024];
    int bytesRead = recv(_streamSock, rawBuf, sizeof(rawBuf), MSG_DONTWAIT);

    if (bytesRead <= 0) {
        if (bytesRead < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            return;
        }
        Serial.println("[SLIMPROTO] Audio stream reached end of file or closed");
        _stopAudioStream();
        _sendStat("STMu");
        _updateMetadata(true, false, false, "Track Ended");
        return;
    }

    _bytesReceived += bytesRead;

    // Decode PCM bytes to stereo float samples [-1.0f, +1.0f]
    float decodedL[512];
    float decodedR[512];
    size_t decodedFrames = 0;

    if (_bitDepth == 16) {
        size_t bytesPerFrame = _channels * 2;
        size_t frameCount = bytesRead / bytesPerFrame;
        if (frameCount > 512) frameCount = 512;

        for (size_t i = 0; i < frameCount; ++i) {
            size_t offset = i * bytesPerFrame;
            int16_t sL = 0;
            int16_t sR = 0;

            if (_bigEndian) {
                sL = static_cast<int16_t>((rawBuf[offset] << 8) | rawBuf[offset + 1]);
                if (_channels == 2) {
                    sR = static_cast<int16_t>((rawBuf[offset + 2] << 8) | rawBuf[offset + 3]);
                } else {
                    sR = sL;
                }
            } else {
                sL = static_cast<int16_t>(rawBuf[offset] | (rawBuf[offset + 1] << 8));
                if (_channels == 2) {
                    sR = static_cast<int16_t>(rawBuf[offset + 2] | (rawBuf[offset + 3] << 8));
                } else {
                    sR = sL;
                }
            }

            decodedL[i] = static_cast<float>(sL) / 32768.0f;
            decodedR[i] = static_cast<float>(sR) / 32768.0f;
        }
        decodedFrames = frameCount;
    } else if (_bitDepth == 24) {
        size_t bytesPerFrame = _channels * 3;
        size_t frameCount = bytesRead / bytesPerFrame;
        if (frameCount > 512) frameCount = 512;

        for (size_t i = 0; i < frameCount; ++i) {
            size_t offset = i * bytesPerFrame;
            int32_t sL = 0;
            int32_t sR = 0;

            if (_bigEndian) {
                sL = static_cast<int32_t>((rawBuf[offset] << 24) | (rawBuf[offset + 1] << 16) | (rawBuf[offset + 2] << 8)) >> 8;
                if (_channels == 2) {
                    sR = static_cast<int32_t>((rawBuf[offset + 3] << 24) | (rawBuf[offset + 4] << 16) | (rawBuf[offset + 5] << 8)) >> 8;
                } else {
                    sR = sL;
                }
            } else {
                sL = static_cast<int32_t>((rawBuf[offset + 2] << 24) | (rawBuf[offset + 1] << 16) | (rawBuf[offset] << 8)) >> 8;
                if (_channels == 2) {
                    sR = static_cast<int32_t>((rawBuf[offset + 5] << 24) | (rawBuf[offset + 4] << 16) | (rawBuf[offset + 3] << 8)) >> 8;
                } else {
                    sR = sL;
                }
            }

            decodedL[i] = static_cast<float>(sL) / 8388608.0f;
            decodedR[i] = static_cast<float>(sR) / 8388608.0f;
        }
        decodedFrames = frameCount;
    }

    if (decodedFrames == 0) return;

    // Resample if incoming rate is 44.1 kHz (CD standard) to 48.0 kHz (DSP Engine standard)
    if (_sampleRate == 44100) {
        float resampledL[512];
        float resampledR[512];
        size_t outFrames = 0;
        const float step = 44100.0f / 48000.0f; // 0.91875f

        for (size_t i = 0; i < decodedFrames; ++i) {
            float curL = decodedL[i];
            float curR = decodedR[i];

            while (_resampPhase < 1.0f && outFrames < 512) {
                resampledL[outFrames] = _lastSampleL + _resampPhase * (curL - _lastSampleL);
                resampledR[outFrames] = _lastSampleR + _resampPhase * (curR - _lastSampleR);
                outFrames++;
                _resampPhase += step;
            }

            if (_resampPhase >= 1.0f) {
                _resampPhase -= 1.0f;
            }
            _lastSampleL = curL;
            _lastSampleR = curR;
        }

        if (outFrames > 0) {
            _ringBuffer.write(resampledL, resampledR, outFrames);
        }
    } else {
        // Direct 48.0 kHz 1:1 lossless feed
        _ringBuffer.write(decodedL, decodedR, decodedFrames);
    }
}

// -------------------------------------------------------------
void SlimprotoClient::_updateMetadata(bool connected, bool streaming, bool paused, const char* statusMsg)
{
    WiFiStreamMetadata meta;
    memset(&meta, 0, sizeof(meta));

    if (connected) {
        strncpy(meta.serverIp, _serverIp.toString().c_str(), sizeof(meta.serverIp) - 1);
        snprintf(meta.serverName, sizeof(meta.serverName) - 1, "%s", _serverIp.toString().c_str());
    } else {
        strncpy(meta.serverName, "Disconnected", sizeof(meta.serverName) - 1);
        strncpy(meta.serverIp, "", sizeof(meta.serverIp) - 1);
    }

    meta.connected = connected;
    meta.streaming = streaming;
    meta.paused    = paused;
    meta.sampleRate = _sampleRate;
    meta.bitDepth   = _bitDepth;

    if (streaming) {
        snprintf(meta.formatInfo, sizeof(meta.formatInfo) - 1,
                 "PCM %u kHz / %u-bit -> 48kHz DSP",
                 _sampleRate / 1000, _bitDepth);
        strncpy(meta.trackTitle, statusMsg, sizeof(meta.trackTitle) - 1);
        strncpy(meta.trackArtist, "Wi-Fi Stream", sizeof(meta.trackArtist) - 1);
        strncpy(meta.trackAlbum, meta.serverName, sizeof(meta.trackAlbum) - 1);
    } else {
        snprintf(meta.formatInfo, sizeof(meta.formatInfo) - 1, "%s", statusMsg);
        strncpy(meta.trackTitle, statusMsg, sizeof(meta.trackTitle) - 1);
        strncpy(meta.trackArtist, "ESP32-S3-DSP Squeezebox Endpoint", sizeof(meta.trackArtist) - 1);
        strncpy(meta.trackAlbum, "", sizeof(meta.trackAlbum) - 1);
    }

    _params.setWiFiStreamMetadata(meta);
}

