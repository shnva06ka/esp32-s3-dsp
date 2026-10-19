/**
 * @file slimproto_client.h
 * @brief Squeezelite-compatible SlimProto and HTTP streaming client for LMS and Roon.
 *
 * Implements Squeezebox SlimProto client protocol:
 * - UDP discovery on port 3483 (finds Logitech Media Server, Lyrion, or Roon Core)
 * - Persistent TCP control connection on port 3483 (HELO, STAT keepalive, strm, audg)
 * - Secondary TCP HTTP connection to stream lossless uncompressed PCM audio
 * - Real-time linear interpolation resampler from 44.1 kHz to 48.0 kHz
 * - Writes stereo float frames [-1.0f, +1.0f] into the shared AudioRingBuffer
 */
#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <lwip/sockets.h>
#include <lwip/netdb.h>
#include "audio_ring_buffer.h"
#include "param_store.h"

class SlimprotoClient {
public:
    SlimprotoClient(ParamStore& params, AudioRingBuffer<16384>& ringBuffer);
    ~SlimprotoClient();

    /** @brief Start background FreeRTOS networking and streaming task on Core 0. */
    bool begin();

    /** @brief Stop streaming client and disconnect. */
    void stop();

    /** @brief Check if connected to LMS / Roon server. */
    bool isConnected() const { return _connected; }

    /** @brief Check if currently streaming audio. */
    bool isStreaming() const { return _streaming; }

    /** @brief Send playback control commands (toggle pause, stop, etc.) */
    void sendPause();
    void sendResume();
    void sendStop();

private:
    ParamStore&              _params;
    AudioRingBuffer<16384>&  _ringBuffer;

    TaskHandle_t             _taskHandle{nullptr};
    bool                     _running{false};
    bool                     _connected{false};
    bool                     _streaming{false};
    bool                     _paused{false};

    int                      _controlSock{-1};
    int                      _streamSock{-1};

    IPAddress                _serverIp{0, 0, 0, 0};
    uint16_t                 _controlPort{3483};
    uint16_t                 _streamPort{9000};

    uint32_t                 _sampleRate{44100};
    uint8_t                  _bitDepth{16};
    uint8_t                  _channels{2};
    bool                     _bigEndian{true};

    uint32_t                 _bytesReceived{0};
    uint32_t                 _streamStartTime{0};

    // Resampler state (44.1kHz -> 48kHz)
    float                    _resampPhase{0.0f};
    float                    _lastSampleL{0.0f};
    float                    _lastSampleR{0.0f};

    static void _taskTrampoline(void* arg);
    void _runTask();

    bool _discoverServer();
    bool _connectControl();
    void _sendHelo();
    void _sendStat(const char* eventCode, uint32_t serverTimestamp = 0);
    void _sendResp(const char* header, size_t len);

    bool _handleControlPacket();
    void _processStrm(const uint8_t* payload, size_t len);
    void _processAudg(const uint8_t* payload, size_t len);
    void _processSetd(const uint8_t* payload, size_t len);

    bool _startAudioStream(const String& httpRequest);
    void _stopAudioStream();
    void _readAudioStreamData();

    void _updateMetadata(bool connected, bool streaming, bool paused, const char* statusMsg);
};

