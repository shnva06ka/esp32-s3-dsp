// SlimProto (Squeezelite style) client for LMS / Roon.
// UDP discovery + TCP control on 3483, PCM pulled over HTTP, decoded to float, resampled 44.1k -> 48k
// and pushed into the Wi-Fi ring buffer.
#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <lwip/sockets.h>
#include <lwip/netdb.h>
#include "audio_ring_buffer.h"
#include "param_store.h"
#include "runtime_tuning.h"

class SlimprotoClient {
public:
    SlimprotoClient(ParamStore& params, AudioRingBuffer<RuntimeTuning::kWiFiRingBufferFrames>& ringBuffer);
    ~SlimprotoClient();

    bool begin();

    void stop();

    bool isConnected() const { return _connected; }

    bool isStreaming() const { return _streaming; }

    void sendPause();
    void sendResume();
    void sendStop();

private:
    ParamStore&              _params;
    AudioRingBuffer<RuntimeTuning::kWiFiRingBufferFrames>& _ringBuffer;

    TaskHandle_t             _taskHandle{nullptr};
    bool                     _running{false};
    bool                     _connected{false};
    bool                     _streaming{false};
    bool                     _paused{false};

    int                      _controlSock{-1};
    int                      _streamSock{-1};

    IPAddress                _serverIp{0, 0, 0, 0};
    uint16_t                 _controlPort{RuntimeTuning::kSlimprotoControlPort};
    uint16_t                 _streamPort{RuntimeTuning::kSlimprotoDefaultStreamPort};

    uint32_t                 _sampleRate{RuntimeTuning::kSlimprotoDefaultRate};
    uint8_t                  _bitDepth{16};
    uint8_t                  _channels{2};
    bool                     _bigEndian{false};

    uint32_t                 _bytesReceived{0};
    uint32_t                 _streamStartTime{0};
    bool                     _headerParsed{false};
    bool                     _startedSent{false};

    uint8_t                  _carryBuf[RuntimeTuning::kSlimprotoCarryBytes]{0};
    size_t                   _carryLen{0};

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
