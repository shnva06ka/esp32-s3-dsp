#pragma once

#include <functional>
#include <cstddef>
#include <atomic>
#include <driver/i2s.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "audio_config.h"
#include "param_store.h"
#include "audio_ring_buffer.h"
#include "lms_codecs.h"

using AudioCallback      = std::function<void(float*, float*, size_t)>;
using RateChangeCallback = std::function<void(uint32_t)>;

void     i2s_request_wifi_rate(uint32_t rate);
uint32_t i2s_active_rate();

class I2SAudio {
public:
    static constexpr size_t kMaxBlockFrames = 512;

    explicit I2SAudio(ParamStore& params);
    ~I2SAudio() { end(); }

    bool begin();
    void process(const AudioCallback& cb, const RateChangeCallback& onRateChange = nullptr);
    void end();

    void     requestWifiSampleRate(uint32_t rate);
    uint32_t activeRate() const { return _activeRateAtomic.load(std::memory_order_acquire); }

    bool getBtResamplerStats(resampler_stats_t* out) const;
    TaskHandle_t getBtTaskHandle() const { return _btTaskHandle; }

private:
    ParamStore& _params;
    std::atomic<bool> _running{false};
    std::atomic<bool> _btTaskExited{true};
    TaskHandle_t  _btTaskHandle{nullptr};

    int32_t* _rxBufAux{nullptr};
    int32_t* _rxBufBt{nullptr};
    int32_t* _txBuf{nullptr};

    resampler_t* _btResampler{nullptr};
    float* _btOutL{nullptr};
    float* _btOutR{nullptr};
    size_t _btOutCap{0};

    float* _leftAux{nullptr};
    float* _rightAux{nullptr};
    float* _leftBt{nullptr};
    float* _rightBt{nullptr};
    float* _leftWifi{nullptr};
    float* _rightWifi{nullptr};
    float* _leftMix{nullptr};
    float* _rightMix{nullptr};

    AudioRingBuffer<RuntimeTuning::kBtRingBufferFrames> _btRingBuffer;

    float _gainAux{1.0f};
    std::atomic<float> _gainBt{0.0f};
    float _gainWifi{0.0f};

    ActiveInputSource _lastActiveSource{ACTIVE_SRC_AUX};

    uint32_t              _activeRate{audio_rate::kDefaultRate};
    size_t                _blockFrames{256};
    float                 _fadeStep{1.0f / (static_cast<float>(audio_rate::kDefaultRate) * 0.050f)};
    size_t                _fadeInSamples{0};
    std::atomic<uint32_t> _activeRateAtomic{audio_rate::kDefaultRate};
    std::atomic<uint32_t> _requestedWifiRate{0};

    // Diagnostics (Core 1 only)
    int64_t  _statsLastUs{0};
    uint64_t _statsFrames{0};
    uint64_t _statsDspUsSum{0};
    uint32_t _statsDspUsMax{0};
    uint32_t _statsBlocks{0};
    uint32_t _statsZeroCross{0};
    uint32_t _statsPitchSamples{0};
    float    _statsPrevL{0.0f};
    uint32_t _rateSwitches{0};

    static constexpr float kScale = 1.0f / 2147483648.0f;

    bool _installMasterI2S(uint32_t rate);
    void _switchMasterRate(uint32_t newRate, const RateChangeCallback& onRateChange);
    bool _allocBuffers();
    void _freeBuffers();
    void _writeBtFrames(size_t frames);
    static void _btTaskTrampoline(void* arg);
    void _runBtTask();
};
