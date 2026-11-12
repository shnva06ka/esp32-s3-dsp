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
#include "runtime_tuning.h"
#include "sl_resample.h"

using AudioCallback = std::function<void(float*, float*, size_t)>;

class I2SAudio {
public:
    explicit I2SAudio(ParamStore& params);
    ~I2SAudio() { end(); }

    bool begin();
    void process(const AudioCallback& cb);
    void end();

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

    static constexpr float kScale = 1.0f / 2147483648.0f;
    static constexpr float kFadeStep = 1.0f / (static_cast<float>(kSampleRate) * 0.050f);

    bool _allocBuffers();
    void _freeBuffers();
    void _writeBtFrames(size_t frames);
    static void _btTaskTrampoline(void* arg);
    void _runBtTask();
};
