#pragma once

#include <functional>
#include <cstddef>
#include <driver/i2s.h>
#include "audio_config.h"
#include "param_store.h"
#include "audio_ring_buffer.h"
#include "runtime_tuning.h"

using AudioCallback = std::function<void(float*, float*, size_t)>;

class I2SAudio {
public:
    explicit I2SAudio(ParamStore& params);
    ~I2SAudio() { end(); }

    bool begin();
    void process(const AudioCallback& cb);
    void end();

private:
    ParamStore& _params;
    bool        _running{false};

    int32_t* _rxBufAux{nullptr};
    int32_t* _rxBufBt{nullptr};
    int32_t* _txBuf{nullptr};

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
    float _gainBt{0.0f};
    float _gainWifi{0.0f};

    ActiveInputSource _lastActiveSource{ACTIVE_SRC_AUX};

    static constexpr float kScale = 1.0f / 2147483648.0f;
    static constexpr float kFadeStep = 1.0f / (static_cast<float>(kSampleRate) * 0.050f);

    void _allocBuffers();
    void _freeBuffers();
    void _writeBtFrames(size_t frames);
};
