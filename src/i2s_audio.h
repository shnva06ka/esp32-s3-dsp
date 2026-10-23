// Dual I2S audio driver and input selector.
// I2S0 (master): PCM1808 ADC in, PCM5102A DAC out. I2S1 (slave): Bluetooth audio from the WROOM.
// Wi-Fi audio comes from a ring buffer filled by the SlimProto client.
// Slave reads use a 0 timeout so a missing clock never blocks the DAC.
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

    // one buffer: read inputs, crossfade, run the callback, write to the DAC
    void process(const AudioCallback& cb);

    void end();

    float rmsInL()  const { return _rmsAuxL; }
    float rmsInR()  const { return _rmsAuxR; }
    float rmsAuxL() const { return _rmsAuxL; }
    float rmsAuxR() const { return _rmsAuxR; }
    float rmsBtL()  const { return _rmsBtL;  }
    float rmsBtR()  const { return _rmsBtR;  }
    float rmsOutL() const { return _rmsOutL; }
    float rmsOutR() const { return _rmsOutR; }

    AudioRingBuffer<RuntimeTuning::kWiFiRingBufferFrames>& getWiFiRingBuffer() { return _wifiRingBuffer; }

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

    // 16384 frames = ~341 ms at 48k
    AudioRingBuffer<RuntimeTuning::kWiFiRingBufferFrames> _wifiRingBuffer;

    // BT comes in at 44.1k and gets resampled to 48k
    AudioRingBuffer<RuntimeTuning::kBtRingBufferFrames>  _btRingBuffer;
    float                  _btLastL{0.0f};
    float                  _btLastR{0.0f};
    float                  _btResampPhase{0.0f};

    float _gainAux{1.0f};
    float _gainBt{0.0f};
    float _gainWifi{0.0f};

    ActiveInputSource _lastActiveSource{ACTIVE_SRC_AUX};

    float _rmsAuxL{0}, _rmsAuxR{0};
    float _rmsBtL{0},  _rmsBtR{0};
    float _rmsOutL{0}, _rmsOutR{0};

    static constexpr float kScale = 1.0f / 2147483648.0f;
    static constexpr float kFadeStep = 1.0f / (static_cast<float>(kSampleRate) * 0.050f); // ~50 ms crossfade

    void  _allocBuffers();
    void  _freeBuffers();
    void  _resampleAndWriteBt(size_t frames);
    float _calcRMS(const float* buf, size_t n) const;
};
