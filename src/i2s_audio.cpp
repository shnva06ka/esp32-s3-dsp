#include "i2s_audio.h"
#include <Arduino.h>
#include <esp_heap_caps.h>
#include <cmath>
#include <cstring>
#include <algorithm>
#if __has_include(<esp_dsp.h>)
#include <esp_dsp.h>
#define HAVE_ESP_DSP 1
#endif

I2SAudio::I2SAudio(ParamStore& params)
    : _params(params)
{
}

bool I2SAudio::begin()
{
    _allocBuffers();

    const i2s_config_t cfg_master = {
        .mode                 = static_cast<i2s_mode_t>(
                                    I2S_MODE_MASTER |
                                    I2S_MODE_TX     |
                                    I2S_MODE_RX),
        .sample_rate          = kSampleRate,
        .bits_per_sample      = I2S_BITS_PER_SAMPLE_32BIT,
        .channel_format       = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count        = 4,
        .dma_buf_len          = static_cast<int>(kBufferFrames),
        .use_apll             = true,      // APLL for low-jitter clock
        .tx_desc_auto_clear   = true,
        .fixed_mclk           = static_cast<int>(kSampleRate * 256),  // 12.288 MHz
    };

    if (i2s_driver_install(kI2SPortMaster, &cfg_master, 0, nullptr) != ESP_OK) {
        return false;
    }

    const i2s_pin_config_t pins_master = {
        .mck_io_num   = kPinMCLK,
        .bck_io_num   = kPinBCK,
        .ws_io_num    = kPinWS,
        .data_out_num = kPinDOUT,
        .data_in_num  = kPinDIN,
    };

    if (i2s_set_pin(kI2SPortMaster, &pins_master) != ESP_OK) {
        i2s_driver_uninstall(kI2SPortMaster);
        return false;
    }
    i2s_zero_dma_buffer(kI2SPortMaster);

    const i2s_config_t cfg_slave = {
        .mode                 = static_cast<i2s_mode_t>(
                                    I2S_MODE_SLAVE |
                                    I2S_MODE_RX),
        .sample_rate          = kSampleRate,
        .bits_per_sample      = I2S_BITS_PER_SAMPLE_32BIT,
        .channel_format       = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count        = 16,
        .dma_buf_len          = 64,  // small blocks so descriptors keep finishing
        .use_apll             = false,
        .tx_desc_auto_clear   = false,
        .fixed_mclk           = 0,
    };

    if (i2s_driver_install(kI2SPortSlave, &cfg_slave, 0, nullptr) != ESP_OK) {
        i2s_driver_uninstall(kI2SPortMaster);
        return false;
    }

    const i2s_pin_config_t pins_slave = {
        .mck_io_num   = I2S_PIN_NO_CHANGE,
        .bck_io_num   = kPinBtBCK,
        .ws_io_num    = kPinBtWS,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num  = kPinBtDIN,
    };

    if (i2s_set_pin(kI2SPortSlave, &pins_slave) != ESP_OK) {
        i2s_driver_uninstall(kI2SPortSlave);
        i2s_driver_uninstall(kI2SPortMaster);
        return false;
    }
    i2s_zero_dma_buffer(kI2SPortSlave);

    _running = true;
    return true;
}

void I2SAudio::process(const AudioCallback& cb)
{
    if (!_running) return;

    const size_t byteCount = kBufferFrames * kChannels * sizeof(int32_t);

    size_t bytesReadAux = 0;
    esp_err_t errAux = i2s_read(kI2SPortMaster, _rxBufAux, byteCount, &bytesReadAux, pdMS_TO_TICKS(20));
    if (errAux != ESP_OK || bytesReadAux == 0) {
        memset(_rxBufAux, 0, byteCount);
    }

    for (size_t i = 0; i < kBufferFrames; ++i) {
        _leftAux[i]  = static_cast<float>(_rxBufAux[i * 2    ]) * kScale;
        _rightAux[i] = static_cast<float>(_rxBufAux[i * 2 + 1]) * kScale;
    }
    _rmsAuxL = _calcRMS(_leftAux,  kBufferFrames);
    _rmsAuxR = _calcRMS(_rightAux, kBufferFrames);

    size_t bytesReadBt = 0;
    while (i2s_read(kI2SPortSlave, _rxBufBt, byteCount, &bytesReadBt, 0) == ESP_OK && bytesReadBt > 0) {
        size_t frames = bytesReadBt / (kChannels * sizeof(int32_t));
        _resampleAndWriteBt(frames);
    }

    _btRingBuffer.read(_leftBt, _rightBt, kBufferFrames);
    _rmsBtL = _calcRMS(_leftBt, kBufferFrames);
    _rmsBtR = _calcRMS(_rightBt, kBufferFrames);

    _wifiRingBuffer.read(_leftWifi, _rightWifi, kBufferFrames);

    const InputSourceMode targetSource = _params.getInputMode();

    const float targetGainAux  = (targetSource == INPUT_MODE_AUX)  ? 1.0f : 0.0f;
    const float targetGainBt   = (targetSource == INPUT_MODE_BT)   ? 1.0f : 0.0f;
    const float targetGainWifi = (targetSource == INPUT_MODE_WIFI) ? 1.0f : 0.0f;

    if (_gainAux == targetGainAux && _gainBt == targetGainBt && _gainWifi == targetGainWifi) {
        const float* srcL = (targetSource == INPUT_MODE_BT)   ? _leftBt   :
                            (targetSource == INPUT_MODE_WIFI) ? _leftWifi : _leftAux;
        const float* srcR = (targetSource == INPUT_MODE_BT)   ? _rightBt   :
                            (targetSource == INPUT_MODE_WIFI) ? _rightWifi : _rightAux;
        memcpy(_leftMix,  srcL, kBufferFrames * sizeof(float));
        memcpy(_rightMix, srcR, kBufferFrames * sizeof(float));
    } else {
        for (size_t i = 0; i < kBufferFrames; ++i) {
            if (_gainAux < targetGainAux) _gainAux = std::min(targetGainAux, _gainAux + kFadeStep);
            else if (_gainAux > targetGainAux) _gainAux = std::max(targetGainAux, _gainAux - kFadeStep);

            if (_gainBt < targetGainBt) _gainBt = std::min(targetGainBt, _gainBt + kFadeStep);
            else if (_gainBt > targetGainBt) _gainBt = std::max(targetGainBt, _gainBt - kFadeStep);

            if (_gainWifi < targetGainWifi) _gainWifi = std::min(targetGainWifi, _gainWifi + kFadeStep);
            else if (_gainWifi > targetGainWifi) _gainWifi = std::max(targetGainWifi, _gainWifi - kFadeStep);

            _leftMix[i]  = _gainAux * _leftAux[i]  + _gainBt * _leftBt[i]  + _gainWifi * _leftWifi[i];
            _rightMix[i] = _gainAux * _rightAux[i] + _gainBt * _rightBt[i] + _gainWifi * _rightWifi[i];
        }
    }

    ActiveInputSource currentActive = (targetSource == INPUT_MODE_BT)   ? ACTIVE_SRC_BT :
                                      (targetSource == INPUT_MODE_WIFI) ? ACTIVE_SRC_WIFI : ACTIVE_SRC_AUX;

    if (currentActive != _lastActiveSource) {
        _lastActiveSource = currentActive;
        _params.setActiveSource(currentActive);
        const char* name = (currentActive == ACTIVE_SRC_BT)   ? "BLUETOOTH" :
                           (currentActive == ACTIVE_SRC_AUX)  ? "AUX (PCM1808)" : "WI-FI";
        Serial.printf("[AUDIO] Active input switched to: %s\n", name);
    }

    cb(_leftMix, _rightMix, kBufferFrames);

    _rmsOutL = _calcRMS(_leftMix,  kBufferFrames);
    _rmsOutR = _calcRMS(_rightMix, kBufferFrames);

    for (size_t i = 0; i < kBufferFrames; ++i) {
        float sL = fminf(fmaxf(_leftMix[i],  -1.0f), 1.0f);
        float sR = fminf(fmaxf(_rightMix[i], -1.0f), 1.0f);
        _txBuf[i * 2    ] = static_cast<int32_t>(sL * 2147483392.0f);
        _txBuf[i * 2 + 1] = static_cast<int32_t>(sR * 2147483392.0f);
    }

    size_t bytesWritten = 0;
    i2s_write(kI2SPortMaster, _txBuf, byteCount, &bytesWritten, pdMS_TO_TICKS(20));
}

void I2SAudio::end()
{
    if (_running) {
        i2s_driver_uninstall(kI2SPortSlave);
        i2s_driver_uninstall(kI2SPortMaster);
        _running = false;
    }
    _freeBuffers();
}

void I2SAudio::_allocBuffers()
{
    const size_t intBufBytes   = kBufferFrames * kChannels * sizeof(int32_t);
    const size_t floatBufBytes = kBufferFrames * sizeof(float);

    auto dramAlloc = [](size_t sz) -> void* {
        void* p = heap_caps_aligned_alloc(16, sz, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (!p) p = heap_caps_malloc(sz, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (!p) p = malloc(sz);
        return p;
    };

    _rxBufAux  = static_cast<int32_t*>(dramAlloc(intBufBytes));
    _rxBufBt   = static_cast<int32_t*>(dramAlloc(intBufBytes));
    _txBuf     = static_cast<int32_t*>(dramAlloc(intBufBytes));

    _leftAux   = static_cast<float*>  (dramAlloc(floatBufBytes));
    _rightAux  = static_cast<float*>  (dramAlloc(floatBufBytes));
    _leftBt    = static_cast<float*>  (dramAlloc(floatBufBytes));
    _rightBt   = static_cast<float*>  (dramAlloc(floatBufBytes));
    _leftWifi  = static_cast<float*>  (dramAlloc(floatBufBytes));
    _rightWifi = static_cast<float*>  (dramAlloc(floatBufBytes));
    _leftMix   = static_cast<float*>  (dramAlloc(floatBufBytes));
    _rightMix  = static_cast<float*>  (dramAlloc(floatBufBytes));
}

void I2SAudio::_freeBuffers()
{
    heap_caps_free(_rxBufAux);  _rxBufAux  = nullptr;
    heap_caps_free(_rxBufBt);   _rxBufBt   = nullptr;
    heap_caps_free(_txBuf);     _txBuf     = nullptr;

    heap_caps_free(_leftAux);   _leftAux   = nullptr;
    heap_caps_free(_rightAux);  _rightAux  = nullptr;
    heap_caps_free(_leftBt);    _leftBt    = nullptr;
    heap_caps_free(_rightBt);   _rightBt   = nullptr;
    heap_caps_free(_leftWifi);  _leftWifi  = nullptr;
    heap_caps_free(_rightWifi); _rightWifi = nullptr;
    heap_caps_free(_leftMix);   _leftMix   = nullptr;
    heap_caps_free(_rightMix);  _rightMix  = nullptr;
}

float I2SAudio::_calcRMS(const float* buf, size_t n) const
{
    if (n == 0) return 0.0f;
    float sum = 0.0f;
#if defined(HAVE_ESP_DSP)
    dsps_dotprod_f32(buf, buf, &sum, static_cast<int>(n));
#else
    for (size_t i = 0; i < n; ++i) sum += buf[i] * buf[i];
#endif
    return sqrtf(sum / static_cast<float>(n));
}

void I2SAudio::_resampleAndWriteBt(size_t frames)
{
    if (frames == 0) return;

    // cap latency at ~85 ms (4096 frames)
    if (_btRingBuffer.available() > RuntimeTuning::kBtLatencyCapFrames) {
        float dropL[RuntimeTuning::kBtLatencyTrimFrames], dropR[RuntimeTuning::kBtLatencyTrimFrames];
        _btRingBuffer.read(dropL, dropR, RuntimeTuning::kBtLatencyTrimFrames);
    }

    float resampL[RuntimeTuning::kBtResampleChunkFrames];
    float resampR[RuntimeTuning::kBtResampleChunkFrames];
    size_t outCount = 0;
    constexpr float step = static_cast<float>(RuntimeTuning::kBtInputSampleRate) /
                           static_cast<float>(RuntimeTuning::kDspOutputSampleRate);

    for (size_t i = 0; i < frames; ++i) {
        float curL = static_cast<float>(_rxBufBt[i * 2    ]) * kScale;
        float curR = static_cast<float>(_rxBufBt[i * 2 + 1]) * kScale;

        while (_btResampPhase < 1.0f && outCount < RuntimeTuning::kBtResampleChunkFrames) {
            resampL[outCount] = _btLastL + _btResampPhase * (curL - _btLastL);
            resampR[outCount] = _btLastR + _btResampPhase * (curR - _btLastR);
            outCount++;
            _btResampPhase += step;
        }

        if (_btResampPhase >= 1.0f) {
            _btResampPhase -= 1.0f;
        }
        _btLastL = curL;
        _btLastR = curR;

        if (outCount >= (RuntimeTuning::kBtResampleChunkFrames - 32)) {
            _btRingBuffer.write(resampL, resampR, outCount);
            outCount = 0;
        }
    }

    if (outCount > 0) {
        _btRingBuffer.write(resampL, resampR, outCount);
    }
}
