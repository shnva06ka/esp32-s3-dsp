#include "i2s_audio.h"
#include "wireless_console.h"
#include <Arduino.h>
#include <esp_heap_caps.h>
#include <cmath>
#include <cstring>
#include <algorithm>

extern "C" size_t squeezelite_output_frames(float *left, float *right, size_t frames);

I2SAudio::I2SAudio(ParamStore& params)
    : _params(params)
{
}

bool I2SAudio::begin()
{
    _allocBuffers();

    _btResampler = resampler_create(kBtSampleRate, kSampleRate);
    _btOutCap    = _btResampler ? resampler_max_out(_btResampler, kBufferFrames) : kBufferFrames;
    _btOutL = static_cast<float*>(heap_caps_aligned_alloc(16, _btOutCap * sizeof(float), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    if (!_btOutL) _btOutL = static_cast<float*>(heap_caps_malloc(_btOutCap * sizeof(float), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    if (!_btOutL) _btOutL = static_cast<float*>(heap_caps_malloc(_btOutCap * sizeof(float), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    _btOutR = static_cast<float*>(heap_caps_aligned_alloc(16, _btOutCap * sizeof(float), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    if (!_btOutR) _btOutR = static_cast<float*>(heap_caps_malloc(_btOutCap * sizeof(float), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    if (!_btOutR) _btOutR = static_cast<float*>(heap_caps_malloc(_btOutCap * sizeof(float), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (!_btResampler || !_btOutL || !_btOutR) {
        Console.println("[AUDIO] ERROR: Failed to set up Bluetooth resampler!");
    } else {
        Console.printf("[AUDIO] Bluetooth resampler %u -> %u Hz ready (Core 0 offload)\n",
                      (unsigned)kBtSampleRate, (unsigned)kSampleRate);
    }

    if (!_btRingBuffer.allocate()) {
        Console.println("[AUDIO] ERROR: Failed to allocate BT ring buffer!");
    }

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
        .dma_buf_count        = 8,
        .dma_buf_len          = static_cast<int>(kBufferFrames),
        .use_apll             = true,
        .tx_desc_auto_clear   = true,
        .fixed_mclk           = static_cast<int>(kSampleRate * 256),
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
        .sample_rate          = kBtSampleRate,
        .bits_per_sample      = I2S_BITS_PER_SAMPLE_32BIT,
        .channel_format       = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count        = 16,
        .dma_buf_len          = 64,
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

    xTaskCreatePinnedToCore(
        _btTaskTrampoline,
        "BtResample",
        8192,
        this,
        4,
        &_btTaskHandle,
        0
    );

    return true;
}

void I2SAudio::_btTaskTrampoline(void* arg)
{
    static_cast<I2SAudio*>(arg)->_runBtTask();
}

void I2SAudio::_runBtTask()
{
    const size_t byteCount = kBufferFrames * kChannels * sizeof(int32_t);
    bool wasBtActive = false;

    while (_running) {
        const InputSourceMode targetSource = _params.getInputMode();
        const bool btActive = (targetSource == INPUT_MODE_BT || _gainBt > 0.0f);
        if (!btActive) {
            wasBtActive = false;
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        if (!wasBtActive) {
            i2s_zero_dma_buffer(kI2SPortSlave);
            wasBtActive = true;
        }

        size_t bytesReadBt = 0;
        if (i2s_read(kI2SPortSlave, _rxBufBt, byteCount, &bytesReadBt, pdMS_TO_TICKS(10)) == ESP_OK &&
            bytesReadBt > 0) {
            size_t frames = bytesReadBt / (kChannels * sizeof(int32_t));
            _writeBtFrames(frames);
        } else {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }

    vTaskDelete(nullptr);
}

void I2SAudio::process(const AudioCallback& cb)
{
    if (!_running) return;

    const size_t byteCount = kBufferFrames * kChannels * sizeof(int32_t);
    const InputSourceMode targetSource = _params.getInputMode();

    static constexpr float kAuxPreamp = 2.0f;
    if (targetSource == INPUT_MODE_AUX || _gainAux > 0.0f) {
        size_t bytesReadAux = 0;
        esp_err_t errAux = i2s_read(kI2SPortMaster, _rxBufAux, byteCount, &bytesReadAux, 0);
        if (errAux == ESP_OK && bytesReadAux > 0) {
            for (size_t i = 0; i < kBufferFrames; ++i) {
                float sL = static_cast<float>(_rxBufAux[i * 2    ]) * (kScale * kAuxPreamp);
                float sR = static_cast<float>(_rxBufAux[i * 2 + 1]) * (kScale * kAuxPreamp);
                _leftAux[i]  = fmaxf(-1.0f, fminf(1.0f, sL));
                _rightAux[i] = fmaxf(-1.0f, fminf(1.0f, sR));
            }
        } else {
            memset(_leftAux,  0, kBufferFrames * sizeof(float));
            memset(_rightAux, 0, kBufferFrames * sizeof(float));
        }
    }

    if (targetSource == INPUT_MODE_BT || _gainBt > 0.0f) {
        size_t avail = _btRingBuffer.available();
        uint32_t underrun = (avail < kBufferFrames) ? static_cast<uint32_t>(kBufferFrames - avail) : 0;
        _btRingBuffer.read(_leftBt, _rightBt, kBufferFrames);
        if (_btResampler && underrun > 0) {
            resampler_record_flow(_btResampler, underrun, 0, 0,
                                  static_cast<uint32_t>(_btRingBuffer.available()),
                                  static_cast<uint32_t>(_btRingBuffer.capacity()));
        }
    }

    size_t wifiFramesRead = 0;
    if (targetSource == INPUT_MODE_WIFI || _gainWifi > 0.0f) {
        wifiFramesRead = squeezelite_output_frames(_leftWifi, _rightWifi, kBufferFrames);
    } else {
        memset(_leftWifi,  0, kBufferFrames * sizeof(float));
        memset(_rightWifi, 0, kBufferFrames * sizeof(float));
    }

    const float targetGainAux  = (targetSource == INPUT_MODE_AUX)  ? 1.0f : 0.0f;
    const float targetGainBt   = (targetSource == INPUT_MODE_BT)   ? 1.0f : 0.0f;
    const float targetGainWifi = (targetSource == INPUT_MODE_WIFI) ? 1.0f : 0.0f;

    const bool wifiSilenceFastPath =
        targetSource == INPUT_MODE_WIFI &&
        wifiFramesRead == 0 &&
        _gainAux == 0.0f &&
        _gainBt == 0.0f &&
        _gainWifi == 1.0f;

    if (wifiSilenceFastPath) {
        memset(_leftMix, 0, kBufferFrames * sizeof(float));
        memset(_rightMix, 0, kBufferFrames * sizeof(float));
    } else if (_gainAux == targetGainAux && _gainBt == targetGainBt && _gainWifi == targetGainWifi) {
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
                           (currentActive == ACTIVE_SRC_AUX)  ? "LINE IN" : "SQUEEZEBOX";
        Console.printf("[AUDIO] Active input switched to: %s\n", name);
    }

    if (!wifiSilenceFastPath) {
        cb(_leftMix, _rightMix, kBufferFrames);
    }

    for (size_t i = 0; i < kBufferFrames; ++i) {
        float sL = fminf(fmaxf(_leftMix[i],  -1.0f), 1.0f);
        float sR = fminf(fmaxf(_rightMix[i], -1.0f), 1.0f);
        _txBuf[i * 2    ] = static_cast<int32_t>(sL * 2147483392.0f);
        _txBuf[i * 2 + 1] = static_cast<int32_t>(sR * 2147483392.0f);
    }

    size_t bytesWritten = 0;
    i2s_write(kI2SPortMaster, _txBuf, byteCount, &bytesWritten, pdMS_TO_TICKS(50));
}

void I2SAudio::end()
{
    if (_running) {
        _running = false;
        if (_btTaskHandle) {
            vTaskDelay(pdMS_TO_TICKS(30));
            _btTaskHandle = nullptr;
        }
        i2s_driver_uninstall(kI2SPortSlave);
        i2s_driver_uninstall(kI2SPortMaster);
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
    resampler_destroy(_btResampler); _btResampler = nullptr;
    heap_caps_free(_btOutL);    _btOutL    = nullptr;
    heap_caps_free(_btOutR);    _btOutR    = nullptr;
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

void I2SAudio::_writeBtFrames(size_t frames)
{
    if (frames == 0) return;

    uint32_t droppedFrames = 0;
    for (int guard = 0; guard < 4 && _btRingBuffer.available() > RuntimeTuning::kBtLatencyCapFrames; ++guard) {
        float dropL[RuntimeTuning::kBtLatencyTrimFrames], dropR[RuntimeTuning::kBtLatencyTrimFrames];
        _btRingBuffer.read(dropL, dropR, RuntimeTuning::kBtLatencyTrimFrames);
        droppedFrames += RuntimeTuning::kBtLatencyTrimFrames;
    }

    float bufL[kBufferFrames];
    float bufR[kBufferFrames];
    for (size_t i = 0; i < frames; ++i) {
        bufL[i] = static_cast<float>(_rxBufBt[i * 2    ]) * kScale;
        bufR[i] = static_cast<float>(_rxBufBt[i * 2 + 1]) * kScale;
    }
    if (_btResampler && _btOutL && _btOutR) {
        size_t outN = resampler_process(_btResampler, bufL, bufR, frames, _btOutL, _btOutR, _btOutCap);
        size_t written = _btRingBuffer.write(_btOutL, _btOutR, outN);
        uint32_t overrun = (outN > written) ? static_cast<uint32_t>(outN - written) : 0;
        resampler_record_flow(_btResampler, 0, overrun, droppedFrames,
                              static_cast<uint32_t>(_btRingBuffer.available()),
                              static_cast<uint32_t>(_btRingBuffer.capacity()));
    }
}

bool I2SAudio::getBtResamplerStats(resampler_stats_t* out) const
{
    if (!_btResampler || !out) return false;
    resampler_get_stats(_btResampler, out);
    return true;
}
