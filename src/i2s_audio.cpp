#include "i2s_audio.h"
#include "wireless_console.h"
#include "dsp_pipeline.h"
#include <Arduino.h>
#include <esp_heap_caps.h>
#include <cstring>
#include <algorithm>

#include "lms_client.h"
#include <esp_timer.h>

static I2SAudio* s_i2sInstance = nullptr;

void i2s_request_wifi_rate(uint32_t rate) {
    if (s_i2sInstance) s_i2sInstance->requestWifiSampleRate(rate);
}

uint32_t i2s_active_rate() {
    return s_i2sInstance ? s_i2sInstance->activeRate() : audio_rate::kDefaultRate;
}

I2SAudio::I2SAudio(ParamStore& params)
    : _params(params)
{
}

void I2SAudio::requestWifiSampleRate(uint32_t rate)
{
    if (rate != 0) {
        _requestedWifiRate.store(rate, std::memory_order_release);
    }
}

bool I2SAudio::_installMasterI2S(uint32_t rate)
{
    const int mclk = (rate <= 96000u) ? static_cast<int>(rate * 256u)
                                      : static_cast<int>(rate * 128u);
    const size_t blk = audio_rate::blockFramesForRate(rate);

    const i2s_config_t cfg_master = {
        .mode                 = static_cast<i2s_mode_t>(
                                    I2S_MODE_MASTER |
                                    I2S_MODE_TX     |
                                    I2S_MODE_RX),
        .sample_rate          = rate,
        .bits_per_sample      = I2S_BITS_PER_SAMPLE_32BIT,
        .channel_format       = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count        = 8,
        .dma_buf_len          = static_cast<int>(blk),
        .use_apll             = true,
        .tx_desc_auto_clear   = true,
        .fixed_mclk           = mclk,
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
    return true;
}

void I2SAudio::_switchMasterRate(uint32_t newRate, const RateChangeCallback& onRateChange)
{
    if (newRate == 0 || newRate == _activeRate) return;
    const int64_t t0 = esp_timer_get_time();

    // Fade out last block smoothly to avoid click on clock switch
    for (size_t i = 0; i < _blockFrames; ++i) {
        const float g = 1.0f - (static_cast<float>(i + 1) / static_cast<float>(_blockFrames));
        _leftMix[i]  *= g;
        _rightMix[i] *= g;
    }
    dsp_pack_f32_to_i32(_txBuf, _leftMix, _rightMix, _blockFrames);
    size_t bw = 0;
    i2s_write(kI2SPortMaster, _txBuf, _blockFrames * kChannels * sizeof(int32_t), &bw, pdMS_TO_TICKS(20));

    i2s_zero_dma_buffer(kI2SPortMaster);
    i2s_driver_uninstall(kI2SPortMaster);
    if (!_installMasterI2S(newRate)) {
        Console.printf("[AUDIO] ERROR: Master I2S switch to %u Hz failed, reverting to %u Hz\n",
                       (unsigned)newRate, (unsigned)_activeRate);
        _installMasterI2S(_activeRate);
        return;
    }

    _activeRate  = newRate;
    _blockFrames = audio_rate::blockFramesForRate(newRate);
    _fadeStep    = 1.0f / (static_cast<float>(newRate) * 0.050f);
    _fadeInSamples = _blockFrames;

    if (onRateChange) {
        onRateChange(newRate);
    }

    _params.setActiveRate(newRate);
    _activeRateAtomic.store(newRate, std::memory_order_release);
    ++_rateSwitches;

    // Reset measurement window so measured Hz reflects the new rate cleanly
    _statsLastUs      = esp_timer_get_time();
    _statsFrames      = 0;
    _statsDspUsSum    = 0;
    _statsDspUsMax    = 0;
    _statsBlocks      = 0;
    _statsZeroCross   = 0;
    _statsPitchSamples = 0;
    _statsPrevL       = 0.0f;

    const int64_t elapsedUs = _statsLastUs - t0;
    const uint32_t mclk = (newRate <= 96000u) ? (newRate * 256u) : (newRate * 128u);
    Console.printf("[AUDIO] Rate switch -> %u Hz (block=%u, mclk=%u, %u us)\n",
                   (unsigned)newRate, (unsigned)_blockFrames, (unsigned)mclk, (unsigned)elapsedUs);
}

bool I2SAudio::begin()
{
    s_i2sInstance = this;
    _activeRate  = _params.getInitialRate();
    _blockFrames = audio_rate::blockFramesForRate(_activeRate);
    _fadeStep    = 1.0f / (static_cast<float>(_activeRate) * 0.050f);
    _activeRateAtomic.store(_activeRate, std::memory_order_release);
    _params.setActiveRate(_activeRate);

    if (!_allocBuffers()) {
        Console.println("[AUDIO] ERROR: Failed to allocate audio buffers!");
        _freeBuffers();
        return false;
    }

    _btResampler = resampler_create(kBtSampleRate, kBtSampleRate);
    _btOutCap    = kMaxBlockFrames;
    _btOutL = static_cast<float*>(heap_caps_aligned_alloc(16, _btOutCap * sizeof(float), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    if (!_btOutL) _btOutL = static_cast<float*>(heap_caps_malloc(_btOutCap * sizeof(float), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    if (!_btOutL) _btOutL = static_cast<float*>(heap_caps_malloc(_btOutCap * sizeof(float), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    _btOutR = static_cast<float*>(heap_caps_aligned_alloc(16, _btOutCap * sizeof(float), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    if (!_btOutR) _btOutR = static_cast<float*>(heap_caps_malloc(_btOutCap * sizeof(float), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    if (!_btOutR) _btOutR = static_cast<float*>(heap_caps_malloc(_btOutCap * sizeof(float), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));

    if (!_btRingBuffer.allocate(false)) {
        Console.println("[AUDIO] ERROR: Failed to allocate BT ring buffer!");
        _freeBuffers();
        return false;
    }

    if (!_installMasterI2S(_activeRate)) {
        _freeBuffers();
        return false;
    }
    Console.printf("[AUDIO] Master I2S started at %u Hz (block=%u)\n",
                   (unsigned)_activeRate, (unsigned)_blockFrames);

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
        _freeBuffers();
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
        _freeBuffers();
        return false;
    }
    i2s_zero_dma_buffer(kI2SPortSlave);

    _running.store(true, std::memory_order_release);
    _btTaskExited.store(false, std::memory_order_release);

    if (xTaskCreatePinnedToCore(
            _btTaskTrampoline,
            "BtResample",
            8192,
            this,
            4,
            &_btTaskHandle,
            0
        ) != pdPASS) {
        _btTaskHandle = nullptr;
        _running.store(false, std::memory_order_release);
        _btTaskExited.store(true, std::memory_order_release);
        i2s_driver_uninstall(kI2SPortSlave);
        i2s_driver_uninstall(kI2SPortMaster);
        _freeBuffers();
        return false;
    }

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

    while (_running.load(std::memory_order_acquire)) {
        const InputSourceMode targetSource = _params.getInputMode();
        const bool btActive = (targetSource == INPUT_MODE_BT || _gainBt.load(std::memory_order_acquire) > 0.0f);
        if (!btActive) {
            if (wasBtActive) {
                _btRingBuffer.clear();
                wasBtActive = false;
            }
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

    _btTaskExited.store(true, std::memory_order_release);
    vTaskDelete(nullptr);
}

void I2SAudio::process(const AudioCallback& cb, const RateChangeCallback& onRateChange)
{
    if (!_running.load(std::memory_order_acquire)) return;

    const InputSourceMode targetSource = _params.getInputMode();
    uint32_t desiredRate = _activeRate;
    if (targetSource == INPUT_MODE_AUX) {
        desiredRate = _params.getAdcRate();
    } else if (targetSource == INPUT_MODE_BT) {
        desiredRate = audio_rate::kBtRate;
    } else if (targetSource == INPUT_MODE_WIFI) {
        const uint32_t req = _requestedWifiRate.load(std::memory_order_acquire);
        if (req != 0) desiredRate = req;
    }
    if (desiredRate != _activeRate) {
        _switchMasterRate(desiredRate, onRateChange);
    }

    const size_t nFrames   = _blockFrames;
    const size_t byteCount = nFrames * kChannels * sizeof(int32_t);

    static constexpr float kAuxPreamp = 2.0f;
    if (targetSource == INPUT_MODE_AUX || _gainAux > 0.0f) {
        size_t bytesReadAux = 0;
        const TickType_t auxTimeout = (targetSource == INPUT_MODE_AUX) ? pdMS_TO_TICKS(5) : 0;
        esp_err_t errAux = i2s_read(kI2SPortMaster, _rxBufAux, byteCount, &bytesReadAux, auxTimeout);
        if (errAux == ESP_OK && bytesReadAux > 0) {
            size_t framesRead = bytesReadAux / (kChannels * sizeof(int32_t));
            if (framesRead > nFrames) framesRead = nFrames;
            const float auxScale = kScale * kAuxPreamp;
            dsp_unpack_i32_stereo_to_f32(_rxBufAux, _leftAux, _rightAux, framesRead, auxScale, auxScale);
            for (size_t i = framesRead; i < nFrames; ++i) {
                _leftAux[i]  = 0.0f;
                _rightAux[i] = 0.0f;
            }
        } else {
            memset(_leftAux,  0, nFrames * sizeof(float));
            memset(_rightAux, 0, nFrames * sizeof(float));
        }
    }

    float gBt = _gainBt.load(std::memory_order_relaxed);

    if (targetSource == INPUT_MODE_BT || gBt > 0.0f) {
        if (_btRingBuffer.available() > RuntimeTuning::kBtLatencyCapFrames) {
            _btRingBuffer.skip(RuntimeTuning::kBtLatencyTrimFrames);
        }

        size_t avail = _btRingBuffer.available();
        uint32_t underrun = (avail < nFrames) ? static_cast<uint32_t>(nFrames - avail) : 0;
        size_t readFrames = _btRingBuffer.read(_leftBt, _rightBt, nFrames);
        for (size_t i = readFrames; i < nFrames; ++i) {
            _leftBt[i]  = 0.0f;
            _rightBt[i] = 0.0f;
        }
        if (_btResampler && underrun > 0) {
            resampler_record_flow(_btResampler, underrun, 0, 0,
                                  static_cast<uint32_t>(_btRingBuffer.available()),
                                  static_cast<uint32_t>(_btRingBuffer.capacity()));
        }
    }

    size_t wifiFramesRead = 0;
    if (targetSource == INPUT_MODE_WIFI || _gainWifi > 0.0f) {
        wifiFramesRead = lms_output_frames(_leftWifi, _rightWifi, nFrames);
        for (size_t i = wifiFramesRead; i < nFrames; ++i) {
            _leftWifi[i]  = 0.0f;
            _rightWifi[i] = 0.0f;
        }
    } else {
        memset(_leftWifi,  0, nFrames * sizeof(float));
        memset(_rightWifi, 0, nFrames * sizeof(float));
    }

    const float targetGainAux  = (targetSource == INPUT_MODE_AUX)  ? 1.0f : 0.0f;
    const float targetGainBt   = (targetSource == INPUT_MODE_BT)   ? 1.0f : 0.0f;
    const float targetGainWifi = (targetSource == INPUT_MODE_WIFI) ? 1.0f : 0.0f;

    const bool wifiSilenceFastPath =
        targetSource == INPUT_MODE_WIFI &&
        wifiFramesRead == 0 &&
        _gainAux == 0.0f &&
        gBt == 0.0f &&
        _gainWifi == 1.0f;

    if (wifiSilenceFastPath) {
        memset(_leftMix, 0, nFrames * sizeof(float));
        memset(_rightMix, 0, nFrames * sizeof(float));
    } else if (_gainAux == targetGainAux && gBt == targetGainBt && _gainWifi == targetGainWifi) {
        const float* srcL = (targetSource == INPUT_MODE_BT)   ? _leftBt   :
                            (targetSource == INPUT_MODE_WIFI) ? _leftWifi : _leftAux;
        const float* srcR = (targetSource == INPUT_MODE_BT)   ? _rightBt   :
                            (targetSource == INPUT_MODE_WIFI) ? _rightWifi : _rightAux;
        memcpy(_leftMix,  srcL, nFrames * sizeof(float));
        memcpy(_rightMix, srcR, nFrames * sizeof(float));
    } else {
        for (size_t i = 0; i < nFrames; ++i) {
            if (_gainAux < targetGainAux) _gainAux = std::min(targetGainAux, _gainAux + _fadeStep);
            else if (_gainAux > targetGainAux) _gainAux = std::max(targetGainAux, _gainAux - _fadeStep);

            if (gBt < targetGainBt) gBt = std::min(targetGainBt, gBt + _fadeStep);
            else if (gBt > targetGainBt) gBt = std::max(targetGainBt, gBt - _fadeStep);

            if (_gainWifi < targetGainWifi) _gainWifi = std::min(targetGainWifi, _gainWifi + _fadeStep);
            else if (_gainWifi > targetGainWifi) _gainWifi = std::max(targetGainWifi, _gainWifi - _fadeStep);

            _leftMix[i]  = _gainAux * _leftAux[i]  + gBt * _leftBt[i]  + _gainWifi * _leftWifi[i];
            _rightMix[i] = _gainAux * _rightAux[i] + gBt * _rightBt[i] + _gainWifi * _rightWifi[i];
        }
        _gainBt.store(gBt, std::memory_order_release);
    }

    if (_fadeInSamples > 0) {
        const size_t total = nFrames;
        for (size_t i = 0; i < nFrames; ++i) {
            const float g = static_cast<float>(i + 1) / static_cast<float>(total);
            _leftMix[i]  *= g;
            _rightMix[i] *= g;
        }
        _fadeInSamples = 0;
    }

    ActiveInputSource currentActive = (targetSource == INPUT_MODE_BT)   ? ACTIVE_SRC_BT :
                                      (targetSource == INPUT_MODE_WIFI) ? ACTIVE_SRC_WIFI : ACTIVE_SRC_AUX;

    if (currentActive != _lastActiveSource) {
        _lastActiveSource = currentActive;
        _params.setActiveSource(currentActive);
    }

    if (!wifiSilenceFastPath) {
        const int64_t tDsp0 = esp_timer_get_time();
        cb(_leftMix, _rightMix, nFrames);
        const uint32_t dspUs = static_cast<uint32_t>(esp_timer_get_time() - tDsp0);
        _statsDspUsSum += dspUs;
        if (dspUs > _statsDspUsMax) _statsDspUsMax = dspUs;
        ++_statsBlocks;

        // Post-DSP left-channel zero-crossing pitch check (active signal only)
        float sumSq = 0.0f;
        for (size_t i = 0; i < nFrames; ++i) sumSq += _leftMix[i] * _leftMix[i];
        if (sumSq > 1e-4f * static_cast<float>(nFrames)) {
            float prev = _statsPrevL;
            for (size_t i = 0; i < nFrames; ++i) {
                const float cur = _leftMix[i];
                if (prev <= 0.0f && cur > 0.0f) ++_statsZeroCross;
                prev = cur;
            }
            _statsPrevL = prev;
            _statsPitchSamples += static_cast<uint32_t>(nFrames);
        }
    }

    dsp_pack_f32_to_i32(_txBuf, _leftMix, _rightMix, nFrames);

    size_t bytesWritten = 0;
    i2s_write(kI2SPortMaster, _txBuf, byteCount, &bytesWritten, pdMS_TO_TICKS(50));
    _statsFrames += bytesWritten / (kChannels * sizeof(int32_t));

    const int64_t nowUs = esp_timer_get_time();
    if (_statsLastUs == 0) _statsLastUs = nowUs;
    const int64_t winUs = nowUs - _statsLastUs;
    if (winUs >= 5000000LL) {
        const uint32_t measHz = static_cast<uint32_t>((_statsFrames * 1000000ULL) / static_cast<uint64_t>(winUs));
        const uint32_t avgDsp = _statsBlocks ? static_cast<uint32_t>(_statsDspUsSum / _statsBlocks) : 0u;
        const uint32_t budgetUs = static_cast<uint32_t>((1000000ULL * nFrames) / _activeRate);
        const uint32_t pitchHz = (_statsPitchSamples > 0)
            ? static_cast<uint32_t>((static_cast<uint64_t>(_statsZeroCross) * _activeRate) / _statsPitchSamples)
            : 0u;
        if (_statsBlocks > 0 || targetSource != INPUT_MODE_WIFI) {
            Console.printf("[AUDIO STATS] fs=%u meas=%uHz blk=%u dsp=%u/%u/%uus pitch=%uHz sw=%u\n",
                           (unsigned)_activeRate, (unsigned)measHz, (unsigned)nFrames,
                           (unsigned)avgDsp, (unsigned)_statsDspUsMax, (unsigned)budgetUs,
                           (unsigned)pitchHz, (unsigned)_rateSwitches);
        }
        _statsLastUs       = nowUs;
        _statsFrames       = 0;
        _statsDspUsSum     = 0;
        _statsDspUsMax     = 0;
        _statsBlocks       = 0;
        _statsZeroCross    = 0;
        _statsPitchSamples = 0;
    }
}

void I2SAudio::end()
{
    if (_running.exchange(false, std::memory_order_acq_rel)) {
        if (_btTaskHandle) {
            for (int w = 0; w < 50 && !_btTaskExited.load(std::memory_order_acquire); ++w) {
                vTaskDelay(pdMS_TO_TICKS(5));
            }
            _btTaskHandle = nullptr;
        }
        i2s_driver_uninstall(kI2SPortSlave);
        i2s_driver_uninstall(kI2SPortMaster);
    }
    _freeBuffers();
}

bool I2SAudio::_allocBuffers()
{
    const size_t intBufBytes   = kMaxBlockFrames * kChannels * sizeof(int32_t);
    const size_t floatBufBytes = kMaxBlockFrames * sizeof(float);

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

    return (_rxBufAux && _rxBufBt && _txBuf &&
            _leftAux && _rightAux && _leftBt && _rightBt &&
            _leftWifi && _rightWifi && _leftMix && _rightMix);
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
    if (frames > kMaxBlockFrames) frames = kMaxBlockFrames;

    float bufL[kMaxBlockFrames];
    float bufR[kMaxBlockFrames];
    dsp_unpack_i32_stereo_to_f32(_rxBufBt, bufL, bufR, frames, kScale, kScale);
    // BT stays at 44.1 kHz natively; copy directly into the BT ring buffer.
    size_t written = _btRingBuffer.write(bufL, bufR, frames);
    if (_btResampler) {
        uint32_t overrun = (frames > written) ? static_cast<uint32_t>(frames - written) : 0;
        resampler_record_flow(_btResampler, 0, overrun, 0,
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
