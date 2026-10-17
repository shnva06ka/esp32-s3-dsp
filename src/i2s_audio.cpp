/**
 * @file i2s_audio.cpp
 * @brief Dual-I2S driver and auto-switching input manager implementation.
 */

#include "i2s_audio.h"
#include <Arduino.h>
#include <esp_heap_caps.h>
#include <cmath>
#include <cstring>
#include <algorithm>

I2SAudio::I2SAudio(ParamStore& params)
    : _params(params)
{
}

// -------------------------------------------------------------
bool I2SAudio::begin()
{
    _allocBuffers();

    // -- 1. I2S_NUM_0: Full-Duplex Master (PCM1808 ADC + PCM5102A DAC) --
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

    // -- 2. I2S_NUM_1: Slave Receiver (External ESP32-WROOM Bluetooth A2DP) --
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
        .dma_buf_len          = 64,  // Smaller DMA blocks (1.33ms) so descriptors finish frequently and never starve
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

// -------------------------------------------------------------
void I2SAudio::process(const AudioCallback& cb)
{
    if (!_running) return;

    const size_t byteCount = kBufferFrames * kChannels * sizeof(int32_t);

    // -- 1. Read AUX from I2S_NUM_0 (Master ADC) --------------
    size_t bytesReadAux = 0;
    esp_err_t errAux = i2s_read(kI2SPortMaster, _rxBufAux, byteCount, &bytesReadAux, pdMS_TO_TICKS(20));
    if (errAux != ESP_OK || bytesReadAux == 0) {
        memset(_rxBufAux, 0, byteCount);
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    for (size_t i = 0; i < kBufferFrames; ++i) {
        _leftAux[i]  = static_cast<float>(_rxBufAux[i * 2    ]) * kScale;
        _rightAux[i] = static_cast<float>(_rxBufAux[i * 2 + 1]) * kScale;
    }
    _rmsAuxL = _calcRMS(_leftAux,  kBufferFrames);
    _rmsAuxR = _calcRMS(_rightAux, kBufferFrames);

    // -- 2. Read Bluetooth from I2S_NUM_1 (Slave RX) -----------
    // Non-blocking read (timeout = 0) so lack of external clock never stalls Core 1
    size_t bytesReadBt = 0;
    i2s_read(kI2SPortSlave, _rxBufBt, byteCount, &bytesReadBt, 0);

    // In non-blocking mode, i2s_read can return ESP_ERR_TIMEOUT with partial bytes.
    // Preserve all valid received frames; pad only the trailing unread frames with silence.
    size_t framesReadBt = bytesReadBt / (kChannels * sizeof(int32_t));
    if (framesReadBt > kBufferFrames) framesReadBt = kBufferFrames;

    for (size_t i = framesReadBt; i < kBufferFrames; ++i) {
        _rxBufBt[i * 2    ] = 0;
        _rxBufBt[i * 2 + 1] = 0;
    }

    // Inspect peak integer amplitude to handle 32-bit MSB (expanded) vs 16-bit LSB formats
    int32_t maxAbsBt = 0;
    for (size_t i = 0; i < framesReadBt; ++i) {
        int32_t aL = std::abs(_rxBufBt[i * 2    ]);
        int32_t aR = std::abs(_rxBufBt[i * 2 + 1]);
        if (aL > maxAbsBt) maxAbsBt = aL;
        if (aR > maxAbsBt) maxAbsBt = aR;
    }

    // Adaptive scale factor:
    // If audio is expanded to 32-bit (via i2s_write_expand), peak > 32767.
    // If audio is unshifted 16-bit in lower bits, peak <= 32767.
    const float btScale = (maxAbsBt > 32767) ? (1.0f / 2147483648.0f) : (1.0f / 32768.0f);

    for (size_t i = 0; i < kBufferFrames; ++i) {
        _leftBt[i]  = static_cast<float>(_rxBufBt[i * 2    ]) * btScale;
        _rightBt[i] = static_cast<float>(_rxBufBt[i * 2 + 1]) * btScale;
    }

    // Compute RMS from valid frames (or silence if 0 frames)
    _rmsBtL = (framesReadBt > 0) ? _calcRMS(_leftBt, framesReadBt) : 0.0f;
    _rmsBtR = (framesReadBt > 0) ? _calcRMS(_rightBt, framesReadBt) : 0.0f;
    const float btMaxRms = (_rmsBtL > _rmsBtR) ? _rmsBtL : _rmsBtR;

    // -- 3. Read Wi-Fi Stream stub from Ring Buffer ------------
    _wifiRingBuffer.read(_leftWifi, _rightWifi, kBufferFrames);

    // -- 4. Manual Input Selector Crossfade Logic --------------
    const InputSourceMode targetSource = _params.getInputMode();

    const float targetGainAux  = (targetSource == INPUT_MODE_AUX)  ? 1.0f : 0.0f;
    const float targetGainBt   = (targetSource == INPUT_MODE_BT)   ? 1.0f : 0.0f;
    const float targetGainWifi = (targetSource == INPUT_MODE_WIFI) ? 1.0f : 0.0f;

    // Per-sample smooth linear crossfading
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

    // Determine and broadcast active source changes
    ActiveInputSource currentActive = (targetSource == INPUT_MODE_BT)   ? ACTIVE_SRC_BT :
                                      (targetSource == INPUT_MODE_WIFI) ? ACTIVE_SRC_WIFI : ACTIVE_SRC_AUX;

    if (currentActive != _lastActiveSource) {
        _lastActiveSource = currentActive;
        _params.setActiveSource(currentActive);
        const char* name = (currentActive == ACTIVE_SRC_BT)   ? "BLUETOOTH" :
                           (currentActive == ACTIVE_SRC_AUX)  ? "AUX (PCM1808)" : "WI-FI";
        Serial.printf("[AUDIO] Active input switched to: %s\n", name);
    }

    // -- 5. Run DSP Pipeline Callback --------------------------
    cb(_leftMix, _rightMix, kBufferFrames);

    _rmsOutL = _calcRMS(_leftMix,  kBufferFrames);
    _rmsOutR = _calcRMS(_rightMix, kBufferFrames);

    // -- 6. Interleave float -> int32 and write to DAC ---------
    auto toI32 = [](float s) -> int32_t {
        if (s > 1.0f) s = 1.0f;
        else if (s < -1.0f) s = -1.0f;
        return (s >= 0.0f) ? static_cast<int32_t>(s * 2147483392.0f)
                           : static_cast<int32_t>(s * 2147483648.0f);
    };

    for (size_t i = 0; i < kBufferFrames; ++i) {
        _txBuf[i * 2    ] = toI32(_leftMix[i]);
        _txBuf[i * 2 + 1] = toI32(_rightMix[i]);
    }

    size_t bytesWritten = 0;
    i2s_write(kI2SPortMaster, _txBuf, byteCount, &bytesWritten, pdMS_TO_TICKS(20));
}

// -------------------------------------------------------------
void I2SAudio::end()
{
    if (_running) {
        i2s_driver_uninstall(kI2SPortSlave);
        i2s_driver_uninstall(kI2SPortMaster);
        _running = false;
    }
    _freeBuffers();
}

// -------------------------------------------------------------
void I2SAudio::_allocBuffers()
{
    const size_t intBufBytes   = kBufferFrames * kChannels * sizeof(int32_t); // 2048 bytes
    const size_t floatBufBytes = kBufferFrames * sizeof(float);               // 1024 bytes

    auto dramAlloc = [](size_t sz) -> void* {
        void* p = heap_caps_malloc(sz, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
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
    float sum = 0.0f;
    for (size_t i = 0; i < n; ++i) sum += buf[i] * buf[i];
    return (n > 0) ? sqrtf(sum / static_cast<float>(n)) : 0.0f;
}
