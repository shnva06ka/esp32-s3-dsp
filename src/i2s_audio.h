/**
 * @file i2s_audio.h
 * @brief Dual-I2S Audio Driver & Input Manager with Auto-Source Switching.
 *
 * Interfaces:
 * - I2S_NUM_0: Full-Duplex Master (48kHz, 32-bit slot)
 *     * PCM1808 ADC capture path (MCLK=4, BCK=5, WS=6, DIN=15)
 *     * PCM5102A DAC playback path (DOUT=7)
 * - I2S_NUM_1: Slave Receiver (48kHz, 32-bit slot)
 *     * External ESP32-WROOM Bluetooth A2DP digital audio (BCK=16, WS=17, DIN=18)
 *
 * Input Selector Modes:
 * - AUTO: Monitors Bluetooth RMS. If > -55 dBFS, crossfades smoothly from AUX
 *         to Bluetooth. Returns to AUX after 3 seconds of Bluetooth silence.
 * - AUX_ONLY: Forces PCM1808 ADC input.
 * - BT_ONLY: Forces external Bluetooth input.
 * - WIFI_STREAM: Pulls from internal circular AudioRingBuffer (stub/placeholder).
 *
 * Safe No-Hardware Operation:
 * - In slave mode, reads use a 0-tick timeout so if no external clock is present,
 *   the audio loop never blocks or starves the master DAC playback path.
 */
#pragma once

#include <functional>
#include <cstddef>
#include <driver/i2s.h>
#include "audio_config.h"
#include "param_store.h"
#include "audio_ring_buffer.h"

// Callback type: (left[], right[], frames)
using AudioCallback = std::function<void(float*, float*, size_t)>;

class I2SAudio {
public:
    explicit I2SAudio(ParamStore& params);
    ~I2SAudio() { end(); }

    /**
     * @brief Initialise both I2S peripherals (I2S_NUM_0 Master, I2S_NUM_1 Slave).
     * @return true on success
     */
    bool begin();

    /**
     * @brief Read inputs, perform source selection / crossfade, run DSP callback, write to DAC.
     * Call from the audio task loop on Core 1.
     */
    void process(const AudioCallback& cb);

    /** @brief Tear down I2S peripherals and free buffers. */
    void end();

    // ── Level meters (updated every process() call) ──────────
    float rmsInL()  const { return _rmsAuxL; } // Alias for backward compatibility
    float rmsInR()  const { return _rmsAuxR; }
    float rmsAuxL() const { return _rmsAuxL; }
    float rmsAuxR() const { return _rmsAuxR; }
    float rmsBtL()  const { return _rmsBtL;  }
    float rmsBtR()  const { return _rmsBtR;  }
    float rmsOutL() const { return _rmsOutL; }
    float rmsOutR() const { return _rmsOutR; }

    // ── Wi-Fi Ring Buffer Access (stub for future Wi-Fi stream tasks) ──
    AudioRingBuffer<2048>& getWiFiRingBuffer() { return _wifiRingBuffer; }

private:
    ParamStore& _params;
    bool        _running{false};

    // Working DMA buffers (int32 interleaved stereo)
    int32_t* _rxBufAux{nullptr}; // I2S_NUM_0 PCM1808 capture
    int32_t* _rxBufBt{nullptr};  // I2S_NUM_1 Bluetooth capture
    int32_t* _txBuf{nullptr};    // I2S_NUM_0 PCM5102A playback

    // Float working buffers [-1.0, +1.0]
    float* _leftAux{nullptr};
    float* _rightAux{nullptr};
    float* _leftBt{nullptr};
    float* _rightBt{nullptr};
    float* _leftWifi{nullptr};
    float* _rightWifi{nullptr};
    float* _leftMix{nullptr};
    float* _rightMix{nullptr};

    // Wi-Fi Stream ring buffer placeholder
    AudioRingBuffer<2048> _wifiRingBuffer;

    // Crossfade gain states (0.0 to 1.0)
    float _gainAux{1.0f};
    float _gainBt{0.0f};
    float _gainWifi{0.0f};

    ActiveInputSource _lastActiveSource{ACTIVE_SRC_AUX};

    // Level meters
    float _rmsAuxL{0}, _rmsAuxR{0};
    float _rmsBtL{0},  _rmsBtR{0};
    float _rmsOutL{0}, _rmsOutR{0};

    // Constants
    static constexpr float kScale = 1.0f / 2147483648.0f;       // Full-scale 32-bit slot
    static constexpr float kFadeStep = 1.0f / 2400.0f;          // ~50ms smooth crossfade (2400 samples @ 48kHz)

    void  _allocBuffers();
    void  _freeBuffers();
    float _calcRMS(const float* buf, size_t n) const;
};
