#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>

#if defined(ESP_PLATFORM) || defined(ARDUINO_ARCH_ESP32)
#include <driver/i2s.h>

static constexpr uint32_t kSampleRate    = SAMPLE_RATE;
static constexpr size_t   kBufferFrames  = BUFFER_FRAMES;
/* Bluetooth I2S feed from the WROOM bridge stays at 44.1 kHz and is resampled to kSampleRate on the S3 */
static constexpr uint32_t kBtSampleRate  = 44100;
static constexpr uint8_t  kChannels      = 2;

static constexpr i2s_port_t kI2SPortMaster = I2S_NUM_0;
static constexpr i2s_port_t kI2SPortSlave  = I2S_NUM_1;

static constexpr int kPinMCLK  = I2S_MCLK_PIN;
static constexpr int kPinBCK   = I2S_BCK_PIN;
static constexpr int kPinWS    = I2S_WS_PIN;
static constexpr int kPinDOUT  = I2S_DOUT_PIN;
static constexpr int kPinDIN   = I2S_DIN_PIN;

#ifndef I2S_BT_BCK_PIN
#  define I2S_BT_BCK_PIN 16
#endif
#ifndef I2S_BT_WS_PIN
#  define I2S_BT_WS_PIN 17
#endif
#ifndef I2S_BT_DIN_PIN
#  define I2S_BT_DIN_PIN 18
#endif

static constexpr int kPinBtBCK = I2S_BT_BCK_PIN;
static constexpr int kPinBtWS  = I2S_BT_WS_PIN;
static constexpr int kPinBtDIN = I2S_BT_DIN_PIN;
#endif

static constexpr int kEQBands = 10;

enum EQFilterType : uint8_t {
    EQ_FILTER_PEAK           = 0,
    EQ_FILTER_LOW_SHELF      = 1,
    EQ_FILTER_HIGH_SHELF     = 2,
    EQ_FILTER_LOW_PASS       = 3,
    EQ_FILTER_HIGH_PASS      = 4,
    EQ_FILTER_FLAT_LOW_PASS  = 5,
    EQ_FILTER_FLAT_HIGH_PASS = 6
};

struct EQBandParams {
    float   freq{1000.0f};
    float   gain{0.0f};
    float   q{0.707f};
    bool    enabled{true};
    uint8_t type{EQ_FILTER_PEAK};
    uint8_t order{2};

    EQBandParams() = default;
    EQBandParams(float f, float g, float q_, bool en = true, uint8_t t = EQ_FILTER_PEAK, uint8_t ord = 2)
        : freq(f), gain(g), q(q_), enabled(en), type(t), order(ord) {}
};

struct SubsonicParams {
    float   freq{80.0f};
    uint8_t order{2};
    bool    enabled{false};

    float   balance{0.0f};
    float   delayMsL{0.0f};
    float   delayMsR{0.0f};
    int16_t phaseL{0};
    int16_t phaseR{0};

    SubsonicParams() = default;
    SubsonicParams(float f, uint8_t ord, bool en)
        : freq(f), order(ord), enabled(en) {}
};

struct CompanderParams {
    float threshold{-18.0f};
    float ratio{4.0f};
    float attackMs{5.0f};
    float releaseMs{150.0f};
    float makeupGain{0.0f};
    bool  enabled{false};
};

struct LimiterParams {
    float ceilingDb{-0.5f};
    float kneeDb{3.0f};
    bool  enabled{false};
};

namespace RuntimeTuning {
static constexpr size_t kBtRingBufferFrames  = 4096;
static constexpr size_t kBtLatencyCapFrames  = 2048;
static constexpr size_t kBtLatencyTrimFrames = 256;
}

namespace audio_rate {

enum class Source : uint8_t {
    Aux  = 1,
    Bt   = 2,
    Wifi = 3,
};

constexpr uint32_t kDefaultRate = 44100;
constexpr uint32_t kBtRate      = 44100;
constexpr uint32_t kMaxAdcRate  = 96000;
constexpr uint32_t kMaxDacRate  = 192000;

inline bool isValidDacRate(uint32_t rate) {
    switch (rate) {
        case 44100:
        case 48000:
        case 88200:
        case 96000:
        case 176400:
        case 192000:
            return true;
        default:
            return false;
    }
}

inline bool isValidAdcRate(uint32_t rate) {
    return isValidDacRate(rate) && rate <= kMaxAdcRate;
}

inline uint32_t clampAdcRate(uint32_t rate) {
    if (rate > kMaxAdcRate && isValidDacRate(rate)) return kMaxAdcRate;
    return isValidAdcRate(rate) ? rate : kDefaultRate;
}

inline uint32_t clampDacRate(uint32_t rate) {
    return isValidDacRate(rate) ? rate : kDefaultRate;
}

inline uint32_t activeRate(Source src, uint32_t adcRate, uint32_t maxDacRate, uint32_t wifiInputRate) {
    switch (src) {
        case Source::Aux:
            return clampAdcRate(adcRate);
        case Source::Bt:
            return kBtRate;
        case Source::Wifi: {
            const uint32_t maxDac = clampDacRate(maxDacRate);
            if (wifiInputRate == 0) return maxDac;
            return std::min(wifiInputRate, maxDac);
        }
        default:
            return kDefaultRate;
    }
}

inline size_t blockFramesForRate(uint32_t rate) {
    return (rate >= 176400u) ? 512u : 256u;
}

}  // namespace audio_rate
