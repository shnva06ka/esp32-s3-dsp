#pragma once

#include <cstdint>
#include <driver/i2s.h>

static constexpr uint32_t kSampleRate    = SAMPLE_RATE;
static constexpr size_t   kBufferFrames  = BUFFER_FRAMES;
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

static constexpr int kEQBands = 10;
