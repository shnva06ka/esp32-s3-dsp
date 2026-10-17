/**
 * @file audio_config.h
 * @brief Compile-time audio constants shared across all modules.
 *
 * GPIO assignments match the YD-ESP32-S3 board wiring:
 *   PCM1808 ADC (capture)  -  GPIO 15 (DIN  -> ESP32 receives)
 *   PCM5102A DAC (playback)  -  GPIO 7 (DOUT -> ESP32 transmits)
 */
#pragma once

#include <cstdint>
#include <driver/i2s.h>

// -- Sample / buffer constants --------------------------------
static constexpr uint32_t kSampleRate    = SAMPLE_RATE;     // 48 000 Hz
static constexpr size_t   kBufferFrames  = BUFFER_FRAMES;   // 256 frames
static constexpr uint8_t  kChannels      = 2;               // stereo

// -- I2S peripherals -------------------------------------------
static constexpr i2s_port_t kI2SPortMaster = I2S_NUM_0; // PCM1808 ADC + PCM5102A DAC (Master)
static constexpr i2s_port_t kI2SPortSlave  = I2S_NUM_1; // External ESP32-WROOM BT (Slave)
static constexpr i2s_port_t kI2SPort       = kI2SPortMaster; // Alias for backward compatibility

// -- GPIO pin map ---------------------------------------------
// I2S_NUM_0: PCM1808 ADC (capture) + PCM5102A DAC (playback)
static constexpr int kPinMCLK  = I2S_MCLK_PIN;  // 4
static constexpr int kPinBCK   = I2S_BCK_PIN;   // 5
static constexpr int kPinWS    = I2S_WS_PIN;    // 6
static constexpr int kPinDOUT  = I2S_DOUT_PIN;  // 7  (ESP -> DAC)
static constexpr int kPinDIN   = I2S_DIN_PIN;   // 15 (ADC -> ESP)

// I2S_NUM_1: Bluetooth digital audio slave receiver (from ESP32-WROOM)
#ifndef I2S_BT_BCK_PIN
#  define I2S_BT_BCK_PIN 16
#endif
#ifndef I2S_BT_WS_PIN
#  define I2S_BT_WS_PIN 17
#endif
#ifndef I2S_BT_DIN_PIN
#  define I2S_BT_DIN_PIN 18
#endif

static constexpr int kPinBtBCK = I2S_BT_BCK_PIN; // 16
static constexpr int kPinBtWS  = I2S_BT_WS_PIN;  // 17
static constexpr int kPinBtDIN = I2S_BT_DIN_PIN; // 18

// -- DSP sizing -----------------------------------------------
static constexpr int kEQBands = 10;

// -- Level constants ------------------------------------------
static constexpr float kFullScale = 1.0f;        // normalised peak

