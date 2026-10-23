#pragma once

#include <cstddef>
#include <cstdint>

namespace RuntimeTuning {
static constexpr size_t   kWiFiRingBufferFrames     = 16384;
static constexpr size_t   kBtRingBufferFrames       = 8192;
static constexpr size_t   kBtLatencyCapFrames       = 4096;
static constexpr size_t   kBtLatencyTrimFrames      = 256;
static constexpr size_t   kBtResampleChunkFrames    = 512;
static constexpr uint32_t kBtInputSampleRate        = 44100;
static constexpr uint32_t kDspOutputSampleRate      = 48000;

static constexpr uint16_t kSlimprotoControlPort     = 3483;
static constexpr uint16_t kSlimprotoDefaultStreamPort = 9000;
static constexpr uint32_t kSlimprotoDefaultRate     = 44100;

static constexpr size_t   kSlimprotoMaxControlPacket = 4096;
static constexpr size_t   kSlimprotoCarryBytes       = 8;
static constexpr size_t   kSlimprotoReadChunkBytes   = 1024;
static constexpr size_t   kSlimprotoDecodeFrames     = 256;
static constexpr size_t   kSlimprotoResampleFrames   = 384;
static constexpr size_t   kSlimprotoMinWriteFrames   = 512;
static constexpr size_t   kSlimprotoStartedFrames    = 1024;
}
