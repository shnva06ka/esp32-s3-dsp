#pragma once

#include <cstddef>
#include <cstdint>

namespace RuntimeTuning {
static constexpr size_t kBtRingBufferFrames  = 16384;
static constexpr size_t kBtLatencyCapFrames  = 8192;
static constexpr size_t kBtLatencyTrimFrames = 256;
}
