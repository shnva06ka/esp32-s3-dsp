/**
 * @file audio_ring_buffer.h
 * @brief Thread-safe lock-free circular ring buffer for audio streaming stubs.
 */
#pragma once

#include <cstddef>
#include <cstring>
#include <atomic>
#include <new>

template <size_t CapacityFrames>
class AudioRingBuffer {
public:
    AudioRingBuffer() : _readIdx(0), _writeIdx(0) {
        _left  = new (std::nothrow) float[CapacityFrames]();
        _right = new (std::nothrow) float[CapacityFrames]();
    }

    ~AudioRingBuffer() {
        delete[] _left;
        delete[] _right;
        _left = nullptr;
        _right = nullptr;
    }

    /** @brief Number of stereo frames available to read. */
    size_t available() const {
        size_t w = _writeIdx.load(std::memory_order_acquire);
        size_t r = _readIdx.load(std::memory_order_relaxed);
        if (w >= r) return w - r;
        return CapacityFrames - (r - w);
    }

    /** @brief Number of stereo frames space available to write. */
    size_t space() const {
        return (CapacityFrames - 1) - available();
    }

    /**
     * @brief Write stereo float samples into the ring buffer.
     * @return Number of frames actually written.
     */
    size_t write(const float* left, const float* right, size_t frames) {
        if (!_left || !_right || !left || !right) return 0;

        size_t toWrite = frames;
        size_t sp = space();
        if (toWrite > sp) toWrite = sp;
        if (toWrite == 0) return 0;

        size_t w = _writeIdx.load(std::memory_order_relaxed);
        for (size_t i = 0; i < toWrite; ++i) {
            size_t idx = (w + i) % CapacityFrames;
            _left[idx]  = left[i];
            _right[idx] = right[i];
        }
        _writeIdx.store((w + toWrite) % CapacityFrames, std::memory_order_release);
        return toWrite;
    }

    /**
     * @brief Read stereo float samples from the ring buffer.
     * If fewer frames are available than requested, remaining frames are zero-padded.
     * @return Number of valid frames read before zero padding.
     */
    size_t read(float* left, float* right, size_t frames) {
        if (!left || !right) return 0;

        if (!_left || !_right) {
            for (size_t i = 0; i < frames; ++i) {
                left[i]  = 0.0f;
                right[i] = 0.0f;
            }
            return 0;
        }

        size_t avail = available();
        size_t toRead = frames;
        if (toRead > avail) toRead = avail;

        size_t r = _readIdx.load(std::memory_order_relaxed);
        for (size_t i = 0; i < toRead; ++i) {
            size_t idx = (r + i) % CapacityFrames;
            left[i]  = _left[idx];
            right[i] = _right[idx];
        }
        // Fill remaining with silence
        for (size_t i = toRead; i < frames; ++i) {
            left[i]  = 0.0f;
            right[i] = 0.0f;
        }

        if (toRead > 0) {
            _readIdx.store((r + toRead) % CapacityFrames, std::memory_order_release);
        }
        return toRead;
    }

    /** @brief Reset the buffer to empty. */
    void clear() {
        _readIdx.store(0, std::memory_order_relaxed);
        _writeIdx.store(0, std::memory_order_relaxed);
        if (_left) memset(_left, 0, CapacityFrames * sizeof(float));
        if (_right) memset(_right, 0, CapacityFrames * sizeof(float));
    }

private:
    float* _left{nullptr};
    float* _right{nullptr};
    std::atomic<size_t> _readIdx{0};
    std::atomic<size_t> _writeIdx{0};
};
