#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <atomic>
#include <esp_heap_caps.h>
#include <Arduino.h>

template <size_t CapacityFrames>
class AudioRingBuffer {
public:
    AudioRingBuffer() = default;

    ~AudioRingBuffer() {
        if (_left)  heap_caps_free(_left);
        if (_right) heap_caps_free(_right);
        _left = nullptr;
        _right = nullptr;
    }

    bool allocate() {
        if (_left && _right) return true;
#if defined(BOARD_HAS_PSRAM)
        if (psramFound()) {
            if (!_left)  _left  = static_cast<float*>(heap_caps_malloc(CapacityFrames * sizeof(float), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
            if (!_right) _right = static_cast<float*>(heap_caps_malloc(CapacityFrames * sizeof(float), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
        }
#endif
        if (!_left)  _left  = static_cast<float*>(malloc(CapacityFrames * sizeof(float)));
        if (!_right) _right = static_cast<float*>(malloc(CapacityFrames * sizeof(float)));
        if (_left)  memset(_left,  0, CapacityFrames * sizeof(float));
        if (_right) memset(_right, 0, CapacityFrames * sizeof(float));
        clear();
        return (_left != nullptr && _right != nullptr);
    }

    bool isAllocated() const { return (_left != nullptr && _right != nullptr); }

    size_t capacity() const { return CapacityFrames; }

    size_t available() const {
        size_t w = _writeIdx.load(std::memory_order_acquire);
        size_t r = _readIdx.load(std::memory_order_relaxed);
        if (w >= r) return w - r;
        return CapacityFrames - (r - w);
    }

    size_t space() const {
        return (CapacityFrames - 1) - available();
    }

    size_t write(const float* left, const float* right, size_t frames) {
        if (!_left || !_right || !left || !right) {
            if (!allocate()) return 0;
        }

        size_t toWrite = frames;
        size_t sp = space();
        if (toWrite > sp) toWrite = sp;
        if (toWrite == 0) return 0;

        size_t w = _writeIdx.load(std::memory_order_relaxed);
        size_t firstChunk = CapacityFrames - w;
        if (firstChunk > toWrite) firstChunk = toWrite;
        size_t secondChunk = toWrite - firstChunk;

        memcpy(_left  + w, left,  firstChunk * sizeof(float));
        memcpy(_right + w, right, firstChunk * sizeof(float));
        if (secondChunk > 0) {
            memcpy(_left,  left  + firstChunk, secondChunk * sizeof(float));
            memcpy(_right, right + firstChunk, secondChunk * sizeof(float));
        }

        _writeIdx.store((w + toWrite) % CapacityFrames, std::memory_order_release);
        return toWrite;
    }

    size_t read(float* left, float* right, size_t frames) {
        if (!left || !right) return 0;

        if (!_left || !_right) {
            memset(left,  0, frames * sizeof(float));
            memset(right, 0, frames * sizeof(float));
            return 0;
        }

        size_t avail = available();
        size_t toRead = (frames < avail) ? frames : avail;

        size_t r = _readIdx.load(std::memory_order_relaxed);
        if (toRead > 0) {
            size_t firstChunk = CapacityFrames - r;
            if (firstChunk > toRead) firstChunk = toRead;
            size_t secondChunk = (toRead > firstChunk) ? (toRead - firstChunk) : 0;

            memcpy(left,  _left  + r, firstChunk * sizeof(float));
            memcpy(right, _right + r, firstChunk * sizeof(float));
            if (secondChunk > 0) {
                memcpy(left  + firstChunk, _left,  secondChunk * sizeof(float));
                memcpy(right + firstChunk, _right, secondChunk * sizeof(float));
            }
            _readIdx.store((r + toRead) % CapacityFrames, std::memory_order_release);
        }

        if (toRead < frames) {
            size_t pad = frames - toRead;
            memset(left  + toRead, 0, pad * sizeof(float));
            memset(right + toRead, 0, pad * sizeof(float));
        }

        return toRead;
    }

    void clear() {
        _readIdx.store(0, std::memory_order_relaxed);
        _writeIdx.store(0, std::memory_order_relaxed);
    }

private:
    float* _left{nullptr};
    float* _right{nullptr};
    std::atomic<size_t> _readIdx{0};
    std::atomic<size_t> _writeIdx{0};
};
