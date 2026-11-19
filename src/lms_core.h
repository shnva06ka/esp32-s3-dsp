#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace lms {

inline void put16(uint8_t* p, uint16_t v) { p[0] = (uint8_t)(v >> 8); p[1] = (uint8_t)v; }
inline void put32(uint8_t* p, uint32_t v) {
    p[0] = (uint8_t)(v >> 24); p[1] = (uint8_t)(v >> 16); p[2] = (uint8_t)(v >> 8); p[3] = (uint8_t)v;
}
inline void put64(uint8_t* p, uint64_t v) { put32(p, (uint32_t)(v >> 32)); put32(p + 4, (uint32_t)v); }
inline uint16_t get16(const uint8_t* p) { return (uint16_t)((p[0] << 8) | p[1]); }
inline uint32_t get32(const uint8_t* p) {
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | (uint32_t)p[3];
}

constexpr size_t kHeloFixedLen   = 36;
constexpr size_t kStatPayloadLen = 53;
constexpr size_t kStatTotalLen   = 8 + kStatPayloadLen;
constexpr size_t kStrmDataLen    = 24;
constexpr size_t kAudgMinLen     = 18;
constexpr size_t kMaxServerFrame = 2 + 65535;

inline size_t buildHelo(uint8_t* out, size_t cap, const uint8_t mac[6], uint8_t deviceId,
                        uint8_t revision, uint64_t bytesReceived, const char* caps) {
    const size_t capsLen = caps ? std::strlen(caps) : 0;
    const size_t payload = kHeloFixedLen + capsLen;
    const size_t total   = 8 + payload;
    if (!out || !mac || cap < total) return 0;
    std::memcpy(out, "HELO", 4);
    put32(out + 4, (uint32_t)payload);
    uint8_t* p = out + 8;
    p[0] = deviceId;
    p[1] = revision;
    std::memcpy(p + 2, mac, 6);
    std::memset(p + 8, 0, 16);
    put16(p + 24, 0x07FF);
    put64(p + 26, bytesReceived);
    p[34] = 'e';
    p[35] = 'n';
    if (capsLen) std::memcpy(p + 36, caps, capsLen);
    return total;
}

struct StatInfo {
    uint32_t streamBufSize{0};
    uint32_t streamBufFullness{0};
    uint64_t bytesReceived{0};
    uint16_t signal{0};
    uint32_t jiffies{0};
    uint32_t outBufSize{0};
    uint32_t outBufFullness{0};
    uint32_t elapsedSeconds{0};
    uint32_t elapsedMs{0};
    uint32_t serverTimestamp{0};
    uint16_t errorCode{0};
};

inline size_t buildStat(uint8_t* out, size_t cap, const char event[4], const StatInfo& s) {
    if (!out || !event || cap < kStatTotalLen) return 0;
    std::memcpy(out, "STAT", 4);
    put32(out + 4, (uint32_t)kStatPayloadLen);
    uint8_t* p = out + 8;
    std::memcpy(p, event, 4);
    p[4] = 0;
    p[5] = 'm';
    p[6] = 0;
    put32(p + 7,  s.streamBufSize);
    put32(p + 11, s.streamBufFullness);
    put64(p + 15, s.bytesReceived);
    put16(p + 23, s.signal);
    put32(p + 25, s.jiffies);
    put32(p + 29, s.outBufSize);
    put32(p + 33, s.outBufFullness);
    put32(p + 37, s.elapsedSeconds);
    put16(p + 41, 0);
    put32(p + 43, s.elapsedMs);
    put32(p + 47, s.serverTimestamp);
    put16(p + 51, s.errorCode);
    return kStatTotalLen;
}

enum class FrameResult { NeedMore, Ok, Invalid };

struct Frame {
    char           op[4];
    const uint8_t* data;
    size_t         dataLen;
};

inline FrameResult parseFrame(const uint8_t* buf, size_t n, Frame& f, size_t& consumed) {
    consumed = 0;
    if (!buf || n < 2) return FrameResult::NeedMore;
    const size_t len = get16(buf);
    if (len < 4) return FrameResult::Invalid;
    if (n < 2 + len) return FrameResult::NeedMore;
    std::memcpy(f.op, buf + 2, 4);
    f.data    = buf + 6;
    f.dataLen = len - 4;
    consumed  = 2 + len;
    return FrameResult::Ok;
}

inline bool opIs(const Frame& f, const char* op4) { return std::memcmp(f.op, op4, 4) == 0; }

struct Strm {
    char     command, autostart, format, pcmSize, pcmRate, pcmChannels, pcmEndian;
    uint8_t  threshold;
    uint8_t  flags, outputThreshold;
    uint32_t replayGain;
    uint16_t port;
    uint32_t ip;
    const uint8_t* http;
    size_t   httpLen;
};

constexpr uint8_t kFlagGapless  = 0x40;
constexpr uint8_t kFlagInvertL  = 0x01;
constexpr uint8_t kFlagInvertR  = 0x02;

inline bool parseStrm(const uint8_t* d, size_t n, Strm& s) {
    if (!d || n < kStrmDataLen) return false;
    s.command = (char)d[0];     s.autostart = (char)d[1];   s.format = (char)d[2];
    s.pcmSize = (char)d[3];     s.pcmRate = (char)d[4];     s.pcmChannels = (char)d[5];
    s.pcmEndian = (char)d[6];   s.threshold = d[7];
    s.flags = d[11];
    s.outputThreshold = d[12];
    s.replayGain = get32(d + 14);
    s.port = get16(d + 18);
    s.ip = get32(d + 20);
    s.http = d + kStrmDataLen;
    s.httpLen = n - kStrmDataLen;
    return true;
}

struct Audg {
    uint32_t left;
    uint32_t right;
};

inline bool parseAudg(const uint8_t* d, size_t n, Audg& a) {
    if (!d || n < kAudgMinLen) return false;
    a.left  = get32(d + 10);
    a.right = get32(d + 14);
    return true;
}

inline uint32_t pcmRateFromCode(char c) {
    switch (c) {
        case '0': return 11025;  case '1': return 22050;  case '2': return 32000;
        case '3': return 44100;  case '4': return 48000;  case '5': return 8000;
        case '6': return 12000;  case '7': return 16000;  case '8': return 24000;
        case '9': return 96000;  default:  return 0;
    }
}

inline unsigned pcmBitsFromCode(char c) {
    switch (c) {
        case '0': return 8;  case '1': return 16;  case '2': return 24;  case '3': return 32;
        default:  return 0;
    }
}

inline float fixed16ToFloat(uint32_t v) { return (float)v * (1.0f / 65536.0f); }

}

namespace lms {

struct Gains {
    float left;
    float right;
};

constexpr float kMaxVolumeGain = 4.0f;

inline float clampGain(float g, float maxGain) {
    if (!(g >= 0.0f)) return 0.0f;
    return g > maxGain ? maxGain : g;
}

inline Gains volumeGains(uint32_t audgLeft, uint32_t audgRight, bool dacEnabled) {
    if (!dacEnabled) return {0.0f, 0.0f};
    return {clampGain(fixed16ToFloat(audgLeft), kMaxVolumeGain),
            clampGain(fixed16ToFloat(audgRight), kMaxVolumeGain)};
}

inline Gains trackGains(uint32_t replayGain16, uint8_t flags) {
    const float rg = (replayGain16 == 0) ? 1.0f : clampGain(fixed16ToFloat(replayGain16), kMaxVolumeGain);
    return {(flags & kFlagInvertL) ? -rg : rg, (flags & kFlagInvertR) ? -rg : rg};
}

}

namespace lms {

struct PcmInfo {
    uint32_t rate{0};
    uint8_t  bits{0};
    uint8_t  channels{0};
    bool     bigEndian{false};
    size_t   headerBytes{0};
};

constexpr int kPcmNeedMore = 0;
constexpr int kPcmInvalid  = -1;

inline uint32_t pcmLe32(const uint8_t* p) { return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24); }
inline uint16_t pcmLe16(const uint8_t* p) { return (uint16_t)(p[0] | (p[1] << 8)); }
inline uint32_t pcmBe32(const uint8_t* p) { return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | (uint32_t)p[3]; }
inline uint16_t pcmBe16(const uint8_t* p) { return (uint16_t)((p[0] << 8) | p[1]); }

inline bool pcmFormatSupported(uint8_t bits, uint8_t channels) {
    return (bits == 8 || bits == 16 || bits == 24 || bits == 32) && (channels == 1 || channels == 2);
}

inline int parsePcmHeader(const uint8_t* b, size_t n, PcmInfo& info) {
    constexpr size_t kMaxSkippedChunk = 4096;
    if (n < 12) return kPcmNeedMore;

    if (std::memcmp(b, "RIFF", 4) == 0 && std::memcmp(b + 8, "WAVE", 4) == 0) {
        bool haveFmt = false;
        size_t pos = 12;
        for (;;) {
            if (n < pos + 8) return kPcmNeedMore;
            const uint32_t size = pcmLe32(b + pos + 4);
            if (std::memcmp(b + pos, "data", 4) == 0) {
                if (!haveFmt) return kPcmInvalid;
                info.headerBytes = pos + 8;
                return (int)info.headerBytes;
            }
            const size_t padded = (size_t)size + (size & 1u);
            if (padded > kMaxSkippedChunk) return kPcmInvalid;
            if (n < pos + 8 + padded) return kPcmNeedMore;
            if (std::memcmp(b + pos, "fmt ", 4) == 0) {
                if (size < 16) return kPcmInvalid;
                const uint16_t fmt = pcmLe16(b + pos + 8);
                if (fmt != 1 && fmt != 0xFFFE) return kPcmInvalid;
                info.channels  = (uint8_t)pcmLe16(b + pos + 10);
                info.rate      = pcmLe32(b + pos + 12);
                info.bits      = (uint8_t)pcmLe16(b + pos + 22);
                info.bigEndian = false;
                if (!pcmFormatSupported(info.bits, info.channels) || info.rate == 0) return kPcmInvalid;
                haveFmt = true;
            }
            pos += 8 + padded;
        }
    }

    if (std::memcmp(b, "FORM", 4) == 0 && std::memcmp(b + 8, "AIFF", 4) == 0) {
        bool haveComm = false;
        size_t pos = 12;
        for (;;) {
            if (n < pos + 8) return kPcmNeedMore;
            const uint32_t size = pcmBe32(b + pos + 4);
            if (std::memcmp(b + pos, "SSND", 4) == 0) {
                if (!haveComm) return kPcmInvalid;
                if (n < pos + 16) return kPcmNeedMore;
                const uint32_t offset = pcmBe32(b + pos + 8);
                if (offset > kMaxSkippedChunk) return kPcmInvalid;
                if (n < pos + 16 + offset) return kPcmNeedMore;
                info.headerBytes = pos + 16 + offset;
                return (int)info.headerBytes;
            }
            const size_t padded = (size_t)size + (size & 1u);
            if (padded > kMaxSkippedChunk) return kPcmInvalid;
            if (n < pos + 8 + padded) return kPcmNeedMore;
            if (std::memcmp(b + pos, "COMM", 4) == 0) {
                if (size < 18) return kPcmInvalid;
                info.channels = (uint8_t)pcmBe16(b + pos + 8);
                info.bits     = (uint8_t)pcmBe16(b + pos + 14);
                const uint16_t exp = (uint16_t)(pcmBe16(b + pos + 16) & 0x7FFF);
                const uint32_t manHi = pcmBe32(b + pos + 18);
                const int shift = 16383 + 31 - (int)exp;
                if (shift < 0 || shift > 31) return kPcmInvalid;
                info.rate = manHi >> shift;
                info.bigEndian = true;
                if (!pcmFormatSupported(info.bits, info.channels) || info.rate == 0) return kPcmInvalid;
                haveComm = true;
            }
            pos += 8 + padded;
        }
    }
    return kPcmInvalid;
}

inline size_t pcmFrameBytes(const PcmInfo& i) { return (size_t)(i.bits / 8) * i.channels; }

inline int32_t pcmReadSample(const uint8_t* p, unsigned bytes, bool big, bool unsigned8) {
    uint32_t u = 0;
    switch (bytes) {
        case 1: {
            const int v = unsigned8 ? ((int)p[0] - 128) : (int)(int8_t)p[0];
            return (int32_t)((uint32_t)v << 24);
        }
        case 2:
            u = big ? (uint32_t)((p[0] << 8) | p[1]) : (uint32_t)((p[1] << 8) | p[0]);
            return (int32_t)(u << 16);
        case 3:
            u = big ? (((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8))
                    : (((uint32_t)p[2] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[0] << 8));
            return (int32_t)u;
        default:
            u = big ? pcmBe32(p) : pcmLe32(p);
            return (int32_t)u;
    }
}

inline void pcmToFloat(const PcmInfo& i, const uint8_t* src, size_t frames, float* L, float* R) {
    const unsigned bytes = i.bits / 8u;
    const bool u8 = (bytes == 1) && !i.bigEndian;
    constexpr float kScale = 1.0f / 2147483648.0f;
    for (size_t f = 0; f < frames; ++f) {
        const float l = (float)pcmReadSample(src, bytes, i.bigEndian, u8) * kScale;
        src += bytes;
        float r = l;
        if (i.channels == 2) {
            r = (float)pcmReadSample(src, bytes, i.bigEndian, u8) * kScale;
            src += bytes;
        }
        L[f] = l;
        R[f] = r;
    }
}

}

namespace lms {

class ByteRing {
public:
    bool init(uint8_t* storage, size_t capacity) {
        if (!storage || capacity < 2 || (capacity & (capacity - 1)) != 0) return false;
        _buf = storage;
        _cap = capacity;
        _mask = capacity - 1;
        reset();
        return true;
    }

    void reset() {
        _w.store(0, std::memory_order_relaxed);
        _r.store(0, std::memory_order_relaxed);
    }

    size_t capacity() const { return _cap; }
    size_t available() const {
        return (size_t)(uint32_t)(_w.load(std::memory_order_acquire) - _r.load(std::memory_order_acquire));
    }
    size_t space() const { return _cap - available(); }

    size_t write(const uint8_t* src, size_t n) {
        const size_t sp = space();
        if (n > sp) n = sp;
        const uint32_t w = _w.load(std::memory_order_relaxed);
        for (size_t done = 0; done < n;) {
            const size_t idx = ((size_t)w + done) & _mask;
            size_t chunk = _cap - idx;
            if (chunk > n - done) chunk = n - done;
            std::memcpy(_buf + idx, src + done, chunk);
            done += chunk;
        }
        _w.store(w + (uint32_t)n, std::memory_order_release);
        return n;
    }

    size_t read(uint8_t* dst, size_t n) {
        const size_t av = available();
        if (n > av) n = av;
        const uint32_t r = _r.load(std::memory_order_relaxed);
        for (size_t done = 0; done < n;) {
            const size_t idx = ((size_t)r + done) & _mask;
            size_t chunk = _cap - idx;
            if (chunk > n - done) chunk = n - done;
            std::memcpy(dst + done, _buf + idx, chunk);
            done += chunk;
        }
        _r.store(r + (uint32_t)n, std::memory_order_release);
        return n;
    }

private:
    uint8_t*              _buf{nullptr};
    size_t                _cap{0};
    size_t                _mask{0};
    std::atomic<uint32_t> _w{0};
    std::atomic<uint32_t> _r{0};
};

}

namespace lms {

enum Event : uint32_t {
    EV_c = 1u << 0,
    EV_e = 1u << 1,
    EV_h = 1u << 2,
    EV_l = 1u << 3,
    EV_s = 1u << 4,
    EV_d = 1u << 5,
    EV_o = 1u << 6,
    EV_u = 1u << 7,
    EV_p = 1u << 8,
    EV_r = 1u << 9,
    EV_f = 1u << 10,
    EV_n = 1u << 11,
    EV_t = 1u << 12,
};

inline const char* eventCode(uint32_t singleEvent) {
    switch (singleEvent) {
        case EV_c: return "STMc"; case EV_e: return "STMe"; case EV_h: return "STMh";
        case EV_l: return "STMl"; case EV_s: return "STMs"; case EV_d: return "STMd";
        case EV_o: return "STMo"; case EV_u: return "STMu"; case EV_p: return "STMp";
        case EV_r: return "STMr"; case EV_f: return "STMf"; case EV_n: return "STMn";
        default:   return "STMt";
    }
}

enum class Phase : uint8_t { Off, Buffering, Playing, Paused, Stopped };

struct Levels {
    uint32_t streamBytes{0};
    bool     streamEof{false};
    uint32_t outFrames{0};
    bool     decoderDone{false};
    bool     underrun{false};
    uint32_t nowMs{0};
};

class Transport {
public:
    static constexpr uint32_t kOutputRate = 96000;

    Phase phase() const { return _phase; }
    bool  isPlaying() const { return _phase == Phase::Playing; }
    bool  isPaused() const { return _phase == Phase::Paused; }

    uint32_t onStart(char autostart, uint8_t thresholdKb, uint8_t outputThresholdTenths, bool keepPlayback, uint32_t outputRate = kOutputRate) {
        const bool wasPlaying = (_phase == Phase::Playing);
        _thresholdBytes  = (uint32_t)thresholdKb * 1024u;
        const uint32_t rate = outputRate ? outputRate : kOutputRate;
        _outThrFrames    = (uint32_t)outputThresholdTenths * (rate / 10u);
        _sendThreshold   = (autostart == '0' || autostart == '2');
        _gateOpen        = (autostart == '1' || autostart == '3');
        _thresholdSent   = false;
        _doneSent        = false;
        _rebuffering     = false;
        _resumeAtMs      = 0;
        if (keepPlayback && wasPlaying) {
            _trackPending = true;
        } else {
            _trackPending = false;
            _phase = Phase::Buffering;
        }
        return EV_c;
    }

    uint32_t onStreamConnected() { return EV_e; }
    uint32_t onHeaders()         { return EV_h; }
    uint32_t onStreamError()     { reset(); _phase = Phase::Stopped; return EV_n; }

    uint32_t onTrackBoundary() {
        if (!_trackPending) return 0;
        _trackPending = false;
        return EV_s;
    }

    uint32_t onPause(uint32_t intervalMs, uint32_t nowMs) {
        if (_phase != Phase::Playing && _phase != Phase::Buffering) return 0;
        _beforePause = _phase;
        _phase = Phase::Paused;
        if (intervalMs != 0) {
            _resumeAtMs = nowMs + intervalMs;
            if (_resumeAtMs == 0) _resumeAtMs = 1;
            return 0;
        }
        _resumeAtMs = 0;
        return EV_p;
    }

    uint32_t onUnpause() {
        if (_phase == Phase::Paused) {
            _phase = _beforePause;
            _resumeAtMs = 0;
            return EV_r;
        }
        _gateOpen = true;
        return 0;
    }

    uint32_t onStop()  { reset(); return EV_f; }
    uint32_t onFlush() { reset(); return EV_f; }
    uint32_t onStatusRequest() { return EV_t; }

    uint32_t tick(const Levels& lv) {
        uint32_t ev = 0;

        if (_phase == Phase::Paused) {
            if (_resumeAtMs != 0 && (int32_t)(lv.nowMs - _resumeAtMs) >= 0) {
                _phase = _beforePause;
                _resumeAtMs = 0;
            }
            return ev;
        }
        if (_phase == Phase::Off || _phase == Phase::Stopped) return ev;

        if (_sendThreshold && !_thresholdSent &&
            (lv.streamBytes >= _thresholdBytes || lv.streamEof)) {
            _thresholdSent = true;
            ev |= EV_l;
        }

        if (_phase == Phase::Buffering && _gateOpen) {
            if (lv.outFrames >= _outThrFrames && lv.outFrames > 0) {
                _phase = Phase::Playing;
                if (!_rebuffering) ev |= EV_s;
                _rebuffering = false;
            } else if (lv.decoderDone) {
                if (lv.outFrames > 0) {
                    _phase = Phase::Playing;
                    if (!_rebuffering) ev |= EV_s;
                    _rebuffering = false;
                } else {
                    _phase = Phase::Stopped;
                    ev |= EV_u;
                }
            }
        }

        if (lv.decoderDone && !_doneSent) {
            _doneSent = true;
            ev |= EV_d;
        }

        if (_phase == Phase::Playing) {
            if (lv.underrun && !lv.decoderDone) {
                _phase = Phase::Buffering;
                _rebuffering = true;
                ev |= EV_o;
            } else if (lv.decoderDone && lv.outFrames == 0 && !_trackPending) {
                _phase = Phase::Stopped;
                ev |= EV_u;
            }
        }
        return ev;
    }

    void reset() {
        _phase = Phase::Off;
        _thresholdSent = _doneSent = _rebuffering = _gateOpen = _trackPending = false;
        _resumeAtMs = 0;
    }

private:
    Phase    _phase{Phase::Off};
    Phase    _beforePause{Phase::Playing};
    uint32_t _thresholdBytes{0};
    uint32_t _outThrFrames{0};
    uint32_t _resumeAtMs{0};
    bool     _sendThreshold{false};
    bool     _thresholdSent{false};
    bool     _gateOpen{false};
    bool     _doneSent{false};
    bool     _rebuffering{false};
    bool     _trackPending{false};
};

}
